using System;
using System.Linq;
using Avalonia.Controls;
using Avalonia.Input;
using Avalonia.Interactivity;
using QuantumDynamix.Math.Functions;

namespace QuantumDynamix.App;

internal sealed partial class WorkspaceView
{
    /// <summary>The numeric inspector beside the Transition Editor. Each mode edits what the graph selects.</summary>
    private enum InspectorMode { Keyframes, Transitions, Region }

    private InspectorMode inspector;
    private bool showingTransition;

    private void InitializeTransitionControls()
    {
        TransitionKind.ItemsSource = new[] { "Constant" }.Concat(
            Enum.GetNames<EasingKind>().Where(name => name != nameof(EasingKind.Custom))).ToArray();
        TransitionDirection.ItemsSource = Enum.GetValues<EasingDirection>();
        ToolTip.SetTip(PrependTransitionButton, "Split the selected transition and insert a new span before it.");
        ToolTip.SetTip(AppendTransitionButton, "Split the selected transition and insert a new span after it.");
        ToolTip.SetTip(PlateauFields, "Fractions of the base In function's length and value change.");
        foreach (TextBox input in new[] { TransitionLength, TransitionStart, TransitionEnd, HoldStart, HoldEnd, HoldValue })
        {
            input.TextChanged += (_, _) => RefreshTransitionPending();
            input.KeyDown += ApplyTransitionOnEnter;
        }
        ToolTip.SetTip(TransitionKind, "Choosing a function applies it to the selected Transition. Undo restores the previous one.");
        ToolTip.SetTip(TransitionDirection, "Choosing a direction applies it to the selected Transition.");
        NewRegionMode.ItemsSource = NewRegionModes;
        NewRegionMode.SelectedIndex = 0;
        ToolTip.SetTip(NewRegionMode, "Type for Append Region and Prepend Region. Adjoining uses the type of the Region it joins.");
    }

    private void ShowTransitions(object? sender, RoutedEventArgs e) => SetInspectorMode(InspectorMode.Transitions);
    private void ShowKeyframes(object? sender, RoutedEventArgs e) => SetInspectorMode(InspectorMode.Keyframes);

    private void SetTransitionMode(bool show) => SetInspectorMode(show ? InspectorMode.Transitions : InspectorMode.Keyframes);

    private void SetInspectorMode(InspectorMode mode)
    {
        inspector = mode;
        RefreshEditMode();
        UpdateKeyframeSelection();
        UpdateTransitionSelection();
        RefreshRegionFields();
    }

    private void RefreshEditMode()
    {
        bool region = IsTrack && SelectedRegion >= 0;
        bool profile = region && EditingChannel is not null;
        ProfileEditModes.IsVisible = region;
        KeyframesHeading.IsVisible = !ProfileEditModes.IsVisible;
        RegionFields.IsVisible = region && inspector == InspectorMode.Region;
        TransitionFields.IsVisible = profile && inspector == InspectorMode.Transitions;
        KeyframeFields.IsVisible = !RegionFields.IsVisible && !TransitionFields.IsVisible;
        RegionInspectorButton.IsEnabled = inspector != InspectorMode.Region;
        TransitionsButton.IsEnabled = profile && inspector != InspectorMode.Transitions;
        KeyframesButton.IsEnabled = profile && inspector != InspectorMode.Keyframes;
    }

    private void RefreshTransitions(int selection)
    {
        RefreshEditMode();
        updatingControls = true;
        if (IsTrack && EditingChannel is { } channel)
        {
            var keys = Session!.Keys(SelectedRegion, channel);
            TransitionShape[] shapes = Session.Regions[SelectedRegion].Shapes(channel);
            TransitionSpan.ItemsSource = shapes.Select((shape, index) =>
                $"{keys[index].Coordinate:G6}–{keys[index + 1].Coordinate:G6} m · {shape.Description}").ToArray();
            TransitionSpan.SelectedIndex = System.Math.Clamp(selection, 0, shapes.Length - 1);
        }
        else
        {
            TransitionSpan.ItemsSource = null;
        }
        updatingControls = false;
        UpdateTransitionSelection();
    }

    private void SelectTransition(object? sender, SelectionChangedEventArgs e)
    {
        if (!updatingControls) { UpdateTransitionSelection(); }
    }

    private void SelectTransitionOnGraph(int index)
    {
        inspector = InspectorMode.Transitions;
        RefreshEditMode();
        TransitionSpan.SelectedIndex = index;
        UpdateTransitionSelection();
    }

    /// <summary>Shows the committed Transition. The form, graph label, and span list always name the committed function.</summary>
    private void UpdateTransitionSelection()
    {
        int span = TransitionSpan.SelectedIndex;
        graph.SelectTransition(-1);
        TransitionShapeNote.IsVisible = false;
        ShowTransitionPending(false);
        if (!IsTrack || span < 0 || EditingChannel is not { } channel) { return; }

        var session = Session!;
        var keys = session.Keys(SelectedRegion, channel);
        TransitionShape shape = session.Regions[SelectedRegion].Shapes(channel)[span];
        double start = keys[span].Coordinate;
        double length = keys[span + 1].Coordinate - start;
        if (TransitionFields.IsVisible) { graph.SelectTransition(span, shape.Description); }
        showingTransition = true;
        TransitionKind.SelectedItem = shape.Name;
        TransitionDirection.SelectedItem = shape.Direction;
        TransitionLength.Text = Format(length);
        TransitionLength.IsReadOnly = keys.Count == 2;
        ToolTip.SetTip(TransitionLength, keys.Count == 2 ? "Use Region Properties to resize the only span." : null);
        TransitionStart.Text = Format(keys[span].Value);
        TransitionEnd.Text = Format(keys[span + 1].Value);
        TransitionStart.IsReadOnly = !session.CanEditTransitionEndpoint(SelectedRegion, span, keys.Count);
        TransitionEnd.IsReadOnly = !session.CanEditTransitionEndpoint(SelectedRegion, span + 1, keys.Count);
        ToolTip.SetTip(TransitionStart, TransitionStart.IsReadOnly ? "Linked at the adjoining Region boundary." : null);
        ToolTip.SetTip(TransitionEnd, span + 1 == keys.Count - 1 && SelectedRegion + 1 < session.Regions.Count
            ? "Changing this value updates the adjoining Region's linked inputs." : null);
        HoldStart.Text = Format(shape.HoldStart);
        HoldEnd.Text = Format(shape.HoldEnd);
        HoldValue.Text = Format(shape.HoldValue);
        DeleteTransitionButton.IsEnabled = keys.Count > 2;
        SelectTransitionKind(null, null);
        showingTransition = false;
        // y = y₀ + (y₁ − y₀)E(u) is flat for every function when y₀ = y₁. Say so instead of drawing a silent straight line.
        TransitionShapeNote.IsVisible = shape.Kind is not null && TransitionShape.IsFlat(keys[span], keys[span + 1]);
        RefreshTransitionPending();
        if (TransitionFields.IsVisible) { RefreshCalculatedForce(start + length / 2); }
    }

    private void SelectTransitionKind(object? sender, SelectionChangedEventArgs? e)
    {
        string? kind = TransitionKind.SelectedItem as string;
        TransitionDirection.IsEnabled = kind is not null and not "Constant" and not nameof(EasingKind.Linear);
        PlateauFields.IsVisible = kind == nameof(EasingKind.Plateau);
        if (e is not null && !showingTransition && !updatingControls && TransitionFields.IsVisible)
        {
            // A function choice applies at once, so the curve, 3D track, and history always match the selector.
            ApplyTransition(sender, e);
        }
    }

    private void SelectTransitionDirection(object? sender, SelectionChangedEventArgs e)
    {
        if (!showingTransition && !updatingControls && TransitionFields.IsVisible && TransitionDirection.IsEnabled)
        {
            ApplyTransition(sender, e);
        }
    }

    private void ApplyTransitionOnEnter(object? sender, KeyEventArgs e)
    {
        if (e.Key == Key.Enter)
        {
            ApplyTransition(sender, e);
            e.Handled = true;
        }
    }

    /// <summary>Shows whether typed lengths, values, or Plateau fractions differ from the committed Transition.</summary>
    private void RefreshTransitionPending()
    {
        if (showingTransition || !TransitionFields.IsVisible || TransitionSpan.SelectedIndex < 0 ||
            EditingChannel is not { } channel || Session is not { } session)
        {
            return;
        }
        int span = TransitionSpan.SelectedIndex;
        var keys = session.Keys(SelectedRegion, channel);
        TransitionShape shape = session.Regions[SelectedRegion].Shapes(channel)[span];
        bool plateau = PlateauFields.IsVisible;
        ShowTransitionPending(TransitionLength.Text != Format(keys[span + 1].Coordinate - keys[span].Coordinate) ||
            TransitionStart.Text != Format(keys[span].Value) || TransitionEnd.Text != Format(keys[span + 1].Value) ||
            (plateau && (HoldStart.Text != Format(shape.HoldStart) || HoldEnd.Text != Format(shape.HoldEnd) ||
                HoldValue.Text != Format(shape.HoldValue))));
    }

    internal bool TransitionValuesPending => ApplyTransitionButton.Classes.Contains("pending");

    private void ShowTransitionPending(bool pending)
    {
        if (pending == TransitionValuesPending) { return; }
        ApplyTransitionButton.Classes.Set("pending", pending);
        const string hint = "Typed Transition values are not applied. Press Enter or Apply.";
        ToolTip.SetTip(ApplyTransitionButton, pending ? hint : null);
        if (pending) { StatusLabel.Text = hint; }
    }

    // Restores the selectors after a rejected choice. Typed values stay for correction.
    private void ShowCommittedFunction()
    {
        int span = TransitionSpan.SelectedIndex;
        if (span < 0 || EditingChannel is not { } channel) { return; }
        TransitionShape shape = Session!.Regions[SelectedRegion].Shapes(channel)[span];
        showingTransition = true;
        TransitionKind.SelectedItem = shape.Name;
        TransitionDirection.SelectedItem = shape.Direction;
        SelectTransitionKind(null, null);
        showingTransition = false;
        RefreshTransitionPending();
    }

    private void PrependTransition(object? sender, RoutedEventArgs e) => InsertTransition(insertBefore: true);
    private void AppendTransition(object? sender, RoutedEventArgs e) => InsertTransition(insertBefore: false);

    private void InsertTransition(bool insertBefore)
    {
        if (EditingChannel is not { } channel || TransitionSpan.SelectedIndex < 0) { return; }
        string? error = Session!.SplitTransition(SelectedRegion, channel, TransitionSpan.SelectedIndex, insertBefore, out int selected);
        FinishTransitionEdit(error, selected, "Added a transition.");
    }

    private void DeleteTransition(object? sender, RoutedEventArgs e)
    {
        int span = TransitionSpan.SelectedIndex;
        if (EditingChannel is not { } channel || span < 0) { return; }
        FinishTransitionEdit(Session!.DeleteTransition(SelectedRegion, channel, span), span, "Deleted the transition.");
    }

    private void ApplyTransition(object? sender, RoutedEventArgs e)
    {
        int span = TransitionSpan.SelectedIndex;
        if (EditingChannel is not { } channel || span < 0) { return; }
        bool fromSelector = sender is ComboBox;
        if (!TryReadNumber(TransitionLength.Text, out double length) ||
            !TryReadNumber(TransitionStart.Text, out double from) || !TryReadNumber(TransitionEnd.Text, out double to))
        {
            StatusLabel.Text = "Enter finite values for the transition length and endpoints.";
            if (fromSelector) { ShowCommittedFunction(); }
            return;
        }
        EasingKind? kind = Enum.TryParse(TransitionKind.SelectedItem as string, out EasingKind selected) ? selected : null;
        var direction = TransitionDirection.IsEnabled && TransitionDirection.SelectedItem is EasingDirection choice
            ? choice : EasingDirection.In;
        var shape = new TransitionShape(kind, direction);
        if (kind == EasingKind.Plateau)
        {
            if (!TryReadNumber(HoldStart.Text, out double a) || !TryReadNumber(HoldEnd.Text, out double b) ||
                !TryReadNumber(HoldValue.Text, out double h))
            {
                StatusLabel.Text = "Enter finite fractions for the plateau start, end, and value.";
                if (fromSelector) { ShowCommittedFunction(); }
                return;
            }
            shape = new(kind, direction, a, b, h);
        }
        var before = Session!.Regions;
        string? error = Session.UpdateTransition(SelectedRegion, channel, span, length, from, to, shape);
        if (error is not null)
        {
            // Rejected edits keep the committed Transition, geometry, and history.
            StatusLabel.Text = error;
            if (fromSelector) { ShowCommittedFunction(); }
            return;
        }
        FinishTransitionEdit(null, span, ReferenceEquals(before, Session.Regions)
            ? $"The Transition is already {shape.Description}."
            : $"Applied {shape.Description} to {Format(Session.Keys(SelectedRegion, channel)[span].Coordinate)}–" +
              $"{Format(Session.Keys(SelectedRegion, channel)[span + 1].Coordinate)} m. Undo restores the previous Transition.");
    }

    private void FinishTransitionEdit(string? error, int selection, string success)
    {
        if (error is null)
        {
            RefreshGraph(Session!);
            RefreshKeyframes(-1, selection);
        }
        StatusLabel.Text = error ?? success;
    }
}
