using System;
using System.Linq;
using Avalonia.Controls;
using Avalonia.Interactivity;
using QuantumDynamix.Math.Functions;

namespace QuantumDynamix.App;

internal sealed partial class WorkspaceView
{
    private bool editingTransitions;

    private void InitializeTransitionControls()
    {
        TransitionKind.ItemsSource = new[] { "Constant" }.Concat(
            Enum.GetNames<EasingKind>().Where(name => name != nameof(EasingKind.Custom))).ToArray();
        TransitionDirection.ItemsSource = Enum.GetValues<EasingDirection>();
        ToolTip.SetTip(PrependTransitionButton, "Split the selected transition and insert a new span before it.");
        ToolTip.SetTip(AppendTransitionButton, "Split the selected transition and insert a new span after it.");
        ToolTip.SetTip(PlateauFields, "Fractions of the base In function's length and value change.");
    }

    private void ShowTransitions(object? sender, RoutedEventArgs e) => SetTransitionMode(true);
    private void ShowKeyframes(object? sender, RoutedEventArgs e) => SetTransitionMode(false);

    private void SetTransitionMode(bool show)
    {
        editingTransitions = show;
        RefreshEditMode();
        UpdateKeyframeSelection();
        UpdateTransitionSelection();
    }

    private void RefreshEditMode()
    {
        ProfileEditModes.IsVisible = IsTrack && EditingChannel is not null;
        KeyframesHeading.IsVisible = !ProfileEditModes.IsVisible;
        TransitionFields.IsVisible = ProfileEditModes.IsVisible && editingTransitions;
        KeyframeFields.IsVisible = !TransitionFields.IsVisible;
        TransitionsButton.IsEnabled = !editingTransitions;
        KeyframesButton.IsEnabled = editingTransitions;
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
                $"{keys[index].Coordinate:G6}–{keys[index + 1].Coordinate:G6} m · {shape.Name}").ToArray();
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
        editingTransitions = true;
        RefreshEditMode();
        TransitionSpan.SelectedIndex = index;
        UpdateTransitionSelection();
    }

    private void UpdateTransitionSelection()
    {
        int span = TransitionSpan.SelectedIndex;
        graph.SelectTransition(TransitionFields.IsVisible ? span : -1);
        if (!IsTrack || span < 0 || EditingChannel is not { } channel) { return; }

        var session = Session!;
        var keys = session.Keys(SelectedRegion, channel);
        TransitionShape shape = session.Regions[SelectedRegion].Shapes(channel)[span];
        double start = keys[span].Coordinate;
        double length = keys[span + 1].Coordinate - start;
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
        if (TransitionFields.IsVisible) { RefreshCalculatedForce(start + length / 2); }
    }

    private void SelectTransitionKind(object? sender, SelectionChangedEventArgs? e)
    {
        string? kind = TransitionKind.SelectedItem as string;
        TransitionDirection.IsEnabled = kind is not null and not "Constant" and not nameof(EasingKind.Linear);
        PlateauFields.IsVisible = kind == nameof(EasingKind.Plateau);
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
        if (!TryReadNumber(TransitionLength.Text, out double length) ||
            !TryReadNumber(TransitionStart.Text, out double from) || !TryReadNumber(TransitionEnd.Text, out double to))
        {
            StatusLabel.Text = "Enter finite values for the transition length and endpoints.";
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
                return;
            }
            shape = new(kind, direction, a, b, h);
        }
        FinishTransitionEdit(Session!.UpdateTransition(SelectedRegion, channel, span, length, from, to, shape),
            span, "Updated the transition.");
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
