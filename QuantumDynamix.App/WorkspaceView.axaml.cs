using System;
using System.Globalization;
using System.Linq;
using Avalonia.Automation;
using Avalonia.Controls;
using Avalonia.Input;
using Avalonia.Interactivity;
using QuantumDynamix.Math.Functions;
using QuantumDynamix.Math.Timelines;

namespace QuantumDynamix.App;

internal sealed partial class WorkspaceView : UserControl
{
    private enum WorkspaceMode { Empty, Track, ReferenceTrack, M3Example, KeyframeExample }

    private WorkspaceMode mode;
    private TrackDocument? track;
    private TrackDocument? referenceTrack;
    private TimelineEditorSession? example;
    private TransitionGraph graph = null!;
    private Guid? trackRegion;
    private string? trackChannel;
    private bool updatingControls;

    private bool IsTrack => mode == WorkspaceMode.Track;
    private TrackDocument? DisplayedTrack => mode switch
    {
        WorkspaceMode.Track => track,
        WorkspaceMode.ReferenceTrack => referenceTrack,
        _ => null
    };
    private TimelineEditorSession? Session => DisplayedTrack?.Timeline ?? (mode == WorkspaceMode.KeyframeExample ? example : null);
    private int SelectedRegion => RegionList.SelectedIndex;
    private string? EditingChannel => ActiveChannel.SelectedItem as string;

    public WorkspaceView()
    {
        InitializeComponent();
        InitializeTransitionControls();
        ViewportSurface.StateChanged += UpdateViewportInfo;
        ViewportSurface.RegionPicked += id => SelectRegionInList(IndexOfRegion(id));
        KeyModifiers modifier = OperatingSystem.IsMacOS() ? KeyModifiers.Meta : KeyModifiers.Control;
        NewProjectMenu.InputGesture = NewProjectMenu.HotKey = new(Key.N, modifier);
        OpenProjectMenu.InputGesture = OpenProjectMenu.HotKey = new(Key.O, modifier);
        SaveProjectMenu.InputGesture = SaveProjectMenu.HotKey = new(Key.S, modifier);
        SaveProjectAsMenu.InputGesture = SaveProjectAsMenu.HotKey = new(Key.S, modifier | KeyModifiers.Shift);
        UndoMenu.InputGesture = new(Key.Z, modifier);
        RedoMenu.InputGesture = OperatingSystem.IsMacOS() ? new(Key.Z, modifier | KeyModifiers.Shift) : new(Key.Y, modifier);
        Open(WorkspaceMode.Empty);
    }

    private void Open(WorkspaceMode next)
    {
        mode = next;
        editingTransitions = false;
        if (mode == WorkspaceMode.KeyframeExample)
        {
            example ??= KeyframeExample.Create();
        }

        graph = new(mode == WorkspaceMode.M3Example ? TransitionGraphFixture.CreateRegions() : []);
        graph.EditProposed += CompleteDrag;
        graph.PreviewChanged += PreviewDrag;
        graph.KeyframeSelected += index =>
        {
            if (editingTransitions) { SetTransitionMode(false); }
            KeyframeList.SelectedIndex = index;
        };
        if (IsTrack) { graph.TransitionSelected += SelectTransitionOnGraph; }
        graph.RegionSelected += SelectRegionInList;
        graph.InvalidDrag += () => StatusLabel.Text = "Keep the station strictly between its neighboring keyframes.";
        TransitionGraphHost.Content = graph;

        ModeLabel.Text = mode switch
        {
            WorkspaceMode.Track => $"Track · {track!.Identity.Name}",
            WorkspaceMode.ReferenceTrack => referenceTrack!.Identity.Name,
            WorkspaceMode.M3Example => "M3 reference example",
            WorkspaceMode.KeyframeExample => "Editable keyframe example",
            _ => "Empty workspace"
        };
        StatusLabel.Text = mode switch
        {
            WorkspaceMode.Track when track!.Timeline.Regions.Count == 0 => "Use Append Region to start the track.",
            WorkspaceMode.Track => "Select a region to edit its profiles.",
            WorkspaceMode.ReferenceTrack => "Development fixture · Edit the middle banking keyframe in radians. Changes stay in this reference session.",
            WorkspaceMode.M3Example => "Example profiles · Use the Region List and channel toggles to inspect them.",
            WorkspaceMode.KeyframeExample => "Dimensionless example · Linear spans · Fixed region endpoints",
            _ => "Use New Track to start authoring, or open an example from File."
        };
        ActiveTrackMenu.IsEnabled = track is not null && !IsTrack;
        RefreshRegions(IsTrack ? trackRegion : null, Session is null ? -1 : 0, IsTrack ? trackChannel : null);
        if (Session is null) { RefreshTrackViewport(); }
    }

    private void RefreshRegions(Guid? selection, int fallback, string? channel)
    {
        TimelineEditorSession? session = Session;
        if (session is not null)
        {
            RefreshGraph(session);
        }

        updatingControls = true;
        var items = graph.Regions.Select((region, index) =>
        {
            var item = new ListBoxItem { Content = graph.RegionName(index) };
            string range = DescribeExtent(region);
            if (session?.SupportsForceRegions == true) { range = $"{DescribeMode(session.Regions[index].Mode)} · {range}"; }
            ToolTip.SetTip(item, range);
            AutomationProperties.SetHelpText(item, range);
            return item;
        }).ToArray();
        RegionList.ItemsSource = items;
        int index = selection is Guid id ? IndexOfRegion(id) : -1;
        RegionList.SelectedIndex = index >= 0 ? index : System.Math.Min(fallback, items.Length - 1);
        if (SelectedRegion >= 0)
        {
            RegionList.ScrollIntoView(SelectedRegion);
        }

        GraphViewMenu.IsEnabled = GraphNavigation.IsEnabled = items.Length != 0;
        ChannelPanel.IsVisible = graph.Channels.Count != 0;
        ChannelVisibility.ItemsSource = graph.Channels.Select(CreateChannelToggle).ToArray();
        AppendRegionButton.IsEnabled = IsTrack;
        PrependRegionButton.IsEnabled = IsTrack && items.Length != 0 && graph.Regions[0].Start > 0;
        updatingControls = false;
        RefreshChannels(channel);
    }

    private CheckBox CreateChannelToggle(string channel)
    {
        TransitionGraph owner = graph;
        var check = new CheckBox
        {
            Content = channel,
            Foreground = owner.ChannelBrush(channel),
            IsChecked = owner.IsChannelVisible(channel),
            MinHeight = 23,
            FontSize = 11,
            Padding = new(3, 0)
        };
        check.Classes.Add("profile-visibility");
        AutomationProperties.SetName(check, $"Show {channel} channel");
        check.IsCheckedChanged += (_, _) => owner.SetChannelVisible(channel, check.IsChecked == true);
        return check;
    }

    private void SelectRegion(object? sender, SelectionChangedEventArgs e)
    {
        if (!updatingControls)
        {
            RefreshChannels(EditingChannel);
        }
    }

    private int IndexOfRegion(Guid id) => Session?.Regions.ToList().FindIndex(entry => entry.Id == id) ?? -1;

    private void SelectRegionInList(int index)
    {
        if (index >= 0)
        {
            RegionList.SelectedIndex = index;
            RegionList.ScrollIntoView(index);
        }
    }

    private static string DescribeExtent(Region region) =>
        $"{region.Start:G6}–{region.End:G6} m · {region.End - region.Start:G6} m long";

    private void RefreshChannels(string? previous)
    {
        int selected = SelectedRegion;
        AuthoredRegion? region = Session is { } session && selected >= 0 ? session.Regions[selected] : null;
        bool canEditProfile = region is not null;
        updatingControls = true;
        string[] channels = region?.Keyframes.Keys.ToArray() ?? [];
        ActiveChannel.ItemsSource = channels;
        int index = Array.IndexOf(channels, previous);
        ActiveChannel.SelectedIndex = index >= 0 || channels.Length == 0 ? index : region?.Mode switch
        {
            RegionAuthoringMode.Force => Array.IndexOf(channels, RegionTrackGeometry.NormalForceChannel),
            RegionAuthoringMode.Geometric => Array.IndexOf(channels, RegionTrackGeometry.PitchChangeChannel),
            _ => 0
        };
        ActiveChannelPanel.IsVisible = canEditProfile;
        AddChannelButton.IsVisible = IsTrack;
        AddChannelMenu.IsEnabled = RegionPropertiesMenu.IsEnabled = DeleteRegionButton.IsEnabled = IsTrack && canEditProfile;
        KeyframePanel.IsVisible = Session is not null;
        RegionExtent.Text = selected >= 0 ? DescribeExtent(graph.Regions[selected]) : null;
        RegionExtent.IsVisible = selected >= 0;
        updatingControls = false;
        RefreshKeyframes(-1);
        ViewportSurface.SetActiveRegion(DisplayedTrack is null ? null : region?.Id);

        if (selected >= 0 && (graph.Regions[selected].End <= graph.Viewport.Start ||
            graph.Regions[selected].Start >= graph.Viewport.End))
        {
            graph.Fit();
        }
    }

    private void SelectChannel(object? sender, SelectionChangedEventArgs e)
    {
        if (!updatingControls)
        {
            RefreshKeyframes(-1);
        }
    }

    private void RefreshKeyframes(int selection, int transition = 0)
    {
        string? channel = EditingChannel;
        var keys = channel is null ? [] : Session!.Keys(SelectedRegion, channel);
        string? description = channel switch
        {
            RegionTrackGeometry.LateralChannel => "Authored coordinate acceleration along the banked left axis, in m/s².",
            RegionTrackGeometry.VerticalChannel => "Authored coordinate acceleration along the banked up axis, in m/s².",
            RegionTrackGeometry.LateralForceChannel => "Authored specific force along the rider's left axis, in g.",
            RegionTrackGeometry.NormalForceChannel => "Authored specific force along the rider's up axis, in g.",
            RegionTrackGeometry.PitchChangeChannel => "Ground-relative pitch change in degrees per heartline metre. Positive raises the tangent.",
            RegionTrackGeometry.YawChangeChannel => "Ground-relative heading change in degrees per heartline metre. Positive turns left.",
            TrackDocument.BankingChannel => "Authored bank angle in radians. Positive bank rotates left toward up.",
            _ => null
        };
        ToolTip.SetTip(ValueInput, description);
        AutomationProperties.SetHelpText(ValueInput, description);

        updatingControls = true;
        KeyframeList.ItemsSource = keys.Select((key, index) =>
            $"{key.Coordinate:G6} m   {key.Value:G6}{(index == 0 || index == keys.Count - 1 ? "  fixed" : "")}").ToArray();
        KeyframeList.SelectedIndex = selection;
        updatingControls = false;
        RefreshEditMode();
        UpdateKeyframeSelection();
        RefreshTransitions(transition);
        StationInput.IsEnabled = ValueInput.IsEnabled = AddKeyButton.IsEnabled = channel is not null;
        UndoMenu.IsEnabled = UndoButton.IsEnabled = Session?.CanUndo == true;
        RedoMenu.IsEnabled = RedoButton.IsEnabled = Session?.CanRedo == true;
        if (IsTrack)
        {
            trackRegion = SelectedRegion >= 0 ? Session!.Regions[SelectedRegion].Id : null;
            trackChannel = channel;
        }
        RefreshProjectControls();
    }

    private void SelectKeyframe(object? sender, SelectionChangedEventArgs e)
    {
        if (!updatingControls)
        {
            UpdateKeyframeSelection();
        }
    }

    private void UpdateKeyframeSelection()
    {
        string? channel = EditingChannel;
        var keys = channel is null ? [] : Session!.Keys(SelectedRegion, channel);
        int index = KeyframeList.SelectedIndex;
        graph.SetSelection(SelectedRegion, channel, keys, index);
        ApplyKeyButton.IsEnabled = DeleteKeyButton.IsEnabled = index > 0 && index < keys.Count - 1;
        StationInput.Text = index >= 0 ? Format(keys[index].Coordinate) : "";
        ValueInput.Text = index >= 0 ? Format(keys[index].Value) : "";
        if (!TransitionFields.IsVisible) { RefreshCalculatedForce(index >= 0 ? keys[index].Coordinate : null); }
    }

    private void RefreshCalculatedForce(double? selectedStation)
    {
        CalculatedForce.IsVisible = false;
        if (SelectedRegion < 0 || Session is not { Geometry: { } geometry } session ||
            session.Regions[SelectedRegion] is not { Mode: RegionAuthoringMode.Force or RegionAuthoringMode.Geometric } region)
        {
            return;
        }

        double station = selectedStation ?? region.Definition.Start;
        EvaluationSide side = station == region.Definition.End ? EvaluationSide.Left : EvaluationSide.Right;
        try
        {
            var force = geometry.CalculateSpecificForceG(station, side);
            CalculatedForce.Text = $"Calculated at {station:G6} m\nNormal {force.Z:F3} g · Lateral {force.Y:F3} g";
            if (region.Mode == RegionAuthoringMode.Geometric)
            {
                var (pitch, heading, pitchRate, yawRate, curvature) = geometry.CalculateGeometry(station, side);
                CalculatedForce.Text += $"\nCurvature {curvature.Length:G5} 1/m\nElevation {pitch:F2}°" +
                    (heading is double azimuth
                        ? $" · Heading {azimuth:F2}°\nElevation rate {pitchRate:F3} °/m · Heading rate {yawRate:F3} °/m"
                        : " · Heading and angle rates undefined at vertical");
            }
        }
        catch (Exception error) when (error is ArgumentException or InvalidOperationException or ArithmeticException)
        {
            CalculatedForce.Text = $"The reconstruction could not be evaluated at {station:G6} m.";
        }
        CalculatedForce.IsVisible = true;
    }

    private bool ReadKey(out TimelineKeyframe key)
    {
        key = default;
        if (!TryReadNumber(StationInput.Text, out double station) || !TryReadNumber(ValueInput.Text, out double value))
        {
            StatusLabel.Text = "Enter a finite number for both station and value.";
            return false;
        }
        key = new(station, value);
        return true;
    }

    private static string Format(double value) => value.ToString("R", CultureInfo.CurrentCulture);

    private static bool TryReadNumber(string? text, out double value) =>
        double.TryParse(text, NumberStyles.Float, CultureInfo.CurrentCulture, out value) && double.IsFinite(value);

    private void AddKeyframe(object? sender, RoutedEventArgs e)
    {
        if (EditingChannel is { } channel && ReadKey(out TimelineKeyframe key))
        {
            string? error = Session!.Add(SelectedRegion, channel, key, out int index);
            FinishKeyEdit(error, index, $"Added a {channel} keyframe.");
        }
    }

    private void ApplyKeyframe(object? sender, RoutedEventArgs e)
    {
        if (EditingChannel is { } channel && ReadKey(out TimelineKeyframe key))
        {
            int index = KeyframeList.SelectedIndex;
            string? error = Session!.Move(SelectedRegion, channel, index, key);
            FinishKeyEdit(error, index, $"Updated the {channel} keyframe.");
        }
    }

    private void DeleteKeyframe(object? sender, RoutedEventArgs e)
    {
        if (EditingChannel is { } channel)
        {
            string? error = Session!.Delete(SelectedRegion, channel, KeyframeList.SelectedIndex);
            FinishKeyEdit(error, -1, $"Deleted the {channel} keyframe.");
        }
    }

    private void CompleteDrag(TransitionEdit edit)
    {
        if (edit is not TransitionEdit.Keyframe key || EditingChannel != key.Channel || key.Original == key.Proposed)
        {
            return;
        }

        int index = KeyframeList.SelectedIndex;
        if (index >= 0 && Session!.Keys(SelectedRegion, key.Channel)[index] == key.Original)
        {
            string? error = Session.Move(SelectedRegion, key.Channel, index, key.Proposed);
            FinishKeyEdit(error, index, $"Moved the {key.Channel} keyframe.");
        }
    }

    private void PreviewDrag(TransitionEdit? edit)
    {
        var viewport = graph.Viewport;
        int index = SelectedRegion;
        if (edit is TransitionEdit.Keyframe key && EditingChannel == key.Channel &&
            key.Proposed.Value >= viewport.MinimumValue && key.Proposed.Value <= viewport.MaximumValue &&
            Session!.PreviewMove(index, key.Channel, KeyframeList.SelectedIndex, key.Proposed) is { } preview)
        {
            Region[] visible = graph.Regions.ToArray();
            visible[index] = preview;
            graph.SetRegionPreview(visible);
        }
        else
        {
            graph.SetRegionPreview(null);
        }
    }

    private void FinishKeyEdit(string? error, int selection, string success)
    {
        if (error is null)
        {
            RefreshGraph(Session!);
            RefreshKeyframes(selection);
        }
        StatusLabel.Text = error ?? success;
    }

    private void RefreshGraph(TimelineEditorSession session)
    {
        graph.SetRegions(session.Regions.Select(entry => entry.Definition).ToArray(), session.MinimumValue,
            session.MaximumValue, session.Regions.Select(entry => entry.Name).ToArray());
        RefreshTrackViewport();
    }

    private void Undo(object? sender, RoutedEventArgs e) => RestoreHistory(undo: true);
    private void Redo(object? sender, RoutedEventArgs e) => RestoreHistory(undo: false);

    private void RestoreHistory(bool undo)
    {
        if (Session is not { } session)
        {
            return;
        }
        int index = SelectedRegion;
        int transition = TransitionSpan.SelectedIndex;
        Guid? selection = index >= 0 ? session.Regions[index].Id : null;
        if (undo ? session.Undo() : session.Redo())
        {
            RefreshRegions(selection, System.Math.Max(0, index), EditingChannel);
            RefreshTransitions(transition);
            StatusLabel.Text = undo ? "Undid the last edit." : "Redid the last edit.";
        }
    }

    protected override void OnKeyDown(KeyEventArgs e)
    {
        base.OnKeyDown(e);
        if (e.Source is TextBox)
        {
            return;
        }
        if (UndoMenu.InputGesture?.Matches(e) == true || RedoMenu.InputGesture?.Matches(e) == true ||
            (e.Key == Key.Z && e.KeyModifiers == (KeyModifiers.Control | KeyModifiers.Shift)))
        {
            RestoreHistory(UndoMenu.InputGesture?.Matches(e) == true);
            e.Handled = true;
        }
    }

    private void ShowEmptyWorkspace(object? sender, RoutedEventArgs e) => Open(WorkspaceMode.Empty);
    private void ShowActiveTrack(object? sender, RoutedEventArgs e) => Open(WorkspaceMode.Track);
    private void ShowM3Example(object? sender, RoutedEventArgs e) => Open(WorkspaceMode.M3Example);
    private void ShowKeyframeExample(object? sender, RoutedEventArgs e) => Open(WorkspaceMode.KeyframeExample);
    private void ShowReferenceTrack(object? sender, RoutedEventArgs e)
    {
        referenceTrack ??= ReferenceTrack.Create();
        Open(WorkspaceMode.ReferenceTrack);
    }

    internal void OpenTrack(TrackDocument document, string? path = null)
    {
        track = document;
        projectPath = path;
        trackRegion = null;
        trackChannel = null;
        Open(WorkspaceMode.Track);
    }

    private void RefreshTrackViewport()
    {
        TrackDocument? document = DisplayedTrack;
        var scene = document?.SampleWireframe(RegionTrackGeometry.DisplayIntervals)
            ?? TrackWireframe.Empty("Select a track with defined geometry.");
        ViewportSurface.SetScene(document?.Identity.Id, scene);
    }

    private void UpdateViewportInfo()
    {
        ViewportDisplayMenu.IsEnabled = ViewportSurface.HasGeometry;
        string? view = ViewportSurface.HasGeometry ? ViewportSurface.View.ToString() : null;
        string? selected = ViewportSurface.Selection?.Label;
        string[] labels = new[] { view, selected }.OfType<string>()
            .Concat(ViewportSurface.Scene.Diagnostics.Distinct()).ToArray();
        ViewportInfo.Text = string.Join(" · ", labels);
    }

    private void ChangeTrackView(object? sender, RoutedEventArgs e)
    {
        if (sender is MenuItem { Tag: string tag } && Enum.TryParse(tag, out TrackView view))
        {
            ViewportSurface.SetView(view);
        }
    }

    private void FrameTrack(object? sender, RoutedEventArgs e) => ViewportSurface.FrameAll();
    private void ZoomIn(object? sender, RoutedEventArgs e) => graph.Zoom(.8);
    private void ZoomOut(object? sender, RoutedEventArgs e) => graph.Zoom(1.25);
    private void PanLeft(object? sender, RoutedEventArgs e) => graph.Pan(-.2);
    private void PanRight(object? sender, RoutedEventArgs e) => graph.Pan(.2);
    private void FitGraph(object? sender, RoutedEventArgs e) => graph.Fit();
    private void FocusWorkspace(object? sender, RoutedEventArgs e) => ViewportSurface.Focus();

    private void Exit(object? sender, RoutedEventArgs e)
    {
        (TopLevel.GetTopLevel(this) as Window)?.Close();
    }
}
