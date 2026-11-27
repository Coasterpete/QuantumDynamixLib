using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading.Tasks;
using Avalonia.Automation;
using Avalonia.Controls;
using Avalonia.Input;
using Avalonia.Interactivity;
using QuantumDynamix.Math.Functions;
using QuantumDynamix.Math.Timelines;
using QuantumDynamix.Operations;
using static QuantumDynamix.App.WorkspaceView;

namespace QuantumDynamix.App;

internal sealed partial class TrackSegmentSettings : Window
{
    private readonly TimelineEditorSession session;
    private readonly OperationalSegment original;
    private readonly List<DeviceAccelerationRegion> regions;
    private readonly TransitionGraph graph = new([]);
    private bool refreshing;
    private int revision;
    private PoweredTransportMotion? motion;
    private double motionStart;
    private bool lastDragWasBoundary;
    private bool showingSpan;
    // History of this window's draft. Apply Settings commits the result as one main-editor Undo step.
    private readonly Stack<DeviceAccelerationRegion[]> draftUndo = new();
    private readonly Stack<DeviceAccelerationRegion[]> draftRedo = new();

    internal TrackSegmentSettings(TimelineEditorSession session, OperationalSegment segment, bool adding)
    {
        this.session = session;
        original = segment;
        regions = segment.Device?.Regions.ToList() ?? [];
        InitializeComponent();
        ApplySegmentButton.Content = adding ? "Add Track Segment" : "Apply Settings";
        SegmentStart.Text = Format(segment.Start);
        SegmentEnd.Text = Format(segment.End);
        EntrySpeed.Text = Format(segment.Device?.EntrySpeed ?? session.ReconstructionSpeed!.Value);
        SpeedLimit.Text = segment.Device?.SpeedLimit is double limit ? Format(limit) : "";
        SegmentType.ItemsSource = new[] { "Normal Track", "Transport" };
        SegmentType.SelectedIndex = (int)segment.Type;
        SpanKind.ItemsSource = new[] { "Constant" }.Concat(Enum.GetNames<EasingKind>().Where(kind => kind != "Custom")).ToArray();
        SpanDirection.ItemsSource = Enum.GetValues<EasingDirection>();
        AutomationProperties.SetName(graph, "Commanded acceleration in metres per second squared by segment offset");
        AccelerationGraphHost.Content = graph;
        ConnectGraph();
        SegmentType.SelectionChanged += (_, _) => RefreshDevice();
        AccelerationRegionList.SelectionChanged += (_, _) => { if (!refreshing) { ShowRegion(0); } };
        AccelerationTransitionList.SelectionChanged += (_, _) => { if (!refreshing) { ShowTransition(); } };
        SpanKind.SelectionChanged += (sender, e) =>
        {
            ShowEasingParameters();
            // An easing choice applies at once, so the curve and the draft always match the selector.
            if (!showingSpan && AccelerationFields.IsVisible && SpanIndex >= 0) { ApplyTransition(sender, e); }
        };
        SpanDirection.SelectionChanged += (sender, e) =>
        {
            if (!showingSpan && AccelerationFields.IsVisible && SpanIndex >= 0 && SpanDirection.IsEnabled) { ApplyTransition(sender, e); }
        };
        foreach (TextBox input in new[] { SpanLength, SpanStart, SpanEnd, PlateauStart, PlateauEnd, PlateauValue })
        { input.KeyDown += (sender, e) => { if (e.Key == Key.Enter) { ApplyTransition(sender, e); e.Handled = true; } }; }
        foreach (TextBox input in new[] { RegionStart, RegionEnd })
        { input.KeyDown += (sender, e) => { if (e.Key == Key.Enter) { ApplyRegionOffsets(sender, e); e.Handled = true; } }; }
        foreach (TextBox input in new[] { SegmentStart, SegmentEnd, EntrySpeed, SpeedLimit })
        { input.TextChanged += (_, _) => InvalidateMotion(); }
        foreach (TextBox input in new[] { SegmentStart, SegmentEnd })
        { input.TextChanged += (_, _) => { if (!refreshing) { RefreshGraphDomain(); } }; }
        Closed += (_, _) => revision++;
        RefreshDevice();
        ShowRegions(0);
    }

    private bool HasDevice => SegmentType.SelectedIndex == (int)OperationalSegmentType.Transport;
    private int RegionIndex => AccelerationRegionList.SelectedIndex;
    private int SpanIndex => AccelerationTransitionList.SelectedIndex;

    private void RefreshDevice()
    {
        DeviceFields.IsVisible = HasDevice;
        InvalidateMotion();
    }

    private void ShowRegions(int selection, int span = 0)
    {
        refreshing = true;
        AccelerationRegionList.ItemsSource = regions.Select((region, index) =>
            $"Region {index + 1} · {region.Start:G6}–{region.End:G6} m").ToArray();
        AccelerationRegionList.SelectedIndex = System.Math.Min(selection, regions.Count - 1);
        refreshing = false;
        RefreshGraphRegions();
        ShowRegion(span);
    }

    /// <summary>Shows every Acceleration Region of the device on one offset axis spanning the whole segment.</summary>
    private void RefreshGraphRegions()
    {
        double low = 0;
        double high = 1;
        foreach (TimelineKeyframe key in regions.SelectMany(region => region.Keys))
        {
            low = System.Math.Min(low, key.Value);
            high = System.Math.Max(high, key.Value);
        }
        graph.SetRegions(regions.Select(region => region.GraphRegion).ToArray(), low, high,
            regions.Select((_, index) => $"Region {index + 1}").ToArray());
        RefreshGraphDomain();
    }

    private void RefreshGraphDomain()
    {
        bool valid = TryReadSegmentLength(out double length);
        graph.SetDomain(valid ? 0 : null, valid ? length : null);
    }

    private void ShowRegion(int selection)
    {
        int index = RegionIndex;
        RemoveAccelerationRegion.IsEnabled = AccelerationFields.IsVisible = index >= 0;
        GraphHint.Text = regions.Count == 0
            ? "Choose Add Region to create an Acceleration Region in the first unassigned interval of this segment."
            : "Drag keyframes or Acceleration Region boundaries. Click a curve to select its Transition. Drag empty space or scroll to pan and zoom.";
        if (index < 0)
        {
            graph.SetSelection(-1, null, [], -1);
            return;
        }

        DeviceAccelerationRegion region = regions[index];
        RegionStart.Text = Format(region.Start);
        RegionEnd.Text = Format(region.End);
        refreshing = true;
        AccelerationTransitionList.ItemsSource = region.Shapes.Select((shape, span) =>
            $"{region.Keys[span].Coordinate:G6}–{region.Keys[span + 1].Coordinate:G6} m · {shape.Description}").ToArray();
        AccelerationTransitionList.SelectedIndex = System.Math.Min(selection, region.Shapes.Count - 1);
        refreshing = false;

        graph.SetSelection(index, DeviceAccelerationRegion.Channel, region.Keys, -1);
        ShowTransition();
    }

    private void ConnectGraph()
    {
        graph.KeyframePolicy = index => RegionIndex >= 0 && index > 0 && index < regions[RegionIndex].Keys.Count - 1
            ? KeyframeDragMode.Free : KeyframeDragMode.ValueOnly;
        graph.BoundaryPolicy = ResolveBoundaryDrag;
        graph.RegionSelected += SelectRegion;
        graph.BoundarySelected += boundary => SelectRegion(boundary.Left >= 0 ? boundary.Left : boundary.Right);
        graph.CurveSelected += (region, _, span) =>
        {
            SelectRegion(region);
            if (region == RegionIndex) { AccelerationTransitionList.SelectedIndex = span; }
        };
        graph.KeyframeSelected += key =>
        {
            if (RegionIndex >= 0)
            {
                // A keyframe starts the Transition after it. The last keyframe ends the final Transition.
                AccelerationTransitionList.SelectedIndex = System.Math.Min(key, regions[RegionIndex].Shapes.Count - 1);
            }
        };
        graph.PreviewChanged += PreviewGraphEdit;
        graph.EditProposed += CommitGraphEdit;
        graph.InvalidDrag += () => SettingsStatus.Text = lastDragWasBoundary
            ? "Keep the Acceleration Region boundary inside the segment and clear of other Acceleration Regions."
            : "Keep the offset strictly between its neighboring keyframes.";
    }

    private void SelectRegion(int index)
    {
        if (index >= 0 && index != RegionIndex) { AccelerationRegionList.SelectedIndex = index; }
    }

    private bool TryReadSegmentLength(out double length)
    {
        length = 0;
        if (!TryReadNumber(SegmentStart.Text, out double start) || !TryReadNumber(SegmentEnd.Text, out double end) ||
            start < 0 || end <= start)
        {
            return false;
        }
        length = end - start;
        return true;
    }

    /// <summary>A shared boundary moves both adjoining Acceleration Regions. A free boundary stays inside the gap beside it.</summary>
    private BoundaryDragLimits? ResolveBoundaryDrag(GraphBoundary boundary)
    {
        if (!TryReadSegmentLength(out double length)) { return null; }
        double minimum = boundary.Left >= 0 ? regions[boundary.Left].Start
            : boundary.Right > 0 ? System.Math.BitDecrement(regions[boundary.Right - 1].End) : System.Math.BitDecrement(0.0);
        double maximum = boundary.Right >= 0 ? regions[boundary.Right].End
            : boundary.Left + 1 < regions.Count ? System.Math.BitIncrement(regions[boundary.Left + 1].Start)
            : System.Math.BitIncrement(length);
        return minimum < boundary.Station && boundary.Station < maximum ? new(minimum, maximum) : null;
    }

    private DeviceAccelerationRegion[]? ApplyGraphEdit(TransitionEdit edit, out string? error)
    {
        error = null;
        try
        {
            DeviceAccelerationRegion[] candidate = [.. regions];
            switch (edit)
            {
                case TransitionEdit.Keyframe key when RegionIndex >= 0:
                    candidate[RegionIndex] = candidate[RegionIndex].Move(key);
                    break;
                case TransitionEdit.Boundary boundary:
                    if (boundary.Left >= 0) { candidate[boundary.Left] = candidate[boundary.Left].Resize(candidate[boundary.Left].Start, boundary.Proposed); }
                    if (boundary.Right >= 0) { candidate[boundary.Right] = candidate[boundary.Right].Resize(boundary.Proposed, candidate[boundary.Right].End); }
                    break;
                default:
                    return null;
            }
            if (!TryReadSegmentLength(out double length))
            {
                error = "Enter finite, increasing, nonnegative start and end stations.";
                return null;
            }
            error = TimelineEditorSession.ValidateAccelerationRegions(candidate, length);
            return error is null ? candidate : null;
        }
        catch (Exception problem) when (problem is ArgumentException or ArithmeticException)
        {
            error = problem.Message;
            return null;
        }
    }

    // Graph drags update only this dialog's draft. Apply Settings commits the segment as one Undo/Redo action.
    private void PreviewGraphEdit(TransitionEdit? edit)
    {
        if (!graph.IsDragging)
        {
            graph.SetRegionPreview(null);
            return;
        }
        if (edit is null) { return; }
        lastDragWasBoundary = edit is TransitionEdit.Boundary;
        if (ApplyGraphEdit(edit, out string? error) is { } candidate)
        {
            graph.SetRegionPreview(candidate.Select(region => region.GraphRegion).ToArray());
            SettingsStatus.Text = "";
            if (edit is TransitionEdit.Boundary boundary)
            {
                if (boundary.Left == RegionIndex) { RegionEnd.Text = Format(boundary.Proposed); }
                if (boundary.Right == RegionIndex) { RegionStart.Text = Format(boundary.Proposed); }
            }
        }
        else if (error is not null)
        {
            SettingsStatus.Text = $"{error} Showing the last valid preview.";
        }
    }

    private void CommitGraphEdit(TransitionEdit edit)
    {
        bool unchanged = edit switch
        {
            TransitionEdit.Keyframe key => key.Original == key.Proposed,
            TransitionEdit.Boundary boundary => boundary.Original == boundary.Proposed,
            _ => true
        };
        if (unchanged)
        {
            ShowRegion(SpanIndex);
            return;
        }
        if (ApplyGraphEdit(edit, out string? error) is not { } candidate)
        {
            SettingsStatus.Text = error ?? "";
            ShowRegion(SpanIndex);
            return;
        }
        ReplaceDraft(candidate, RegionIndex, SpanIndex);
    }

    private void ZoomInGraph(object? sender, RoutedEventArgs e) => graph.Zoom(.8);
    private void ZoomOutGraph(object? sender, RoutedEventArgs e) => graph.Zoom(1.25);
    private void FitGraph(object? sender, RoutedEventArgs e) => graph.Fit();

    /// <summary>Shows the draft's Transition. The selectors, span list, and graph label always name the applied easing.</summary>
    private void ShowTransition()
    {
        int index = SpanIndex;
        SpanShapeNote.IsVisible = false;
        if (index < 0 || RegionIndex < 0) { return; }
        DeviceAccelerationRegion region = regions[RegionIndex];
        TransitionShape shape = region.Shapes[index];
        SpanLength.Text = Format(region.Keys[index + 1].Coordinate - region.Keys[index].Coordinate);
        SpanLength.IsReadOnly = region.Shapes.Count == 1;
        SpanStart.Text = Format(region.Keys[index].Value);
        SpanEnd.Text = Format(region.Keys[index + 1].Value);
        ShowAppliedEasing(shape);
        PlateauStart.Text = Format(shape.HoldStart);
        PlateauEnd.Text = Format(shape.HoldEnd);
        PlateauValue.Text = Format(shape.HoldValue);
        RemoveAccelerationTransition.IsEnabled = region.Shapes.Count > 1;
        graph.SelectTransition(index, shape.Description);
        SpanShapeNote.IsVisible = shape.Kind is not null && TransitionShape.IsFlat(region.Keys[index], region.Keys[index + 1]);
    }

    private void ShowAppliedEasing(TransitionShape shape)
    {
        showingSpan = true;
        SpanKind.SelectedItem = shape.Name;
        SpanDirection.SelectedItem = shape.Direction;
        ShowEasingParameters();
        showingSpan = false;
    }

    private void ShowEasingParameters()
    {
        string? kind = SpanKind.SelectedItem as string;
        SpanDirection.IsEnabled = kind is not null and not "Constant" and not nameof(EasingKind.Linear);
        PlateauParameters.IsVisible = kind == nameof(EasingKind.Plateau);
    }

    private bool ReadSegmentInterval(out double start, out double end)
    {
        bool validStart = TryReadNumber(SegmentStart.Text, out start);
        bool validEnd = TryReadNumber(SegmentEnd.Text, out end);
        if (validStart && validEnd && start >= 0 && end > start) { return true; }
        SettingsStatus.Text = "Enter finite, increasing, nonnegative start and end stations.";
        return false;
    }

    private OperationalSegment? ReadSettings()
    {
        SettingsStatus.Text = "";
        if (!ReadSegmentInterval(out double start, out double end)) { return null; }
        PoweredTransportDevice? device = null;
        if (HasDevice)
        {
            double? maximum = null;
            if (!string.IsNullOrWhiteSpace(SpeedLimit.Text))
            {
                if (!TryReadNumber(SpeedLimit.Text, out double limit))
                { SettingsStatus.Text = "Enter a finite speed limit or leave it blank."; return null; }
                maximum = limit;
            }
            if (!TryReadNumber(EntrySpeed.Text, out double speed))
            { SettingsStatus.Text = "Enter a finite entry speed."; return null; }
            IReadOnlyList<DeviceAccelerationRegion> profile = original.Device is { } previous && regions.SequenceEqual(previous.Regions)
                ? previous.Regions : regions.ToArray();
            device = new(speed, maximum, profile);
        }
        var candidate = new OperationalSegment(original.Id, start, end,
            HasDevice ? OperationalSegmentType.Transport : OperationalSegmentType.NormalTrack, device);
        SettingsStatus.Text = session.ValidateSegment(candidate) ?? "";
        return SettingsStatus.Text.Length == 0 ? candidate : null;
    }

    private void AddRegion(object? sender, RoutedEventArgs e)
    {
        if (!ReadSegmentInterval(out double start, out double end)) { return; }
        double offset = 0;
        foreach (DeviceAccelerationRegion region in regions)
        {
            if (offset < region.Start) { break; }
            offset = region.End;
        }
        double stop = regions.FirstOrDefault(region => region.Start > offset)?.Start ?? end - start;
        if (offset >= stop)
        { SettingsStatus.Text = "Edit an existing Acceleration Region's offsets to free an interval."; return; }
        ChangeRegion(() => DeviceAccelerationRegion.Zero(offset, stop), -1, 0);
    }

    private void RemoveRegion(object? sender, RoutedEventArgs e)
    {
        int index = RegionIndex;
        ReplaceDraft(regions.Where((_, i) => i != index).ToArray(), index, 0);
    }

    private void ReplaceDraft(DeviceAccelerationRegion[] candidate, int region, int span)
    {
        draftUndo.Push([.. regions]);
        draftRedo.Clear();
        regions.Clear();
        regions.AddRange(candidate);
        InvalidateMotion();
        ShowRegions(region, span);
        RefreshDraftHistory();
    }

    private void UndoDraft(object? sender, RoutedEventArgs e) => RestoreDraft(draftUndo, draftRedo, "Undid");
    private void RedoDraft(object? sender, RoutedEventArgs e) => RestoreDraft(draftRedo, draftUndo, "Redid");

    private void RestoreDraft(Stack<DeviceAccelerationRegion[]> from, Stack<DeviceAccelerationRegion[]> to, string verb)
    {
        if (!from.TryPop(out DeviceAccelerationRegion[]? previous)) { return; }
        graph.CancelDrag();
        int region = RegionIndex;
        int span = SpanIndex;
        to.Push([.. regions]);
        regions.Clear();
        regions.AddRange(previous);
        InvalidateMotion();
        ShowRegions(System.Math.Max(0, region), System.Math.Max(0, span));
        RefreshDraftHistory();
        SettingsStatus.Text = $"{verb} the last Acceleration Region edit in this window.";
    }

    private void RefreshDraftHistory()
    {
        AccelerationUndo.IsEnabled = draftUndo.Count > 0;
        AccelerationRedo.IsEnabled = draftRedo.Count > 0;
    }

    protected override void OnKeyDown(KeyEventArgs e)
    {
        base.OnKeyDown(e);
        // Text boxes keep their own text undo.
        if (e.Handled || e.Source is TextBox) { return; }
        KeyModifiers command = OperatingSystem.IsMacOS() ? KeyModifiers.Meta : KeyModifiers.Control;
        if (e.Key == Key.Z && e.KeyModifiers == command)
        {
            UndoDraft(this, e);
            e.Handled = true;
        }
        else if ((e.Key == Key.Y && e.KeyModifiers == command) || (e.Key == Key.Z && e.KeyModifiers == (command | KeyModifiers.Shift)))
        {
            RedoDraft(this, e);
            e.Handled = true;
        }
    }

    private void ApplyRegionOffsets(object? sender, RoutedEventArgs e)
    {
        if (!TryReadNumber(RegionStart.Text, out double start) || !TryReadNumber(RegionEnd.Text, out double end) ||
            start < 0 || end <= start)
        { SettingsStatus.Text = "Enter increasing, nonnegative Acceleration Region offsets."; return; }
        ChangeRegion(() => regions[RegionIndex].Resize(start, end), RegionIndex, SpanIndex);
    }

    private void SplitTransition(object? sender, RoutedEventArgs e) =>
        ChangeRegion(() => regions[RegionIndex].Split(SpanIndex), RegionIndex, SpanIndex + 1);

    private void MergeTransition(object? sender, RoutedEventArgs e) =>
        ChangeRegion(() => regions[RegionIndex].Merge(SpanIndex), RegionIndex, System.Math.Max(0, SpanIndex - 1));

    private void ApplyTransition(object? sender, RoutedEventArgs e)
    {
        if (RegionIndex < 0 || SpanIndex < 0) { return; }
        bool fromSelector = sender is ComboBox;
        if (!TryReadNumber(SpanLength.Text, out double length) || !TryReadNumber(SpanStart.Text, out double from) ||
            !TryReadNumber(SpanEnd.Text, out double to))
        {
            SettingsStatus.Text = "Enter a finite Transition length and endpoint accelerations.";
            if (fromSelector) { ShowAppliedEasing(regions[RegionIndex].Shapes[SpanIndex]); }
            return;
        }
        EasingKind? kind = Enum.TryParse(SpanKind.SelectedItem as string, out EasingKind selected) ? selected : null;
        var direction = SpanDirection.IsEnabled && SpanDirection.SelectedItem is EasingDirection chosen ? chosen : EasingDirection.In;
        var shape = new TransitionShape(kind, direction);
        if (kind == EasingKind.Plateau)
        {
            if (!TryReadNumber(PlateauStart.Text, out double a) || !TryReadNumber(PlateauEnd.Text, out double b) ||
                !TryReadNumber(PlateauValue.Text, out double h))
            {
                SettingsStatus.Text = "Enter finite Plateau hold fractions.";
                if (fromSelector) { ShowAppliedEasing(regions[RegionIndex].Shapes[SpanIndex]); }
                return;
            }
            shape = new(kind, direction, a, b, h);
        }
        int span = SpanIndex;
        TransitionShape applied = regions[RegionIndex].Shapes[span];
        if (!ChangeRegion(() => regions[RegionIndex].Edit(span, length, from, to, shape), RegionIndex, span))
        {
            // A rejected choice keeps the draft, and the selectors return to the applied easing.
            if (fromSelector) { ShowAppliedEasing(applied); }
        }
        else if (fromSelector)
        {
            SettingsStatus.Text = $"Applied {shape.Description}. Undo restores {applied.Description}.";
        }
    }

    private bool ChangeRegion(Func<DeviceAccelerationRegion> build, int index, int span)
    {
        if (!ReadSegmentInterval(out double start, out double end)) { return false; }
        try
        {
            DeviceAccelerationRegion updated = build();
            if (index >= 0 && updated.Keys.SequenceEqual(regions[index].Keys) && updated.Shapes.SequenceEqual(regions[index].Shapes))
            {
                // An unchanged Transition adds no draft history.
                ShowRegion(span);
                return true;
            }
            DeviceAccelerationRegion[] candidate = regions.Where((_, i) => i != index).Append(updated).OrderBy(region => region.Start).ToArray();
            // A shortened segment can have multiple offending Regions. Allow each to be repaired separately.
            if (updated.End > end - start || candidate.Where(region => region != updated)
                .Any(region => updated.Start < region.End && updated.End > region.Start))
            {
                SettingsStatus.Text = "Keep this Acceleration Region inside the segment and clear of other Acceleration Regions.";
                return false;
            }
            ReplaceDraft(candidate, Array.IndexOf(candidate, updated), span);
            return true;
        }
        catch (Exception error) when (error is ArgumentException or ArithmeticException)
        {
            SettingsStatus.Text = error.Message;
            return false;
        }
    }

    private void InvalidateMotion()
    {
        revision++;
        motion = null;
        MotionReadouts.IsVisible = MotionSummary.IsVisible = false;
        SettingsStatus.Text = "";
    }

    private async void Calculate(object? sender, RoutedEventArgs e)
    {
        InvalidateMotion();
        OperationalSegment? segment = ReadSettings();
        if (segment?.Device is not { } device) { return; }

        int requested = revision;
        CalculateButton.IsEnabled = false;
        SettingsStatus.Text = "Calculating prescribed motion…";
        try
        {
            PoweredTransportMotion calculated = await Task.Run(() => device.Calculate(segment.Length));
            if (revision != requested) { return; }
            motion = calculated;
            motionStart = segment.Start;
            PoweredTransportSample final = calculated.Sample(calculated.End);
            MotionSummary.Text = $"{(calculated.Stopped ? "Stopped" : "Segment end")} at station {segment.Start + calculated.End:G8} m" +
                $" · {final.Speed:G6} m/s · {final.ElapsedSeconds:G6} s";
            SampleOffset.Text = Format(calculated.End);
            MotionReadouts.IsVisible = MotionSummary.IsVisible = true;
            ReadSample(null, e);
        }
        catch (Exception error) when (error is ArgumentException or InvalidOperationException or ArithmeticException)
        {
            if (revision == requested) { SettingsStatus.Text = error.Message; }
        }
        finally { CalculateButton.IsEnabled = true; }
    }

    private void ReadSample(object? sender, RoutedEventArgs e)
    {
        if (motion is null) { return; }
        SettingsStatus.Text = "";
        MotionSample.Text = "";
        try
        {
            if (!TryReadNumber(SampleOffset.Text, out double offset) || offset < 0 || offset > motion.End)
            { SettingsStatus.Text = $"Enter a sample offset from 0 to {Format(motion.End)} m."; return; }
            PoweredTransportSample sample = motion.Sample(offset);
            MotionSample.Text = $"Station {motionStart + offset:G8} m · Speed {sample.Speed:G6} m/s · Elapsed {sample.ElapsedSeconds:G6} s\n" +
                $"Commanded {sample.CommandedAcceleration:G6} m/s² · Calculated {sample.CalculatedAcceleration:G6} m/s²";
        }
        catch (Exception error) when (error is ArgumentException or InvalidOperationException or ArithmeticException)
        { SettingsStatus.Text = error.Message; }
    }

    private void ApplySettings(object? sender, RoutedEventArgs e)
    {
        if (ReadSettings() is not { } segment) { return; }
        if (session.SetSegment(segment) is { } problem) { SettingsStatus.Text = problem; }
        else { Close(true); }
    }

    private void Cancel(object? sender, RoutedEventArgs e) => Close(false);
}
