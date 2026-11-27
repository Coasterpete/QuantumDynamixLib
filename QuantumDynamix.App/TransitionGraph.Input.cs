using System;
using System.Collections.Generic;
using Avalonia;
using Avalonia.Input;
using QuantumDynamix.Math.Functions;
using QuantumDynamix.Math.Timelines;

namespace QuantumDynamix.App;

/// <summary>How a keyframe of the editable profile may move on the graph.</summary>
internal enum KeyframeDragMode { None, ValueOnly, Free }

/// <summary>A Region boundary on the distance axis. <paramref name="Left"/> ends and <paramref name="Right"/> starts here.</summary>
/// <remarks>Either index is -1 when no Region adjoins on that side.</remarks>
internal readonly record struct GraphBoundary(double Station, int Left, int Right);

/// <summary>Exclusive station limits for one boundary drag.</summary>
internal readonly record struct BoundaryDragLimits(double Minimum, double Maximum);

internal sealed partial class TransitionGraph
{
    // Pointer movement below this many pixels on an axis leaves that coordinate unchanged.
    private const double DragThreshold = 3;
    private const double BoundaryHitRadius = 5;
    private const double CurveHitRadius = 7;

    private enum Gesture { None, Edit, Pan }

    private IPointer? dragPointer;
    private Gesture gesture;
    private MouseButton gestureButton;
    private Point pressPoint;
    private double panStart;
    private double panEnd;
    private bool panMoved;
    private int pressRegion = -1;
    private bool stationUnlocked;
    private bool valueUnlocked;
    private GraphBoundary? activeBoundary;
    private GraphBoundary? hoverBoundary;
    private Cursor? resizeCursor;
    private Cursor? handCursor;

    /// <summary>Selects which keyframes of the editable profile can be dragged. The default allows interior keyframes.</summary>
    internal Func<int, KeyframeDragMode>? KeyframePolicy { get; set; }

    /// <summary>Returns drag limits for a boundary, or null when it is display-only. Boundaries are inert without a policy.</summary>
    internal Func<GraphBoundary, BoundaryDragLimits?>? BoundaryPolicy { get; set; }

    internal bool IsDragging => gesture == Gesture.Edit;
    internal bool IsPanning => gesture == Gesture.Pan;
    internal TransitionEdit? DragCandidate => IsDragging ? edit.Candidate : null;

    protected override void OnPointerPressed(PointerPressedEventArgs e)
    {
        base.OnPointerPressed(e);
        TransitionGraphViewport viewport = Viewport;
        if (dragPointer is not null || !viewport.HasPlot)
        {
            return;
        }

        Point point = e.GetPosition(this);
        PointerUpdateKind kind = e.GetCurrentPoint(this).Properties.PointerUpdateKind;
        if (kind is PointerUpdateKind.MiddleButtonPressed or PointerUpdateKind.RightButtonPressed)
        {
            // Navigation never proposes an edit.
            if (regions.Count > 0)
            {
                Focus();
                BeginGesture(e, point, Gesture.Pan,
                    kind == PointerUpdateKind.MiddleButtonPressed ? MouseButton.Middle : MouseButton.Right);
            }
            return;
        }
        if (kind != PointerUpdateKind.LeftButtonPressed)
        {
            return;
        }

        GraphPosition position = viewport.Unproject(point);
        if (resolveDragTarget is not null)
        {
            // An explicit interaction policy replaces the built-in hit testing.
            if (resolveDragTarget(position) is { } explicitTarget)
            {
                TryBeginEdit(e, point, explicitTarget, position, null);
            }
            return;
        }
        if (regions.Count == 0)
        {
            return;
        }
        Focus();

        if (editChannel is { } channel && !hiddenChannels.Contains(channel) && viewport.Plot.Inflate(8).Contains(point))
        {
            int hit = HitKeyframe(viewport, point);
            if (hit >= 0)
            {
                KeyframeDragMode mode = KeyframeMode(hit);
                // A fixed keyframe drawn on a movable boundary does not hide the boundary handle.
                if (mode != KeyframeDragMode.None || DraggableBoundaryAt(viewport, point) is null)
                {
                    selectedKey = hit;
                    KeyframeSelected?.Invoke(hit);
                    e.Handled = true;
                    if (mode != KeyframeDragMode.None && hit < keys.Count)
                    {
                        TryBeginEdit(e, point, KeyframeTarget(channel, keys[hit], hit, mode), position, null);
                    }
                    InvalidateVisual();
                    return;
                }
            }
        }

        if (BoundaryPolicy is not null && HitBoundary(viewport, point) is { } boundary)
        {
            e.Handled = true;
            BoundarySelected?.Invoke(boundary);
            if (BoundaryPolicy(boundary) is { } limits)
            {
                TryBeginEdit(e, point, BoundaryTarget(boundary, limits), position, boundary);
            }
            return;
        }

        if (viewport.Plot.Inflate(2).Contains(point) && HitCurve(viewport, point) is { } curve)
        {
            e.Handled = true;
            CurveSelected?.Invoke(curve.Region, curve.Channel, curve.Span);
            return;
        }

        int region = RegionAt(position.Distance);
        if (point.Y >= 0 && point.Y < viewport.Plot.Top - 5 && point.X >= viewport.Plot.Left &&
            point.X <= viewport.Plot.Right)
        {
            if (region >= 0)
            {
                RegionSelected?.Invoke(region);
                e.Handled = true;
            }
            return;
        }
        if (viewport.Plot.Contains(point))
        {
            // Click selects the Region under the pointer. Dragging empty graph space pans instead.
            pressRegion = region;
            BeginGesture(e, point, Gesture.Pan, MouseButton.Left);
        }
    }

    protected override void OnPointerMoved(PointerEventArgs e)
    {
        base.OnPointerMoved(e);
        Point point = e.GetPosition(this);
        if (dragPointer == e.Pointer)
        {
            if (gesture == Gesture.Edit && Viewport is { HasPlot: true } viewport)
            {
                edit.Update(viewport.Unproject(point));
                PreviewChanged?.Invoke(edit.Candidate);
                InvalidateVisual();
            }
            else if (gesture == Gesture.Pan)
            {
                PanTo(point);
            }
            e.Handled = true;
            return;
        }
        if (dragPointer is null)
        {
            UpdateHover(point);
        }
    }

    protected override void OnPointerReleased(PointerReleasedEventArgs e)
    {
        base.OnPointerReleased(e);
        if (dragPointer != e.Pointer || e.InitialPressMouseButton != gestureButton)
        {
            return;
        }

        e.Handled = true;
        if (gesture == Gesture.Pan)
        {
            bool click = !panMoved && gestureButton == MouseButton.Left;
            int region = pressRegion;
            EndDrag();
            e.Pointer.Capture(null);
            if (click && region >= 0)
            {
                RegionSelected?.Invoke(region);
            }
            return;
        }

        TransitionEdit? proposal = Viewport is { HasPlot: true } viewport
            ? edit.Finish(viewport.Unproject(e.GetPosition(this))) : null;
        EndDrag();
        e.Pointer.Capture(null);
        if (proposal is not null)
        {
            EditProposed?.Invoke(proposal);
        }
        else
        {
            InvalidDrag?.Invoke();
        }
    }

    protected override void OnPointerCaptureLost(PointerCaptureLostEventArgs e)
    {
        base.OnPointerCaptureLost(e);
        if (dragPointer == e.Pointer)
        {
            EndDrag();
        }
    }

    protected override void OnPointerExited(PointerEventArgs e)
    {
        base.OnPointerExited(e);
        if (dragPointer is null && hoverBoundary is not null)
        {
            hoverBoundary = null;
            InvalidateVisual();
        }
    }

    protected override void OnPointerWheelChanged(PointerWheelEventArgs e)
    {
        base.OnPointerWheelChanged(e);
        if (regions.Count == 0 || gesture == Gesture.Edit || !Viewport.HasPlot)
        {
            return;
        }

        if (e.KeyModifiers.HasFlag(KeyModifiers.Shift) || (e.Delta.Y == 0 && e.Delta.X != 0))
        {
            double steps = e.Delta.X != 0 ? e.Delta.X : e.Delta.Y;
            Pan(-.1 * steps);
        }
        else if (e.Delta.Y != 0)
        {
            ZoomAt(e.GetPosition(this).X, System.Math.Pow(.85, e.Delta.Y));
        }
        e.Handled = true;
    }

    protected override void OnKeyDown(KeyEventArgs e)
    {
        base.OnKeyDown(e);
        if (e.Key == Key.Escape && dragPointer is not null)
        {
            CancelDrag();
            e.Handled = true;
        }
    }

    /// <summary>Ends any drag or pan without proposing an edit.</summary>
    internal void CancelDrag()
    {
        if (dragPointer is { } pointer)
        {
            EndDrag();
            pointer.Capture(null);
        }
    }

    private void BeginGesture(PointerPressedEventArgs e, Point point, Gesture kind, MouseButton button)
    {
        gesture = kind;
        gestureButton = button;
        pressPoint = point;
        panStart = Viewport.Start;
        panEnd = Viewport.End;
        panMoved = false;
        dragPointer = e.Pointer;
        e.Pointer.Capture(this);
        e.Handled = true;
    }

    private void TryBeginEdit(PointerPressedEventArgs e, Point point, TransitionDragTarget target, GraphPosition position,
        GraphBoundary? boundary)
    {
        stationUnlocked = false;
        valueUnlocked = false;
        if (edit.TryBegin(target, position))
        {
            Focus();
            BeginGesture(e, point, Gesture.Edit, MouseButton.Left);
            activeBoundary = boundary;
            InvalidateVisual();
        }
    }

    private void EndDrag()
    {
        bool editing = gesture == Gesture.Edit;
        dragPointer = null;
        gesture = Gesture.None;
        activeBoundary = null;
        pressRegion = -1;
        if (editing)
        {
            edit.Cancel();
            PreviewChanged?.Invoke(null);
        }
        InvalidateVisual();
    }

    private void PanTo(Point point)
    {
        Rect plot = Viewport.Plot;
        double offset = point.X - pressPoint.X;
        if (!panMoved && System.Math.Abs(offset) < DragThreshold || plot.Width <= 0)
        {
            return;
        }
        panMoved = true;
        double span = panEnd - panStart;
        SetView(panStart - offset / plot.Width * span + span / 2, span);
    }

    /// <summary>Zooms about a horizontal control coordinate so the station under it stays in place.</summary>
    internal void ZoomAt(double x, double factor)
    {
        TransitionGraphViewport viewport = Viewport;
        if (regions.Count == 0 || !viewport.HasPlot)
        {
            return;
        }
        double fraction = System.Math.Clamp((x - viewport.Plot.Left) / viewport.Plot.Width, 0, 1);
        double station = viewport.Start + fraction * (viewport.End - viewport.Start);
        double span = (viewport.End - viewport.Start) * factor;
        SetView(station + (.5 - fraction) * span, span);
    }

    private KeyframeDragMode KeyframeMode(int index) => KeyframePolicy?.Invoke(index)
        ?? (index > 0 && index < keys.Count - 1 ? KeyframeDragMode.Free : KeyframeDragMode.None);

    private TransitionDragTarget KeyframeTarget(string channel, TimelineKeyframe original, int index, KeyframeDragMode mode)
    {
        // A value-only keyframe keeps its station, so its admissible interval contains only that station.
        double previous = mode == KeyframeDragMode.Free ? keys[index - 1].Coordinate : System.Math.BitDecrement(original.Coordinate);
        double next = mode == KeyframeDragMode.Free ? keys[index + 1].Coordinate : System.Math.BitIncrement(original.Coordinate);
        return new(previous, next, (origin, pointer) =>
        {
            TransitionGraphViewport viewport = Viewport;
            Vector moved = viewport.Project(pointer) - viewport.Project(origin);
            stationUnlocked |= mode == KeyframeDragMode.Free && System.Math.Abs(moved.X) >= DragThreshold;
            valueUnlocked |= System.Math.Abs(moved.Y) >= DragThreshold;
            // Past the threshold, each coordinate follows the exact pointer displacement without rounding.
            double station = stationUnlocked ? original.Coordinate + (pointer.Distance - origin.Distance) : original.Coordinate;
            double value = valueUnlocked ? original.Value + (pointer.Value - origin.Value) : original.Value;
            return new TransitionEdit.Keyframe(channel, original, new(station, value));
        });
    }

    private TransitionDragTarget BoundaryTarget(GraphBoundary boundary, BoundaryDragLimits limits) =>
        new(limits.Minimum, limits.Maximum, (origin, pointer) =>
        {
            TransitionGraphViewport viewport = Viewport;
            stationUnlocked |= System.Math.Abs(viewport.ProjectDistance(pointer.Distance) -
                viewport.ProjectDistance(origin.Distance)) >= DragThreshold;
            double station = stationUnlocked ? boundary.Station + (pointer.Distance - origin.Distance) : boundary.Station;
            return new TransitionEdit.Boundary(boundary.Station, station, boundary.Left, boundary.Right);
        });

    private int RegionAt(double station)
    {
        for (int i = 0; i < regions.Count; i++)
        {
            if (station >= regions[i].Start && station <= regions[i].End)
            {
                return i;
            }
        }
        return -1;
    }

    private static IEnumerable<GraphBoundary> BoundariesOf(IReadOnlyList<Region> list)
    {
        for (int i = 0; i < list.Count; i++)
        {
            if (i == 0 || list[i - 1].End != list[i].Start)
            {
                yield return new(list[i].Start, -1, i);
            }
            bool shared = i + 1 < list.Count && list[i].End == list[i + 1].Start;
            yield return new(list[i].End, i, shared ? i + 1 : -1);
        }
    }

    private GraphBoundary? HitBoundary(TransitionGraphViewport viewport, Point point)
    {
        Rect plot = viewport.Plot;
        if (point.Y < plot.Top - 8 || point.Y > plot.Bottom + 16)
        {
            return null;
        }
        GraphBoundary? hit = null;
        double closest = BoundaryHitRadius;
        foreach (GraphBoundary boundary in BoundariesOf(regions))
        {
            if (boundary.Station < viewport.Start || boundary.Station > viewport.End)
            {
                continue;
            }
            double distance = System.Math.Abs(viewport.ProjectDistance(boundary.Station) - point.X);
            if (distance <= closest)
            {
                closest = distance;
                hit = boundary;
            }
        }
        return hit;
    }

    private GraphBoundary? DraggableBoundaryAt(TransitionGraphViewport viewport, Point point) =>
        BoundaryPolicy is not null && HitBoundary(viewport, point) is { } boundary && BoundaryPolicy(boundary) is not null
            ? boundary : null;

    private (int Region, string Channel, int Span)? HitCurve(TransitionGraphViewport viewport, Point point)
    {
        double station = viewport.Unproject(point).Distance;
        (int, string, int)? hit = null;
        double closest = CurveHitRadius;
        for (int index = 0; index < regions.Count; index++)
        {
            Region region = regions[index];
            if (!(station >= region.Start && station <= region.End))
            {
                continue;
            }
            foreach (string channel in channels)
            {
                if (hiddenChannels.Contains(channel) || !region.Channels.TryGetValue(channel, out ScalarTimeline? timeline))
                {
                    continue;
                }
                EvaluationSide side = station >= timeline.Maximum ? EvaluationSide.Left : EvaluationSide.Right;
                double value = timeline.Evaluate(station, side).Value;
                // Favor the edited profile when curves overlap.
                double distance = System.Math.Abs(viewport.Project(new(station, value)).Y - point.Y) -
                    (index == activeRegion && channel == editChannel ? 2 : 0);
                if (distance <= closest)
                {
                    closest = distance;
                    int span = 0;
                    while (span + 1 < timeline.Segments.Count && station >= timeline.Segments[span + 1].Minimum) { span++; }
                    hit = (index, channel, span);
                }
            }
        }
        return hit;
    }

    private void UpdateHover(Point point)
    {
        TransitionGraphViewport viewport = Viewport;
        if (!viewport.HasPlot || regions.Count == 0 || resolveDragTarget is not null)
        {
            return;
        }
        GraphBoundary? boundary = null;
        bool overKeyframe = editChannel is { } channel && !hiddenChannels.Contains(channel) &&
            HitKeyframe(viewport, point) is int key && key >= 0 && KeyframeMode(key) != KeyframeDragMode.None;
        if (!overKeyframe)
        {
            boundary = DraggableBoundaryAt(viewport, point);
        }
        Cursor = overKeyframe ? handCursor ??= new Cursor(StandardCursorType.Hand)
            : boundary is not null ? resizeCursor ??= new Cursor(StandardCursorType.SizeWestEast)
            : null;
        if (boundary != hoverBoundary)
        {
            hoverBoundary = boundary;
            InvalidateVisual();
        }
    }

    private int HitKeyframe(TransitionGraphViewport viewport, Point point)
    {
        double closest = 7;
        int hit = -1;
        for (int index = 0; index < keys.Count; index++)
        {
            TimelineKeyframe key = keys[index];
            if (key.Coordinate < viewport.Start || key.Coordinate > viewport.End)
            {
                continue;
            }

            Vector offset = viewport.Project(new(key.Coordinate, key.Value)) - point;
            if (offset.Length < closest)
            {
                closest = offset.Length;
                hit = index;
            }
        }
        return hit;
    }
}
