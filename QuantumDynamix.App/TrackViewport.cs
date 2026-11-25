using System;
using System.Linq;
using Avalonia;
using Avalonia.Controls;
using Avalonia.Input;
using Avalonia.Media;
using Avalonia.Media.Immutable;
using QuantumDynamix.Math;

namespace QuantumDynamix.App;

/// <summary>Avalonia input and line drawing for the engineering viewport.</summary>
internal sealed class TrackViewport : Control
{
    private static readonly ImmutablePen HeartlinePen = new(Brushes.WhiteSmoke, 1.5);
    private static readonly ImmutablePen SpinePen = new(Brushes.SandyBrown, 1.5);
    private static readonly ImmutablePen RailPen = new(Brushes.LightSkyBlue, 1.5);
    private static readonly ImmutablePen SupportPen = new(Brushes.LightGreen, 1.5);
    private static readonly ImmutablePen ForwardPen = new(Brushes.IndianRed, 1.5);
    private static readonly ImmutablePen UpPen = new(Brushes.DodgerBlue, 1.5);
    private static readonly ImmutablePen SelectedPen = new(Brushes.Gold, 2.5);

    private readonly TrackCamera camera = new();
    private Guid? trackId;
    private Vector3d[] framePoints = [];
    private double axisLength;
    private bool keepFramed = true;
    private IPointer? pointer;
    private MouseButton pressButton;
    private bool pan;
    private Point pressPosition;
    private Point previousPosition;
    private bool moved;

    internal TrackView View { get; private set; } = TrackView.Perspective;
    internal TrackWireframe Scene { get; private set; } = TrackWireframe.Empty("Select a track with defined geometry.");
    internal WireframeSelection? Selection { get; private set; }
    internal Guid? ActiveRegionId { get; private set; }
    internal ProjectedWireframe Projected => new(Scene, camera.ProjectTo(Bounds.Width, Bounds.Height), axisLength);
    internal bool HasGeometry => framePoints.Length > 0;
    internal event Action? StateChanged;
    internal event Action<Guid>? RegionPicked;

    internal void SetActiveRegion(Guid? regionId)
    {
        if (ActiveRegionId == regionId)
        {
            return;
        }
        ActiveRegionId = regionId;
        if (Selection?.RegionId is Guid selectedRegion && selectedRegion != regionId)
        {
            Selection = null;
        }
        Redraw();
    }

    internal void SetScene(Guid? document, TrackWireframe scene)
    {
        bool changedTrack = trackId != document;
        trackId = document;
        bool needsFrame = changedTrack || !HasGeometry;
        Scene = scene;
        framePoints = scene.Paths.SelectMany(path => path.Vertices.Select(vertex => vertex.Position))
            .Concat(scene.ControlPoints.Select(point => point.Position)).ToArray();
        var centre = scene.Paths.Where(path => path.Kind == WireframePathKind.Heartline).ToArray();
        axisLength = centre.Length == 0 ? 0 : .035 *
            ((centre[^1].Vertices[^1].HeartlineStation ?? 0) - (centre[0].Vertices[0].HeartlineStation ?? 0));
        if (changedTrack || !HasGeometry)
        {
            Selection = null;
        }
        else if (Selection is { } selection)
        {
            Selection = RestoreSelection(selection);
        }
        if (needsFrame)
        {
            camera.SetView(View);
            FrameAll();
        }
        else if (keepFramed)
        {
            FrameAll();
        }
        else
        {
            Redraw();
        }
    }

    internal void SetView(TrackView view)
    {
        View = view;
        camera.SetView(view);
        FrameAll();
    }

    internal void FrameAll()
    {
        keepFramed = true;
        camera.Frame(framePoints, Bounds.Width, Bounds.Height);
        Redraw();
    }

    protected override void OnSizeChanged(SizeChangedEventArgs e)
    {
        base.OnSizeChanged(e);
        if (keepFramed)
        {
            camera.Frame(framePoints, e.NewSize.Width, e.NewSize.Height);
        }
        InvalidateVisual();
    }

    public override void Render(DrawingContext context)
    {
        base.Render(context);
        Rect area = new(Bounds.Size);
        context.FillRectangle(Brushes.Black, area);
        if (!HasGeometry)
        {
            return;
        }

        using (context.PushClip(area))
        {
            ProjectedWireframe frame = Projected;
            foreach (ProjectedLine line in frame.Lines.OrderByDescending(line => (line.Start.Depth + line.End.Depth) / 2))
            {
                context.DrawLine(IsHighlighted(line.Selection) ? SelectedPen : PenFor(line.Appearance),
                    Screen(line.Start), Screen(line.End));
            }
            foreach (ProjectedMarker marker in frame.Markers.OrderByDescending(marker => marker.Point.Depth))
            {
                IPen pen = IsHighlighted(marker.Selection) ? SelectedPen : HeartlinePen;
                Point center = Screen(marker.Point);
                if (marker.Selection.Element == WireframeElement.RegionBoundary)
                {
                    context.DrawEllipse(null, pen, center, 6, 6);
                }
                else
                {
                    context.DrawRectangle(Brushes.Black, pen, new Rect(center.X - 3, center.Y - 3, 6, 6));
                }
            }
        }
    }

    protected override void OnPointerPressed(PointerPressedEventArgs e)
    {
        base.OnPointerPressed(e);
        if (!HasGeometry || pointer is not null)
        {
            return;
        }
        pressButton = e.GetCurrentPoint(this).Properties.PointerUpdateKind switch
        {
            PointerUpdateKind.LeftButtonPressed => MouseButton.Left,
            PointerUpdateKind.RightButtonPressed => MouseButton.Right,
            PointerUpdateKind.MiddleButtonPressed => MouseButton.Middle,
            _ => MouseButton.None
        };
        pan = pressButton == MouseButton.Middle ||
            (pressButton == MouseButton.Right && e.KeyModifiers.HasFlag(KeyModifiers.Shift));
        if (pressButton == MouseButton.None || (pressButton == MouseButton.Right && !pan && View != TrackView.Perspective))
        {
            return;
        }

        Focus();
        pointer = e.Pointer;
        pointer.Capture(this);
        pressPosition = previousPosition = e.GetPosition(this);
        moved = false;
        e.Handled = true;
    }

    protected override void OnPointerMoved(PointerEventArgs e)
    {
        base.OnPointerMoved(e);
        if (pointer != e.Pointer)
        {
            return;
        }

        Point position = e.GetPosition(this);
        Vector delta = position - previousPosition;
        previousPosition = position;
        Vector fromPress = position - pressPosition;
        moved |= fromPress.Length > 4;
        if (new Rect(Bounds.Size).Contains(position) && pressButton != MouseButton.Left && delta.Length > 0)
        {
            if (pan)
            {
                camera.Pan(delta.X, delta.Y, Bounds.Height);
            }
            else
            {
                camera.Orbit(-delta.X * .008, delta.Y * .008);
            }
            keepFramed = false;
            InvalidateVisual();
        }
        e.Handled = true;
    }

    protected override void OnPointerReleased(PointerReleasedEventArgs e)
    {
        base.OnPointerReleased(e);
        if (pointer != e.Pointer || e.InitialPressMouseButton != pressButton)
        {
            return;
        }
        pointer = null;
        e.Pointer.Capture(null);
        Point position = e.GetPosition(this);
        Vector fromPress = position - pressPosition;
        if (pressButton == MouseButton.Left && !moved && fromPress.Length <= 4 &&
            new Rect(Bounds.Size).Contains(position))
        {
            Selection = Projected.Pick(position.X, position.Y, 7);
            if (Selection?.RegionId is Guid regionId)
            {
                RegionPicked?.Invoke(regionId);
            }
            Redraw();
        }
        e.Handled = true;
    }

    protected override void OnPointerCaptureLost(PointerCaptureLostEventArgs e)
    {
        base.OnPointerCaptureLost(e);
        if (pointer == e.Pointer) { pointer = null; }
    }

    protected override void OnPointerWheelChanged(PointerWheelEventArgs e)
    {
        base.OnPointerWheelChanged(e);
        if (HasGeometry && e.Delta.Y != 0 && new Rect(Bounds.Size).Contains(e.GetPosition(this)))
        {
            camera.Zoom(System.Math.Pow(.85, e.Delta.Y));
            keepFramed = false;
            InvalidateVisual();
            e.Handled = true;
        }
    }

    private WireframeSelection? RestoreSelection(WireframeSelection selection)
    {
        // Selection uses stable engineering identifiers. A renamed region receives its new label.
        if (selection.Element == WireframeElement.RegionBoundary)
        {
            foreach (WireframeBoundary boundary in Scene.Boundaries)
            {
                if (boundary.RegionId == selection.RegionId && (boundary.IsStart ? "start" : "end") == selection.Id)
                {
                    return WireframeSelection.ForBoundary(boundary);
                }
            }
            return null;
        }

        bool exists = selection.Element == WireframeElement.ControlPoint
            ? Scene.ControlPoints.Any(point => point.Index.ToString() == selection.Id)
            : Scene.Paths.Any(path => path.Id == selection.Id && path.RegionId == selection.RegionId &&
                ProjectedWireframe.ElementFor(path.Kind) == selection.Element);
        return exists ? selection : null;
    }

    private void Redraw()
    {
        InvalidateVisual();
        StateChanged?.Invoke();
    }

    private static Point Screen(ProjectedPoint point) => new(point.X, point.Y);

    private bool IsHighlighted(WireframeSelection item) => item == Selection ||
        (item.Element is WireframeElement.Heartline or WireframeElement.RegionBoundary &&
            item.RegionId is Guid regionId && regionId == ActiveRegionId);

    private static IPen PenFor(WireframeElement element) => element switch
    {
        WireframeElement.Spine => SpinePen,
        WireframeElement.Rail => RailPen,
        WireframeElement.Support => SupportPen,
        WireframeElement.Forward => ForwardPen,
        WireframeElement.Left => SupportPen,
        WireframeElement.Up => UpPen,
        _ => HeartlinePen
    };
}
