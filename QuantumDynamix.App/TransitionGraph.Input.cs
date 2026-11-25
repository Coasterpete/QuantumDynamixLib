using Avalonia;
using Avalonia.Input;
using QuantumDynamix.Math.Functions;
using QuantumDynamix.Math.Timelines;

namespace QuantumDynamix.App;

internal sealed partial class TransitionGraph
{
    private IPointer? dragPointer;

    protected override void OnPointerPressed(PointerPressedEventArgs e)
    {
        base.OnPointerPressed(e);
        TransitionGraphViewport viewport = Viewport;
        if (dragPointer is not null || !viewport.HasPlot
            || e.GetCurrentPoint(this).Properties.PointerUpdateKind != PointerUpdateKind.LeftButtonPressed)
        {
            return;
        }

        Point point = e.GetPosition(this);
        GraphPosition position = viewport.Unproject(point);
        TransitionDragTarget? target = resolveDragTarget?.Invoke(position);
        if (resolveDragTarget is null && editChannel is { } channel && !hiddenChannels.Contains(channel)
            && viewport.Plot.Inflate(2).Contains(point))
        {
            int hit = HitKeyframe(viewport, point);
            if (hit >= 0)
            {
                KeyframeSelected?.Invoke(hit);
                e.Handled = true;
                if (hit > 0 && hit < keys.Count - 1)
                {
                    TimelineKeyframe original = keys[hit];
                    target = new(keys[hit - 1].Coordinate, keys[hit + 1].Coordinate, (origin, pointer) =>
                        new TransitionEdit.Keyframe(channel, original, new(
                            original.Coordinate + (pointer.Distance - origin.Distance),
                            original.Value + (pointer.Value - origin.Value))));
                }
            }
            else if (TransitionSelected is { } select && activeRegion >= 0 &&
                activeRegion < regions.Count && keys.Count > 1 &&
                position.Distance >= keys[0].Coordinate && position.Distance <= keys[^1].Coordinate)
            {
                var timeline = regions[activeRegion].Channels[channel];
                double value = timeline.Evaluate(position.Distance, EvaluationSide.Right).Value;
                if (System.Math.Abs(viewport.Project(new(position.Distance, value)).Y - point.Y) <= 8)
                {
                    int span = 0;
                    while (span + 2 < keys.Count && position.Distance >= keys[span + 1].Coordinate) { span++; }
                    Focus();
                    select(span);
                    e.Handled = true;
                }
            }
        }
        if (target is not null && edit.TryBegin(target, position))
        {
            Focus();
            dragPointer = e.Pointer;
            e.Pointer.Capture(this);
            e.Handled = true;
        }
        else if (resolveDragTarget is null && point.Y >= 0 && point.Y < viewport.Plot.Top - 5 &&
            point.X >= viewport.Plot.Left && point.X <= viewport.Plot.Right)
        {
            for (int i = 0; i < regions.Count; i++)
            {
                if (position.Distance >= regions[i].Start && position.Distance <= regions[i].End)
                {
                    Focus();
                    RegionSelected?.Invoke(i);
                    e.Handled = true;
                    break;
                }
            }
        }
    }

    protected override void OnPointerMoved(PointerEventArgs e)
    {
        base.OnPointerMoved(e);
        if (dragPointer == e.Pointer && Viewport is { HasPlot: true } viewport)
        {
            edit.Update(viewport.Unproject(e.GetPosition(this)));
            PreviewChanged?.Invoke(edit.Candidate);
            InvalidateVisual();
            e.Handled = true;
        }
    }

    protected override void OnPointerReleased(PointerReleasedEventArgs e)
    {
        base.OnPointerReleased(e);
        if (dragPointer != e.Pointer || e.InitialPressMouseButton != MouseButton.Left)
        {
            return;
        }

        TransitionEdit? proposal = Viewport is { HasPlot: true } viewport
            ? edit.Finish(viewport.Unproject(e.GetPosition(this))) : null;
        EndDrag();
        e.Pointer.Capture(null);
        e.Handled = true;
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

    protected override void OnKeyDown(KeyEventArgs e)
    {
        base.OnKeyDown(e);
        if (e.Key == Key.Escape && dragPointer is not null)
        {
            CancelDrag();
            e.Handled = true;
        }
    }

    private void CancelDrag()
    {
        if (dragPointer is { } pointer)
        {
            EndDrag();
            pointer.Capture(null);
        }
    }

    private void EndDrag()
    {
        dragPointer = null;
        edit.Cancel();
        PreviewChanged?.Invoke(null);
        InvalidateVisual();
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
