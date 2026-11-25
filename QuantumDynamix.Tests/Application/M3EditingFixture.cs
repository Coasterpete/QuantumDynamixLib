using System;
using System.Collections.Generic;
using Avalonia;
using Avalonia.Input;
using QuantumDynamix.App;
using QuantumDynamix.Math.Timelines;

namespace QuantumDynamix.Tests.Application;

// The earlier M3 worktree supplied this policy for its own synthetic profiles.
internal sealed class M3EditingFixture
{
    private const double MarkerHitRadius = 7;
    private readonly Stack<M3FixtureTimeline> undo = new();
    private readonly Stack<M3FixtureTimeline> redo = new();
    private M3FixtureTimeline? preview;

    internal M3EditingFixture()
    {
        Graph = new(Committed.Regions, position => HitTestMarker(Viewport, Viewport.Project(position)));
        Graph.PreviewChanged += edit =>
        {
            preview = Committed.WithEdit(edit);
            Graph.SetRegionPreview(preview?.Regions);
        };
        Graph.EditProposed += edit =>
        {
            if (Committed.WithEdit(edit) is { } accepted)
            {
                undo.Push(Committed);
                redo.Clear();
                SetCommitted(accepted);
            }
        };
        Graph.KeyDown += (_, e) =>
        {
            if (UndoGesture.Matches(e))
            {
                Undo();
            }
            else if (RedoGesture.Matches(e))
            {
                Redo();
            }
        };
        SetCommitted(Committed);
    }

    internal TransitionGraph Graph { get; }
    internal M3FixtureTimeline Committed { get; private set; } = M3FixtureTimeline.CreateInitial();
    internal M3FixtureTimeline Displayed => preview ?? Committed;
    internal TransitionGraphViewport Viewport => Graph.Viewport;
    internal bool CanUndo => undo.Count != 0;
    internal bool CanRedo => redo.Count != 0;
    internal static KeyGesture UndoGesture { get; } = new(Key.Z,
        OperatingSystem.IsMacOS() ? KeyModifiers.Meta : KeyModifiers.Control);
    internal static KeyGesture RedoGesture { get; } = OperatingSystem.IsMacOS()
        ? new(Key.Z, KeyModifiers.Meta | KeyModifiers.Shift) : new(Key.Y, KeyModifiers.Control);

    internal IEnumerable<Point[]> CurvePaths(string channel) => Graph.CurvePaths(channel);
    internal void Undo() => Restore(undo, redo);
    internal void Redo() => Restore(redo, undo);

    private void Restore(Stack<M3FixtureTimeline> from, Stack<M3FixtureTimeline> to)
    {
        if (from.TryPop(out M3FixtureTimeline? previous))
        {
            to.Push(Committed);
            SetCommitted(previous);
        }
    }

    private void SetCommitted(M3FixtureTimeline timeline)
    {
        Committed = timeline;
        Graph.SetRegions(timeline.Regions, 0, 1);
        Graph.SetSelection(-1, M3ProfileFixture.EditableChannel, [timeline.Keyframe], 0);
    }

    private static double DiamondY(Rect plot) => plot.Bottom + 8;

    private static Point ProjectKeyframe(TransitionGraphViewport viewport, TimelineKeyframe keyframe) =>
        viewport.Project(new(keyframe.Coordinate, keyframe.Value));

    private TransitionDragTarget? HitTestMarker(TransitionGraphViewport viewport, Point pointer)
    {
        IReadOnlyList<Region> regions = Committed.Regions;
        if (System.Math.Abs(pointer.Y - DiamondY(viewport.Plot)) <= MarkerHitRadius)
        {
            int nearest = 0;
            double nearestDistance = double.PositiveInfinity;
            for (int i = 1; i < regions.Count; i++)
            {
                double distance = System.Math.Abs(pointer.X - viewport.ProjectDistance(regions[i].Start));
                if (distance < nearestDistance)
                {
                    nearest = i;
                    nearestDistance = distance;
                }
            }
            if (nearestDistance <= MarkerHitRadius)
            {
                double station = regions[nearest].Start;
                return new(regions[nearest - 1].Start, regions[nearest].End, (origin, current) =>
                    new TransitionEdit.Boundary(station, station + (current.Distance - origin.Distance)));
            }
            return null;
        }

        TimelineKeyframe keyframe = Committed.Keyframe;
        Point delta = pointer - ProjectKeyframe(viewport, keyframe);
        if (delta.X * delta.X + delta.Y * delta.Y <= MarkerHitRadius * MarkerHitRadius)
        {
            return new(viewport.Start, viewport.End, (origin, current) => new TransitionEdit.Keyframe(
                M3ProfileFixture.EditableChannel, keyframe,
                new(keyframe.Coordinate, keyframe.Value + (current.Value - origin.Value))));
        }
        return null;
    }

}
