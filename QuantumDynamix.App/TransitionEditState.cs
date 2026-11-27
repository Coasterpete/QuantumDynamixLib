using System;
using QuantumDynamix.Math.Timelines;

namespace QuantumDynamix.App;

internal readonly record struct GraphPosition(double Distance, double Value)
{
    internal bool IsFinite => double.IsFinite(Distance) && double.IsFinite(Value);
}

internal abstract record TransitionEdit
{
    internal sealed record Keyframe(string Channel, TimelineKeyframe Original, TimelineKeyframe Proposed) : TransitionEdit;

    /// <summary>A Region boundary moved along the distance axis.</summary>
    /// <param name="Left">Index of the Region that ends at the boundary, or -1.</param>
    /// <param name="Right">Index of the Region that starts at the boundary, or -1.</param>
    internal sealed record Boundary(double Original, double Proposed, int Left = -1, int Right = -1) : TransitionEdit;
}

// A future interaction policy supplies the target, allowed neighbors, and mapping from pointer movement to an edit.
internal sealed record TransitionDragTarget(
    double PreviousDistance, double NextDistance, Func<GraphPosition, GraphPosition, TransitionEdit?> Propose);

internal sealed class TransitionEditState
{
    private TransitionDragTarget? target;
    private GraphPosition origin;

    internal TransitionEdit? Candidate { get; private set; }

    internal bool TryBegin(TransitionDragTarget selected, GraphPosition position)
    {
        if (target is not null || !position.IsFinite || !double.IsFinite(selected.PreviousDistance)
            || !double.IsFinite(selected.NextDistance) || selected.PreviousDistance >= selected.NextDistance)
        {
            return false;
        }

        target = selected;
        origin = position;
        return true;
    }

    internal void Update(GraphPosition position)
    {
        Candidate = null;
        if (target is not { } selected || !position.IsFinite)
        {
            return;
        }

        TransitionEdit? edit = selected.Propose(origin, position);
        Candidate = edit switch
        {
            TransitionEdit.Keyframe keyframe when double.IsFinite(keyframe.Proposed.Value)
                && IsBetweenNeighbors(keyframe.Proposed.Coordinate, selected) => edit,
            TransitionEdit.Boundary boundary when IsBetweenNeighbors(boundary.Proposed, selected) => edit,
            _ => null
        };
    }

    internal TransitionEdit? Finish(GraphPosition position)
    {
        Update(position);
        TransitionEdit? completed = Candidate;
        Cancel();
        return completed;
    }

    internal void Cancel()
    {
        target = null;
        Candidate = null;
    }

    private static bool IsBetweenNeighbors(double distance, TransitionDragTarget target) =>
        distance > target.PreviousDistance && distance < target.NextDistance;
}
