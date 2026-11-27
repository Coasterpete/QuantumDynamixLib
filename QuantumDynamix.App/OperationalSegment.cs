using System;
using System.Collections.Generic;
using System.Linq;
using QuantumDynamix.Math.Functions;
using QuantumDynamix.Math.Timelines;
using QuantumDynamix.Operations;

namespace QuantumDynamix.App;

internal enum OperationalSegmentType { NormalTrack, Transport }

internal sealed record OperationalSegment(Guid Id, double Start, double End, OperationalSegmentType Type,
    PoweredTransportDevice? Device)
{
    internal double Length => End - Start;
    internal string Label => Type == OperationalSegmentType.NormalTrack ? "Normal Track" : "Transport";
    public override string ToString() => $"{Label} · {Start:G6}–{End:G6} m";
}

internal sealed record PoweredTransportDevice(double EntrySpeed, double? SpeedLimit,
    IReadOnlyList<DeviceAccelerationRegion> Regions)
{
    internal PoweredTransportMotion Calculate(double length) =>
        PoweredTransportMotion.Calculate(length, EntrySpeed, SpeedLimit, Regions.Select(region => region.Timeline));
}

// Coordinates are offsets from the operational segment start, in heartline metres.
internal sealed class DeviceAccelerationRegion
{
    internal const string Channel = "Commanded acceleration (m/s²)";

    private DeviceAccelerationRegion(TimelineKeyframe[] keys, TransitionShape[] shapes, ScalarTimeline timeline)
    {
        Keys = Array.AsReadOnly((TimelineKeyframe[])keys.Clone());
        Shapes = Array.AsReadOnly((TransitionShape[])shapes.Clone());
        Timeline = timeline;
    }

    internal IReadOnlyList<TimelineKeyframe> Keys { get; }
    internal IReadOnlyList<TransitionShape> Shapes { get; }
    internal ScalarTimeline Timeline { get; }
    internal double Start => Timeline.Minimum;
    internal double End => Timeline.Maximum;

    internal Region GraphRegion => new(ProfileDomain.Distance, Start, End,
        [new(Channel, Timeline)]);

    internal static DeviceAccelerationRegion Create(TimelineKeyframe[] keys, TransitionShape[] shapes)
    {
        if (shapes.Any(shape => shape.Kind == EasingKind.Custom))
        { throw new ArgumentException("Choose a built-in transition function."); }
        if (TimelineEditorSession.BuildTimeline(ProfileDomain.Distance, keys, out ScalarTimeline? timeline, shapes) is { } error)
        { throw new ArgumentException(error); }
        if (timeline!.Minimum < 0 || !double.IsFinite(keys.Max(key => key.Value) - keys.Min(key => key.Value)))
        { throw new ArgumentException("Use nonnegative offsets and a finite acceleration range."); }
        return new(keys, shapes, timeline);
    }

    internal static DeviceAccelerationRegion Zero(double start, double end) =>
        Create([new(start, 0), new(end, 0)], [TransitionShape.Linear]);

    internal DeviceAccelerationRegion Resize(double start, double end)
    {
        var keys = Keys.ToArray();
        for (int i = 1; i < keys.Length - 1; i++)
        {
            double fraction = (keys[i].Coordinate - Start) / (End - Start);
            keys[i] = keys[i] with { Coordinate = start + (end - start) * fraction };
        }
        keys[0] = keys[0] with { Coordinate = start };
        keys[^1] = keys[^1] with { Coordinate = end };
        return Create(keys, Shapes.ToArray());
    }

    internal DeviceAccelerationRegion Split(int index)
    {
        var keys = Keys.ToList();
        var shapes = Shapes.ToList();
        double middle = keys[index].Coordinate + (keys[index + 1].Coordinate - keys[index].Coordinate) / 2;
        keys.Insert(index + 1, new(middle, Timeline.Evaluate(middle).Value));
        shapes.Insert(index + 1, TransitionShape.Linear);
        return Create(keys.ToArray(), shapes.ToArray());
    }

    internal DeviceAccelerationRegion Merge(int index)
    {
        if (Shapes.Count == 1) { throw new ArgumentException("An Acceleration Region needs at least one Transition."); }
        var keys = Keys.ToList();
        var shapes = Shapes.ToList();
        int join = System.Math.Min(index + 1, keys.Count - 2);
        keys.RemoveAt(join);
        shapes.RemoveAt(join);
        shapes[join - 1] = TransitionShape.Linear;
        return Create(keys.ToArray(), shapes.ToArray());
    }

    internal DeviceAccelerationRegion Edit(int index, double length, double from, double to, TransitionShape shape)
    {
        var keys = Keys.ToArray();
        var shapes = Shapes.ToArray();
        double start = keys[index].Coordinate;
        double end = keys[index + 1].Coordinate;
        if (!double.IsFinite(length) || length <= 0) { throw new ArgumentException("Enter a positive Transition length."); }
        if (length != end - start)
        {
            if (index + 1 < keys.Length - 1) { end = start + length; }
            else if (index > 0) { start = end - length; }
            else { throw new ArgumentException("Edit the Acceleration Region offsets to resize its only Transition."); }
        }
        keys[index] = new(start, from);
        keys[index + 1] = new(end, to);
        shapes[index] = shape;
        return Create(keys, shapes);
    }

    /// <summary>Moves an interior keyframe, or changes only the value of the first or last keyframe.</summary>
    /// <remarks>The start and end offsets change through <see cref="Resize"/>, so an endpoint keeps its offset.</remarks>
    internal DeviceAccelerationRegion Move(TransitionEdit.Keyframe edit)
    {
        var keys = Keys.ToArray();
        int index = Array.IndexOf(keys, edit.Original);
        if (index < 0)
        { throw new ArgumentException("Select a keyframe of this Acceleration Region."); }
        if ((index == 0 || index == keys.Length - 1) && edit.Proposed.Coordinate != edit.Original.Coordinate)
        { throw new ArgumentException("Drag the Acceleration Region boundary to change its start or end offset."); }
        if (index > 0 && index < keys.Length - 1 &&
            !(edit.Proposed.Coordinate > keys[index - 1].Coordinate && edit.Proposed.Coordinate < keys[index + 1].Coordinate))
        { throw new ArgumentException("Keep the offset strictly between its neighboring keyframes."); }
        keys[index] = edit.Proposed;
        return Create(keys, Shapes.ToArray());
    }
}
