using System;
using System.Collections.Generic;
using System.Linq;

namespace QuantumDynamix.App;

internal sealed partial class TimelineEditorSession
{
    internal IReadOnlyList<OperationalSegment> Segments => current.Segments;

    internal OperationalSegment? NewSegment()
    {
        if (Geometry is null) { return null; }
        double start = Regions[0].Definition.Start;
        foreach (OperationalSegment next in Segments)
        {
            if (next.Start > start) { return new(Guid.NewGuid(), start, next.Start, OperationalSegmentType.NormalTrack, null); }
            start = next.End;
        }
        double end = Regions[^1].Definition.End;
        return end > start ? new(Guid.NewGuid(), start, end, OperationalSegmentType.NormalTrack, null) : null;
    }

    internal string? SetSegment(OperationalSegment segment)
    {
        if (Segments.Contains(segment)) { return null; }
        return ReplaceSegments([.. Segments.Where(existing => existing.Id != segment.Id), segment]);
    }

    internal string? ValidateSegment(OperationalSegment segment) =>
        ValidateSegments(Segments.Where(existing => existing.Id != segment.Id).Append(segment).OrderBy(item => item.Start).ToArray());

    internal void RemoveSegment(Guid id)
    {
        if (Segments.Any(segment => segment.Id == id))
        { CommitSnapshot(current with { Segments = Array.AsReadOnly(Segments.Where(segment => segment.Id != id).ToArray()) }); }
    }

    private string? ReplaceSegments(OperationalSegment[] candidates)
    {
        OperationalSegment[] sorted = candidates.OrderBy(segment => segment.Start).ToArray();
        if (ValidateSegments(sorted) is { } error) { return error; }
        for (int i = 0; i < sorted.Length; i++)
        {
            if (sorted[i].Device is { } device)
            { sorted[i] = sorted[i] with { Device = device with { Regions = Array.AsReadOnly(device.Regions.ToArray()) } }; }
        }
        CommitSnapshot(current with { Segments = Array.AsReadOnly(sorted) });
        return null;
    }

    private string? ValidateSegments(IReadOnlyList<OperationalSegment> segments)
    {
        if (ValidateSegmentStations(segments, Regions) is { } invalid) { return invalid; }
        var ids = new HashSet<Guid>();
        double previous = 0;
        foreach (OperationalSegment segment in segments)
        {
            if (segment.Id == Guid.Empty || !ids.Add(segment.Id)) { return "Track segments need unique, nonempty IDs."; }
            if (segment.Start < previous) { return "Operational track segments may touch but cannot overlap."; }
            previous = segment.End;
            if (segment.Type == OperationalSegmentType.NormalTrack && segment.Device is null) { continue; }
            if (segment.Type != OperationalSegmentType.Transport || segment.Device is not { } device)
            { return "Transport requires an idealized powered transport device. Normal Track has no device."; }
            if (!double.IsFinite(device.EntrySpeed) || device.EntrySpeed < 0 ||
                device.SpeedLimit is double limit && (!double.IsFinite(limit) || limit <= 0 || device.EntrySpeed > limit))
            { return "Enter a nonnegative entry speed and an optional positive speed limit at or above entry speed."; }
            if (ValidateAccelerationRegions(device.Regions, segment.Length) is { } error) { return error; }
        }
        return null;
    }

    internal static string? ValidateAccelerationRegions(IReadOnlyList<DeviceAccelerationRegion> regions, double length)
    {
        double previous = 0;
        foreach (DeviceAccelerationRegion region in regions)
        {
            if (region.Start < previous || region.End > length)
            { return "Keep Acceleration Regions disjoint and inside the segment offsets."; }
            previous = region.End;
        }
        return null;
    }

    private static string? ValidateSegmentStations(IReadOnlyList<OperationalSegment> segments, IReadOnlyList<AuthoredRegion> regions)
    {
        foreach (OperationalSegment segment in segments)
        {
            if (!double.IsFinite(segment.Start) || !double.IsFinite(segment.End) || segment.End <= segment.Start)
            { return "Enter distinct, finite track segment stations in increasing order."; }
            if (regions.Count == 0 || segment.Start < regions[0].Definition.Start || segment.End > regions[^1].Definition.End)
            { return $"The {segment} segment must stay within the authored heartline. Edit its stations first."; }
        }
        return null;
    }
}
