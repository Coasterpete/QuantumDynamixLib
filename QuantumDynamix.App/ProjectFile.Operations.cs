using System;
using System.IO;
using System.Linq;
using System.Text.Json;
using System.Text.Json.Serialization;
using QuantumDynamix.Math.Timelines;

namespace QuantumDynamix.App;

internal static partial class ProjectFile
{
    private sealed record SegmentData(Guid Id, double StartMetres, double EndMetres, string Type,
        [property: JsonIgnore(Condition = JsonIgnoreCondition.WhenWritingNull)] DeviceData? Device = null);

    private sealed record DeviceData(string Type, double EntrySpeedMetresPerSecond,
        AccelerationRegionData[] AccelerationRegions,
        [property: JsonIgnore(Condition = JsonIgnoreCondition.WhenWritingNull)] double? SpeedLimitMetresPerSecond = null);

    private sealed record AccelerationRegionData(AccelerationKeyData[] Keyframes, TransitionData[] Transitions);
    private sealed record AccelerationKeyData(double OffsetMetres, double AccelerationMetresPerSecondSquared);

    private static SegmentData WriteSegment(OperationalSegment segment) => new(segment.Id, segment.Start, segment.End,
        JsonNamingPolicy.CamelCase.ConvertName(segment.Type.ToString()),
        segment.Device is { } device ? new("idealizedPoweredTransport", device.EntrySpeed,
            device.Regions.Select(region => new AccelerationRegionData(
                region.Keys.Select(key => new AccelerationKeyData(key.Coordinate, key.Value)).ToArray(),
                region.Shapes.Select(WriteTransition).ToArray())).ToArray(), device.SpeedLimit) : null);

    private static OperationalSegment ReadSegment(SegmentData? source, int index)
    {
        string label = $"Operational segment {index + 1}";
        if (source is null || !TryReadEnum(source.Type, out OperationalSegmentType type))
        { throw new InvalidDataException($"{label} has a missing or unsupported type."); }
        PoweredTransportDevice? device = null;
        if (source.Device is { } data)
        {
            if (data.Type != "idealizedPoweredTransport")
            { throw new InvalidDataException($"{label} has an unsupported device type."); }
            var regions = data.AccelerationRegions.Select((region, number) =>
                ReadAccelerationRegion(region, $"{label}, Acceleration Region {number + 1}")).ToArray();
            device = new(data.EntrySpeedMetresPerSecond, data.SpeedLimitMetresPerSecond, regions);
        }
        return new(source.Id, source.StartMetres, source.EndMetres, type, device);
    }

    private static DeviceAccelerationRegion ReadAccelerationRegion(AccelerationRegionData? region, string label)
    {
        if (region is null) { throw new InvalidDataException($"{label} is missing."); }
        try
        {
            return DeviceAccelerationRegion.Create(region.Keyframes.Select(key => key is null
                ? throw new InvalidDataException($"{label} has a missing keyframe.")
                : new TimelineKeyframe(key.OffsetMetres, key.AccelerationMetresPerSecondSquared)).ToArray(),
                region.Transitions.Select(shape => ReadTransition(shape, label)).ToArray());
        }
        catch (ArgumentException error) { throw new InvalidDataException($"{label}. {error.Message}", error); }
    }
}
