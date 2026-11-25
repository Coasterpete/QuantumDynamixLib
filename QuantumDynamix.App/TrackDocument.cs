using System;
using QuantumDynamix.Geometry;

namespace QuantumDynamix.App;

internal sealed record TrackIdentity(Guid Id, string Name);

internal sealed class TrackDocument(string name, TrackWireframeSource? geometry = null, TimelineEditorSession? timeline = null)
{
    internal const string BankingChannel = "Banking (rad)";

    internal TrackIdentity Identity { get; } = new(Guid.NewGuid(), name);
    internal TimelineEditorSession Timeline { get; } = timeline ?? new();

    internal static TrackDocument CreateAuthored(string name, RigidPose3d pose, double speed) =>
        new(name, timeline: new(pose, speed));

    internal TrackWireframe SampleWireframe(int stationIntervals)
    {
        if (Timeline.Geometry is { } authored)
        {
            return stationIntervals == RegionTrackGeometry.DisplayIntervals
                ? authored.DisplayScene : authored.Sample(Timeline.Regions, stationIntervals);
        }
        return geometry?.Sample(Timeline.Regions, stationIntervals)
            ?? TrackWireframe.Empty("Append a Region to generate the track geometry.");
    }
}
