using System;
using System.Collections.Generic;
using QuantumDynamix.Geometry;
using QuantumDynamix.Math;
using QuantumDynamix.Math.Functions;
using QuantumDynamix.Math.Timelines;
using QuantumDynamix.Splines.Curves;
using QuantumDynamix.Track;

namespace QuantumDynamix.App;

/// <summary>The quarter-circle rail example in README.md, with a separate editable banking timeline.</summary>
internal static class ReferenceTrack
{
    internal static TrackDocument Create()
    {
        var heartline = SplineCurve3d.Clamped([new(10, 0, 0), new(10, 10, 0), new(0, 10, 0)],
            degree: 2, weights: [1, System.Math.Sqrt(.5), 1]);
        double length = new ArcLengthMap(heartline).Length;
        TrackOffsetProfile spineOffsets = TrackOffsetProfile.Constant(new(0, 1, 0), length);
        var section = new RailCrossSection([SplineCurve3d.Clamped([new(0, -.1, 0), new(0, .1, 0)], degree: 1)]);
        var zeroRoll = new CustomFunction(0, length, (_, _) => new(0));
        var arrangement = new RailArrangement([
            new("inner", TrackOffsetProfile.Constant(new(0, .5, 0), length), section, zeroRoll),
            new("outer", TrackOffsetProfile.Constant(new(0, -.5, 0), length), section, zeroRoll)
        ]);
        var settings = new RailValidationSettings(positionTolerance: 1e-8, angularTolerance: 1e-7, curvatureTolerance: 1e-7,
            stationDerivativeTolerance: 1e-7, minimumSpeedRatio: 1e-10, minimumFrameProjection: 1e-8);

        // Keep the README's zero bank initially. The middle key provides an editable starting point.
        TimelineKeyframe[] keys = [new(0, 0), new(length / 2, 0), new(length, 0)];
        var bank = ScalarTimeline.FromKeyframes(ProfileDomain.Distance, keys);
        var region = new AuthoredRegion(Guid.NewGuid(), "Quarter circle", new(ProfileDomain.Distance, 0, length,
            [KeyValuePair.Create(TrackDocument.BankingChannel, bank)]),
            new Dictionary<string, TimelineKeyframe[]> { [TrackDocument.BankingChannel] = keys });
        HeartlineTrack WithBanking(ScalarTimeline profile) => new(heartline, Vector3d.UnitZ, spineOffsets, profile);

        HeartlineTrack track = WithBanking(bank);
        var binding = new TrackBankingBinding(region.Id, TrackDocument.BankingChannel, WithBanking);
        var source = new TrackWireframeSource(track, new(track, arrangement, settings), binding);
        return new("Quarter-circle Reference Track", source, new(region));
    }
}
