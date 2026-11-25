using System.Collections.Generic;
using System.Linq;
using QuantumDynamix.App;
using QuantumDynamix.Math.Timelines;

namespace QuantumDynamix.Tests.Application;

internal sealed class M3FixtureTimeline
{
    private static readonly double[] JoinTolerance = [1e-12];

    private M3FixtureTimeline(IReadOnlyList<Region> regions, TimelineKeyframe keyframe)
    {
        Regions = regions;
        Keyframe = keyframe;
    }

    internal IReadOnlyList<Region> Regions { get; }
    internal TimelineKeyframe Keyframe { get; }

    internal static M3FixtureTimeline CreateInitial() =>
        new(M3ProfileFixture.CreateRegions(), M3ProfileFixture.InitialKeyframe);

    internal M3FixtureTimeline? WithEdit(TransitionEdit? edit)
    {
        double[] boundaries = Regions.Select(region => region.Start).Append(Regions[^1].End).ToArray();
        TimelineKeyframe keyframe = Keyframe;
        switch (edit)
        {
            case TransitionEdit.Boundary move:
                int index = System.Array.IndexOf(boundaries, move.Original);
                if (index <= 0 || index >= boundaries.Length - 1 || move.Proposed == move.Original
                    || !(move.Proposed > boundaries[index - 1] && move.Proposed < boundaries[index + 1]))
                {
                    return null;
                }
                boundaries[index] = move.Proposed;
                break;
            case TransitionEdit.Keyframe change when change.Channel == M3ProfileFixture.EditableChannel
                && change.Original == keyframe && change.Proposed.Coordinate == keyframe.Coordinate
                && change.Proposed.Value != keyframe.Value && change.Proposed.Value is >= 0 and <= 1:
                keyframe = change.Proposed;
                break;
            default:
                return null;
        }

        IReadOnlyList<Region> regions = M3ProfileFixture.CreateRegions(boundaries, keyframe);
        for (int i = 1; i < regions.Count; i++)
        {
            Region left = regions[i - 1];
            Region right = regions[i];
            foreach (string channel in left.Channels.Keys)
            {
                if (left.DiagnoseJoin(right, channel, JoinTolerance).MatchingOrder < 0)
                {
                    return null;
                }
            }
        }
        return new(regions, keyframe);
    }
}
