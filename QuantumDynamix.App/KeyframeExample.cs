using System;
using System.Collections.Generic;
using System.Linq;
using QuantumDynamix.Math.Timelines;

namespace QuantumDynamix.App;

// Independent, dimensionless regression data for the existing keyframe editor.
internal static class KeyframeExample
{
    internal static TimelineEditorSession Create()
    {
        Dictionary<string, TimelineKeyframe[]>[] keys =
        [
            new()
            {
                ["Red"] = [new(0, .2), new(18, .38), new(50, .55)],
                ["Green"] = [new(0, .1), new(24, .31), new(50, .42)],
                ["Gold"] = [new(0, .76), new(28, .65), new(50, .58)]
            },
            new()
            {
                ["Red"] = [new(50, .55), new(77, .69), new(100, .9)],
                ["Green"] = [new(50, .42), new(72, .58), new(100, .7)],
                ["Gold"] = [new(50, .58), new(79, .46), new(100, .33)]
            }
        ];
        return new(keys.Select((channels, index) =>
        {
            TimelineKeyframe[] first = channels.Values.First();
            var region = new Region(ProfileDomain.Distance, first[0].Coordinate, first[^1].Coordinate,
                channels.Select(pair => KeyValuePair.Create(pair.Key,
                    ScalarTimeline.FromKeyframes(ProfileDomain.Distance, pair.Value))));
            return new AuthoredRegion(Guid.NewGuid(), $"Transition {index + 1}", region, channels);
        }).ToArray());
    }
}
