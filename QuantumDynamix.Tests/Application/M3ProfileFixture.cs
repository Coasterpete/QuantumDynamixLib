using System;
using System.Collections.Generic;
using System.Linq;
using QuantumDynamix.Math.Functions;
using QuantumDynamix.Math.Timelines;
using QuantumDynamix.Splines.Curves;

namespace QuantumDynamix.Tests.Application;

internal static class M3ProfileFixture
{
    internal const string EditableChannel = "Blue";
    private static readonly double[] Boundaries = [0, 25, 55, 80, 100];
    private static readonly TimelineKeyframe BlueStart = new(0, .12);
    private static readonly TimelineKeyframe BlueEnd = new(100, .92);
    internal static TimelineKeyframe InitialKeyframe { get; } = new(40,
        FunctionSegment.Transition(0, 100, BlueStart.Value, BlueEnd.Value).Evaluate(40).Value);

    private static readonly (string Channel, IScalarFunction Profile)[] FixedProfiles =
    [
        ("Red", SmoothTransition(.15, .95, 3)),
        ("Green", SmoothTransition(.09, .89, 5)),
        ("Purple", SmoothTransition(.18, .98, 7)),
        ("Gold", FunctionSegment.Transition(0, 100, .06, .86,
            new EasingFunction(EasingKind.Cosine, EasingDirection.InOut)))
    ];

    internal static IReadOnlyList<Region> CreateRegions() => CreateRegions(Boundaries, InitialKeyframe);

    internal static IReadOnlyList<Region> CreateRegions(IReadOnlyList<double> boundaries, TimelineKeyframe keyframe)
    {
        // Keep the original straight profile until the editable keyframe's value changes.
        ScalarTimeline blue = ScalarTimeline.FromKeyframes(ProfileDomain.Distance, [BlueStart, keyframe, BlueEnd]);
        IEnumerable<(string Channel, IScalarFunction Profile)> profiles =
            FixedProfiles.Prepend((EditableChannel, blue));

        // Every region evaluates the same full profiles at its own stations.
        return Array.AsReadOnly(boundaries.Zip(boundaries.Skip(1), (start, end) =>
            new Region(ProfileDomain.Distance, start, end, profiles.Select(pair =>
            {
                var interval = new CustomFunction(start, end, pair.Profile.Evaluate,
                    pair.Profile.Breakpoints.Where(station => station > start && station < end));
                return KeyValuePair.Create(pair.Channel, new ScalarTimeline(ProfileDomain.Distance, [interval]));
            }))).ToArray());
    }

    private static FunctionSegment SmoothTransition(double from, double to, int degree)
    {
        // One clamped span with repeated endpoint coefficients gives the smoothstep polynomials.
        int repeats = (degree + 1) / 2;
        var easing = ScalarSpline.Clamped(Enumerable.Repeat(0.0, repeats).Concat(Enumerable.Repeat(1.0, repeats)), degree);
        return FunctionSegment.Transition(0, 100, from, to, easing);
    }
}
