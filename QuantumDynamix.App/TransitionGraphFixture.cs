using System;
using System.Collections.Generic;
using System.Linq;
using QuantumDynamix.Math.Functions;
using QuantumDynamix.Math.Timelines;
using QuantumDynamix.Splines.Curves;

namespace QuantumDynamix.App;

internal static class TransitionGraphFixture
{
    internal static IReadOnlyList<Region> CreateRegions()
    {
        double[] boundaries = [0, 25, 55, 80, 100];
        // All five values are dimensionless. Restrict each full profile to the four displayed regions.
        var profiles = new Dictionary<string, FunctionSegment>
        {
            ["Blue"] = FunctionSegment.Transition(0, 100, .12, .92),
            ["Red"] = SmoothTransition(.15, .95, 3),
            ["Green"] = SmoothTransition(.09, .89, 5),
            ["Purple"] = SmoothTransition(.18, .98, 7),
            ["Gold"] = FunctionSegment.Transition(0, 100, .06, .86,
                new EasingFunction(EasingKind.Cosine, EasingDirection.InOut))
        };

        return Array.AsReadOnly(boundaries.Zip(boundaries.Skip(1), (start, end) =>
            new Region(ProfileDomain.Distance, start, end, profiles.Select(profile =>
            {
                FunctionSegment segment = profile.Value;
                var interval = new CustomFunction(start, end, segment.Evaluate,
                    segment.Breakpoints.Where(station => station > start && station < end));
                return KeyValuePair.Create(profile.Key, new ScalarTimeline(ProfileDomain.Distance, [interval]));
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
