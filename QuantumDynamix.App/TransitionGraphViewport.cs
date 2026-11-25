using System;
using System.Collections.Generic;
using System.Linq;
using Avalonia;
using QuantumDynamix.Math.Functions;
using QuantumDynamix.Math.Timelines;

namespace QuantumDynamix.App;

internal readonly record struct TransitionGraphViewport(Rect Plot, double Start, double End,
    double MinimumValue = 0, double MaximumValue = 1)
{
    internal bool HasPlot => Plot.Width > 0 && Plot.Height > 0;

    internal double ProjectDistance(double distance) => Plot.Left + (distance - Start) / (End - Start) * Plot.Width;

    internal Point Project(GraphPosition position) =>
        new(ProjectDistance(position.Distance),
            Plot.Bottom - (position.Value - MinimumValue) / (MaximumValue - MinimumValue) * Plot.Height);

    internal GraphPosition Unproject(Point point) =>
        new(Start + (point.X - Plot.Left) / Plot.Width * (End - Start),
            MinimumValue + (Plot.Bottom - point.Y) / Plot.Height * (MaximumValue - MinimumValue));

    internal IEnumerable<Point[]> Sample(ScalarTimeline timeline)
    {
        double first = System.Math.Max(timeline.Minimum, Start);
        double start = first;
        double stop = System.Math.Min(timeline.Maximum, End);
        if (start >= stop)
        {
            yield break;
        }

        foreach (double end in timeline.Breakpoints.Where(station => station > first && station < stop).Append(stop))
        {
            // Include the exact endpoints and keep each chord at most one device-independent pixel wide.
            int steps = System.Math.Max(1, (int)System.Math.Ceiling(ProjectDistance(end) - ProjectDistance(start)));
            var points = new Point[steps + 1];
            points[0] = Project(new(start, timeline.Evaluate(start, EvaluationSide.Right).Value));
            for (int i = 1; i < steps; i++)
            {
                double distance = start + (end - start) * (i / (double)steps);
                points[i] = Project(new(distance, timeline.Evaluate(distance).Value));
            }

            points[^1] = Project(new(end, timeline.Evaluate(end, EvaluationSide.Left).Value));
            yield return points;
            start = end;
        }
    }
}
