using System;
using System.Collections.Generic;
using System.Linq;
using QuantumDynamix.Math.Functions;
using QuantumDynamix.Math.Timelines;
using QuantumDynamix.Track;

namespace QuantumDynamix.Operations;

internal readonly record struct PoweredTransportSample(double Offset, double ElapsedSeconds, double Speed,
    double CommandedAcceleration, double CalculatedAcceleration);

/// <summary>Forward-only prescribed motion over a device interval. All stations are segment-relative metres.</summary>
internal sealed class PoweredTransportMotion
{
    private const string Channel = "acceleration";
    private static readonly DistanceTrajectorySettings DistanceSettings = new(1e-7, 1e-8, 20, 200_000);
    private static readonly TimeTrajectorySettings TimeSettings = new(1e-7, 1e-8, 20, 200_000);
    private readonly Piece[] pieces;
    private readonly ScalarTimeline[] commands;
    private readonly double length;

    private PoweredTransportMotion(Piece[] pieces, ScalarTimeline[] commands, double length, double end, bool stopped)
    {
        this.pieces = pieces;
        this.commands = commands;
        this.length = length;
        End = end;
        Stopped = stopped;
    }

    internal double End { get; }
    internal bool Stopped { get; }

    internal PoweredTransportSample Sample(double offset)
    {
        if (!double.IsFinite(offset) || offset < 0 || offset > End)
        { throw new ArgumentOutOfRangeException(nameof(offset), $"Motion is available from 0 to {End:G8} m along the segment."); }
        double command = CommandAt(offset);
        if (pieces.Length == 0) { return new(offset, 0, 0, command, 0); }
        Piece piece = pieces.FirstOrDefault(piece => offset < piece.End) ?? pieces[^1];
        var motion = piece.Sample(offset);
        return Stopped && offset == End
            ? new(offset, motion.Elapsed, 0, command, 0)
            : new(offset, motion.Elapsed, motion.Speed, command, motion.Acceleration);
    }

    private double CommandAt(double offset)
    {
        foreach (ScalarTimeline command in commands)
        {
            if (offset >= command.Minimum && (offset < command.Maximum || offset == length && offset == command.Maximum))
            { return command.Evaluate(offset).Value; }
        }
        return 0;
    }

    internal static PoweredTransportMotion Calculate(double length, double initialSpeed, double? limit,
        IEnumerable<ScalarTimeline> regions)
    {
        if (!double.IsFinite(length) || length <= 0 || !double.IsFinite(initialSpeed) || initialSpeed < 0 ||
            limit is double cap && (!double.IsFinite(cap) || cap <= 0 || initialSpeed > cap))
        { throw new ArgumentException("Use a positive interval and a nonnegative entry speed at or below the positive speed limit."); }

        ScalarTimeline[] commands = regions.ToArray();
        var spans = new List<FunctionSegment>();
        double previous = 0;
        foreach (ScalarTimeline command in commands)
        {
            if (command.Domain != ProfileDomain.Distance || command.Minimum < previous || command.Maximum > length)
            { throw new ArgumentException("Keep Acceleration Regions ordered, disjoint, and inside the segment."); }
            if (command.Minimum > previous) { spans.Add(Constant(previous, command.Minimum, 0)); }
            foreach (IScalarFunction span in command.Segments)
            {
                if (span is not FunctionSegment mapped || mapped.Scale != 0 &&
                    mapped.Function is not EasingFunction { Kind: not EasingKind.Custom })
                { throw new ArgumentException("Powered transport requires built-in Transition easing functions."); }
                spans.Add(mapped);
            }
            previous = command.Maximum;
        }
        if (previous < length) { spans.Add(Constant(previous, length, 0)); }

        var pieces = new List<Piece>();
        double speed = initialSpeed;
        double elapsed = 0;
        double end = 0;
        bool stopped = false;
        foreach (FunctionSegment span in spans)
        {
            double start = span.Minimum;
            double finish = span.Maximum;
            if (span.Scale == 0)
            {
                AddConstant(start, finish, span.Offset);
            }
            else
            {
                double first = span.Evaluate(start).Value;
                double last = span.Evaluate(finish).Value;
                if (first < 0 && last > 0 || first > 0 && last < 0)
                {
                    // Built-in easings are monotone. Splitting at the sign change makes speed monotone on each piece.
                    double split = Crossing(start, finish, station => first < 0
                        ? span.Evaluate(station).Value >= 0 : span.Evaluate(station).Value <= 0);
                    AddVariable(span, start, split, first > 0);
                    if (split < finish) { AddVariable(span, split, finish, last > 0); }
                }
                else { AddVariable(span, start, finish, first > 0 || last > 0); }
            }
            if (stopped) { break; }
        }
        return new(pieces.ToArray(), commands, length, end, stopped);

        void AddConstant(double start, double finish, double acceleration)
        {
            if (acceleration <= 0 && speed == 0)
            {
                stopped = true;
                return;
            }
            if (acceleration > 0 && limit is double ceiling)
            {
                double reach = start + (ceiling - speed) / acceleration * (ceiling / 2 + speed / 2);
                if (reach <= finish)
                {
                    if (reach > start) { AddConstantPiece(start, reach, acceleration, ceiling); }
                    else if (speed != ceiling)
                    { throw new InvalidOperationException("The speed-limit crossing cannot be resolved at this offset."); }
                    if (reach < finish) { AddConstantPiece(reach, finish, 0, ceiling); }
                    return;
                }
            }
            if (acceleration < 0)
            {
                double stop = start + speed / -acceleration * (speed / 2);
                if (stop <= finish)
                {
                    if (!(stop > start)) { throw new InvalidOperationException("The stopping offset cannot be resolved."); }
                    AddConstantPiece(start, stop, acceleration, 0);
                    stopped = true;
                    return;
                }
            }
            double final = acceleration == 0 ? speed : SpeedAfter(speed, acceleration, finish - start);
            AddConstantPiece(start, finish, acceleration, final);
        }

        void AddConstantPiece(double start, double finish, double acceleration, double final)
        {
            double entry = speed;
            double origin = elapsed;
            double duration = (finish - start) / (entry / 2 + final / 2);
            var trajectory = Profile(Constant(0, duration, acceleration), ProfileDomain.Time)
                .IntegrateTime(new(start, entry), TimeSettings);
            AddPiece(finish, station =>
            {
                if (station == finish) { return (origin + duration, final, acceleration); }
                if (station == start) { return (origin, entry, acceleration); }
                double distance = station - start;
                double velocity = final == 0 ? SpeedAfter(0, -acceleration, finish - station)
                    : acceleration == 0 ? entry : SpeedAfter(entry, acceleration, distance);
                double time = distance / (entry / 2 + velocity / 2);
                var sample = trajectory.Sample(System.Math.Min(time, duration));
                return (origin + sample.ElapsedSeconds, System.Math.Clamp(sample.HeartlineVelocityMetresPerSecond,
                    System.Math.Min(entry, final), System.Math.Max(entry, final)), sample.LongitudinalAccelerationMetresPerSecondSquared);
            });
        }

        void AddVariable(FunctionSegment span, double start, double finish, bool accelerating)
        {
            if (speed == 0)
            {
                throw new InvalidOperationException($"A nonlinear Transition at {start:G8} m offset requires positive entry speed. Start with a constant positive command to depart from rest.");
            }
            if (accelerating && limit == speed)
            {
                AddConstantPiece(start, finish, 0, speed);
                return;
            }

            IScalarFunction function = span;
            AccelerationRangeProvider? ranges = null;
            if (start != span.Minimum || finish != span.Maximum)
            {
                function = new CustomFunction(start, finish, span.Evaluate,
                    span.Breakpoints.Where(point => point > start && point < finish));
                ranges = (_, low, high) => AccelerationEnclosures.ForBuiltIn(span, low, high);
            }

            LongitudinalTrajectory trajectory;
            try { trajectory = Profile(function, ProfileDomain.Distance).IntegrateDistance(speed, DistanceSettings, ranges); }
            catch (TrajectoryException error)
            {
                throw new InvalidOperationException($"{error.Code} over offsets {error.StartCoordinate:G8}–{error.EndCoordinate:G8} m. {error.Message}", error);
            }

            double origin = elapsed;
            double limitStation = finish;
            bool limited = accelerating && limit is double maximum && trajectory.Sample(finish).HeartlineVelocityMetresPerSecond > maximum;
            if (limited)
            {
                limitStation = Crossing(start, finish, station => trajectory.Sample(station).HeartlineVelocityMetresPerSecond >= limit!.Value);
            }
            AddPiece(limitStation, station =>
            {
                var state = trajectory.Sample(station);
                return (origin + state.ElapsedSeconds,
                    limited ? System.Math.Min(state.HeartlineVelocityMetresPerSecond, limit!.Value) : state.HeartlineVelocityMetresPerSecond,
                    state.LongitudinalAccelerationMetresPerSecondSquared);
            });
            if (limitStation < finish) { AddConstantPiece(limitStation, finish, 0, limit!.Value); }
        }

        void AddPiece(double finish, Func<double, (double Elapsed, double Speed, double Acceleration)> sample)
        {
            (elapsed, speed, _) = sample(finish);
            if (!double.IsFinite(elapsed) || !double.IsFinite(speed))
            { throw new InvalidOperationException("The motion exceeds numerical range."); }
            pieces.Add(new(finish, sample));
            end = finish;
        }
    }

    private static double SpeedAfter(double speed, double acceleration, double distance) =>
        System.Math.Sqrt(System.Math.FusedMultiplyAdd(2 * acceleration, distance, speed * speed));

    private static double Crossing(double low, double high, Func<double, bool> reached)
    {
        for (int i = 0; i < 64; i++)
        {
            double middle = low + (high - low) / 2;
            if (middle == low || middle == high) { break; }
            if (reached(middle)) { high = middle; }
            else { low = middle; }
        }
        return high;
    }

    private static FunctionSegment Constant(double start, double end, double acceleration) =>
        FunctionSegment.Transition(start, end, acceleration, acceleration);

    private static LongitudinalAccelerationProfile Profile(IScalarFunction function, ProfileDomain domain) => new(
        [new(new(domain, function.Minimum, function.Maximum, [new(Channel, new(domain, [function]))]), Channel)],
        -1, [0, 0, 0, 0]);

    private sealed record Piece(double End, Func<double, (double Elapsed, double Speed, double Acceleration)> Sample);
}
