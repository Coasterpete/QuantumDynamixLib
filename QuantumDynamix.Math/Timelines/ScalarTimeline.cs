using System;
using System.Collections.Generic;
using System.Linq;
using QuantumDynamix.Math.Functions;

namespace QuantumDynamix.Math.Timelines;

/// <summary>The independent variable, in seconds or metres. Output units belong to the bound property.</summary>
public enum ProfileDomain { Time, Distance }
public enum BoundaryPolicy { Reject, Hold, Zero, Repeat }
public readonly record struct TimelineKeyframe(double Coordinate, double Value);
public readonly record struct ContinuityDiagnostic(double Coordinate, int ContinuousDerivativeOrder);

/// <summary>Maps a normalized function into a physical interval and scales its value and derivatives.</summary>
public sealed class FunctionSegment : IScalarFunction
{
    private readonly Dictionary<double, double>? breakpointSources;

    public double Minimum { get; }
    public double Maximum { get; }
    public double Offset { get; }
    public double Scale { get; }
    public IScalarFunction Function { get; }
    public IReadOnlyList<double> Breakpoints { get; }

    public FunctionSegment(double minimum, double maximum, IScalarFunction function, double scale = 1, double offset = 0)
    {
        Check.Interval(minimum, maximum);
        ArgumentNullException.ThrowIfNull(function);
        if (function.Minimum != 0 || function.Maximum != 1) { throw new ArgumentException("Segment functions require domain [0,1].", nameof(function)); }
        Check.Finite(scale, nameof(scale));
        Check.Finite(offset, nameof(offset));
        Minimum = minimum;
        Maximum = maximum;
        Function = function;
        Offset = offset;
        Scale = scale;
        var points = function.Breakpoints.Select(x => (Mapped: minimum + (maximum - minimum) * x, Source: x)).ToArray();
        Breakpoints = Array.AsReadOnly(points.Select(point => point.Mapped).ToArray());
        breakpointSources = points.Length == 0 ? null : BreakpointMapping.UniqueSources(points);
    }

    public static FunctionSegment Transition(double minimum, double maximum, double from, double to, IScalarFunction? easing = null)
    {
        Check.Finite(from, nameof(from));
        Check.Finite(to, nameof(to));
        easing ??= new EasingFunction();
        if (!Check.Near(easing.Evaluate(0).Value, 0, 1e-12) || !Check.Near(easing.Evaluate(1).Value, 1, 1e-12))
        { throw new ArgumentException("Transition easing must run from zero to one.", nameof(easing)); }
        return new(minimum, maximum, easing, to - from, from);
    }

    public ScalarJet Evaluate(double argument, EvaluationSide side = EvaluationSide.Automatic)
    {
        Check.Argument(argument, Minimum, Maximum, side);
        if (Scale == 0) { return new(Offset); }
        double x = argument == Maximum ? 1 : (argument - Minimum) / (Maximum - Minimum);
        if (breakpointSources is not null && argument > Minimum && argument < Maximum
            && breakpointSources.TryGetValue(argument, out double source))
        {
            x = source;
        }
        return Function.Evaluate(x, side).Transform(Offset, Scale, 1 / (Maximum - Minimum));
    }
}

/// <summary>Immutable, contiguous piecewise scalar mathematics; no integration, clocks, or physical state.</summary>
public sealed class ScalarTimeline : IScalarFunction
{
    private readonly IScalarFunction[] segments;
    public ProfileDomain Domain { get; }
    public BoundaryPolicy Boundary { get; }
    public bool AllowsDiscontinuities { get; }
    public double ContinuityTolerance { get; }
    public double Minimum => segments[0].Minimum;
    public double Maximum => segments[^1].Maximum;
    public IReadOnlyList<IScalarFunction> Segments { get; }
    public IReadOnlyList<double> Breakpoints { get; }

    public ScalarTimeline(ProfileDomain domain, IEnumerable<IScalarFunction> segments, BoundaryPolicy boundary = BoundaryPolicy.Reject,
        bool allowDiscontinuities = false, double continuityTolerance = 1e-10)
    {
        ArgumentNullException.ThrowIfNull(segments);
        if (!Enum.IsDefined(domain) || !Enum.IsDefined(boundary)) { throw new ArgumentOutOfRangeException(nameof(domain)); }
        if (!double.IsFinite(continuityTolerance) || continuityTolerance < 0) { throw new ArgumentOutOfRangeException(nameof(continuityTolerance)); }
        this.segments = segments.ToArray();
        if (this.segments.Length == 0 || this.segments.Any(s => s is null)) { throw new ArgumentException("Supply at least one non-null segment.", nameof(segments)); }
        for (int i = 0; i < this.segments.Length; i++)
        {
            Check.Interval(this.segments[i].Minimum, this.segments[i].Maximum);
            if (i > 0 && this.segments[i - 1].Maximum != this.segments[i].Minimum)
            { throw new ArgumentException("Segments must be ordered and exactly contiguous; overlaps and gaps are not implicit.", nameof(segments)); }
        }
        Check.Interval(Minimum, Maximum);
        Domain = domain;
        Boundary = boundary;
        AllowsDiscontinuities = allowDiscontinuities;
        ContinuityTolerance = continuityTolerance;
        Segments = Array.AsReadOnly(this.segments);
        Breakpoints = Array.AsReadOnly(this.segments.SelectMany(s => s.Breakpoints.Append(s.Minimum))
            .Where(x => x > Minimum && x < Maximum).Distinct().Order().ToArray());
        if (!allowDiscontinuities && ValidateContinuity(0).Count != 0)
        { throw new ArgumentException("A value discontinuity requires explicit permission.", nameof(allowDiscontinuities)); }
    }

    public static ScalarTimeline FromKeyframes(ProfileDomain domain, IEnumerable<TimelineKeyframe> keyframes,
        BoundaryPolicy boundary = BoundaryPolicy.Reject, IScalarFunction? easing = null)
    {
        ArgumentNullException.ThrowIfNull(keyframes);
        TimelineKeyframe[] keys = keyframes.ToArray();
        if (keys.Length < 2) { throw new ArgumentException("Supply at least two keyframes.", nameof(keyframes)); }
        return new(domain, keys.Zip(keys.Skip(1), (a, b) => FunctionSegment.Transition(a.Coordinate, b.Coordinate, a.Value, b.Value, easing)), boundary);
    }

    public ScalarJet Evaluate(double argument, EvaluationSide side = EvaluationSide.Automatic)
    {
        Check.Finite(argument, nameof(argument));
        if (!Enum.IsDefined(side)) { throw new ArgumentOutOfRangeException(nameof(side)); }
        if (Boundary == BoundaryPolicy.Repeat)
        {
            double shifted = argument - Minimum;
            if (!double.IsFinite(shifted)) { throw new ArgumentOutOfRangeException(nameof(argument)); }
            double length = Maximum - Minimum;
            double remainder = shifted % length;
            argument = Minimum + (remainder < 0 ? remainder + length : remainder);
        }
        if (argument < Minimum || argument > Maximum)
        {
            return Boundary switch
            {
                BoundaryPolicy.Hold => new(argument < Minimum ? segments[0].Evaluate(Minimum, EvaluationSide.Right).Value : segments[^1].Evaluate(Maximum, EvaluationSide.Left).Value),
                BoundaryPolicy.Zero => new(0),
                _ => throw new ArgumentOutOfRangeException(nameof(argument))
            };
        }
        if (argument == Minimum || argument == Maximum)
        {
            bool atStart = argument == Minimum;
            ScalarJet inside = atStart ? segments[0].Evaluate(Minimum, EvaluationSide.Right) : segments[^1].Evaluate(Maximum, EvaluationSide.Left);
            if (Boundary == BoundaryPolicy.Reject) { return inside; }
            ScalarJet outside = Boundary switch
            {
                BoundaryPolicy.Hold => new(inside.Value),
                BoundaryPolicy.Zero => new(0),
                _ => segments[^1].Evaluate(Maximum, EvaluationSide.Left)
            };
            ScalarJet left = atStart ? outside : inside;
            ScalarJet right = atStart ? inside : outside;
            return Choose(left, right, side);
        }
        int index = 0;
        for (; index < segments.Length; index++)
        {
            if (argument < segments[index].Maximum) { break; }
        }
        if (index > 0 && argument == segments[index].Minimum)
        {
            return Choose(segments[index - 1].Evaluate(argument, EvaluationSide.Left), segments[index].Evaluate(argument, EvaluationSide.Right), side);
        }
        return segments[index].Evaluate(argument, side);
    }

    /// <summary>Returns only joins that fail the requested order. -1 means the values jump.</summary>
    public IReadOnlyList<ContinuityDiagnostic> ValidateContinuity(int requiredOrder)
    {
        if (requiredOrder is < 0 or > 3) { throw new ArgumentOutOfRangeException(nameof(requiredOrder)); }
        var failures = new List<ContinuityDiagnostic>();
        foreach (double point in Breakpoints)
        {
            int order = Check.Continuity(Evaluate(point, EvaluationSide.Left), Evaluate(point, EvaluationSide.Right), ContinuityTolerance);
            if (order < requiredOrder) { failures.Add(new(point, order)); }
        }
        if (Boundary != BoundaryPolicy.Reject)
        {
            foreach (double point in new[] { Minimum, Maximum })
            {
                int order = Check.Continuity(Evaluate(point, EvaluationSide.Left), Evaluate(point, EvaluationSide.Right), ContinuityTolerance);
                if (order < requiredOrder) { failures.Add(new(point, order)); }
            }
        }
        return failures.AsReadOnly();
    }

    /// <summary>Stateless time-origin evaluation, safe for repeated solver trial evaluations.</summary>
    public ScalarJet EvaluateAtTime(double simulationTime, double originTime, EvaluationSide side = EvaluationSide.Automatic)
    {
        if (Domain != ProfileDomain.Time) { throw new InvalidOperationException("This profile is distance based."); }
        Check.Finite(simulationTime, nameof(simulationTime));
        Check.Finite(originTime, nameof(originTime));
        return Evaluate(simulationTime - originTime, side);
    }

    private ScalarJet Choose(ScalarJet left, ScalarJet right, EvaluationSide side) => side switch
    {
        EvaluationSide.Left => left,
        EvaluationSide.Right => right,
        _ => right.LimitOrder(System.Math.Max(0, Check.Continuity(left, right, ContinuityTolerance)))
    };
}
