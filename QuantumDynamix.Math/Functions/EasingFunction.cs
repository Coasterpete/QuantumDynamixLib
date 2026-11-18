using System;
using System.Collections.Generic;
using System.Linq;

namespace QuantumDynamix.Math.Functions;

public enum EasingKind { Linear, Quadratic, Cubic, Quartic, Quintic, Sine, Cosine, Plateau, Custom }
public enum EasingDirection { In, Out, InOut }

/// <summary>Normalized transitions. Sine and Cosine remain distinct presets with shared mathematics.</summary>
public sealed class EasingFunction : IScalarFunction
{
    private readonly Func<double, EvaluationSide, ScalarJet>? custom;
    private readonly double holdStart;
    private readonly double holdEnd;
    private readonly double holdValue;
    public EasingKind Kind { get; }
    public EasingDirection Direction { get; }
    public double Minimum => 0;
    public double Maximum => 1;
    public IReadOnlyList<double> Breakpoints { get; }

    public EasingFunction(EasingKind kind = EasingKind.Linear, EasingDirection direction = EasingDirection.In,
        double holdStart = 0.25, double holdEnd = 0.75, double holdValue = 0.5)
        : this(kind, direction, holdStart, holdEnd, holdValue, null, Array.Empty<double>()) { }

    private EasingFunction(EasingKind kind, EasingDirection direction, double a, double b, double h,
        Func<double, EvaluationSide, ScalarJet>? custom, IEnumerable<double> customBreakpoints)
    {
        if (!Enum.IsDefined(kind) || !Enum.IsDefined(direction)) { throw new ArgumentOutOfRangeException(nameof(kind)); }
        if (kind == EasingKind.Custom && custom is null) { throw new ArgumentException("Use the Custom factory to supply a function.", nameof(kind)); }
        if (!(a > 0 && a < b && b < 1 && h > 0 && h < 1)) { throw new ArgumentException("Plateau requires 0 < start < end < 1 and 0 < value < 1."); }
        Kind = kind;
        Direction = direction;
        holdStart = a;
        holdEnd = b;
        holdValue = h;
        this.custom = custom;
        double[] points = kind == EasingKind.Plateau ? new[] { a, b } : customBreakpoints.ToArray();
        if (direction == EasingDirection.Out) { points = points.Select(x => 1 - x).Order().ToArray(); }
        if (direction == EasingDirection.InOut)
        { points = points.Select(x => x / 2).Concat(points.Select(x => 1 - x / 2)).Append(0.5).Distinct().Order().ToArray(); }
        Breakpoints = Array.AsReadOnly(points);
    }

    public static EasingFunction Custom(Func<double, EvaluationSide, ScalarJet> function,
        EasingDirection direction = EasingDirection.In, IEnumerable<double>? breakpoints = null)
    {
        var checkedFunction = new CustomFunction(0, 1, function, breakpoints);
        if (!Check.Near(checkedFunction.Evaluate(0).Value, 0, 1e-12) || !Check.Near(checkedFunction.Evaluate(1).Value, 1, 1e-12))
        { throw new ArgumentException("A custom transition must start at zero and end at one.", nameof(function)); }
        return new(EasingKind.Custom, direction, 0.25, 0.75, 0.5, checkedFunction.Evaluate, checkedFunction.Breakpoints);
    }

    public ScalarJet Evaluate(double argument, EvaluationSide side = EvaluationSide.Automatic)
    {
        Check.Argument(argument, 0, 1, side);
        if (Direction == EasingDirection.In) { return Base(argument, side); }
        if (Direction == EasingDirection.Out) { return Base(1 - argument, Reverse(side)).Transform(1, -1, -1); }
        if (argument == 0.5 && side == EvaluationSide.Automatic)
        {
            ScalarJet left = Base(1, EvaluationSide.Left).Transform(0, 0.5, 2);
            ScalarJet right = Base(1, EvaluationSide.Left).Transform(1, -0.5, -2);
            return right.LimitOrder(System.Math.Max(0, Check.Continuity(left, right)));
        }
        return argument < 0.5 || (argument == 0.5 && side == EvaluationSide.Left)
            ? Base(2 * argument, side).Transform(0, 0.5, 2)
            : Base(2 - 2 * argument, Reverse(side)).Transform(1, -0.5, -2);
    }

    private ScalarJet Base(double x, EvaluationSide side)
    {
        if (Kind == EasingKind.Custom) { return custom!(x, side); }
        if (Kind == EasingKind.Plateau)
        {
            if (x < holdStart) { return SmoothShoulder(x / holdStart).Transform(0, holdValue, 1 / holdStart); }
            if (x <= holdEnd) { return new(holdValue); }
            return SmoothShoulder((x - holdEnd) / (1 - holdEnd)).Transform(holdValue, 1 - holdValue, 1 / (1 - holdEnd));
        }
        if (Kind is EasingKind.Sine or EasingKind.Cosine)
        {
            double k = System.Math.PI / 2;
            double sine = x == 1 ? 1 : System.Math.Sin(k * x);
            double cosine = x == 1 ? 0 : System.Math.Cos(k * x);
            // 2*sin(theta/2)^2 avoids cancellation near zero.
            double value = x == 1 ? 1 : 2 * System.Math.Pow(System.Math.Sin(k * x / 2), 2);
            return new(value, k * sine, k * k * cosine, -k * k * k * sine);
        }
        int n = Kind switch { EasingKind.Quadratic => 2, EasingKind.Cubic => 3, EasingKind.Quartic => 4, EasingKind.Quintic => 5, _ => 1 };
        return new(System.Math.Pow(x, n), n * System.Math.Pow(x, n - 1),
            n < 2 ? 0 : n * (n - 1) * System.Math.Pow(x, n - 2),
            n < 3 ? 0 : n * (n - 1) * (n - 2) * System.Math.Pow(x, n - 3));
    }

    internal static ScalarJet SmoothShoulder(double x)
    {
        if (x > 0.5) { return SmoothShoulder(1 - x).Transform(1, -1, -1); }
        // Q'=140*x^3*(1-x)^3. First three derivatives vanish at either endpoint.
        double value = x * x * x * x * (35 + x * (-84 + x * (70 - 20 * x)));
        double first = 140 * x * x * x * System.Math.Pow(1 - x, 3);
        double second = 420 * x * x * (1 - x) * (1 - x) * (1 - 2 * x);
        double third = 840 * x * (1 - x) * (1 - 5 * x + 5 * x * x);
        return new(value, first, second, third);
    }

    private static EvaluationSide Reverse(EvaluationSide side) => side switch
    { EvaluationSide.Left => EvaluationSide.Right, EvaluationSide.Right => EvaluationSide.Left, _ => EvaluationSide.Automatic };
}

/// <summary>Normalized rise-hold-fall envelope: zero at both ends, one on the hold interval.</summary>
public sealed class RiseHoldFallEnvelope : IScalarFunction
{
    public double RiseEnd { get; }
    public double FallStart { get; }
    public double Minimum => 0;
    public double Maximum => 1;
    public IReadOnlyList<double> Breakpoints { get; }
    public RiseHoldFallEnvelope(double riseEnd = 0.25, double fallStart = 0.75)
    {
        if (!(riseEnd > 0 && riseEnd <= fallStart && fallStart < 1)) { throw new ArgumentException("Require 0 < rise end <= fall start < 1."); }
        RiseEnd = riseEnd;
        FallStart = fallStart;
        Breakpoints = Array.AsReadOnly(new[] { riseEnd, fallStart }.Distinct().ToArray());
    }
    public ScalarJet Evaluate(double argument, EvaluationSide side = EvaluationSide.Automatic)
    {
        Check.Argument(argument, 0, 1, side);
        if (argument < RiseEnd) { return EasingFunction.SmoothShoulder(argument / RiseEnd).Transform(0, 1, 1 / RiseEnd); }
        if (argument <= FallStart) { return new(1); }
        return EasingFunction.SmoothShoulder((1 - argument) / (1 - FallStart)).Transform(0, 1, -1 / (1 - FallStart));
    }
}
