using System;
using System.Collections.Generic;
using QuantumDynamix.Math.Functions;

namespace QuantumDynamix.Math.Numerics;

public sealed class QuadratureSettings
{
    public double AbsoluteTolerance { get; }
    public double RelativeTolerance { get; }
    public int MaximumDepth { get; }
    public int MaximumEvaluations { get; }
    public QuadratureSettings(double absoluteTolerance = 1e-8, double relativeTolerance = 1e-10,
        int maximumDepth = 20, int maximumEvaluations = 100000)
    {
        if (!double.IsFinite(absoluteTolerance) || absoluteTolerance <= 0 || !double.IsFinite(relativeTolerance)
            || relativeTolerance < 0 || relativeTolerance >= 1 || maximumDepth < 1 || maximumEvaluations < 15)
        { throw new ArgumentOutOfRangeException(nameof(absoluteTolerance), "Supply positive finite absolute tolerance and valid quadrature limits."); }
        AbsoluteTolerance = absoluteTolerance;
        RelativeTolerance = relativeTolerance;
        MaximumDepth = maximumDepth;
        MaximumEvaluations = maximumEvaluations;
    }
}

public readonly record struct QuadratureResult(double Value, double EstimatedAbsoluteError, int Evaluations, int Subintervals);

public sealed class NumericalConvergenceException : InvalidOperationException
{
    public int Iterations { get; }
    public double Residual { get; }
    internal (double Start, double End)? ParameterInterval { get; init; }
    internal (string Quantity, string Unit, double Limit)? ResidualBudget { get; init; }
    internal NumericalConvergenceException(string message, int iterations, double residual, Exception? inner = null) : base(message, inner)
    { Iterations = iterations; Residual = residual; }
}

/// <summary>Adaptive embedded 7/15 Gaussian-Kronrod quadrature; error estimates are not certified bounds.</summary>
public static class AdaptiveGaussKronrod
{
    // Mathematical Gauss-Kronrod abscissae and weights on [-1,1].
    private static readonly double[] Nodes = { 0.9914553711208126, 0.9491079123427585, 0.8648644233597691,
        0.7415311855993944, 0.5860872354676911, 0.4058451513773972, 0.2077849550078985, 0 };
    private static readonly double[] Kronrod = { 0.02293532201052922, 0.06309209262997855, 0.1047900103222502,
        0.1406532597155259, 0.1690047266392679, 0.1903505780647854, 0.2044329400752989, 0.2094821410847278 };
    private static readonly double[] Gauss = { 0.1294849661688697, 0.2797053914892767, 0.3818300505051189, 0.4179591836734694 };

    internal static IEnumerable<(double Argument, double Weight)> WeightedNodes(double a, double b)
    {
        double half = (b - a) / 2;
        double center = a + half;
        yield return (center, half * Kronrod[7]);
        for (int i = 0; i < 7; i++)
        {
            yield return (center - half * Nodes[i], half * Kronrod[i]);
            yield return (center + half * Nodes[i], half * Kronrod[i]);
        }
    }

    public static QuadratureResult Integrate(Func<double, double> function, double minimum, double maximum,
        QuadratureSettings? settings = null)
    {
        ArgumentNullException.ThrowIfNull(function);
        Check.Finite(minimum, nameof(minimum));
        Check.Finite(maximum, nameof(maximum));
        if (minimum == maximum) { return new(0, 0, 0, 0); }
        if (minimum > maximum)
        {
            QuadratureResult reversed = Integrate(function, maximum, minimum, settings);
            return reversed with { Value = -reversed.Value };
        }
        Check.Interval(minimum, maximum);
        settings ??= new();
        var intervals = new List<Interval> { Rule(minimum, maximum, 0) };
        int evaluations = 15;
        double total = intervals[0].Value;
        double error = intervals[0].Error;
        double tolerance = settings.AbsoluteTolerance + settings.RelativeTolerance * System.Math.Abs(total);
        while (error > tolerance)
        {
            int worst = 0;
            for (int i = 1; i < intervals.Count; i++) { if (intervals[i].Error > intervals[worst].Error) { worst = i; } }
            Interval old = intervals[worst];
            double middle = old.Minimum + (old.Maximum - old.Minimum) / 2;
            if (old.Depth >= settings.MaximumDepth || evaluations > settings.MaximumEvaluations - 30 || !(middle > old.Minimum && middle < old.Maximum))
            {
                throw new NumericalConvergenceException(FormattableString.Invariant(
                    $"Quadrature exhausted its subdivision or evaluation budget over [{minimum:R}, {maximum:R}] after {evaluations} integrand evaluations. The active subinterval count is {intervals.Count}."),
                    evaluations, error)
                { ResidualBudget = ("Quadrature error estimate", "integral units", tolerance) };
            }
            Interval left = Rule(old.Minimum, middle, old.Depth + 1);
            Interval right = Rule(middle, old.Maximum, old.Depth + 1);
            intervals[worst] = left;
            intervals.Add(right);
            total += left.Value + right.Value - old.Value;
            error = System.Math.Max(0, error + left.Error + right.Error - old.Error);
            evaluations += 30;
            tolerance = settings.AbsoluteTolerance + settings.RelativeTolerance * System.Math.Abs(total);
        }
        if (!double.IsFinite(total) || !double.IsFinite(error)) { throw new ArithmeticException("Quadrature exceeds finite range."); }
        return new(total, error, evaluations, intervals.Count);

        Interval Rule(double a, double b, int depth)
        {
            double half = (b - a) / 2;
            double center = a + half;
            double centerValue = Value(center);
            double k = Kronrod[7] * centerValue;
            double g = Gauss[3] * centerValue;
            double absolute = Kronrod[7] * System.Math.Abs(centerValue);
            Span<double> lows = stackalloc double[7];
            Span<double> highs = stackalloc double[7];
            for (int j = 0; j < 7; j++)
            {
                lows[j] = Value(center - half * Nodes[j]);
                highs[j] = Value(center + half * Nodes[j]);
                double sum = lows[j] + highs[j];
                k += Kronrod[j] * sum;
                absolute += Kronrod[j] * (System.Math.Abs(lows[j]) + System.Math.Abs(highs[j]));
                if (j % 2 == 1) { g += Gauss[(j - 1) / 2] * sum; }
            }
            double mean = k / 2;
            double asc = Kronrod[7] * System.Math.Abs(centerValue - mean);
            for (int j = 0; j < 7; j++) { asc += Kronrod[j] * (System.Math.Abs(lows[j] - mean) + System.Math.Abs(highs[j] - mean)); }
            double estimate = System.Math.Abs((k - g) * half);
            asc *= half;
            if (asc > 0 && estimate > 0) { estimate = asc * System.Math.Min(1, System.Math.Pow(200 * estimate / asc, 1.5)); }
            estimate = System.Math.Max(estimate, 50 * 2.2204460492503131e-16 * absolute * half);
            if (!double.IsFinite(k * half) || !double.IsFinite(estimate)) { throw new ArithmeticException("Quadrature arithmetic exceeds finite range."); }
            return new(a, b, k * half, estimate, depth);
        }
        double Value(double x)
        {
            double value = function(x);
            if (!double.IsFinite(value)) { throw new ArithmeticException("Integrand returned a non-finite value."); }
            return value;
        }
    }

    private readonly record struct Interval(double Minimum, double Maximum, double Value, double Error, int Depth);
}
