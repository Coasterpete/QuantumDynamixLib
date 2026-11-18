using System;
using System.Collections.Generic;
using System.Linq;

namespace QuantumDynamix.Math.Functions;

/// <summary>Automatic reports only derivatives defined across a join; sided requests return limits.</summary>
public enum EvaluationSide { Automatic, Left, Right }

/// <summary>A value and up to three analytic derivatives with respect to the function's argument.</summary>
public readonly record struct ScalarJet
{
    private readonly double first;
    private readonly double second;
    private readonly double third;
    public double Value { get; }
    public int DerivativeOrder { get; }
    public double FirstDerivative => Derivative(1);
    public double SecondDerivative => Derivative(2);
    public double ThirdDerivative => Derivative(3);

    public ScalarJet(double value, double first = 0, double second = 0, double third = 0, int derivativeOrder = 3)
    {
        Check.Finite(value, nameof(value));
        Check.Finite(first, nameof(first));
        Check.Finite(second, nameof(second));
        Check.Finite(third, nameof(third));
        if (derivativeOrder is < 0 or > 3) { throw new ArgumentOutOfRangeException(nameof(derivativeOrder)); }
        Value = value;
        this.first = first;
        this.second = second;
        this.third = third;
        DerivativeOrder = derivativeOrder;
    }

    public double Derivative(int order)
    {
        if (order < 0 || order > 3) { throw new ArgumentOutOfRangeException(nameof(order)); }
        if (order > DerivativeOrder) { throw new InvalidOperationException($"Derivative order {order} is unavailable; available order is {DerivativeOrder}."); }
        return order switch { 0 => Value, 1 => first, 2 => second, _ => third };
    }

    internal ScalarJet Transform(double offset, double amplitude, double argumentScale) => new(
        offset + amplitude * Value, amplitude * argumentScale * first,
        amplitude * argumentScale * argumentScale * second,
        amplitude * argumentScale * argumentScale * argumentScale * third, DerivativeOrder);

    internal ScalarJet LimitOrder(int order) => new(Value, first, second, third, System.Math.Min(order, DerivativeOrder));
}

/// <summary>A pure scalar function. Breakpoints identify possible changes in derivative continuity.</summary>
public interface IScalarFunction
{
    double Minimum { get; }
    double Maximum { get; }
    IReadOnlyList<double> Breakpoints { get; }
    ScalarJet Evaluate(double argument, EvaluationSide side = EvaluationSide.Automatic);
}

/// <summary>Caller-defined mathematics; derivatives and sided limits are supplied by the caller.</summary>
public sealed class CustomFunction : IScalarFunction
{
    private readonly Func<double, EvaluationSide, ScalarJet> evaluate;
    public double Minimum { get; }
    public double Maximum { get; }
    public IReadOnlyList<double> Breakpoints { get; }

    public CustomFunction(double minimum, double maximum, Func<double, EvaluationSide, ScalarJet> evaluate,
        IEnumerable<double>? breakpoints = null)
    {
        Check.Interval(minimum, maximum);
        ArgumentNullException.ThrowIfNull(evaluate);
        Minimum = minimum;
        Maximum = maximum;
        this.evaluate = evaluate;
        double[] points = (breakpoints ?? Array.Empty<double>()).ToArray();
        if (points.Any(x => !double.IsFinite(x) || x <= minimum || x >= maximum)
            || !points.SequenceEqual(points.Distinct().Order()))
        {
            throw new ArgumentException("Breakpoints must be distinct, ordered, and strictly inside the domain.", nameof(breakpoints));
        }
        Breakpoints = Array.AsReadOnly(points);
    }

    public ScalarJet Evaluate(double argument, EvaluationSide side = EvaluationSide.Automatic)
    {
        Check.Argument(argument, Minimum, Maximum, side);
        return evaluate(argument, side);
    }
}

internal static class Check
{
    internal static void Finite(double value, string name)
    {
        if (!double.IsFinite(value)) { throw new ArgumentOutOfRangeException(name, "Value must be finite."); }
    }

    internal static void Interval(double minimum, double maximum)
    {
        Finite(minimum, nameof(minimum));
        Finite(maximum, nameof(maximum));
        if (!(maximum > minimum) || !double.IsFinite(maximum - minimum))
        {
            throw new ArgumentException("Resolve a finite, strictly positive interval.");
        }
    }

    internal static void Argument(double value, double minimum, double maximum, EvaluationSide side)
    {
        if (!Enum.IsDefined(side)) { throw new ArgumentOutOfRangeException(nameof(side)); }
        if (!double.IsFinite(value) || value < minimum || value > maximum)
        {
            throw new ArgumentOutOfRangeException(nameof(value), "Argument lies outside the finite function domain.");
        }
    }

    internal static bool Near(double a, double b, double tolerance) =>
        System.Math.Abs(a - b) <= tolerance * System.Math.Max(1, System.Math.Max(System.Math.Abs(a), System.Math.Abs(b)));

    internal static int Continuity(ScalarJet left, ScalarJet right, double tolerance = 1e-10)
    {
        int available = System.Math.Min(left.DerivativeOrder, right.DerivativeOrder);
        for (int i = 0; i <= available; i++)
        {
            if (!Near(left.Derivative(i), right.Derivative(i), tolerance)) { return i - 1; }
        }
        return available;
    }
}
