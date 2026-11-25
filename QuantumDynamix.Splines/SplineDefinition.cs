using System;
using System.Collections.Generic;
using System.Linq;
using QuantumDynamix.Math.Functions;

namespace QuantumDynamix.Splines;

/// <summary>Immutable degree, knots, and positive weights shared by scalar and spatial curves.</summary>
public sealed class SplineDefinition
{
    private readonly double[] knots;
    private readonly double[] scaledWeights;
    public int Degree { get; }
    public int ControlPointCount => scaledWeights.Length;
    public IReadOnlyList<double> Knots { get; }
    public IReadOnlyList<double> Weights { get; }
    public IReadOnlyList<double> Breakpoints { get; }
    public double Minimum => knots[Degree];
    public double Maximum => knots[ControlPointCount];
    public bool IsPeriodic { get; }

    public SplineDefinition(int degree, IEnumerable<double> knots, IEnumerable<double>? weights = null, bool isPeriodic = false)
    {
        ArgumentNullException.ThrowIfNull(knots);
        this.knots = knots.ToArray();
        if (degree < 1 || degree > (this.knots.Length - 2) / 2) { throw new ArgumentOutOfRangeException(nameof(degree)); }
        Degree = degree;
        int count = this.knots.Length - degree - 1;
        for (int i = 0; i < this.knots.Length; i++)
        {
            Check.Finite(this.knots[i], nameof(knots));
            if (i > 0 && this.knots[i] < this.knots[i - 1]) { throw new ArgumentException("Knots must be nondecreasing.", nameof(knots)); }
            if (i > degree && this.knots[i] == this.knots[i - degree - 1])
            {
                throw new ArgumentException("Knot multiplicity exceeds degree plus one.", nameof(knots));
            }
        }
        Check.Interval(this.knots[degree], this.knots[count]);
        if (!double.IsFinite(this.knots[^1] - this.knots[0])) { throw new ArgumentException("Knot range is not finite.", nameof(knots)); }
        double[] supplied = weights?.ToArray() ?? Enumerable.Repeat(1.0, count).ToArray();
        if (supplied.Length != count || supplied.Any(w => !double.IsFinite(w) || w <= 0))
        {
            throw new ArgumentException("Supply one positive finite weight per control point.", nameof(weights));
        }
        double maximumWeight = supplied.Max();
        scaledWeights = supplied.Select(w => w / maximumWeight).ToArray();
        if (scaledWeights.Any(w => w == 0)) { throw new ArgumentException("Weight ratios underflow double precision.", nameof(weights)); }
        Knots = Array.AsReadOnly(this.knots);
        Weights = Array.AsReadOnly(supplied);
        Breakpoints = Array.AsReadOnly(this.knots.Where(x => x > Minimum && x < Maximum).Distinct().ToArray());
        IsPeriodic = isPeriodic;
    }

    public static SplineDefinition Clamped(int controlPointCount, int degree = 3, IEnumerable<double>? weights = null)
    {
        if (degree < 1 || controlPointCount <= degree) { throw new ArgumentOutOfRangeException(nameof(controlPointCount)); }
        double[] values = new double[controlPointCount + degree + 1];
        for (int i = 0; i < values.Length; i++)
        {
            values[i] = i <= degree ? 0 : i >= controlPointCount ? 1 : (double)(i - degree) / (controlPointCount - degree);
        }
        return new(degree, values, weights);
    }

    /// <summary>Use with controls and weights extended by their first degree entries.</summary>
    public static SplineDefinition UniformPeriodic(int uniqueControlPointCount, int degree, IEnumerable<double>? extendedWeights = null)
    {
        if (degree < 1 || uniqueControlPointCount <= degree) { throw new ArgumentOutOfRangeException(nameof(uniqueControlPointCount)); }
        double[] values = Enumerable.Range(0, uniqueControlPointCount + 2 * degree + 1)
            .Select(i => (double)(i - degree) / uniqueControlPointCount).ToArray();
        return new(degree, values, extendedWeights, true);
    }

    /// <summary>Guaranteed continuity; -1 denotes a possible value discontinuity.</summary>
    public int ContinuityAt(double parameter)
    {
        Check.Argument(parameter, Minimum, Maximum, EvaluationSide.Automatic);
        if (parameter == Minimum || parameter == Maximum) { return IsPeriodic ? Degree - 1 : int.MaxValue; }
        int multiplicity = knots.Count(t => t == parameter);
        return multiplicity == 0 ? int.MaxValue : Degree - multiplicity;
    }

    /// <summary>Local rational basis derivatives. Polynomial basis derivatives above degree are zero;
    /// rational basis derivatives can remain nonzero.</summary>
    public SplineBasisSample EvaluateBasis(double parameter, int derivativeOrder = 3, EvaluationSide side = EvaluationSide.Automatic)
    {
        Check.Argument(parameter, Minimum, Maximum, side);
        if (derivativeOrder < 0 || derivativeOrder > Degree + 3) { throw new ArgumentOutOfRangeException(nameof(derivativeOrder)); }
        int span;
        if (parameter == Maximum)
        {
            span = ControlPointCount - 1;
            while (span > Degree && knots[span] == Maximum) { span--; }
        }
        else if (parameter == Minimum)
        {
            span = Degree;
            while (span + 1 < ControlPointCount && knots[span + 1] == Minimum) { span++; }
        }
        else
        {
            int low = Degree;
            int high = ControlPointCount;
            while (low + 1 < high)
            {
                int middle = low + (high - low) / 2;
                if (knots[middle] < parameter || (side != EvaluationSide.Left && knots[middle] == parameter)) { low = middle; }
                else { high = middle; }
            }
            span = low;
        }
        // Work only on the p+1 local basis functions; differentiate the degree recurrence.
        int p = Degree;
        int first = span - p;
        int columns = p + 2;
        long bufferLength = (derivativeOrder + 1L) * columns;
        const int MaximumStackEntries = 128;
        Span<double> previous = bufferLength <= MaximumStackEntries
            ? stackalloc double[(int)bufferLength] : new double[bufferLength];
        Span<double> current = bufferLength <= MaximumStackEntries
            ? stackalloc double[(int)bufferLength] : new double[bufferLength];
        // Each pass overwrites a growing region. Inactive cells and the right sentinel stay zero.
        previous.Clear();
        current.Clear();
        previous[p] = 1;
        for (int degree = 1; degree <= p; degree++)
        {
            for (int local = p - degree; local <= p; local++)
            {
                int i = first + local;
                double left = knots[i + degree] - knots[i];
                double right = knots[i + degree + 1] - knots[i + 1];
                current[local] = (left == 0 ? 0 : (parameter - knots[i]) / left * previous[local])
                    + (right == 0 ? 0 : (knots[i + degree + 1] - parameter) / right * previous[local + 1]);
                for (int order = 1; order <= System.Math.Min(degree, derivativeOrder); order++)
                {
                    int index = order * columns + local;
                    int previousIndex = index - columns;
                    current[index] = (left == 0 ? 0 : degree / left * previous[previousIndex])
                        - (right == 0 ? 0 : degree / right * previous[previousIndex + 1]);
                }
            }
            Span<double> reusable = previous;
            previous = current;
            current = reusable;
        }
        double[] denominator = new double[derivativeOrder + 1];
        for (int order = 0; order <= derivativeOrder; order++)
        {
            int row = order * columns;
            for (int j = 0; j <= p; j++) { denominator[order] += previous[row + j] * scaledWeights[first + j]; }
        }
        if (!(denominator[0] > 0) || !double.IsFinite(denominator[0])) { throw new ArithmeticException("Rational denominator is not resolvable."); }
        double[,] rational = new double[derivativeOrder + 1, p + 1];
        for (int order = 0; order <= derivativeOrder; order++)
        {
            int row = order * columns;
            for (int j = 0; j <= p; j++)
            {
                double numerator = previous[row + j] * scaledWeights[first + j];
                double binomial = 1;
                for (int k = 1; k <= order; k++)
                {
                    binomial *= (double)(order - k + 1) / k;
                    numerator -= binomial * denominator[k] * rational[order - k, j];
                }
                rational[order, j] = numerator / denominator[0];
                if (!double.IsFinite(rational[order, j])) { throw new ArithmeticException("Spline basis derivative exceeds finite range."); }
            }
        }
        return new(first, rational);
    }

    public double WrapParameter(double parameter)
    {
        Check.Finite(parameter, nameof(parameter));
        if (!IsPeriodic) { throw new InvalidOperationException("Wrapping requires an explicitly periodic definition."); }
        double shifted = parameter - Minimum;
        if (!double.IsFinite(shifted)) { throw new ArgumentOutOfRangeException(nameof(parameter)); }
        double remainder = shifted % (Maximum - Minimum);
        return Minimum + (remainder < 0 ? remainder + Maximum - Minimum : remainder);
    }
}

/// <summary>Read-only local basis values and derivatives; coefficient indices begin at FirstControlPoint.</summary>
public sealed class SplineBasisSample
{
    private readonly double[,] values;
    public int FirstControlPoint { get; }
    public int Count => values.GetLength(1);
    public int DerivativeOrder => values.GetLength(0) - 1;
    internal SplineBasisSample(int first, double[,] values) { FirstControlPoint = first; this.values = values; }
    public double this[int order, int localControlPoint] => values[order, localControlPoint];
}
