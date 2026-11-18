using System;
using System.Collections.Generic;
using System.Linq;
using QuantumDynamix.Math;
using QuantumDynamix.Math.Functions;

namespace QuantumDynamix.Splines.Curves;

public readonly record struct CurveJet3d
{
    private readonly Vector3d first;
    private readonly Vector3d second;
    private readonly Vector3d third;
    public Vector3d Position { get; }
    public int DerivativeOrder { get; }
    public Vector3d FirstDerivative => Derivative(1);
    public Vector3d SecondDerivative => Derivative(2);
    public Vector3d ThirdDerivative => Derivative(3);
    internal CurveJet3d(Vector3d position, Vector3d first, Vector3d second, Vector3d third, int order)
    { Position = position; this.first = first; this.second = second; this.third = third; DerivativeOrder = order; }
    public Vector3d Derivative(int order)
    {
        if (order is < 0 or > 3) { throw new ArgumentOutOfRangeException(nameof(order)); }
        if (order > DerivativeOrder) { throw new InvalidOperationException("Requested derivative is discontinuous; request a sided limit."); }
        return order switch { 0 => Position, 1 => first, 2 => second, _ => third };
    }
}

/// <summary>Immutable spatial B-spline/NURBS curve. Parameter derivatives are not station derivatives.</summary>
public sealed class SplineCurve3d
{
    private readonly Vector3d[] controls;
    public SplineDefinition Definition { get; }
    public IReadOnlyList<Vector3d> ControlPoints { get; }
    public double Minimum => Definition.Minimum;
    public double Maximum => Definition.Maximum;

    public SplineCurve3d(SplineDefinition definition, IEnumerable<Vector3d> controlPoints)
    {
        ArgumentNullException.ThrowIfNull(definition);
        ArgumentNullException.ThrowIfNull(controlPoints);
        controls = controlPoints.ToArray();
        if (controls.Length != definition.ControlPointCount) { throw new ArgumentException("Control count does not match the spline definition.", nameof(controlPoints)); }
        Definition = definition;
        ControlPoints = Array.AsReadOnly(controls);
        if (definition.IsPeriodic)
        {
            double positionScale = System.Math.Max(1, controls.Max(p => (p - controls[0]).Length));
            for (int order = 0; order < definition.Degree; order++)
            {
                Vector3d a = EvaluateDerivative(Minimum, order, EvaluationSide.Right);
                Vector3d b = EvaluateDerivative(Maximum, order, EvaluationSide.Left);
                double scale = order == 0 ? positionScale : System.Math.Max(1, System.Math.Max(a.Length, b.Length));
                if ((a - b).Length > 1e-9 * scale)
                { throw new ArgumentException("Periodic curve does not match through degree minus one derivatives.", nameof(controlPoints)); }
            }
        }
    }

    public static SplineCurve3d Clamped(IEnumerable<Vector3d> controls, int degree = 5, IEnumerable<double>? weights = null)
    {
        ArgumentNullException.ThrowIfNull(controls);
        Vector3d[] points = controls.ToArray();
        return new(SplineDefinition.Clamped(points.Length, degree, weights), points);
    }

    public static SplineCurve3d Periodic(IEnumerable<Vector3d> controls, int degree = 5, IEnumerable<double>? weights = null)
    {
        ArgumentNullException.ThrowIfNull(controls);
        Vector3d[] points = controls.ToArray();
        double[] supplied = weights?.ToArray() ?? Enumerable.Repeat(1.0, points.Length).ToArray();
        if (supplied.Length != points.Length) { throw new ArgumentException("Weight count must match controls.", nameof(weights)); }
        return new(SplineDefinition.UniformPeriodic(points.Length, degree, supplied.Concat(supplied.Take(degree))),
            points.Concat(points.Take(degree)));
    }

    public CurveJet3d Evaluate(double parameter, EvaluationSide side = EvaluationSide.Automatic)
    {
        SplineBasisSample basis = Definition.EvaluateBasis(parameter, 3, side);
        Vector3d[] values = new Vector3d[4];
        for (int order = 0; order <= 3; order++)
        {
            for (int j = 0; j < basis.Count; j++) { values[order] += controls[basis.FirstControlPoint + j] * basis[order, j]; }
        }
        int available = side == EvaluationSide.Automatic ? System.Math.Clamp(Definition.ContinuityAt(parameter), 0, 3) : 3;
        return new(values[0], values[1], values[2], values[3], available);
    }

    private Vector3d EvaluateDerivative(double parameter, int order, EvaluationSide side)
    {
        SplineBasisSample basis = Definition.EvaluateBasis(parameter, order, side);
        Vector3d value = default;
        for (int j = 0; j < basis.Count; j++) { value += controls[basis.FirstControlPoint + j] * basis[order, j]; }
        return value;
    }
}

/// <summary>A single-valued scalar profile using the same rational basis as spatial geometry.</summary>
public sealed class ScalarSpline : IScalarFunction
{
    private readonly double[] coefficients;
    public SplineDefinition Definition { get; }
    public IReadOnlyList<double> Coefficients { get; }
    public double Minimum => Definition.Minimum;
    public double Maximum => Definition.Maximum;
    public IReadOnlyList<double> Breakpoints => Definition.Breakpoints;

    public ScalarSpline(SplineDefinition definition, IEnumerable<double> coefficients)
    {
        ArgumentNullException.ThrowIfNull(definition);
        ArgumentNullException.ThrowIfNull(coefficients);
        this.coefficients = coefficients.ToArray();
        if (this.coefficients.Length != definition.ControlPointCount || this.coefficients.Any(c => !double.IsFinite(c)))
        { throw new ArgumentException("Supply one finite coefficient per control point.", nameof(coefficients)); }
        Definition = definition;
        Coefficients = Array.AsReadOnly(this.coefficients);
        if (definition.IsPeriodic)
        {
            double valueScale = System.Math.Max(1, this.coefficients.Max(c => System.Math.Abs(c - this.coefficients[0])));
            SplineBasisSample left = definition.EvaluateBasis(Minimum, definition.Degree - 1, EvaluationSide.Right);
            SplineBasisSample right = definition.EvaluateBasis(Maximum, definition.Degree - 1, EvaluationSide.Left);
            for (int order = 0; order < definition.Degree; order++)
            {
                double a = Sum(left, order);
                double b = Sum(right, order);
                bool matches = order == 0 ? System.Math.Abs(a - b) <= 1e-9 * valueScale : Check.Near(a, b, 1e-9);
                if (!matches) { throw new ArgumentException("Periodic coefficients do not close smoothly.", nameof(coefficients)); }
            }
        }
    }

    public static ScalarSpline Clamped(IEnumerable<double> coefficients, int degree = 3, IEnumerable<double>? weights = null)
    {
        ArgumentNullException.ThrowIfNull(coefficients);
        double[] values = coefficients.ToArray();
        return new(SplineDefinition.Clamped(values.Length, degree, weights), values);
    }

    public static ScalarSpline Periodic(IEnumerable<double> coefficients, int degree = 3, IEnumerable<double>? weights = null)
    {
        ArgumentNullException.ThrowIfNull(coefficients);
        double[] values = coefficients.ToArray();
        double[] supplied = weights?.ToArray() ?? Enumerable.Repeat(1.0, values.Length).ToArray();
        if (supplied.Length != values.Length) { throw new ArgumentException("Weight count must match coefficients.", nameof(weights)); }
        return new(SplineDefinition.UniformPeriodic(values.Length, degree, supplied.Concat(supplied.Take(degree))), values.Concat(values.Take(degree)));
    }

    public ScalarJet Evaluate(double parameter, EvaluationSide side = EvaluationSide.Automatic)
    {
        SplineBasisSample basis = Definition.EvaluateBasis(parameter, 3, side);
        int available = side == EvaluationSide.Automatic ? System.Math.Clamp(Definition.ContinuityAt(parameter), 0, 3) : 3;
        return new(Sum(basis, 0), Sum(basis, 1), Sum(basis, 2), Sum(basis, 3), available);
    }

    private double Sum(SplineBasisSample basis, int order)
    {
        double value = 0;
        for (int j = 0; j < basis.Count; j++) { value += coefficients[basis.FirstControlPoint + j] * basis[order, j]; }
        if (!double.IsFinite(value)) { throw new ArithmeticException("Spline value exceeds finite range."); }
        return value;
    }
}
