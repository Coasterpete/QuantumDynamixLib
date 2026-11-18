using System;
using System.Collections.Generic;
using System.Linq;
using QuantumDynamix.Geometry;
using QuantumDynamix.Math;
using QuantumDynamix.Math.Functions;
using QuantumDynamix.Math.Numerics;
using QuantumDynamix.Splines.Curves;

namespace QuantumDynamix.Splines.Fitting;

/// <summary>Dimensionless strength after domain/value normalization. Geometry roughness uses the fixed
/// original arc-length coordinate. Scalar roughness uses the original scalar argument. No knots or weights change.</summary>
public sealed class SmoothingSettings
{
    public double Strength { get; }
    /// <summary>Null selects order two for scalar profiles and order three for spatial geometry.</summary>
    public int? RoughnessOrder { get; }
    public double? RegionMinimum { get; }
    public double? RegionMaximum { get; }
    public bool LockBoundaries { get; }
    public int BoundaryDerivativeOrder { get; }
    public int QuadratureSubdivisions { get; }
    public double ValueScale { get; }
    public double ConstraintTolerance { get; }
    public Func<double, double>? RoughnessWeight { get; }

    public SmoothingSettings(double strength = 0, int? roughnessOrder = null, double? regionMinimum = null, double? regionMaximum = null,
        bool lockBoundaries = true, int boundaryDerivativeOrder = 3, int quadratureSubdivisions = 2,
        double valueScale = 1, double constraintTolerance = 1e-9, Func<double, double>? roughnessWeight = null)
    {
        if (!double.IsFinite(strength) || strength < 0 || roughnessOrder is < 1 or > 3 || boundaryDerivativeOrder is < 0 or > 3
            || quadratureSubdivisions < 1 || !double.IsFinite(valueScale) || valueScale <= 0)
        { throw new ArgumentOutOfRangeException(nameof(strength), "Invalid smoothing strength, derivative order, quadrature count, or value scale."); }
        if (regionMinimum.HasValue != regionMaximum.HasValue) { throw new ArgumentException("Supply both region endpoints or neither."); }
        if (regionMinimum.HasValue) { Check.Interval(regionMinimum.Value, regionMaximum!.Value); }
        ConstrainedSplineFitter.ValidateTolerance(constraintTolerance);
        Strength = strength;
        RoughnessOrder = roughnessOrder;
        RegionMinimum = regionMinimum;
        RegionMaximum = regionMaximum;
        LockBoundaries = lockBoundaries;
        BoundaryDerivativeOrder = boundaryDerivativeOrder;
        QuadratureSubdivisions = quadratureSubdivisions;
        ValueScale = valueScale;
        ConstraintTolerance = constraintTolerance;
        RoughnessWeight = roughnessWeight;
    }
}

public readonly record struct ScalarComparisonSample(double Argument, ScalarJet Original, ScalarJet Candidate)
{
    public double Difference => Candidate.Value - Original.Value;
}
public readonly record struct CurveComparisonSample(double Parameter, Vector3d Original, Vector3d Candidate,
    double? OriginalCurvature, double? CandidateCurvature)
{
    public double Displacement => (Candidate - Original).Length;
}
/// <summary>Roughness and deviations are quadrature/sample estimates, not continuum bounds.</summary>
public readonly record struct SmoothingMetrics(double OriginalRoughness, double CandidateRoughness,
    double MaximumSampleDeviation, int FrozenControlCount, int QuadratureSampleCount);
public sealed record ScalarSmoothingResult(ScalarSpline Original, ScalarSpline? Candidate, FitDiagnostics Diagnostics,
    SmoothingMetrics Metrics, IReadOnlyList<ScalarComparisonSample> Comparison);
public sealed record CurveSmoothingResult(SplineCurve3d Original, SplineCurve3d? Candidate, FitDiagnostics Diagnostics,
    SmoothingMetrics Metrics, IReadOnlyList<CurveComparisonSample> Comparison);

/// <summary>Non-destructive constrained quadratic fairing. Feature constraints are exact value/derivative equations.
/// Curvature is reported for comparison; nonlinear curvature and physical-force optimization are not performed.</summary>
public static class SplineSmoothing
{
    public static ScalarSmoothingResult SmoothScalar(ScalarSpline original, SmoothingSettings? settings = null,
        IEnumerable<int>? lockedControlPoints = null, IEnumerable<ScalarFitConstraint>? constraints = null)
    {
        ArgumentNullException.ThrowIfNull(original);
        settings ??= new();
        SplineDefinition definition = original.Definition;
        (double a, double b) = Region(definition, settings);
        int[] frozen = Frozen(definition, a, b, lockedControlPoints);
        var conditions = (constraints ?? Array.Empty<ScalarFitConstraint>()).ToList();
        if (settings.LockBoundaries)
        {
            foreach ((double point, EvaluationSide side) in new[] { (a, EvaluationSide.Right), (b, EvaluationSide.Left) })
            {
                ScalarJet value = original.Evaluate(point, side);
                for (int order = 0; order <= settings.BoundaryDerivativeOrder; order++)
                { conditions.Add(new(point, order, value.Derivative(order), side)); }
            }
        }
        List<LinearEquation> equations = ConstrainedSplineFitter.Equations(definition, conditions);
        var locked = frozen.ToDictionary(i => i, i => original.Coefficients[i]);
        List<FairingRow> rows = Rows(definition, settings, a, b, null);
        ScalarFitResult fitted = Fair(definition, original.Coefficients.ToArray(), rows, settings, equations, locked);
        ScalarSpline? candidate = settings.Strength == 0 && fitted.Curve is not null ? original : fitted.Curve;
        double[] parameters = ComparisonParameters(definition, rows, a, b);
        ScalarComparisonSample[] comparison = candidate is null ? Array.Empty<ScalarComparisonSample>() : parameters.Select(x =>
            new ScalarComparisonSample(x, original.Evaluate(x, EvaluationSide.Right), candidate.Evaluate(x, EvaluationSide.Right))).ToArray();
        SmoothingMetrics metrics = Metrics(rows, original.Coefficients.ToArray(), candidate?.Coefficients.ToArray(), frozen.Length,
            comparison.Select(c => System.Math.Abs(c.Difference)).DefaultIfEmpty(0).Max());
        return new(original, candidate, fitted.Diagnostics, metrics, Array.AsReadOnly(comparison));
    }

    public static CurveSmoothingResult SmoothCurve(SplineCurve3d original, SmoothingSettings? settings = null,
        IEnumerable<int>? lockedControlPoints = null, IEnumerable<CurveFitConstraint>? constraints = null)
    {
        ArgumentNullException.ThrowIfNull(original);
        settings ??= new();
        SplineDefinition definition = original.Definition;
        (double a, double b) = Region(definition, settings);
        int[] frozen = Frozen(definition, a, b, lockedControlPoints);
        var conditions = (constraints ?? Array.Empty<CurveFitConstraint>()).ToList();
        if (settings.LockBoundaries)
        {
            foreach ((double point, EvaluationSide side) in new[] { (a, EvaluationSide.Right), (b, EvaluationSide.Left) })
            {
                CurveJet3d value = original.Evaluate(point, side);
                for (int order = 0; order <= settings.BoundaryDerivativeOrder; order++)
                { conditions.Add(new(point, order, value.Derivative(order), side)); }
            }
        }
        var arcLength = new ArcLengthMap(original);
        List<FairingRow> rows = Rows(definition, settings, a, b, arcLength);
        var fits = new List<ScalarFitResult>();
        for (int axis = 0; axis < 3; axis++)
        {
            double[] coefficients = original.ControlPoints.Select(p => ConstrainedSplineFitter.Component(p, axis)).ToArray();
            var equations = ConstrainedSplineFitter.Equations(definition, conditions.Select(c =>
                new ScalarFitConstraint(c.Parameter, c.DerivativeOrder, ConstrainedSplineFitter.Component(c.Value, axis), c.Side)));
            fits.Add(Fair(definition, coefficients, rows, settings, equations, frozen.ToDictionary(i => i, i => coefficients[i])));
        }
        ScalarFitResult? failure = fits.FirstOrDefault(f => f.Curve is null);
        if (failure is not null)
        { return new(original, null, failure.Diagnostics, new(0, 0, 0, frozen.Length, rows.Count), Array.Empty<CurveComparisonSample>()); }
        Vector3d[] points = Enumerable.Range(0, definition.ControlPointCount).Select(i => new Vector3d(
            fits[0].Curve!.Coefficients[i], fits[1].Curve!.Coefficients[i], fits[2].Curve!.Coefficients[i])).ToArray();
        SplineCurve3d candidate = settings.Strength == 0 ? original : new(definition, points);
        CurveComparisonSample[] comparison = ComparisonParameters(definition, rows, a, b).Select(x =>
            new CurveComparisonSample(x, original.Evaluate(x).Position, candidate.Evaluate(x).Position, Curvature(original, x), Curvature(candidate, x))).ToArray();
        double before = 0;
        double after = 0;
        for (int axis = 0; axis < 3; axis++)
        {
            SmoothingMetrics metric = Metrics(rows, original.ControlPoints.Select(p => ConstrainedSplineFitter.Component(p, axis)).ToArray(),
                candidate.ControlPoints.Select(p => ConstrainedSplineFitter.Component(p, axis)).ToArray(), frozen.Length, 0);
            before += metric.OriginalRoughness;
            after += metric.CandidateRoughness;
        }
        return new(original, candidate, ConstrainedSplineFitter.Combine(fits.Select(f => f.Diagnostics)),
            new(before, after, comparison.Select(c => c.Displacement).DefaultIfEmpty(0).Max(), frozen.Length, rows.Count), Array.AsReadOnly(comparison));
    }

    private static ScalarFitResult Fair(SplineDefinition definition, double[] coefficients, List<FairingRow> rows,
        SmoothingSettings settings, List<LinearEquation> equations, Dictionary<int, double> locked)
    {
        if (settings.Strength == 0)
        {
            double maximum = equations.Select(e => System.Math.Abs(ConstrainedLeastSquares.Dot(e.Row, coefficients) - e.Target)
                / System.Math.Max(1, System.Math.Abs(e.Target))).DefaultIfEmpty(0).Max();
            return maximum > settings.ConstraintTolerance
                ? new(null, new(FitStatus.Infeasible, "Strength zero preserves the original, which conflicts with the requested constraints.", 0, 0, 0, maximum))
                : new(new(definition, coefficients), new(FitStatus.Success, "Strength zero preserves the original exactly.", 0, 0, 0, maximum));
        }
        double[,] matrix = new double[2 * rows.Count, coefficients.Length];
        double[] rhs = new double[2 * rows.Count];
        for (int i = 0; i < rows.Count; i++)
        {
            FairingRow row = rows[i];
            double dataScale = System.Math.Sqrt(row.Weight);
            double roughScale = System.Math.Sqrt(row.Weight * row.RoughnessWeight) * System.Math.Sqrt(settings.Strength);
            for (int j = 0; j < coefficients.Length; j++)
            {
                matrix[2 * i, j] = row.Value[j] * dataScale;
                matrix[2 * i + 1, j] = row.Roughness[j] * roughScale;
            }
            rhs[2 * i] = ConstrainedLeastSquares.Dot(row.Value, coefficients) * dataScale;
        }
        ScalarFitResult result = ConstrainedSplineFitter.SolveScalar(definition, matrix, rhs, equations, locked, settings.ConstraintTolerance);
        return locked.Count == coefficients.Length && result.Curve is not null
            ? result with { Diagnostics = result.Diagnostics with { Message = "All coefficients are frozen; the original coefficients are retained." } }
            : result;
    }

    private static List<FairingRow> Rows(SplineDefinition definition, SmoothingSettings settings, double a, double b, ArcLengthMap? arcLength)
    {
        double[] spans = definition.Breakpoints.Where(x => x > a && x < b).Prepend(a).Append(b).ToArray();
        int roughnessOrder = settings.RoughnessOrder ?? (arcLength is null ? 2 : 3);
        double normalization = arcLength is null ? b - a : arcLength.StationAt(b) - arcLength.StationAt(a);
        if (!(normalization > 0)) { throw new ArgumentException("Smoothing region has no resolved extent."); }
        var rows = new List<FairingRow>();
        for (int span = 1; span < spans.Length; span++)
        {
            for (int subdivision = 0; subdivision < settings.QuadratureSubdivisions; subdivision++)
            {
                double low = spans[span - 1] + (spans[span] - spans[span - 1]) * subdivision / settings.QuadratureSubdivisions;
                double high = subdivision + 1 == settings.QuadratureSubdivisions ? spans[span]
                    : spans[span - 1] + (spans[span] - spans[span - 1]) * (subdivision + 1) / settings.QuadratureSubdivisions;
                foreach ((double parameter, double weight) in AdaptiveGaussKronrod.WeightedNodes(low, high))
                {
                    double[] value = ConstrainedSplineFitter.Row(definition, parameter, 0, EvaluationSide.Right);
                    double[] roughness = new double[value.Length];
                    double a1 = normalization;
                    double a2 = 0;
                    double a3 = 0;
                    double measure = 1 / normalization;
                    if (arcLength is not null)
                    {
                        CurveJet3d jet = arcLength.Curve.Evaluate(parameter, EvaluationSide.Right);
                        double speed = jet.FirstDerivative.Length;
                        if (!(speed > 0)) { throw new InvalidOperationException("Arc-length fairing requires a regular original curve."); }
                        Vector3d tangent = jet.FirstDerivative / speed;
                        double speedRate = Vector3d.Dot(tangent, jet.SecondDerivative);
                        double speedSecond = Vector3d.Dot((jet.SecondDerivative - tangent * speedRate) / speed, jet.SecondDerivative)
                            + Vector3d.Dot(tangent, jet.ThirdDerivative);
                        a1 = normalization / speed;
                        a2 = -a1 * a1 * (speedRate / speed);
                        a3 = a1 * a1 * a1 * (3 * (speedRate / speed) * (speedRate / speed) - speedSecond / speed);
                        measure = speed / normalization;
                    }
                    double[] d1 = ConstrainedSplineFitter.Row(definition, parameter, 1, EvaluationSide.Right);
                    double[] d2 = ConstrainedSplineFitter.Row(definition, parameter, 2, EvaluationSide.Right);
                    double[] d3 = ConstrainedSplineFitter.Row(definition, parameter, 3, EvaluationSide.Right);
                    for (int j = 0; j < value.Length; j++)
                    {
                        roughness[j] = roughnessOrder switch
                        {
                            1 => a1 * d1[j],
                            2 => a1 * a1 * d2[j] + a2 * d1[j],
                            _ => a1 * a1 * a1 * d3[j] + 3 * a1 * a2 * d2[j] + a3 * d1[j]
                        };
                    }
                    double roughnessWeight = settings.RoughnessWeight?.Invoke(parameter) ?? 1;
                    if (!double.IsFinite(roughnessWeight) || roughnessWeight < 0)
                    { throw new ArgumentException("Custom roughness weights must be finite and nonnegative.", nameof(settings)); }
                    rows.Add(new(parameter, value, roughness, weight * measure / settings.ValueScale / settings.ValueScale, roughnessWeight));
                }
            }
        }
        return rows;
    }

    private static SmoothingMetrics Metrics(List<FairingRow> rows, double[] original, double[]? candidate, int frozen, double deviation)
    {
        double before = 0;
        double after = 0;
        foreach (FairingRow row in rows)
        {
            double a = ConstrainedLeastSquares.Dot(row.Roughness, original);
            double b = candidate is null ? 0 : ConstrainedLeastSquares.Dot(row.Roughness, candidate);
            before += row.Weight * row.RoughnessWeight * a * a;
            after += row.Weight * row.RoughnessWeight * b * b;
        }
        return new(before, after, deviation, frozen, rows.Count);
    }

    private static (double A, double B) Region(SplineDefinition definition, SmoothingSettings settings)
    {
        double a = settings.RegionMinimum ?? definition.Minimum;
        double b = settings.RegionMaximum ?? definition.Maximum;
        Check.Argument(a, definition.Minimum, definition.Maximum, EvaluationSide.Automatic);
        Check.Argument(b, definition.Minimum, definition.Maximum, EvaluationSide.Automatic);
        return (a, b);
    }

    private static int[] Frozen(SplineDefinition definition, double a, double b, IEnumerable<int>? locked)
    {
        int[] supplied = locked?.ToArray() ?? Array.Empty<int>();
        if (supplied.Any(i => i < 0 || i >= definition.ControlPointCount)) { throw new ArgumentOutOfRangeException(nameof(locked)); }
        return Enumerable.Range(0, definition.ControlPointCount)
            .Where(i => (a > definition.Minimum && definition.Knots[i] < a)
                || (b < definition.Maximum && definition.Knots[i + definition.Degree + 1] > b))
            .Concat(supplied).Distinct().Order().ToArray();
    }

    private static double[] ComparisonParameters(SplineDefinition definition, List<FairingRow> rows, double a, double b) =>
        rows.Select(r => r.Parameter).Concat(definition.Breakpoints).Append(a).Append(b).Append(definition.Minimum).Append(definition.Maximum).Distinct().Order().ToArray();

    private static double? Curvature(SplineCurve3d curve, double parameter)
    {
        try { return CurveGeometry.Evaluate(curve, parameter, EvaluationSide.Right).Curvature; }
        catch (InvalidOperationException) { return null; }
    }

    private sealed record FairingRow(double Parameter, double[] Value, double[] Roughness, double Weight, double RoughnessWeight);
}
