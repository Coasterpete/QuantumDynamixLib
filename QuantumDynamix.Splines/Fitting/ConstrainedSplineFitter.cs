using System;
using System.Collections.Generic;
using System.Linq;
using QuantumDynamix.Math;
using QuantumDynamix.Math.Functions;
using QuantumDynamix.Math.Numerics;
using QuantumDynamix.Splines.Curves;

namespace QuantumDynamix.Splines.Fitting;

public enum FitStatus { Success, Infeasible, RankDeficient, NumericalFailure }
public sealed record FitDiagnostics(FitStatus Status, string Message, int DataRank, int ConstraintRank,
    double WeightedResidualNorm, double MaximumRelativeConstraintResidual);
public readonly record struct ScalarFitSample(double Parameter, double Value, double Weight = 1);
public readonly record struct CurveFitSample(double Parameter, Vector3d Position, double Weight = 1);
public readonly record struct ScalarFitConstraint(double Parameter, int DerivativeOrder, double Value, EvaluationSide Side = EvaluationSide.Automatic);
public readonly record struct CurveFitConstraint(double Parameter, int DerivativeOrder, Vector3d Value, EvaluationSide Side = EvaluationSide.Automatic);
public sealed record ScalarFitResult(ScalarSpline? Curve, FitDiagnostics Diagnostics);
public sealed record CurveFitResult(SplineCurve3d? Curve, FitDiagnostics Diagnostics);

/// <summary>Fixed-knot, fixed-weight fitting. Weights multiply squared residuals, not residuals themselves.</summary>
public static class ConstrainedSplineFitter
{
    public static ScalarFitResult FitScalar(SplineDefinition definition, IEnumerable<ScalarFitSample> samples,
        IEnumerable<ScalarFitConstraint>? constraints = null, double constraintTolerance = 1e-9)
    {
        ArgumentNullException.ThrowIfNull(definition);
        ArgumentNullException.ThrowIfNull(samples);
        ValidateTolerance(constraintTolerance);
        ScalarFitSample[] data = samples.ToArray();
        double[,] matrix = new double[data.Length, definition.ControlPointCount];
        double[] rhs = new double[data.Length];
        for (int i = 0; i < data.Length; i++)
        {
            Check.Finite(data[i].Value, nameof(samples));
            if (!double.IsFinite(data[i].Weight) || data[i].Weight <= 0) { throw new ArgumentOutOfRangeException(nameof(samples), "Sample weights must be positive and finite."); }
            double[] row = Row(definition, data[i].Parameter, 0, EvaluationSide.Automatic);
            double weight = System.Math.Sqrt(data[i].Weight);
            for (int j = 0; j < row.Length; j++) { matrix[i, j] = weight * row[j]; }
            rhs[i] = weight * data[i].Value;
        }
        List<LinearEquation> equations = Equations(definition, constraints ?? Array.Empty<ScalarFitConstraint>());
        return SolveScalar(definition, matrix, rhs, equations, null, constraintTolerance);
    }

    public static CurveFitResult FitCurve(SplineDefinition definition, IEnumerable<CurveFitSample> samples,
        IEnumerable<CurveFitConstraint>? constraints = null, double constraintTolerance = 1e-9)
    {
        ArgumentNullException.ThrowIfNull(definition);
        ArgumentNullException.ThrowIfNull(samples);
        CurveFitSample[] data = samples.ToArray();
        CurveFitConstraint[] conditions = constraints?.ToArray() ?? Array.Empty<CurveFitConstraint>();
        ScalarFitResult[] results = Enumerable.Range(0, 3).Select(axis => FitScalar(definition,
            data.Select(s => new ScalarFitSample(s.Parameter, Component(s.Position, axis), s.Weight)),
            conditions.Select(c => new ScalarFitConstraint(c.Parameter, c.DerivativeOrder, Component(c.Value, axis), c.Side)), constraintTolerance)).ToArray();
        ScalarFitResult? failure = results.FirstOrDefault(r => r.Curve is null);
        if (failure is not null) { return new(null, failure.Diagnostics); }
        Vector3d[] points = Enumerable.Range(0, definition.ControlPointCount).Select(i => new Vector3d(
            results[0].Curve!.Coefficients[i], results[1].Curve!.Coefficients[i], results[2].Curve!.Coefficients[i])).ToArray();
        return new(new(definition, points), Combine(results.Select(r => r.Diagnostics)));
    }

    internal static ScalarFitResult SolveScalar(SplineDefinition definition, double[,] matrix, double[] rhs,
        IReadOnlyList<LinearEquation> equations, IReadOnlyDictionary<int, double>? locked, double tolerance)
    {
        try
        {
            LeastSquaresSolution solution = ConstrainedLeastSquares.Solve(matrix, rhs, equations, locked, tolerance);
            FitStatus status = solution.Values is not null ? FitStatus.Success : solution.Infeasible ? FitStatus.Infeasible : FitStatus.RankDeficient;
            var diagnostics = new FitDiagnostics(status, solution.Message, solution.Rank, solution.ConstraintRank, solution.Residual, solution.ConstraintResidual);
            return new(solution.Values is null ? null : new ScalarSpline(definition, solution.Values), diagnostics);
        }
        catch (ArithmeticException error)
        { return new(null, new(FitStatus.NumericalFailure, error.Message, 0, 0, double.PositiveInfinity, double.PositiveInfinity)); }
    }

    internal static List<LinearEquation> Equations(SplineDefinition definition, IEnumerable<ScalarFitConstraint> constraints)
    {
        var result = new List<LinearEquation>();
        foreach (ScalarFitConstraint constraint in constraints)
        {
            Check.Finite(constraint.Value, nameof(constraints));
            result.Add(new(Row(definition, constraint.Parameter, constraint.DerivativeOrder, constraint.Side), constraint.Value));
        }
        if (definition.IsPeriodic)
        {
            for (int order = 0; order < definition.Degree; order++)
            {
                double[] a = Row(definition, definition.Minimum, order, EvaluationSide.Right);
                double[] b = Row(definition, definition.Maximum, order, EvaluationSide.Left);
                result.Add(new(a.Zip(b, (x, y) => x - y).ToArray(), 0));
            }
        }
        return result;
    }

    internal static double[] Row(SplineDefinition definition, double parameter, int order, EvaluationSide side)
    {
        if (order < 0 || order > definition.Degree + 3) { throw new ArgumentOutOfRangeException(nameof(order)); }
        if (side == EvaluationSide.Automatic && definition.ContinuityAt(parameter) < order)
        { throw new ArgumentException("A derivative constraint at a nonsmooth knot requires a side.", nameof(side)); }
        SplineBasisSample basis = definition.EvaluateBasis(parameter, order, side);
        double[] row = new double[definition.ControlPointCount];
        for (int j = 0; j < basis.Count; j++) { row[basis.FirstControlPoint + j] = basis[order, j]; }
        return row;
    }

    internal static double Component(Vector3d value, int axis) => axis switch { 0 => value.X, 1 => value.Y, _ => value.Z };
    internal static FitDiagnostics Combine(IEnumerable<FitDiagnostics> diagnostics)
    {
        FitDiagnostics[] all = diagnostics.ToArray();
        return new(FitStatus.Success, "All coordinate fits satisfy the constraints.", all.Min(d => d.DataRank), all.Min(d => d.ConstraintRank),
            ConstrainedLeastSquares.Norm(all.Select(d => d.WeightedResidualNorm)), all.Max(d => d.MaximumRelativeConstraintResidual));
    }
    internal static void ValidateTolerance(double tolerance)
    {
        if (!double.IsFinite(tolerance) || tolerance <= 0) { throw new ArgumentOutOfRangeException(nameof(tolerance)); }
    }
}
