using System;
using System.Collections.Generic;
using System.Linq;
using QuantumDynamix.Math.Functions;
using QuantumDynamix.Math.Numerics;
using QuantumDynamix.Splines.Curves;

namespace QuantumDynamix.Geometry;

public readonly record struct ArcLengthNode(double Parameter, double Station);
public readonly record struct ArcLengthInversion(double Parameter, double StationResidual, double EstimatedLengthError, int Iterations);

/// <summary>Immutable cumulative arc-length map. Zero-length spans and possible value jumps are rejected.
/// Cached values and partial integrals share the requested absolute/relative length budget.</summary>
public sealed class ArcLengthMap
{
    private readonly double[] parameters;
    private readonly double[] stations;
    private readonly double[] errors;
    public SplineCurve3d Curve { get; }
    public QuadratureSettings Settings { get; }
    public double Length => stations[^1];
    public QuadratureResult Integration { get; }
    public IReadOnlyList<ArcLengthNode> Nodes { get; }

    public ArcLengthMap(SplineCurve3d curve, QuadratureSettings? settings = null)
    {
        ArgumentNullException.ThrowIfNull(curve);
        Curve = curve;
        Settings = settings ?? new();
        if (curve.Definition.Breakpoints.Any(x => curve.Definition.ContinuityAt(x) < 0))
        { throw new ArgumentException("Arc length requires a continuous curve definition.", nameof(curve)); }
        parameters = curve.Definition.Breakpoints.Prepend(curve.Minimum).Append(curve.Maximum).ToArray();
        stations = new double[parameters.Length];
        errors = new double[parameters.Length];
        int evaluations = 0;
        int subintervals = 0;
        for (int i = 1; i < parameters.Length; i++)
        {
            double a = parameters[i - 1];
            double b = parameters[i];
            QuadratureResult result = IntegrateSpan(a, b);
            stations[i] = stations[i - 1] + result.Value;
            errors[i] = errors[i - 1] + result.EstimatedAbsoluteError;
            if (!double.IsFinite(stations[i]) || !(stations[i] > stations[i - 1]))
            {
                throw new ArgumentException(FormattableString.Invariant(
                    $"Arc-length integration cannot advance the cumulative station over spline parameters [{a:R}, {b:R}]. The integrated span length is {result.Value:R} m after {result.Subintervals} quadrature subintervals."),
                    nameof(curve));
            }
            evaluations = checked(evaluations + result.Evaluations);
            subintervals = checked(subintervals + result.Subintervals);
        }
        Integration = new(Length, errors[^1], evaluations, subintervals);
        Nodes = Array.AsReadOnly(parameters.Select((p, i) => new ArcLengthNode(p, stations[i])).ToArray());
    }

    public double StationAt(double parameter)
    {
        Check.Argument(parameter, Curve.Minimum, Curve.Maximum, EvaluationSide.Automatic);
        int exact = Array.BinarySearch(parameters, parameter);
        if (exact >= 0) { return stations[exact]; }
        int lower = ~exact - 1;
        return stations[lower] + IntegrateSpan(parameters[lower], parameter).Value;
    }

    public ArcLengthInversion Invert(double station, int maximumIterations = 64)
    {
        Check.Argument(station, 0, Length, EvaluationSide.Automatic);
        if (maximumIterations < 1) { throw new ArgumentOutOfRangeException(nameof(maximumIterations)); }
        double tolerance = Settings.AbsoluteTolerance + Settings.RelativeTolerance * Length;
        int exact = Array.BinarySearch(stations, station);
        if (exact >= 0) { return new(parameters[exact], 0, errors[exact], 0); }
        int index = ~exact - 1;
        double low = parameters[index];
        double high = parameters[index + 1];
        double parameter = low + (high - low) * ((station - stations[index]) / (stations[index + 1] - stations[index]));
        double residual = double.PositiveInfinity;
        double error = errors[index];
        int iterations = 0;
        for (int iteration = 1; iteration <= maximumIterations; iteration++)
        {
            iterations = iteration;
            QuadratureResult partial = IntegrateSpan(parameters[index], parameter);
            residual = stations[index] + partial.Value - station;
            error = errors[index] + partial.EstimatedAbsoluteError;
            if (System.Math.Abs(residual) + error <= tolerance) { return new(parameter, residual, error, iteration); }
            if (residual > 0) { high = parameter; } else { low = parameter; }
            double speed = Curve.Evaluate(parameter, EvaluationSide.Right).FirstDerivative.Length;
            double next = speed > 0 ? parameter - residual / speed : double.NaN;
            if (!double.IsFinite(next) || next <= low || next >= high) { next = low + (high - low) / 2; }
            if (!(next > low && next < high)) { break; }
            parameter = next;
        }
        double combinedError = System.Math.Abs(residual) + error;
        throw new NumericalConvergenceException(FormattableString.Invariant(
            $"Arc-length inversion for station {station:R} m failed after {iterations} iterations. The spline parameter bracket is [{low:R}, {high:R}]. The absolute station residual plus estimated length error is {combinedError:R} m against a limit of {tolerance:R} m."),
            iterations, combinedError)
        {
            ParameterInterval = (low, high),
            ResidualBudget = ("Absolute station residual plus arc-length error estimate", "m", tolerance)
        };
    }

    private QuadratureResult IntegrateSpan(double a, double b)
    {
        if (a == b) { return new(0, 0, 0, 0); }
        double fraction = (b - a) / (Curve.Maximum - Curve.Minimum);
        var local = new QuadratureSettings(System.Math.Max(double.Epsilon, Settings.AbsoluteTolerance * fraction / 4),
            Settings.RelativeTolerance / 4, Settings.MaximumDepth, Settings.MaximumEvaluations);
        try { return AdaptiveGaussKronrod.Integrate(x => Curve.Evaluate(x, EvaluationSide.Right).FirstDerivative.Length, a, b, local); }
        catch (NumericalConvergenceException ex)
        {
            double? tolerance = ex.ResidualBudget?.Limit;
            string limit = tolerance is double value ? FormattableString.Invariant($" against a limit of {value:R} m") : "";
            throw new NumericalConvergenceException(FormattableString.Invariant(
                $"Arc-length integration failed with an estimated length error of {ex.Residual:R} m{limit}. {ex.Message}"),
                ex.Iterations, ex.Residual, ex)
            {
                ParameterInterval = (a, b),
                ResidualBudget = tolerance.HasValue ? ("Arc-length quadrature error estimate", "m", tolerance.Value) : null
            };
        }
        catch (ArithmeticException ex)
        {
            throw new ArithmeticException(FormattableString.Invariant(
                $"Arc-length integration failed over spline parameters [{a:R}, {b:R}]. {ex.Message}"), ex);
        }
    }
}
