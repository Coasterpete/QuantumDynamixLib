using System;
using System.Collections.Generic;
using System.Linq;
using QuantumDynamix.Math.Functions;
using QuantumDynamix.Math.Numerics;
using QuantumDynamix.Math.Timelines;

namespace QuantumDynamix.Track;

public readonly record struct TrackDistanceNode(double HeartlineStation, double Station);
public readonly record struct TrackDistanceInversion(double HeartlineStation, double StationResidual, double EstimatedLengthError, int Iterations);

/// <summary>Cumulative distance of one derived spine or rail, parameterized by heartline metres.
/// Quadrature errors exclude transported-frame and reference-station inversion errors. Profiles must remain pure.
/// Regularity is checked at known boundaries and integration nodes, not certified over the continuous domain.</summary>
public sealed class TrackDistanceMap
{
    private readonly RailGeometry geometry;
    private readonly double[] heartStations;
    private readonly double[] stations;
    private readonly double[] errors;
    public string? RailId { get; }
    public bool IsSpine => RailId is null;
    public QuadratureSettings Settings { get; }
    public double Length => stations[^1];
    public QuadratureResult Integration { get; }
    public IReadOnlyList<TrackDistanceNode> Nodes { get; }
    public double EstimatedFrameAngularError => geometry.Track.Frames.EstimatedAccumulatedAngularError;
    public double ReferenceStationErrorBudget => geometry.Track.HeartlineStations.Settings.AbsoluteTolerance
        + geometry.Track.HeartlineStations.Settings.RelativeTolerance * geometry.HeartlineLength;

    internal TrackDistanceMap(RailGeometry geometry, string? railId, QuadratureSettings? settings)
    {
        this.geometry = geometry;
        RailId = railId;
        Settings = settings ?? new();
        heartStations = geometry.Breakpoints.Prepend(0).Append(geometry.HeartlineLength).ToArray();
        stations = new double[heartStations.Length];
        errors = new double[heartStations.Length];
        int evaluations = 0;
        int intervals = 0;
        for (int i = 0; i < heartStations.Length; i++)
        {
            double s = heartStations[i];
            if (i > 0 && i < heartStations.Length - 1)
            {
                TrackOffsetSample left = geometry.EvaluateCenterline(railId, s, EvaluationSide.Left);
                TrackOffsetSample right = geometry.EvaluateCenterline(railId, s, EvaluationSide.Right);
                double gap = ((right.Frame.Position - left.Frame.Position) + (right.Displacement - left.Displacement)).Length;
                if (gap > geometry.Settings.PositionTolerance)
                { throw Failure(RailDiagnosticCode.PositionDiscontinuity, s, "A position jump prevents a continuous distance map (metres).", gap); }
            }
            if (i > 0) { Speed(s, EvaluationSide.Left); }
            if (i < heartStations.Length - 1) { Speed(s, EvaluationSide.Right); }
            if (i == 0) { continue; }
            QuadratureResult result = Integrate(heartStations[i - 1], s);
            stations[i] = stations[i - 1] + result.Value;
            errors[i] = errors[i - 1] + result.EstimatedAbsoluteError;
            if (!double.IsFinite(stations[i]) || !(stations[i] > stations[i - 1]))
            { throw Failure(RailDiagnosticCode.SingularCenterline, s, "A zero-length or unresolved distance interval is not invertible."); }
            evaluations = checked(evaluations + result.Evaluations);
            intervals = checked(intervals + result.Subintervals);
        }
        Integration = new(Length, errors[^1], evaluations, intervals);
        Nodes = Array.AsReadOnly(heartStations.Select((s, i) => new TrackDistanceNode(s, stations[i])).ToArray());
    }

    public double StationAtHeartlineStation(double heartlineStation)
    {
        Check.Argument(heartlineStation, 0, geometry.HeartlineLength, EvaluationSide.Automatic);
        int exact = Array.BinarySearch(heartStations, heartlineStation);
        if (exact >= 0) { return stations[exact]; }
        int lower = ~exact - 1;
        return stations[lower] + Integrate(heartStations[lower], heartlineStation).Value;
    }

    public TrackDistanceInversion Invert(double station, int maximumIterations = 64)
    {
        Check.Argument(station, 0, Length, EvaluationSide.Automatic);
        if (maximumIterations < 1) { throw new ArgumentOutOfRangeException(nameof(maximumIterations)); }
        double tolerance = Settings.AbsoluteTolerance + Settings.RelativeTolerance * Length;
        int exact = Array.BinarySearch(stations, station);
        if (exact >= 0) { return new(heartStations[exact], 0, errors[exact], 0); }
        int lower = ~exact - 1;
        double low = heartStations[lower];
        double high = heartStations[lower + 1];
        double s = low + (high - low) * ((station - stations[lower]) / (stations[lower + 1] - stations[lower]));
        double residual = double.PositiveInfinity;
        double error = errors[lower];
        for (int iteration = 1; iteration <= maximumIterations; iteration++)
        {
            QuadratureResult partial = Integrate(heartStations[lower], s);
            residual = stations[lower] + partial.Value - station;
            error = errors[lower] + partial.EstimatedAbsoluteError;
            if (System.Math.Abs(residual) + error <= tolerance) { return new(s, residual, error, iteration); }
            if (residual > 0) { high = s; } else { low = s; }
            double next = s - residual / Speed(s, EvaluationSide.Right);
            if (!double.IsFinite(next) || next <= low || next >= high) { next = low + (high - low) / 2; }
            if (!(next > low && next < high)) { break; }
            s = next;
        }
        throw new NumericalConvergenceException("Derived distance inversion did not meet its station error budget (metres).", maximumIterations, System.Math.Abs(residual) + error);
    }

    public FeatureStations ResolveFeatures() => new(geometry.Track.ResolveFeatures().Stations.Select(f =>
        new KeyValuePair<string, double>(f.Key, StationAtHeartlineStation(f.Value))));

    private QuadratureResult Integrate(double a, double b)
    {
        if (a == b) { return new(0, 0, 0, 0); }
        var local = new QuadratureSettings(System.Math.Max(double.Epsilon, Settings.AbsoluteTolerance * ((b - a) / geometry.HeartlineLength) / 4),
            Settings.RelativeTolerance / 4, Settings.MaximumDepth, Settings.MaximumEvaluations);
        return AdaptiveGaussKronrod.Integrate(s => Speed(s, EvaluationSide.Right), a, b, local);
    }

    private double Speed(double s, EvaluationSide side)
    {
        var sample = geometry.EvaluateCenterline(RailId, s, side).Curve;
        if (sample.DerivativeOrder < 1)
        { throw Failure(RailDiagnosticCode.UnavailableDerivative, s, "Derived distance needs a first centerline derivative."); }
        double speed = sample.FirstDerivative.Length;
        if (!(speed > geometry.Settings.MinimumSpeedRatio))
        { throw Failure(RailDiagnosticCode.SingularCenterline, s, "Distance-map speed is below the supplied regularity floor (dimensionless).", speed); }
        return speed;
    }

    private RailGeometryException Failure(RailDiagnosticCode code, double station, string message, double? residual = null) =>
        new(new(code, RailId, station, message, residual));
}
