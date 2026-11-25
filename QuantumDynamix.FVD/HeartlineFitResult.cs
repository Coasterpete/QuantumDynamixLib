using System;
using System.Linq;
using QuantumDynamix.Geometry;
using QuantumDynamix.Math;
using QuantumDynamix.Math.Functions;
using QuantumDynamix.Splines.Curves;
using QuantumDynamix.Track;

namespace QuantumDynamix.FVD;

/// <summary>Verified fitted track or a diagnostic failure. The numerical source is available in either case.</summary>
/// <remarks>The result and fitted track can reevaluate callbacks from the reconstruction request during queries.
/// Keep custom acceleration, banking, and motion callbacks pure, deterministic, and stable for their full lifetime.
/// See <see cref="HeartlineReconstructionRequest"/>.</remarks>
public sealed class HeartlineFitResult
{
    internal HeartlineFitResult(ReconstructedHeartline source, HeartlineTrack? track, HeartlineFitDiagnostics diagnostics,
        HeartlineFailure? failure)
    {
        Source = source;
        Track = track;
        Diagnostics = diagnostics;
        Failure = failure;
    }

    public ReconstructedHeartline Source { get; }
    public bool Succeeded => Track is not null;
    public HeartlineTrack? Track { get; }
    public HeartlineFitDiagnostics Diagnostics { get; }
    public HeartlineFailure? Failure { get; }

    /// <param name="sourceStationMetres">Local numerical station s = S - S0 in metres from the source start.</param>
    /// <returns>Arc length in metres from the fitted track start.</returns>
    public double GetFittedStation(double sourceStationMetres) => RequireMap().StationAt(sourceStationMetres);

    /// <param name="fittedStationMetres">Fitted arc length in metres from the fitted track start.</param>
    /// <returns>Local numerical station in metres from the source start.</returns>
    public double GetSourceStation(double fittedStationMetres) => RequireMap().Invert(fittedStationMetres).Parameter;

    /// <param name="sourceStationMetres">Local numerical station s = S - S0 in metres from the source start.</param>
    /// <param name="side">Limit used for the original motion and fitted geometry. C1 knots require an explicit side.</param>
    /// <returns>Motion at the same source time, with station expressed in fitted arc-length metres.</returns>
    public LongitudinalMotionSample GetFittedMotion(double sourceStationMetres, EvaluationSide side)
    {
        ArcLengthMap map = RequireMap();
        Check.Argument(sourceStationMetres, 0, Source.LengthMetres, side);
        if (side == EvaluationSide.Automatic && map.Curve.Definition.ContinuityAt(sourceStationMetres) < 2)
        { throw new ArgumentException("Select the left or right limit at a fitted C1 knot.", nameof(side)); }
        return HeartlineFitMapping.TransformMotion(map, Source.Evaluate(sourceStationMetres, side), side, out _);
    }

    private ArcLengthMap RequireMap() => Track?.HeartlineStations
        ?? throw new InvalidOperationException("Fitted station queries require a successful spline conversion.");
}

internal static class HeartlineFitMapping
{
    internal static IScalarFunction Bank(ReconstructedHeartline source, ArcLengthMap map)
    {
        // All authored bank breakpoints are retained as spline knots. The arc-length map
        // returns their exact corresponding parameter on a sided query at a cached node.
        return new CustomFunction(0, map.Length,
            (station, side) => BankAtParameter(source, map.Curve, map.Invert(station).Parameter, side),
            map.Nodes.Skip(1).SkipLast(1).Select(node => node.Station));
    }

    internal static ScalarJet BankAtParameter(ReconstructedHeartline source, SplineCurve3d curve,
        double u, EvaluationSide side, CurveJet3d? geometryJet = null)
    {
        EvaluationSide limit = Inward(u, source.LengthMetres, side);
        ScalarJet original = source.BankAt(u, limit);
        // An interior station can invert to an endpoint, where banking independently uses the inward limit.
        CurveJet3d jet = limit == side && geometryJet.HasValue ? geometryJet.Value : curve.Evaluate(u, limit);
        int order = System.Math.Min(original.DerivativeOrder, jet.DerivativeOrder);
        if (order == 0) { return new(original.Value, derivativeOrder: 0); }
        double h = jet.FirstDerivative.Length;
        double first = original.FirstDerivative / h;
        if (order == 1) { return new(original.Value, first, derivativeOrder: 1); }
        Vector3d tangent = jet.FirstDerivative / h;
        double dh = Vector3d.Dot(tangent, jet.SecondDerivative);
        double second = (original.SecondDerivative / h - first * (dh / h)) / h;
        if (order == 2) { return new(original.Value, first, second, derivativeOrder: 2); }
        Vector3d dT = (jet.SecondDerivative - tangent * dh) / h;
        double ddh = Vector3d.Dot(dT, jet.SecondDerivative) + Vector3d.Dot(tangent, jet.ThirdDerivative);
        double third = (original.ThirdDerivative / h - 3 * second * dh - first * (ddh / h)) / h / h;
        return new(original.Value, first, second, third);
    }

    internal static LongitudinalMotionSample TransformMotion(ArcLengthMap map, ReconstructedHeartlineSample source,
        EvaluationSide side, out double arcLengthError)
    {
        double u = source.StationMetres;
        var (station, mapError) = map.StationAndErrorAt(u);
        arcLengthError = mapError;
        CurveJet3d jet = map.Curve.Evaluate(u, Inward(u, map.Curve.Maximum, side));
        double speedRatio = jet.FirstDerivative.Length;
        double speedRatioDerivative = Vector3d.Dot(jet.FirstDerivative / speedRatio, jet.SecondDerivative);
        LongitudinalMotionSample motion = source.Motion;
        double v = motion.HeartlineVelocityMetresPerSecond;
        double mappedSpeed = speedRatio * v;
        double acceleration = speedRatio * motion.LongitudinalAccelerationMetresPerSecondSquared
            + (speedRatioDerivative * v) * v;
        Check.Finite(mappedSpeed, nameof(mappedSpeed));
        Check.Finite(acceleration, nameof(acceleration));
        TrajectoryErrorEstimate errors = motion.EstimatedAbsoluteError;
        // Include the station correspondence discrepancy as well as the underlying M2 estimate.
        double stationError = System.Math.Max(errors.StationMetres, source.StationCorrespondenceError);
        double timeError = System.Math.Max(errors.ElapsedTimeSeconds, source.TimeCorrespondenceError);
        double velocityError = System.Math.Max(errors.VelocityMetresPerSecond, source.VelocityCorrespondenceError);
        return new(motion.ElapsedSeconds, station, mappedSpeed, acceleration,
            new(timeError, mapError + speedRatio * stationError,
                speedRatio * velocityError + System.Math.Abs(speedRatioDerivative * v) * stationError));
    }

    private static EvaluationSide Inward(double u, double length, EvaluationSide side) =>
        u == 0 ? EvaluationSide.Right : u == length ? EvaluationSide.Left : side;
}
