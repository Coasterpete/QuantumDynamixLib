using System;
using System.Collections.Generic;
using QuantumDynamix.Math;
using QuantumDynamix.Math.Functions;
using QuantumDynamix.Splines.Fitting;
using QuantumDynamix.Track;

namespace QuantumDynamix.FVD;

internal sealed class HeartlineFitVerification(ReconstructedHeartline source, HeartlineFitSettings settings,
    HeartlineTrack track, double[] boundaries)
{
    private readonly SortedSet<int> failingSpans = new();
    private double position;
    private double tangent;
    private double curvature;
    private double drift;
    private Vector3d acceleration;
    private (double Error, double Station, EvaluationSide Side)? frame;
    private (double Error, double Station, EvaluationSide Side)? roll;
    private double worstRatio = 1;

    internal HeartlineFailure? WorstFailure { get; private set; }
    internal IReadOnlyCollection<int> FailingSpans => failingSpans;
    internal HeartlineFailure? IncompleteVerification { get; private set; }

    internal bool Verify(ref int samples)
    {
        for (int span = 0; span + 1 < boundaries.Length; span++)
        {
            double a = boundaries[span];
            double b = boundaries[span + 1];
            int remaining = settings.MaximumVerificationSamples - samples;
            double cells = System.Math.Max(3, System.Math.Ceiling((b - a) / settings.MaximumVerificationSpacingMetres));
            if (cells > remaining - 2)
            {
                IncompleteVerification = HeartlineSplineConversion.Failure(HeartlineFailureCode.NumericalBudgetExceeded,
                    "The verification-sample limit cannot cover the fitted track.", (a, b),
                    measurement: ("Samples required through this span", "samples", samples + cells + 2, settings.MaximumVerificationSamples));
                return false;
            }
            Measure(a, EvaluationSide.Right, span);
            Measure(b, EvaluationSide.Left, span);
            samples += 2;

            double previous = a;
            for (int i = 0; i < (int)cells; i++)
            {
                // Offset the verification grid from the rational fractions used by the fit.
                double s = a + (b - a) * ((i + 0.3819660112501051) / cells);
                if (!(s > previous && s < b))
                {
                    IncompleteVerification = HeartlineSplineConversion.Failure(HeartlineFailureCode.NumericalBudgetExceeded,
                        "Independent verification stations cannot be resolved in this fitted span.", (a, b));
                    return false;
                }
                Measure(s, EvaluationSide.Right, span);
                samples++;
                previous = s;
            }
        }
        return true;
    }

    private void Measure(double s, EvaluationSide side, int span)
    {
        ReconstructedHeartlineSample expected = source.Evaluate(s, side);
        LongitudinalMotionSample motion = HeartlineFitMapping.TransformMotion(track.HeartlineStations, expected, side, out double lengthError);
        HeartlineForceSample actual = HeartlineForceEvaluator.EvaluateFitted(track, source, motion, side);
        Vector3d world = actual.WorldCoordinateAcceleration;
        Vector3d forward = actual.BankedFrame.Forward;
        Vector3d observedCurvature = (world - forward * motion.LongitudinalAccelerationMetresPerSecondSquared)
            / actual.HeartlineSpeed / actual.HeartlineSpeed;
        Vector3d observedAcceleration = new(Vector3d.Dot(world, forward), Vector3d.Dot(world, actual.BankedFrame.Left),
            Vector3d.Dot(world, actual.BankedFrame.Up));
        Vector3d target = new(expected.Motion.LongitudinalAccelerationMetresPerSecondSquared,
            expected.LateralCoordinateAccelerationMetresPerSecondSquared,
            expected.VerticalCoordinateAccelerationMetresPerSecondSquared);
        var (angle, frameAngle, signedRoll) = HeartlineFrameMath.Compare(expected.BankedFrame, actual.BankedFrame);

        position = Maximum(position, (actual.BankedFrame.Position - expected.BankedFrame.Position).Length,
            settings.PositionToleranceMetres, "Position error", "m");
        tangent = Maximum(tangent, angle, settings.TangentToleranceRadians, "Tangent error", "rad");
        curvature = Maximum(curvature, (observedCurvature - expected.CurvatureVectorPerMetre!.Value).Length,
            settings.CurvatureTolerancePerMetre, "Curvature error", "1/m");
        double stationDrift = System.Math.Abs(motion.HeartlineStationMetres - s);
        drift = System.Math.Max(drift, stationDrift);
        MeasureError(stationDrift + lengthError, settings.StationDriftToleranceMetres, "Station drift plus arc-length error estimate", "m");

        MeasureError(frameAngle, settings.BankedFrameToleranceRadians, "Banked-frame error", "rad");
        if (frame is null || frameAngle > frame.Value.Error) { frame = (frameAngle, s, side); }
        if (signedRoll is double r)
        {
            MeasureError(System.Math.Abs(r), settings.BankRollToleranceRadians, "Bank roll error", "rad");
            if (roll is null || System.Math.Abs(r) > System.Math.Abs(roll.Value.Error)) { roll = (r, s, side); }
        }

        Vector3d delta = observedAcceleration - target;
        Vector3d tolerances = settings.CoordinateAccelerationTolerancesMetresPerSecondSquared;
        acceleration = new(
            Maximum(acceleration.X, System.Math.Abs(delta.X), tolerances.X, "Forward acceleration error", "m/s²"),
            Maximum(acceleration.Y, System.Math.Abs(delta.Y), tolerances.Y, "Lateral acceleration error", "m/s²"),
            Maximum(acceleration.Z, System.Math.Abs(delta.Z), tolerances.Z, "Vertical acceleration error", "m/s²"));

        double Maximum(double prior, double value, double limit, string quantity, string unit)
        {
            MeasureError(value, limit, quantity, unit);
            return System.Math.Max(prior, value);
        }

        void MeasureError(double value, double limit, string quantity, string unit)
        {
            if (!(value > limit)) { return; }
            failingSpans.Add(span);
            double ratio = value / limit;
            if (ratio <= worstRatio) { return; }
            worstRatio = ratio;
            WorstFailure = HeartlineSplineConversion.Failure(HeartlineFailureCode.FitToleranceExceeded,
                $"{quantity} exceeds the fit tolerance at the source station and side.", (s, s), side,
                (quantity, unit, value, limit));
        }
    }

    internal HeartlineFitDiagnostics Diagnostics(FitDiagnostics? solver, int passes, int samples) => new()
    {
        Solver = solver,
        RefinementPasses = passes,
        VerificationSamples = samples,
        MaximumPositionErrorMetres = position,
        MaximumTangentErrorRadians = tangent,
        MaximumCurvatureErrorPerMetre = curvature,
        MaximumStationDriftMetres = drift,
        LengthErrorMetres = System.Math.Abs(track.HeartlineStations.Length - source.LengthMetres),
        MaximumCoordinateAccelerationErrorsMetresPerSecondSquared = acceleration,
        WorstBankedFrameError = frame,
        WorstBankRollError = roll
    };
}
