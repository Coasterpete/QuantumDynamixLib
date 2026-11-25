using System;
using System.Collections.Generic;
using System.Linq;
using QuantumDynamix.Geometry;
using QuantumDynamix.Math;
using QuantumDynamix.Math.Functions;

namespace QuantumDynamix.FVD;

internal sealed class HeartlineReconstructionIntegrator(HeartlineReconstructionInputs inputs,
    HeartlineReconstructionSettings settings, ReconstructionWork work)
{
    private readonly List<HeartlineIntegratedPiece> pieces = new();
    private double positionError;
    private double angularError;
    private double curvatureResidual;
    private double rollResidual;

    internal ReconstructedHeartline Integrate(RigidPose3d initial, bool fromEnd)
    {
        double length = inputs.Motion.Length;
        double angularBudget = System.Math.Min(settings.FrameAngleToleranceRadians, settings.PositionToleranceMetres / length / 4);
        Quaterniond rotation = initial.Orientation;
        Vector3d offset = default;
        double maximumOutputRounding = 0;
        double rotationRoundoff = 16 * (System.Math.BitIncrement(1.0) - 1);
        int direction = fromEnd ? -1 : 1;
        EvaluationSide startSide = fromEnd ? EvaluationSide.Left : EvaluationSide.Right;
        EvaluationSide endSide = fromEnd ? EvaluationSide.Right : EvaluationSide.Left;
        double station = fromEnd ? length : 0;
        double step = settings.MaximumStepMetres;
        HeartlineMotionMap.MotionQuery? retryStartMotion = null;

        IEnumerable<double> boundaries = fromEnd ? inputs.Boundaries.Reverse() : inputs.Boundaries;
        foreach (double boundary in boundaries.Skip(1))
        {
            while (direction * (boundary - station) > 0)
            {
                work.Charge(station);
                double remaining = direction * (boundary - station);
                // Share a final interval between two trials when a full step would leave a short tail.
                double end = remaining <= step ? boundary
                    : station + direction * (remaining / 2 <= step ? remaining / 2 : step);
                if (direction * (end - station) > settings.MaximumStepMetres)
                { end = fromEnd ? System.Math.BitIncrement(end) : System.Math.BitDecrement(end); }
                double width = direction * (end - station);
                double mid = station + (end - station) / 2;
                double quarter = station + (mid - station) / 2;
                double thirdQuarter = mid + (end - mid) / 2;
                double fraction = width / length;
                double quadratureBudget = settings.PositionToleranceMetres * fraction / 32;
                if (!(direction * (quarter - station) > 0 && direction * (mid - quarter) > 0
                    && direction * (thirdQuarter - mid) > 0 && direction * (end - thirdQuarter) > 0)
                    || !(quadratureBudget > 0 && angularBudget * fraction > 0))
                {
                    throw work.Fail(HeartlineFailureCode.NumericalBudgetExceeded,
                        "The integration step or its local tolerance is no longer representable.", (station, end));
                }

                double localAngle = double.PositiveInfinity;
                double localPosition = double.PositiveInfinity;
                double localCurvature = double.PositiveInfinity;
                double localRoll = double.PositiveInfinity;
                try
                {
                    var start = new Stage(station, startSide, inputs.Target(station, startSide, work, ref retryStartMotion));
                    Stage quarterStage = Sample(quarter);
                    Stage middle = Sample(mid);
                    Stage thirdQuarterStage = Sample(thirdQuarter);
                    Stage finish = Sample(end, endSide);
                    Vector3d a = Omega(start, rotation);
                    Vector3d b = Omega(quarterStage, rotation);
                    Vector3d c = Omega(middle, rotation);
                    Vector3d d = Omega(thirdQuarterStage, rotation);
                    Vector3d e = Omega(finish, rotation);
                    double rate = System.Math.Max(System.Math.Max(a.Length, b.Length),
                        System.Math.Max(c.Length, System.Math.Max(d.Length, e.Length)));
                    if (width * rate <= 1)
                    {
                        Quaterniond coarseEnd = IntegrateRotation(rotation, start, middle, finish, a, c, e);
                        Quaterniond firstEnd = IntegrateRotation(rotation, start, quarterStage, middle, a, b, c);
                        Quaterniond midpointRotation = (rotation * firstEnd).Normalized();
                        Vector3d midpointRate = inputs.RequiresOrientation ? Omega(middle, midpointRotation) : c;
                        Quaterniond secondEnd = IntegrateRotation(midpointRotation, middle, thirdQuarterStage, finish,
                            midpointRate, d, e);
                        var coarse = new HeartlineRotationPiece(station, end, rotation, coarseEnd, a,
                            inputs.RequiresOrientation ? Omega(finish, (rotation * coarseEnd).Normalized()) : e);
                        var first = new HeartlineRotationPiece(station, mid, rotation, firstEnd, a, midpointRate);
                        var second = new HeartlineRotationPiece(mid, end, first.EndRotation, secondEnd, midpointRate,
                            inputs.RequiresOrientation ? Omega(finish, (first.EndRotation * secondEnd).Normalized()) : e);
                        // Compare in the step's relative basis so a short step is not limited
                        // by cancellation between two nearly equal world orientations.
                        localAngle = System.Math.Max(HeartlineFrameMath.PrincipalAngle(coarseEnd, firstEnd * secondEnd),
                            HeartlineFrameMath.PrincipalAngle(coarse.RelativeOrientation(mid), firstEnd));
                        if (localAngle <= angularBudget * fraction)
                        {
                            Probe(first, second, out localCurvature, out localRoll);
                            if (localCurvature <= settings.CurvatureTolerancePerMetre
                                && localRoll <= settings.UnbankedRollRateToleranceRadiansPerMetre)
                            {
                                var (dx, ex) = first.Displacement(mid, quadratureBudget, work);
                                var (dy, ey) = second.Displacement(end, quadratureBudget, work);
                                var (whole, wholeError) = coarse.Displacement(end, quadratureBudget, work);
                                var (coarseHalf, halfError) = coarse.Displacement(mid, quadratureBudget, work);
                                localPosition = System.Math.Max((dx + dy - whole).Length + wholeError,
                                    (dx - coarseHalf).Length + halfError) + ex + ey;
                                if (localPosition <= settings.PositionToleranceMetres * fraction / 2)
                                {
                                    Vector3d midOffset = offset + dx;
                                    Vector3d endOffset = midOffset + dy;
                                    double angleIncrement = localAngle + (rate == 0 && coarseEnd == Quaterniond.Identity ? 0 : rotationRoundoff);
                                    positionError += localPosition + width * (angularError + angleIncrement)
                                        + ((midOffset - offset) - dx).Length + ((endOffset - midOffset) - dy).Length;
                                    angularError += angleIncrement;
                                    maximumOutputRounding = System.Math.Max(maximumOutputRounding,
                                        System.Math.Max(OutputRounding(midOffset), OutputRounding(endOffset)));
                                    if (!(angularError <= settings.FrameAngleToleranceRadians))
                                    { throw work.Fail(HeartlineFailureCode.NumericalBudgetExceeded,
                                        "Accumulated orientation error exceeds the frame tolerance.", (station, end),
                                        measurement: ("Frame error estimate", "rad", angularError, settings.FrameAngleToleranceRadians)); }
                                    if (!(positionError + maximumOutputRounding <= settings.PositionToleranceMetres))
                                    {
                                        throw work.Fail(HeartlineFailureCode.NumericalBudgetExceeded,
                                            "Accumulated drift or world-position roundoff exceeds the reconstruction tolerance.", (station, end),
                                            measurement: ("Position error estimate", "m", positionError + maximumOutputRounding, settings.PositionToleranceMetres));
                                    }
                                    curvatureResidual = System.Math.Max(curvatureResidual, localCurvature);
                                    rollResidual = System.Math.Max(rollResidual, localRoll);
                                    pieces.Add(new(first, offset, midOffset, quadratureBudget));
                                    pieces.Add(new(second, midOffset, endOffset, quadratureBudget));
                                    offset = endOffset;
                                    rotation = second.EndRotation;
                                    station = end;
                                    retryStartMotion = null;
                                    step = System.Math.Min(settings.MaximumStepMetres, System.Math.Max(step, 2 * width));
                                    continue;
                                }
                            }
                        }
                    }
                }
                catch (Exception ex) when (ex is ArithmeticException or ArgumentOutOfRangeException)
                {
                    throw work.Fail(HeartlineFailureCode.NumericalRangeExceeded, "Frame or position arithmetic exceeds finite range.",
                        (station, end), inner: ex);
                }

                if (work.Remaining == 0)
                {
                    var metric = localAngle > angularBudget * fraction ? ("Frame error estimate", "rad", localAngle, angularBudget * fraction)
                        : localCurvature > settings.CurvatureTolerancePerMetre ? ("Curvature residual", "1/m", localCurvature, settings.CurvatureTolerancePerMetre)
                        : localRoll > settings.UnbankedRollRateToleranceRadiansPerMetre
                            ? ("Unbanked roll rate", "rad/m", localRoll, settings.UnbankedRollRateToleranceRadiansPerMetre)
                            : ("Position error estimate", "m", localPosition, settings.PositionToleranceMetres * fraction / 2);
                    throw work.Fail(HeartlineFailureCode.NumericalBudgetExceeded,
                        "The integration work limit was exhausted before meeting the step tolerance.", (station, end), measurement: metric);
                }
                step = width / 2;
            }
        }

        if (fromEnd) { pieces.Reverse(); }
        return new(inputs, settings, initial.Position, positionError, pieces.ToArray(), new HeartlineReconstructionDiagnostics
        {
            ProvenForwardSpeedLowerBoundMetresPerSecond = inputs.Motion.LowerSpeed,
            EstimatedPositionErrorMetres = positionError + maximumOutputRounding,
            EstimatedFrameErrorRadians = angularError,
            MaximumObservedCurvatureResidualPerMetre = curvatureResidual,
            MaximumObservedUnbankedRollRateRadiansPerMetre = rollResidual,
            MaximumEstimatedStationMappingErrorMetres = work.StationError,
            MaximumEstimatedTimeMappingErrorSeconds = work.TimeError,
            WorkUsed = work.Used,
            AccelerationJoins = inputs.AccelerationJoins,
            BankJoins = inputs.BankJoins
        });

        double OutputRounding(Vector3d value) => (((initial.Position + value) - initial.Position) - value).Length;
    }

    private Stage Sample(double station, EvaluationSide side = EvaluationSide.Right) =>
        new(station, side, inputs.Target(station, side, work));

    private Vector3d Omega(Stage stage, Quaterniond orientation) => inputs.RequiresOrientation
        ? inputs.ResolveOrientedTarget(stage.Target, orientation, stage.Station, stage.Side, work).AngularVelocity
        : stage.Target.AngularVelocity;

    private void Probe(HeartlineRotationPiece first, HeartlineRotationPiece second, out double curvature, out double roll)
    {
        double curvatureMaximum = 0;
        double rollMaximum = 0;
        ProbePiece(first);
        ProbePiece(second);
        curvature = curvatureMaximum;
        roll = rollMaximum;

        void ProbePiece(HeartlineRotationPiece piece)
        {
            // Interior probes offset from the Runge–Kutta stages as well as at the half-piece.
            ReadOnlySpan<double> fractions = [0.21132486540518713, 0.5, 0.7886751345948129];
            foreach (double fraction in fractions)
            {
                double station = piece.Start + (piece.End - piece.Start) * fraction;
                Vector3d dense = piece.AngularRate(station);
                Vector3d target = Omega(Sample(station), inputs.RequiresOrientation ? piece.Orientation(station) : default);
                curvatureMaximum = System.Math.Max(curvatureMaximum, new Vector3d(0, dense.Y - target.Y, dense.Z - target.Z).Length);
                rollMaximum = System.Math.Max(rollMaximum, System.Math.Abs(dense.X));
            }
        }
    }

    private Quaterniond IntegrateRotation(Quaterniond orientation, Stage start, Stage middle, Stage end,
        Vector3d startRate, Vector3d middleRate, Vector3d endRate)
    {
        double h = end.Station - start.Station;
        Quaterniond q = Quaterniond.Identity;
        Quaterniond k1 = HeartlineRotationPiece.Rate(q, startRate);
        Quaterniond q2 = q + k1 * (h / 2);
        Quaterniond k2 = HeartlineRotationPiece.Rate(q2, StageRate(middle, q2, middleRate));
        Quaterniond q3 = q + k2 * (h / 2);
        Quaterniond k3 = HeartlineRotationPiece.Rate(q3, StageRate(middle, q3, middleRate));
        Quaterniond q4 = q + k3 * h;
        Quaterniond k4 = HeartlineRotationPiece.Rate(q4, StageRate(end, q4, endRate));
        return (q + (k1 + k2 * 2 + k3 * 2 + k4) * (h / 6)).Normalized();

        Vector3d StageRate(Stage sample, Quaterniond relative, Vector3d precomputed) => inputs.RequiresOrientation
            ? Omega(sample, (orientation * relative).Normalized()) : precomputed;
    }

    private readonly record struct Stage(double Station, EvaluationSide Side, ReconstructionTarget Target);
}

internal readonly record struct HeartlineIntegratedPiece(HeartlineRotationPiece Rotation,
    Vector3d StartOffset, Vector3d EndOffset, double QuadratureTolerance);
