using System;
using System.Collections.Generic;
using System.Linq;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using QuantumDynamix.FVD;
using QuantumDynamix.Geometry;
using QuantumDynamix.Math;
using QuantumDynamix.Math.Functions;
using QuantumDynamix.Math.Numerics;
using QuantumDynamix.Math.Timelines;
using QuantumDynamix.Track;

namespace QuantumDynamix.Tests;

internal static class HeartlineReconstructionFixtures
{
    internal static HeartlineReconstructionSettings Settings(int work = 2_000_000, double maximumStep = 1,
        double[]? steps = null, double[]? continuity = null, double minimumSpeed = 1e-4,
        double stationMappingTolerance = 1e-8, double timeMappingTolerance = 1e-8, double bankJoinTolerance = 1e-10) =>
        new(continuity ?? [1e-10], steps)
        {
            PositionToleranceMetres = 2e-6,
            FrameAngleToleranceRadians = 2e-6,
            CurvatureTolerancePerMetre = 2e-7,
            UnbankedRollRateToleranceRadiansPerMetre = 2e-7,
            BankJoinToleranceRadians = bankJoinTolerance,
            StationMappingToleranceMetres = stationMappingTolerance,
            TimeMappingToleranceSeconds = timeMappingTolerance,
            MinimumForwardSpeedMetresPerSecond = minimumSpeed,
            MaximumStepMetres = maximumStep,
            MaximumWork = work
        };

    internal static HeartlineFitSettings FitSettings(double frame = 3e-6, double roll = 3e-6,
        int maxSamples = 12_000, int maxControls = 256, int maxPasses = 10, int continuity = 2,
        double regularity = 0.5, QuadratureSettings? length = null, double verificationSpacing = 0.3,
        double frameTransportTolerance = 1e-9) =>
        new()
        {
            PositionToleranceMetres = 3e-5,
            TangentToleranceRadians = 4e-6,
            CurvatureTolerancePerMetre = 4e-6,
            BankedFrameToleranceRadians = frame,
            BankRollToleranceRadians = roll,
            StationDriftToleranceMetres = 3e-5,
            CoordinateAccelerationTolerancesMetresPerSecondSquared = new(5e-4, 5e-4, 5e-4),
            MinimumContinuousKnotOrder = continuity,
            MinimumParameterSpeed = regularity,
            ArcLengthSettings = length ?? new(2e-9, 0),
            FrameTransportToleranceRadians = frameTransportTolerance,
            MaximumVerificationSpacingMetres = verificationSpacing,
            MaximumVerificationSamples = maxSamples,
            MaximumControlPoints = maxControls,
            MaximumRefinementPasses = maxPasses
        };

    internal static ScalarTimeline Constant(ProfileDomain domain, double start, double end, double value) =>
        new(domain, [FunctionSegment.Transition(start, end, value, value)]);

    internal static ScalarTimeline Function(ProfileDomain domain, double start, double end, Func<double, ScalarJet> value) =>
        new(domain, [new CustomFunction(start, end, (coordinate, _) => value(coordinate))]);

    internal static Region Targets(ScalarTimeline lateral, ScalarTimeline vertical) =>
        new(lateral.Domain, lateral.Minimum, lateral.Maximum, new Dictionary<string, ScalarTimeline>
        {
            ["lateral"] = lateral,
            ["vertical"] = vertical
        });

    internal static Region Targets(double start, double end, double lateral, double vertical = 0) =>
        Targets(Constant(ProfileDomain.Distance, start, end, lateral), Constant(ProfileDomain.Distance, start, end, vertical));

    internal static LongitudinalTrajectory Distance(double start, double end, double initialSpeed = 10, double acceleration = 0) =>
        Distance(Constant(ProfileDomain.Distance, start, end, acceleration), initialSpeed);

    internal static LongitudinalTrajectory Distance(ScalarTimeline acceleration, double initialSpeed = 10) =>
        MotionProfile(acceleration).IntegrateDistance(initialSpeed, new(1e-9, 1e-9, 32, 100_000));

    internal static LongitudinalTrajectory Time(ScalarTimeline acceleration, double station, double velocity) =>
        MotionProfile(acceleration).IntegrateTime(new(station, velocity), new(1e-9, 1e-9, 32, 50_000));

    private static LongitudinalAccelerationProfile MotionProfile(ScalarTimeline acceleration)
    {
        var region = new Region(acceleration.Domain, acceleration.Minimum, acceleration.Maximum,
            new Dictionary<string, ScalarTimeline> { ["longitudinal"] = acceleration });
        return new([new(region, "longitudinal")], -1, [0, 0, 0, 0]);
    }

    internal static ReconstructedHeartline Reconstruct(Region[] regions, LongitudinalTrajectory motion,
        ScalarTimeline? bank = null, RigidPose3d? pose = null, HeartlineReconstructionSettings? settings = null,
        double? clock = null, Func<LongitudinalAccelerationBinding, double, double, AccelerationRange?>? bounds = null) =>
        HeartlineReconstructor.Reconstruct(new(regions, "lateral", "vertical", motion, pose ?? RigidPose3d.Identity,
            bank, clock, bounds), settings ?? Settings());

    internal static ReconstructedHeartline Helix()
    {
        const double length = 12.5;
        Quaterniond rotation = Quaterniond.FromAxisAngle(Vector3d.UnitZ, System.Math.PI / 2)
            * Quaterniond.FromAxisAngle(Vector3d.UnitY, -System.Math.Atan2(3, 4));
        return Reconstruct([Targets(0, length, 16)], Distance(0, length),
            Function(ProfileDomain.Distance, 0, length, s => new(0.12 * s, 0.12)),
            new RigidPose3d(new(4, 0, 0), rotation));
    }

    internal static Vector3d CirclePosition(double s, bool vertical = false)
    {
        double angle = s / 20;
        double x = 20 * System.Math.Sin(angle);
        double y = 20 * (1 - System.Math.Cos(angle));
        return vertical ? new(x, 0, y) : new(x, y, 0);
    }

    internal static Vector3d CircleTangent(double s, bool vertical = false)
    {
        double c = System.Math.Cos(s / 20);
        double k = System.Math.Sin(s / 20);
        return vertical ? new(c, 0, k) : new(c, k, 0);
    }

    internal static Vector3d CircleCurvature(double s, bool vertical = false)
    {
        double x = -System.Math.Sin(s / 20) / 20;
        double k = System.Math.Cos(s / 20) / 20;
        return vertical ? new(x, 0, k) : new(x, k, 0);
    }

    internal static void VerifyAnalytic(TestContext context, string fixture, ReconstructedHeartline source,
        double[] stations, Func<double, (Vector3d Position, Vector3d Tangent, Vector3d Curvature)> exact)
    {
        double position = 0;
        double tangent = 0;
        double curvature = 0;
        foreach (double station in stations)
        {
            var sample = source.Evaluate(station);
            var expected = exact(station);
            position = System.Math.Max(position, (sample.UnbankedFrame.Position - expected.Position).Length);
            tangent = System.Math.Max(tangent, (sample.UnbankedFrame.Forward - expected.Tangent).Length);
            curvature = System.Math.Max(curvature, (sample.CurvatureVectorPerMetre!.Value - expected.Curvature).Length);
        }
        Assert.IsTrue(position <= 2e-7, $"Position error {position:R} m");
        Assert.IsTrue(tangent <= 2e-8, $"Unit-tangent difference {tangent:R}");
        Assert.IsTrue(curvature <= 2e-7, $"Curvature error {curvature:R} 1/m");
        context.WriteLine(FormattableString.Invariant(
            $"M3 {fixture} analytical residuals: position={position:G9} m, tangent vector={tangent:G9}, curvature={curvature:G9} 1/m"));
    }

    internal static void Log(TestContext context, string fixture, ReconstructedHeartline source, HeartlineFitResult? fit = null)
    {
        var d = source.Diagnostics;
        context.WriteLine(FormattableString.Invariant(
            $"M3 {fixture} reconstruction: position={d.EstimatedPositionErrorMetres:G9} m, frame={d.EstimatedFrameErrorRadians:G9} rad, curvature residual={d.MaximumObservedCurvatureResidualPerMetre:G9} 1/m, unbanked roll rate={d.MaximumObservedUnbankedRollRateRadiansPerMetre:G9} rad/m, station mapping={d.MaximumEstimatedStationMappingErrorMetres:G9} m, time mapping={d.MaximumEstimatedTimeMappingErrorSeconds:G9} s, work={d.WorkUsed}"));
        if (fit is null) { return; }
        var f = fit.Diagnostics;
        Vector3d a = f.MaximumCoordinateAccelerationErrorsMetresPerSecondSquared ?? default;
        context.WriteLine(FormattableString.Invariant(
            $"M3 {fixture} fit: success={fit.Succeeded}, position={f.MaximumPositionErrorMetres:G9} m, tangent={f.MaximumTangentErrorRadians:G9} rad, curvature={f.MaximumCurvatureErrorPerMetre:G9} 1/m, frame={f.WorstBankedFrameError?.ErrorRadians:G9} rad, roll={f.WorstBankRollError?.SignedErrorRadians:G9} rad, station drift={f.MaximumStationDriftMetres:G9} m, length={f.LengthErrorMetres:G9} m, FVD=({a.X:G9}, {a.Y:G9}, {a.Z:G9}) m/s², samples={f.VerificationSamples}, passes={f.RefinementPasses}, failure={fit.Failure}"));
    }
}
