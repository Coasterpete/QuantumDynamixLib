using System;
using System.Linq;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using QuantumDynamix.FVD;
using QuantumDynamix.Geometry;
using QuantumDynamix.Math;
using QuantumDynamix.Math.Functions;
using QuantumDynamix.Math.Timelines;
using QuantumDynamix.Splines.Fitting;
using QuantumDynamix.Track;
using static QuantumDynamix.Tests.HeartlineReconstructionFixtures;

namespace QuantumDynamix.Tests;

[TestClass]
public sealed class HeartlineSplineConversionTests
{
    public TestContext TestContext { get; set; } = null!;

    [TestMethod]
    [DataRow(false)]
    [DataRow(true)]
    public void QuinticFitPreservesAnalyticCircleAtIndependentStations(bool vertical)
    {
        double length = 10 * System.Math.PI;
        const double origin = 50;
        var source = Reconstruct([Targets(origin, origin + 9, vertical ? 0 : 5, vertical ? 5 : 0),
            Targets(origin + 9, origin + length, vertical ? 0 : 5, vertical ? 5 : 0)], Distance(origin, origin + length));
        var fit = HeartlineReconstructor.FitToTrack(source, FitSettings(continuity: 3));
        Log(TestContext, vertical ? "vertical circle fitted" : "horizontal circle fitted", source, fit);
        Assert.IsTrue(fit.Succeeded, fit.Failure?.ToString());
        Assert.AreSame(source, fit.Source);
        var track = fit.Track!;
        Assert.AreEqual(5, track.Heartline.Definition.Degree);
        Assert.IsFalse(track.Heartline.Definition.IsPeriodic);
        Assert.IsTrue(track.Heartline.Definition.Breakpoints.All(u => track.Heartline.Definition.ContinuityAt(u) >= 3));
        Assert.IsTrue(fit.Diagnostics.VerificationSamples > track.Heartline.Definition.ControlPointCount);

        foreach (double s in new[] { 1.129, 8.731, 9, 14.832, 19.365, 25.741, 30.283 })
        {
            const EvaluationSide side = EvaluationSide.Right;
            double fittedStation = fit.GetFittedStation(s);
            Assert.AreEqual(s, fit.GetSourceStation(fittedStation), 1e-8);
            LongitudinalMotionSample motion = fit.GetFittedMotion(s, side);
            Assert.AreEqual(fittedStation, motion.HeartlineStationMetres);
            var force = HeartlineForceEvaluator.Evaluate(track, motion, new(0, 0, -9.81), side);
            TestMath.Near(CirclePosition(s, vertical), force.BankedFrame.Position, 3.1e-5);
            TestMath.Near(CircleTangent(s, vertical), force.BankedFrame.Forward, 4.1e-6);
            TestMath.Near(CircleCurvature(s, vertical) * 100, force.WorldCoordinateAcceleration, 8.7e-4);
            Assert.AreEqual(Vector3d.Dot(force.WorldCoordinateAcceleration + Vector3d.UnitZ * 9.81, force.BankedFrame.Up),
                force.TrackSpecificForce.Z, 1e-12);
        }
    }

    [TestMethod]
    public void ChangingBankFitsWithOriginalAngleAndFvdAcceleration()
    {
        var source = Helix();
        var fit = HeartlineReconstructor.FitToTrack(source, FitSettings());
        Log(TestContext, "changing-bank helix fitted", source, fit);
        Assert.IsTrue(fit.Succeeded, fit.Failure?.ToString());
        var track = fit.Track!;
        foreach (double s in new[] { 0, 0.237, 1.813, 4.731, 7.129, 11.736, 12.5 })
        {
            const EvaluationSide side = EvaluationSide.Right;
            double fitted = fit.GetFittedStation(s);
            var force = HeartlineForceEvaluator.Evaluate(track, fit.GetFittedMotion(s, side), Vector3d.Zero, side);
            var (si, co) = System.Math.SinCos(s / 5);
            Vector3d normal = new(-co, -si, 0);
            Assert.AreEqual(0.12 * s, track.Banking.Evaluate(fitted).Value, 1e-9);
            TestMath.Near(normal, force.BankedFrame.Left, 3.1e-6);
            TestMath.Near(new(0, 16, 0), force.TrackSpecificForce, 8.7e-4);
            TestMath.Near(normal * 16, force.WorldCoordinateAcceleration, 1e-3);
        }
        Assert.IsNotNull(fit.Diagnostics.WorstBankRollError);
        Assert.IsNotNull(fit.Diagnostics.WorstBankedFrameError);
    }

    [TestMethod]
    public void FittedStepRetainsSidedSecondDerivativesAndFvdLimits()
    {
        var source = Reconstruct([Targets(17, 22, 0), Targets(22, 32, 5)], Distance(17, 32),
            settings: Settings(steps: [22]));
        var fit = HeartlineReconstructor.FitToTrack(source, FitSettings());
        Log(TestContext, "fitted curvature step", source, fit);
        Assert.IsTrue(fit.Succeeded, fit.Failure?.ToString());
        var track = fit.Track!;
        Assert.AreEqual(1, track.Heartline.Definition.ContinuityAt(5));
        double fitted = fit.GetFittedStation(5);
        Assert.AreEqual(5, fit.GetSourceStation(fitted));
        Assert.AreEqual(5, track.HeartlineStations.Invert(fitted).Parameter);
        Assert.ThrowsExactly<ArgumentException>(() => fit.GetFittedMotion(5, EvaluationSide.Automatic));
        var left = Force(EvaluationSide.Left);
        var right = Force(EvaluationSide.Right);
        Assert.AreEqual(left.BankedFrame, right.BankedFrame);
        TestMath.Near(Vector3d.Zero, left.WorldCoordinateAcceleration, 2e-9);
        TestMath.Near(Vector3d.UnitY * 5, right.WorldCoordinateAcceleration, 2e-9);

        double before = System.Math.BitDecrement(5);
        double after = System.Math.BitIncrement(5);
        foreach (double adjacent in new[] { before, after })
        {
            var motion = fit.GetFittedMotion(adjacent, EvaluationSide.Right);
            double parameter = fit.GetSourceStation(motion.HeartlineStationMetres);
            Assert.AreEqual(adjacent, parameter);
            var near = HeartlineForceEvaluator.Evaluate(track, motion, Vector3d.Zero, EvaluationSide.Right);
            var jet = track.Heartline.Evaluate(parameter, EvaluationSide.Right);
            Assert.AreEqual(jet.Position, near.BankedFrame.Position);
            TestMath.Near(jet.FirstDerivative.Normalized(), near.BankedFrame.Forward, 2e-14);
            TestMath.Near(Vector3d.UnitZ, near.BankedFrame.Up, 2e-14);
            Assert.AreEqual(adjacent < 5 ? 0 : 5, near.WorldCoordinateAcceleration.Y, 2e-9);
        }

        HeartlineForceSample Force(EvaluationSide side) =>
            HeartlineForceEvaluator.Evaluate(track, fit.GetFittedMotion(5, side), Vector3d.Zero, side);
    }

    [TestMethod]
    public void TimeAuthoredCurvatureStepMapsToAnExplicitFittedKnot()
    {
        const double clock = 42;
        const double start = 140;
        Region[] regions = [Timed(clock, clock + 0.5, 0), Timed(clock + 0.5, clock + 1.5, 5)];
        var source = Reconstruct(regions, Distance(start, start + 15), clock: clock, settings: Settings(steps: [clock + 0.5]));
        double join = source.BoundaryStationsMetres[1];
        Assert.AreEqual(5, join, 1e-9);
        Assert.AreEqual(clock + 0.5, source.Evaluate(join, EvaluationSide.Left).AuthoredCoordinate);
        Assert.AreEqual(start + 5, source.Evaluate(join, EvaluationSide.Right).Motion.HeartlineStationMetres, 1e-9);
        var fit = HeartlineReconstructor.FitToTrack(source, FitSettings());
        Log(TestContext, "time-authored curvature step fitted", source, fit);
        Assert.IsTrue(fit.Succeeded, fit.Failure?.ToString());
        Assert.AreEqual(1, fit.Track!.Heartline.Definition.ContinuityAt(join));
        foreach (EvaluationSide side in new[] { EvaluationSide.Left, EvaluationSide.Right })
        {
            var force = HeartlineForceEvaluator.Evaluate(fit.Track, fit.GetFittedMotion(join, side), Vector3d.Zero, side);
            Assert.AreEqual(side == EvaluationSide.Left ? 0 : 5, force.TrackSpecificForce.Y, 2e-9);
        }

        static Region Timed(double a, double b, double left) =>
            Targets(Constant(ProfileDomain.Time, a, b, left), Constant(ProfileDomain.Time, a, b, 0));
    }

    [TestMethod]
    public void FittedMotionPreservesTimeAndUsesBothChainRuleTerms()
    {
        var source = Reconstruct([Targets(Function(ProfileDomain.Distance, 100, 124, x => new((100 + 4 * (x - 100)) / 20)),
            Constant(ProfileDomain.Distance, 100, 124, 0))], Distance(100, 124, 10, 2));
        var fit = HeartlineReconstructor.FitToTrack(source, FitSettings());
        Log(TestContext, "varying-speed fitted circle", source, fit);
        Assert.IsTrue(fit.Succeeded, fit.Failure?.ToString());

        bool changedSpeed = false;
        bool secondChainTerm = false;
        foreach (double s in new[] { 0.901, 3.123, 8.932, 10.419, 15.237, 19.493, 22.371 })
        {
            var original = source.Evaluate(s);
            var mapped = fit.GetFittedMotion(s, EvaluationSide.Right);
            var fitJet = fit.Track!.Heartline.Evaluate(s, EvaluationSide.Right);
            Vector3d expectedAcceleration = 2 * fitJet.FirstDerivative + (100 + 4 * s) * fitJet.SecondDerivative;
            var f = HeartlineForceEvaluator.Evaluate(fit.Track, mapped, Vector3d.Zero, EvaluationSide.Right);
            Assert.AreEqual(100 + s, original.Motion.HeartlineStationMetres, 1e-13);
            Assert.AreEqual(s / (5 + System.Math.Sqrt(25 + s)), mapped.ElapsedSeconds, 2e-13);
            TestMath.Near(expectedAcceleration, f.WorldCoordinateAcceleration, 2e-8);
            changedSpeed |= System.Math.Abs(mapped.HeartlineVelocityMetresPerSecond - original.Motion.HeartlineVelocityMetresPerSecond) > 1e-10;
            secondChainTerm |= System.Math.Abs(mapped.LongitudinalAccelerationMetresPerSecondSquared - 2 * fitJet.FirstDerivative.Length) > 1e-10;
        }
        Assert.IsTrue(changedSpeed && secondChainTerm);
    }

    [TestMethod]
    public void AcceleratingTimeAuthoredReconstructionFitsAndPassesForwardFvd()
    {
        const double length = 262.5;
        const double duration = 15;
        const double initialSpeed = 16;
        const double acceleration = 0.2;
        const int waves = 4;
        double phasePerMetre = 2 * System.Math.PI * waves / length;
        var regions = new Region[waves];
        for (int i = 0; i < regions.Length; i++)
        {
            double start = i == 0 ? 0 : TimeAt(length * i / waves);
            double end = i == regions.Length - 1 ? duration : TimeAt(length * (i + 1) / waves);
            regions[i] = Targets(
                Function(ProfileDomain.Time, start, end, t => new(Lateral(StationAt(t)), derivativeOrder: 0)),
                Function(ProfileDomain.Time, start, end, t => new(Vertical(StationAt(t)), derivativeOrder: 0)));
        }
        var motion = Time(Constant(ProfileDomain.Time, 0, duration, acceleration), 0, initialSpeed);
        var bank = Function(ProfileDomain.Distance, 0, length,
            s => new(0.15 * System.Math.Sin(phasePerMetre * s + 0.2), derivativeOrder: 0));
        var source = Reconstruct(regions, motion, bank, settings: Settings(work: 8_000_000));
        Assert.AreEqual(length, source.LengthMetres);
        const double seed = length * 48 / 64;
        double knot = source.BoundaryStationsMetres[3];
        Assert.AreEqual(System.Math.BitIncrement(seed), knot);

        var settings = FitSettings(maxSamples: 100_000, maxControls: 512, maxPasses: 12);
        var fit = HeartlineReconstructor.FitToTrack(source, settings);
        Log(TestContext, "262.5 m time-authored accelerating fit", source, fit);
        Assert.IsTrue(fit.Succeeded, fit.Failure?.ToString());
        var track = fit.Track!;
        CollectionAssert.IsSubsetOf(source.BoundaryStationsMetres.Skip(1).SkipLast(1).ToArray(),
            track.Heartline.Definition.Breakpoints.ToArray());
        Assert.IsTrue(track.Frames.Nodes.Any(node => node.Parameter == knot));
        Assert.IsTrue(track.Frames.EstimatedAccumulatedAngularError <= settings.FrameTransportToleranceRadians);
        Assert.IsTrue(fit.Diagnostics.VerificationSamples > track.Heartline.Definition.ControlPointCount);
        var fvdErrors = fit.Diagnostics.MaximumCoordinateAccelerationErrorsMetresPerSecondSquared!.Value;
        var limits = settings.CoordinateAccelerationTolerancesMetresPerSecondSquared;
        Assert.IsTrue(fvdErrors.X <= limits.X && fvdErrors.Y <= limits.Y && fvdErrors.Z <= limits.Z);

        foreach (double s in new[] { 0, 23.417, 87.23, knot, 234.89, length })
        {
            EvaluationSide side = s == length ? EvaluationSide.Left : EvaluationSide.Right;
            LongitudinalMotionSample mapped = fit.GetFittedMotion(s, side);
            HeartlineForceSample force = HeartlineForceEvaluator.Evaluate(track, mapped, Vector3d.Zero, side);
            Assert.AreEqual(TimeAt(s), mapped.ElapsedSeconds, 1e-8);
            Assert.AreEqual(acceleration, force.TrackSpecificForce.X, limits.X);
            Assert.AreEqual(Lateral(s), force.TrackSpecificForce.Y, limits.Y);
            Assert.AreEqual(Vertical(s), force.TrackSpecificForce.Z, limits.Z);
        }

        static double TimeAt(double s) =>
            2 * s / (initialSpeed + System.Math.Sqrt(initialSpeed * initialSpeed + 2 * acceleration * s));
        static double StationAt(double t) => t * initialSpeed + 0.5 * acceleration * t * t;
        double Lateral(double s) => 4 * System.Math.Sin(phasePerMetre * s);
        double Vertical(double s) => 2 * System.Math.Sin(phasePerMetre * s * 2 + 0.3);
    }

    [TestMethod]
    public void TimeBoundaryPinCarriesCorrespondenceErrorThroughFittedMotion()
    {
        const double clock = 30;
        const double eventElapsed = 1;
        const double eventStation = 11;
        const double eventSpeed = 12;
        Region[] regions = [Timed(clock, clock + 1), Timed(clock + 1, clock + 2)];
        var source = Reconstruct(regions, Distance(100, 124, 10, 2), clock: clock,
            settings: Settings(stationMappingTolerance: 1.5, timeMappingTolerance: 0.2));
        var fit = HeartlineReconstructor.FitToTrack(source, FitSettings());
        Assert.IsTrue(fit.Succeeded, fit.Failure?.ToString());
        double join = source.BoundaryStationsMetres[1];
        double fittedEventStation = fit.GetFittedStation(eventStation);

        foreach (EvaluationSide side in new[] { EvaluationSide.Left, EvaluationSide.Right })
        {
            Assert.AreEqual(clock + eventElapsed, source.Evaluate(join, side).AuthoredCoordinate);
            var mapped = fit.GetFittedMotion(join, side);
            var error = mapped.EstimatedAbsoluteError;
            Assert.IsTrue(error.StationMetres >= System.Math.Abs(mapped.HeartlineStationMetres - fittedEventStation));
            Assert.IsTrue(error.ElapsedTimeSeconds >= System.Math.Abs(mapped.ElapsedSeconds - eventElapsed));
            Assert.IsTrue(error.VelocityMetresPerSecond >= System.Math.Abs(mapped.HeartlineVelocityMetresPerSecond - eventSpeed));
            TestContext.WriteLine(FormattableString.Invariant(
                $"M3.1 pinned event: station={mapped.HeartlineStationMetres - fittedEventStation:G9} m (estimate {error.StationMetres:G9} m), time={mapped.ElapsedSeconds - eventElapsed:G9} s (estimate {error.ElapsedTimeSeconds:G9} s), speed={mapped.HeartlineVelocityMetresPerSecond - eventSpeed:G9} m/s (estimate {error.VelocityMetresPerSecond:G9} m/s)"));
        }

        static Region Timed(double a, double b) => Targets(Constant(ProfileDomain.Time, a, b, 0), Constant(ProfileDomain.Time, a, b, 0));
    }

    [TestMethod]
    public void ClusteredC4ConstraintsRetryLocallyAndVerifyRecoveredFit()
    {
        var source = Reconstruct([Targets(0, 4, 15), Targets(4, 8, 15), Targets(8, 12, 15), Targets(12, 64, 15)],
            Distance(0, 64));
        var initial = HeartlineReconstructor.FitToTrack(source, FitSettings(continuity: 4, maxControls: 12, maxPasses: 1));
        Assert.AreEqual(HeartlineFailureCode.SplineSolveFailed, initial.Failure!.Code);
        Assert.AreEqual((0.0, source.LengthMetres), initial.Failure.SourceStationInterval);
        StringAssert.Contains(initial.Failure.Message, "Spline solve");
        Assert.AreEqual(FitStatus.Infeasible, initial.Diagnostics.Solver!.Status);
        Assert.AreEqual(0, initial.Diagnostics.VerificationSamples);

        var atCap = HeartlineReconstructor.FitToTrack(source, FitSettings(continuity: 4, maxControls: 12, maxPasses: 2));
        Assert.AreEqual(2, atCap.Diagnostics.RefinementPasses);
        Assert.AreEqual(FitStatus.Success, atCap.Diagnostics.Solver!.Status);
        Assert.AreEqual(12, atCap.Diagnostics.Solver.ConstraintRank);

        var settings = FitSettings(continuity: 4, maxControls: 64, maxPasses: 8);
        var fit = HeartlineReconstructor.FitToTrack(source, settings);
        Log(TestContext, "clustered C4 retry", source, fit);
        Assert.IsTrue(fit.Succeeded, fit.Failure?.ToString());
        var definition = fit.Track!.Heartline.Definition;
        Assert.IsTrue(definition.ControlPointCount <= settings.MaximumControlPoints);
        Assert.IsTrue(definition.Breakpoints.All(u => definition.ContinuityAt(u) >= 4));
        CollectionAssert.IsSubsetOf(new[] { 4.0, 8, 12 }, definition.Breakpoints.ToArray());
        Assert.IsTrue(fit.Diagnostics.RefinementPasses is > 1 and <= 8);
        Assert.IsTrue(fit.Diagnostics.VerificationSamples > 0 && fit.Diagnostics.VerificationSamples <= settings.MaximumVerificationSamples);
        Assert.IsTrue(fit.Diagnostics.MaximumCoordinateAccelerationErrorsMetresPerSecondSquared!.Value.Y
            <= settings.CoordinateAccelerationTolerancesMetresPerSecondSquared.Y);
    }

    [TestMethod]
    public void DeclaredBreakpointsResolveANarrowAccelerationPulseWithCoarseMaximumStep()
    {
        var source = ReconstructPulse(declareBreakpoints: true);
        CollectionAssert.AreEqual(new[] { 0.0, PulseStart, PulseEnd, 1 }, source.BoundaryStationsMetres.ToArray());
        Assert.AreEqual(0.2, source.Evaluate(PulseCentre).CurvatureVectorPerMetre!.Value.Length, 2e-7);
        Assert.AreEqual(0, source.Evaluate(0.2).CurvatureVectorPerMetre!.Value.Length, 2e-7);

        var fit = HeartlineReconstructor.FitToTrack(source, FitSettings(verificationSpacing: 0.005));
        Log(TestContext, "declared narrow pulse", source, fit);
        Assert.IsTrue(fit.Succeeded, fit.Failure?.ToString());
        var motion = fit.GetFittedMotion(PulseCentre, EvaluationSide.Right);
        var force = HeartlineForceEvaluator.Evaluate(fit.Track!, motion, Vector3d.Zero, EvaluationSide.Right);
        Assert.AreEqual(20, force.TrackSpecificForce.Y, 5e-4);
    }

    [TestMethod]
    public void FinerIndependentVerificationRejectsANarrowPulseMissedDuringReconstruction()
    {
        var source = ReconstructPulse(declareBreakpoints: false);
        var fit = HeartlineReconstructor.FitToTrack(source, FitSettings(maxPasses: 1, verificationSpacing: 0.005));
        Assert.IsFalse(fit.Succeeded);
        Assert.AreSame(source, fit.Source);
        Assert.AreEqual(HeartlineFailureCode.FitToleranceExceeded, fit.Failure!.Code);
        Assert.AreEqual("Lateral acceleration error", fit.Failure.Measurement!.Value.Quantity);
        double station = fit.Failure.SourceStationInterval!.Value.StartMetres;
        Assert.IsTrue(station > PulseStart && station < PulseEnd);
    }

    private const double PulseCentre = 0.16;
    private const double PulseStart = 0.15;
    private const double PulseEnd = 0.17;

    private static ReconstructedHeartline ReconstructPulse(bool declareBreakpoints)
    {
        var pulse = new CustomFunction(0, 1, (station, _) =>
        {
            double fraction = (station - PulseCentre) / 0.01;
            double shoulder = System.Math.Max(0, 1 - fraction * fraction);
            return new ScalarJet(20 * System.Math.Pow(shoulder, 5), derivativeOrder: 0);
        }, declareBreakpoints ? [PulseStart, PulseEnd] : null);
        var lateral = new ScalarTimeline(ProfileDomain.Distance, [pulse]);
        return Reconstruct([Targets(lateral, Constant(ProfileDomain.Distance, 0, 1, 0))], Distance(0, 1));
    }

    [TestMethod]
    public void CompleteFrameAngleAndSignedRollUseTheShortestRotation()
    {
        var source = new CurveFrame(Vector3d.Zero, Vector3d.UnitX, Vector3d.UnitY, Vector3d.UnitZ);
        var turned = Quaterniond.FromAxisAngle(Vector3d.UnitZ, 0.12);
        var fitted = new CurveFrame(Vector3d.Zero, turned.Rotate(source.Forward), turned.Rotate(source.Left), source.Up).Banked(0.2);
        var (heading, frame, roll) = HeartlineFrameMath.Compare(source, fitted);
        Assert.AreEqual(0.12, heading, 1e-14);
        Assert.AreEqual(2 * System.Math.Acos(System.Math.Cos(0.06) * System.Math.Cos(0.1)), frame, 1e-14);
        Assert.AreEqual(0.2, roll!.Value, 1e-14);
        Assert.AreEqual(-0.2, HeartlineFrameMath.Compare(source, source.Banked(-0.2)).Roll!.Value, 1e-14);
        Assert.AreEqual(System.Math.PI, HeartlineFrameMath.Compare(source, source.Banked(System.Math.PI)).Frame, 1e-14);
        Assert.AreEqual(0, HeartlineFrameMath.PrincipalAngle(turned, turned * -1), 1e-15);
    }

    [TestMethod]
    [DataRow(false)]
    [DataRow(true)]
    public void ExcessBankOrientationErrorRejectsFitWithoutChangingSource(bool rollOnly)
    {
        var source = Helix();
        double before = source.Evaluate(7.3).BankRadians;
        var fit = HeartlineReconstructor.FitToTrack(source, FitSettings(frame: rollOnly ? 3e-6 : 1e-12,
            roll: rollOnly ? 1e-12 : 3e-6, maxPasses: 7));
        Log(TestContext, rollOnly ? "rejected bank roll" : "rejected banked frame", source, fit);
        Assert.IsFalse(fit.Succeeded);
        Assert.IsNull(fit.Track);
        Assert.AreEqual(HeartlineFailureCode.FitToleranceExceeded, fit.Failure!.Code);
        Assert.AreEqual(rollOnly ? "Bank roll error" : "Banked-frame error", fit.Failure.Measurement!.Value.Quantity);
        Assert.IsNotNull(fit.Failure.Side);
        Assert.AreSame(source, fit.Source);
        Assert.AreEqual(before, fit.Source.Evaluate(7.3).BankRadians);
        Assert.ThrowsExactly<InvalidOperationException>(() => fit.GetFittedStation(7.3));
    }
}
