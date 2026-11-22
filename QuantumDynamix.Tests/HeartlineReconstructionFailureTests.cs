using System;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using QuantumDynamix.FVD;
using QuantumDynamix.Geometry;
using QuantumDynamix.Math;
using QuantumDynamix.Math.Functions;
using QuantumDynamix.Math.Timelines;
using QuantumDynamix.Track;
using static QuantumDynamix.Tests.HeartlineReconstructionFixtures;

namespace QuantumDynamix.Tests;

[TestClass]
public sealed class HeartlineReconstructionFailureTests
{
    [TestMethod]
    public void InvalidInitialPoseAndRegionArrangementHaveTypedFailures()
    {
        var time = Distance(5, 15);
        var invalidPose = Assert.ThrowsExactly<HeartlineReconstructionException>(() =>
            Reconstruct([Targets(5, 15, 0)], time, pose: default(RigidPose3d)));
        Assert.AreEqual(HeartlineFailureCode.InvalidInput, invalidPose.Failure.Code);

        var coverage = Assert.ThrowsExactly<HeartlineReconstructionException>(() => Reconstruct([Targets(5, 14, 0)], time));
        Assert.AreEqual(HeartlineFailureCode.InvalidInput, coverage.Failure.Code);
        var gap = Assert.ThrowsExactly<HeartlineReconstructionException>(() =>
            Reconstruct([Targets(5, 8, 0), Targets(9, 15, 0)], time));
        Assert.AreEqual(HeartlineFailureCode.InvalidInput, gap.Failure.Code);
        Assert.AreEqual(1, gap.Failure.RegionIndex);
    }

    [TestMethod]
    [DataRow(0.0, 0.0)]
    [DataRow(-1.0, 0.0)]
    [DataRow(2.0, -1.0)]
    public void TimeTrajectoryRejectsInitialOrTerminalNonpositiveSpeed(double v0, double a)
    {
        var motion = Time(Constant(ProfileDomain.Time, 0, 2, a), 10, v0);
        var ex = Assert.ThrowsExactly<HeartlineReconstructionException>(() => Reconstruct([Targets(10, 14, 0)], motion));
        Assert.AreEqual(HeartlineFailureCode.NonPositiveSpeed, ex.Failure.Code);
    }

    [TestMethod]
    public void PositiveEndpointSpeedsCannotHideAnInteriorStop()
    {
        // v(t) = (t - 1)^2 has positive endpoint velocities and reaches zero at t = 1.
        var a = new ScalarTimeline(ProfileDomain.Time, [FunctionSegment.Transition(0, 2, -2, 2)]);
        var motion = Time(a, 10, 1);
        Assert.AreEqual(1, motion.Sample(2).HeartlineVelocityMetresPerSecond, 1e-12);
        Assert.AreEqual(0, motion.Sample(1).HeartlineVelocityMetresPerSecond, 1e-12);
        var ex = Assert.ThrowsExactly<HeartlineReconstructionException>(() =>
            Reconstruct([Targets(10, 11, 0)], motion, settings: Settings(work: 2000)));
        Assert.IsTrue(ex.Failure.Code is HeartlineFailureCode.SpeedPositivityUnresolved or HeartlineFailureCode.NonPositiveSpeed);
        Assert.IsNotNull(ex.Failure.AuthoredInterval);
    }

    [TestMethod]
    public void CustomTimeMotionRequiresValidAccelerationEnclosures()
    {
        var motion = Time(Function(ProfileDomain.Time, 30, 32, _ => new(0)), 100, 10);
        Region[] regions = [Targets(100, 120, 0)];
        var missing = Assert.ThrowsExactly<HeartlineReconstructionException>(() => Reconstruct(regions, motion));
        Assert.AreEqual(HeartlineFailureCode.MissingAccelerationBounds, missing.Failure.Code);
        Assert.IsInstanceOfType<TrajectoryException>(missing.InnerException);
        var invalid = Assert.ThrowsExactly<HeartlineReconstructionException>(() =>
            Reconstruct(regions, motion, bounds: (_, _, _) => new(1, 2)));
        Assert.AreEqual(HeartlineFailureCode.InvalidAccelerationBounds, invalid.Failure.Code);
        var source = Reconstruct(regions, motion, bounds: (_, a, b) =>
        {
            Assert.IsTrue(a >= 30 && b <= 32);
            return new(0, 0);
        });
        Assert.AreEqual(20, source.LengthMetres, 1e-12);
        Assert.AreEqual(10, source.Diagnostics.ProvenForwardSpeedLowerBoundMetresPerSecond, 1e-12);
    }

    [TestMethod]
    public void ReconstructionBudgetFailureCarriesLocalIntervalAndNeverReturnsPartialGeometry()
    {
        const int limit = 200;
        var ex = Assert.ThrowsExactly<HeartlineReconstructionException>(() =>
            Reconstruct([Targets(0, 20, 5)], Distance(0, 20), settings: Settings(work: limit, maximumStep: 0.1)));
        Assert.AreEqual(HeartlineFailureCode.NumericalBudgetExceeded, ex.Failure.Code);
        Assert.IsNotNull(ex.Failure.SourceStationInterval);
        Assert.IsTrue(ex.WorkUsed <= limit);
    }

    [TestMethod]
    [DataRow(1, 20)]
    [DataRow(1000, 5)]
    public void FailedFitKeepsItsEvaluableNumericalSourceAndHonorsResourceCaps(int samples, int controls)
    {
        var source = Reconstruct([Targets(0, 10, 0)], Distance(0, 10));
        var fit = HeartlineReconstructor.FitToTrack(source, FitSettings(maxSamples: samples, maxControls: controls));
        Assert.IsFalse(fit.Succeeded);
        Assert.IsNull(fit.Track);
        Assert.AreEqual(HeartlineFailureCode.NumericalBudgetExceeded, fit.Failure!.Code);
        Assert.AreSame(source, fit.Source);
        Assert.IsTrue(fit.Diagnostics.VerificationSamples <= samples);
        TestMath.Near(new(7.3, 0, 0), fit.Source.Evaluate(7.3).UnbankedFrame.Position, 1e-13);
        Assert.IsTrue(HeartlineReconstructor.FitToTrack(source, FitSettings()).Succeeded);
    }

    [TestMethod]
    public void FitRejectsUnmetRegularityMarginAndSpendsNoFvdSamples()
    {
        var source = Reconstruct([Targets(0, 10, 0)], Distance(0, 10));
        var fit = HeartlineReconstructor.FitToTrack(source, FitSettings(regularity: 1.1, maxPasses: 2));
        Assert.IsFalse(fit.Succeeded);
        Assert.AreEqual(HeartlineFailureCode.SplineRegularityUnresolved, fit.Failure!.Code);
        Assert.AreEqual(2, fit.Diagnostics.RefinementPasses);
        Assert.AreEqual(0, fit.Diagnostics.VerificationSamples);
    }

    [TestMethod]
    public void FitPropagatesArcLengthQuadratureExhaustion()
    {
        var source = Helix();
        var fit = HeartlineReconstructor.FitToTrack(source, FitSettings(length: new(1e-25, 0, 1, 15)));
        Assert.IsFalse(fit.Succeeded);
        Assert.AreEqual(HeartlineFailureCode.NumericalBudgetExceeded, fit.Failure!.Code);
        StringAssert.Contains(fit.Failure.Message, "Arc-length integration");
        StringAssert.Contains(fit.Failure.Message, "15 integrand evaluations");
        var interval = fit.Failure.SourceStationInterval!.Value;
        Assert.AreEqual(0, interval.StartMetres);
        Assert.IsTrue(interval.EndMetres > 0 && interval.EndMetres <= source.LengthMetres);
        var measurement = fit.Failure.Measurement!.Value;
        Assert.AreEqual("Arc-length quadrature error estimate", measurement.Quantity);
        Assert.AreEqual("m", measurement.Unit);
        Assert.AreEqual(1e-25 * (interval.EndMetres / source.LengthMetres) / 4, measurement.Limit);
        Assert.IsTrue(double.IsFinite(measurement.Observed) && measurement.Observed > measurement.Limit);
        Assert.IsNotNull(fit.Diagnostics.Solver);
        Assert.AreSame(source, fit.Source);
    }

    [TestMethod]
    public void FitPropagatesFrameAngularBudgetExhaustion()
    {
        var source = Helix();
        var settings = FitSettings(frameTransportTolerance: 1e-24);
        var fit = HeartlineReconstructor.FitToTrack(source, settings);

        Assert.IsFalse(fit.Succeeded);
        Assert.AreEqual(HeartlineFailureCode.NumericalBudgetExceeded, fit.Failure!.Code);
        StringAssert.Contains(fit.Failure.Message, "Frame transport");
        StringAssert.Contains(fit.Failure.Message, "subdivision depth 20");
        var interval = fit.Failure.SourceStationInterval!.Value;
        Assert.IsTrue(0 <= interval.StartMetres && interval.StartMetres < interval.EndMetres && interval.EndMetres <= source.LengthMetres);
        var measurement = fit.Failure.Measurement!.Value;
        Assert.AreEqual("Frame angular error estimate", measurement.Quantity);
        Assert.AreEqual("rad", measurement.Unit);
        Assert.AreEqual(settings.FrameTransportToleranceRadians * ((interval.EndMetres - interval.StartMetres) / source.LengthMetres),
            measurement.Limit);
        Assert.IsTrue(double.IsFinite(measurement.Observed) && measurement.Observed > measurement.Limit);
        Assert.AreEqual(0, fit.Diagnostics.VerificationSamples);
    }

    [TestMethod]
    public void IncompleteLaterVerificationOnlyReportsItsOwnCandidateDiagnostics()
    {
        var source = Helix();
        const int sampleBudget = 50;
        var fit = HeartlineReconstructor.FitToTrack(source, FitSettings(maxSamples: sampleBudget));
        Assert.IsFalse(fit.Succeeded);
        Assert.AreEqual(HeartlineFailureCode.NumericalBudgetExceeded, fit.Failure!.Code);
        Assert.AreSame(source, fit.Source);
        Assert.IsTrue(fit.Diagnostics.RefinementPasses > 1);
        Assert.IsTrue(fit.Diagnostics.VerificationSamples > 0 && fit.Diagnostics.VerificationSamples <= sampleBudget);
        Assert.IsNotNull(fit.Diagnostics.Solver);
        Assert.IsNull(fit.Diagnostics.MaximumPositionErrorMetres);
        Assert.IsNull(fit.Diagnostics.MaximumCoordinateAccelerationErrorsMetresPerSecondSquared);
        Assert.IsNull(fit.Diagnostics.WorstBankedFrameError);
    }

    [TestMethod]
    [DataRow(false, false)]
    [DataRow(true, false)]
    [DataRow(false, true)]
    [DataRow(true, true)]
    public void AcceptedJoinMismatchKeepsTheRequestedContinuousKnot(bool bankMismatch, bool withinFitTolerance)
    {
        const double join = 5;
        double bankJump = bankMismatch ? withinFitTolerance ? 1e-7 : 0.001 : 0;
        ScalarTimeline? bank = bankMismatch ? new ScalarTimeline(ProfileDomain.Distance,
            [FunctionSegment.Transition(0, join, 0, 0), FunctionSegment.Transition(join, 10, bankJump, bankJump)],
            allowDiscontinuities: true) : null;
        double rightTarget = bankMismatch ? 5 : withinFitTolerance ? 5.000001 : 5.005;
        var source = Reconstruct([Targets(0, join, 5), Targets(join, 10, rightTarget)], Distance(0, 10),
            bank, settings: Settings(continuity: [0.01], bankJoinTolerance: 0.002));

        var left = source.Evaluate(join, EvaluationSide.Left);
        var right = source.Evaluate(join, EvaluationSide.Right);
        Assert.AreEqual(rightTarget, right.LateralCoordinateAccelerationMetresPerSecondSquared);
        Assert.AreEqual(bankJump, right.BankRadians);
        Assert.IsNull(source.Evaluate(join).CurvatureVectorPerMetre);
        double requiredError = (left.CurvatureVectorPerMetre!.Value - right.CurvatureVectorPerMetre!.Value).Length / 2;

        var settings = FitSettings();
        var fit = HeartlineReconstructor.FitToTrack(source, settings);
        if (withinFitTolerance)
        {
            Assert.IsTrue(fit.Succeeded, fit.Failure?.ToString());
            Assert.AreEqual(2, fit.Track!.Heartline.Definition.ContinuityAt(join));
            double station = fit.GetFittedStation(join);
            Assert.AreEqual(0, fit.Track.Banking.Evaluate(station, EvaluationSide.Left).Value);
            Assert.AreEqual(bankJump, fit.Track.Banking.Evaluate(station, EvaluationSide.Right).Value);
            return;
        }

        Assert.IsFalse(fit.Succeeded);
        Assert.AreSame(source, fit.Source);
        Assert.AreEqual(HeartlineFailureCode.FitToleranceExceeded, fit.Failure!.Code);
        Assert.AreEqual((join, join), fit.Failure.SourceStationInterval);
        Assert.AreEqual("Minimum sided curvature error", fit.Failure.Measurement!.Value.Quantity);
        Assert.AreEqual(requiredError, fit.Failure.Measurement.Value.Observed);
        Assert.IsTrue(requiredError > settings.CurvatureTolerancePerMetre);
        Assert.AreEqual(0, fit.Diagnostics.RefinementPasses);
        Assert.AreEqual(0, fit.Diagnostics.VerificationSamples);
        Assert.IsNull(fit.Diagnostics.Solver);
    }
}
