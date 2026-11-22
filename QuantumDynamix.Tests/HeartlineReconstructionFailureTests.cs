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
}
