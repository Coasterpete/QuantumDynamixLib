using System;
using System.Linq;
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
public sealed class HeartlineReconstructionTests
{
    public TestContext TestContext { get; set; } = null!;

    [TestMethod]
    public void StraightUsesLocalStationAndPreservesPitchedInitialPoseAndAuthoredBank()
    {
        var pose = new RigidPose3d(new(4, -3, 2), Quaterniond.FromAxisAngle(Vector3d.UnitY, -0.4));
        Vector3d forward = pose.TransformDirection(Vector3d.UnitX);
        Vector3d up = pose.TransformDirection(Vector3d.UnitZ);
        var source = Reconstruct([Targets(100, 112, 0)], Distance(100, 112),
            Function(ProfileDomain.Distance, 100, 112, s => new(0.07 * (s - 100), 0.07)), pose);
        Assert.AreEqual(100, source.AuthoredStartStationMetres);
        Assert.AreEqual(12, source.LengthMetres);
        foreach (double s in new[] { 0, 2.7, 7.132, 12 })
        {
            var sample = source.Evaluate(s);
            Assert.AreEqual(s, sample.StationMetres);
            Assert.AreEqual(100 + s, sample.Motion.HeartlineStationMetres, 1e-13);
            Assert.AreEqual(100 + s, sample.AuthoredCoordinate, 1e-13);
            TestMath.Near(pose.Position + forward * s, sample.UnbankedFrame.Position, 2e-13);
            TestMath.Near(forward, sample.UnbankedFrame.Forward, 2e-14);
            TestMath.Near(up, sample.UnbankedFrame.Up, 2e-14);
            TestMath.Near(Vector3d.Zero, sample.CurvatureVectorPerMetre!.Value, 0);
            Assert.AreEqual(0.07 * s, sample.BankRadians, 2e-14);
        }
        Assert.ThrowsExactly<ArgumentOutOfRangeException>(() => source.Evaluate(100));
        Log(TestContext, "straight", source);
    }

    [TestMethod]
    [DataRow(false)]
    [DataRow(true)]
    public void QuarterCircleMatchesAnalyticPositionTangentAndCurvature(bool vertical)
    {
        double length = 10 * System.Math.PI;
        var source = Reconstruct([Targets(0, length, vertical ? 0 : 5, vertical ? 5 : 0)], Distance(0, length));
        string fixture = vertical ? "vertical quarter circle" : "horizontal quarter circle";
        VerifyAnalytic(TestContext, fixture, source, [0, 2.331, 7.119, 13.492, 22.091, length],
            s => (CirclePosition(s, vertical), CircleTangent(s, vertical), CircleCurvature(s, vertical)));
        Log(TestContext, fixture, source);
    }

    [TestMethod]
    public void ChangingBankReconstructsThreeDimensionalHelix()
    {
        var source = Helix();
        double[] stations = [0, 0.671, 4.337, 7.193, 10.031, 12.5];
        VerifyAnalytic(TestContext, "changing-bank helix", source, stations, s =>
        {
            var (si, co) = System.Math.SinCos(s / 5);
            return (new(4 * co, 4 * si, 0.6 * s), new(-0.8 * si, 0.8 * co, 0.6), new(-0.16 * co, -0.16 * si, 0));
        });
        foreach (double s in stations)
        {
            var sample = source.Evaluate(s);
            var (si, co) = System.Math.SinCos(s / 5);
            Vector3d expectedNormal = new(-co, -si, 0);
            TestMath.Near(expectedNormal, sample.BankedFrame.Left, 2e-8);
            Assert.AreEqual(0.12 * s, sample.BankRadians, 0);
        }
        Log(TestContext, "changing-bank helix", source);
    }

    [TestMethod]
    public void ArbitraryStationPositionAndTangentDifferentiateToTheReportedGeometry()
    {
        var source = Helix();
        const double h = 1e-4;
        foreach (double s in new[] { 0.821, 3.019, 5.371, 9.203, 12.041 })
        {
            var left = source.Evaluate(s - h);
            var centre = source.Evaluate(s);
            var right = source.Evaluate(s + h);
            TestMath.Near(centre.UnbankedFrame.Forward, (right.UnbankedFrame.Position - left.UnbankedFrame.Position) / (2 * h), 3e-9);
            TestMath.Near(centre.CurvatureVectorPerMetre!.Value, (right.UnbankedFrame.Forward - left.UnbankedFrame.Forward) / (2 * h), 3e-9);
        }
    }

    [TestMethod]
    public void AdjacentRegionsAndIndependentChannelBreakpointsPreserveCircle()
    {
        var independentlySegmented = new ScalarTimeline(ProfileDomain.Distance,
            [FunctionSegment.Transition(0, 2, 0, 0), FunctionSegment.Transition(2, 7, 0, 0)]);
        var source = Reconstruct([Targets(Constant(ProfileDomain.Distance, 0, 7, 5), independentlySegmented),
            Targets(7, 16, 5)], Distance(0, 16), settings: Settings(continuity: [0, 0]));
        CollectionAssert.AreEqual(new[] { 0.0, 2, 7, 16 }, source.BoundaryStationsMetres.ToArray());
        Assert.AreEqual(3, source.Diagnostics.AccelerationJoins.Count);
        foreach (double s in new[] { 2.0, 7, 11.283, 16 })
        {
            TestMath.Near(CirclePosition(s), source.Evaluate(s).UnbankedFrame.Position, 2e-7);
            TestMath.Near(source.Evaluate(s, EvaluationSide.Left).CurvatureVectorPerMetre!.Value,
                source.Evaluate(s, EvaluationSide.Right).CurvatureVectorPerMetre!.Value, 2e-14);
        }
        Log(TestContext, "continuous Regions", source);
    }

    [TestMethod]
    public void ExplicitCurvatureStepKeepsSharedPositionAndTangent()
    {
        Region[] regions = [Targets(17, 22, 0), Targets(22, 32, 5)];
        var trajectory = Distance(17, 32);
        var ex = Assert.ThrowsExactly<HeartlineReconstructionException>(() => Reconstruct(regions, trajectory));
        Assert.AreEqual(HeartlineFailureCode.ContinuityRequirementFailed, ex.Failure.Code);
        Assert.AreEqual((5.0, 5.0), ex.Failure.SourceStationInterval);

        var source = Reconstruct(regions, trajectory, settings: Settings(steps: [22]));
        var left = source.Evaluate(5, EvaluationSide.Left);
        var right = source.Evaluate(5, EvaluationSide.Right);
        Assert.AreEqual(left.UnbankedFrame, right.UnbankedFrame);
        Assert.AreEqual(22, right.Motion.HeartlineStationMetres);
        Assert.IsNull(source.Evaluate(5).CurvatureVectorPerMetre);
        TestMath.Near(Vector3d.Zero, left.CurvatureVectorPerMetre!.Value, 0);
        TestMath.Near(Vector3d.UnitY * 0.05, right.CurvatureVectorPerMetre!.Value, 1e-15);
        TestMath.Near(new Vector3d(5, 0, 0) + CirclePosition(10), source.Evaluate(15).UnbankedFrame.Position, 2e-7);
        Log(TestContext, "straight to arc curvature step", source);
    }

    [TestMethod]
    public void RoundedAuthoredStationKeepsTheIncomingSideOfMotionAndTransverseJoins()
    {
        const double origin = 1_000_000;
        var profile = new ScalarTimeline(ProfileDomain.Distance,
            [FunctionSegment.Transition(origin, origin + 5, 0, 0), FunctionSegment.Transition(origin + 5, origin + 10, 2, 2)],
            allowDiscontinuities: true);
        var source = Reconstruct([Targets(origin, origin + 5, 0), Targets(origin + 5, origin + 10, 5)],
            Distance(profile), settings: Settings(steps: [origin + 5]));
        var before = source.Evaluate(System.Math.BitDecrement(5.0));
        Assert.AreEqual(origin + 5, before.Motion.HeartlineStationMetres);
        Assert.AreEqual(0.0, before.Motion.LongitudinalAccelerationMetresPerSecondSquared);
        Assert.AreEqual(0.0, before.LateralCoordinateAccelerationMetresPerSecondSquared);
        Assert.AreEqual(2.0, source.Evaluate(5).Motion.LongitudinalAccelerationMetresPerSecondSquared);
        Assert.AreEqual(5.0, source.Evaluate(5).LateralCoordinateAccelerationMetresPerSecondSquared);
    }

    [TestMethod]
    public void StraightAndInflectionFollowIntegratedHeadingWithoutNormalFlip()
    {
        const double k = 0.004;
        var source = Reconstruct([Targets(0, 2, 0),
            Targets(Function(ProfileDomain.Distance, 2, 10, s => new(100 * k * (s - 2) * (s - 6))),
                Constant(ProfileDomain.Distance, 2, 10, 0))], Distance(0, 10));
        foreach (double s in new[] { 1.3, 2, 3.111, 5.998, 6, 6.002, 8.713, 10 })
        {
            var sample = source.Evaluate(s);
            double heading = Heading(s);
            TestMath.Near(new(System.Math.Cos(heading), System.Math.Sin(heading), 0), sample.UnbankedFrame.Forward, 2e-8);
            TestMath.Near(Vector3d.UnitZ, sample.UnbankedFrame.Up, 2e-14);
            double start = System.Math.Min(2, s);
            Vector3d expected = new(start + Simpson(x => System.Math.Cos(Heading(x)), start, s),
                Simpson(x => System.Math.Sin(Heading(x)), start, s), 0);
            TestMath.Near(expected, sample.UnbankedFrame.Position, 2e-7);
        }
        Log(TestContext, "straight and inflection", source);

        static double Heading(double s)
        {
            double h = System.Math.Max(0, s - 2);
            return k * h * h * (h / 3 - 2);
        }
    }

    [TestMethod]
    [DataRow(ProfileDomain.Time, ProfileDomain.Time)]
    [DataRow(ProfileDomain.Time, ProfileDomain.Distance)]
    [DataRow(ProfileDomain.Distance, ProfileDomain.Time)]
    [DataRow(ProfileDomain.Distance, ProfileDomain.Distance)]
    public void TimeAndDistanceAuthoringAgreeWithNonzeroOrigins(ProfileDomain motionDomain, ProfileDomain targetDomain)
    {
        const double clock = 30;
        const double stationOrigin = 100;
        LongitudinalTrajectory motion = motionDomain == ProfileDomain.Time
            ? Time(Constant(ProfileDomain.Time, clock, clock + 2, 2), stationOrigin, 10)
            : Distance(stationOrigin, stationOrigin + 24, 10, 2);
        bool byTime = targetDomain == ProfileDomain.Time;
        double start = byTime ? clock : stationOrigin;
        double join = start + (byTime ? 1 : 11);
        double end = start + (byTime ? 2 : 24);
        Region[] regions = [Make(start, join), Make(join, end)];
        var source = Reconstruct(regions, motion, clock: clock);
        Assert.AreEqual(24, source.LengthMetres, 2e-12);
        Assert.IsTrue(source.BoundaryStationsMetres.Any(s => System.Math.Abs(s - 11) < 1e-8));
        foreach (double s in new[] { 0, 2.047, 10.137, 16.271, source.LengthMetres })
        {
            var actual = source.Evaluate(s);
            double elapsed = s / (5 + System.Math.Sqrt(25 + s));
            Assert.AreEqual(stationOrigin + s, actual.Motion.HeartlineStationMetres, 1e-8);
            Assert.AreEqual(elapsed, actual.Motion.ElapsedSeconds, 1e-8);
            Assert.AreEqual(byTime ? clock + elapsed : stationOrigin + s, actual.AuthoredCoordinate, 1e-8);
            Assert.AreEqual(s, actual.StationMetres, 0);
            TestMath.Near(CirclePosition(s), actual.UnbankedFrame.Position, 2e-7);
            TestMath.Near(CircleCurvature(s), actual.CurvatureVectorPerMetre!.Value, 2e-7);
        }
        Log(TestContext, $"{motionDomain} motion / {targetDomain} targets", source);

        Region Make(double a, double b) => Targets(Function(targetDomain, a, b, x =>
        {
            double t = x - clock;
            return byTime ? new((10 + 2 * t) * (10 + 2 * t) / 20) : new((100 + 4 * (x - stationOrigin)) / 20);
        }), Constant(targetDomain, a, b, 0));
    }

    [TestMethod]
    [DataRow(ProfileDomain.Time, false)]
    [DataRow(ProfileDomain.Time, true)]
    [DataRow(ProfileDomain.Distance, false)]
    [DataRow(ProfileDomain.Distance, true)]
    public void MinimumForwardSpeedIsInclusiveButCannotBeRoundedUp(ProfileDomain domain, bool braking)
    {
        double minimum = braking ? 2 : 1.1;
        double initial = braking ? 3 : minimum;
        double acceleration = braking ? -1 : 0;
        var motion = domain == ProfileDomain.Time
            ? Time(Constant(domain, 30, 31, acceleration), 100, initial)
            : Distance(100, braking ? 102.5 : 101, initial, acceleration);
        Region[] regions = [Targets(100, 103, 0)];

        var source = Reconstruct(regions, motion, settings: Settings(minimumSpeed: minimum));
        Assert.AreEqual(minimum, source.Diagnostics.ProvenForwardSpeedLowerBoundMetresPerSecond);
        Assert.AreEqual(minimum, source.Evaluate(source.LengthMetres).Motion.HeartlineVelocityMetresPerSecond, 2e-14);
        Assert.IsTrue(Reconstruct(regions, motion, settings: Settings(minimumSpeed: System.Math.BitDecrement(minimum)))
            .Diagnostics.ProvenForwardSpeedLowerBoundMetresPerSecond >= System.Math.BitDecrement(minimum));

        var failure = Assert.ThrowsExactly<HeartlineReconstructionException>(() =>
            Reconstruct(regions, motion, settings: Settings(minimumSpeed: System.Math.BitIncrement(minimum))));
        Assert.AreEqual(braking ? HeartlineFailureCode.SpeedPositivityUnresolved : HeartlineFailureCode.SpeedBelowMinimum,
            failure.Failure.Code);
    }

    private static double Simpson(Func<double, double> f, double a, double b)
    {
        const int count = 2048;
        double step = (b - a) / count;
        double sum = f(a) + f(b);
        for (int i = 1; i < count; i++) { sum += (i % 2 == 0 ? 2 : 4) * f(a + i * step); }
        return sum * step / 3;
    }
}
