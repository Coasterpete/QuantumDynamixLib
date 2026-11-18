using System;
using System.Linq;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using QuantumDynamix.Geometry;
using QuantumDynamix.Math;
using QuantumDynamix.Math.Functions;
using QuantumDynamix.Math.Numerics;
using QuantumDynamix.Math.Timelines;
using QuantumDynamix.Track;
using static QuantumDynamix.Tests.RailTestFixtures;

namespace QuantumDynamix.Tests;

[TestClass]
public sealed class RailDistanceTests
{
    [TestMethod]
    [DataRow(0.01)]
    [DataRow(1.0)]
    [DataRow(1000.0)]
    public void CircularHeartlineSpineAndTwoRailsHaveTheirOwnKnownLengthsAndCurvatures(double radius)
    {
        var circle = SplineFoundationTests.QuarterCircle(radius);
        double length = new ArcLengthMap(circle).Length;
        var track = new HeartlineTrack(circle, Vector3d.UnitZ, TrackOffsetProfile.Constant(new(0, radius * 0.1, 0), length),
            features: new[] { new TrackFeature("start", circle.Minimum), new TrackFeature("middle", 0.5), new TrackFeature("end", circle.Maximum) });
        var geometry = new RailGeometry(track, new(new[] { Rail("inner", new(0, radius * 0.05, 0), length, Section(radius * 0.001)),
            Rail("outer", new(0, -radius * 0.05, 0), length, Section(radius * 0.001)) }), Settings);
        TrackDistanceMap spine = geometry.CreateSpineDistanceMap();
        Assert.IsTrue(spine.IsSpine);
        Assert.AreEqual(radius * 0.9 * System.Math.PI / 2, spine.Length, 2e-9 * System.Math.Max(1, radius));
        Assert.AreEqual(track.IntegrateSpineLength().Value, spine.Length, 2e-9 * System.Math.Max(1, radius));
        foreach (var pair in new[] { (Id: "inner", Factor: 0.85), (Id: "outer", Factor: 0.95) })
        {
            TrackDistanceMap map = geometry.CreateRailDistanceMap(pair.Id);
            Assert.AreEqual(pair.Id, map.RailId);
            Assert.AreEqual(length * pair.Factor, map.Length, 2e-9 * System.Math.Max(1, radius));
            RailSample sample = geometry.Evaluate(pair.Id, length * 0.4);
            Assert.AreEqual(radius * pair.Factor, sample.Position.Length, 2e-11 * System.Math.Max(1, radius));
            Assert.AreEqual(1 / (radius * pair.Factor), sample.Curvature, 2e-10 / radius);
            Assert.AreEqual(pair.Factor, sample.SpeedPerHeartlineMetre, 2e-13);
            foreach (double fraction in new[] { 0, 0.1, 0.4, 0.8, 1 })
            {
                double station = map.StationAtHeartlineStation(length * fraction);
                Assert.AreEqual(length * fraction * pair.Factor, station, 2e-9 * System.Math.Max(1, radius));
                TrackDistanceInversion inverse = map.Invert(station);
                Assert.AreEqual(length * fraction, inverse.HeartlineStation, 2e-8 * System.Math.Max(1, radius));
                Assert.IsTrue(System.Math.Abs(inverse.StationResidual) + inverse.EstimatedLengthError <= map.Settings.AbsoluteTolerance + map.Settings.RelativeTolerance * map.Length);
            }
            Assert.AreEqual(map.Length, map.ResolveFeatures().Resolve("end"));
            Assert.AreEqual(map.Length / 2, map.ResolveFeatures().Resolve("middle"), 2e-9 * System.Math.Max(1, radius));
            Assert.AreEqual(0, map.Nodes[0].Station);
            Assert.AreEqual(length, map.Nodes[^1].HeartlineStation);
            Assert.AreEqual(geometry.Track.Frames.EstimatedAccumulatedAngularError, map.EstimatedFrameAngularError);
            Assert.IsTrue(map.ReferenceStationErrorBudget > 0);
        }
        Assert.AreEqual(0, geometry.Validate().Count);
    }

    [TestMethod]
    public void NonuniformRailMetricIntegratesParabolaAndInvertsWithReportedResidual()
    {
        var rail = new RailDefinition("r", new(Constant(0, 10), new CustomFunction(0, 10, (s, _) => new(s * s, 2 * s, 2)), Constant(0, 10)),
            Section(), Constant(0, 10));
        RailGeometry geometry = Straight(rail);
        TrackDistanceMap map = geometry.CreateRailDistanceMap("r");
        static double LengthTo(double s) => s * System.Math.Sqrt(1 + 4 * s * s) / 2 + System.Math.Asinh(2 * s) / 4;
        Assert.AreEqual(LengthTo(10), map.Length, 2e-8);
        foreach (double station in new[] { 0.01, 0.3, 2, 6, 9.99 })
        {
            Assert.AreEqual(LengthTo(station), map.StationAtHeartlineStation(station), 2e-8);
            TrackDistanceInversion inverse = map.Invert(LengthTo(station));
            Assert.AreEqual(station, inverse.HeartlineStation, 2e-8);
            Assert.IsTrue(System.Math.Abs(inverse.StationResidual) + inverse.EstimatedLengthError <= map.Settings.AbsoluteTolerance + map.Settings.RelativeTolerance * map.Length);
        }
        Assert.ThrowsExactly<NumericalConvergenceException>(() => map.Invert(map.Length * 0.5, 1));
        Assert.ThrowsExactly<NumericalConvergenceException>(() => geometry.CreateRailDistanceMap("r", new QuadratureSettings(1e-12, 0, maximumEvaluations: 15)));
        Assert.ThrowsExactly<ArgumentOutOfRangeException>(() => map.Invert(map.Length + 1));
        Assert.ThrowsExactly<ArgumentOutOfRangeException>(() => map.Invert(double.NaN));
        Assert.ThrowsExactly<ArgumentOutOfRangeException>(() => map.Invert(0, 0));
        Assert.ThrowsExactly<ArgumentOutOfRangeException>(() => map.StationAtHeartlineStation(11));
    }

    [TestMethod]
    public void LengthMapPartitionsContinuousProfileJoinsWithoutEquatingItsMetricToHeartline()
    {
        var lateral = ScalarTimeline.FromKeyframes(ProfileDomain.Distance, new[] {
            new TimelineKeyframe(0, 0), new TimelineKeyframe(5, 0), new TimelineKeyframe(10, 5) });
        var geometry = Straight(new RailDefinition("r", new(Constant(0, 10), lateral, Constant(0, 10)), Section(), Constant(0, 10)));
        TrackDistanceMap map = geometry.CreateRailDistanceMap("r");
        Assert.AreEqual(5 + 5 * System.Math.Sqrt(2), map.Length, 2e-10);
        Assert.AreEqual(5, map.StationAtHeartlineStation(5), 2e-12);
        Assert.AreEqual(5, map.Invert(5).HeartlineStation);
        Assert.IsTrue(map.Nodes.Any(n => n.HeartlineStation == 5));
        Assert.IsTrue(geometry.Validate().Any(d => d.Code == RailDiagnosticCode.TangentDiscontinuity));
    }

    [TestMethod]
    public void PositionJumpIsRejectedInsteadOfIntegratingAcrossMissingGeometry()
    {
        var jump = new CustomFunction(0, 10, (s, side) => new(s < 5 || s == 5 && side == EvaluationSide.Left ? 0 : 1), new[] { 5.0 });
        var geometry = Straight(new RailDefinition("r", new(Constant(0, 10), jump, Constant(0, 10)), Section(), Constant(0, 10)));
        RailGeometryException failure = Assert.ThrowsExactly<RailGeometryException>(() => geometry.CreateRailDistanceMap("r"));
        Assert.AreEqual(RailDiagnosticCode.PositionDiscontinuity, failure.Diagnostic.Code);
        Assert.AreEqual(5, failure.Diagnostic.HeartlineStation);
    }

    [TestMethod]
    public void SectionFrameSingularityDoesNotRemoveValidCenterlineDistance()
    {
        var rail = new RailDefinition("r", new(new CustomFunction(0, 10, (s, _) => new(-s, -1)),
            new CustomFunction(0, 10, (s, _) => new(s, 1)), Constant(0, 10)), Section(), Constant(0, 10));
        RailGeometry geometry = Straight(rail);
        RailSample sample = geometry.Evaluate("r", 2);
        TestMath.Near(Vector3d.UnitY, sample.Tangent, 2e-14);
        Assert.AreEqual(0, sample.Curvature);
        RailGeometryException failure = Assert.ThrowsExactly<RailGeometryException>(() => sample.SectionFrame);
        Assert.AreEqual(RailDiagnosticCode.SingularSectionFrame, failure.Diagnostic.Code);
        Assert.AreEqual(10, geometry.CreateRailDistanceMap("r").Length, 2e-12);
        Assert.IsTrue(geometry.Validate().Any(d => d.Code == RailDiagnosticCode.SingularSectionFrame));
    }

    [TestMethod]
    public void DistanceMapDoesNotLoseAnOffsetJumpSmallerThanWorldCoordinateResolution()
    {
        var jump = new CustomFunction(0, 10, (s, side) => new(s < 5 || s == 5 && side == EvaluationSide.Left ? 0 : 1e-5), new[] { 5.0 });
        var geometry = new RailGeometry(new(Line(origin: new(1e12, 1e12, 1e12)), Vector3d.UnitZ, TrackOffsetProfile.Constant(default, 10)),
            new(new[] { new RailDefinition("r", new(Constant(0, 10), jump, Constant(0, 10)), Section(), Constant(0, 10)) }), Settings);
        RailGeometryException failure = Assert.ThrowsExactly<RailGeometryException>(() => geometry.CreateRailDistanceMap("r"));
        Assert.AreEqual(RailDiagnosticCode.PositionDiscontinuity, failure.Diagnostic.Code);
        Assert.AreEqual(1e-5, failure.Diagnostic.Residual);
        Assert.IsTrue(geometry.Validate().Any(d => d.Code == RailDiagnosticCode.PositionDiscontinuity));
    }
}
