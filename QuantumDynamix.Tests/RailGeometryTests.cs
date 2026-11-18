using System;
using System.Collections.Generic;
using System.Linq;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using QuantumDynamix.Geometry;
using QuantumDynamix.Math;
using QuantumDynamix.Math.Functions;
using QuantumDynamix.Math.Timelines;
using QuantumDynamix.Splines;
using QuantumDynamix.Splines.Curves;
using QuantumDynamix.Track;
using static QuantumDynamix.Tests.RailTestFixtures;

namespace QuantumDynamix.Tests;

[TestClass]
public sealed class RailGeometryTests
{
    [TestMethod]
    public void ArbitraryArrangementUsesBankedSpineRelativeCoordinatesAndIndependentSectionRoll()
    {
        const double bank = 0.7;
        const double roll = -0.2;
        Vector3d spine = new(0.3, -0.4, 1.2);
        Vector3d[] positions = { new(0, 0.5, -0.3), new(0.2, -0.6, 0.4), new(-0.3, 0.1, 0.8) };
        var track = new HeartlineTrack(Line(), Vector3d.UnitZ, TrackOffsetProfile.Constant(spine, 10), Constant(bank, 10));
        var geometry = new RailGeometry(track, new(positions.Select((p, i) => Rail($"r{i}", p, 10, roll: Constant(roll, 10)))), Settings);
        for (int i = 0; i < positions.Length; i++)
        {
            Vector3d q = spine + positions[i];
            RailSample sample = geometry.Evaluate($"r{i}", 4);
            TestMath.Near(new(4 + q.X, q.Y * System.Math.Cos(bank) - q.Z * System.Math.Sin(bank),
                q.Y * System.Math.Sin(bank) + q.Z * System.Math.Cos(bank)), sample.Position, 2e-14);
            TestMath.Near(Vector3d.UnitX, sample.Centerline.FirstDerivative, 2e-14);
            TestMath.Near(default, sample.Centerline.SecondDerivative, 2e-14);
            Assert.AreEqual(0, sample.Curvature);
            TestMath.Near(new(0, System.Math.Cos(bank + roll), System.Math.Sin(bank + roll)), sample.SectionFrame.Left, 2e-14);
            TestMath.Near(Vector3d.Cross(sample.SectionFrame.Forward, sample.SectionFrame.Left), sample.SectionFrame.Up, 2e-14);
        }
        Assert.AreEqual(3, geometry.Arrangement.Rails.Count);
        Assert.AreEqual(0, geometry.Validate().Count);
    }

    [TestMethod]
    public void BankedRailMatchesAnalyticHelixAndItsOwnCurvature()
    {
        const double k = 0.3;
        const double h = 2;
        const double s = 3;
        var track = new HeartlineTrack(Line(), Vector3d.UnitZ, TrackOffsetProfile.Constant(new(0, 0, 0.7), 10),
            new CustomFunction(0, 10, (x, _) => new(k * x, k)));
        var geometry = new RailGeometry(track, new(new[] { Rail("rail", new(0, 0, 1.3), 10) }), Settings);
        RailSample sample = geometry.Evaluate("rail", s);
        TestMath.Near(new(s, -h * System.Math.Sin(k * s), h * System.Math.Cos(k * s)), sample.Position, 3e-13);
        TestMath.Near(new(1, -h * k * System.Math.Cos(k * s), -h * k * System.Math.Sin(k * s)), sample.Centerline.FirstDerivative, 3e-13);
        TestMath.Near(new(0, h * k * k * System.Math.Sin(k * s), -h * k * k * System.Math.Cos(k * s)), sample.Centerline.SecondDerivative, 3e-13);
        Assert.AreEqual(System.Math.Sqrt(1 + h * h * k * k), sample.SpeedPerHeartlineMetre, 2e-14);
        Assert.AreEqual(h * k * k / (1 + h * h * k * k), sample.Curvature, 2e-14);
        TestMath.Near(sample.Tangent, sample.SectionFrame.Forward, 2e-14);
        Assert.IsTrue(Vector3d.Dot(sample.SectionFrame.Forward, sample.BankedReferenceFrame.Forward) < 0.99);
        Assert.AreEqual(10 * System.Math.Sqrt(1 + h * h * k * k), geometry.CreateRailDistanceMap("rail").Length, 2e-9);
    }

    [TestMethod]
    public void VaryingForwardLateralAndVerticalOffsetsHaveAnalyticStationDerivatives()
    {
        var offsets = new TrackOffsetProfile(new CustomFunction(0, 10, (s, _) => new(0.1 * s * s, 0.2 * s, 0.2)),
            new CustomFunction(0, 10, (s, _) => new(-0.3 * s * s, -0.6 * s, -0.6)),
            new CustomFunction(0, 10, (s, _) => new(0.02 * s * s * s, 0.06 * s * s, 0.12 * s, 0.12)));
        var geometry = Straight(new RailDefinition("r", offsets, Section(), Constant(0, 10)));
        RailSample sample = geometry.Evaluate("r", 2);
        TestMath.Near(new(2.4, -1.2, 0.16), sample.Position, 2e-14);
        TestMath.Near(new(1.4, -1.2, 0.24), sample.Centerline.FirstDerivative, 2e-14);
        TestMath.Near(new(0.2, -0.6, 0.24), sample.Centerline.SecondDerivative, 2e-14);
        Assert.AreEqual(0, geometry.Validate().Count);
    }

    [TestMethod]
    public void SimultaneousBankAndChangingRailOffsetsIncludeBothProductRuleCrossTerms()
    {
        const double k = 0.2;
        const double s = 3;
        double h = 0.1 * s * s;
        double first = 0.2 * s;
        const double second = 0.2;
        var track = new HeartlineTrack(Line(), Vector3d.UnitZ, TrackOffsetProfile.Constant(default, 10),
            new CustomFunction(0, 10, (x, _) => new(k * x, k)));
        var offsets = new TrackOffsetProfile(Constant(0, 10), Constant(0, 10),
            new CustomFunction(0, 10, (x, _) => new(0.1 * x * x, 0.2 * x, 0.2)));
        var geometry = new RailGeometry(track, new(new[] { new RailDefinition("r", offsets, Section(), Constant(0, 10)) }), Settings);
        RailSample sample = geometry.Evaluate("r", s);
        double sin = System.Math.Sin(k * s);
        double cos = System.Math.Cos(k * s);
        TestMath.Near(new(s, -h * sin, h * cos), sample.Position, 1e-13);
        TestMath.Near(new(1, -first * sin - h * k * cos, first * cos - h * k * sin), sample.Centerline.FirstDerivative, 1e-13);
        TestMath.Near(new(0, -second * sin - 2 * first * k * cos + h * k * k * sin,
            second * cos - 2 * first * k * sin - h * k * k * cos), sample.Centerline.SecondDerivative, 1e-13);
    }

    [TestMethod]
    public void RationalNurbsCrossSectionRetainsItsGeometryAndProfileDerivatives()
    {
        const double radius = 0.2;
        SplineCurve3d source = SplineFoundationTests.QuarterCircle(radius);
        var boundary = new SplineCurve3d(source.Definition, source.ControlPoints.Select(p => new Vector3d(0, p.X, p.Y)));
        var geometry = Straight(Rail("r", default, 10, new(new[] { boundary })));
        RailSurfaceSample surface = geometry.EvaluateSurface("r", 3, 0, 0.5);
        TestMath.Near(new(3, radius / System.Math.Sqrt(2), radius / System.Math.Sqrt(2)), surface.Position, 1e-14);
        TestMath.Near(Vector3d.UnitX, surface.HeartlineStationDerivative, 1e-14);
        Assert.AreEqual(0, Vector3d.Dot(surface.Position - new Vector3d(3, 0, 0), surface.ProfileDerivative), 1e-14);
        Assert.AreEqual(surface.ProfileDerivative.Length, surface.AreaVector.Length, 1e-14);
        Assert.AreEqual(0, geometry.Validate().Count);
    }

    [TestMethod]
    public void SectionAngularRateAndSurfacePartialsAgreeWithIndependentSpatialDifferences()
    {
        var curve = SplineCurve3d.Clamped(new Vector3d[] { new(0, 0, 0), new(1, 1, -1), new(3, 2, 2), new(5, -1, 4), new(6, 2, 3), new(8, 0, 6) });
        double length = new ArcLengthMap(curve).Length;
        var track = new HeartlineTrack(curve, Vector3d.UnitZ, TrackOffsetProfile.Constant(new(0.1, 0.2, -0.5), length),
            new CustomFunction(0, length, (s, _) => new(0.04 * s * s, 0.08 * s, 0.08)), frameAngularTolerance: 1e-10);
        var geometry = new RailGeometry(track, new(new[] { Rail("r", new(0.1, 0.4, 0.2), length,
            roll: new CustomFunction(0, length, (s, _) => new(0.2 * s, 0.2))) }), Settings);
        double station = 0.43 * length;
        const double delta = 2e-4;
        RailSample sample = geometry.Evaluate("r", station);
        RailSample before = geometry.Evaluate("r", station - delta);
        RailSample after = geometry.Evaluate("r", station + delta);
        TestMath.Near((after.Position - before.Position) / (2 * delta), sample.Centerline.FirstDerivative, 2e-7);
        TestMath.Near((after.Centerline.FirstDerivative - before.Centerline.FirstDerivative) / (2 * delta), sample.Centerline.SecondDerivative, 2e-7);
        TestMath.Near((after.SectionFrame.Left - before.SectionFrame.Left) / (2 * delta),
            Vector3d.Cross(sample.SectionAngularRate, sample.SectionFrame.Left), 2e-7);
        TestMath.Near((after.SectionFrame.Up - before.SectionFrame.Up) / (2 * delta),
            Vector3d.Cross(sample.SectionAngularRate, sample.SectionFrame.Up), 2e-7);
        const double v = 0.7;
        RailSurfaceSample surface = geometry.EvaluateSurface("r", station, 0, v);
        TestMath.Near((geometry.EvaluateSurface("r", station + delta, 0, v).Position - geometry.EvaluateSurface("r", station - delta, 0, v).Position)
            / (2 * delta), surface.HeartlineStationDerivative, 2e-7);
        TestMath.Near((geometry.EvaluateSurface("r", station, 0, v + delta).Position - geometry.EvaluateSurface("r", station, 0, v - delta).Position)
            / (2 * delta), surface.ProfileDerivative, 2e-10);
        Assert.IsTrue(surface.AreaVector.Length > 0);
    }

    [TestMethod]
    public void SpatialGeometryIsEquivariantUnderRigidWorldTransform()
    {
        Vector3d[] controls = { new(0, 0, 0), new(1, 2, -1), new(3, 2, 2), new(5, -1, 4), new(6, 2, 3), new(7, 0, 6) };
        Quaterniond rotation = Quaterniond.FromAxisAngle(new(1, 2, 3), 0.9);
        Vector3d origin = new(300, -200, 100);
        RailGeometry Make(SplineCurve3d curve, Vector3d up)
        {
            double length = new ArcLengthMap(curve).Length;
            var track = new HeartlineTrack(curve, up, TrackOffsetProfile.Constant(new(0, 0, -0.5), length), Constant(0.4, length), frameAngularTolerance: 1e-9);
            return new(track, new(new[] { Rail("r", new(0.1, 0.3, 0), length, roll: Constant(0.3, length)) }), Settings);
        }
        RailGeometry a = Make(SplineCurve3d.Clamped(controls), Vector3d.UnitZ);
        RailGeometry b = Make(SplineCurve3d.Clamped(controls.Select(p => rotation.Rotate(p) + origin)), rotation.Rotate(Vector3d.UnitZ));
        Assert.AreEqual(5, a.Track.Heartline.Definition.Degree);
        foreach (double s in new[] { 0, a.HeartlineLength * 0.4, a.HeartlineLength })
        {
            RailSample x = a.Evaluate("r", s);
            RailSample y = b.Evaluate("r", s, EvaluationSide.Left);
            TestMath.Near(rotation.Rotate(x.Position) + origin, y.Position, 2e-8);
            TestMath.Near(rotation.Rotate(x.Centerline.FirstDerivative), y.Centerline.FirstDerivative, 2e-8);
            TestMath.Near(rotation.Rotate(x.CurvatureVector), y.CurvatureVector, 2e-8);
            TestMath.Near(rotation.Rotate(x.SectionFrame.Up), y.SectionFrame.Up, 2e-8);
        }
    }

    [TestMethod]
    public void SpacingAndGaugeHaveExplicitAndDifferentMeasurementConventions()
    {
        var rails = new[] { Rail("a", default, 10), Rail("b", new(0, 3, 4), 10) };
        var spacings = new[] { new RailSpacingDefinition("centers", "a", "b", RailSpacingMeasurement.Euclidean, Constant(5, 10)),
            new RailSpacingDefinition("lateral", "a", "b", RailSpacingMeasurement.BankedLateral, Constant(3, 10)),
            new RailSpacingDefinition("gauge", "a", "b", RailSpacingMeasurement.Euclidean, Constant(System.Math.Sqrt(2.8 * 2.8 + 16), 10), "positive", "negative"),
            new RailSpacingDefinition("reverse", "b", "a", RailSpacingMeasurement.BankedLateral, Constant(-3, 10)) };
        var geometry = new RailGeometry(new(Line(), Vector3d.UnitZ, TrackOffsetProfile.Constant(default, 10)), new(rails, spacings), Settings);
        Assert.AreEqual(5, geometry.MeasureSpacing("centers", 2), 1e-14);
        Assert.AreEqual(3, geometry.MeasureSpacing("lateral", 2), 1e-14);
        Assert.AreEqual(System.Math.Sqrt(23.84), geometry.MeasureSpacing("gauge", 2), 1e-14);
        Assert.AreEqual(-3, geometry.MeasureSpacing("reverse", 2), 1e-14);
        Assert.AreEqual(0, geometry.Validate().Count);
    }

    [TestMethod]
    public void RolledDatumGaugeAndLargeWorldOriginsRetainSmallSectionDisplacements()
    {
        var rails = new[] { Rail("a", new(0, -0.5, 0), 10, roll: Constant(System.Math.PI / 2, 10)), Rail("b", new(0, 0.5, 0), 10) };
        var measurement = new RailSpacingDefinition("g", "a", "b", RailSpacingMeasurement.Euclidean, Constant(System.Math.Sqrt(0.82), 10), "positive", "negative");
        var track = new HeartlineTrack(Line(origin: new(1e12, 1e12, 1e12)), Vector3d.UnitZ, TrackOffsetProfile.Constant(default, 10));
        var geometry = new RailGeometry(track, new(rails, new[] { measurement }), Settings);
        Assert.AreEqual(System.Math.Sqrt(0.82), geometry.MeasureSpacing("g", 5), 1e-14);
        Assert.AreEqual(0, geometry.Validate().Count);
    }

    [TestMethod]
    public void RepeatedHeartlineKnotsLimitAutomaticRailDerivativeAvailability()
    {
        var definition = new SplineDefinition(5, new[] { 0.0, 0, 0, 0, 0, 0, 0.5, 0.5, 0.5, 1, 1, 1, 1, 1, 1 });
        var curve = new SplineCurve3d(definition, Enumerable.Range(0, 9).Select(i => new Vector3d(i, 0.1 * i * i, 0)));
        var stations = new ArcLengthMap(curve);
        var geometry = new RailGeometry(new(curve, Vector3d.UnitZ, TrackOffsetProfile.Constant(default, stations.Length)),
            new(new[] { Rail("r", new(0, 0.2, 0), stations.Length) }), Settings);
        double s = stations.StationAt(0.5);
        Assert.AreEqual(1, geometry.Evaluate("r", s).DerivativeOrder);
        Assert.ThrowsExactly<RailGeometryException>(() => geometry.Evaluate("r", s).CurvatureVector);
        Assert.AreEqual(2, geometry.Evaluate("r", s, EvaluationSide.Left).DerivativeOrder);
        Assert.AreEqual(2, geometry.Evaluate("r", s, EvaluationSide.Right).DerivativeOrder);
    }

    [TestMethod]
    public void ValueOnlyProfilesDoNotManufactureCenterlineOrSectionDerivatives()
    {
        var valueOnly = new CustomFunction(0, 10, (_, _) => new(0, derivativeOrder: 0));
        var offsets = new TrackOffsetProfile(Constant(0, 10), valueOnly, Constant(0, 10));
        var geometry = Straight(new RailDefinition("r", offsets, Section(), Constant(0, 10)));
        Assert.AreEqual(0, geometry.Evaluate("r", 2).DerivativeOrder);
        Assert.ThrowsExactly<RailGeometryException>(() => geometry.CreateRailDistanceMap("r"));
        Assert.IsTrue(geometry.Validate().Any(d => d.Code == RailDiagnosticCode.UnavailableDerivative));
        geometry = Straight(Rail("r", default, 10, roll: valueOnly));
        Assert.AreEqual(2, geometry.Evaluate("r", 2).DerivativeOrder);
        _ = geometry.Evaluate("r", 2).SectionFrame;
        Assert.ThrowsExactly<RailGeometryException>(() => geometry.Evaluate("r", 2).SectionAngularRate);
        Assert.AreEqual(10, geometry.CreateRailDistanceMap("r").Length, 1e-12);
    }

    [TestMethod]
    public void DefinitionsCopyCollectionsAndRejectInvalidReferencesDomainsAndControls()
    {
        var boundaries = new[] { Section().Boundaries[0] };
        var datums = new Dictionary<string, Vector3d> { ["d"] = new(0, 0.2, 0) };
        var section = new RailCrossSection(boundaries, datums);
        boundaries[0] = null!;
        datums.Clear();
        Assert.IsNotNull(section.Boundaries[0]);
        Assert.AreEqual(new Vector3d(0, 0.2, 0), section.Datums["d"]);
        RailDefinition[] rails = { Rail("r", default, 10, section) };
        var arrangement = new RailArrangement(rails);
        rails[0] = null!;
        Assert.AreEqual("r", arrangement.Rails[0].Id);
        Assert.ThrowsExactly<ArgumentException>(() => new RailCrossSection(new[] { Line() }));
        Assert.ThrowsExactly<ArgumentException>(() => new RailCrossSection(Array.Empty<SplineCurve3d>()));
        Assert.ThrowsExactly<ArgumentException>(() => new RailArrangement(new[] { Rail("r", default, 10), Rail("r", default, 10) }));
        Assert.ThrowsExactly<ArgumentException>(() => new RailArrangement(new[] { Rail("r", default, 10) },
            new[] { new RailSpacingDefinition("g", "r", "missing", RailSpacingMeasurement.Euclidean, Constant(1, 10)) }));
        Assert.ThrowsExactly<ArgumentException>(() => new RailSpacingDefinition("g", "a", "b", RailSpacingMeasurement.Euclidean, Constant(1, 10), "one"));
        Assert.ThrowsExactly<ArgumentException>(() => new RailArrangement(new[] { Rail("a", default, 10), Rail("b", default, 10) },
            new[] { new RailSpacingDefinition("g", "a", "b", RailSpacingMeasurement.Euclidean, Constant(1, 10), "unknown", "positive") }));
        Assert.ThrowsExactly<ArgumentException>(() => Straight(Rail("r", default, 1)));
        var time = ScalarTimeline.FromKeyframes(ProfileDomain.Time, new[] { new TimelineKeyframe(0, 0), new TimelineKeyframe(10, 1) });
        Assert.ThrowsExactly<ArgumentException>(() => Straight(Rail("r", default, 10, roll: time)));
        Assert.ThrowsExactly<ArgumentOutOfRangeException>(() => new RailValidationSettings(double.NaN, 1e-7, 1e-7, 1e-7, 0, 1e-8));
        Assert.ThrowsExactly<ArgumentOutOfRangeException>(() => Straight(Rail("r", default, 10)).Evaluate("r", -1));
        Assert.ThrowsExactly<KeyNotFoundException>(() => arrangement.GetRail("missing"));
    }
}
