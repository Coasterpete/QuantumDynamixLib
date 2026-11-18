using System;
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
public sealed class RailValidationTests
{
    [TestMethod]
    [DataRow(0.0)]
    [DataRow(1e-12)]
    public void CollapsedAndNearlyCollapsedCircularOffsetsHaveLocatedSingularityDiagnostics(double remainingRadius)
    {
        var circle = SplineFoundationTests.QuarterCircle();
        double length = new ArcLengthMap(circle).Length;
        var geometry = new RailGeometry(new(circle, Vector3d.UnitZ, TrackOffsetProfile.Constant(default, length)),
            new(new[] { Rail("inner", new(0, 1 - remainingRadius, 0), length) }), Settings);
        RailGeometryException error = Assert.ThrowsExactly<RailGeometryException>(() => geometry.Evaluate("inner", length / 2).Tangent);
        Assert.AreEqual(RailDiagnosticCode.SingularCenterline, error.Diagnostic.Code);
        Assert.AreEqual("inner", error.Diagnostic.RailId);
        Assert.AreEqual(length / 2, error.Diagnostic.HeartlineStation);
        Assert.IsTrue(error.Diagnostic.Residual <= Settings.MinimumSpeedRatio);
        Assert.ThrowsExactly<RailGeometryException>(() => geometry.CreateRailDistanceMap("inner"));
        Assert.IsTrue(geometry.Validate().Any(d => d.Code == RailDiagnosticCode.SingularCenterline));
    }

    [TestMethod]
    public void StraightCollapsedRailAndSpineFailWithoutChangingDefinitions()
    {
        var offsets = new TrackOffsetProfile(new CustomFunction(0, 10, (s, _) => new(-s, -1)), Constant(0, 10), Constant(0, 10));
        var geometry = Straight(new RailDefinition("r", offsets, Section(), Constant(0, 10)));
        Assert.ThrowsExactly<RailGeometryException>(() => geometry.CreateRailDistanceMap("r"));
        TestMath.Near(default, geometry.Evaluate("r", 4).Position, 1e-14);
        Assert.AreSame(offsets, geometry.Arrangement.Rails[0].Offsets);
        geometry = new(new(Line(), Vector3d.UnitZ, offsets), new(new[] { Rail("r", default, 10) }), Settings);
        Assert.ThrowsExactly<RailGeometryException>(() => geometry.CreateSpineDistanceMap());
    }

    [TestMethod]
    public void NearParallelProjectedLeftHonorsCallerSuppliedFrameFloor()
    {
        var offsets = new TrackOffsetProfile(new CustomFunction(0, 10, (s, _) => new((-1 + 1e-6) * s, -1 + 1e-6)),
            new CustomFunction(0, 10, (s, _) => new(s, 1)), Constant(0, 10));
        var arrangement = new RailArrangement(new[] { new RailDefinition("r", offsets, Section(), Constant(0, 10)) });
        var track = new HeartlineTrack(Line(), Vector3d.UnitZ, TrackOffsetProfile.Constant(default, 10));
        var strict = new RailGeometry(track, arrangement, new(1e-8, 1e-7, 1e-7, 1e-7, 1e-10, 1e-5));
        Assert.ThrowsExactly<RailGeometryException>(() => strict.Evaluate("r", 3).SectionFrame);
        var permissive = new RailGeometry(track, arrangement, new(1e-8, 1e-7, 1e-7, 1e-7, 1e-10, 1e-8));
        CurveFrame frame = permissive.Evaluate("r", 3).SectionFrame;
        Assert.AreEqual(1, frame.Left.Length, 1e-10);
        Assert.AreEqual(0, Vector3d.Dot(frame.Forward, frame.Left), 1e-9);
    }

    [TestMethod]
    public void CoincidentRailsAndWrongSpacingAreReportedWithoutMovingEitherRail()
    {
        var arrangement = new RailArrangement(new[] { Rail("a", default, 10), Rail("b", default, 10) },
            new[] { new RailSpacingDefinition("g", "a", "b", RailSpacingMeasurement.Euclidean, Constant(1, 10)) });
        var geometry = new RailGeometry(new(Line(), Vector3d.UnitZ, TrackOffsetProfile.Constant(default, 10)), arrangement, Settings);
        var diagnostics = geometry.Validate();
        Assert.IsTrue(diagnostics.Any(d => d.Code == RailDiagnosticCode.CoincidentRails && d.RelatedRailId == "b"));
        Assert.IsTrue(diagnostics.Any(d => d.Code == RailDiagnosticCode.SpacingMismatch && d.Residual == 1));
        TestMath.Near(geometry.Evaluate("a", 2).Position, geometry.Evaluate("b", 2).Position, 0);
    }

    [TestMethod]
    public void DegenerateAndDiscontinuousProfileBoundariesAreDiagnosed()
    {
        var point = new RailCrossSection(new[] { SplineCurve3d.Clamped(new[] { Vector3d.Zero, Vector3d.Zero }, 1) });
        Assert.IsTrue(Straight(Rail("r", default, 10, point)).Validate().Any(d => d.Code == RailDiagnosticCode.DegenerateProfile));
        var jump = new SplineCurve3d(new SplineDefinition(1, new[] { 0.0, 0, 0.5, 0.5, 1, 1 }),
            new Vector3d[] { new(0, 0, 0), new(0, 0.1, 0), new(0, 0.2, 0), new(0, 0.3, 0) });
        Assert.IsTrue(Straight(Rail("r", default, 10, new(new[] { jump }))).Validate().Any(d => d.Code == RailDiagnosticCode.ProfileDiscontinuity));
    }

    [TestMethod]
    public void SweptProfileCanBeSingularEvenWithRegularRailCenterline()
    {
        var circle = SplineFoundationTests.QuarterCircle();
        double length = new ArcLengthMap(circle).Length;
        var section = new RailCrossSection(new[] { SplineCurve3d.Clamped(new Vector3d[] { new(0, 0, 0), new(0, 1, 0) }, 1) });
        var geometry = new RailGeometry(new(circle, Vector3d.UnitZ, TrackOffsetProfile.Constant(default, length)),
            new(new[] { Rail("r", default, length, section) }), Settings);
        Assert.AreEqual(1, geometry.Evaluate("r", length / 2).SpeedPerHeartlineMetre, 1e-14);
        Assert.IsTrue(geometry.EvaluateSurface("r", length / 2, 0, 1).AreaVector.Length < 1e-12);
        Assert.IsTrue(geometry.Validate().Any(d => d.Code == RailDiagnosticCode.SingularSweep));
    }

    [TestMethod]
    public void ProfileSlopeAndRollJoinsProduceDistinctDerivativeAndOrientationDiagnostics()
    {
        var lateral = ScalarTimeline.FromKeyframes(ProfileDomain.Distance, new[] {
            new TimelineKeyframe(0, 0), new TimelineKeyframe(5, 0), new TimelineKeyframe(10, 1) });
        var geometry = Straight(new RailDefinition("r", new(Constant(0, 10), lateral, Constant(0, 10)), Section(), Constant(0, 10)));
        Assert.AreEqual(0, geometry.Evaluate("r", 5).DerivativeOrder);
        var diagnostics = geometry.Validate();
        Assert.IsTrue(diagnostics.Any(d => d.HeartlineStation == 5 && d.Code == RailDiagnosticCode.StationDerivativeDiscontinuity));
        Assert.IsTrue(diagnostics.Any(d => d.HeartlineStation == 5 && d.Code == RailDiagnosticCode.TangentDiscontinuity));
        var roll = new CustomFunction(0, 10, (s, side) => new(s < 5 || s == 5 && side == EvaluationSide.Left ? 0 : 0.3), new[] { 5.0 });
        geometry = Straight(Rail("r", default, 10, roll: roll));
        Assert.AreEqual(2, geometry.Evaluate("r", 5).DerivativeOrder);
        Assert.IsTrue(geometry.Validate().Any(d => d.HeartlineStation == 5 && d.Code == RailDiagnosticCode.SectionOrientationDiscontinuity));
    }

    [TestMethod]
    public void VerticalClosedLoopAndUnwrappedBankAndRollRemainContinuousThroughInversions()
    {
        var curve = SplineCurve3d.Periodic(Enumerable.Range(0, 10).Select(i =>
            new Vector3d(5 * System.Math.Cos(2 * System.Math.PI * i / 10), 0, 5 * System.Math.Sin(2 * System.Math.PI * i / 10))));
        double length = new ArcLengthMap(curve).Length;
        double rate = 2 * System.Math.PI / length;
        var bank = new CustomFunction(0, length, (s, _) => new(rate * s, rate));
        var track = new HeartlineTrack(curve, Vector3d.UnitY, TrackOffsetProfile.Constant(new(0, 0, -0.3), length), bank);
        var geometry = new RailGeometry(track, new(new[] { Rail("single", new(0, 0.2, 0), length,
            roll: new CustomFunction(0, length, (s, _) => new(rate * s, rate))) }), Settings);
        foreach (double s in Enumerable.Range(0, 101).Select(i => length * i / 100))
        {
            CurveFrame frame = geometry.Evaluate("single", s).SectionFrame;
            Assert.AreEqual(1, frame.Forward.Length, 1e-13);
            Assert.AreEqual(1, frame.Left.Length, 1e-13);
            Assert.AreEqual(0, Vector3d.Dot(frame.Forward, frame.Up), 1e-13);
            TestMath.Near(frame.Up, Vector3d.Cross(frame.Forward, frame.Left), 1e-13);
        }
        var failures = geometry.Validate(closedCircuit: true);
        Assert.AreEqual(0, failures.Count, string.Join(Environment.NewLine, failures));
        // An open section is never silently wrapped by evaluation or inversion.
        Assert.ThrowsExactly<ArgumentOutOfRangeException>(() => geometry.Evaluate("single", length + 0.1));
    }

    [TestMethod]
    public void ClosedHeartlineWithUnclosedBankProducesRailSeamDiagnostics()
    {
        var curve = SplineCurve3d.Periodic(Enumerable.Range(0, 9).Select(i =>
            new Vector3d(5 * System.Math.Cos(2 * System.Math.PI * i / 9), 5 * System.Math.Sin(2 * System.Math.PI * i / 9), 0)));
        double length = new ArcLengthMap(curve).Length;
        var bank = new CustomFunction(0, length, (s, _) => new(0.3 * s / length, 0.3 / length));
        var geometry = new RailGeometry(new(curve, Vector3d.UnitZ, TrackOffsetProfile.Constant(default, length), bank),
            new(new[] { Rail("r", new(0, 0, 0.4), length) }), Settings);
        var diagnostics = geometry.Validate(closedCircuit: true);
        Assert.IsTrue(diagnostics.Any(d => d.HeartlineStation == length && d.Code == RailDiagnosticCode.PositionDiscontinuity));
        Assert.IsTrue(diagnostics.Any(d => d.HeartlineStation == length && d.Code == RailDiagnosticCode.SectionOrientationDiscontinuity));
        Assert.AreEqual(0.3, bank.Evaluate(length).Value);
    }

    [TestMethod]
    public void ExplicitBankCompensationClosesNonplanarRmfHolonomyWithoutAutomaticCorrection()
    {
        double[] heights = { 0, 0.2, 1.4, -0.5, 0.8, 2, -1, 0, 0.4 };
        var curve = SplineCurve3d.Periodic(Enumerable.Range(0, heights.Length).Select(i =>
            new Vector3d(4 * System.Math.Cos(2 * System.Math.PI * i / heights.Length), 4 * System.Math.Sin(2 * System.Math.PI * i / heights.Length), heights[i])));
        double length = new ArcLengthMap(curve).Length;
        var offsets = TrackOffsetProfile.Constant(new(0, 0, -0.2), length);
        var initial = new HeartlineTrack(curve, Vector3d.UnitZ, offsets, frameAngularTolerance: 1e-9);
        double twist = initial.Frames.ClosureTwist!.Value;
        Assert.IsTrue(System.Math.Abs(twist) > 1e-5);
        var arrangement = new RailArrangement(new[] { Rail("r", new(0, 0.2, 0), length) });
        var uncompensated = new RailGeometry(initial, arrangement, Settings);
        Assert.IsTrue(uncompensated.Validate(true).Any(d => d.HeartlineStation == length && d.Code == RailDiagnosticCode.SectionOrientationDiscontinuity));
        var compensation = new CustomFunction(0, length, (s, _) => new(-twist * s / length, -twist / length));
        var compensated = new RailGeometry(new(curve, Vector3d.UnitZ, offsets, compensation, frameAngularTolerance: 1e-9), arrangement, Settings);
        var failures = compensated.Validate(true);
        Assert.AreEqual(0, failures.Count, string.Join(Environment.NewLine, failures));
        Assert.AreEqual(twist, initial.Frames.ClosureTwist);
    }

    [TestMethod]
    public void SectionJoinsCheckPositionsDerivativesFramesProfilesAndRailIdentity()
    {
        RailGeometry Make(double origin, double roll = 0, double width = 0.1, string id = "r") => new(
            new(Line(origin: new(origin, 0, 0)), Vector3d.UnitZ, TrackOffsetProfile.Constant(default, 10)),
            new(new[] { Rail(id, new(0, 0.2, 0), 10, Section(width), Constant(roll, 10)) }), Settings);
        RailGeometry first = Make(0);
        Assert.AreEqual(0, first.ValidateJoin(Make(10)).Count);
        Assert.IsTrue(first.ValidateJoin(Make(10.01)).Any(d => d.Code == RailDiagnosticCode.PositionDiscontinuity));
        Assert.IsTrue(first.ValidateJoin(Make(10, 0.2)).Any(d => d.Code == RailDiagnosticCode.SectionOrientationDiscontinuity));
        Assert.IsTrue(first.ValidateJoin(Make(10, width: 0.2)).Any(d => d.Code == RailDiagnosticCode.ProfileDiscontinuity));
        Assert.IsTrue(first.ValidateJoin(Make(10, id: "other")).Any(d => d.Code == RailDiagnosticCode.IncompatibleArrangement));
    }

    [TestMethod]
    public void LargeOriginsDoNotConcealSmallOpenRailSeams()
    {
        RailGeometry Make(double origin) => new(new(Line(origin: new(origin, 1e12, 1e12)), Vector3d.UnitZ,
            TrackOffsetProfile.Constant(default, 10)), new(new[] { Rail("r", default, 10) }), Settings);
        var diagnostics = Make(1e12).ValidateJoin(Make(1e12 + 10.01));
        Assert.IsTrue(diagnostics.Any(d => d.Code == RailDiagnosticCode.PositionDiscontinuity && d.Residual > 0.009));
    }

    [TestMethod]
    public void ValidationReportsArithmeticAndSamplingLimitsInsteadOfClaimingSuccess()
    {
        var invalid = new CustomFunction(0, 10, (_, _) => new(double.NaN));
        var geometry = Straight(Rail("r", default, 10, roll: invalid));
        Assert.IsTrue(geometry.Validate().Any(d => d.Code == RailDiagnosticCode.NumericalFailure));
        var settings = new RailValidationSettings(1e-8, 1e-7, 1e-7, 1e-7, 1e-10, 1e-8, maximumEvaluations: 3);
        geometry = new(new(Line(), Vector3d.UnitZ, TrackOffsetProfile.Constant(default, 10)),
            new(new[] { Rail("r", default, 10) }), settings);
        Assert.IsTrue(geometry.Validate().Any(d => d.Code == RailDiagnosticCode.SamplingLimit));
        var circle = SplineFoundationTests.QuarterCircle();
        double length = new ArcLengthMap(circle).Length;
        settings = new(1e-8, 1e-7, 1e-7, 1e-7, 1e-10, 1e-8, samplesPerSpan: 1, maximumDepth: 0, samplingAngularTolerance: 0.01);
        geometry = new(new(circle, Vector3d.UnitZ, TrackOffsetProfile.Constant(default, length)),
            new(new[] { Rail("r", default, length, Section(0.01)) }), settings);
        Assert.IsTrue(geometry.Validate().Any(d => d.Code == RailDiagnosticCode.SamplingLimit));
    }

    [TestMethod]
    public void FullSectionTurnsDriveAdaptiveSamplingEvenWhenEndpointOrientationsCoincide()
    {
        double rate = 4 * System.Math.PI / 10;
        var rail = Rail("r", default, 10, roll: new CustomFunction(0, 10, (s, _) => new(rate * s, rate)));
        var settings = new RailValidationSettings(1e-8, 1e-7, 1e-7, 1e-7, 1e-10, 1e-8, samplesPerSpan: 1, maximumDepth: 0);
        var geometry = new RailGeometry(new(Line(), Vector3d.UnitZ, TrackOffsetProfile.Constant(default, 10)), new(new[] { rail }), settings);
        Assert.IsTrue(geometry.Validate().Any(d => d.Code == RailDiagnosticCode.SamplingLimit));
        Assert.AreEqual(0, Straight(rail).Validate().Count);
    }

    [TestMethod]
    public void ZeroDatumGapIsAllowedByAnExplicitMeasurementWithoutConflatingRailCenters()
    {
        var rails = new[] { Rail("a", new(0, -0.1, 0), 10), Rail("b", new(0, 0.1, 0), 10) };
        var spacing = new RailSpacingDefinition("touching-datums", "a", "b", RailSpacingMeasurement.Euclidean,
            Constant(0, 10), "positive", "negative");
        var geometry = new RailGeometry(new(Line(), Vector3d.UnitZ, TrackOffsetProfile.Constant(default, 10)), new(rails, new[] { spacing }), Settings);
        Assert.AreEqual(0, geometry.MeasureSpacing(spacing.Id, 2), 1e-14);
        Assert.AreEqual(0, geometry.Validate().Count);
    }

    [TestMethod]
    public void SecondStationDerivativeJumpIsReportedEvenWhenGeometricCurvatureIsZero()
    {
        var forward = new CustomFunction(0, 10, (s, side) =>
        {
            if (s < 5 || s == 5 && side == EvaluationSide.Left) { return new(0); }
            double x = s - 5;
            return new(0.1 * x * x, 0.2 * x, 0.2, derivativeOrder: s == 5 && side == EvaluationSide.Automatic ? 1 : 3);
        }, new[] { 5.0 });
        var geometry = Straight(new RailDefinition("r", new(forward, Constant(0, 10), Constant(0, 10)), Section(), Constant(0, 10)));
        var failures = geometry.Validate();
        Assert.IsTrue(failures.Any(d => d.Code == RailDiagnosticCode.StationSecondDerivativeDiscontinuity && d.HeartlineStation == 5));
        Assert.IsFalse(failures.Any(d => d.Code == RailDiagnosticCode.CurvatureDiscontinuity));
        Assert.AreEqual(1, geometry.Evaluate("r", 5).DerivativeOrder);
    }

    [TestMethod]
    public void UnrepresentablePairSeparationReturnsANumericalDiagnostic()
    {
        var geometry = Straight(Rail("a", new(0, 1e308, 0), 10), Rail("b", new(0, -1e308, 0), 10));
        var failures = geometry.Validate();
        Assert.IsTrue(failures.Any(d => d.Code == RailDiagnosticCode.NumericalFailure));
    }
}
