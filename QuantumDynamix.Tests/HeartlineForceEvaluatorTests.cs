using System;
using System.Linq;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using QuantumDynamix.FVD;
using QuantumDynamix.Geometry;
using QuantumDynamix.Math;
using QuantumDynamix.Math.Functions;
using QuantumDynamix.Math.Timelines;
using QuantumDynamix.Splines;
using QuantumDynamix.Splines.Curves;
using QuantumDynamix.Track;

namespace QuantumDynamix.Tests;

[TestClass]
public sealed class HeartlineForceEvaluatorTests
{
    private static readonly Vector3d Gravity = new(0, 0, -10);
    private const double FixtureTolerance = 1e-12;

    [TestMethod]
    [DataRow(0.0, 0.0, -10.0)]
    [DataRow(0.0, 0.0, 0.0)]
    [DataRow(2.0, -3.0, -4.0)]
    public void LevelStraightAtConstantSpeedReportsGravityReaction(double gx, double gy, double gz)
    {
        HeartlineTrack track = CreateTrack();
        HeartlineForceSample sample = HeartlineForceEvaluator.Evaluate(track, ConstantSpeed(track, 10), 4, new(gx, gy, gz));

        Assert.AreEqual(4.0, sample.HeartlineStation);
        Assert.AreEqual(10.0, sample.HeartlineSpeed);
        TestMath.Near(new(4, 0, 0), sample.BankedFrame.Position, FixtureTolerance);
        TestMath.Near(Vector3d.Zero, sample.WorldCoordinateAcceleration, FixtureTolerance);
        TestMath.Near(new(-gx, -gy, -gz), sample.TrackSpecificForce, FixtureTolerance);
    }

    [TestMethod]
    public void PrescribedSpeedRampConvertsSpatialSlopeToTangentialAcceleration()
    {
        HeartlineTrack track = CreateTrack();
        double length = track.HeartlineStations.Length;
        var speed = ScalarTimeline.FromKeyframes(ProfileDomain.Distance,
            new[] { new TimelineKeyframe(0, 4), new TimelineKeyframe(length, 4 + 0.5 * length) });

        HeartlineForceSample sample = HeartlineForceEvaluator.Evaluate(track, speed, 4, Gravity);

        Assert.AreEqual(6, sample.HeartlineSpeed, FixtureTolerance);
        TestMath.Near(new(3, 0, 0), sample.WorldCoordinateAcceleration, FixtureTolerance);
        TestMath.Near(new(3, 0, 10), sample.TrackSpecificForce, FixtureTolerance);
    }

    [TestMethod]
    [DataRow(0)]
    [DataRow(-45)]
    [DataRow(45)]
    public void QuarterCircleBankingRedistributesSpecificForce(int bankDegrees)
    {
        const double radius = 10;
        HeartlineTrack track = CreateTrack(SplineFoundationTests.QuarterCircle(radius),
            bankDegrees * System.Math.PI / 180, spineOffset: new(0, 2, 0));
        double station = radius * System.Math.PI / 6;

        HeartlineForceSample sample = HeartlineForceEvaluator.Evaluate(track, ConstantSpeed(track, 10), station, Gravity);

        Vector3d radialPosition = new(5 * System.Math.Sqrt(3), 5, 0);
        // Position and acceleration each vary by at most one unit per metre of station here.
        // The default inverse station budget for this arc is about 1.16e-8 m.
        TestMath.Near(radialPosition, sample.BankedFrame.Position, 2e-8);
        TestMath.Near(-radialPosition, sample.WorldCoordinateAcceleration, 2e-8);

        double diagonalForce = 10 * System.Math.Sqrt(2);
        Vector3d expectedSpecificForce = bankDegrees switch
        {
            -45 => new(0, 0, diagonalForce),
            45 => new(0, diagonalForce, 0),
            _ => new(0, 10, 10)
        };
        TestMath.Near(expectedSpecificForce, sample.TrackSpecificForce, FixtureTolerance);
    }

    [TestMethod]
    public void AtConvexCrestMatchingGravityGivesZeroSpecificForce()
    {
        SplineCurve3d circle = SplineFoundationTests.QuarterCircle(10);
        var verticalCircle = new SplineCurve3d(circle.Definition,
            circle.ControlPoints.Select(p => new Vector3d(p.Y, 0, p.X)));
        HeartlineTrack track = CreateTrack(verticalCircle);

        HeartlineForceSample sample = HeartlineForceEvaluator.Evaluate(track, ConstantSpeed(track, 10), 0, Gravity);

        TestMath.Near(Gravity, sample.WorldCoordinateAcceleration, FixtureTolerance);
        TestMath.Near(Vector3d.Zero, sample.TrackSpecificForce, FixtureTolerance);
    }

    [TestMethod]
    public void ContinuousSpeedWithSlopeJumpRequiresSidedAcceleration()
    {
        HeartlineTrack track = CreateTrack();
        double length = track.HeartlineStations.Length;
        var speed = ScalarTimeline.FromKeyframes(ProfileDomain.Distance, new[]
        {
            new TimelineKeyframe(0, 6), new TimelineKeyframe(4, 6), new TimelineKeyframe(length, 6 + 0.5 * (length - 4))
        });

        HeartlineForceSample left = HeartlineForceEvaluator.Evaluate(track, speed, 4, Gravity, EvaluationSide.Left);
        HeartlineForceSample right = HeartlineForceEvaluator.Evaluate(track, speed, 4, Gravity, EvaluationSide.Right);

        Assert.AreEqual(6.0, left.HeartlineSpeed);
        Assert.AreEqual(6.0, right.HeartlineSpeed);
        TestMath.Near(new(0, 0, 10), left.TrackSpecificForce, FixtureTolerance);
        TestMath.Near(new(3, 0, 10), right.TrackSpecificForce, FixtureTolerance);
        Assert.ThrowsExactly<InvalidOperationException>(() => HeartlineForceEvaluator.Evaluate(track, speed, 4, Gravity));
    }

    [TestMethod]
    public void C1GeometryJoinReturnsDistinctCurvatureLimits()
    {
        // H(u) = (2u, max(0, u - 1)^2, 0). At u = 1, K is 0 on the left and +Y/2 on the right.
        var curve = new SplineCurve3d(new SplineDefinition(2, new[] { 0.0, 0, 0, 1, 2, 2, 2 }),
            new[] { Vector3d.Zero, new Vector3d(1, 0, 0), new Vector3d(3, 0, 0), new Vector3d(4, 1, 0) });
        HeartlineTrack track = CreateTrack(curve);
        double station = track.HeartlineStations.StationAt(1);
        ScalarTimeline speed = ConstantSpeed(track, 2);

        HeartlineForceSample left = HeartlineForceEvaluator.Evaluate(track, speed, station, Gravity, EvaluationSide.Left);
        HeartlineForceSample right = HeartlineForceEvaluator.Evaluate(track, speed, station, Gravity, EvaluationSide.Right);

        TestMath.Near(Vector3d.Zero, left.WorldCoordinateAcceleration, FixtureTolerance);
        TestMath.Near(new(0, 2, 0), right.WorldCoordinateAcceleration, FixtureTolerance);
        TestMath.Near(new(0, 0, 10), left.TrackSpecificForce, FixtureTolerance);
        TestMath.Near(new(0, 2, 10), right.TrackSpecificForce, FixtureTolerance);
        Assert.ThrowsExactly<InvalidOperationException>(() => HeartlineForceEvaluator.Evaluate(track, speed, station, Gravity));
    }

    [TestMethod]
    public void C2GeometryJoinNeedsOnlyTheBankAngleAtTheRequestedStation()
    {
        // H(u) = (u, max(0, u - 1)^3, 0) has continuous curvature and a third derivative jump.
        var curve = new SplineCurve3d(new SplineDefinition(3, new[] { 0.0, 0, 0, 0, 1, 2, 2, 2, 2 }),
            new[] { Vector3d.Zero, new Vector3d(1.0 / 3, 0, 0), new Vector3d(1, 0, 0), new Vector3d(5.0 / 3, 0, 0), new Vector3d(2, 1, 0) });
        double length = new ArcLengthMap(curve).Length;
        var bank = new CustomFunction(0, length, (s, _) => new(s * System.Math.PI / 2, derivativeOrder: 0));
        var track = new HeartlineTrack(curve, Vector3d.UnitZ, TrackOffsetProfile.Constant(new(0, 2, 0), length), bank);
        double station = track.HeartlineStations.StationAt(1);

        HeartlineForceSample sample = HeartlineForceEvaluator.Evaluate(track, ConstantSpeed(track, 2), station, Gravity);

        TestMath.Near(Vector3d.Zero, sample.WorldCoordinateAcceleration, FixtureTolerance);
        TestMath.Near(new(0, 10, 0), sample.TrackSpecificForce, FixtureTolerance);
    }

    [TestMethod]
    [DataRow(EvaluationSide.Automatic)]
    [DataRow(EvaluationSide.Left)]
    [DataRow(EvaluationSide.Right)]
    public void PeriodicHeartlineEndpointsUseTheirOwnInwardLimits(EvaluationSide side)
    {
        var curve = SplineCurve3d.Periodic(new[]
        {
            Vector3d.Zero, new Vector3d(2, 0, 0), new Vector3d(3, 1, 0), new Vector3d(2, 3, 0), new Vector3d(0, 2, 0)
        }, degree: 2);
        HeartlineTrack track = CreateTrack(curve);
        double length = track.HeartlineStations.Length;
        var speed = ScalarTimeline.FromKeyframes(ProfileDomain.Distance, new[]
        {
            new TimelineKeyframe(-1, 3), new TimelineKeyframe(0, 2), new TimelineKeyframe(length, 2), new TimelineKeyframe(length + 1, 3)
        });

        HeartlineForceSample start = HeartlineForceEvaluator.Evaluate(track, speed, 0, Gravity, side);
        HeartlineForceSample end = HeartlineForceEvaluator.Evaluate(track, speed, length, Gravity, side);

        // The seam shares its position and tangent. The quadratic spans have inward curvatures +Y/4 and +Y/2.
        TestMath.Near(new(1, 0, 0), start.BankedFrame.Position, FixtureTolerance);
        TestMath.Near(start.BankedFrame.Position, end.BankedFrame.Position, FixtureTolerance);
        TestMath.Near(new(0, 1, 0), start.WorldCoordinateAcceleration, FixtureTolerance);
        TestMath.Near(new(0, 2, 0), end.WorldCoordinateAcceleration, FixtureTolerance);
    }

    [TestMethod]
    public void RejectsNullTrackOrTimelineAndUndefinedSide()
    {
        HeartlineTrack track = CreateTrack();
        ScalarTimeline speed = ConstantSpeed(track, 2);
        Assert.ThrowsExactly<ArgumentNullException>(() => HeartlineForceEvaluator.Evaluate(null!, speed, 0, Gravity));
        Assert.ThrowsExactly<ArgumentNullException>(() => HeartlineForceEvaluator.Evaluate(track, null!, 0, Gravity));
        Assert.ThrowsExactly<ArgumentOutOfRangeException>(() => HeartlineForceEvaluator.Evaluate(track, speed, 0, Gravity, (EvaluationSide)99));
    }

    [TestMethod]
    [DataRow(-1.0)]
    [DataRow(11.0)]
    [DataRow(double.NaN)]
    [DataRow(double.PositiveInfinity)]
    public void RejectsInvalidStation(double station)
    {
        HeartlineTrack track = CreateTrack();
        ScalarTimeline speed = ConstantSpeed(track, 2);
        Assert.ThrowsExactly<ArgumentOutOfRangeException>(() => HeartlineForceEvaluator.Evaluate(track, speed, station, Gravity));
    }

    [TestMethod]
    public void RejectsTimeDomainSpeed()
    {
        HeartlineTrack track = CreateTrack();
        var speed = new ScalarTimeline(ProfileDomain.Time, ConstantSpeed(track, 2).Segments);
        Assert.ThrowsExactly<ArgumentException>(() => HeartlineForceEvaluator.Evaluate(track, speed, 4, Gravity));
    }

    [TestMethod]
    [DataRow(BoundaryPolicy.Hold)]
    [DataRow(BoundaryPolicy.Zero)]
    [DataRow(BoundaryPolicy.Repeat)]
    public void RejectsOtherBoundaryPolicies(BoundaryPolicy boundary)
    {
        HeartlineTrack track = CreateTrack();
        // Zero endpoint values make the timeline continuous under each policy.
        var segment = new FunctionSegment(0, track.HeartlineStations.Length, new RiseHoldFallEnvelope());
        var speed = new ScalarTimeline(ProfileDomain.Distance, new[] { segment }, boundary);
        Assert.ThrowsExactly<ArgumentException>(() => HeartlineForceEvaluator.Evaluate(track, speed, 4, Gravity));
    }

    [TestMethod]
    public void RejectsSpeedTimelineWithValueJump()
    {
        HeartlineTrack track = CreateTrack();
        var speed = new ScalarTimeline(ProfileDomain.Distance, new[]
        {
            FunctionSegment.Transition(0, 4, 2, 2), FunctionSegment.Transition(4, track.HeartlineStations.Length, 3, 3)
        }, allowDiscontinuities: true);
        Assert.ThrowsExactly<ArgumentException>(() => HeartlineForceEvaluator.Evaluate(track, speed, 4, Gravity, EvaluationSide.Right));
    }

    [TestMethod]
    [DataRow(1.0, 11.0)]
    [DataRow(-1.0, 9.0)]
    public void RejectsSpeedTimelineThatOnlyCoversPartOfTrack(double minimum, double maximum)
    {
        HeartlineTrack track = CreateTrack();
        var speed = new ScalarTimeline(ProfileDomain.Distance, new[] { FunctionSegment.Transition(minimum, maximum, 2, 2) });
        Assert.ThrowsExactly<ArgumentException>(() => HeartlineForceEvaluator.Evaluate(track, speed, 4, Gravity));
    }

    [TestMethod]
    [DataRow(0.0)]
    [DataRow(-1.0)]
    public void RejectsNonpositiveSpeedAtSample(double value)
    {
        HeartlineTrack track = CreateTrack();
        ScalarTimeline speed = ConstantSpeed(track, value);
        Assert.ThrowsExactly<ArgumentOutOfRangeException>(() => HeartlineForceEvaluator.Evaluate(track, speed, 4, Gravity));
    }

    [TestMethod]
    public void RejectsUnavailableSpeedDerivativeEvenWithExplicitSide()
    {
        HeartlineTrack track = CreateTrack();
        var valuesOnly = new CustomFunction(0, track.HeartlineStations.Length, (_, _) => new(2, derivativeOrder: 0));
        var speed = new ScalarTimeline(ProfileDomain.Distance, new[] { valuesOnly });
        Assert.ThrowsExactly<InvalidOperationException>(() => HeartlineForceEvaluator.Evaluate(track, speed, 4, Gravity, EvaluationSide.Left));
    }

    private static HeartlineTrack CreateTrack(SplineCurve3d? curve = null, double bankRadians = 0, Vector3d spineOffset = default)
    {
        curve ??= SplineCurve3d.Clamped(new[] { Vector3d.Zero, new Vector3d(10, 0, 0) }, degree: 1);
        double length = new ArcLengthMap(curve).Length;
        var bank = new CustomFunction(0, length, (_, _) => new(bankRadians));
        return new(curve, Vector3d.UnitZ, TrackOffsetProfile.Constant(spineOffset, length), bank);
    }

    private static ScalarTimeline ConstantSpeed(HeartlineTrack track, double speed) => new(ProfileDomain.Distance,
        new[] { FunctionSegment.Transition(0, track.HeartlineStations.Length, speed, speed) });
}
