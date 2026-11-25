using System;
using System.Linq;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using QuantumDynamix.App;
using QuantumDynamix.Geometry;
using QuantumDynamix.Math;
using QuantumDynamix.Math.Functions;
using QuantumDynamix.Math.Timelines;
using static QuantumDynamix.App.RegionTrackGeometry;
using SysMath = System.Math;

namespace QuantumDynamix.Tests.Application;

[TestClass]
public sealed class MathematicalTransitionTests
{
    [TestMethod]
    [DataRow(EasingKind.Linear, .25)]
    [DataRow(EasingKind.Quadratic, .0625)]
    [DataRow(EasingKind.Cubic, .015625)]
    [DataRow(EasingKind.Quartic, .00390625)]
    [DataRow(EasingKind.Quintic, .0009765625)]
    [DataRow(EasingKind.Sine, .07612046748871326)]
    [DataRow(EasingKind.Cosine, .07612046748871326)]
    [DataRow(EasingKind.Plateau, .5)]
    public void BuiltInFunctionsUseTheLibraryShapeAndPreserveEndpoints(EasingKind kind, double quarter)
    {
        foreach (EasingDirection direction in Enum.GetValues<EasingDirection>())
        {
            var shape = new TransitionShape(kind, direction);
            Assert.IsNull(TimelineEditorSession.BuildTimeline(ProfileDomain.Distance, [new(10, 4), new(18, 2)],
                out ScalarTimeline? timeline, [shape]));
            Assert.AreEqual(4, timeline!.Evaluate(10).Value, 1e-14);
            Assert.AreEqual(2, timeline.Evaluate(18).Value, 1e-14);
            var (station, normalized) = direction switch
            {
                EasingDirection.In => (12.0, quarter),
                EasingDirection.Out => (16.0, 1 - quarter),
                _ => (11.0, quarter / 2)
            };
            Assert.AreEqual(4 - 2 * normalized, timeline.Evaluate(station).Value, 1e-14);
            Assert.AreEqual(shape, TransitionShape.FromSegment(timeline.Segments[0]));
        }
    }

    [TestMethod]
    public void ConstantAndPlateauExposeOnlyTheirActualContinuity()
    {
        Assert.IsNull(TimelineEditorSession.BuildTimeline(ProfileDomain.Distance, [new(0, 2), new(5, 2), new(10, 5)],
            out ScalarTimeline? timeline, [new(null), new(EasingKind.Plateau, HoldStart: .2, HoldEnd: .7, HoldValue: .3)]));
        Assert.AreEqual(new ScalarJet(2), timeline!.Evaluate(2));
        Assert.AreEqual(new ScalarJet(2.9), timeline.Evaluate(7));
        Assert.IsEmpty(timeline.ValidateContinuity(3));
        Assert.IsNotNull(TimelineEditorSession.BuildTimeline(ProfileDomain.Distance, [new(0, 2), new(5, 3)], out _, [new(null)]));

        Assert.IsNull(TimelineEditorSession.BuildTimeline(ProfileDomain.Distance, [new(0, 2), new(5, 3), new(10, 5)],
            out timeline, [new(EasingKind.Quadratic), new(EasingKind.Quintic, EasingDirection.InOut)]));
        Assert.IsEmpty(timeline!.ValidateContinuity(0));
        Assert.AreEqual(0, timeline.Evaluate(5).DerivativeOrder);
        Assert.AreEqual(1, timeline.Evaluate(7.5).DerivativeOrder);
    }

    [TestMethod]
    [DataRow(false, VerticalChannel, 0.0)]
    [DataRow(true, NormalForceChannel, 1.0)]
    public void ShapeOnlyChangesRebuildBothExistingRegionModes(bool force, string channel, double baseline)
    {
        var mode = force ? RegionAuthoringMode.Force : RegionAuthoringMode.CoordinateAcceleration;
        var session = new TimelineEditorSession(RigidPose3d.Identity, 12);
        Assert.IsNull(session.AddRegion(false, 24, 3, mode));
        Assert.IsNull(session.Add(0, channel, new(11, baseline + .7), out _));
        Assert.IsNull(session.Add(0, channel, new(18, baseline + .3), out _));
        Assert.IsNull(session.AddRegion(false, 10, 0, mode));
        Vector3d oldEnd = session.Geometry!.DisplayScene.Boundaries[^1].Position;
        Assert.IsNull(session.UpdateTransition(0, channel, 0, 8, baseline, baseline + .7, new(EasingKind.Quintic)));
        Assert.IsNull(session.UpdateTransition(0, channel, 1, 6, baseline + .7, baseline + .3,
            new(EasingKind.Plateau, HoldStart: .2, HoldEnd: .6, HoldValue: .4)));
        Vector3d changed = session.Geometry!.DisplayScene.Boundaries[^1].Position;
        Assert.IsGreaterThan(.02, (changed - oldEnd).Length);
        foreach (double station in new[] { 5.0, 10, 13, 16 })
        {
            double target = session.Regions[0].Definition.Channels[channel].Evaluate(station).Value;
            double result = force ? session.Geometry.CalculateSpecificForceG(station, EvaluationSide.Right).Z
                : session.Geometry.CalculateGeometry(station, EvaluationSide.Right).Curvature.Length * 12 * 12;
            Assert.AreEqual(target, result, 4e-6);
        }
        var beforeResize = session.Regions[0].Shapes(channel);
        Assert.IsNull(session.UpdateRegion(0, "Resized", 30));
        Assert.AreEqual(33, session.Regions[1].Definition.Start);
        Assert.AreEqual(13, session.Keys(0, channel)[1].Coordinate);
        CollectionAssert.AreEqual(beforeResize, session.Regions[0].Shapes(channel));
        AssertGeometryJoin(session, 33);
        var scene = session.Geometry;
        Assert.IsNotNull(session.UpdateTransition(0, channel, 1, 30, baseline + .7, baseline + .3, new(EasingKind.Cubic)));
        Assert.AreSame(scene, session.Geometry);
        Assert.IsTrue(session.Undo());
        Assert.AreEqual(changed, session.Geometry!.DisplayScene.Boundaries[^1].Position);
        Assert.IsNotNull(session.UpdateTransition(0, channel, 0, 8, baseline, baseline + .7, new(null)));
        Assert.IsTrue(session.CanRedo);
        Assert.IsTrue(session.Redo());
        Assert.AreSame(scene, session.Geometry);
        Assert.IsNull(session.Move(0, channel, 1, new(14, baseline + .6)));
        CollectionAssert.AreEqual(beforeResize, session.Regions[0].Shapes(channel));
        Assert.IsNull(session.DeleteTransition(0, channel, 1));
        CollectionAssert.AreEqual(new[] { beforeResize[0], beforeResize[2] }, session.Regions[0].Shapes(channel));
        Assert.IsNull(session.UpdateTransition(0, channel, 1, 19, baseline + .6, baseline + .15, new(EasingKind.Cubic)));
        Assert.AreEqual(baseline + .15, session.Keys(1, channel)[0].Value);
        AssertGeometryJoin(session, 33);
        Assert.IsTrue(session.Undo());
        Assert.AreEqual(baseline, session.Keys(1, channel)[0].Value);
    }

    [TestMethod]
    [DataRow(6.0)]
    [DataRow(-6.0)]
    public void ConstantPitchGivesTheAnalyticVerticalPlaneArc(double pitch)
    {
        var session = new TimelineEditorSession(RigidPose3d.Identity, 12);
        Assert.IsNull(session.AddRegion(false, 12, 0, RegionAuthoringMode.Geometric));
        Assert.IsNull(session.UpdateTransition(0, PitchChangeChannel, 0, 12, pitch, pitch, new(null)));
        double rate = pitch * SysMath.PI / 180;
        var end = session.Geometry!.DisplayScene.Boundaries[^1];
        Near(new(SysMath.Sin(12 * rate) / rate, 0, (1 - SysMath.Cos(12 * rate)) / rate), end.Position, 2e-6);
        Near(new(SysMath.Cos(12 * rate), 0, SysMath.Sin(12 * rate)), end.Orientation!.Value.Forward);
        Assert.AreEqual(SysMath.Abs(rate), session.Geometry.CalculateGeometry(12, EvaluationSide.Left).Curvature.Length, 1e-9);
    }

    [TestMethod]
    [DataRow(89.9999)]
    [DataRow(90.0)]
    public void GroundHeadingContinuesThroughVerticalWithCombinedPitchAndYaw(double initialPitch)
    {
        var session = new TimelineEditorSession(new RigidPose3d(Vector3d.Zero, Pose(.4, initialPitch)), 10);
        Assert.IsNull(session.AddRegion(false, 10, 4, RegionAuthoringMode.Geometric));
        Vector3d straight = session.Geometry!.DisplayScene.Boundaries[^1].Position;
        Near(10 * Pose(.4, initialPitch).Rotate(Vector3d.UnitX), straight);
        Assert.IsNull(session.UpdateTransition(0, YawChangeChannel, 0, 10, 4, 4, new(null)));
        double yaw = 4 * SysMath.PI / 180;
        double heading = .4 + 10 * yaw;
        double horizontal = SysMath.Cos(initialPitch * SysMath.PI / 180);
        Near(new(horizontal * (SysMath.Sin(heading) - SysMath.Sin(.4)) / yaw,
                horizontal * (SysMath.Cos(.4) - SysMath.Cos(heading)) / yaw, straight.Z),
            session.Geometry!.DisplayScene.Boundaries[^1].Position);
        Assert.IsNull(session.UpdateTransition(0, PitchChangeChannel, 0, 10, 2, 2, new(null)));
        Vector3d expected = Pose(heading, initialPitch + 20).Rotate(Vector3d.UnitX);
        var end = session.Geometry!.DisplayScene.Boundaries[^1];
        Near(expected, end.Orientation!.Value.Forward);
        Vector3d position = end.Position;
        Assert.IsLessThan(-.5, position.Y);
        Assert.IsNull(session.UpdateTransition(0, TrackDocument.BankingChannel, 0, 10, 0, .5,
            new(EasingKind.Cosine, EasingDirection.InOut)));
        Near(position, session.Geometry!.DisplayScene.Boundaries[^1].Position);
        Assert.IsGreaterThan(.4, (end.Orientation.Value.Left - session.Geometry.DisplayScene.Boundaries[^1].Orientation!.Value.Left).Length);
    }

    [TestMethod]
    public void FourModesInSequenceRebindStartsAndPreserveNonlinearShapes()
    {
        var track = new TimelineEditorSession(RigidPose3d.Identity, 15);
        Assert.IsNull(track.AddRegion(false, 18, 10, RegionAuthoringMode.Geometric));
        Assert.IsNull(track.Add(0, PitchChangeChannel, new(19, 4), out _));
        Assert.IsNull(track.Add(0, YawChangeChannel, new(19, 2), out _));
        Assert.IsNull(track.UpdateTransition(0, PitchChangeChannel, 1, 9, 4, 1.5, new(EasingKind.Sine, EasingDirection.Out)));
        Assert.IsNull(track.AddRegion(false, 12, 0, RegionAuthoringMode.Force));
        var force = track.Keys(1, NormalForceChannel);
        Assert.IsNull(track.UpdateTransition(1, NormalForceChannel, 0, 12, force[0].Value, 1.15, new(EasingKind.Cubic)));
        Assert.IsNull(track.AddRegion(false, 10, 0, RegionAuthoringMode.Geometric));
        var yaw = track.Keys(2, YawChangeChannel);
        Assert.IsNull(track.UpdateTransition(2, YawChangeChannel, 0, 10, yaw[0].Value, 1, new(EasingKind.Plateau)));
        Assert.IsNull(track.AddRegion(false, 8, 0, RegionAuthoringMode.CoordinateAcceleration));
        foreach (double station in new[] { 28.0, 40, 50 }) { AssertGeometryJoin(track, station); }
        Vector3d before = track.Geometry!.DisplayScene.Boundaries[^1].Position;
        Assert.IsNull(track.Move(0, PitchChangeChannel, 1, new(20, 3)));
        Assert.IsNull(track.UpdateRegion(0, "Geometric approach", 20));
        Assert.AreEqual(EasingKind.Cubic, track.Regions[1].Shapes(NormalForceChannel)[0].Kind);
        Assert.AreEqual(EasingKind.Plateau, track.Regions[2].Shapes(YawChangeChannel)[0].Kind);
        Assert.IsGreaterThan(1, (before - track.Geometry!.DisplayScene.Boundaries[^1].Position).Length);
        foreach (double station in new[] { 30.0, 42, 52 }) { AssertGeometryJoin(track, station); }
        double linked = track.Keys(3, LateralChannel)[0].Value;
        Assert.IsNull(track.UpdateTransition(2, YawChangeChannel, 0, 10,
            track.Keys(2, YawChangeChannel)[0].Value, 1.5, track.Regions[2].Shapes(YawChangeChannel)[0]));
        Assert.AreNotEqual(linked, track.Keys(3, LateralChannel)[0].Value);
        AssertGeometryJoin(track, 52);
    }

    [TestMethod]
    public void VerticalMixedJoinRejectsSidewaysCurvatureWithoutChangingHistoryAndAcceptsPitchCurvature()
    {
        var track = new TimelineEditorSession(new RigidPose3d(Vector3d.Zero, Pose(.4, 90)), 10);
        Assert.IsNull(track.AddRegion(false, 10, 10));
        Assert.IsNull(track.UpdateTransition(0, LateralChannel, 0, 10, 1, 1, new(null)));
        Assert.IsNull(track.UpdateTransition(0, VerticalChannel, 0, 10, 2, 2, new(null)));
        Assert.IsTrue(track.Undo());
        var scene = track.Geometry;
        StringAssert.Contains(track.AddRegion(true, 5, 0, RegionAuthoringMode.Geometric)!, "vertical mixed-mode join");
        Assert.AreSame(scene, track.Geometry);
        Assert.HasCount(1, track.Regions);
        Assert.IsTrue(track.CanRedo);
        Assert.IsTrue(track.Undo());
        Assert.IsNull(track.UpdateTransition(0, VerticalChannel, 0, 10, 2, 2, new(null)));
        Assert.IsNull(track.AddRegion(true, 5, 0, RegionAuthoringMode.Geometric));
        AssertGeometryJoin(track, 10);
        Assert.AreEqual(10, track.ReconstructionAnchor!.Value.Station);
        Assert.IsNull(track.SplitTransition(0, PitchChangeChannel, 0, insertBefore: true, out _));
        Assert.AreEqual(10, track.ReconstructionAnchor!.Value.Station);
        AssertGeometryJoin(track, 10);
    }

    private static Quaterniond Pose(double headingRadians, double pitchDegrees) =>
        Quaterniond.FromAxisAngle(Vector3d.UnitZ, headingRadians) *
        Quaterniond.FromAxisAngle(Vector3d.UnitY, -pitchDegrees * SysMath.PI / 180);

    private static void AssertGeometryJoin(TimelineEditorSession track, double station)
    {
        var geometry = track.Geometry!;
        var boundaries = geometry.DisplayScene.Boundaries.Where(boundary => boundary.HeartlineStation == station).ToArray();
        Assert.HasCount(2, boundaries);
        Near(boundaries[0].Position, boundaries[1].Position);
        Near(boundaries[0].Orientation!.Value.Forward, boundaries[1].Orientation!.Value.Forward);
        Near(boundaries[0].Orientation!.Value.Left, boundaries[1].Orientation!.Value.Left);
        Near(geometry.CalculateGeometry(station, EvaluationSide.Left).Curvature,
            geometry.CalculateGeometry(station, EvaluationSide.Right).Curvature);
        Near(geometry.CalculateSpecificForceG(station, EvaluationSide.Left),
            geometry.CalculateSpecificForceG(station, EvaluationSide.Right), 5e-6);
    }

    private static void Near(Vector3d expected, Vector3d actual, double tolerance = 2e-7) =>
        Assert.IsLessThan(tolerance, (expected - actual).Length, $"Expected {expected}, got {actual}.");
}
