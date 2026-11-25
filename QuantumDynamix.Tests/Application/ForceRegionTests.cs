using System;
using System.Collections.Generic;
using System.Linq;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using QuantumDynamix.App;
using QuantumDynamix.Geometry;
using QuantumDynamix.Math;
using QuantumDynamix.Math.Functions;
using QuantumDynamix.Math.Timelines;
using static QuantumDynamix.App.RegionTrackGeometry;

namespace QuantumDynamix.Tests.Application;

[TestClass]
public sealed class ForceRegionTests
{
    private const double G = 9.80665;
    private const double Speed = 10;
    private static readonly CurveFrame Level = new(Vector3d.Zero, Vector3d.UnitX, Vector3d.UnitY, Vector3d.UnitZ);

    [TestMethod]
    [DataRow(0.0)]
    [DataRow(0.6)]
    [DataRow(1.5707963267948966)]
    public void StraightStartUsesGravityResolvedInTheActualRiderFrame(double pitch)
    {
        var pose = new RigidPose3d(new(4, -3, 7), Quaterniond.FromAxisAngle(Vector3d.UnitZ, .8) *
            Quaterniond.FromAxisAngle(Vector3d.UnitY, -pitch));
        var session = new TimelineEditorSession(pose, Speed);
        Assert.IsNull(session.AddRegion(false, 12, 20, RegionAuthoringMode.Force));
        Region region = session.Regions[0].Definition;
        Assert.AreEqual(System.Math.Cos(pitch), region.Channels[NormalForceChannel].Evaluate(20).Value, 1e-14);
        Assert.AreEqual(0, region.Channels[LateralForceChannel].Evaluate(20).Value, 1e-14);
        AssertNear(pose.Position + pose.TransformDirection(Vector3d.UnitX) * 12,
            session.Geometry!.DisplayScene.Boundaries[1].Position);
        AssertNear(new(System.Math.Sin(pitch), 0, System.Math.Cos(pitch)),
            session.Geometry.CalculateSpecificForceG(27.13, EvaluationSide.Right), 1e-7);
    }

    [TestMethod]
    [DataRow(0.0)]
    [DataRow(0.45)]
    public void ConstantLateralAccelerationMakesTheAnalyticHorizontalCircleWithBankedRails(double bank)
    {
        const double lateral = .6;
        const double length = 19;
        var (sin, cos) = System.Math.SinCos(bank);
        double riderLeft = lateral * cos + sin;
        double riderUp = cos - lateral * sin;
        AuthoredRegion region = ForceRegion(8, 8 + length, riderLeft, riderUp, bank);
        var geometry = new RegionTrackGeometry([region], Speed, Level, 8);
        double curvature = lateral * G / (Speed * Speed);
        var (turnSin, turnCos) = System.Math.SinCos(curvature * length);
        CurveFrame end = geometry.DisplayScene.Boundaries[1].Orientation!.Value;
        AssertNear(new(turnSin / curvature, (1 - turnCos) / curvature, 0), end.Position);
        AssertNear(new(turnCos, turnSin, 0), end.Forward);
        AssertNear(new(-turnSin * cos, turnCos * cos, sin), end.Left);
        AssertNear(new(turnSin * sin, -turnCos * sin, cos), end.Up);
        Vector3d leftRail = PathEnd("left");
        Vector3d rightRail = PathEnd("right");
        AssertNear(end.Left, leftRail - rightRail);
        AssertNear(end.Position - end.Up * .75, (leftRail + rightRail) * .5);
        AssertNear(end.Position - end.Up, PathEnd("Spine"));
        AssertNear(new(0, riderLeft, riderUp), geometry.CalculateSpecificForceG(11.234, EvaluationSide.Right), 4e-6);

        Vector3d PathEnd(string id) => geometry.DisplayScene.Paths.Single(path => path.Id == id).Vertices[^1].Position;
    }

    [TestMethod]
    public void ZeroTransverseSpecificForceUsesEvolvingGravityProjectionWhilePrepending()
    {
        const double station = 12;
        AuthoredRegion before = ForceRegion(2, station, 0, 0, 0);
        AuthoredRegion after = ForceRegion(station, 22, 0, 0, 0);
        var geometry = new RegionTrackGeometry([before, after], Speed, Level, station);
        Assert.AreEqual(Level, geometry.DisplayScene.Boundaries[1].Orientation!.Value);
        Assert.AreEqual(Level, geometry.DisplayScene.Boundaries[2].Orientation!.Value);
        double k = G / (Speed * Speed);
        AssertAnalytic(geometry.DisplayScene.Boundaries[0], -10);
        AssertAnalytic(geometry.DisplayScene.Boundaries[^1], 10);
        Vector3d specific = geometry.CalculateSpecificForceG(5.132, EvaluationSide.Right);
        Assert.AreEqual(0, specific.Y, 4e-6);
        Assert.AreEqual(0, specific.Z, 4e-6);

        void AssertAnalytic(WireframeBoundary point, double s)
        {
            AssertNear(new(System.Math.Atan(System.Math.Sinh(k * s)) / k,
                0, -System.Math.Log(System.Math.Cosh(k * s)) / k), point.Position);
            AssertNear(new(1 / System.Math.Cosh(k * s), 0, -System.Math.Tanh(k * s)), point.Orientation!.Value.Forward);
        }
    }

    [TestMethod]
    public void MixedModesRebindDownstreamStartsAndRestoreTheWholeSnapshot()
    {
        var session = new TimelineEditorSession(RigidPose3d.Identity, Speed);
        Assert.IsNull(session.AddRegion(false, 20, 10, RegionAuthoringMode.Force));
        Assert.IsNull(session.AddRegion(false, 8, 0, RegionAuthoringMode.CoordinateAcceleration));
        Assert.IsNull(session.AddRegion(false, 7, 0, RegionAuthoringMode.Force));
        RegionTrackGeometry initial = session.Geometry!;
        Assert.IsNull(session.Add(0, NormalForceChannel, new(20, 1.65), out _));
        Assert.IsNull(session.Add(0, LateralForceChannel, new(20, .32), out _));
        RegionTrackGeometry edited = session.Geometry!;
        AuthoredRegion[] definitions = session.Regions.ToArray();
        Assert.AreEqual(0, definitions[1].Keyframes[VerticalChannel][^1].Value);
        Assert.IsGreaterThan(.1, definitions[1].Keyframes[VerticalChannel][0].Value);
        Assert.AreNotEqual(1, definitions[2].Keyframes[NormalForceChannel][0].Value);
        Assert.IsGreaterThan(.1, (initial.DisplayScene.Boundaries[^1].Position - edited.DisplayScene.Boundaries[^1].Position).Length);
        AssertJoin(edited, 1, 30);
        AssertJoin(edited, 3, 38);

        Assert.IsTrue(session.Undo());
        Assert.IsTrue(session.Undo());
        Assert.AreSame(initial, session.Geometry);
        Assert.AreEqual(0, session.Regions[1].Keyframes[VerticalChannel][0].Value);
        Assert.IsTrue(session.Redo());
        Assert.IsTrue(session.Redo());
        Assert.AreSame(edited, session.Geometry);
        CollectionAssert.AreEqual(definitions, session.Regions.ToArray());
    }

    [TestMethod]
    public void PrependingAndDeletingAMixedRegionRetainsTheExistingTrackAnchor()
    {
        var pose = new RigidPose3d(new(2, 3, 4), Quaterniond.FromAxisAngle(Vector3d.UnitY, -.4));
        var session = new TimelineEditorSession(pose, Speed);
        Assert.IsNull(session.AddRegion(false, 12, 10));
        Assert.IsNull(session.Add(0, VerticalChannel, new(16, 1), out _));
        TrackWireframe original = session.Geometry!.DisplayScene;
        Assert.IsNull(session.AddRegion(true, 6, 0, RegionAuthoringMode.Force));
        AssertJoin(session.Geometry!, 1, 10);
        TrackWireframe extended = session.Geometry!.DisplayScene;
        AssertNear(pose.Position - pose.TransformDirection(Vector3d.UnitX) * 6, extended.Boundaries[0].Position);
        AssertNear(original.Boundaries[^1].Position, extended.Boundaries[^1].Position);
        Assert.IsNull(session.DeleteRegion(0));
        AssertNear(original.Boundaries[0].Position, session.Geometry!.DisplayScene.Boundaries[0].Position);
        AssertNear(original.Boundaries[^1].Position, session.Geometry.DisplayScene.Boundaries[^1].Position);
    }

    [TestMethod]
    public void InvalidOrUnrepresentableTargetsLeaveThePreviousGeometryAndRedoIntact()
    {
        var session = new TimelineEditorSession(RigidPose3d.Identity, Speed);
        Assert.IsNotNull(session.AddRegion(false, 0, 0, RegionAuthoringMode.Force));
        Assert.IsFalse(session.CanUndo);
        Assert.IsNull(session.AddRegion(false, 20, 0, RegionAuthoringMode.Force));
        RegionTrackGeometry straight = session.Geometry!;
        Assert.IsNull(session.Add(0, NormalForceChannel, new(10, 1.5), out _));
        RegionTrackGeometry bent = session.Geometry!;
        Assert.IsTrue(session.Undo());
        Assert.IsNotNull(session.Add(0, NormalForceChannel, new(10, double.NaN), out _));
        StringAssert.Contains(session.Add(0, NormalForceChannel, new(10, 3e307), out _)!, "geometry could not be generated");
        Assert.AreSame(straight, session.Geometry);
        Assert.HasCount(2, session.Keys(0, NormalForceChannel));
        Assert.IsTrue(session.Redo());
        Assert.AreSame(bent, session.Geometry);
    }

    private static AuthoredRegion ForceRegion(double start, double end, double left, double up, double bank)
    {
        var keys = new Dictionary<string, TimelineKeyframe[]>
        {
            [LateralForceChannel] = [new(start, left), new(end, left)],
            [NormalForceChannel] = [new(start, up), new(end, up)],
            [TrackDocument.BankingChannel] = [new(start, bank), new(end, bank)]
        };
        var channels = keys.Select(pair => KeyValuePair.Create(pair.Key,
            ScalarTimeline.FromKeyframes(ProfileDomain.Distance, pair.Value)));
        return new(Guid.NewGuid(), "Force", new(ProfileDomain.Distance, start, end, channels), keys, RegionAuthoringMode.Force);
    }

    private static void AssertJoin(RegionTrackGeometry geometry, int incoming, double station)
    {
        CurveFrame left = geometry.DisplayScene.Boundaries[incoming].Orientation!.Value;
        CurveFrame right = geometry.DisplayScene.Boundaries[incoming + 1].Orientation!.Value;
        AssertNear(left.Position, right.Position);
        AssertNear(left.Forward, right.Forward);
        AssertNear(left.Up, right.Up);
        AssertNear(geometry.CalculateSpecificForceG(station, EvaluationSide.Left),
            geometry.CalculateSpecificForceG(station, EvaluationSide.Right), 1e-10);
    }

    private static void AssertNear(Vector3d expected, Vector3d actual, double tolerance = 2e-6) =>
        Assert.IsLessThan(tolerance, (expected - actual).Length, $"Expected {expected}, found {actual}.");
}
