using System;
using System.Linq;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using QuantumDynamix.App;
using QuantumDynamix.Geometry;
using QuantumDynamix.Math;
using QuantumDynamix.Math.Functions;
using QuantumDynamix.Track;
using static QuantumDynamix.Tests.RailTestFixtures;

namespace QuantumDynamix.Tests.Application;

[TestClass]
public sealed class TrackWireframeTests
{
    [TestMethod]
    [DataRow(0.0, 0.0)]
    [DataRow(0.6, 0.4)]
    public void StraightAuthoredRegionUsesTheUprightCrossSectionInItsLocalFrame(double heading, double pitch)
    {
        const double length = 12;
        var pose = new RigidPose3d(new(4, -3, 7), Quaterniond.FromAxisAngle(Vector3d.UnitZ, heading) *
            Quaterniond.FromAxisAngle(Vector3d.UnitY, -pitch));
        TrackDocument document = TrackDocument.CreateAuthored("Straight", pose, 10);
        Assert.IsNull(document.Timeline.AddRegion(prepend: false, length: length, initialStation: 0));
        TrackWireframe scene = document.SampleWireframe(RegionTrackGeometry.DisplayIntervals);
        Assert.IsEmpty(scene.Diagnostics);

        foreach (WireframeBoundary boundary in scene.Boundaries)
        {
            CurveFrame frame = boundary.Orientation!.Value;
            TestMath.Near(pose.TransformDirection(Vector3d.UnitX), frame.Forward, 1e-10);
            TestMath.Near(pose.TransformDirection(Vector3d.UnitY), frame.Left, 1e-10);
            TestMath.Near(pose.TransformDirection(Vector3d.UnitZ), frame.Up, 1e-10);
        }

        foreach (var (id, left, up) in new[] { ("Heartline", 0.0, 0.0), ("Spine", 0.0, -1.0),
            ("left", .5, -.75), ("right", -.5, -.75) })
        {
            WireframeVertex[] vertices = Vertices(scene, id);
            Assert.AreEqual(0d, vertices[0].HeartlineStation);
            Assert.AreEqual(length, vertices[^1].HeartlineStation);
            foreach (WireframeVertex vertex in vertices)
            {
                Vector3d expected = pose.Position + pose.TransformDirection(new(vertex.HeartlineStation!.Value, left, up));
                TestMath.Near(expected, vertex.Position, 1e-10);
            }
        }
    }

    [TestMethod]
    public void ReferenceGeometryUsesTheDocumentedSplineAndOffsetCenterlines()
    {
        TrackDocument document = ReferenceTrack.Create();
        TrackWireframe scene = document.SampleWireframe(16);
        double length = document.Timeline.Regions.Single().Definition.End;
        Assert.IsEmpty(scene.Diagnostics);
        Assert.HasCount(3, scene.ControlPoints);
        Assert.AreEqual(new Vector3d(10, 10, 0), scene.ControlPoints[1].Position);
        Assert.HasCount(2, scene.Boundaries);
        Assert.AreEqual(length, scene.Boundaries[1].HeartlineStation);
        TestMath.Near(Vector3d.UnitY, scene.Boundaries[0].Orientation!.Value.Forward, 1e-12);
        Assert.IsFalse(scene.Paths.Any(path => path.Kind == WireframePathKind.Support));

        foreach ((string id, double radius) in new[] { ("Heartline", 10.0), ("Spine", 9.0), ("inner", 8.5), ("outer", 9.5) })
        {
            WireframeVertex[] vertices = Vertices(scene, id);
            Assert.AreEqual(0d, vertices[0].HeartlineStation);
            Assert.AreEqual(length, vertices[^1].HeartlineStation);
            foreach (WireframeVertex vertex in vertices)
            {
                double angle = vertex.HeartlineStation!.Value / 10;
                TestMath.Near(new(radius * System.Math.Cos(angle), radius * System.Math.Sin(angle), 0), vertex.Position, 1e-8);
            }
        }
    }

    [TestMethod]
    public void OnlyExplicitBankingBindingMovesOffsetsAndHistoryRestoresThem()
    {
        TrackDocument document = ReferenceTrack.Create();
        TimelineEditorSession timeline = document.Timeline;
        const string channel = TrackDocument.BankingChannel;
        double station = timeline.Keys(0, channel)[1].Coordinate;
        double angle = System.Math.PI / 6;
        TrackWireframe before = document.SampleWireframe(16);
        Assert.IsNotNull(timeline.PreviewMove(0, channel, 1, new(station, angle)));
        CollectionAssert.AreEqual(Vertices(before, "inner"), Vertices(document.SampleWireframe(16), "inner"));

        Assert.IsNull(timeline.Move(0, channel, 1, new(station, angle)));
        TrackWireframe after = document.SampleWireframe(16);
        CollectionAssert.AreEqual(Vertices(before, "Heartline"), Vertices(after, "Heartline"));
        Assert.AreEqual(.75, Vertices(after, "inner").First(v => v.HeartlineStation == station).Position.Z, 1e-12);
        Assert.AreEqual(.5, Vertices(after, "Spine").First(v => v.HeartlineStation == station).Position.Z, 1e-12);
        Assert.IsTrue(timeline.Undo());
        CollectionAssert.AreEqual(Vertices(before, "inner"), Vertices(document.SampleWireframe(16), "inner"));
        Assert.IsTrue(timeline.Redo());
        CollectionAssert.AreEqual(Vertices(after, "inner"), Vertices(document.SampleWireframe(16), "inner"));

        double end = timeline.Regions[0].Definition.End;
        Assert.IsNull(timeline.AddRegion(prepend: false, length: 1, initialStation: 0));
        Assert.IsNull(timeline.Add(1, channel, new(end + .5, 1), out _));
        CollectionAssert.AreEqual(Vertices(after, "inner"), Vertices(document.SampleWireframe(16), "inner"));
        Assert.IsNull(timeline.DeleteRegion(0));
        TrackWireframe missingBinding = document.SampleWireframe(16);
        Assert.IsTrue(missingBinding.Paths.All(path => path.Kind == WireframePathKind.Heartline));
        Assert.IsTrue(missingBinding.Diagnostics.Any(message => message.Contains("bound", StringComparison.Ordinal)));
    }

    [TestMethod]
    public void RailBreakpointsUseBothSidesAndSupportsUseAuthoredDatums()
    {
        var railOffset = new CustomFunction(0, 10, (s, side) => new(s > 5 || (s == 5 && side == EvaluationSide.Right) ? 2 : 0), [5]);
        var definition = new RailDefinition("rail", new(Constant(0, 10), Constant(0, 10), railOffset), Section(1), Constant(0, 10));
        RailGeometry rails = Straight(definition);
        Vector3d foundation = new(6, 1, -1);
        var source = new TrackWireframeSource(rails.Track, rails,
            supports: [new("member", "rail", "positive", 6, foundation)]);
        TrackWireframe scene = source.Sample([], 12);
        WireframePath[] trackRail = scene.Paths.Where(path => path.Kind == WireframePathKind.Rail).ToArray();
        Assert.HasCount(2, trackRail);
        TestMath.Near(new(5, 0, 0), trackRail[0].Vertices[^1].Position, 1e-12);
        TestMath.Near(new(5, 0, 2), trackRail[1].Vertices[0].Position, 1e-12);
        WireframePath support = scene.Paths.Single(path => path.Kind == WireframePathKind.Support);
        Assert.AreEqual(foundation, support.Vertices[0].Position);
        TestMath.Near(new(6, 1, 2), support.Vertices[1].Position, 1e-12);
        var heartlineOnly = new TrackWireframeSource(new ArcLengthMap(Line()));
        Assert.IsTrue(heartlineOnly.Sample([], 4).Paths.All(path => path.Kind == WireframePathKind.Heartline));
    }

    internal static WireframeVertex[] Vertices(TrackWireframe scene, string pathId) =>
        scene.Paths.Where(path => path.Id == pathId).SelectMany(path => path.Vertices).ToArray();
}
