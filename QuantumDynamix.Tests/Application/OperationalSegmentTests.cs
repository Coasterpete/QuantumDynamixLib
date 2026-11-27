using System;
using System.IO;
using System.Linq;
using System.Text.Json.Nodes;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using QuantumDynamix.App;
using QuantumDynamix.Geometry;
using QuantumDynamix.Math.Functions;

namespace QuantumDynamix.Tests.Application;

[TestClass]
public sealed class OperationalSegmentTests
{
    [TestMethod]
    public void AbsoluteStationsSurviveRegionMovesAndRejectOrphaningEdits()
    {
        TimelineEditorSession session = AuthoredTrack().Timeline;
        var segment = new OperationalSegment(Guid.NewGuid(), 15, 42, OperationalSegmentType.NormalTrack, null);
        var before = session.Geometry;
        var region = session.Regions[0];
        Assert.IsNull(session.SetSegment(segment));
        Assert.AreSame(before, session.Geometry);
        Assert.AreSame(region, session.Regions[0]);
        session.MarkSaved();

        Assert.IsNull(session.UpdateRegion(0, region.Name, 25));
        Assert.AreEqual(35, session.Regions[1].Definition.Start);
        Assert.AreEqual(segment, session.Segments.Single());
        Assert.IsTrue(session.HasUnsavedChanges);
        Assert.IsTrue(session.Undo());
        Assert.AreSame(before, session.Geometry);
        Assert.IsFalse(session.HasUnsavedChanges);

        Assert.IsNotNull(session.UpdateRegion(0, region.Name, 1));
        Assert.IsNotNull(session.DeleteRegion(0));
        Assert.IsNotNull(session.DeleteRegion(1));
        Assert.IsFalse(session.HasUnsavedChanges);
        Assert.IsTrue(session.CanRedo);

        Assert.IsNull(session.SetSegment(segment with { Start = 18, End = 45 }));
        Assert.AreEqual(18, session.Segments.Single().Start);
        Assert.IsFalse(session.CanRedo);
        var overlapped = new OperationalSegment(Guid.NewGuid(), 18, 19, OperationalSegmentType.NormalTrack, null);
        StringAssert.Contains(session.SetSegment(overlapped)!, "overlap");
        Assert.AreSame(before, session.Geometry);
        Assert.IsTrue(session.Undo());
        Assert.AreEqual(15, session.Segments.Single().Start);
        Assert.IsFalse(session.HasUnsavedChanges);
        Assert.IsTrue(session.Redo());
        Assert.AreEqual(45, session.Segments.Single().End);
    }

    [TestMethod]
    public void DistanceCommandsUseTransitionsAndAnActiveSpeedLimit()
    {
        var acceleration = DeviceAccelerationRegion.Create([new(0, 0), new(6, 3)], [new(EasingKind.Quadratic)]);
        var braking = DeviceAccelerationRegion.Create([new(8, -1), new(10, -1)], [new(null)]);
        var unrestricted = new PoweredTransportDevice(2, null, [acceleration, braking]).Calculate(10);
        var middle = unrestricted.Sample(3);
        Assert.AreEqual(.75, middle.CommandedAcceleration, 1e-10);
        // a(s) = s²/12 over the first six metres: v² = 4 + s³/18.
        Assert.AreEqual(System.Math.Sqrt(5.5), middle.Speed, 1e-8);
        Assert.AreEqual(System.Math.Sqrt(12), unrestricted.Sample(10).Speed, 1e-8);

        var limited = new PoweredTransportDevice(2, 3, [acceleration, braking]).Calculate(10);
        var held = limited.Sample(5.75);
        Assert.IsGreaterThan(0, held.CommandedAcceleration);
        Assert.AreEqual(0, held.CalculatedAcceleration);
        Assert.AreEqual(3, held.Speed, 1e-8);
        Assert.AreEqual(3, limited.Sample(7).Speed, 1e-8);
        Assert.AreEqual(-1, limited.Sample(8).CalculatedAcceleration);
        Assert.AreEqual(System.Math.Sqrt(5), limited.Sample(10).Speed, 1e-8);
        Assert.IsGreaterThan(held.ElapsedSeconds, limited.Sample(10).ElapsedSeconds);
    }

    [TestMethod]
    public void SpeedLimitReleasesWhenOneNonlinearTransitionChangesSign()
    {
        var command = DeviceAccelerationRegion.Create([new(0, 2), new(10, -2)], [new(EasingKind.Quadratic)]);
        var device = new PoweredTransportDevice(3, 4, [command]);
        var result = device.Calculate(10);
        Assert.AreEqual(4, result.Sample(5).Speed, 1e-8);
        Assert.AreEqual(0, result.Sample(5).CalculatedAcceleration);
        Assert.IsLessThan(4, result.Sample(10).Speed);
        Assert.AreEqual(-2, result.Sample(10).CalculatedAcceleration, 1e-10);

        var reversed = DeviceAccelerationRegion.Create([new(0, -1), new(10, 2)], [new(EasingKind.Sine, EasingDirection.InOut)]);
        var other = (device with { Regions = [reversed] }).Calculate(10);
        Assert.IsLessThan(3, other.Sample(2).Speed);
        Assert.AreEqual(4, other.Sample(10).Speed, 1e-8);
    }

    [TestMethod]
    public void RestLaunchAndBrakingStopUseExistingMotionAndDoNotReverse()
    {
        var launch = DeviceAccelerationRegion.Create([new(0, 2), new(4, 2)], [new(null)]);
        var brake = DeviceAccelerationRegion.Create([new(4, -1), new(15, -1)], [new(null)]);
        var device = new PoweredTransportDevice(0, 10, [launch, brake]);
        var motion = device.Calculate(15);
        Assert.AreEqual(0, motion.Sample(0).Speed);
        Assert.AreEqual(2, motion.Sample(1).Speed, 1e-9);
        Assert.AreEqual(4, motion.Sample(4).Speed, 1e-9);
        Assert.AreEqual(-1, motion.Sample(4).CommandedAcceleration);
        Assert.AreEqual(12, motion.End, 1e-9);
        Assert.IsTrue(motion.Stopped);
        var stopped = motion.Sample(motion.End);
        Assert.AreEqual(6, stopped.ElapsedSeconds, 1e-9);
        Assert.AreEqual(0, stopped.Speed);
        Assert.AreEqual(0, stopped.CalculatedAcceleration);
        Assert.AreEqual(-1, stopped.CommandedAcceleration);
        Assert.ThrowsExactly<ArgumentOutOfRangeException>(() => motion.Sample(13));

        var rest = (device with { Regions = [] }).Calculate(15);
        Assert.IsTrue(rest.Stopped);
        Assert.AreEqual(0, rest.End);
        Assert.AreEqual(0, rest.Sample(0).ElapsedSeconds);
    }

    [TestMethod]
    public void ImpossibleNonlinearCommandCanBeEditedAndRecalculated()
    {
        var braking = DeviceAccelerationRegion.Create([new(0, -2), new(8, -3)], [new(EasingKind.Cubic)]);
        var device = new PoweredTransportDevice(1, null, [braking]);
        var failure = Assert.ThrowsExactly<InvalidOperationException>(() => device.Calculate(8));
        StringAssert.Contains(failure.Message, "offsets");
        Assert.ThrowsExactly<ArgumentException>(() => braking.Edit(0, 8, -2, -3, new(null)));
        Assert.AreEqual(-2, braking.Keys[0].Value);
        var recover = braking.Edit(0, 8, 1, 3, new(EasingKind.Cubic, EasingDirection.Out));
        var motion = (device with { Regions = [recover] }).Calculate(8);
        Assert.AreEqual(8, motion.End);
        Assert.IsFalse(motion.Stopped);
        Assert.IsGreaterThan(1, motion.Sample(8).Speed);
    }

    [TestMethod]
    public void VersionThreePersistsDeviceCommandsAndRebuildsTheSameGeometry()
    {
        TrackDocument track = AuthoredTrack();
        var region = DeviceAccelerationRegion.Create([new(2, 0), new(6, 1), new(15, -1)],
            [new(EasingKind.Plateau, EasingDirection.Out, .2, .7, .4), new(EasingKind.Quintic, EasingDirection.InOut)]);
        var secondary = DeviceAccelerationRegion.Zero(18, 22);
        var plain = new OperationalSegment(Guid.NewGuid(), 10, 14, OperationalSegmentType.NormalTrack, null);
        var segment = new OperationalSegment(Guid.NewGuid(), 14, 42, OperationalSegmentType.Transport, new(5, 8, [region, secondary]));
        Assert.IsNull(track.Timeline.SetSegment(segment));
        Assert.IsNull(track.Timeline.SetSegment(plain));
        string path = Path.GetTempFileName();
        try
        {
            ProjectFile.Save(path, track);
            string json = File.ReadAllText(path);
            var root = JsonNode.Parse(json)!;
            Assert.AreEqual(3, (int)root["version"]!);
            Assert.IsFalse(json.Contains("elapsedSeconds", StringComparison.OrdinalIgnoreCase));
            TrackDocument reopened = ProjectFile.Load(path);
            Assert.AreEqual(plain, reopened.Timeline.Segments[0]);
            OperationalSegment restored = reopened.Timeline.Segments[1];
            Assert.AreEqual(segment.Id, restored.Id);
            Assert.AreEqual(segment.Start, restored.Start);
            Assert.AreEqual(segment.End, restored.End);
            Assert.AreEqual(5, restored.Device!.EntrySpeed);
            Assert.AreEqual(8d, restored.Device.SpeedLimit);
            CollectionAssert.AreEqual(region.Keys.ToArray(), restored.Device.Regions[0].Keys.ToArray());
            CollectionAssert.AreEqual(region.Shapes.ToArray(), restored.Device.Regions[0].Shapes.ToArray());
            Assert.AreEqual(18, restored.Device.Regions[1].Start);
            Assert.IsFalse(reopened.Timeline.HasUnsavedChanges);
            Assert.IsFalse(reopened.Timeline.CanUndo);
            CollectionAssert.AreEqual(track.SampleWireframe(30).Paths.SelectMany(path => path.Vertices).ToArray(),
                reopened.SampleWireframe(30).Paths.SelectMany(path => path.Vertices).ToArray());

            root["version"] = 2;
            File.WriteAllText(path, root.ToJsonString());
            StringAssert.Contains(Assert.ThrowsExactly<InvalidDataException>(() => ProjectFile.Load(path)).Message, "version 3");
            root["version"] = 3;
            root["track"]!["operationalSegments"]![1]!["device"]!["type"] = "chainLift";
            File.WriteAllText(path, root.ToJsonString());
            StringAssert.Contains(Assert.ThrowsExactly<InvalidDataException>(() => ProjectFile.Load(path)).Message, "unsupported device");
        }
        finally { File.Delete(path); }
    }

    internal static TrackDocument AuthoredTrack()
    {
        TrackDocument track = TrackDocument.CreateAuthored("Operations Test Track", RigidPose3d.Identity, 10);
        Assert.IsNull(track.Timeline.AddRegion(prepend: false, 20, 10, RegionAuthoringMode.Force));
        Assert.IsNull(track.Timeline.AddRegion(prepend: false, 30, 0, RegionAuthoringMode.Geometric));
        return track;
    }
}
