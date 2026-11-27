using System;
using System.IO;
using System.Linq;
using System.Text.Json.Nodes;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using QuantumDynamix.App;
using QuantumDynamix.Geometry;
using QuantumDynamix.Math;
using QuantumDynamix.Math.Functions;
using QuantumDynamix.Math.Timelines;
using static QuantumDynamix.App.RegionTrackGeometry;

namespace QuantumDynamix.Tests.Application;

[TestClass]
public sealed class ProjectFileTests
{
    private DirectoryInfo directory = null!;
    private string Filename => Path.Combine(directory.FullName, "Test.qdproject");

    [TestInitialize]
    public void CreateDirectory() => directory = Directory.CreateTempSubdirectory("qd-project-tests-");

    [TestCleanup]
    public void RemoveDirectory() => directory.Delete(recursive: true);

    [TestMethod]
    public void EmptyProjectRetainsStartingPositionHeadingPitchAndSpeed()
    {
        TrackDocument original = NewTrack(System.Math.PI / 2);
        ProjectFile.Save(Filename, original);
        TrackDocument saved = ProjectFile.Load(Filename);
        Assert.AreEqual(original.Identity.Name, saved.Identity.Name);
        Assert.AreEqual(original.Timeline.InitialFrame, saved.Timeline.InitialFrame);
        Assert.AreEqual(original.Timeline.ReconstructionSpeed, saved.Timeline.ReconstructionSpeed);
        Assert.IsEmpty(saved.Timeline.Regions);
        Assert.IsNull(saved.Timeline.Geometry);
        Assert.IsNull(saved.Timeline.ReconstructionAnchor);
        Assert.IsFalse(saved.Timeline.HasUnsavedChanges);
        Assert.IsFalse(saved.Timeline.CanUndo);
        Assert.IsFalse(saved.Timeline.CanRedo);

        Assert.IsNull(original.Timeline.AddRegion(false, 10, 7, RegionAuthoringMode.Force));
        Assert.IsNull(saved.Timeline.AddRegion(false, 10, 7, RegionAuthoringMode.Force));
        Assert.AreEqual(original.Timeline.Geometry!.DisplayScene.Boundaries[0].Orientation,
            saved.Timeline.Geometry!.DisplayScene.Boundaries[0].Orientation);
        Assert.AreEqual(original.Timeline.Geometry.DisplayScene.Boundaries[^1].Position,
            saved.Timeline.Geometry.DisplayScene.Boundaries[^1].Position);
    }

    [TestMethod]
    [DataRow(false, false)]
    [DataRow(true, true)]
    [DataRow(false, true)]
    [DataRow(true, false)]
    public void ResizedRegionsRoundTripProfilesBankingGeometryAndForces(bool firstForce, bool secondForce)
    {
        TrackDocument original = NewTrack();
        TimelineEditorSession track = original.Timeline;
        RegionAuthoringMode first = firstForce ? RegionAuthoringMode.Force : RegionAuthoringMode.CoordinateAcceleration;
        RegionAuthoringMode second = secondForce ? RegionAuthoringMode.Force : RegionAuthoringMode.CoordinateAcceleration;
        var (lateral, vertical) = TransverseChannels(first);
        Assert.IsNull(track.AddRegion(false, 12.5, 8, first));
        Assert.IsNull(track.Add(0, vertical, new(14.25, 1.345678901234567), out _));
        Assert.IsNull(track.Add(0, lateral, new(14.25, .2456789012345678), out _));
        Assert.IsNull(track.Add(0, TrackDocument.BankingChannel, new(14.25, .3523456789012345), out _));
        Assert.IsNull(track.AddChannel(0, "Auxiliary φ", -2.2, 1.7));
        Assert.IsNull(track.AddRegion(false, 9.5, 0, second));
        Assert.IsNull(track.Add(1, TransverseChannels(second).Vertical, new(25.25, secondForce ? 1.25 : .25), out _));
        Assert.IsNull(track.UpdateRegion(0, "Lift exit", 15.75));

        ProjectFile.Save(Filename, original);
        Assert.AreEqual(1, (int)JsonNode.Parse(File.ReadAllText(Filename))!["version"]!);
        TrackDocument reopened = ProjectFile.Load(Filename);
        TimelineEditorSession loaded = reopened.Timeline;
        Assert.IsFalse(loaded.CanUndo);
        Assert.IsFalse(loaded.CanRedo);
        Assert.IsFalse(loaded.HasUnsavedChanges);
        Assert.AreEqual(8, loaded.Regions[0].Definition.Start);
        Assert.AreEqual(23.75, loaded.Regions[1].Definition.Start);
        Assert.AreEqual(33.25, loaded.Regions[1].Definition.End);
        Assert.AreEqual(28.5, loaded.Keys(1, TransverseChannels(second).Vertical)[1].Coordinate);
        Assert.AreEqual(.3523456789012345, loaded.Keys(0, TrackDocument.BankingChannel)[1].Value);
        Assert.AreEqual(.3523456789012345 / 2,
            loaded.Regions[0].Definition.Channels[TrackDocument.BankingChannel].Evaluate(11.9375).Value);
        AssertMatchingAuthoringAndResults(track, loaded);
        AssertJoin(loaded, 23.75);

        Assert.IsNull(track.Move(0, vertical, 1, new(16, firstForce ? 1.6 : 1.9)));
        Assert.IsNull(loaded.Move(0, vertical, 1, new(16, firstForce ? 1.6 : 1.9)));
        Assert.IsTrue(loaded.HasUnsavedChanges);
        AssertMatchingAuthoringAndResults(track, loaded);
        AssertJoin(loaded, 23.75);
        Assert.IsTrue(loaded.Undo());
        Assert.IsFalse(loaded.HasUnsavedChanges);
        Assert.IsTrue(loaded.Redo());
        Assert.IsTrue(loaded.HasUnsavedChanges);

        ProjectFile.Save(Filename, reopened);
        AssertMatchingAuthoringAndResults(loaded, ProjectFile.Load(Filename).Timeline);
    }

    [TestMethod]
    public void VersionTwoRoundTripsMixedModeNonlinearProfilesAndAllowsFurtherEditing()
    {
        TrackDocument original = NewTrack();
        TimelineEditorSession track = original.Timeline;
        Assert.IsNull(track.AddRegion(false, 20, 6, RegionAuthoringMode.Force));
        Assert.IsNull(track.Add(0, NormalForceChannel, new(12, 1.4), out _));
        Assert.IsNull(track.Add(0, NormalForceChannel, new(19, 1.2), out _));
        Assert.IsNull(track.UpdateTransition(0, NormalForceChannel, 0, 6, track.Keys(0, NormalForceChannel)[0].Value, 1.4,
            new(EasingKind.Sine, EasingDirection.Out)));
        Assert.IsNull(track.UpdateTransition(0, NormalForceChannel, 1, 7, 1.4, 1.2,
            new(EasingKind.Plateau, EasingDirection.InOut, .15, .72, .3)));
        Assert.IsNull(track.UpdateTransition(0, TrackDocument.BankingChannel, 0, 20, 0, .3,
            new(EasingKind.Quintic, EasingDirection.InOut)));
        Assert.IsNull(track.AddRegion(false, 12, 0, RegionAuthoringMode.Geometric));
        Assert.IsNull(track.Add(1, PitchChangeChannel, new(32, 3), out _));
        Assert.IsNull(track.UpdateTransition(1, PitchChangeChannel, 1, 6, 3, 2, new(EasingKind.Cubic)));
        ProjectFile.Save(Filename, original);
        var json = JsonNode.Parse(File.ReadAllText(Filename))!;
        Assert.AreEqual(2, (int)json["version"]!);
        TimelineEditorSession loaded = ProjectFile.Load(Filename).Timeline;
        AssertMatchingAuthoringAndResults(track, loaded);
        AssertJoin(loaded, 26);
        Assert.IsNull(loaded.Move(0, NormalForceChannel, 1, new(13, 1.5)));
        Assert.IsNull(track.Move(0, NormalForceChannel, 1, new(13, 1.5)));
        AssertMatchingAuthoringAndResults(track, loaded);

        var profile = json["track"]!["regions"]![0]!["profiles"]!.AsArray()
            .Single(item => (string?)item!["channel"] == NormalForceChannel)!;
        profile["transitions"]![0]!["type"] = "unregistered";
        File.WriteAllText(Filename, json.ToJsonString());
        StringAssert.Contains(Assert.ThrowsExactly<InvalidDataException>(() => ProjectFile.Load(Filename)).Message,
            "unsupported transition type");
    }

    [TestMethod]
    public void GeometricPrependKeepsTheCarriedHeadingOnReopenAfterPassingVertical()
    {
        TrackDocument original = NewTrack(80 * System.Math.PI / 180);
        TimelineEditorSession track = original.Timeline;
        Assert.IsNull(track.AddRegion(false, 8, 20, RegionAuthoringMode.Geometric));
        Assert.IsNull(track.UpdateTransition(0, PitchChangeChannel, 0, 8, -4, -4, new(null)));
        Assert.IsNull(track.UpdateTransition(0, YawChangeChannel, 0, 8, 2, 2, new(null)));
        Assert.IsNull(track.AddRegion(true, 6, 0, RegionAuthoringMode.Geometric));
        var previousEnd = track.Geometry!.DisplayScene.Boundaries[^1];
        Assert.IsNull(track.AddRegion(true, 5, 0, RegionAuthoringMode.Geometric));
        Assert.IsNotNull(track.ReconstructionAnchor!.Value.GeometricHeadingRadians);
        Assert.IsLessThan(2e-6, (previousEnd.Position - track.Geometry!.DisplayScene.Boundaries[^1].Position).Length);
        ProjectFile.Save(Filename, original);
        AssertMatchingAuthoringAndResults(track, ProjectFile.Load(Filename).Timeline);
    }

    [TestMethod]
    public void VersionOneRetainsAUserNamedProfileThatMatchesANewGeometricChannel()
    {
        TrackDocument original = NewTrack();
        TimelineEditorSession track = original.Timeline;
        Assert.IsNull(track.AddRegion(false, 10, 6));
        Assert.IsNull(track.AddChannel(0, PitchChangeChannel, 1, 2));
        ProjectFile.Save(Filename, original);
        Assert.AreEqual(1, (int)JsonNode.Parse(File.ReadAllText(Filename))!["version"]!);
        TimelineEditorSession loaded = ProjectFile.Load(Filename).Timeline;
        AssertMatchingAuthoringAndResults(track, loaded);
        Assert.AreEqual(1.5, loaded.Regions[0].Definition.Channels[PitchChangeChannel].Evaluate(11).Value, 1e-12);
    }

    [TestMethod]
    public void SavedPrependAndFirstRegionRemovalRetainWorldPose()
    {
        TrackDocument original = NewTrack();
        var timeline = original.Timeline;
        Assert.IsNull(timeline.AddRegion(false, 16, 10));
        Assert.IsNull(timeline.Add(0, VerticalChannel, new(18, 1.2), out _));
        Assert.IsNull(timeline.AddRegion(true, 6, 0, RegionAuthoringMode.Force));
        Assert.AreEqual(4, timeline.Regions[0].Definition.Start);
        Assert.AreEqual(10, timeline.ReconstructionAnchor!.Value.Station);
        ProjectFile.Save(Filename, original);
        TimelineEditorSession loaded = ProjectFile.Load(Filename).Timeline;
        AssertMatchingAuthoringAndResults(timeline, loaded);
        AssertJoin(loaded, 10);

        Assert.IsNull(timeline.DeleteRegion(0));
        Assert.IsNull(loaded.DeleteRegion(0));
        AssertMatchingAuthoringAndResults(timeline, loaded);
        ProjectFile.Save(Filename, original);
        AssertMatchingAuthoringAndResults(timeline, ProjectFile.Load(Filename).Timeline);

        // Removing an original curved Region moves the anchor to its downstream world pose.
        Assert.IsNull(timeline.AddRegion(false, 9, 0));
        Assert.IsNull(timeline.DeleteRegion(0));
        Assert.AreNotEqual(timeline.InitialFrame, timeline.ReconstructionAnchor!.Value.Frame);
        ProjectFile.Save(Filename, original);
        AssertMatchingAuthoringAndResults(timeline, ProjectFile.Load(Filename).Timeline);
    }

    [TestMethod]
    [DataRow("{", "Invalid or incomplete project JSON")]
    [DataRow("[]", "not a QUANTUM DYNAMIX project")]
    [DataRow("""{"format":"quantum-dynamix-project"}""", "integer format version")]
    [DataRow("""{"format":"quantum-dynamix-project","version":4}""", "version 4 is unsupported")]
    [DataRow("""{"format":"quantum-dynamix-project","version":1,"version":1}""", "Invalid or incomplete project JSON")]
    [DataRow("""{"format":"quantum-dynamix-project","version":1,"track":null}""", "Invalid or incomplete project JSON")]
    public void RejectsInvalidOrUnsupportedProjectEnvelopes(string json, string diagnostic)
    {
        File.WriteAllText(Filename, json);
        StringAssert.Contains(Assert.ThrowsExactly<InvalidDataException>(() => ProjectFile.Load(Filename)).Message, diagnostic);
    }

    [TestMethod]
    [DataRow("missingPosition", "Invalid or incomplete project JSON")]
    [DataRow("speed", "finite reconstruction speed")]
    [DataRow("invalidFrame", "orthonormal frame")]
    [DataRow("missingAnchor", "needs a reconstruction anchor")]
    [DataRow("anchorStation", "first Region join")]
    [DataRow("duplicateId", "unique, nonempty ID")]
    [DataRow("gap", "preceding Region's end")]
    [DataRow("mode", "unsupported authoring mode")]
    [DataRow("missingForceProfile", "both transverse profiles")]
    [DataRow("interpolation", "unsupported interpolation")]
    [DataRow("endpoint", "keyframes at both Region endpoints")]
    [DataRow("duplicateKey", "numeric limits")]
    [DataRow("impossibleBankJoin", "Track geometry could not be generated")]
    public void RejectsMalformedAuthoringData(string problem, string diagnostic)
    {
        TrackDocument document = NewTrack();
        Assert.IsNull(document.Timeline.AddRegion(false, 10, 0, RegionAuthoringMode.Force));
        Assert.IsNull(document.Timeline.AddRegion(false, 10, 0, RegionAuthoringMode.Force));
        ProjectFile.Save(Filename, document);

        JsonNode root = JsonNode.Parse(File.ReadAllText(Filename))!;
        JsonNode track = root["track"]!;
        JsonArray regions = track["regions"]!.AsArray();
        JsonNode first = regions[0]!;
        JsonNode second = regions[1]!;
        JsonArray profiles = first["profiles"]!.AsArray();
        JsonNode bank = profiles.Single(profile => (string?)profile!["channel"] == TrackDocument.BankingChannel)!;
        JsonArray keys = bank["keyframes"]!.AsArray();
        switch (problem)
        {
            case "missingPosition": track["startingFrame"]!.AsObject().Remove("positionMetres"); break;
            case "speed": track["reconstructionSpeedMetresPerSecond"] = 0; break;
            case "invalidFrame": track["startingFrame"]!["up"] = track["startingFrame"]!["left"]!.DeepClone(); break;
            case "missingAnchor": track["reconstructionAnchor"] = null; break;
            case "anchorStation": track["reconstructionAnchor"]!["stationMetres"] = 5; break;
            case "duplicateId": second["id"] = first["id"]!.DeepClone(); break;
            case "gap": second["startMetres"] = 11; break;
            case "mode": first["mode"] = "other"; break;
            case "missingForceProfile": profiles.Remove(profiles.Single(profile => (string?)profile!["channel"] == NormalForceChannel)); break;
            case "interpolation": bank["interpolation"] = "cubic"; break;
            case "endpoint": keys[0]!["stationMetres"] = 1; break;
            case "duplicateKey": keys.Insert(1, keys[0]!.DeepClone()); break;
            case "impossibleBankJoin": keys[^1]!["value"] = 1; break;
            default: Assert.Fail($"Unknown test case {problem}."); break;
        }
        File.WriteAllText(Filename, root.ToJsonString());
        StringAssert.Contains(Assert.ThrowsExactly<InvalidDataException>(() => ProjectFile.Load(Filename)).Message, diagnostic);
    }

    [TestMethod]
    public void FailedReplacementCleansUpTemporaryFileAndLeavesExistingData()
    {
        TrackDocument document = NewTrack();
        ProjectFile.Save(Filename, document);
        byte[] saved = File.ReadAllBytes(Filename);

        if (OperatingSystem.IsWindows())
        {
            using var locked = new FileStream(Filename, FileMode.Open, FileAccess.Read, FileShare.Read);
            Exception error = Assert.Throws<Exception>(() =>
                ProjectFile.Save(Filename, TrackDocument.CreateAuthored("Replacement", RigidPose3d.Identity, 8)));
            Assert.IsTrue(error is IOException or UnauthorizedAccessException);
        }
        string existingDirectory = directory.CreateSubdirectory("Existing.qdproject").FullName;
        string existingFile = Path.Combine(existingDirectory, "Original.qdproject");
        File.WriteAllBytes(existingFile, saved);
        Exception failedMove = Assert.Throws<Exception>(() => ProjectFile.Save(existingDirectory, NewTrack()));
        Assert.IsTrue(failedMove is IOException or UnauthorizedAccessException);

        CollectionAssert.AreEqual(saved, File.ReadAllBytes(Filename));
        CollectionAssert.AreEqual(saved, File.ReadAllBytes(existingFile));
        CollectionAssert.AreEqual(new[] { Filename }, Directory.GetFiles(directory.FullName));
    }

    private static TrackDocument NewTrack(double pitch = .32) =>
        TrackDocument.CreateAuthored("Alpine φ", new(new(4.124567890123456, -3, 7),
            Quaterniond.FromAxisAngle(Vector3d.UnitZ, .61) * Quaterniond.FromAxisAngle(Vector3d.UnitY, -pitch)),
            12.123456789012345);

    private static void AssertJoin(TimelineEditorSession track, double station)
    {
        Vector3d left = track.Geometry!.CalculateSpecificForceG(station, EvaluationSide.Left);
        Vector3d right = track.Geometry.CalculateSpecificForceG(station, EvaluationSide.Right);
        Assert.IsLessThan(1e-10, (left - right).Length);
    }

    private static void AssertMatchingAuthoringAndResults(TimelineEditorSession expected, TimelineEditorSession actual)
    {
        Assert.AreEqual(expected.InitialFrame, actual.InitialFrame);
        Assert.AreEqual(expected.ReconstructionSpeed, actual.ReconstructionSpeed);
        Assert.AreEqual(expected.ReconstructionAnchor, actual.ReconstructionAnchor);
        Assert.HasCount(expected.Regions.Count, actual.Regions);
        for (int i = 0; i < expected.Regions.Count; i++)
        {
            AuthoredRegion before = expected.Regions[i];
            AuthoredRegion after = actual.Regions[i];
            Assert.AreEqual(before.Id, after.Id);
            Assert.AreEqual(before.Name, after.Name);
            Assert.AreEqual(before.Mode, after.Mode);
            Assert.AreEqual(before.Definition.Start, after.Definition.Start);
            Assert.AreEqual(before.Definition.End, after.Definition.End);
            CollectionAssert.AreEqual(before.Keyframes.Keys.ToArray(), after.Keyframes.Keys.ToArray());
            foreach (string channel in before.Keyframes.Keys)
            {
                CollectionAssert.AreEqual(before.Keyframes[channel], after.Keyframes[channel]);
                CollectionAssert.AreEqual(before.Shapes(channel), after.Shapes(channel));
                double station = before.Definition.Start + .38 * (before.Definition.End - before.Definition.Start);
                Assert.AreEqual(before.Definition.Channels[channel].Evaluate(station), after.Definition.Channels[channel].Evaluate(station));
            }
            double midpoint = (before.Definition.Start + before.Definition.End) / 2;
            Assert.AreEqual(expected.Geometry!.CalculateSpecificForceG(midpoint, EvaluationSide.Right),
                actual.Geometry!.CalculateSpecificForceG(midpoint, EvaluationSide.Right));
        }
        TrackWireframe oldScene = expected.Geometry!.DisplayScene;
        TrackWireframe newScene = actual.Geometry!.DisplayScene;
        CollectionAssert.AreEqual(oldScene.Boundaries.ToArray(), newScene.Boundaries.ToArray());
        Assert.HasCount(oldScene.Paths.Count, newScene.Paths);
        foreach (var (before, after) in oldScene.Paths.Zip(newScene.Paths))
        {
            Assert.AreEqual(before.Kind, after.Kind);
            Assert.AreEqual(before.Id, after.Id);
            Assert.AreEqual(before.RegionId, after.RegionId);
            CollectionAssert.AreEqual(before.Vertices.ToArray(), after.Vertices.ToArray());
        }
    }
}
