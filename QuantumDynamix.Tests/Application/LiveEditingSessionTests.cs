using System;
using System.Collections.Concurrent;
using System.Linq;
using System.Threading;
using System.Threading.Tasks;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using QuantumDynamix.App;
using QuantumDynamix.Geometry;
using QuantumDynamix.Math.Functions;
using QuantumDynamix.Math.Timelines;
using static QuantumDynamix.App.RegionTrackGeometry;

namespace QuantumDynamix.Tests.Application;

/// <summary>Session rules behind live graph editing: drafts, previews, Region end moves, and endpoint keyframes.</summary>
[TestClass]
public sealed class LiveEditingSessionTests
{
    internal static TrackDocument TwoRegionTrack(RegionAuthoringMode mode = RegionAuthoringMode.CoordinateAcceleration)
    {
        TrackDocument document = TrackDocument.CreateAuthored("Live editing", RigidPose3d.Identity, 10);
        TimelineEditorSession session = document.Timeline;
        Assert.IsNull(session.AddRegion(prepend: false, 20, 0, mode));
        Assert.IsNull(session.AddRegion(prepend: false, 20, 0, mode));
        var (_, vertical) = TransverseChannels(mode);
        Assert.IsNull(session.SplitTransition(0, vertical, 0, insertBefore: false, out _));
        session.MarkSaved();
        return document;
    }

    [TestMethod]
    [DataRow(false)]
    [DataRow(true)]
    public void DraftValidatesWithoutCommittingAndPreviewMatchesTheCommittedGeometry(bool force)
    {
        RegionAuthoringMode mode = force ? RegionAuthoringMode.Force : RegionAuthoringMode.CoordinateAcceleration;
        TimelineEditorSession session = TwoRegionTrack(mode).Timeline;
        var (_, vertical) = TransverseChannels(mode);
        var regions = session.Regions;
        RegionTrackGeometry geometry = session.Geometry!;
        bool couldRedo = session.CanRedo;
        TimelineKeyframe key = session.Keys(0, vertical)[1];
        TimelineKeyframe moved = key with { Value = key.Value + (mode == RegionAuthoringMode.Force ? .4 : 2) };

        Assert.IsNull(session.Draft(edit => edit.Move(0, vertical, 1, moved), out TrackEditDraft? draft));
        Assert.IsNotNull(draft);
        Assert.AreSame(regions, session.Regions);
        Assert.AreSame(geometry, session.Geometry);
        Assert.IsFalse(session.HasUnsavedChanges);
        Assert.AreEqual(couldRedo, session.CanRedo);
        Assert.AreEqual(moved, draft.Regions[0].Keyframes[vertical][1]);

        Assert.IsNull(session.BuildPreview(draft, out TrackEditPreview? preview));
        Assert.IsNotNull(preview!.Geometry);
        Assert.AreSame(regions, session.Regions);

        Assert.IsNull(session.Move(0, vertical, 1, moved));
        WireframeBoundary[] committed = session.Geometry!.DisplayScene.Boundaries.ToArray();
        WireframeBoundary[] previewed = preview.Geometry.DisplayScene.Boundaries.ToArray();
        Assert.HasCount(committed.Length, previewed);
        for (int i = 0; i < committed.Length; i++)
        {
            Assert.AreEqual(committed[i].HeartlineStation, previewed[i].HeartlineStation);
            TestMath.Near(committed[i].Position, previewed[i].Position, 1e-12);
        }
        Assert.IsTrue(session.Undo());
        Assert.AreSame(regions, session.Regions);
        Assert.IsFalse(session.HasUnsavedChanges);
    }

    [TestMethod]
    public void DraftReportsTheSameValidationErrorsAndRejectsHistoryChanges()
    {
        TimelineEditorSession session = TwoRegionTrack().Timeline;
        string? committed = session.MoveRegionEnd(0, 0);
        Assert.IsNotNull(committed);
        Assert.AreEqual(committed, session.Draft(edit => edit.MoveRegionEnd(0, 0), out TrackEditDraft? draft));
        Assert.IsNull(draft);
        Assert.IsNull(session.Draft(edit => edit.MoveRegionEnd(0, 20), out draft));
        Assert.IsNull(draft, "An unchanged edit has no draft.");
        Assert.ThrowsExactly<InvalidOperationException>(() =>
            session.Draft(edit => { edit.Draft(_ => null, out _); return null; }, out _));
        Assert.ThrowsExactly<InvalidOperationException>(() => session.Draft(edit =>
            edit.SetSegment(new(Guid.NewGuid(), 1, 2, OperationalSegmentType.NormalTrack, null)), out _));
        Assert.IsEmpty(session.Segments);
        Assert.IsFalse(session.HasUnsavedChanges);
        Assert.IsNull(session.MoveRegionEnd(0, 22), "The session leaves draft mode after an exception.");
    }

    [TestMethod]
    public void MovingARegionEndRipplesFollowingRegionsAndKeepsOperationalStations()
    {
        TimelineEditorSession session = TwoRegionTrack().Timeline;
        Region second = session.Regions[1].Definition;
        double middle = session.Keys(0, VerticalChannel)[1].Coordinate;
        Assert.IsNull(session.MoveRegionEnd(0, 26));
        Assert.AreEqual(26, session.Regions[0].Definition.End);
        Assert.AreEqual(26, session.Regions[1].Definition.Start);
        Assert.AreEqual(46, session.Regions[1].Definition.End);
        Assert.AreEqual(middle * 26 / 20, session.Keys(0, VerticalChannel)[1].Coordinate, 1e-12);
        Assert.AreEqual(26, session.Geometry!.DisplayScene.Boundaries[1].HeartlineStation);
        Assert.IsTrue(session.Undo());
        Assert.AreSame(second, session.Regions[1].Definition);

        var segment = new OperationalSegment(Guid.NewGuid(), 30, 38, OperationalSegmentType.NormalTrack, null);
        Assert.IsNull(session.SetSegment(segment));
        StringAssert.Contains(session.MoveRegionEnd(0, 15)!, "within the authored heartline");
        Assert.IsNull(session.MoveRegionEnd(0, 19));
        Assert.AreEqual(segment, session.Segments.Single());
    }

    [TestMethod]
    public void EndpointKeyframesChangeOnlyEditableValuesAndKeepTheirStations()
    {
        TimelineEditorSession session = TwoRegionTrack().Timeline;
        Assert.AreEqual(KeyframeDragMode.ValueOnly, session.KeyframeEditMode(0, VerticalChannel, 0));
        Assert.AreEqual(KeyframeDragMode.Free, session.KeyframeEditMode(0, VerticalChannel, 1));
        Assert.AreEqual(KeyframeDragMode.ValueOnly, session.KeyframeEditMode(0, VerticalChannel, 2));
        Assert.AreEqual(KeyframeDragMode.None, session.KeyframeEditMode(1, VerticalChannel, 0));
        Assert.AreEqual(KeyframeDragMode.ValueOnly, session.KeyframeEditMode(1, VerticalChannel, 1));

        StringAssert.Contains(session.Move(0, VerticalChannel, 2, new(21, .5))!, "keeps its station");
        StringAssert.Contains(session.Move(1, VerticalChannel, 0, new(20, .5))!, "linked");
        Assert.IsNull(session.Move(0, VerticalChannel, 2, new(20, .5)));
        Assert.AreEqual(.5, session.Keys(1, VerticalChannel)[0].Value, "The adjoining Region's start is linked.");
        Assert.AreEqual(.5, session.Regions[1].Definition.Channels[VerticalChannel].Evaluate(20, EvaluationSide.Right).Value);

        var example = KeyframeExample.Create();
        Assert.AreEqual(KeyframeDragMode.None, example.KeyframeEditMode(0, "Red", 0));
        Assert.AreEqual(KeyframeDragMode.Free, example.KeyframeEditMode(0, "Red", 1));
        StringAssert.Contains(example.Move(0, "Red", 0, new(0, .3))!, "Region endpoints are fixed");
    }

    [TestMethod]
    public async Task LatestWinsWorkerSkipsSupersededRequestsAndIgnoresCancelledResults()
    {
        var applied = new ConcurrentQueue<string>();
        var worker = new LatestWinsWorker<string>(applied.Enqueue);
        using var gate = new ManualResetEventSlim();
        worker.Request(() => { gate.Wait(TimeSpan.FromSeconds(10)); return "first"; });
        worker.Request(() => "superseded");
        worker.Request(() => "latest");
        Assert.IsTrue(worker.IsBusy);
        gate.Set();
        await WaitUntilIdle(worker);
        CollectionAssert.AreEqual(new[] { "first", "latest" }, applied.ToArray());

        gate.Reset();
        worker.Request(() => { gate.Wait(TimeSpan.FromSeconds(10)); return "cancelled"; });
        worker.Cancel();
        gate.Set();
        await WaitUntilIdle(worker);
        CollectionAssert.AreEqual(new[] { "first", "latest" }, applied.ToArray());
    }

    // The worker is UI-thread affine. Without a UI synchronization context this test polls instead of awaiting WhenIdle.
    private static async Task WaitUntilIdle(LatestWinsWorker<string> worker)
    {
        using var timeout = new CancellationTokenSource(TimeSpan.FromSeconds(10));
        while (worker.IsBusy) { await Task.Delay(5, timeout.Token); }
    }
}
