using System;
using System.Linq;
using System.Threading.Tasks;
using Avalonia;
using Avalonia.Controls;
using Avalonia.Headless;
using Avalonia.Input;
using Avalonia.Interactivity;
using Avalonia.LogicalTree;
using Avalonia.Threading;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using QuantumDynamix.App;
using QuantumDynamix.Geometry;
using QuantumDynamix.Math.Functions;
using QuantumDynamix.Math.Timelines;
using static QuantumDynamix.App.RegionTrackGeometry;

namespace QuantumDynamix.Tests.Application;

/// <summary>Direct manipulation in the Transition Editor and the embedded Acceleration Region Editor.</summary>
public sealed partial class AppNavigationTests
{
    [TestMethod]
    public async Task KeyframeDragUpdatesCurvesAndViewportLiveAndCommitsOneUndoStep()
    {
        await WithMainWindowAsync(async window =>
        {
            var (workspace, document, graph, viewport) = OpenLiveTrack(window, LiveEditingSessionTests.TwoRegionTrack());
            TimelineEditorSession session = document.Timeline;
            workspace.FindControl<ListBox>("ActiveChannel")!.SelectedItem = VerticalChannel;
            var regions = session.Regions;
            TrackWireframe committed = viewport.Scene;
            TimelineKeyframe key = session.Keys(0, VerticalChannel)[1];

            window.MouseDown(GraphPoint(window, graph, key.Coordinate, key.Value), MouseButton.Left);
            Assert.IsTrue(graph.IsDragging);
            Assert.AreEqual(1, workspace.FindControl<ListBox>("KeyframeList")!.SelectedIndex);
            foreach (double value in new[] { .2, .5, .8 })
            {
                window.MouseMove(GraphPoint(window, graph, 12, value), RawInputModifiers.LeftMouseButton);
                Assert.AreEqual(value, graph.DisplayedRegions[0].Channels[VerticalChannel].Evaluate(12).Value, 1e-12);
                Assert.AreSame(regions, session.Regions);
            }
            Assert.AreEqual(12, Number(workspace.FindControl<TextBox>("StationInput")!.Text), 1e-12);
            Assert.AreEqual(.8, Number(workspace.FindControl<TextBox>("ValueInput")!.Text), 1e-12);

            await workspace.WhenLivePreviewIdle();
            TrackWireframe preview = viewport.PreviewScene!;
            Assert.IsNotNull(preview);
            Assert.AreSame(committed, viewport.Scene);
            Assert.IsGreaterThan(.5, (preview.Boundaries[^1].Position - committed.Boundaries[^1].Position).Length);
            Assert.IsFalse(session.HasUnsavedChanges, "A drag does not change saved project data.");
            Capture(window, "live-keyframe-drag.png");

            window.MouseUp(GraphPoint(window, graph, 12, .8), MouseButton.Left);
            Assert.IsNull(viewport.PreviewScene);
            AssertKey(new(12, .8), session.Keys(0, VerticalChannel)[1]);
            Assert.IsTrue(session.HasUnsavedChanges);
            for (int i = 0; i < preview.Boundaries.Count; i++)
            {
                TestMath.Near(preview.Boundaries[i].Position, viewport.Scene.Boundaries[i].Position, 1e-12);
            }

            Click(window, "UndoButton");
            Assert.AreSame(regions, session.Regions, "The whole drag is one Undo step.");
            Assert.AreSame(committed, viewport.Scene);
            Assert.IsFalse(session.HasUnsavedChanges);
            Click(window, "RedoButton");
            AssertKey(new(12, .8), session.Keys(0, VerticalChannel)[1]);
        });
    }

    [TestMethod]
    [DataRow("Escape")]
    [DataRow("CaptureLoss")]
    [DataRow("Undo")]
    public async Task CancellingALiveDragRestoresTheCommittedState(string cancel)
    {
        await WithMainWindowAsync(async window =>
        {
            var (workspace, document, graph, viewport) = OpenLiveTrack(window, LiveEditingSessionTests.TwoRegionTrack());
            TimelineEditorSession session = document.Timeline;
            workspace.FindControl<ListBox>("ActiveChannel")!.SelectedItem = VerticalChannel;
            var regions = session.Regions;
            TrackWireframe committed = viewport.Scene;
            bool couldRedo = session.CanRedo;
            IPointer? pointer = null;
            graph.AddHandler(InputElement.PointerPressedEvent, (_, e) => pointer = e.Pointer, RoutingStrategies.Bubble, true);

            window.MouseDown(GraphPoint(window, graph, 10, 0), MouseButton.Left);
            window.MouseMove(GraphPoint(window, graph, 12, .8), RawInputModifiers.LeftMouseButton);
            await workspace.WhenLivePreviewIdle();
            Assert.IsNotNull(viewport.PreviewScene);

            RawInputModifiers command = OperatingSystem.IsMacOS() ? RawInputModifiers.Meta : RawInputModifiers.Control;
            switch (cancel)
            {
                case "Escape":
                    window.KeyPress(Key.Escape, RawInputModifiers.None, PhysicalKey.None, null);
                    window.KeyRelease(Key.Escape, RawInputModifiers.None, PhysicalKey.None, null);
                    break;
                case "CaptureLoss":
                    pointer!.Capture(null);
                    break;
                default:
                    window.KeyPress(Key.Z, command, PhysicalKey.None, null);
                    window.KeyRelease(Key.Z, command, PhysicalKey.None, null);
                    break;
            }
            Assert.IsFalse(graph.IsDragging);
            Assert.IsNull(viewport.PreviewScene);
            Assert.AreSame(graph.Regions, graph.DisplayedRegions);
            Assert.AreSame(committed, viewport.Scene);
            Assert.AreEqual(Local(10), workspace.FindControl<TextBox>("StationInput")!.Text);
            Assert.AreEqual(Local(0), workspace.FindControl<TextBox>("ValueInput")!.Text);

            window.MouseUp(GraphPoint(window, graph, 12, .8), MouseButton.Left);
            await workspace.WhenLivePreviewIdle();
            Assert.AreSame(regions, session.Regions, "Cancelling leaves the committed Regions and history in place.");
            Assert.AreEqual(couldRedo, session.CanRedo);
            Assert.IsFalse(session.HasUnsavedChanges);
            Assert.IsNull(viewport.PreviewScene);
        });
    }

    [TestMethod]
    public async Task RegionBoundaryDragIsSeparateFromEndpointKeyframeDrag()
    {
        await WithMainWindowAsync(async window =>
        {
            var (workspace, document, graph, viewport) = OpenLiveTrack(window, LiveEditingSessionTests.TwoRegionTrack());
            TimelineEditorSession session = document.Timeline;
            var regions = session.Regions;
            var status = workspace.FindControl<TextBlock>("StatusLabel")!;
            var length = workspace.FindControl<TextBox>("RegionLengthInput")!;
            double y = graph.Viewport.Plot.Top + graph.Viewport.Plot.Height * .4;
            Point Boundary(double station) =>
                graph.TranslatePoint(new Point(graph.Viewport.ProjectDistance(station), y), window)!.Value;

            // The end boundary of Region 1 resizes it. Region 2 keeps its length and moves.
            workspace.FindControl<ListBox>("RegionList")!.SelectedIndex = 1;
            window.MouseDown(Boundary(20), MouseButton.Left);
            Assert.IsTrue(graph.IsDragging);
            Assert.AreEqual(0, graph.SelectedRegion, "Grabbing a boundary selects the Region it ends.");
            Assert.AreEqual(regions[0].Id, viewport.ActiveRegionId);
            Assert.IsTrue(workspace.FindControl<StackPanel>("RegionFields")!.IsVisible);
            Assert.AreEqual(Local(20), length.Text);

            window.MouseMove(Boundary(26), RawInputModifiers.LeftMouseButton);
            Assert.AreEqual(26, graph.DisplayedRegions[0].End, 1e-12);
            Assert.AreEqual(26, graph.DisplayedRegions[1].Start, 1e-12);
            Assert.AreEqual(46, graph.DisplayedRegions[1].End, 1e-12);
            Assert.AreEqual(26, Number(length.Text), 1e-12);
            Assert.AreSame(regions, session.Regions);
            await workspace.WhenLivePreviewIdle();
            TrackWireframe lastValid = viewport.PreviewScene!;
            Assert.AreEqual(46, lastValid.Boundaries[^1].HeartlineStation, 1e-12);
            Capture(window, "live-boundary-drag.png");

            // An invalid intermediate position keeps the last valid preview.
            window.MouseMove(Boundary(-5), RawInputModifiers.LeftMouseButton);
            await workspace.WhenLivePreviewIdle();
            Assert.AreEqual(26, graph.DisplayedRegions[0].End, 1e-12);
            Assert.AreSame(lastValid, viewport.PreviewScene);
            StringAssert.Contains(status.Text!, "last valid preview");

            window.MouseMove(Boundary(26), RawInputModifiers.LeftMouseButton);
            window.MouseUp(Boundary(26), MouseButton.Left);
            Assert.AreEqual(26, session.Regions[0].Definition.End, 1e-12);
            Assert.AreEqual(46, session.Regions[1].Definition.End, 1e-12);
            Assert.AreEqual(46, viewport.Scene.Boundaries[^1].HeartlineStation, 1e-12);
            Assert.IsNull(viewport.PreviewScene);
            Click(window, "UndoButton");
            Assert.AreSame(regions, session.Regions);

            // An endpoint keyframe on the same boundary changes only its value, and the adjoining start follows.
            window.MouseDown(GraphPoint(window, graph, 20, 0), MouseButton.Left);
            Assert.IsTrue(workspace.FindControl<StackPanel>("KeyframeFields")!.IsVisible);
            window.MouseMove(GraphPoint(window, graph, 26, .3), RawInputModifiers.LeftMouseButton);
            Assert.AreEqual(20, graph.DisplayedRegions[0].End);
            const string bank = TrackDocument.BankingChannel;
            Assert.AreEqual(.3, graph.DisplayedRegions[0].Channels[bank].Evaluate(20, EvaluationSide.Left).Value, 1e-12);
            Assert.AreEqual(.3, graph.DisplayedRegions[1].Channels[bank].Evaluate(20, EvaluationSide.Right).Value, 1e-12);
            window.MouseUp(GraphPoint(window, graph, 26, .3), MouseButton.Left);
            await workspace.WhenLivePreviewIdle();
            Assert.AreEqual(20, session.Keys(0, bank)[1].Coordinate, "A boundary keyframe keeps its exact station.");
            Assert.AreEqual(.3, session.Keys(0, bank)[1].Value, 1e-12);
            Assert.AreEqual(20, session.Regions[0].Definition.End);
            Assert.AreEqual(session.Keys(0, bank)[1].Value, session.Keys(1, bank)[0].Value);
        });
    }

    [TestMethod]
    public async Task InlineAppendAndPrependCreateRegionsWithoutDialogs()
    {
        await WithMainWindowAsync(async window =>
        {
            TrackDocument document = TrackDocument.CreateAuthored("Inline Regions", RigidPose3d.Identity, 10);
            document.Timeline.MarkSaved();
            var (workspace, _, graph, viewport) = OpenLiveTrack(window, document);
            TimelineEditorSession session = document.Timeline;
            var status = workspace.FindControl<TextBlock>("StatusLabel")!;
            Assert.IsTrue(workspace.FindControl<Grid>("NewRegionPanel")!.IsVisible);
            Assert.IsTrue(workspace.FindControl<TextBox>("FirstStationInput")!.IsVisible);

            AddRegionInline(window, null);
            StringAssert.Contains(status.Text!, "first Region's heartline length");
            Assert.IsEmpty(session.Regions);

            AddRegionInline(window, "30", "10");
            Assert.HasCount(1, graph.Regions);
            Assert.AreEqual(10, graph.Regions[0].Start);
            Assert.IsTrue(viewport.HasGeometry);
            Assert.IsFalse(workspace.FindControl<TextBox>("FirstStationInput")!.IsVisible);
            Assert.AreEqual("", workspace.FindControl<TextBox>("NewRegionLength")!.Text);

            // A blank length repeats the adjoining Region's length. The selected type applies to the new Region.
            AddRegionInline(window, null);
            Assert.AreEqual(40, graph.Regions[1].Start);
            Assert.AreEqual(70, graph.Regions[1].End);
            Assert.AreEqual(RegionAuthoringMode.CoordinateAcceleration, session.Regions[1].Mode);
            AddRegionInline(window, null, mode: "Geometric");
            Assert.AreEqual(RegionAuthoringMode.Geometric, session.Regions[2].Mode);
            Assert.AreEqual(100, graph.Regions[2].End);
            Assert.IsTrue(graph.Regions[2].Channels.ContainsKey(PitchChangeChannel));
            Assert.AreEqual(session.Regions[2].Id, viewport.ActiveRegionId);
            Assert.HasCount(6, viewport.Scene.Boundaries);

            // Prepend fills at most the space before the first Region and keeps the adjoining type.
            AddRegionInline(window, null, mode: "Adjoining", prepend: true);
            Assert.HasCount(4, session.Regions);
            Assert.AreEqual(0, graph.Regions[0].Start);
            Assert.AreEqual(10, graph.Regions[0].End);
            Assert.AreEqual(RegionAuthoringMode.CoordinateAcceleration, session.Regions[0].Mode);
            Assert.IsFalse(workspace.FindControl<Button>("PrependRegionButton")!.IsEnabled);

            Click(window, "UndoButton");
            Assert.HasCount(3, session.Regions);
            Assert.AreEqual(10, graph.Regions[0].Start);
            Click(window, "RedoButton");
            Assert.HasCount(4, session.Regions);
            Assert.IsEmpty(window.OwnedWindows);
            Capture(window, "inline-regions.png");
            await Task.CompletedTask;
        });
    }

    [TestMethod]
    public async Task CurveAndEmptySpaceClicksSelectRegionsProfilesAndTransitions()
    {
        await WithMainWindowAsync(async window =>
        {
            TrackDocument document = LiveEditingSessionTests.TwoRegionTrack();
            Assert.IsNull(document.Timeline.Move(1, VerticalChannel, 1, new(40, .5)));
            document.Timeline.MarkSaved();
            var (workspace, _, graph, viewport) = OpenLiveTrack(window, document);
            var list = workspace.FindControl<ListBox>("RegionList")!;
            Assert.AreEqual(0, list.SelectedIndex);

            Click(window, GraphPoint(window, graph, 30, .25));
            Assert.AreEqual(1, list.SelectedIndex);
            Assert.AreEqual(1, graph.SelectedRegion);
            Assert.AreEqual(document.Timeline.Regions[1].Id, viewport.ActiveRegionId);
            Assert.AreEqual(VerticalChannel, workspace.FindControl<ListBox>("ActiveChannel")!.SelectedItem);
            Assert.IsTrue(workspace.FindControl<StackPanel>("TransitionFields")!.IsVisible);
            Assert.AreEqual(0, workspace.FindControl<ComboBox>("TransitionSpan")!.SelectedIndex);

            Click(window, GraphPoint(window, graph, 5, .7));
            Assert.AreEqual(0, list.SelectedIndex, "A click in empty graph space selects the Region under it.");
            Assert.AreEqual(document.Timeline.Regions[0].Id, viewport.ActiveRegionId);
            Assert.IsFalse(document.Timeline.HasUnsavedChanges);
            await Task.CompletedTask;
        });
    }

    [TestMethod]
    public async Task GraphPanAndZoomNeverEditRegionsOrKeyframes()
    {
        await WithMainWindowAsync(async window =>
        {
            var (workspace, document, graph, viewport) = OpenLiveTrack(window, LiveEditingSessionTests.TwoRegionTrack());
            TimelineEditorSession session = document.Timeline;
            var regions = session.Regions;
            int proposals = 0;
            graph.EditProposed += _ => proposals++;

            Point anchor = GraphPoint(window, graph, 10, .5);
            window.MouseWheel(anchor, new Vector(0, 2), RawInputModifiers.None);
            TransitionGraphViewport zoomed = graph.Viewport;
            Assert.AreEqual(40 * .85 * .85, zoomed.End - zoomed.Start, 1e-9);
            Point local = window.TranslatePoint(anchor, graph)!.Value;
            Assert.AreEqual(10, zoomed.Unproject(local).Distance, 1e-9, "The station under the pointer stays in place.");

            foreach (MouseButton button in new[] { MouseButton.Middle, MouseButton.Right, MouseButton.Left })
            {
                double start = graph.Viewport.Start;
                Point from = GraphPoint(window, graph, 30, .7);
                RawInputModifiers held = button switch
                {
                    MouseButton.Middle => RawInputModifiers.MiddleMouseButton,
                    MouseButton.Right => RawInputModifiers.RightMouseButton,
                    _ => RawInputModifiers.LeftMouseButton
                };
                window.MouseDown(from, button);
                window.MouseMove(from + new Point(-60, 25), held);
                window.MouseUp(from + new Point(-60, 25), button);
                Assert.IsGreaterThan(start, graph.Viewport.Start, $"{button} drag pans toward later stations.");
            }
            double before = graph.Viewport.Start;
            window.MouseWheel(GraphPoint(window, graph, 20, .5), new Vector(0, 1), RawInputModifiers.Shift);
            Assert.AreNotEqual(before, graph.Viewport.Start);

            Assert.AreEqual(0, proposals);
            Assert.AreSame(regions, session.Regions);
            Assert.IsFalse(session.HasUnsavedChanges);
            Assert.IsNull(viewport.PreviewScene);
            Assert.AreEqual(0, graph.SelectedRegion, "A panning drag does not change the selection.");
            Click(window, "FitButton");
            Assert.AreEqual(0, graph.Viewport.Start);
            Assert.AreEqual(40, graph.Viewport.End);
            await Task.CompletedTask;
        });
    }

    [TestMethod]
    public async Task AccelerationRegionEditorDragsKeyframesAndBoundariesInsideTheSettingsDraft()
    {
        await WithMainWindowAsync(async window =>
        {
            TrackDocument document = OperationalSegmentTests.AuthoredTrack();
            var accelerate = DeviceAccelerationRegion.Create([new(0, 0), new(10, 5), new(20, 5)],
                [TransitionShape.Linear, TransitionShape.Linear]);
            var brake = DeviceAccelerationRegion.Create([new(25, -2), new(30, -2)], [new(null)]);
            var segment = new OperationalSegment(Guid.NewGuid(), 15, 45, OperationalSegmentType.Transport,
                new PoweredTransportDevice(4, null, [accelerate, brake]));
            TimelineEditorSession session = document.Timeline;
            Assert.IsNull(session.SetSegment(segment));
            session.MarkSaved();
            OperationalSegment stored = session.Segments.Single();
            var (workspace, _, _, _) = OpenLiveTrack(window, document);
            ClickEditorControl(window, "OperationsButton");
            workspace.FindControl<ListBox>("SegmentList")!.SelectedIndex = 0;
            Click(window, "SegmentSettingsButton");
            TrackSegmentSettings dialog = SegmentDialog(window);
            dialog.UpdateLayout();
            Dispatcher.UIThread.RunJobs();
            Assert.IsEmpty(window.OwnedWindows.OfType<TrackSegmentSettings>().Where(owned => owned != dialog));

            var graph = (TransitionGraph)dialog.FindControl<ContentControl>("AccelerationGraphHost")!.Content!;
            Assert.HasCount(2, graph.Regions, "The graph shows every Acceleration Region of the segment.");
            Assert.AreEqual(0, graph.Viewport.Start);
            Assert.AreEqual(30, graph.Viewport.End);
            Point At(double offset, double value) =>
                graph.TranslatePoint(graph.Viewport.Project(new(offset, value)), dialog)!.Value;
            string channel = DeviceAccelerationRegion.Channel;

            dialog.MouseDown(At(10, 5), MouseButton.Left);
            dialog.MouseMove(At(12, 6), RawInputModifiers.LeftMouseButton);
            Assert.AreEqual(6, graph.DisplayedRegions[0].Channels[channel].Evaluate(12).Value, 1e-12);
            Assert.AreEqual(5, graph.Regions[0].Channels[channel].Evaluate(10).Value, 1e-12);
            dialog.MouseUp(At(12, 6), MouseButton.Left);
            Assert.AreEqual(6, graph.Regions[0].Channels[channel].Evaluate(12).Value, 1e-12);
            Assert.AreSame(stored, session.Segments.Single(), "Graph edits stay in the settings draft.");

            double y = graph.Viewport.Plot.Top + graph.Viewport.Plot.Height * .6;
            Point Boundary(double offset) => graph.TranslatePoint(new(graph.Viewport.ProjectDistance(offset), y), dialog)!.Value;
            dialog.MouseDown(Boundary(20), MouseButton.Left);
            dialog.MouseMove(Boundary(23), RawInputModifiers.LeftMouseButton);
            Assert.AreEqual(23, graph.DisplayedRegions[0].End, 1e-12);
            Capture(dialog, "acceleration-editor-boundary-drag.png");
            Assert.AreEqual(23, Number(dialog.FindControl<TextBox>("RegionEnd")!.Text), 1e-12);
            dialog.MouseMove(Boundary(27), RawInputModifiers.LeftMouseButton);
            Assert.AreEqual(23, graph.DisplayedRegions[0].End, 1e-12, "Overlapping the next Acceleration Region keeps the last valid preview.");
            dialog.MouseMove(Boundary(24), RawInputModifiers.LeftMouseButton);
            dialog.MouseUp(Boundary(24), MouseButton.Left);
            Assert.AreEqual(24, graph.Regions[0].End, 1e-12);
            Assert.AreEqual(12 * 24 / 20.0, graph.Regions[0].Channels[channel].Segments[0].Maximum, 1e-12);

            Button apply = dialog.GetLogicalDescendants().OfType<Button>().Single(button => Equals(button.Content, "Apply Settings"));
            Click(dialog, apply);
            DeviceAccelerationRegion applied = session.Segments.Single().Device!.Regions[0];
            Assert.AreEqual(24, applied.End, 1e-12);
            Assert.AreEqual(6, applied.Keys[1].Value, 1e-12);
            OpenMenu(workspace, "UndoMenu");
            Assert.AreSame(stored, session.Segments.Single(), "Apply Settings is one Undo step.");
            await Task.CompletedTask;
        });
    }

    [TestMethod]
    public async Task TrackSegmentSettingsSizesToTheSelectedSegmentType()
    {
        await WithMainWindowAsync(async window =>
        {
            TrackDocument document = OperationalSegmentTests.AuthoredTrack();
            document.Timeline.MarkSaved();
            var (workspace, _, _, _) = OpenLiveTrack(window, document);
            ClickEditorControl(window, "OperationsButton");
            Click(window, "AddSegmentButton");
            TrackSegmentSettings dialog = SegmentDialog(window);
            Settle(dialog);
            double normal = dialog.Bounds.Height;
            Assert.IsLessThan(220, normal, "Normal Track needs only the segment fields.");
            Capture(dialog, "segment-settings-normal.png");

            dialog.FindControl<ComboBox>("SegmentType")!.SelectedItem = "Transport";
            Settle(dialog);
            Assert.IsGreaterThan(normal + 250, dialog.Bounds.Height);
            Assert.IsGreaterThan(220, dialog.FindControl<ContentControl>("AccelerationGraphHost")!.Bounds.Height);
            Capture(dialog, "segment-settings-transport.png");
            Click(dialog, dialog.GetLogicalDescendants().OfType<Button>().Single(button => Equals(button.Content, "Cancel")));
            Assert.IsEmpty(workspace.FindControl<ListBox>("SegmentList")!.Items);
            await Task.CompletedTask;

            static void Settle(Window owned)
            {
                for (int pass = 0; pass < 3; pass++)
                {
                    owned.UpdateLayout();
                    Dispatcher.UIThread.RunJobs();
                }
            }
        });
    }

    private static (WorkspaceView Workspace, TrackDocument Document, TransitionGraph Graph, TrackViewport Viewport)
        OpenLiveTrack(Window window, TrackDocument document)
    {
        Click(window, "OpenEditorButton");
        var workspace = (WorkspaceView)window.Content!;
        workspace.OpenTrack(document);
        window.UpdateLayout();
        Dispatcher.UIThread.RunJobs();
        var graph = (TransitionGraph)workspace.FindControl<ContentControl>("TransitionGraphHost")!.Content!;
        return (workspace, document, graph, workspace.FindControl<TrackViewport>("ViewportSurface")!);
    }

    private static Point GraphPoint(Window window, TransitionGraph graph, double station, double value) =>
        graph.TranslatePoint(graph.Viewport.Project(new(station, value)), window)!.Value;

    private static void Click(Window window, Point point)
    {
        window.MouseDown(point, MouseButton.Left);
        window.MouseUp(point, MouseButton.Left);
        Dispatcher.UIThread.RunJobs();
    }

    // Drags carry the exact pointer displacement, so pointer-derived values are compared to the projection round trip.
    private static double Number(string? text)
    {
        Assert.IsTrue(WorkspaceView.TryReadNumber(text, out double value), $"'{text}' is not a finite number.");
        return value;
    }

    private static void AssertKey(TimelineKeyframe expected, TimelineKeyframe actual)
    {
        Assert.AreEqual(expected.Coordinate, actual.Coordinate, 1e-12);
        Assert.AreEqual(expected.Value, actual.Value, 1e-12);
    }
}
