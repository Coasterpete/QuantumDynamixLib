using System;
using System.Linq;
using System.Threading;
using System.Threading.Tasks;
using Avalonia;
using Avalonia.Controls;
using Avalonia.Headless;
using Avalonia.Input;
using Avalonia.Interactivity;
using Avalonia.Threading;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using QuantumDynamix.App;
using QuantumDynamix.Math.Timelines;
using SystemMath = System.Math;

namespace QuantumDynamix.Tests.Application;

[TestClass]
[DoNotParallelize]
public sealed class M3EditingFixtureTests
{
    [TestMethod]
    public async Task KeyframePreviewsVerticallyAndCommitsOneUndoableChange()
    {
        await WithGraph((window, graph) =>
        {
            M3FixtureTimeline initial = graph.Committed;
            var offset = new Point(3, 2);
            Point press = KeyframePoint(graph, window) + offset;
            Point middle = WindowPoint(graph.Graph, window, graph.Viewport.Project(new(58, .62))) + offset;
            Point release = WindowPoint(graph.Graph, window, graph.Viewport.Project(new(62, .75))) + offset;
            window.MouseDown(press, MouseButton.Left);
            window.MouseMove(middle, RawInputModifiers.LeftMouseButton);
            Assert.AreSame(initial, graph.Committed);
            Assert.IsFalse(graph.CanUndo);
            Assert.AreEqual(.62, graph.Displayed.Keyframe.Value, 1e-12);
            Point keyframePoint = graph.Viewport.Project(new(initial.Keyframe.Coordinate, .62));
            Assert.IsTrue(graph.CurvePaths("Blue").Any(span => SystemMath.Abs(span[^1].X - keyframePoint.X) < 1e-10
                && SystemMath.Abs(span[^1].Y - keyframePoint.Y) < 1e-10));

            window.MouseMove(release, RawInputModifiers.LeftMouseButton);
            window.MouseUp(release, MouseButton.Left);
            M3FixtureTimeline accepted = graph.Committed;
            Assert.AreSame(accepted, graph.Displayed);
            Assert.AreEqual(initial.Keyframe.Coordinate, accepted.Keyframe.Coordinate);
            Assert.AreEqual(.75, accepted.Keyframe.Value, 1e-12);
            Assert.AreEqual(.75, Evaluate(accepted, "Blue", 40), 1e-12);
            Assert.AreEqual((.12 + .75) / 2, Evaluate(accepted, "Blue", 20), 1e-12);
            Assert.AreEqual(Evaluate(initial, "Red", 40), Evaluate(accepted, "Red", 40));
            AssertContinuousJoins(accepted);

            Press(window, M3EditingFixture.UndoGesture);
            Assert.AreSame(initial, graph.Committed);
            Assert.IsFalse(graph.CanUndo);
            Assert.IsTrue(graph.CanRedo);
            Press(window, M3EditingFixture.RedoGesture);
            Assert.AreSame(accepted, graph.Committed);
            Assert.IsFalse(graph.CanRedo);
        });
    }

    [TestMethod]
    [DataRow(25.0, 46.0)]
    [DataRow(55.0, 40.0)]
    [DataRow(80.0, 89.0)]
    public async Task DiamondMovesSharedBoundaryWithoutMovingKeyframe(double station, double destination)
    {
        await WithGraph((window, graph) =>
        {
            Point keyframe = KeyframePoint(graph, window);
            window.MouseDown(keyframe, MouseButton.Left);
            window.MouseUp(WindowPoint(graph.Graph, window, graph.Viewport.Project(new(40, .7))), MouseButton.Left);
            M3FixtureTimeline initial = graph.Committed;
            int index = initial.Regions.ToList().FindIndex(region => region.Start == station);
            Point press = DiamondPoint(graph, window, station);
            Point release = DiamondPoint(graph, window, destination) + new Point(0, -30);

            window.MouseDown(press, MouseButton.Left);
            window.MouseMove(release, RawInputModifiers.LeftMouseButton);
            Assert.AreSame(initial, graph.Committed);
            Assert.AreEqual(destination, graph.Displayed.Regions[index].Start, 1e-12);
            window.MouseUp(release, MouseButton.Left);
            M3FixtureTimeline accepted = graph.Committed;
            Assert.AreEqual(destination, accepted.Regions[index].Start, 1e-12);
            Assert.AreEqual(accepted.Regions[index].Start, accepted.Regions[index - 1].End);
            Assert.AreEqual(initial.Keyframe, accepted.Keyframe);
            Assert.AreEqual(.7, Evaluate(accepted, "Blue", 40), 1e-12);
            Assert.AreEqual(.12 + (.7 - .12) * .75, Evaluate(accepted, "Blue", 30), 1e-12);
            Assert.AreEqual(.7 + (.92 - .7) / 3, Evaluate(accepted, "Blue", 60), 1e-12);
            AssertContinuousJoins(accepted);
            graph.Undo();
            Assert.AreSame(initial, graph.Committed);
            graph.Redo();
            Assert.AreSame(accepted, graph.Committed);
        });
    }

    [TestMethod]
    [DataRow(0.0)]
    [DataRow(55.0)]
    [DataRow(58.0)]
    public async Task InvalidBoundaryReleaseDiscardsEarlierPreviewAndKeepsRedo(double invalidStation)
    {
        await WithGraph((window, graph) =>
        {
            Point press = DiamondPoint(graph, window, 25);
            Point valid = DiamondPoint(graph, window, 32);
            Point invalid = DiamondPoint(graph, window, invalidStation);
            M3FixtureTimeline initial = graph.Committed;
            window.MouseDown(press, MouseButton.Left);
            window.MouseUp(valid, MouseButton.Left);
            M3FixtureTimeline priorEdit = graph.Committed;
            graph.Undo();
            Assert.AreSame(initial, graph.Committed);

            window.MouseDown(press, MouseButton.Left);
            window.MouseMove(valid, RawInputModifiers.LeftMouseButton);
            Assert.AreEqual(32, graph.Displayed.Regions[0].End, 1e-12);
            window.MouseMove(invalid, RawInputModifiers.LeftMouseButton);
            Assert.AreSame(initial, graph.Displayed);
            window.MouseUp(invalid, MouseButton.Left);
            Assert.AreSame(initial, graph.Committed);
            Assert.IsFalse(graph.CanUndo);
            graph.Redo();
            Assert.AreSame(priorEdit, graph.Committed);
        });
    }

    [TestMethod]
    [DataRow(false)]
    [DataRow(true)]
    public async Task CaptureLossAndEscapeCancelPreview(bool useEscape)
    {
        await WithGraph((window, graph) =>
        {
            M3FixtureTimeline initial = graph.Committed;
            Func<IPointer?> capturedPointer = ObservePointer(graph.Graph);
            Point start = DiamondPoint(graph, window, 25);
            Point end = DiamondPoint(graph, window, 36);
            window.MouseDown(start, MouseButton.Left);
            IPointer? pointer = capturedPointer();
            Assert.IsNotNull(pointer);
            Assert.AreSame(graph.Graph, pointer.Captured);
            window.MouseMove(end, RawInputModifiers.LeftMouseButton);
            Assert.AreEqual(36, graph.Displayed.Regions[0].End, 1e-12);
            if (useEscape)
            {
                Press(window, new KeyGesture(Key.Escape));
            }
            else
            {
                pointer.Capture(null);
            }
            Assert.IsNull(pointer.Captured);
            Assert.AreSame(initial, graph.Displayed);
            window.MouseUp(end, MouseButton.Left);
            Assert.AreSame(initial, graph.Committed);
            Assert.IsFalse(graph.CanUndo);
        });
    }

    [TestMethod]
    public async Task DragsUseResizedPlotAndKeepCaptureOutsideGraph()
    {
        await WithGraph((window, graph) =>
        {
            Resize(window, 760, 390);
            Point keyframe = KeyframePoint(graph, window);
            window.MouseDown(keyframe, MouseButton.Left);
            Resize(window, 840, 430);
            Point target = WindowPoint(graph.Graph, window, graph.Viewport.Project(new(40, .68)));
            window.MouseMove(target, RawInputModifiers.LeftMouseButton);
            Assert.AreEqual(.68, graph.Displayed.Keyframe.Value, 1e-12);
            window.MouseUp(target, MouseButton.Left);
            Assert.AreEqual(.68, graph.Committed.Keyframe.Value, 1e-12);

            Func<IPointer?> capturedPointer = ObservePointer(graph.Graph);
            window.MouseDown(DiamondPoint(graph, window, 25), MouseButton.Left);
            IPointer? pointer = capturedPointer();
            Assert.IsNotNull(pointer);
            Assert.AreSame(graph.Graph, pointer.Captured);
            Point outsideGraph = WindowPoint(graph.Graph, window, new(graph.Viewport.ProjectDistance(33), -8));
            window.MouseMove(outsideGraph, RawInputModifiers.LeftMouseButton);
            Assert.AreEqual(33, graph.Displayed.Regions[0].End, 1e-12);
            window.MouseUp(outsideGraph, MouseButton.Left);
            Assert.IsNull(pointer.Captured);
            Assert.AreEqual(33, graph.Committed.Regions[0].End, 1e-12);
        });
    }

    [TestMethod]
    public async Task DisplayOnlyMarkersAndNoOpDragsDoNotCreateHistory()
    {
        await WithGraph((window, graph) =>
        {
            M3FixtureTimeline initial = graph.Committed;
            Rect plot = graph.Viewport.Plot;
            double boundaryX = graph.Viewport.ProjectDistance(25);
            Point square = WindowPoint(graph.Graph, window, new(boundaryX, plot.Top));
            Point diamond = DiamondPoint(graph, window, 25);
            Point keyframe = KeyframePoint(graph, window);
            Drag(square, square + new Point(40, 20), MouseButton.Left);
            Drag(diamond, diamond + new Point(40, 0), MouseButton.Right);
            Drag(diamond, diamond, MouseButton.Left);
            Drag(keyframe, keyframe + new Point(40, 0), MouseButton.Left);
            Drag(keyframe, keyframe + new Point(0, -plot.Height), MouseButton.Left);
            Assert.AreSame(initial, graph.Committed);
            Assert.IsFalse(graph.CanUndo);

            void Drag(Point start, Point end, MouseButton button)
            {
                window.MouseDown(start, button);
                window.MouseMove(end, button == MouseButton.Left
                    ? RawInputModifiers.LeftMouseButton : RawInputModifiers.RightMouseButton);
                window.MouseUp(end, button);
            }
        });
    }

    private static Point WindowPoint(Control graph, Window window, Point point) =>
        graph.TranslatePoint(point, window) ?? throw new InvalidOperationException("Graph is outside the window tree.");

    private static Point KeyframePoint(M3EditingFixture graph, Window window)
    {
        TimelineKeyframe keyframe = graph.Committed.Keyframe;
        return WindowPoint(graph.Graph, window, graph.Viewport.Project(new(keyframe.Coordinate, keyframe.Value)));
    }

    private static Point DiamondPoint(M3EditingFixture graph, Window window, double station) =>
        WindowPoint(graph.Graph, window, new(graph.Viewport.ProjectDistance(station), graph.Viewport.Plot.Bottom + 8));

    private static double Evaluate(M3FixtureTimeline timeline, string channel, double station) =>
        timeline.Regions.First(region => station >= region.Start && station <= region.End).Channels[channel].Evaluate(station).Value;

    private static void AssertContinuousJoins(M3FixtureTimeline timeline)
    {
        for (int i = 1; i < timeline.Regions.Count; i++)
        {
            Region left = timeline.Regions[i - 1];
            Region right = timeline.Regions[i];
            Assert.AreEqual(left.End, right.Start);
            foreach (string channel in left.Channels.Keys)
            {
                RegionJoinDiagnostic join = left.DiagnoseJoin(right, channel, [1e-12]);
                Assert.AreEqual(0, join.MatchingOrder, $"{channel} at {join.Coordinate} m");
            }
        }
    }

    private static Func<IPointer?> ObservePointer(TransitionGraph graph)
    {
        IPointer? pointer = null;
        graph.AddHandler(InputElement.PointerPressedEvent, (_, e) => pointer = e.Pointer,
            RoutingStrategies.Bubble, handledEventsToo: true);
        return () => pointer;
    }

    private static void Press(Window window, KeyGesture gesture)
    {
        RawInputModifiers modifiers = RawInputModifiers.None;
        if (gesture.KeyModifiers.HasFlag(KeyModifiers.Control))
        {
            modifiers |= RawInputModifiers.Control;
        }
        if (gesture.KeyModifiers.HasFlag(KeyModifiers.Meta))
        {
            modifiers |= RawInputModifiers.Meta;
        }
        if (gesture.KeyModifiers.HasFlag(KeyModifiers.Shift))
        {
            modifiers |= RawInputModifiers.Shift;
        }
        window.KeyPress(gesture.Key, modifiers, PhysicalKey.None, null);
        window.KeyRelease(gesture.Key, modifiers, PhysicalKey.None, null);
    }

    private static void Resize(Window window, double width, double height)
    {
        window.Width = width;
        window.Height = height;
        window.UpdateLayout();
        Dispatcher.UIThread.RunJobs();
    }

    private static async Task WithGraph(Action<Window, M3EditingFixture> test)
    {
        await using var session = HeadlessUnitTestSession.StartNew(typeof(HeadlessAppHost));
        await session.Dispatch(() =>
        {
            var graph = new M3EditingFixture();
            var window = new Window
            {
                Width = 580,
                Height = 300,
                Content = new Border { Padding = new Thickness(18), Background = Avalonia.Media.Brushes.Black, Child = graph.Graph }
            };
            window.Show();
            try
            {
                window.UpdateLayout();
                Dispatcher.UIThread.RunJobs();
                using var initialFrame = window.CaptureRenderedFrame();
                test(window, graph);
            }
            finally
            {
                window.Close();
            }
        }, CancellationToken.None);
    }
}
