using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Threading;
using System.Threading.Tasks;
using Avalonia;
using Avalonia.Controls;
using Avalonia.Headless;
using Avalonia.Input;
using Avalonia.Interactivity;
using Avalonia.Media.Imaging;
using Avalonia.Threading;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using QuantumDynamix.App;
using QuantumDynamix.Math.Functions;
using QuantumDynamix.Math.Timelines;
using SkiaSharp;
using DesktopApp = QuantumDynamix.App.App;
using SystemMath = System.Math;

namespace QuantumDynamix.Tests.Application;

[TestClass]
[DoNotParallelize]
public sealed class TransitionGraphHeadlessTests
{
    [TestMethod]
    public async Task RenderedPathsAndPixelsMatchTimelineBeforeAndAfterResize()
    {
        IReadOnlyList<Region> regions = TransitionGraphFixture.CreateRegions();
        await WithGraph(regions, (window, graph) =>
        {
            CheckRenderedCurves(window, graph, regions);
            Rect initialPlot = graph.Viewport.Plot;
            window.Width = 810;
            window.Height = 350;
            window.UpdateLayout();
            Dispatcher.UIThread.RunJobs();
            Assert.IsGreaterThan(initialPlot.Width, graph.Viewport.Plot.Width);
            Assert.IsGreaterThan(initialPlot.Height, graph.Viewport.Plot.Height);
            CheckRenderedCurves(window, graph, regions);
        });
    }

    [TestMethod]
    public async Task GraphRendersMoreThanFiveChannelsAndFourRegionsWithMissingChannelSpans()
    {
        Region[] regions = Enumerable.Range(0, 7).Select(region =>
        {
            double start = 10 * region;
            double end = start + 10;
            return new Region(ProfileDomain.Distance, start, end,
                Enumerable.Range(0, region % 2 == 0 ? 8 : 7).Select(channel =>
                    KeyValuePair.Create($"Channel {channel}", ScalarTimeline.FromKeyframes(
                        ProfileDomain.Distance, [new(start, .05 * channel), new(end, .15 + .05 * channel)]))));
        }).ToArray();

        await WithGraph(regions, (_, graph) =>
        {
            Assert.HasCount(8, graph.Channels);
            Assert.HasCount(7, graph.Regions);
            Point[][] paths = graph.CurvePaths("Channel 7").ToArray();
            Assert.HasCount(4, paths);
            for (int index = 0; index < paths.Length; index++)
            {
                Assert.AreEqual(20 * index, graph.Viewport.Unproject(paths[index][0]).Distance, 1e-12);
                Assert.AreEqual(20 * index + 10, graph.Viewport.Unproject(paths[index][^1]).Distance, 1e-12);
            }
        });
    }

    [TestMethod]
    [DataRow(false)]
    [DataRow(true)]
    public async Task ExplicitDragTargetReceivesDistanceAndValueFromPointerEvents(bool isBoundary)
    {
        var keyframe = new TimelineKeyframe(25, .4);
        bool moved = false;
        var target = new TransitionDragTarget(0, 55, (start, current) =>
        {
            moved = current.Distance > start.Distance;
            return isBoundary ? new TransitionEdit.Boundary(25, current.Distance)
                : new TransitionEdit.Keyframe("Blue", keyframe, new(current.Distance, current.Value));
        });
        await WithGraph(TransitionGraphFixture.CreateRegions(), (window, graph) =>
        {
            var proposals = new List<TransitionEdit>();
            graph.EditProposed += proposals.Add;
            Point press = WindowPoint(graph, window, graph.Viewport.Project(new(25, .4)));
            Point release = WindowPoint(graph, window, graph.Viewport.Project(new(40, .7)));
            window.MouseDown(press, MouseButton.Right);
            window.MouseUp(release, MouseButton.Right);
            Assert.IsEmpty(proposals);

            window.MouseDown(press, MouseButton.Left);
            window.MouseMove(release, RawInputModifiers.LeftMouseButton);
            Assert.IsTrue(moved);
            Assert.IsEmpty(proposals);
            window.MouseUp(release, MouseButton.Left);
            Assert.HasCount(1, proposals);
            if (isBoundary)
            {
                var boundary = (TransitionEdit.Boundary)proposals[0];
                Assert.AreEqual(25, boundary.Original);
                Assert.AreEqual(40, boundary.Proposed, 1e-12);
            }
            else
            {
                var point = (TransitionEdit.Keyframe)proposals[0];
                Assert.AreEqual(keyframe, point.Original);
                Assert.AreEqual(40, point.Proposed.Coordinate, 1e-12);
                Assert.AreEqual(.7, point.Proposed.Value, 1e-12);
            }

            Point outsideNeighbors = WindowPoint(graph, window, graph.Viewport.Project(new(60, .7)));
            window.MouseDown(press, MouseButton.Left);
            window.MouseMove(outsideNeighbors, RawInputModifiers.LeftMouseButton);
            window.MouseUp(outsideNeighbors, MouseButton.Left);
            Assert.HasCount(1, proposals);

            IPointer? capturedPointer = null;
            graph.AddHandler(InputElement.PointerPressedEvent, (_, args) => capturedPointer = args.Pointer,
                RoutingStrategies.Bubble, handledEventsToo: true);
            window.MouseDown(press, MouseButton.Left);
            window.MouseMove(release, RawInputModifiers.LeftMouseButton);
            Assert.IsNotNull(capturedPointer);
            Assert.AreSame(graph, capturedPointer.Captured);
            capturedPointer.Capture(null);
            window.MouseUp(release, MouseButton.Left);
            Assert.HasCount(1, proposals);
        }, _ => target);
    }

    [TestMethod]
    public async Task SquareAndDiamondAreInertWithoutABoundaryPolicy()
    {
        IReadOnlyList<Region> regions = TransitionGraphFixture.CreateRegions();
        await WithGraph(regions, (window, graph) =>
        {
            var proposals = new List<TransitionEdit>();
            graph.EditProposed += proposals.Add;
            int presses = 0;
            graph.AddHandler(InputElement.PointerPressedEvent, (_, _) => presses++, RoutingStrategies.Bubble, handledEventsToo: true);
            Rect plot = graph.Viewport.Plot;
            double boundaryX = graph.Viewport.ProjectDistance(25);
            foreach (double y in new[] { plot.Top, plot.Bottom + 8 })
            {
                Point marker = WindowPoint(graph, window, new(boundaryX, y));
                Point end = WindowPoint(graph, window, new(boundaryX + 40, y));
                window.MouseDown(marker, MouseButton.Left);
                window.MouseMove(end, RawInputModifiers.LeftMouseButton);
                window.MouseUp(end, MouseButton.Left);
            }
            Assert.AreEqual(2, presses);
            Assert.IsEmpty(proposals);
            Assert.AreSame(regions, graph.Regions);
            Assert.IsFalse(graph.IsDragging);
        });
    }

    [TestMethod]
    public async Task BoundaryPolicyMakesTheSquareAndDiamondDragTheRegionBoundary()
    {
        IReadOnlyList<Region> regions = TransitionGraphFixture.CreateRegions();
        await WithGraph(regions, (window, graph) =>
        {
            graph.BoundaryPolicy = boundary => boundary.Left >= 0 && boundary.Right >= 0
                ? new BoundaryDragLimits(regions[boundary.Left].Start, regions[boundary.Right].End) : null;
            var proposals = new List<TransitionEdit>();
            var selected = new List<GraphBoundary>();
            graph.EditProposed += proposals.Add;
            graph.BoundarySelected += selected.Add;
            Rect plot = graph.Viewport.Plot;
            foreach ((double y, double target) in new[] { (plot.Bottom + 8, 40.0), (plot.Top, 30.0) })
            {
                Point start = WindowPoint(graph, window, new(graph.Viewport.ProjectDistance(25), y));
                Point end = WindowPoint(graph, window, new(graph.Viewport.ProjectDistance(target), y + 6));
                window.MouseDown(start, MouseButton.Left);
                window.MouseMove(end, RawInputModifiers.LeftMouseButton);
                Assert.IsTrue(graph.IsDragging);
                Assert.AreEqual(target, ((TransitionEdit.Boundary)graph.DragCandidate!).Proposed, 1e-12);
                window.MouseUp(end, MouseButton.Left);
                var moved = (TransitionEdit.Boundary)proposals[^1];
                Assert.AreEqual(25, moved.Original);
                Assert.AreEqual(target, moved.Proposed, 1e-12);
                Assert.AreEqual((0, 1), (moved.Left, moved.Right));
            }

            // The outer start has no policy limits. It can be selected, but not dragged.
            Point outer = WindowPoint(graph, window, new(graph.Viewport.ProjectDistance(0), plot.Top + 30));
            window.MouseDown(outer, MouseButton.Left);
            window.MouseMove(outer + new Point(40, 0), RawInputModifiers.LeftMouseButton);
            window.MouseUp(outer + new Point(40, 0), MouseButton.Left);
            Assert.HasCount(2, proposals);
            Assert.AreEqual(new GraphBoundary(0, -1, 0), selected[^1]);
        });
    }

    [TestMethod]
    public async Task PointerJitterDoesNotMoveAKeyframeAndEachAxisUnlocksSeparately()
    {
        await WithGraph(TransitionGraphFixture.CreateRegions(), (window, graph) =>
        {
            graph.SetSelection(0, "Blue", [new(0, .1), new(10, .4), new(25, .2)], -1);
            var proposals = new List<TransitionEdit.Keyframe>();
            graph.EditProposed += edit => proposals.Add((TransitionEdit.Keyframe)edit);
            Point key = WindowPoint(graph, window, graph.Viewport.Project(new(10, .4)));

            Drag(new Point(1, -2));
            Assert.AreEqual(proposals[^1].Original, proposals[^1].Proposed, "A click with a slight movement is not an edit.");
            Drag(new Point(30, 1));
            Assert.AreEqual(.4, proposals[^1].Proposed.Value, "The value stays exact until the pointer moves vertically.");
            // Past the threshold the station follows the exact pointer displacement, without rounding.
            Point local = window.TranslatePoint(key, graph)!.Value;
            double expected = 10 + (graph.Viewport.Unproject(local + new Point(30, 1)).Distance - graph.Viewport.Unproject(local).Distance);
            Assert.AreEqual(expected, proposals[^1].Proposed.Coordinate, 1e-12);
            Assert.AreEqual(1, graph.SelectedKeyframe);

            void Drag(Point offset)
            {
                window.MouseDown(key, MouseButton.Left);
                window.MouseMove(key + offset, RawInputModifiers.LeftMouseButton);
                window.MouseUp(key + offset, MouseButton.Left);
            }
        });
    }

    [TestMethod]
    [DataRow(EasingKind.Quadratic, EasingDirection.In)]
    [DataRow(EasingKind.Cubic, EasingDirection.Out)]
    [DataRow(EasingKind.Quartic, EasingDirection.InOut)]
    [DataRow(EasingKind.Quintic, EasingDirection.In)]
    [DataRow(EasingKind.Sine, EasingDirection.In)]
    [DataRow(EasingKind.Cosine, EasingDirection.Out)]
    [DataRow(EasingKind.Plateau, EasingDirection.In)]
    [DataRow(EasingKind.Plateau, EasingDirection.InOut)]
    public async Task NonlinearTransitionsRasterizeOnTheirShapeNotTheChord(EasingKind kind, EasingDirection direction)
    {
        var profile = new ScalarTimeline(ProfileDomain.Distance,
            [FunctionSegment.Transition(0, 100, .1, .9, new EasingFunction(kind, direction))]);
        Region[] regions = [new(ProfileDomain.Distance, 0, 100, [KeyValuePair.Create("Blue", profile)])];
        await WithGraph(regions, (window, graph) =>
        {
            graph.SetSelection(0, "Blue", [new(0, .1), new(100, .9)], -1);
            CheckStroke(pixel => SystemMath.Abs(pixel.Red - 0x24) <= 40 && pixel.Green <= 40 && pixel.Blue >= 215, "profile");

            // The selected Transition's 1 px white highlight blends with the blue stroke under it.
            graph.SelectTransition(0, new TransitionShape(kind, direction).Description);
            Assert.AreEqual($"{kind} {direction}", graph.SelectedTransitionLabel);
            CheckStroke(pixel => pixel.Red >= 110 && pixel.Green >= 90 && pixel.Blue >= 200, "selection highlight");

            void CheckStroke(Func<SKColor, bool> color, string stroke)
            {
                window.UpdateLayout();
                Dispatcher.UIThread.RunJobs();
                using var frame = window.CaptureRenderedFrame();
                using var png = new MemoryStream();
                frame!.Save(png, PngBitmapEncoderOptions.Default);
                png.Position = 0;
                using SKBitmap pixels = SKBitmap.Decode(png);
                int separated = 0;
                foreach (double u in new[] { .15, .3, .4, .6, .7, .85 })
                {
                    double value = .1 + .8 * ReferenceEasing.Evaluate(kind, direction, u);
                    Assert.AreEqual(value, profile.Evaluate(100 * u).Value, 1e-12);
                    Point curve = WindowPoint(graph, window, graph.Viewport.Project(new(100 * u, value)));
                    Assert.IsTrue(HasColorNear(pixels, curve, color), $"The {kind} {direction} {stroke} is not drawn at {u}.");
                    Point chord = WindowPoint(graph, window, graph.Viewport.Project(new(100 * u, .1 + .8 * u)));
                    if (SystemMath.Abs(chord.Y - curve.Y) > 8)
                    {
                        separated++;
                        Assert.IsFalse(HasColorNear(pixels, chord, color), $"The {kind} {direction} {stroke} is drawn on the straight chord at {u}.");
                    }
                }
                Assert.IsGreaterThan(0, separated);
            }
        });
    }

    private static bool HasColorNear(SKBitmap pixels, Point location, Func<SKColor, bool> matches)
    {
        for (int row = (int)location.Y - 2; row <= (int)location.Y + 2; row++)
        {
            for (int column = (int)location.X - 2; column <= (int)location.X + 2; column++)
            {
                if (matches(pixels.GetPixel(column, row)))
                {
                    return true;
                }
            }
        }
        return false;
    }

    private static void CheckRenderedCurves(Window window, TransitionGraph graph, IReadOnlyList<Region> regions)
    {
        using var frame = window.CaptureRenderedFrame();
        Assert.IsNotNull(frame);
        using var png = new MemoryStream();
        frame.Save(png, PngBitmapEncoderOptions.Default);
        png.Position = 0;
        using SKBitmap pixels = SKBitmap.Decode(png);
        Assert.IsNotNull(pixels);

        (string Channel, string Color)[] channels =
            [("Blue", "#2400FF"), ("Red", "#FF0808"), ("Green", "#97FF00"), ("Purple", "#8000FF"), ("Gold", "#FFB800")];
        TransitionGraphViewport viewport = graph.Viewport;
        Assert.AreEqual(viewport.Plot.TopLeft, viewport.Project(new(0, 1)));
        Assert.AreEqual(viewport.Plot.BottomRight, viewport.Project(new(100, 0)));

        foreach ((string channel, string color) in channels)
        {
            using IEnumerator<Point[]> paths = graph.CurvePaths(channel).GetEnumerator();
            foreach (Region region in regions)
            {
                ScalarTimeline timeline = region.Channels[channel];
                double start = timeline.Minimum;
                foreach (double end in timeline.Breakpoints.Append(timeline.Maximum))
                {
                    Assert.IsTrue(paths.MoveNext());
                    Point[] points = paths.Current;
                    Assert.AreEqual(viewport.ProjectDistance(start), points[0].X, 1e-9);
                    Assert.AreEqual(viewport.ProjectDistance(end), points[^1].X, 1e-9);
                    for (int i = 0; i < points.Length; i++)
                    {
                        GraphPosition coordinate = viewport.Unproject(points[i]);
                        double evaluated = i == 0 ? timeline.Evaluate(start, EvaluationSide.Right).Value
                            : i == points.Length - 1 ? timeline.Evaluate(end, EvaluationSide.Left).Value
                            : timeline.Evaluate(coordinate.Distance).Value;
                        Assert.AreEqual(evaluated, coordinate.Value, 1e-10);

                        if (i > 0)
                        {
                            Point previous = points[i - 1];
                            Assert.IsTrue(points[i].X > previous.X && points[i].X - previous.X <= 1.000000001);
                            double midpoint = viewport.Unproject((previous + points[i]) / 2).Distance;
                            double expectedY = viewport.Project(new(midpoint, timeline.Evaluate(midpoint).Value)).Y;
                            Assert.AreEqual(expectedY, (previous.Y + points[i].Y) / 2, .01);
                        }
                    }
                    start = end;
                }
            }
            Assert.IsFalse(paths.MoveNext());

            // At 83 m the five fixture curves have sufficient separation to inspect the actual rasterized strokes.
            const double station = 83;
            double value = regions[^1].Channels[channel].Evaluate(station).Value;
            Point location = WindowPoint(graph, window, viewport.Project(new(station, value)));
            AssertPixelNear(pixels, location, SKColor.Parse(color), channel);
        }
    }

    private static void AssertPixelNear(SKBitmap pixels, Point location, SKColor expected, string channel)
    {
        int x = (int)SystemMath.Floor(location.X);
        int y = (int)SystemMath.Floor(location.Y);
        for (int row = y - 1; row <= y + 1; row++)
        {
            for (int column = x - 1; column <= x + 1; column++)
            {
                SKColor pixel = pixels.GetPixel(column, row);
                if (SystemMath.Abs(pixel.Red - expected.Red) <= 20 && SystemMath.Abs(pixel.Green - expected.Green) <= 20
                    && SystemMath.Abs(pixel.Blue - expected.Blue) <= 20)
                {
                    return;
                }
            }
        }
        Assert.Fail($"The {channel} stroke was not rendered near {location} in {pixels.Width} x {pixels.Height}.");
    }

    private static Point WindowPoint(Control graph, Window window, Point point) =>
        graph.TranslatePoint(point, window) ?? throw new InvalidOperationException("Graph is outside the window tree.");

    private static async Task WithGraph(IReadOnlyList<Region> regions, Action<Window, TransitionGraph> test,
        Func<GraphPosition, TransitionDragTarget?>? resolveDragTarget = null)
    {
        await using var session = HeadlessUnitTestSession.StartNew(typeof(HeadlessAppHost));
        await session.Dispatch(() =>
        {
            var graph = new TransitionGraph(regions, resolveDragTarget);
            var window = new Window { Width = 580, Height = 300, Content = graph };
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

internal static class HeadlessAppHost
{
    public static AppBuilder BuildAvaloniaApp() => AppBuilder.Configure<DesktopApp>()
        .UseSkia().UseHeadless(new AvaloniaHeadlessPlatformOptions
        {
            UseHeadlessDrawing = false,
            ShouldRenderOnUIThread = true
        });
}
