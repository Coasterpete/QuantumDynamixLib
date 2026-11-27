using System;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Threading;
using System.Threading.Tasks;
using Avalonia;
using Avalonia.Automation;
using Avalonia.Controls;
using Avalonia.Headless;
using Avalonia.Input;
using Avalonia.Interactivity;
using Avalonia.LogicalTree;
using Avalonia.Media.Imaging;
using Avalonia.Threading;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using QuantumDynamix.App;
using QuantumDynamix.Geometry;
using QuantumDynamix.Math;
using QuantumDynamix.Math.Functions;
using QuantumDynamix.Math.Timelines;
using QuantumDynamix.Splines.Curves;
using QuantumDynamix.Track;

namespace QuantumDynamix.Tests.Application;

[TestClass]
[DoNotParallelize]
public sealed partial class AppNavigationTests
{
    [TestMethod]
    public async Task MainWindowStartsWithBrandedWelcome()
    {
        await WithMainWindow(window =>
        {
            Assert.AreEqual("QUANTUM DYNAMIX", window.Title);
            Assert.IsNotNull(window.Icon);
            Assert.IsNotNull(window.FindControl<Image>("WelcomeArtwork")?.Source);
            Assert.IsNull(window.FindControl<Image>("WelcomeLogo"));
            Assert.IsTrue(window.FindControl<Button>("OpenEditorButton")?.IsEffectivelyVisible);
            foreach (string name in new[] { "LibraryButton", "SetupButton", "InfoButton", "NewsButton" })
            {
                Button? pending = window.FindControl<Button>(name);
                Assert.IsNotNull(pending);
                Assert.IsTrue(pending.IsEffectivelyVisible);
                Assert.IsFalse(pending.IsEnabled);
            }
        });
    }

    [TestMethod]
    public async Task EditorOpensWorkspaceWithBottomDockedGraphHost()
    {
        await WithMainWindow(window =>
        {
            Click(window, "OpenEditorButton");
            Dispatcher.UIThread.RunJobs();

            WorkspaceView? workspace = window.Content as WorkspaceView;
            Assert.IsNotNull(workspace);
            ContentControl? graphHost = workspace.FindControl<ContentControl>("TransitionGraphHost");
            Assert.IsNotNull(graphHost);
            Assert.IsInstanceOfType<TransitionGraph>(graphHost.Content);
            var graph = (TransitionGraph)graphHost.Content;
            Assert.IsGreaterThan(0, graphHost.Bounds.Height);
            Assert.AreEqual(graphHost.Bounds.Size, graph.Bounds.Size);
            Assert.IsTrue(graph.Viewport.HasPlot);

            Point? graphBottom = graphHost.TranslatePoint(new Point(0, graphHost.Bounds.Height), workspace);
            Assert.IsTrue(graphBottom.HasValue);
            Assert.AreEqual(28.5, workspace.Bounds.Height - graphBottom.Value.Y, 1);
            Assert.IsEmpty(graph.Regions);
            Assert.IsEmpty(graph.Channels);

            OpenMenu(workspace, "M3ExampleMenu");
            graph = (TransitionGraph)graphHost.Content!;
            Assert.HasCount(4, graph.Regions);
            Assert.HasCount(5, graph.Channels);
            Assert.AreEqual("M3 reference example", workspace.FindControl<TextBlock>("ModeLabel")!.Text);
            Assert.IsFalse(workspace.FindControl<ScrollViewer>("KeyframePanel")!.IsVisible);

            OpenMenu(workspace, "EmptyWorkspaceMenu");
            Assert.IsEmpty(((TransitionGraph)graphHost.Content!).Regions);
        });
    }

    [TestMethod]
    public async Task ExampleWorkflowEditsMultipleChannelsAndRegionsWithSharedHistory()
    {
        await WithMainWindow(window =>
        {
            Click(window, "OpenEditorButton");
            var workspace = (WorkspaceView)window.Content!;
            OpenMenu(workspace, "KeyframeExampleMenu");
            var graph = (TransitionGraph)workspace.FindControl<ContentControl>("TransitionGraphHost")!.Content!;
            var regions = workspace.FindControl<ListBox>("RegionList")!;
            var channels = workspace.FindControl<ListBox>("ActiveChannel")!;
            var visibility = workspace.FindControl<ItemsControl>("ChannelVisibility")!;
            var station = workspace.FindControl<TextBox>("StationInput")!;
            var value = workspace.FindControl<TextBox>("ValueInput")!;
            var status = workspace.FindControl<TextBlock>("StatusLabel")!;
            var greenToggle = visibility.ItemsSource!.Cast<CheckBox>().Single(item => (string?)item.Content == "Green");

            Assert.AreEqual(0, regions.SelectedIndex);
            Assert.AreEqual("Red", channels.SelectedItem);
            Click(window, greenToggle);
            Assert.IsFalse(greenToggle.IsChecked);
            Click(window, greenToggle);
            Assert.IsTrue(greenToggle.IsChecked);

            station.Text = "30";
            value.Text = "NaN";
            Click(window, "AddKeyButton");
            StringAssert.Contains(status.Text!, "finite number");
            Assert.IsFalse(workspace.FindControl<Button>("UndoButton")!.IsEnabled);
            value.Text = ".68".Replace(".", CultureInfo.CurrentCulture.NumberFormat.NumberDecimalSeparator);
            Click(window, "AddKeyButton");
            Assert.AreEqual(.68, graph.Regions[0].Channels["Red"].Evaluate(30).Value, 1e-12);
            Assert.AreEqual(.53, graph.Regions[0].Channels["Red"].Evaluate(24).Value, 1e-12);

            Point oldPoint = FromGraph(30, .68);
            Point newPoint = FromGraph(33, .74);
            window.MouseDown(oldPoint, MouseButton.Left);
            window.MouseMove(newPoint, RawInputModifiers.LeftMouseButton);
            Assert.AreEqual(.68, graph.Regions[0].Channels["Red"].Evaluate(30).Value, 1e-12);
            window.MouseUp(newPoint, MouseButton.Left);
            Assert.AreEqual(.74, graph.Regions[0].Channels["Red"].Evaluate(33).Value, 1e-12);
            Assert.AreEqual(33, double.Parse(station.Text!, CultureInfo.CurrentCulture), 1e-10);

            station.Text = "18";
            Click(window, "ApplyKeyButton");
            StringAssert.Contains(status.Text!, "between");
            Assert.AreEqual(.74, graph.Regions[0].Channels["Red"].Evaluate(33).Value, 1e-12);
            station.Text = "35";
            value.Text = ".8".Replace(".", CultureInfo.CurrentCulture.NumberFormat.NumberDecimalSeparator);
            Click(window, "ApplyKeyButton");
            Assert.AreEqual(.8, graph.Regions[0].Channels["Red"].Evaluate(35).Value, 1e-12);

            Click(window, (Control)regions.ContainerFromIndex(1)!);
            Click(window, (Control)channels.ContainerFromIndex(1)!);
            Assert.AreEqual(1, regions.SelectedIndex);
            Assert.AreEqual("Green", channels.SelectedItem);
            var keys = workspace.FindControl<ListBox>("KeyframeList")!;
            Click(window, (Control)keys.ContainerFromIndex(1)!);
            Click(window, "DeleteKeyButton");
            Assert.AreEqual(.5432, graph.Regions[1].Channels["Green"].Evaluate(72).Value, 1e-12);

            Click(window, "UndoButton");
            Assert.AreEqual(.58, graph.Regions[1].Channels["Green"].Evaluate(72).Value, 1e-12);
            Click(window, "UndoButton");
            Assert.AreEqual(.74, graph.Regions[0].Channels["Red"].Evaluate(33).Value, 1e-12);
            Click(window, "RedoButton");
            Assert.AreEqual(.8, graph.Regions[0].Channels["Red"].Evaluate(35).Value, 1e-12);
            Click(window, "RedoButton");
            Assert.AreEqual(.5432, graph.Regions[1].Channels["Green"].Evaluate(72).Value, 1e-12);

            Click(window, "ZoomInButton");
            Assert.AreEqual(10, graph.Viewport.Start, 1e-12);
            Assert.AreEqual(90, graph.Viewport.End, 1e-12);
            Click(window, "PanRightButton");
            Assert.AreEqual(20, graph.Viewport.Start, 1e-12);
            Assert.AreEqual(100, graph.Viewport.End, 1e-12);
            Assert.AreEqual(graph.Viewport.Plot.Left, graph.CurvePaths("Red").First()[0].X, 1e-12);
            Click(window, "FitButton");
            Assert.AreEqual(0, graph.Viewport.Start);
            Assert.AreEqual(100, graph.Viewport.End);

            OpenMenu(workspace, "EmptyWorkspaceMenu");
            OpenMenu(workspace, "KeyframeExampleMenu");
            var reopened = (TransitionGraph)workspace.FindControl<ContentControl>("TransitionGraphHost")!.Content!;
            Assert.AreEqual(.8, reopened.Regions[0].Channels["Red"].Evaluate(35).Value, 1e-12);

            Point FromGraph(double coordinate, double keyValue) => graph.TranslatePoint(
                graph.Viewport.Project(new(coordinate, keyValue)), window)!.Value;
        });
    }

    [TestMethod]
    public async Task NewTrackAuthorsRegionsAndProfilesWithOneHistory()
    {
        await WithMainWindow(window =>
        {
            Click(window, "OpenEditorButton");
            var workspace = (WorkspaceView)window.Content!;
            Click(window, "NewTrackButton");
            FillForm(window, "Create", ("Track name", "Ridge Line"));
            Assert.AreEqual("Track · Ridge Line", workspace.FindControl<TextBlock>("ModeLabel")!.Text);
            var graph = (TransitionGraph)workspace.FindControl<ContentControl>("TransitionGraphHost")!.Content!;
            var list = workspace.FindControl<ListBox>("RegionList")!;
            Assert.IsEmpty(graph.Regions);
            Assert.IsEmpty(graph.Channels);
            Capture(window, "01-new-track.png");

            const string banking = TrackDocument.BankingChannel;
            AddRegionInline(window, "50", "20");
            Assert.AreEqual(0, list.SelectedIndex);
            Assert.AreEqual(0, graph.SelectedRegion);
            Assert.AreEqual(0, graph.Regions[0].Channels[banking].Evaluate(45).Value, 1e-12);
            Assert.AreEqual(banking, workspace.FindControl<ListBox>("ActiveChannel")!.SelectedItem);
            Assert.AreEqual("20–70 m · 50 m long", workspace.FindControl<TextBlock>("RegionExtent")!.Text);

            Click(window, "AddChannelButton");
            FillForm(window, "Add", ("Channel name", "Join profile"), ("Start value", Local(.2)), ("End value", Local(.6)));
            Region main = graph.Regions[0];

            Assert.IsFalse(workspace.FindControl<TextBox>("FirstStationInput")!.IsVisible);
            // A blank length prepends the adjoining length, limited to the 20 m available before the first Region.
            AddRegionInline(window, null, prepend: true);
            Assert.AreEqual(0, graph.SelectedRegion);
            Assert.AreEqual(0, graph.Regions[0].Start);
            Assert.AreEqual(.2, graph.Regions[0].Channels["Join profile"].Evaluate(10).Value, 1e-12);
            Assert.AreEqual(0, graph.Regions[0].DiagnoseJoin(main, "Join profile", [0]).MatchingOrder);
            Assert.IsFalse(workspace.FindControl<Button>("PrependRegionButton")!.IsEnabled);

            AddRegionInline(window, "30");
            Assert.AreEqual(2, graph.SelectedRegion);
            Assert.AreSame(main, graph.Regions[1]);
            Region exit = graph.Regions[2];
            Assert.AreEqual(70, exit.Start);
            Assert.AreEqual(100, exit.End);
            Assert.AreEqual(.6, exit.Channels["Join profile"].Evaluate(85).Value, 1e-12);
            Assert.AreEqual(0, main.DiagnoseJoin(exit, "Join profile", [0]).MatchingOrder);
            Assert.HasCount(4, exit.Channels);
            Click(window, "UndoButton");
            Assert.HasCount(2, graph.Regions);
            Click(window, "RedoButton");
            Assert.AreSame(exit, graph.Regions[2]);

            Point header = graph.TranslatePoint(new Point(graph.Viewport.ProjectDistance(45), 8), window)!.Value;
            window.MouseDown(header, MouseButton.Left);
            window.MouseUp(header, MouseButton.Left);
            Assert.AreEqual(1, list.SelectedIndex);
            EditRegionInline(window, name: "Main profile");
            Assert.AreEqual("Main profile", graph.RegionName(1));
            Assert.AreEqual("Main profile", ((ListBoxItem)list.SelectedItem!).Content);

            Click(window, "AddChannelButton");
            FillForm(window, "Add", ("Channel name", "Profile B"), ("Start value", "1"), ("End value", "0"));
            Assert.HasCount(5, graph.Channels);
            Assert.AreEqual("Profile B", workspace.FindControl<ListBox>("ActiveChannel")!.SelectedItem);
            Assert.IsFalse(graph.Regions[0].Channels.ContainsKey("Profile B"));
            ClickEditorControl(window, "KeyframesButton");
            workspace.FindControl<TextBox>("StationInput")!.Text = "45";
            workspace.FindControl<TextBox>("ValueInput")!.Text = Local(.8);
            Click(window, "AddKeyButton");
            Assert.AreEqual(.8, graph.Regions[1].Channels["Profile B"].Evaluate(45).Value, 1e-12);
            Assert.AreEqual(.9, graph.Regions[1].Channels["Profile B"].Evaluate(32.5).Value, 1e-12);
            Assert.AreEqual(0, graph.Regions[1].Channels[banking].Evaluate(45).Value, 1e-12);
            Capture(window, "02-authored-profiles.png");

            Click(window, "DeleteRegionButton");
            Assert.HasCount(2, graph.Regions);
            Assert.AreEqual(20, graph.Regions[1].Start);
            Assert.IsFalse(graph.Channels.Contains("Profile B"));
            Assert.HasCount(2, graph.CurvePaths(banking).ToArray());
            Click(window, "UndoButton");
            Assert.AreEqual(.8, graph.Regions[1].Channels["Profile B"].Evaluate(45).Value, 1e-12);
            Click(window, "UndoButton");
            Assert.AreEqual(.5, graph.Regions[1].Channels["Profile B"].Evaluate(45).Value, 1e-12);
            Click(window, "UndoButton");
            Assert.IsFalse(graph.Channels.Contains("Profile B"));
            Click(window, "UndoButton");
            Assert.AreEqual("Region 1", graph.RegionName(1));
            Click(window, "RedoButton");
            Click(window, "RedoButton");
            Click(window, "RedoButton");
            Assert.AreEqual("Main profile", graph.RegionName(1));
            Assert.AreEqual(.8, graph.Regions[1].Channels["Profile B"].Evaluate(45).Value, 1e-12);

            Rect previous = graph.Viewport.Plot;
            window.Width = 1040;
            window.Height = 740;
            window.UpdateLayout();
            Dispatcher.UIThread.RunJobs();
            Assert.IsLessThan(previous.Width, graph.Viewport.Plot.Width);
            Click(window, "ZoomInButton");
            Click(window, "PanRightButton");
            Assert.AreEqual(20, graph.Viewport.Start, 1e-12);
            Assert.AreEqual(100, graph.Viewport.End, 1e-12);
            Assert.AreEqual(graph.Viewport.ProjectDistance(20), graph.CurvePaths("Profile B").First()[0].X, 1e-12);
            Click(window, "FitButton");
            Assert.AreEqual(0, graph.Viewport.Start);

            Click(window, (Control)list.ContainerFromIndex(1)!);
            var channels = workspace.FindControl<ListBox>("ActiveChannel")!;
            Click(window, (Control)channels.ContainerFromIndex(4)!);
            Point GraphPoint(double distance, double value) => graph.TranslatePoint(
                graph.Viewport.Project(new(distance, value)), window)!.Value;
            Point origin = GraphPoint(45, .8);
            Point target = GraphPoint(48, .72);
            window.MouseDown(origin, MouseButton.Left);
            window.MouseMove(target, RawInputModifiers.LeftMouseButton);
            Assert.AreEqual(.8, graph.Regions[1].Channels["Profile B"].Evaluate(45).Value, 1e-12);
            Assert.AreEqual(.72, graph.DisplayedRegions[1].Channels["Profile B"].Evaluate(48).Value, 1e-12);
            window.KeyPress(Key.Escape, RawInputModifiers.None, PhysicalKey.None, null);
            window.KeyRelease(Key.Escape, RawInputModifiers.None, PhysicalKey.None, null);
            window.MouseUp(target, MouseButton.Left);
            Assert.AreSame(graph.Regions, graph.DisplayedRegions);
            Assert.AreEqual(.8, graph.Regions[1].Channels["Profile B"].Evaluate(45).Value, 1e-12);

            window.MouseDown(origin, MouseButton.Left);
            window.MouseMove(target, RawInputModifiers.LeftMouseButton);
            window.MouseUp(target, MouseButton.Left);
            Assert.AreEqual(.72, graph.Regions[1].Channels["Profile B"].Evaluate(48).Value, 1e-12);
            Click(window, "UndoButton");
            Assert.AreEqual(.8, graph.Regions[1].Channels["Profile B"].Evaluate(45).Value, 1e-12);
        });
    }

    [TestMethod]
    public async Task RegionGeometryFollowsCommittedProfilesAndLengthInTheViewport()
    {
        await WithMainWindow(window =>
        {
            Click(window, "OpenEditorButton");
            var workspace = (WorkspaceView)window.Content!;
            var viewport = workspace.FindControl<TrackViewport>("ViewportSurface")!;
            var list = workspace.FindControl<ListBox>("RegionList")!;
            var channels = workspace.FindControl<ListBox>("ActiveChannel")!;
            Click(window, "NewTrackButton");
            FillForm(window, "Create", ("Track name", "Survey Track"), ("Prescribed forward speed (m/s)", "10"),
                ("Start X (m)", "4"), ("Start Y (m)", "5"), ("Start Z (m)", "6"), ("Start heading (degrees from +X toward +Y)", "90"));
            var graph = (TransitionGraph)workspace.FindControl<ContentControl>("TransitionGraphHost")!.Content!;
            Assert.IsFalse(viewport.HasGeometry);

            AddRegionInline(window, "20", "10");
            Assert.IsTrue(viewport.HasGeometry);
            Guid first = viewport.ActiveRegionId!.Value;
            AssertNear(new(4, 5, 6), Boundary(first, start: true).Position);

            AddRegionInline(window, "15");
            Guid second = viewport.ActiveRegionId!.Value;
            AssertNear(new(4, 40, 6), Boundary(second, start: false).Position);
            AssertJoin(first, second);
            OpenMenu(workspace, "TopViewMenu");
            Click(window, (Control)list.ContainerFromIndex(0)!);
            ProjectedLine[] lines = viewport.Projected.Lines.Where(item =>
                item.Selection.RegionId == second && item.Selection.Element == WireframeElement.Heartline).ToArray();
            ProjectedLine line = lines[lines.Length / 2];
            Point point = viewport.TranslatePoint(new((line.Start.X + line.End.X) / 2, (line.Start.Y + line.End.Y) / 2), window)!.Value;
            window.MouseDown(point, MouseButton.Left);
            window.MouseUp(point, MouseButton.Left);
            Assert.AreEqual(1, graph.SelectedRegion);
            Assert.AreEqual(1, list.SelectedIndex);
            Assert.AreEqual(second, viewport.ActiveRegionId);

            Vector3d straightEnd = Boundary(second, start: false).Position;
            AddKey(37.5, 0.5);
            AssertNear(straightEnd, Boundary(second, start: false).Position);
            double railHeight(string id) => viewport.Scene.Paths.Single(path => path.RegionId == second && path.Id == id)
                .Vertices.Single(vertex => vertex.HeartlineStation == 37.5).Position.Z;
            Assert.AreEqual(System.Math.Sin(0.5), railHeight("left") - railHeight("right"), 2e-6);

            Click(window, (Control)list.ContainerFromIndex(0)!);
            channels.SelectedItem = RegionTrackGeometry.LateralChannel;
            AddKey(20, 4);
            // The triangular acceleration input integrates to 40 m²/s²; dividing by v² gives 0.4 rad of yaw.
            var expectedForward = new Vector3d(-System.Math.Sin(.4), System.Math.Cos(.4), 0);
            WireframeBoundary joint = Boundary(first, start: false);
            AssertNear(expectedForward, joint.Orientation!.Value.Forward);
            AssertNear(joint.Position + expectedForward * 15, Boundary(second, start: false).Position);
            AssertJoin(first, second);

            TrackWireframe beforeResize = viewport.Scene;
            EditRegionInline(window, length: "30");
            Assert.AreEqual(40, graph.Regions[1].Start);
            Assert.AreEqual(55, graph.Regions[1].End);
            Assert.AreEqual(4, graph.Regions[0].Channels[RegionTrackGeometry.LateralChannel].Evaluate(25).Value);
            Assert.AreEqual(.5, graph.Regions[1].Channels[TrackDocument.BankingChannel].Evaluate(47.5).Value);
            AssertJoin(first, second);
            AssertNear(new(-System.Math.Sin(.6), System.Math.Cos(.6), 0), Boundary(first, start: false).Orientation!.Value.Forward);

            TrackWireframe afterResize = viewport.Scene;
            Click(window, "UndoButton");
            Assert.AreSame(beforeResize, viewport.Scene);
            Assert.AreEqual(30, graph.Regions[1].Start);
            Click(window, "RedoButton");
            Assert.AreSame(afterResize, viewport.Scene);
            Assert.AreEqual(40, graph.Regions[1].Start);

            WireframeBoundary Boundary(Guid id, bool start) => viewport.Scene.Boundaries.Single(b => b.RegionId == id && b.IsStart == start);

            void AssertJoin(Guid incoming, Guid outgoing)
            {
                WireframeBoundary left = Boundary(incoming, start: false);
                WireframeBoundary right = Boundary(outgoing, start: true);
                Assert.AreEqual(left.HeartlineStation, right.HeartlineStation);
                AssertNear(left.Position, right.Position);
                AssertNear(left.Orientation!.Value.Forward, right.Orientation!.Value.Forward);
                AssertNear(left.Orientation.Value.Left, right.Orientation.Value.Left);
            }

            void AddKey(double station, double value)
            {
                workspace.FindControl<TextBox>("StationInput")!.Text = Local(station);
                workspace.FindControl<TextBox>("ValueInput")!.Text = Local(value);
                Click(window, "AddKeyButton");
            }
        });
    }

    [TestMethod]
    [DataRow("Force-based", RegionTrackGeometry.NormalForceChannel, 1.7)]
    [DataRow("Geometric", RegionTrackGeometry.PitchChangeChannel, 5.0)]
    public async Task MathematicalTransitionsEditTheRealProfileAndHeartline(string mode, string channel, double target)
    {
        await WithMainWindow(window =>
        {
            Click(window, "OpenEditorButton");
            var workspace = (WorkspaceView)window.Content!;
            Click(window, "NewTrackButton");
            FillForm(window, "Create", ("Track name", "Transition Track"));
            AddRegionInline(window, "24", "4", mode);
            Assert.AreEqual(channel, workspace.FindControl<ListBox>("ActiveChannel")!.SelectedItem);
            var graph = (TransitionGraph)workspace.FindControl<ContentControl>("TransitionGraphHost")!.Content!;
            var viewport = workspace.FindControl<TrackViewport>("ViewportSurface")!;
            var span = workspace.FindControl<ComboBox>("TransitionSpan")!;
            var kind = workspace.FindControl<ComboBox>("TransitionKind")!;
            Vector3d straight = viewport.Scene.Boundaries[^1].Position;

            Click(window, "TransitionsButton");
            Click(window, "AppendTransitionButton");
            Assert.AreEqual(1, span.SelectedIndex);
            Click(window, "PrependTransitionButton");
            Assert.HasCount(3, graph.Regions[0].Channels[channel].Segments);
            kind.SelectedItem = "Quartic";
            workspace.FindControl<TextBox>("TransitionLength")!.Text = Local(5);
            workspace.FindControl<TextBox>("TransitionEnd")!.Text = Local(target);
            ClickTransition("ApplyTransitionButton");
            ScalarTimeline applied = graph.Regions[0].Channels[channel];
            Assert.AreEqual(21, applied.Segments[1].Maximum);
            Assert.AreEqual(target, applied.Evaluate(21).Value, 1e-12);
            double baseline = applied.Evaluate(16).Value;
            Assert.AreEqual(baseline + (target - baseline) / 16, applied.Evaluate(18.5).Value, 1e-12);
            Assert.IsGreaterThan(.05, (viewport.Scene.Boundaries[^1].Position - straight).Length);
            StringAssert.Contains(workspace.FindControl<TextBlock>("CalculatedForce")!.Text!, "Calculated at");
            if (mode == "Geometric")
            {
                StringAssert.Contains(workspace.FindControl<TextBlock>("CalculatedForce")!.Text!, "Curvature");
            }

            ClickTransition("UndoButton");
            AssertNear(straight, viewport.Scene.Boundaries[^1].Position);
            ClickTransition("RedoButton");
            Assert.AreSame(applied, graph.Regions[0].Channels[channel]);

            window.UpdateLayout();
            var viewportPoint = graph.Viewport.Project(new(24, applied.Evaluate(24).Value));
            Point graphPoint = graph.TranslatePoint(viewportPoint, window)!.Value;
            window.MouseDown(graphPoint, MouseButton.Left);
            window.MouseUp(graphPoint, MouseButton.Left);
            Assert.AreEqual(2, span.SelectedIndex);
            Assert.AreEqual("Linear", kind.SelectedItem);

            // "Adjoining" keeps the neighbor's Region type.
            AddRegionInline(window, "3", mode: "Adjoining", prepend: true);
            Assert.HasCount(2, graph.Regions);
            Assert.IsTrue(graph.Regions[0].Channels.ContainsKey(channel));

            void ClickTransition(string name)
            {
                var button = workspace.FindControl<Button>(name)!;
                button.BringIntoView();
                Dispatcher.UIThread.RunJobs();
                Click(window, button);
            }
        });
    }

    [TestMethod]
    public async Task ForceRegionWorkflowEditsTheTrackThroughTheExistingEditorAndViewport()
    {
        await WithMainWindow(window =>
        {
            Click(window, "OpenEditorButton");
            var workspace = (WorkspaceView)window.Content!;
            var viewport = workspace.FindControl<TrackViewport>("ViewportSurface")!;
            var list = workspace.FindControl<ListBox>("RegionList")!;
            var profiles = workspace.FindControl<ListBox>("ActiveChannel")!;
            var calculated = workspace.FindControl<TextBlock>("CalculatedForce")!;
            Click(window, "NewTrackButton");
            FillForm(window, "Create", ("Track name", "Force Track"));
            var graph = (TransitionGraph)workspace.FindControl<ContentControl>("TransitionGraphHost")!.Content!;

            Assert.AreEqual("Adjoining", workspace.FindControl<ComboBox>("NewRegionMode")!.SelectedItem);
            Assert.AreEqual("0", workspace.FindControl<TextBox>("FirstStationInput")!.Text);
            AddRegionInline(window, "20", mode: "Force-based");
            Guid first = viewport.ActiveRegionId!.Value;
            Assert.AreEqual(RegionTrackGeometry.NormalForceChannel, profiles.SelectedItem);
            Assert.HasCount(4, viewport.Scene.Paths);
            AssertNear(new(20, 0, 0), viewport.Scene.Boundaries[1].Position);
            Assert.IsTrue(calculated.IsVisible);
            StringAssert.Contains(calculated.Text!, $"Normal {1.0:F3} g");

            AddKey(RegionTrackGeometry.NormalForceChannel, 10, 1.6);
            Vector3d lifted = viewport.Scene.Boundaries[1].Position;
            Assert.IsGreaterThan(1, lifted.Z);
            StringAssert.Contains(calculated.Text!, $"Normal {1.6:F3} g");
            AddKey(RegionTrackGeometry.LateralForceChannel, 10, .35);
            Vector3d turned = viewport.Scene.Boundaries[1].Position;
            Assert.IsGreaterThan(.5, turned.Y);
            StringAssert.Contains(calculated.Text!, $"Lateral {.35:F3} g");
            AddKey(TrackDocument.BankingChannel, 10, .4);
            TrackWireframe banked = viewport.Scene;
            Assert.IsGreaterThan(.5, (turned - banked.Boundaries[1].Position).Length);
            double zDifference = RailAt("left").Z - RailAt("right").Z;
            Assert.IsGreaterThan(.3, zDifference);
            Assert.IsTrue(viewport.Projected.Lines.Any(line => line.Selection.Element == WireframeElement.Rail));
            Capture(window, "force-region-authoring");

            AddRegionInline(window, "9", mode: "Adjoining");
            Guid second = viewport.ActiveRegionId!.Value;
            TrackWireframe extended = viewport.Scene;
            Assert.AreNotEqual(first, second);
            AssertNear(extended.Boundaries[1].Position, extended.Boundaries[2].Position);
            AssertNear(extended.Boundaries[1].Orientation!.Value.Forward, extended.Boundaries[2].Orientation!.Value.Forward);
            Assert.AreEqual(0, graph.Regions[0].DiagnoseJoin(graph.Regions[1], RegionTrackGeometry.NormalForceChannel, [1e-12]).MatchingOrder);

            window.UpdateLayout();
            Point header = graph.TranslatePoint(new Point(graph.Viewport.ProjectDistance(5), 8), window)!.Value;
            window.MouseDown(header, MouseButton.Left);
            window.MouseUp(header, MouseButton.Left);
            AssertSelection(0, first);
            ProjectedMarker marker = viewport.Projected.Markers.Single(item => item.Selection.RegionId == second && item.Selection.Id == "end");
            Point markerPoint = viewport.TranslatePoint(new(marker.Point.X, marker.Point.Y), window)!.Value;
            window.MouseDown(markerPoint, MouseButton.Left);
            window.MouseUp(markerPoint, MouseButton.Left);
            AssertSelection(1, second);

            Click(window, "UndoButton");
            Assert.AreSame(banked, viewport.Scene);
            AssertSelection(0, first);
            Click(window, "RedoButton");
            Assert.AreSame(extended, viewport.Scene);
            Assert.HasCount(2, graph.Regions);

            Vector3d RailAt(string id) => viewport.Scene.Paths.Single(path => path.Id == id)
                .Vertices.Single(vertex => vertex.HeartlineStation == 10).Position;

            void AddKey(string channel, double station, double value)
            {
                profiles.SelectedItem = channel;
                workspace.FindControl<TextBox>("StationInput")!.Text = Local(station);
                workspace.FindControl<TextBox>("ValueInput")!.Text = Local(value);
                Click(window, "AddKeyButton");
                Assert.AreEqual(value, graph.Regions[0].Channels[channel].Evaluate(station).Value, 1e-10);
            }

            void AssertSelection(int index, Guid id)
            {
                Assert.AreEqual(index, list.SelectedIndex);
                Assert.AreEqual(index, graph.SelectedRegion);
                Assert.AreEqual(id, viewport.ActiveRegionId);
            }
        });
    }

    [TestMethod]
    public void GeometryPreservesPrependPoseAndRejectsUnrepresentablePlacement()
    {
        var pose = new RigidPose3d(new(5, 6, 7), Quaterniond.FromAxisAngle(Vector3d.UnitY, -.3));
        TrackDocument document = TrackDocument.CreateAuthored("Incline", pose, 10);
        TimelineEditorSession session = document.Timeline;
        Assert.IsNull(session.AddRegion(false, 20, 10));
        Guid original = session.Regions[0].Id;
        Assert.IsNull(session.Add(0, RegionTrackGeometry.VerticalChannel, new(20, 3), out _));
        TrackWireframe before = document.SampleWireframe(RegionTrackGeometry.DisplayIntervals);
        AssertNear(pose.Position, before.Boundaries[0].Position);
        AssertNear(pose.TransformDirection(Vector3d.UnitX), before.Boundaries[0].Orientation!.Value.Forward);
        Assert.IsGreaterThan(.5, before.Boundaries[1].Orientation!.Value.Forward.Z);

        Assert.IsNull(session.AddRegion(true, 7, 0));
        TrackWireframe after = document.SampleWireframe(RegionTrackGeometry.DisplayIntervals);
        AssertNear(pose.Position - pose.TransformDirection(Vector3d.UnitX) * 7, after.Boundaries[0].Position);
        AssertNear(before.Boundaries[0].Position, after.Boundaries[1].Position);
        AssertNear(before.Boundaries[1].Position, after.Boundaries.Single(b => b.RegionId == original && !b.IsStart).Position);
        Assert.IsNull(session.DeleteRegion(0));
        AssertNear(before.Boundaries[0].Position, document.SampleWireframe(RegionTrackGeometry.DisplayIntervals).Boundaries[0].Position);

        TrackDocument far = TrackDocument.CreateAuthored("Unrepresentable", new(new(1e20, 0, 0), Quaterniond.Identity), 10);
        StringAssert.Contains(far.Timeline.AddRegion(false, 10, 0)!, "world position is too large");
        Assert.IsEmpty(far.Timeline.Regions);
        Assert.IsFalse(far.Timeline.CanUndo);
        Assert.IsEmpty(far.SampleWireframe(RegionTrackGeometry.DisplayIntervals).Paths);
    }

    private static void AssertNear(Vector3d expected, Vector3d actual) =>
        Assert.IsLessThan(2e-6, (expected - actual).Length, $"Expected {expected}, found {actual}.");

    [TestMethod]
    public async Task AuthoredTimelineCanSupplyBankingForAnExplicitHeartline()
    {
        await WithMainWindow(window =>
        {
            const string channel = TrackDocument.BankingChannel;
            const double length = 10;
            const double originalAngle = .2;
            double movedAngle = System.Math.PI / 6;
            Click(window, "OpenEditorButton");
            var workspace = (WorkspaceView)window.Content!;
            Click(window, "NewTrackButton");
            FillForm(window, "Create", ("Track name", "Banking test"));
            var graph = (TransitionGraph)workspace.FindControl<ContentControl>("TransitionGraphHost")!.Content!;

            Assert.IsTrue(workspace.FindControl<TextBox>("FirstStationInput")!.IsVisible);
            Assert.AreEqual("0", workspace.FindControl<TextBox>("FirstStationInput")!.Text);
            AddRegionInline(window, "10");
            workspace.FindControl<TextBox>("StationInput")!.Text = "4";
            workspace.FindControl<TextBox>("ValueInput")!.Text = Local(originalAngle);
            Click(window, "AddKeyButton");
            ScalarTimeline originalBank = graph.Regions[0].Channels[channel];

            // Bind the authored banking to an explicit heartline.
            var heartline = SplineCurve3d.Clamped([Vector3d.Zero, new Vector3d(length, 0, 0)], 1);
            var offsets = TrackOffsetProfile.Constant(Vector3d.UnitZ, length);
            var originalTrack = new HeartlineTrack(heartline, Vector3d.UnitZ, offsets, originalBank);

            Point GraphPoint(double station, double value) =>
                graph.TranslatePoint(graph.Viewport.Project(new(station, value)), window)!.Value;
            Point start = GraphPoint(4, originalAngle);
            Point end = GraphPoint(6, movedAngle);
            window.MouseDown(start, MouseButton.Left);
            window.MouseMove(end, RawInputModifiers.LeftMouseButton);
            Assert.AreSame(originalBank, graph.Regions[0].Channels[channel]);
            ScalarTimeline previewBank = graph.DisplayedRegions[0].Channels[channel];
            Assert.AreEqual(6, previewBank.Breakpoints.Single(), 1e-12);
            Assert.AreEqual(movedAngle, previewBank.Evaluate(6).Value, 1e-12);
            window.MouseUp(end, MouseButton.Left);

            ScalarTimeline editedBank = graph.Regions[0].Channels[channel];
            Assert.AreSame(graph.Regions, graph.DisplayedRegions);
            Assert.AreEqual(movedAngle, editedBank.Evaluate(6).Value, 1e-12);
            var editedTrack = new HeartlineTrack(heartline, Vector3d.UnitZ, offsets, editedBank);
            HeartlineTrackSample sample = editedTrack.Evaluate(6, EvaluationSide.Right);
            TestMath.Near(new(6, 0, 0), sample.HeartlinePosition, 1e-12);
            TestMath.Near(new(6, -.5, System.Math.Sqrt(3) / 2), sample.SpinePosition, 1e-12);
            Assert.AreEqual(originalAngle, originalTrack.Banking.Evaluate(4).Value, 1e-12);

            Click(window, "UndoButton");
            Assert.AreSame(originalBank, graph.Regions[0].Channels[channel]);
            Click(window, "RedoButton");
            Assert.AreSame(editedBank, graph.Regions[0].Channels[channel]);
        });
    }

    [TestMethod]
    public async Task ReferenceViewportFollowsCommittedBankingAndPreservesTheUserTrack()
    {
        await WithMainWindowAsync(async window =>
        {
            Click(window, "OpenEditorButton");
            var workspace = (WorkspaceView)window.Content!;
            TrackViewport viewport = workspace.FindControl<TrackViewport>("ViewportSurface")!;
            Click(window, "NewTrackButton");
            FillForm(window, "Create", ("Track name", "My Track"));
            Assert.IsFalse(viewport.HasGeometry);
            StringAssert.Contains(workspace.FindControl<TextBlock>("ViewportInfo")!.Text!, "Append a Region");

            OpenMenu(workspace, "ReferenceTrackMenu");
            Assert.IsTrue(viewport.HasGeometry);
            Assert.AreEqual("Quarter-circle Reference Track", workspace.FindControl<TextBlock>("ModeLabel")!.Text);
            Assert.IsFalse(workspace.FindControl<Button>("AppendRegionButton")!.IsEnabled);
            var graph = (TransitionGraph)workspace.FindControl<ContentControl>("TransitionGraphHost")!.Content!;
            TrackWireframe before = viewport.Scene;
            OpenMenu(workspace, "TopViewMenu");
            Assert.AreEqual(TrackView.Top, viewport.View);
            Assert.AreSame(before, viewport.Scene);
            AssertRailDrawsAtProjectedPosition(window, viewport);
            OpenMenu(workspace, "PerspectiveViewMenu");

            double station = graph.Regions[0].End / 2;
            const double angle = .65;
            Point GraphPoint(double value) => graph.TranslatePoint(graph.Viewport.Project(new(station, value)), window)!.Value;
            ProjectedLine railBeforeDrag = viewport.Projected.Lines.First(line => line.Selection.Id == "inner");
            window.MouseDown(GraphPoint(0), MouseButton.Left);
            window.MouseMove(GraphPoint(angle), RawInputModifiers.LeftMouseButton);
            Assert.AreNotSame(graph.Regions, graph.DisplayedRegions);
            // The viewport previews the bound banking live. The committed scene is unchanged until release.
            await workspace.WhenLivePreviewIdle();
            Assert.AreSame(before, viewport.Scene);
            Assert.IsNotNull(viewport.PreviewScene);
            Assert.AreEqual(1.5 * System.Math.Sin(angle), RailHeight(viewport.PreviewScene), 1e-10);
            Assert.AreNotEqual(railBeforeDrag, viewport.Projected.Lines.First(line => line.Selection.Id == "inner"));
            window.MouseUp(GraphPoint(angle), MouseButton.Left);
            Assert.IsNull(viewport.PreviewScene);
            TrackWireframe after = viewport.Scene;
            var heartline = TrackWireframeTests.Vertices(before, "Heartline");
            CollectionAssert.AreEqual(heartline, TrackWireframeTests.Vertices(after, "Heartline"));
            Assert.AreEqual(1.5 * System.Math.Sin(angle), RailHeight(after), 1e-10);
            Click(window, "UndoButton");
            Assert.AreEqual(0, RailHeight(viewport.Scene), 1e-12);
            Click(window, "RedoButton");
            Assert.AreEqual(RailHeight(after), RailHeight(viewport.Scene), 1e-12);

            OpenMenu(workspace, "ActiveTrackMenu");
            Assert.AreEqual("Track · My Track", workspace.FindControl<TextBlock>("ModeLabel")!.Text);
            Assert.IsFalse(viewport.HasGeometry);
            OpenMenu(workspace, "ReferenceTrackMenu");
            Assert.AreEqual(RailHeight(after), RailHeight(viewport.Scene), 1e-12);

            double RailHeight(TrackWireframe scene) => TrackWireframeTests.Vertices(scene, "inner")
                .MinBy(vertex => System.Math.Abs(vertex.HeartlineStation!.Value - station)).Position.Z;
        });
    }

    [TestMethod]
    public async Task ViewportPointerNavigationAndSelectionLeaveTheTransitionEditorInPlace()
    {
        await WithMainWindow(window =>
        {
            Click(window, "OpenEditorButton");
            var workspace = (WorkspaceView)window.Content!;
            OpenMenu(workspace, "ReferenceTrackMenu");
            var viewport = workspace.FindControl<TrackViewport>("ViewportSurface")!;
            var graph = (TransitionGraph)workspace.FindControl<ContentControl>("TransitionGraphHost")!.Content!;
            TransitionGraphViewport graphView = graph.Viewport;
            TrackWireframe scene = viewport.Scene;

            Point center = viewport.TranslatePoint(new(viewport.Bounds.Width / 2, viewport.Bounds.Height / 2), window)!.Value;
            ProjectedLine first = viewport.Projected.Lines[0];
            window.MouseWheel(center, new Vector(0, 1), RawInputModifiers.None);
            Assert.AreNotEqual(first, viewport.Projected.Lines[0]);
            OpenMenu(workspace, "FrameTrackMenu");
            Assert.AreEqual(first, viewport.Projected.Lines[0]);

            Point end = center + new Vector(32, 16);
            window.MouseDown(center, MouseButton.Right);
            window.MouseMove(end, RawInputModifiers.RightMouseButton);
            window.MouseUp(end, MouseButton.Right);
            ProjectedLine orbited = viewport.Projected.Lines[0];
            Assert.AreNotEqual(first, orbited);
            window.MouseDown(center, MouseButton.Middle);
            window.MouseMove(end, RawInputModifiers.MiddleMouseButton);
            window.MouseUp(end, MouseButton.Middle);
            Assert.AreNotEqual(orbited, viewport.Projected.Lines[0]);
            Assert.AreEqual(graphView, graph.Viewport);
            Assert.AreSame(scene, viewport.Scene);

            OpenMenu(workspace, "TopViewMenu");
            ProjectedWireframe frame = viewport.Projected;
            ProjectedLine rail = frame.Lines.Where(line => line.Selection.Id == "inner").ElementAt(66);
            Point local = new((rail.Start.X + rail.End.X) / 2, (rail.Start.Y + rail.End.Y) / 2);
            Point click = viewport.TranslatePoint(local, window)!.Value;
            window.MouseDown(click, MouseButton.Left);
            window.MouseUp(click, MouseButton.Left);
            Assert.AreEqual("inner", viewport.Selection?.Id);
            Assert.AreSame(scene, viewport.Scene);
        });
    }

    [TestMethod]
    public async Task RegionSelectionIsSharedByTheListGraphAndViewport()
    {
        await WithMainWindow(window =>
        {
            Click(window, "OpenEditorButton");
            var workspace = (WorkspaceView)window.Content!;
            var timeline = new TimelineEditorSession();
            Assert.IsNull(timeline.AddRegion(prepend: false, length: 3, initialStation: 2));
            Assert.IsNull(timeline.AddRegion(prepend: false, length: 3, initialStation: 0));
            var heartline = SplineCurve3d.Clamped([Vector3d.Zero, new Vector3d(10, 0, 0)], 1);
            workspace.OpenTrack(new("Selection test", new TrackWireframeSource(new ArcLengthMap(heartline)), timeline));
            OpenMenu(workspace, "TopViewMenu");
            var list = workspace.FindControl<ListBox>("RegionList")!;
            var graph = (TransitionGraph)workspace.FindControl<ContentControl>("TransitionGraphHost")!.Content!;
            var viewport = workspace.FindControl<TrackViewport>("ViewportSurface")!;
            AssertSelected(0);

            Point header = graph.TranslatePoint(new Point(graph.Viewport.ProjectDistance(6.5), 8), window)!.Value;
            window.MouseDown(header, MouseButton.Left);
            window.MouseUp(header, MouseButton.Left);
            AssertSelected(1);

            Click(window, (Control)list.ContainerFromIndex(0)!);
            AssertSelected(0);

            ProjectedMarker marker = viewport.Projected.Markers.Single(marker =>
                marker.Selection.RegionId == timeline.Regions[1].Id && marker.Selection.Id == "end");
            Point point = viewport.TranslatePoint(new(marker.Point.X, marker.Point.Y), window)!.Value;
            window.MouseDown(point, MouseButton.Left);
            window.MouseUp(point, MouseButton.Left);
            AssertSelected(1);
            Assert.AreEqual(marker.Selection, viewport.Selection);

            void AssertSelected(int index)
            {
                Assert.AreEqual(index, list.SelectedIndex);
                Assert.AreEqual(index, graph.SelectedRegion);
                Assert.AreEqual(timeline.Regions[index].Id, viewport.ActiveRegionId);
                Assert.AreEqual(TrackDocument.BankingChannel, workspace.FindControl<ListBox>("ActiveChannel")!.SelectedItem);
            }
        });
    }

    private static void AssertRailDrawsAtProjectedPosition(Window window, TrackViewport viewport)
    {
        using var frame = window.CaptureRenderedFrame();
        Assert.IsNotNull(frame);
        using var png = new MemoryStream();
        frame.Save(png, PngBitmapEncoderOptions.Default);
        png.Position = 0;
        using var pixels = SkiaSharp.SKBitmap.Decode(png);
        ProjectedLine rail = viewport.Projected.Lines.Where(line => line.Selection.Id == "inner").ElementAt(66);
        var midpoint = new Point((rail.Start.X + rail.End.X) / 2, (rail.Start.Y + rail.End.Y) / 2);
        Point center = viewport.TranslatePoint(midpoint, window)!.Value;
        bool hasRailColor = (from x in Enumerable.Range((int)center.X - 2, 5)
            from y in Enumerable.Range((int)center.Y - 2, 5)
            let pixel = pixels.GetPixel(x, y)
            where pixel.Blue > 125 && pixel.Blue - pixel.Red > 55
            select pixel).Any();
        Assert.IsTrue(hasRailColor, $"Expected the inner rail at {center}.");
    }

    [TestMethod]
    public async Task InvalidTrackEditsPreserveDataAndHistoryAcrossWorkspaceSwitches()
    {
        await WithMainWindow(window =>
        {
            Click(window, "OpenEditorButton");
            var workspace = (WorkspaceView)window.Content!;
            Click(window, "NewTrackButton");
            FillForm(window, "Create", ("Track name", "West Track"));
            AddRegionInline(window, "20", "5");
            TransitionGraph Graph() => (TransitionGraph)workspace.FindControl<ContentControl>("TransitionGraphHost")!.Content!;
            var definition = Graph().Regions[0];
            var status = workspace.FindControl<TextBlock>("StatusLabel")!;

            AddRegionInline(window, "6", prepend: true);
            StringAssert.Contains(status.Text!, "at or after");
            StringAssert.Contains(status.Text!, "5 m is available");
            Assert.AreSame(definition, Graph().Regions.Single());

            OpenMenu(workspace, "AddChannelMenu");
            AuthoringDialog dialog = FillForm(window, "Add", ("Channel name", TrackDocument.BankingChannel), ("Start value", "0"), ("End value", "1"));
            AssertFormError(dialog, "already exists");
            ClickDialog(dialog, "Cancel");
            OpenMenu(workspace, "AddChannelMenu");
            dialog = FillForm(window, "Add", ("Channel name", "Overflow"), ("Start value", Local(-1e308)), ("End value", Local(1e308)));
            AssertFormError(dialog, "numeric limits");
            ClickDialog(dialog, "Cancel");
            workspace.FindControl<TextBox>("StationInput")!.Text = "25";
            workspace.FindControl<TextBox>("ValueInput")!.Text = "1";
            Click(window, "AddKeyButton");
            StringAssert.Contains(workspace.FindControl<TextBlock>("StatusLabel")!.Text!, "strictly inside");
            Assert.AreSame(definition, Graph().Regions.Single());

            OpenMenu(workspace, "M3ExampleMenu");
            Assert.HasCount(5, Graph().Channels);
            Assert.IsFalse(workspace.FindControl<Button>("DeleteRegionButton")!.IsEnabled);
            OpenMenu(workspace, "KeyframeExampleMenu");
            Assert.HasCount(3, Graph().Channels);
            OpenMenu(workspace, "EmptyWorkspaceMenu");
            Assert.IsEmpty(Graph().Regions);
            OpenMenu(workspace, "ActiveTrackMenu");
            Assert.AreSame(definition, Graph().Regions.Single());

            var graph = Graph();
            graph.Focus();
            bool mac = OperatingSystem.IsMacOS();
            RawInputModifiers modifier = mac ? RawInputModifiers.Meta : RawInputModifiers.Control;
            window.KeyPress(Key.Z, modifier, PhysicalKey.None, null);
            window.KeyRelease(Key.Z, modifier, PhysicalKey.None, null);
            Assert.IsEmpty(Graph().Regions);
            Key redo = mac ? Key.Z : Key.Y;
            modifier |= mac ? RawInputModifiers.Shift : RawInputModifiers.None;
            window.KeyPress(redo, modifier, PhysicalKey.None, null);
            window.KeyRelease(redo, modifier, PhysicalKey.None, null);
            Assert.AreSame(definition, Graph().Regions.Single());

            Click(window, "NewTrackButton");
            dialog = GetDialog(window);
            ClickDialog(dialog, "Cancel");
            Assert.AreSame(definition, Graph().Regions.Single());
            Click(window, "NewTrackButton");
            FillForm(window, "Create", ("Track name", "East Track"));
            ChooseUnsavedChanges(window, "Discard");
            Assert.IsEmpty(Graph().Regions);
            Assert.IsFalse(workspace.FindControl<Button>("UndoButton")!.IsEnabled);
            Assert.AreEqual("Track · East Track", workspace.FindControl<TextBlock>("ModeLabel")!.Text);

            AddRegionInline(window, null);
            StringAssert.Contains(status.Text!, "first Region's heartline length");
            AddRegionInline(window, "0");
            StringAssert.Contains(status.Text!, "positive heartline length");
            AddRegionInline(window, Local(1e308), Local(1e308));
            StringAssert.Contains(status.Text!, "cannot be represented");
            Assert.IsEmpty(Graph().Regions);
            Assert.IsFalse(workspace.FindControl<Button>("UndoButton")!.IsEnabled);
        });
    }

    private static string Local(double value) => value.ToString("R", CultureInfo.CurrentCulture);

    [TestMethod]
    public async Task ProjectMenusSaveAsReopenAndContinueAuthoringInANewWindow()
    {
        DirectoryInfo folder = Directory.CreateTempSubdirectory("qd-project-headless-");
        string original = Path.Combine(folder.FullName, "Original.qdproject");
        string copy = Path.Combine(folder.FullName, "Copy.qdproject");
        try
        {
            await WithMainWindowAsync(async window =>
            {
                Click(window, "OpenEditorButton");
                var workspace = (WorkspaceView)window.Content!;
                Assert.IsFalse(workspace.FindControl<MenuItem>("SaveProjectMenu")!.IsEnabled);
                OpenMenu(workspace, "NewProjectMenu");
                FillForm(window, "Create", ("Track name", "North Track"), ("Start X (m)", "12"),
                    ("Start pitch (degrees above horizontal)", "15"));
                Assert.IsTrue(workspace.HasUnsavedChanges);
                StringAssert.Contains(window.Title!, "*");
                AddRegionInline(window, "20");
                AddRegionInline(window, "15", mode: "Force-based");

                var channels = workspace.FindControl<ListBox>("ActiveChannel")!;
                channels.SelectedItem = RegionTrackGeometry.NormalForceChannel;
                Assert.AreEqual(RegionTrackGeometry.NormalForceChannel, channels.SelectedItem);
                workspace.FindControl<TextBox>("StationInput")!.Text = "28";
                workspace.FindControl<TextBox>("ValueInput")!.Text = Local(1.25);
                Click(window, "AddKeyButton");
                Click(window, "TransitionsButton");
                workspace.FindControl<ComboBox>("TransitionKind")!.SelectedItem = "Quintic";
                ClickEditorControl(window, "ApplyTransitionButton");
                ClickEditorControl(window, "UndoButton");
                Assert.AreEqual("Linear", workspace.FindControl<ComboBox>("TransitionKind")!.SelectedItem);
                ClickEditorControl(window, "RedoButton");
                Assert.AreEqual("Quintic", workspace.FindControl<ComboBox>("TransitionKind")!.SelectedItem);
                ClickEditorControl(window, "KeyframesButton");

                workspace.ProjectSavePicker = options =>
                {
                    Assert.AreEqual("qdproject", options.DefaultExtension);
                    Assert.IsTrue(options.ShowOverwritePrompt);
                    Assert.AreEqual("North Track", options.SuggestedFileName);
                    return Task.FromResult<string?>(original);
                };
                RawInputModifiers modifier = OperatingSystem.IsMacOS() ? RawInputModifiers.Meta : RawInputModifiers.Control;
                window.KeyPress(Key.S, modifier, PhysicalKey.None, null);
                window.KeyRelease(Key.S, modifier, PhysicalKey.None, null);
                await WaitForProjectOperation(workspace);
                Assert.IsTrue(File.Exists(original));
                Assert.IsFalse(workspace.HasUnsavedChanges);
                Assert.IsFalse(window.Title!.Contains('*'));

                Click(window, "UndoButton");
                Assert.IsTrue(workspace.HasUnsavedChanges);
                Click(window, "RedoButton");
                Assert.IsFalse(workspace.HasUnsavedChanges);

                workspace.ProjectSavePicker = _ => Task.FromResult<string?>(copy);
                OpenMenu(workspace, "SaveProjectAsMenu");
                await WaitForProjectOperation(workspace);
                CollectionAssert.AreEqual(File.ReadAllBytes(original), File.ReadAllBytes(copy));
                window.Close();
                Assert.IsFalse(window.IsVisible);
            });

            await WithMainWindowAsync(async window =>
            {
                Click(window, "OpenEditorButton");
                var workspace = (WorkspaceView)window.Content!;
                workspace.ProjectOpenPicker = options =>
                {
                    Assert.AreEqual("*.qdproject", options.FileTypeFilter!.Single().Patterns!.Single());
                    return Task.FromResult<string?>(copy);
                };
                OpenMenu(workspace, "OpenProjectMenu");
                await WaitForProjectOperation(workspace);
                var regions = workspace.FindControl<ListBox>("RegionList")!;
                var viewport = workspace.FindControl<TrackViewport>("ViewportSurface")!;
                var channels = workspace.FindControl<ListBox>("ActiveChannel")!;
                Assert.AreEqual(0, regions.SelectedIndex);
                Assert.AreEqual(TrackDocument.BankingChannel, channels.SelectedItem);
                Assert.HasCount(8, viewport.Scene.Paths);
                Assert.AreEqual(viewport.Scene.Boundaries[0].RegionId, viewport.ActiveRegionId);
                Assert.IsFalse(workspace.FindControl<Button>("UndoButton")!.IsEnabled);
                Assert.IsFalse(workspace.FindControl<Button>("RedoButton")!.IsEnabled);
                Assert.IsFalse(workspace.HasUnsavedChanges);

                regions.SelectedIndex = 1;
                Assert.AreEqual(viewport.Scene.Boundaries[2].RegionId, viewport.ActiveRegionId);
                channels.SelectedItem = RegionTrackGeometry.NormalForceChannel;
                Assert.AreEqual(RegionTrackGeometry.NormalForceChannel, channels.SelectedItem);
                workspace.FindControl<ListBox>("KeyframeList")!.SelectedIndex = 1;
                StringAssert.Contains(workspace.FindControl<TextBlock>("CalculatedForce")!.Text!, "Normal 1");
                workspace.FindControl<TextBox>("ValueInput")!.Text = Local(1.4);
                Click(window, "ApplyKeyButton");
                Assert.IsTrue(workspace.HasUnsavedChanges);
                OpenMenu(workspace, "SaveProjectMenu");
                await WaitForProjectOperation(workspace);
                Assert.IsFalse(workspace.HasUnsavedChanges);
                Assert.AreEqual(1.4, ProjectFile.Load(copy).Timeline.Keys(1, RegionTrackGeometry.NormalForceChannel)[1].Value);
                var originalTrack = ProjectFile.Load(original).Timeline;
                Assert.AreEqual(1.25, originalTrack.Keys(1, RegionTrackGeometry.NormalForceChannel)[1].Value);
                Assert.AreEqual(EasingKind.Quintic, originalTrack.Regions[1].Shapes(RegionTrackGeometry.NormalForceChannel)[0].Kind);
                Assert.AreEqual(EasingKind.Quintic,
                    ProjectFile.Load(copy).Timeline.Regions[1].Shapes(RegionTrackGeometry.NormalForceChannel)[0].Kind);
            });
        }
        finally
        {
            folder.Delete(recursive: true);
        }
    }

    [TestMethod]
    public async Task DirtyProjectSurvivesCancelledAndFailedOpenSaveAndClose()
    {
        DirectoryInfo folder = Directory.CreateTempSubdirectory("qd-project-headless-");
        string invalid = Path.Combine(folder.FullName, "Invalid.qdproject");
        string good = Path.Combine(folder.FullName, "Saved.qdproject");
        File.WriteAllText(invalid, "{");
        try
        {
            await WithMainWindowAsync(async window =>
            {
                Click(window, "OpenEditorButton");
                var workspace = (WorkspaceView)window.Content!;
                Click(window, "NewTrackButton");
                FillForm(window, "Create", ("Track name", "Keep this track"));
                AddRegionInline(window, "20");
                var viewport = workspace.FindControl<TrackViewport>("ViewportSurface")!;
                TrackWireframe scene = viewport.Scene;
                var status = workspace.FindControl<TextBlock>("StatusLabel")!;

                workspace.ProjectOpenPicker = _ => Task.FromResult<string?>(null);
                OpenMenu(workspace, "OpenProjectMenu");
                Assert.IsEmpty(window.OwnedWindows);

                workspace.ProjectOpenPicker = _ => Task.FromResult<string?>(invalid);
                OpenMenu(workspace, "OpenProjectMenu");
                ChooseUnsavedChanges(window, "Cancel");
                Assert.AreSame(scene, viewport.Scene);

                OpenMenu(workspace, "OpenProjectMenu");
                ChooseUnsavedChanges(window, "Discard");
                await WaitForProjectOperation(workspace);
                StringAssert.Contains(status.Text!, "Invalid or incomplete project JSON");
                Assert.AreSame(scene, viewport.Scene);
                Assert.IsTrue(workspace.HasUnsavedChanges);
                Assert.IsTrue(workspace.FindControl<Button>("UndoButton")!.IsEnabled);

                window.Close();
                ChooseUnsavedChanges(window, "Cancel");
                Assert.IsTrue(window.IsVisible);

                workspace.ProjectSavePicker = _ => Task.FromResult<string?>(null);
                window.Close();
                ChooseUnsavedChanges(window, "Save");
                Assert.IsTrue(window.IsVisible);
                Assert.IsTrue(workspace.HasUnsavedChanges);

                workspace.ProjectSavePicker = _ => Task.FromResult<string?>(folder.FullName);
                window.Close();
                ChooseUnsavedChanges(window, "Save");
                await WaitForProjectOperation(workspace);
                StringAssert.Contains(status.Text!, "Could not save project");
                Assert.IsTrue(window.IsVisible);
                Assert.IsTrue(workspace.HasUnsavedChanges);
                Assert.AreSame(scene, viewport.Scene);

                workspace.ProjectSavePicker = _ => Task.FromResult<string?>(good);
                OpenMenu(workspace, "EmptyWorkspaceMenu");
                Assert.IsTrue(workspace.HasUnsavedChanges);
                window.Close();
                ChooseUnsavedChanges(window, "Save");
                await WaitForProjectOperation(workspace);
                Assert.IsFalse(window.IsVisible);
                Assert.AreEqual("Keep this track", ProjectFile.Load(good).Identity.Name);
            });
        }
        finally
        {
            folder.Delete(recursive: true);
        }
    }

    private static void ChooseUnsavedChanges(Window window, string choice)
    {
        var dialog = window.OwnedWindows.OfType<SaveChangesDialog>().Single();
        Click(dialog, dialog.GetLogicalDescendants().OfType<Button>().Single(button => Equals(button.Content, choice)));
    }

    private static async Task WaitForProjectOperation(WorkspaceView workspace)
    {
        using var timeout = new CancellationTokenSource(TimeSpan.FromSeconds(10));
        while (workspace.ProjectOperationInProgress)
        {
            await Task.Delay(10, timeout.Token);
        }
        Dispatcher.UIThread.RunJobs();
    }

    private static AuthoringDialog GetDialog(Window window) =>
        window.OwnedWindows.OfType<AuthoringDialog>().Single();

    /// <summary>Uses the inline Region List controls. A null length uses the adjoining Region's length.</summary>
    private static void AddRegionInline(Window window, string? length, string? firstStation = null, string? mode = null,
        bool prepend = false)
    {
        var workspace = (WorkspaceView)window.Content!;
        if (mode is not null)
        {
            workspace.FindControl<ComboBox>("NewRegionMode")!.SelectedItem = mode;
        }
        workspace.FindControl<TextBox>("NewRegionLength")!.Text = length ?? "";
        if (firstStation is not null)
        {
            workspace.FindControl<TextBox>("FirstStationInput")!.Text = firstStation;
        }
        int dialogs = window.OwnedWindows.Count;
        ClickEditorControl(window, prepend ? "PrependRegionButton" : "AppendRegionButton");
        Assert.HasCount(dialogs, window.OwnedWindows, "Append and Prepend Region must not open a dialog.");
    }

    /// <summary>Edits the selected Region through the inline Region inspector.</summary>
    private static void EditRegionInline(Window window, string? name = null, string? length = null)
    {
        var workspace = (WorkspaceView)window.Content!;
        OpenMenu(workspace, "RegionPropertiesMenu");
        Assert.IsTrue(workspace.FindControl<StackPanel>("RegionFields")!.IsVisible);
        if (name is not null) { workspace.FindControl<TextBox>("RegionNameInput")!.Text = name; }
        if (length is not null) { workspace.FindControl<TextBox>("RegionLengthInput")!.Text = length; }
        ClickEditorControl(window, "ApplyRegionButton");
    }

    private static AuthoringDialog FillForm(Window window, string action, params (string Label, string Value)[] fields)
    {
        AuthoringDialog dialog = GetDialog(window);
        foreach ((string label, string value) in fields)
        {
            dialog.GetLogicalDescendants().OfType<TextBox>().Single(input => AutomationProperties.GetName(input) == label).Text = value;
        }
        ClickDialog(dialog, action);
        return dialog;
    }

    private static void ClickDialog(AuthoringDialog dialog, string action) => Click(dialog,
        dialog.GetLogicalDescendants().OfType<Button>().Single(button => Equals(button.Content, action)));

    private static void AssertFormError(AuthoringDialog dialog, string expected)
    {
        Assert.IsTrue(dialog.IsVisible);
        Assert.IsTrue(dialog.GetLogicalDescendants().OfType<TextBlock>()
            .Any(block => block.IsVisible && block.Text?.Contains(expected, StringComparison.Ordinal) == true));
    }

    private static void Capture(Window window, string name)
    {
        string? directory = Environment.GetEnvironmentVariable("QUANTUM_DYNAMIX_SCREENSHOTS");
        if (string.IsNullOrWhiteSpace(directory))
        {
            return;
        }
        window.UpdateLayout();
        Dispatcher.UIThread.RunJobs();
        using var frame = window.CaptureRenderedFrame();
        Assert.IsNotNull(frame);
        Directory.CreateDirectory(directory);
        using var png = File.Create(Path.Combine(directory, name));
        frame.Save(png, PngBitmapEncoderOptions.Default);
    }

    [TestMethod]
    public async Task ExitClosesWelcomeWindow()
    {
        await WithMainWindow(window =>
        {
            Click(window, "ExitButton");

            Assert.IsFalse(window.IsVisible);
        });
    }

    private static Task WithMainWindow(Action<Window> test) => WithMainWindowAsync(window =>
    {
        test(window);
        return Task.CompletedTask;
    });

    private static async Task WithMainWindowAsync(Func<Window, Task> test)
    {
        await using var session = HeadlessUnitTestSession.StartNew(typeof(HeadlessAppHost));
        await session.Dispatch<bool>(async () =>
        {
            var window = new MainWindow();
            window.Show();
            try
            {
                await test(window);
                return true;
            }
            finally
            {
                if (window.IsVisible)
                {
                    foreach (Window dialog in window.OwnedWindows.ToArray()) { dialog.Close(); }
                    Dispatcher.UIThread.RunJobs();
                    window.Close();
                    if (window.OwnedWindows.OfType<SaveChangesDialog>().Any())
                    {
                        ChooseUnsavedChanges(window, "Discard");
                    }
                }
            }
        }, CancellationToken.None);
    }

    private static void Click(Window window, string buttonName)
    {
        Button? button = window.Content is WorkspaceView workspace
            ? workspace.FindControl<Button>(buttonName)
            : window.FindControl<Button>(buttonName);
        Assert.IsNotNull(button);
        Click(window, button);
    }

    private static void ClickEditorControl(Window window, string buttonName)
    {
        var button = ((WorkspaceView)window.Content!).FindControl<Button>(buttonName)!;
        button.BringIntoView();
        Dispatcher.UIThread.RunJobs();
        Click(window, button);
    }

    private static void Click(Window window, Control control)
    {
        window.UpdateLayout();
        Point localCenter = new(control.Bounds.Width / 2, control.Bounds.Height / 2);
        Point? center = control.TranslatePoint(localCenter, window);
        Assert.IsTrue(center.HasValue);
        window.MouseDown(center.Value, MouseButton.Left);
        window.MouseUp(center.Value, MouseButton.Left);
        Dispatcher.UIThread.RunJobs();
    }

    private static void OpenMenu(WorkspaceView workspace, string itemName)
    {
        var item = workspace.FindControl<MenuItem>(itemName)!;
        item.RaiseEvent(new RoutedEventArgs(MenuItem.ClickEvent));
        workspace.UpdateLayout();
        Dispatcher.UIThread.RunJobs();
    }
}
