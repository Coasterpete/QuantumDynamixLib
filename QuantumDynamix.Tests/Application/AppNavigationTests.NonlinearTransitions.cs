using System;
using System.Linq;
using System.Threading.Tasks;
using Avalonia;
using Avalonia.Controls;
using Avalonia.Headless;
using Avalonia.Input;
using Avalonia.LogicalTree;
using Avalonia.Threading;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using QuantumDynamix.App;
using QuantumDynamix.Geometry;
using QuantumDynamix.Math;
using QuantumDynamix.Math.Functions;
using QuantumDynamix.Math.Timelines;
using static QuantumDynamix.App.RegionTrackGeometry;

namespace QuantumDynamix.Tests.Application;

/// <summary>Function and direction choices in the Transition Editor and the Acceleration Region Editor.</summary>
public sealed partial class AppNavigationTests
{
    private static readonly EasingKind[] NonlinearKinds =
        [EasingKind.Quadratic, EasingKind.Cubic, EasingKind.Quartic, EasingKind.Quintic, EasingKind.Sine, EasingKind.Cosine, EasingKind.Plateau];

    [TestMethod]
    public async Task FunctionAndDirectionChoicesApplyAtOnceAndDrawTheDocumentedShapes()
    {
        await WithMainWindowAsync(async window =>
        {
            var (workspace, document, graph, viewport) = OpenLiveTrack(window, LiveEditingSessionTests.TwoRegionTrack());
            TimelineEditorSession session = document.Timeline;
            const double speed = 10;
            var kind = workspace.FindControl<ComboBox>("TransitionKind")!;
            var direction = workspace.FindControl<ComboBox>("TransitionDirection")!;
            var spans = workspace.FindControl<ComboBox>("TransitionSpan")!;
            var note = workspace.FindControl<TextBlock>("TransitionShapeNote")!;
            var status = workspace.FindControl<TextBlock>("StatusLabel")!;
            workspace.FindControl<ListBox>("ActiveChannel")!.SelectedItem = VerticalChannel;
            ClickEditorControl(window, "TransitionsButton");
            Assert.AreEqual(0, spans.SelectedIndex);

            // Equal endpoint values make y₀ + (y₁ − y₀)E(u) flat for every function. The editor says so.
            kind.SelectedItem = nameof(EasingKind.Sine);
            Assert.AreEqual(EasingKind.Sine, Shape().Kind, "A function choice applies without Apply.");
            Assert.IsTrue(note.IsVisible);
            Assert.AreEqual(0, Profile().Evaluate(5).Value);
            ClickEditorControl(window, "UndoButton");
            Assert.AreEqual(EasingKind.Linear, Shape().Kind);
            Assert.AreEqual(nameof(EasingKind.Linear), kind.SelectedItem, "Undo shows the restored function.");

            var end = workspace.FindControl<TextBox>("TransitionEnd")!;
            end.Text = Local(.8);
            Dispatcher.UIThread.RunJobs();
            Assert.IsTrue(workspace.TransitionValuesPending, "Typed values are marked until applied.");
            StringAssert.Contains(status.Text!, "not applied");
            end.Focus();
            window.KeyPress(Key.Enter, RawInputModifiers.None, PhysicalKey.Enter, null);
            window.KeyRelease(Key.Enter, RawInputModifiers.None, PhysicalKey.Enter, null);
            Assert.AreEqual(.8, session.Keys(0, VerticalChannel)[1].Value, "Enter applies typed values.");
            Assert.IsFalse(note.IsVisible);
            Assert.IsFalse(workspace.TransitionValuesPending);
            Vector3d linearEnd = viewport.Scene.Boundaries[^1].Position;

            foreach (EasingKind function in NonlinearKinds)
            {
                foreach (EasingDirection way in Enum.GetValues<EasingDirection>())
                {
                    TrackWireframe before = viewport.Scene;
                    kind.SelectedItem = function.ToString();
                    direction.SelectedItem = way;
                    string label = $"{function} {way}";
                    Assert.AreEqual(new TransitionShape(function, way), Shape(), label);
                    Assert.AreEqual(function.ToString(), kind.SelectedItem, label);
                    Assert.AreEqual(way, direction.SelectedItem, label);
                    Assert.AreEqual(label, graph.SelectedTransitionLabel);
                    StringAssert.EndsWith((string)spans.SelectedItem!, $"· {label}");
                    Assert.AreNotSame(before, viewport.Scene, $"{label} rebuilds the 3D track.");
                    Assert.IsGreaterThan(1e-3, (viewport.Scene.Boundaries[^1].Position - linearEnd).Length, label);

                    ScalarTimeline profile = Profile();
                    double largestBend = 0;
                    for (int step = 1; step < 20; step++)
                    {
                        double station = step / 2.0;
                        double expected = .8 * ReferenceEasing.Evaluate(function, way, station / 10);
                        Assert.AreEqual(expected, profile.Evaluate(station).Value, 1e-12, $"{label} at {station} m");
                        // The reconstruction's curvature follows the authored vertical acceleration: κ = a / v².
                        Vector3d curvature = session.Geometry!.CalculateGeometry(station, EvaluationSide.Right).Curvature;
                        Assert.AreEqual(expected / (speed * speed), curvature.Length, 1e-9, $"{label} curvature at {station} m");
                        largestBend = System.Math.Max(largestBend, System.Math.Abs(
                            graph.Viewport.Project(new(station, expected)).Y - graph.Viewport.Project(new(station, .08 * station)).Y));
                    }
                    Assert.IsGreaterThan(3, largestBend, $"{label} is visibly different from a straight line on the graph.");
                    AssertRenderedPathMatches(graph, VerticalChannel, profile, 0, 10);
                }
            }
            Capture(window, "nonlinear-transition.png");

            // A rejected choice keeps the committed Transition, the selectors, and the history.
            var regions = session.Regions;
            bool couldRedo = session.CanRedo;
            kind.SelectedItem = "Constant";
            StringAssert.Contains(status.Text!, "Constant transition requires equal endpoint values");
            Assert.AreSame(regions, session.Regions);
            Assert.AreEqual(nameof(EasingKind.Plateau), kind.SelectedItem);
            Assert.AreEqual(couldRedo, session.CanRedo);

            window.UpdateLayout();
            ClickEditorControl(window, "UndoButton");
            Assert.AreEqual("Undid the last edit.", status.Text);
            Assert.AreEqual(new TransitionShape(EasingKind.Plateau, EasingDirection.Out), Shape());
            Assert.AreEqual(EasingDirection.Out, direction.SelectedItem);
            ClickEditorControl(window, "RedoButton");
            Assert.AreEqual(new TransitionShape(EasingKind.Plateau, EasingDirection.InOut), Shape());
            await Task.CompletedTask;

            TransitionShape Shape() => session.Regions[0].Shapes(VerticalChannel)[0];
            ScalarTimeline Profile() => graph.Regions[0].Channels[VerticalChannel];
        });
    }

    [TestMethod]
    public async Task LiveKeyframeDragPreviewKeepsTheNonlinearFunction()
    {
        await WithMainWindowAsync(async window =>
        {
            TrackDocument document = LiveEditingSessionTests.TwoRegionTrack();
            TimelineEditorSession session = document.Timeline;
            Assert.IsNull(session.UpdateTransition(0, VerticalChannel, 0, 10, 0, .8, new(EasingKind.Sine)));
            session.MarkSaved();
            var (workspace, _, graph, viewport) = OpenLiveTrack(window, document);
            workspace.FindControl<ListBox>("ActiveChannel")!.SelectedItem = VerticalChannel;
            TrackWireframe committed = viewport.Scene;

            window.MouseDown(GraphPoint(window, graph, 10, .8), MouseButton.Left);
            window.MouseMove(GraphPoint(window, graph, 14, .6), RawInputModifiers.LeftMouseButton);
            ScalarTimeline preview = graph.DisplayedRegions[0].Channels[VerticalChannel];
            double key = preview.Segments[0].Maximum;
            Assert.AreEqual(14, key, 1e-12);
            for (int step = 1; step < 10; step++)
            {
                double station = key * step / 10;
                double expected = preview.Evaluate(key).Value * ReferenceEasing.Evaluate(EasingKind.Sine, EasingDirection.In, step / 10.0);
                Assert.AreEqual(expected, preview.Evaluate(station).Value, 1e-12, "The previewed span keeps its Sine shape.");
            }
            await workspace.WhenLivePreviewIdle();
            Assert.IsNotNull(viewport.PreviewScene);
            Assert.AreSame(committed, viewport.Scene);

            window.MouseUp(GraphPoint(window, graph, 14, .6), MouseButton.Left);
            Assert.AreEqual(new TransitionShape(EasingKind.Sine), session.Regions[0].Shapes(VerticalChannel)[0]);
            Assert.AreEqual(EasingKind.Linear, session.Regions[0].Shapes(VerticalChannel)[1].Kind);
            AssertRenderedPathMatches(graph, VerticalChannel, graph.Regions[0].Channels[VerticalChannel], 0, key);
        });
    }

    [TestMethod]
    public async Task AccelerationEditorAppliesEasingsAtOnceWithDraftUndoAndMotionResults()
    {
        await WithMainWindowAsync(async window =>
        {
            TrackDocument document = OperationalSegmentTests.AuthoredTrack();
            var launch = DeviceAccelerationRegion.Create([new(0, 0), new(10, 6)], [TransitionShape.Linear]);
            TimelineEditorSession session = document.Timeline;
            Assert.IsNull(session.SetSegment(new(Guid.NewGuid(), 15, 45, OperationalSegmentType.Transport,
                new PoweredTransportDevice(4, null, [launch]))));
            session.MarkSaved();
            OperationalSegment stored = session.Segments.Single();
            var (workspace, _, _, _) = OpenLiveTrack(window, document);
            ClickEditorControl(window, "OperationsButton");
            workspace.FindControl<ListBox>("SegmentList")!.SelectedIndex = 0;
            Click(window, "SegmentSettingsButton");
            TrackSegmentSettings dialog = SegmentDialog(window);
            dialog.UpdateLayout();
            var graph = (TransitionGraph)dialog.FindControl<ContentControl>("AccelerationGraphHost")!.Content!;
            var kind = dialog.FindControl<ComboBox>("SpanKind")!;
            var direction = dialog.FindControl<ComboBox>("SpanDirection")!;
            var status = dialog.FindControl<TextBlock>("SettingsStatus")!;
            string channel = DeviceAccelerationRegion.Channel;

            foreach (EasingKind function in NonlinearKinds)
            {
                foreach (EasingDirection way in Enum.GetValues<EasingDirection>())
                {
                    kind.SelectedItem = function.ToString();
                    direction.SelectedItem = way;
                    string label = $"{function} {way}";
                    Assert.AreEqual(label, graph.SelectedTransitionLabel);
                    StringAssert.EndsWith((string)dialog.FindControl<ComboBox>("AccelerationTransitionList")!.SelectedItem!, $"· {label}");
                    ScalarTimeline command = graph.Regions[0].Channels[channel];
                    for (int step = 1; step < 10; step++)
                    {
                        Assert.AreEqual(6 * ReferenceEasing.Evaluate(function, way, step / 10.0), command.Evaluate(step).Value, 1e-12, label);
                    }
                    AssertRenderedPathMatches(graph, channel, command, 0, 10);
                }
            }
            Assert.AreSame(stored, session.Segments.Single(), "Easing choices change only the settings draft.");

            // The calculated motion uses the chosen command curve.
            dialog.FindControl<TextBox>("SampleOffset")!.Text = "5";
            Button calculate = dialog.FindControl<Button>("CalculateButton")!;
            calculate.BringIntoView();
            Click(dialog, calculate);
            for (int poll = 0; !calculate.IsEnabled && poll < 400; poll++) { await Task.Delay(10); }
            dialog.FindControl<TextBox>("SampleOffset")!.Text = "5";
            Click(dialog, dialog.GetLogicalDescendants().OfType<Button>().Single(button => Equals(button.Content, "Read Sample")));
            double expected = 6 * ReferenceEasing.Evaluate(EasingKind.Plateau, EasingDirection.InOut, .5);
            StringAssert.Contains(dialog.FindControl<TextBlock>("MotionSample")!.Text!, $"Commanded {expected:G6} m/s²");

            kind.SelectedItem = "Constant";
            StringAssert.Contains(status.Text!, "Constant transition requires equal endpoint values");
            Assert.AreEqual(nameof(EasingKind.Plateau), kind.SelectedItem, "A rejected choice restores the applied easing.");

            Click(dialog, dialog.FindControl<Button>("AccelerationUndo")!);
            Assert.AreEqual("Plateau Out", graph.SelectedTransitionLabel);
            dialog.KeyPress(Key.Y, OperatingSystem.IsMacOS() ? RawInputModifiers.Meta : RawInputModifiers.Control, PhysicalKey.Y, null);
            Assert.AreEqual("Plateau InOut", graph.SelectedTransitionLabel);

            // A new Acceleration Region starts at zero, so every easing draws flat until its endpoints differ.
            Click(dialog, dialog.GetLogicalDescendants().OfType<Button>().Single(button => Equals(button.Content, "Add Region")));
            Assert.AreEqual(1, dialog.FindControl<ComboBox>("AccelerationRegionList")!.SelectedIndex);
            kind.SelectedItem = nameof(EasingKind.Quintic);
            Assert.IsTrue(dialog.FindControl<TextBlock>("SpanShapeNote")!.IsVisible);

            Click(dialog, dialog.GetLogicalDescendants().OfType<Button>().Single(button => Equals(button.Content, "Cancel")));
            Assert.AreSame(stored, session.Segments.Single());
        });
    }

    [TestMethod]
    [DataRow(EasingKind.Sine, EasingDirection.In)]
    [DataRow(EasingKind.Quintic, EasingDirection.Out)]
    [DataRow(EasingKind.Plateau, EasingDirection.InOut)]
    public void ForceTransitionsReconstructTheCalculatedForceOfTheChosenFunction(EasingKind function, EasingDirection way)
    {
        TimelineEditorSession session = LiveEditingSessionTests.TwoRegionTrack(RegionAuthoringMode.Force).Timeline;
        Assert.IsNull(session.UpdateTransition(0, NormalForceChannel, 0, 10, 1, 1.6, new(function, way)));
        double largestDifference = 0;
        for (int step = 1; step < 10; step++)
        {
            double station = step;
            double authored = 1 + .6 * ReferenceEasing.Evaluate(function, way, step / 10.0);
            double calculated = session.Geometry!.CalculateSpecificForceG(station, EvaluationSide.Right).Z;
            Assert.AreEqual(authored, calculated, 1e-6, $"Normal force at {station} m");
            largestDifference = System.Math.Max(largestDifference, System.Math.Abs(calculated - (1 + .06 * station)));
        }
        Assert.IsGreaterThan(.02, largestDifference, "The calculated force follows the function, not a straight line.");
    }

    /// <summary>Checks the drawn polyline of one span against the profile the graph displays.</summary>
    private static void AssertRenderedPathMatches(TransitionGraph graph, string channel, ScalarTimeline profile, double start, double end)
    {
        TransitionGraphViewport viewport = graph.Viewport;
        GraphPosition[] drawn = graph.CurvePaths(channel).SelectMany(path => path).Select(viewport.Unproject)
            .Where(position => position.Distance > start && position.Distance < end).ToArray();
        Assert.IsGreaterThan(20, drawn.Length);
        foreach (GraphPosition position in drawn)
        {
            Assert.AreEqual(profile.Evaluate(position.Distance).Value, position.Value, 1e-9);
        }
    }
}
