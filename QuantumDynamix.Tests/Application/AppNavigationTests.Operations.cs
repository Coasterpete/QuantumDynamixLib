using System;
using System.IO;
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

namespace QuantumDynamix.Tests.Application;

public sealed partial class AppNavigationTests
{
    [TestMethod]
    public async Task OperationsWorkflowSelectsEditsCalculatesAndReopensThroughTheEditor()
    {
        string path = Path.Combine(Path.GetTempPath(), $"operations-{Guid.NewGuid():N}.qdproject");
        try
        {
            await WithMainWindowAsync(async window =>
            {
                Click(window, "OpenEditorButton");
                var workspace = (WorkspaceView)window.Content!;
                TrackDocument document = OperationalSegmentTests.AuthoredTrack();
                document.Timeline.MarkSaved();
                workspace.OpenTrack(document);
                var viewport = workspace.FindControl<TrackViewport>("ViewportSurface")!;
                var list = workspace.FindControl<ListBox>("SegmentList")!;
                var regionList = workspace.FindControl<ListBox>("RegionList")!;
                var scene = viewport.Scene;
                viewport.SetView(TrackView.Top);

                ClickEditorControl(window, "OperationsButton");
                Assert.IsTrue(list.IsEffectivelyVisible);
                Assert.IsFalse(regionList.IsEffectivelyVisible);
                Click(window, "AddSegmentButton");
                TrackSegmentSettings dialog = SegmentDialog(window);
                Assert.IsFalse(dialog.FindControl<StackPanel>("DeviceFields")!.IsVisible);
                Set("SegmentEnd", "22");
                ClickSetting("Add Track Segment");
                var normal = (OperationalSegment)list.SelectedItem!;
                Assert.AreEqual(OperationalSegmentType.NormalTrack, normal.Type);
                Assert.AreEqual(normal.Id, viewport.ActiveSegmentId);
                Assert.AreSame(scene, viewport.Scene);

                Click(window, "AddSegmentButton");
                dialog = SegmentDialog(window);
                Set("SegmentEnd", "43");
                ClickSetting("Add Track Segment");
                var transport = (OperationalSegment)list.SelectedItem!;
                list.SelectedIndex = 0;
                Assert.AreEqual(normal.Id, viewport.ActiveSegmentId);
                var line = viewport.Projected.Lines.Where(line => line.Selection.SegmentId == transport.Id).ToArray();
                var center = line[line.Length / 2];
                Point? pick = viewport.TranslatePoint(new((center.Start.X + center.End.X) / 2,
                    (center.Start.Y + center.End.Y) / 2), window);
                Assert.IsNotNull(pick);
                window.MouseDown(pick.Value, MouseButton.Left);
                window.MouseUp(pick.Value, MouseButton.Left);
                Dispatcher.UIThread.RunJobs();
                Assert.AreSame(transport, list.SelectedItem);
                Assert.AreEqual(transport.Id, viewport.ActiveSegmentId);

                Click(window, "SegmentSettingsButton");
                dialog = SegmentDialog(window);
                dialog.FindControl<ComboBox>("SegmentType")!.SelectedItem = "Transport";
                Assert.IsTrue(dialog.FindControl<StackPanel>("DeviceFields")!.IsVisible);
                Set("EntrySpeed", "2");
                Set("SpeedLimit", "1");
                ClickSetting("Calculate Longitudinal Motion");
                StringAssert.Contains(dialog.FindControl<TextBlock>("SettingsStatus")!.Text!, "speed limit");
                Set("SpeedLimit", "4");
                ClickSetting("Add Region");
                ClickSetting("Split");
                dialog.FindControl<ComboBox>("AccelerationTransitionList")!.SelectedIndex = 0;
                Set("SpanLength", "10");
                Set("SpanEnd", "3");
                dialog.FindControl<ComboBox>("SpanKind")!.SelectedItem = "Quadratic";
                ClickSetting("Apply Transition");
                var graph = (TransitionGraph)dialog.FindControl<ContentControl>("AccelerationGraphHost")!.Content!;
                Assert.AreEqual(.75, graph.Regions.Single().Channels[DeviceAccelerationRegion.Channel].Evaluate(5).Value, 1e-10);
                Set("SampleOffset", "15");
                ClickSetting("Calculate Longitudinal Motion");
                var calculate = dialog.FindControl<Button>("CalculateButton")!;
                for (int poll = 0; !calculate.IsEnabled && poll < 400; poll++) { await Task.Delay(10); }
                Assert.IsTrue(calculate.IsEnabled);
                StringAssert.Contains(dialog.FindControl<TextBlock>("MotionSummary")!.Text!, "Segment end");
                Set("SampleOffset", "15");
                ClickSetting("Read Sample");
                string sample = dialog.FindControl<TextBlock>("MotionSample")!.Text!;
                StringAssert.Contains(sample, "Station 37 m · Speed 4 m/s");
                StringAssert.Contains(sample, "Calculated 0 m/s²");
                ClickSetting("Apply Settings");
                Assert.AreEqual(OperationalSegmentType.Transport, ((OperationalSegment)list.SelectedItem!).Type);
                Assert.HasCount(2, regionList.Items);
                Assert.AreSame(scene, viewport.Scene);
                Assert.IsTrue(workspace.HasUnsavedChanges);

                workspace.ProjectSavePicker = _ => Task.FromResult<string?>(path);
                OpenMenu(workspace, "SaveProjectMenu");
                await WaitForProjectOperation(workspace);
                Assert.IsFalse(workspace.HasUnsavedChanges);
                OpenMenu(workspace, "UndoMenu");
                Assert.AreEqual(OperationalSegmentType.NormalTrack, ((OperationalSegment)list.SelectedItem!).Type);
                Assert.IsTrue(workspace.HasUnsavedChanges);
                OpenMenu(workspace, "RedoMenu");
                Assert.IsFalse(workspace.HasUnsavedChanges);
                Assert.AreSame(scene, viewport.Scene);

                workspace.ProjectOpenPicker = _ => Task.FromResult<string?>(path);
                OpenMenu(workspace, "OpenProjectMenu");
                await WaitForProjectOperation(workspace);
                Assert.IsTrue(regionList.IsEffectivelyVisible);
                ClickEditorControl(window, "OperationsButton");
                list.SelectedIndex = 1;
                Assert.AreEqual(transport.Id, viewport.ActiveSegmentId);
                Click(window, "SegmentSettingsButton");
                dialog = SegmentDialog(window);
                Assert.AreEqual("43", dialog.FindControl<TextBox>("SegmentEnd")!.Text);
                Assert.AreEqual("Quadratic", dialog.FindControl<ComboBox>("SpanKind")!.SelectedItem);
                Set("SegmentEnd", "44");
                ClickSetting("Apply Settings");
                Assert.AreEqual(44, ((OperationalSegment)list.SelectedItem!).End);
                Assert.IsTrue(workspace.HasUnsavedChanges);
                OpenMenu(workspace, "UndoMenu");
                Assert.AreEqual(43, ((OperationalSegment)list.SelectedItem!).End);
                Assert.IsFalse(workspace.HasUnsavedChanges);
                OpenMenu(workspace, "RedoMenu");
                Assert.AreEqual(44, ((OperationalSegment)list.SelectedItem!).End);
                Assert.IsTrue(workspace.HasUnsavedChanges);

                void Set(string name, string value)
                {
                    dialog.FindControl<TextBox>(name)!.Text = value;
                    Dispatcher.UIThread.RunJobs();
                }

                void ClickSetting(string label)
                {
                    Button button = dialog.GetLogicalDescendants().OfType<Button>().Single(button => Equals(button.Content, label));
                    button.BringIntoView();
                    Dispatcher.UIThread.RunJobs();
                    Click(dialog, button);
                }
            });
        }
        finally { if (File.Exists(path)) { File.Delete(path); } }
    }

    private static TrackSegmentSettings SegmentDialog(Window window) => window.OwnedWindows.OfType<TrackSegmentSettings>().Single();
}
