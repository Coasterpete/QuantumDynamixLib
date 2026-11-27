using System;
using System.Linq;
using System.Threading.Tasks;
using Avalonia.Controls;
using Avalonia.Interactivity;

namespace QuantumDynamix.App;

internal sealed partial class WorkspaceView
{
    private bool operationsOpen;
    private bool refreshingSegments;
    private Guid? activeSegment;
    private OperationalSegment? SelectedSegment => SegmentList.SelectedItem as OperationalSegment;

    private void ToggleOperations(object? sender, RoutedEventArgs e)
    {
        operationsOpen = !operationsOpen;
        RefreshOperations();
        if (operationsOpen) { SegmentList.Focus(); }
    }

    private async void AddSegment(object? sender, RoutedEventArgs e)
    {
        if (Session?.NewSegment() is { } segment) { await ShowSegmentSettings(segment, adding: true); }
        else { StatusLabel.Text = "No unassigned heartline interval. Edit an existing segment to free track."; }
    }

    private async void EditSegment(object? sender, RoutedEventArgs e)
    {
        if (SelectedSegment is { } segment) { await ShowSegmentSettings(segment, adding: false); }
    }

    private async Task ShowSegmentSettings(OperationalSegment segment, bool adding)
    {
        TimelineEditorSession session = Session!;
        var dialog = new TrackSegmentSettings(session, segment, adding);
        if (await dialog.ShowDialog<bool>(OwnerWindow))
        {
            activeSegment = segment.Id;
            RefreshOperations();
            RefreshHistoryControls();
            RefreshProjectControls();
            StatusLabel.Text = adding ? "Added the track segment." : "Applied track segment settings.";
        }
    }

    private void RemoveSegment(object? sender, RoutedEventArgs e)
    {
        if (SelectedSegment is not { } segment) { return; }
        Session!.RemoveSegment(segment.Id);
        RefreshOperations();
        RefreshHistoryControls();
        RefreshProjectControls();
        StatusLabel.Text = "Removed the track segment.";
    }

    private void SelectSegment(object? sender, SelectionChangedEventArgs e)
    {
        if (refreshingSegments) { return; }
        activeSegment = SelectedSegment?.Id;
        ViewportSurface.SetActiveSegment(activeSegment);
        RefreshSegmentActions();
    }

    private void SelectSegmentById(Guid? id)
    {
        SegmentList.SelectedItem = Session?.Segments.FirstOrDefault(segment => segment.Id == id);
        if (SegmentList.SelectedIndex >= 0) { SegmentList.ScrollIntoView(SegmentList.SelectedIndex); }
    }

    private void RefreshOperations()
    {
        RegionTrackGeometry? geometry = IsTrack ? Session?.Geometry : null;
        bool show = operationsOpen && geometry is not null;
        OperationsButton.IsEnabled = geometry is not null;
        OperationsPanel.IsVisible = show;
        RegionListPanel.IsVisible = !show;
        refreshingSegments = true;
        SegmentList.ItemsSource = show ? Session!.Segments : null;
        if (show) { SelectSegmentById(activeSegment); }
        refreshingSegments = false;

        WireframePath[] paths = [];
        if (show)
        {
            WireframeVertex[] heartline = ViewportSurface.Scene.Paths.Where(path => path.Kind == WireframePathKind.Heartline)
                .SelectMany(path => path.Vertices).DistinctBy(vertex => vertex.HeartlineStation).ToArray();
            paths = Session!.Segments.Select(segment => new WireframePath(WireframePathKind.OperationalSegment,
                segment.ToString(), [At(segment.Start), .. heartline.Where(vertex =>
                    vertex.HeartlineStation > segment.Start && vertex.HeartlineStation < segment.End), At(segment.End)],
                SegmentId: segment.Id)).ToArray();

            WireframeVertex At(double station) => new(geometry!.UnbankedFrameAt(station).Position, station);
        }
        ViewportSurface.SetOperations(show, paths, show ? SelectedSegment?.Id : null);
        RefreshSegmentActions();
    }

    private void RefreshSegmentActions()
    {
        AddSegmentButton.IsEnabled = OperationsPanel.IsVisible;
        SegmentSettingsButton.IsEnabled = RemoveSegmentButton.IsEnabled = SelectedSegment is not null;
    }

    private void RefreshHistoryControls()
    {
        UndoMenu.IsEnabled = UndoButton.IsEnabled = Session?.CanUndo == true;
        RedoMenu.IsEnabled = RedoButton.IsEnabled = Session?.CanRedo == true;
    }
}
