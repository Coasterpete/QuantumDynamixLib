using System;
using System.Linq;
using System.Threading.Tasks;

namespace QuantumDynamix.App;

/// <summary>Direct manipulation in the Transition Editor with live graph and viewport previews.</summary>
/// <remarks>
/// A drag never changes the committed session. Each pointer position is validated through the same session edit
/// used for commits, shown immediately on the graph, and reconstructed off the UI thread for the viewport.
/// Release commits once through the normal edit path. Escape, capture loss, or Undo/Redo discards the preview.
/// </remarks>
internal sealed partial class WorkspaceView
{
    private readonly LatestWinsWorker<LivePreview> previewWorker;
    private bool lastDragWasBoundary;

    private sealed record LivePreview(TrackWireframe? Scene, string? Error);

    internal bool LivePreviewPending => previewWorker.IsBusy;
    internal Task WhenLivePreviewIdle() => previewWorker.WhenIdle();

    private void PreviewDrag(TransitionEdit? candidate)
    {
        if (!graph.IsDragging)
        {
            EndLivePreview(restoreFields: true);
            return;
        }
        if (Session is not { } session)
        {
            return;
        }
        if (candidate is null)
        {
            StatusLabel.Text = $"{DescribeInvalidDrag()} Showing the last valid preview.";
            return;
        }

        lastDragWasBoundary = candidate is TransitionEdit.Boundary;
        string? error = DraftDragEdit(session, candidate, out TrackEditDraft? draft);
        if (error is not null)
        {
            StatusLabel.Text = $"{error} Showing the last valid preview.";
            return;
        }
        ShowDragReadout(candidate);
        if (draft is null)
        {
            // The pointer is back at the committed position.
            previewWorker.Cancel();
            graph.SetRegionPreview(null);
            ViewportSurface.SetPreviewScene(null);
            return;
        }

        graph.SetRegionPreview(draft.Regions.Select(region => region.Definition).ToArray());
        if (DisplayedTrack is { } document)
        {
            previewWorker.Request(() => BuildLivePreview(document, session, draft));
        }
    }

    private static LivePreview BuildLivePreview(TrackDocument document, TimelineEditorSession session, TrackEditDraft draft)
    {
        string? error = session.BuildPreview(draft, out TrackEditPreview? preview);
        return error is null ? new(document.SamplePreview(preview!), null) : new(null, error);
    }

    private void ApplyLivePreview(LivePreview result)
    {
        if (!graph.IsDragging)
        {
            return;
        }
        if (result.Error is { } error)
        {
            StatusLabel.Text = $"{error} The 3D view keeps the last valid preview.";
        }
        else if (result.Scene is { } scene)
        {
            ViewportSurface.SetPreviewScene(scene);
        }
    }

    private void EndLivePreview(bool restoreFields)
    {
        previewWorker.Cancel();
        // The first Open runs before any graph exists.
        if (graph is not null) { graph.SetRegionPreview(null); }
        ViewportSurface.SetPreviewScene(null);
        if (restoreFields && Session is not null && SelectedRegion >= 0)
        {
            UpdateKeyframeSelection();
            RefreshRegionFields();
        }
    }

    private string? DraftDragEdit(TimelineEditorSession session, TransitionEdit candidate, out TrackEditDraft? draft)
    {
        draft = null;
        int region = SelectedRegion;
        switch (candidate)
        {
            case TransitionEdit.Keyframe key when region >= 0 && EditingChannel == key.Channel:
                int index = KeyframeList.SelectedIndex;
                if (index < 0 || session.Keys(region, key.Channel)[index] != key.Original)
                {
                    return "The dragged keyframe is no longer selected.";
                }
                return session.Draft(edit => edit.Move(region, key.Channel, index, key.Proposed), out draft);
            case TransitionEdit.Boundary boundary when IsTrack && boundary.Left >= 0 && boundary.Left < session.Regions.Count:
                return session.Draft(edit => edit.MoveRegionEnd(boundary.Left, boundary.Proposed), out draft);
            default:
                return null;
        }
    }

    private void ShowDragReadout(TransitionEdit candidate)
    {
        const string release = "Release to apply. Press Escape to cancel.";
        switch (candidate)
        {
            case TransitionEdit.Keyframe key:
                StationInput.Text = Format(key.Proposed.Coordinate);
                ValueInput.Text = Format(key.Proposed.Value);
                StatusLabel.Text = $"{key.Channel} keyframe · {key.Proposed.Coordinate:G6} m · {key.Proposed.Value:G6}. {release}";
                break;
            case TransitionEdit.Boundary boundary when IsTrack && boundary.Left >= 0:
                AuthoredRegion region = track!.Timeline.Regions[boundary.Left];
                double length = boundary.Proposed - region.Definition.Start;
                if (boundary.Left == SelectedRegion)
                {
                    RegionLengthInput.Text = Format(length);
                    RegionSummary.Text = $"{DescribeMode(region.Mode)} · {Format(region.Definition.Start)}–{Format(boundary.Proposed)} m";
                }
                StatusLabel.Text = $"{region.Name} end · {boundary.Proposed:G6} m · {length:G6} m long. {release}";
                break;
        }
    }

    private void CompleteDrag(TransitionEdit edit)
    {
        switch (edit)
        {
            case TransitionEdit.Keyframe key when EditingChannel == key.Channel && key.Original != key.Proposed:
                int index = KeyframeList.SelectedIndex;
                if (index >= 0 && Session!.Keys(SelectedRegion, key.Channel)[index] == key.Original)
                {
                    string? error = Session.Move(SelectedRegion, key.Channel, index, key.Proposed);
                    FinishKeyEdit(error, index, $"Moved the {key.Channel} keyframe.");
                }
                break;
            case TransitionEdit.Boundary boundary when IsTrack && boundary.Left >= 0 && boundary.Original != boundary.Proposed:
                CommitRegionEnd(boundary.Left, boundary.Proposed);
                break;
        }
    }

    private void CommitRegionEnd(int index, double end)
    {
        TimelineEditorSession session = track!.Timeline;
        AuthoredRegion region = session.Regions[index];
        string? error = session.MoveRegionEnd(index, end);
        if (error is not null)
        {
            StatusLabel.Text = error;
            return;
        }
        RefreshRegions(region.Id, index, EditingChannel);
        StatusLabel.Text = $"Resized {region.Name} · {DescribeExtent(session.Regions[index].Definition)}.";
    }

    /// <summary>Only a Region end can be dragged. The following Regions move with it, as with a length edit.</summary>
    private BoundaryDragLimits? ResolveBoundaryDrag(GraphBoundary boundary) =>
        IsTrack && boundary.Left >= 0 && boundary.Left < track!.Timeline.Regions.Count
            ? new BoundaryDragLimits(track.Timeline.Regions[boundary.Left].Definition.Start, double.MaxValue)
            : null;

    private void SelectBoundaryOnGraph(GraphBoundary boundary)
    {
        int region = boundary.Left >= 0 ? boundary.Left : boundary.Right;
        if (region < 0) { return; }
        SelectRegionInList(region);
        if (IsTrack && inspector != InspectorMode.Region)
        {
            SetInspectorMode(InspectorMode.Region);
        }
    }

    private void SelectCurveOnGraph(int region, string channel, int span)
    {
        SelectRegionInList(region);
        if (Session is null || SelectedRegion != region)
        {
            return;
        }
        if (EditingChannel != channel && ActiveChannel.ItemsSource is string[] channels && channels.Contains(channel))
        {
            ActiveChannel.SelectedItem = channel;
        }
        if (IsTrack && EditingChannel == channel)
        {
            SelectTransitionOnGraph(span);
        }
    }

    private string DescribeInvalidDrag() => lastDragWasBoundary
        ? "Keep the Region end after its start station."
        : "Keep the station strictly between its neighboring keyframes.";

    private void ReportInvalidDrag() => StatusLabel.Text = DescribeInvalidDrag();
}
