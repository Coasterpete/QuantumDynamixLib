using System;
using System.Collections.Generic;

namespace QuantumDynamix.App;

/// <summary>Validated Region inputs and the reconstruction anchor for an edit that has not been committed.</summary>
internal sealed record TrackEditDraft(IReadOnlyList<AuthoredRegion> Regions, TrackReconstructionAnchor? Anchor);

/// <summary>The Regions and reconstructed geometry of a draft. Mixed-mode joins may rebind Region start values.</summary>
internal sealed record TrackEditPreview(IReadOnlyList<AuthoredRegion> Regions, RegionTrackGeometry? Geometry);

internal sealed partial class TimelineEditorSession
{
    private bool drafting;
    private TrackEditDraft? draft;

    /// <summary>Runs an edit through the normal validation path without changing the committed state or history.</summary>
    /// <returns>The validation error, or null. A null draft with no error means the edit changes nothing.</returns>
    internal string? Draft(Func<TimelineEditorSession, string?> edit, out TrackEditDraft? result)
    {
        if (drafting) { throw new InvalidOperationException("Draft edits cannot be nested."); }
        drafting = true;
        draft = null;
        try
        {
            string? error = edit(this);
            result = error is null ? draft : null;
            return error;
        }
        finally
        {
            drafting = false;
            draft = null;
        }
    }

    /// <summary>Builds a draft's geometry with the same reconstruction used for commits.</summary>
    /// <remarks>This reads only immutable session settings and the draft, so it can run off the UI thread.</remarks>
    internal string? BuildPreview(TrackEditDraft edit, out TrackEditPreview? preview)
    {
        preview = null;
        AuthoredRegion[] regions = [.. edit.Regions];
        string? error = BuildSnapshot(regions, edit.Anchor, out Snapshot? snapshot);
        if (error is not null) { return error; }
        preview = new(snapshot!.Regions, snapshot.Geometry);
        return null;
    }
}
