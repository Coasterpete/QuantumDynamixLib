using System;
using System.Collections.Generic;
using System.Linq;
using QuantumDynamix.FVD;
using QuantumDynamix.Geometry;
using QuantumDynamix.Math;
using QuantumDynamix.Math.Functions;
using QuantumDynamix.Math.Timelines;

namespace QuantumDynamix.App;

internal enum RegionAuthoringMode { CoordinateAcceleration, Force, Geometric }

internal sealed record AuthoredRegion(Guid Id, string Name, Region Definition,
    IReadOnlyDictionary<string, TimelineKeyframe[]> Keyframes, RegionAuthoringMode Mode = RegionAuthoringMode.CoordinateAcceleration)
{
    internal TransitionShape[] Shapes(string channel) =>
        Definition.Channels[channel].Segments.Select(TransitionShape.FromSegment).ToArray();
}

internal readonly record struct TrackReconstructionAnchor(double Station, CurveFrame Frame, double? GeometricHeadingRadians = null);

internal sealed partial class TimelineEditorSession
{
    private sealed record Snapshot(AuthoredRegion[] Regions, double MinimumValue, double MaximumValue,
        RegionTrackGeometry? Geometry = null, TrackReconstructionAnchor? Anchor = null)
    {
        internal IReadOnlyList<OperationalSegment> Segments { get; init; } = [];
    }

    private readonly Stack<Snapshot> undo = new();
    private readonly Stack<Snapshot> redo = new();
    private Snapshot current;
    private Snapshot? saved;

    internal TimelineEditorSession(params AuthoredRegion[] regions) => current = new(regions, 0, 1);

    internal TimelineEditorSession(RigidPose3d initialPose, double speed)
        : this(new CurveFrame(initialPose.Position, initialPose.TransformDirection(Vector3d.UnitX),
            initialPose.TransformDirection(Vector3d.UnitY), initialPose.TransformDirection(Vector3d.UnitZ)), speed) { }

    private TimelineEditorSession(CurveFrame initialFrame, double speed) : this()
    {
        if (!RegionTrackGeometry.SupportsSpeed(speed)) { throw new ArgumentOutOfRangeException(nameof(speed)); }
        ReconstructionSpeed = speed;
        InitialFrame = initialFrame;
    }

    internal static TimelineEditorSession RestoreAuthored(CurveFrame initialFrame, double speed,
        AuthoredRegion[] regions, TrackReconstructionAnchor? anchor, OperationalSegment[]? segments = null)
    {
        var session = new TimelineEditorSession(initialFrame, speed);
        string? error = session.BuildSnapshot(regions, anchor, out Snapshot? snapshot);
        if (error is not null) { throw new InvalidOperationException(error); }
        session.current = snapshot!;
        if (segments is { Length: > 0 } && session.ReplaceSegments(segments) is { } problem)
        { throw new InvalidOperationException(problem); }
        session.undo.Clear();
        session.MarkSaved();
        return session;
    }

    internal CurveFrame InitialFrame { get; }
    internal double? ReconstructionSpeed { get; }
    internal TrackReconstructionAnchor? ReconstructionAnchor => current.Anchor;
    internal IReadOnlyList<AuthoredRegion> Regions => current.Regions;
    internal double MinimumValue => current.MinimumValue;
    internal double MaximumValue => current.MaximumValue;
    internal bool CanUndo => undo.Count > 0;
    internal bool CanRedo => redo.Count > 0;
    internal bool SupportsForceRegions => ReconstructionSpeed.HasValue;
    internal RegionTrackGeometry? Geometry => current.Geometry;
    internal bool HasUnsavedChanges => !ReferenceEquals(current, saved);
    internal void MarkSaved() => saved = current;

    internal IReadOnlyList<TimelineKeyframe> Keys(int region, string channel) => Regions[region].Keyframes[channel];

    internal string? AddRegion(bool prepend, double length, double initialStation, RegionAuthoringMode? mode = null)
    {
        if (!double.IsFinite(length) || length <= 0)
        {
            return "Enter a finite, positive heartline length.";
        }
        if (prepend && Regions.Count == 0)
        {
            return "Append the first region before prepending.";
        }

        AuthoredRegion? neighbor = Regions.Count == 0 ? null : Regions[prepend ? 0 : Regions.Count - 1];
        RegionAuthoringMode selectedMode = mode ?? neighbor?.Mode ?? RegionAuthoringMode.CoordinateAcceleration;
        if (!Enum.IsDefined(selectedMode) || (selectedMode != RegionAuthoringMode.CoordinateAcceleration && !SupportsForceRegions))
        {
            return "Force-based and Geometric Regions require a track with a prescribed reconstruction speed.";
        }
        if (neighbor is not null && neighbor.Definition.Domain != ProfileDomain.Distance)
        {
            return "Track authoring requires a distance-based adjoining region.";
        }

        double join = neighbor is null ? initialStation
            : prepend ? neighbor.Definition.Start : neighbor.Definition.End;
        if (!double.IsFinite(join) || join < 0)
        {
            return "Enter a finite, nonnegative first station.";
        }
        if (prepend && length > join)
        {
            return "The new region must start at or after station 0 m.";
        }
        double start = prepend ? join - length : join;
        double end = prepend ? join : join + length;
        if (!double.IsFinite(end) || end <= start)
        {
            return "This length cannot be represented at the adjoining station.";
        }

        var profiles = new Dictionary<string, ScalarTimeline>(StringComparer.Ordinal);
        var keyframes = new Dictionary<string, TimelineKeyframe[]>(StringComparer.Ordinal);
        EvaluationSide side = prepend ? EvaluationSide.Right : EvaluationSide.Left;
        var (lateral, vertical) = RegionTrackGeometry.TransverseChannels(selectedMode);
        IEnumerable<string> channels = neighbor?.Keyframes.Keys ?? [TrackDocument.BankingChannel];
        bool initializeTransverse = SupportsForceRegions && neighbor?.Mode != selectedMode;
        Vector3d transverse = default;
        if (SupportsForceRegions)
        {
            var (neighborLateral, neighborVertical) = RegionTrackGeometry.TransverseChannels(neighbor?.Mode ?? selectedMode);
            channels = channels.Where(channel => channel != neighborLateral && channel != neighborVertical &&
                channel != lateral && channel != vertical);
            channels = [TrackDocument.BankingChannel, lateral, vertical, .. channels.Where(channel => channel != TrackDocument.BankingChannel)];
            try
            {
                if (neighbor is not null && neighbor.Mode != selectedMode)
                {
                    transverse = Geometry!.TransverseAt(join, side, selectedMode);
                }
                else if (neighbor is null && selectedMode == RegionAuthoringMode.Force)
                {
                    transverse = -RegionTrackGeometry.ProjectGravity(InitialFrame) / HeartlineForceEvaluator.StandardGravity;
                }
            }
            catch (Exception error) when (error is ArgumentException or InvalidOperationException or ArithmeticException)
            {
                return $"The adjoining transverse values could not be resolved. {error.Message}";
            }
        }
        foreach (string channel in channels)
        {
            double value = initializeTransverse && channel == lateral ? transverse.Y
                : initializeTransverse && channel == vertical ? transverse.Z
                : neighbor?.Definition.Channels[channel].Evaluate(join, side).Value ?? 0;
            TimelineKeyframe[] keys = [new(start, value), new(end, value)];
            string? error = BuildTimeline(ProfileDomain.Distance, keys, out ScalarTimeline? timeline);
            if (error is not null)
            {
                return error;
            }
            profiles.Add(channel, timeline!);
            keyframes.Add(channel, keys);
        }

        int number = Regions.Count + 1;
        string name = $"Region {number}";
        while (Regions.Any(region => region.Name == name))
        {
            name = $"Region {++number}";
        }
        var entry = new AuthoredRegion(Guid.NewGuid(), name,
            new Region(ProfileDomain.Distance, start, end, profiles), keyframes, selectedMode);
        return Commit(prepend ? [entry, .. Regions] : [.. Regions, entry]);
    }

    internal string? UpdateRegion(int index, string name, double length)
    {
        if (string.IsNullOrWhiteSpace(name))
        {
            return "Enter a region name.";
        }
        if (!double.IsFinite(length) || length <= 0)
        {
            return "Enter a finite, positive heartline length.";
        }

        AuthoredRegion region = Regions[index];
        name = name.Trim();
        double oldLength = region.Definition.End - region.Definition.Start;
        if (length == oldLength)
        {
            return name == region.Name ? null : Replace(index, region with { Name = name });
        }

        var updated = (AuthoredRegion[])current.Regions.Clone();
        double start = region.Definition.Start;
        string? error = Reposition(region with { Name = name }, start, start + length, out updated[index]);
        if (error is not null) { return error; }
        error = RepositionFollowing(updated, index + 1);
        return error ?? Commit(updated);
    }

    /// <summary>Moves a Region's end station. Following Regions keep their lengths and move with it, as with a length edit.</summary>
    internal string? MoveRegionEnd(int index, double end)
    {
        AuthoredRegion region = Regions[index];
        if (!double.IsFinite(end) || end <= region.Definition.Start)
        {
            return "Keep the Region end after its start station.";
        }
        if (end == region.Definition.End) { return null; }

        var updated = (AuthoredRegion[])current.Regions.Clone();
        string? error = Reposition(region, region.Definition.Start, end, out updated[index]);
        if (error is not null) { return error; }
        error = RepositionFollowing(updated, index + 1);
        return error ?? Commit(updated);
    }

    internal string? DeleteRegion(int index)
    {
        AuthoredRegion[] updated = [.. Regions.Take(index), .. Regions.Skip(index + 1)];
        if (SupportsForceRegions && index > 0)
        {
            string? error = RepositionFollowing(updated, index);
            if (error is not null) { return error; }
        }
        return Commit(updated);
    }

    private static string? RepositionFollowing(AuthoredRegion[] regions, int index)
    {
        for (; index < regions.Length; index++)
        {
            AuthoredRegion entry = regions[index];
            double start = regions[index - 1].Definition.End;
            if (start == entry.Definition.Start) { break; }
            double end = start + (entry.Definition.End - entry.Definition.Start);
            string? error = Reposition(entry, start, end, out regions[index]);
            if (error is not null) { return error; }
        }
        return null;
    }

    private static string? Reposition(AuthoredRegion entry, double start, double end, out AuthoredRegion updated)
    {
        updated = entry;
        if (!double.IsFinite(end) || end <= start)
        {
            return "This length cannot be represented at the adjoining station.";
        }

        Region original = entry.Definition;
        var profiles = new Dictionary<string, ScalarTimeline>(StringComparer.Ordinal);
        var keyframes = new Dictionary<string, TimelineKeyframe[]>(StringComparer.Ordinal);
        foreach ((string channel, TimelineKeyframe[] keys) in entry.Keyframes)
        {
            var moved = (TimelineKeyframe[])keys.Clone();
            moved[0] = keys[0] with { Coordinate = start };
            moved[^1] = keys[^1] with { Coordinate = end };
            for (int i = 1; i < moved.Length - 1; i++)
            {
                double fraction = (keys[i].Coordinate - original.Start) / (original.End - original.Start);
                moved[i] = keys[i] with { Coordinate = start + (end - start) * fraction };
                if (moved[i].Coordinate <= moved[i - 1].Coordinate || moved[i].Coordinate >= end)
                {
                    return $"The new station range cannot keep the keyframes in {entry.Name} distinct.";
                }
            }
            string? error = BuildTimeline(original.Domain, moved, out ScalarTimeline? timeline, entry.Shapes(channel));
            if (error is not null) { return error; }
            keyframes.Add(channel, moved);
            profiles.Add(channel, timeline!);
        }
        updated = entry with { Definition = new(original.Domain, start, end, profiles), Keyframes = keyframes };
        return null;
    }

    internal string? AddChannel(int index, string channel, double startValue, double endValue)
    {
        if (string.IsNullOrWhiteSpace(channel))
        {
            return "Enter a channel name.";
        }
        channel = channel.Trim();
        Region region = Regions[index].Definition;
        if (region.Channels.ContainsKey(channel))
        {
            return "That channel already exists in this region.";
        }
        if (SupportsForceRegions && RegionTrackGeometry.IsTransverseChannel(channel))
        {
            return "That name is reserved for a track authoring profile.";
        }
        return ReplaceTimeline(index, channel, [new(region.Start, startValue), new(region.End, endValue)]);
    }

    internal string? Add(int region, string channel, TimelineKeyframe key, out int index)
    {
        TimelineKeyframe[] keys = Regions[region].Keyframes[channel];
        index = Array.FindIndex(keys, candidate => candidate.Coordinate >= key.Coordinate);
        if (!double.IsFinite(key.Coordinate) || index <= 0 || keys[index].Coordinate == key.Coordinate)
        {
            return "Use a distinct station strictly inside the selected region.";
        }

        return InsertKey(region, channel, index, key, insertBefore: false);
    }

    internal string? Move(int region, string channel, int index, TimelineKeyframe key)
    {
        string? error = PrepareMove(region, channel, index, key, out TimelineKeyframe[]? updated);
        return updated is null ? error : ReplaceTimeline(region, channel, updated, Regions[region].Shapes(channel));
    }

    internal Region? PreviewMove(int region, string channel, int index, TimelineKeyframe key)
    {
        _ = PrepareMove(region, channel, index, key, out TimelineKeyframe[]? updated);
        return updated is not null &&
            BuildRegionWithTimeline(Regions[region].Definition, channel, updated, Regions[region].Shapes(channel),
                out Region? preview) is null
            ? preview : null;
    }

    /// <summary>Whether a keyframe can change on the graph or in the keyframe fields, and which coordinates may change.</summary>
    /// <remarks>Region endpoint stations belong to the Region boundary. Only an editable endpoint value can change.</remarks>
    internal KeyframeDragMode KeyframeEditMode(int region, string channel, int index)
    {
        int count = Regions[region].Keyframes[channel].Length;
        if (index < 0 || index >= count) { return KeyframeDragMode.None; }
        if (index > 0 && index < count - 1) { return KeyframeDragMode.Free; }
        return CanEditTransitionEndpoint(region, index, count) ? KeyframeDragMode.ValueOnly : KeyframeDragMode.None;
    }

    private string? PrepareMove(int region, string channel, int index, TimelineKeyframe key, out TimelineKeyframe[]? updated)
    {
        updated = null;
        TimelineKeyframe[] keys = Regions[region].Keyframes[channel];
        switch (KeyframeEditMode(region, channel, index))
        {
            case KeyframeDragMode.None when index >= 0 && index < keys.Length && SupportsForceRegions:
                return "The values at adjoining Region boundaries are linked.";
            case KeyframeDragMode.None:
                return "Select an interior keyframe to move. Region endpoints are fixed.";
            case KeyframeDragMode.ValueOnly when key.Coordinate != keys[index].Coordinate:
                return "A Region endpoint keeps its station. Drag the Region boundary to change the Region length.";
            case KeyframeDragMode.Free when !(key.Coordinate > keys[index - 1].Coordinate && key.Coordinate < keys[index + 1].Coordinate):
                return "Keep the station strictly between its neighboring keyframes.";
        }
        if (key == keys[index])
        {
            return null;
        }

        updated = (TimelineKeyframe[])keys.Clone();
        updated[index] = key;
        return null;
    }

    internal string? Delete(int region, string channel, int index)
    {
        TimelineKeyframe[] keys = Regions[region].Keyframes[channel];
        if (index <= 0 || index >= keys.Length - 1)
        {
            return "Select an interior keyframe to delete. Region endpoints are fixed.";
        }
        return MergeAt(region, channel, index, keepRight: false);
    }

    internal string? SplitTransition(int region, string channel, int span, bool insertBefore, out int selected)
    {
        TimelineKeyframe[] keys = Regions[region].Keyframes[channel];
        double start = keys[span].Coordinate;
        double end = keys[span + 1].Coordinate;
        double middle = start + (end - start) / 2;
        selected = insertBefore ? span : span + 1;
        if (!(middle > start && middle < end))
        {
            return "This transition is too short to split into distinct stations.";
        }
        double value = Regions[region].Definition.Channels[channel].Evaluate(middle).Value;
        return InsertKey(region, channel, span + 1, new(middle, value), insertBefore);
    }

    private string? InsertKey(int region, string channel, int index, TimelineKeyframe key, bool insertBefore)
    {
        AuthoredRegion entry = Regions[region];
        TimelineKeyframe[] keys = entry.Keyframes[channel];
        TransitionShape[] shapes = entry.Shapes(channel);
        int position = insertBefore ? index - 1 : index;
        return ReplaceTimeline(region, channel, [.. keys[..index], key, .. keys[index..]],
            [.. shapes[..position], TransitionShape.Linear, .. shapes[position..]]);
    }

    internal string? DeleteTransition(int region, string channel, int span)
    {
        int count = Keys(region, channel).Count - 1;
        if (count == 1) { return "A profile requires at least one transition."; }
        return span < count - 1 ? MergeAt(region, channel, span + 1, keepRight: true)
            : MergeAt(region, channel, span, keepRight: false);
    }

    private string? MergeAt(int region, string channel, int index, bool keepRight)
    {
        AuthoredRegion entry = Regions[region];
        TimelineKeyframe[] keys = entry.Keyframes[channel];
        TransitionShape[] shapes = entry.Shapes(channel);
        int removed = keepRight ? index - 1 : index;
        return ReplaceTimeline(region, channel, [.. keys[..index], .. keys[(index + 1)..]],
            [.. shapes[..removed], .. shapes[(removed + 1)..]]);
    }

    internal bool CanEditTransitionEndpoint(int region, int key, int keyCount) => SupportsForceRegions
        ? key > 0 || region == 0
        : key > 0 && key < keyCount - 1;

    internal string? UpdateTransition(int region, string channel, int span, double length,
        double startValue, double endValue, TransitionShape shape)
    {
        AuthoredRegion entry = Regions[region];
        TimelineKeyframe[] keys = entry.Keyframes[channel];
        TransitionShape[] shapes = entry.Shapes(channel);
        TimelineKeyframe start = keys[span];
        TimelineKeyframe end = keys[span + 1];
        if ((!CanEditTransitionEndpoint(region, span, keys.Length) && startValue != start.Value) ||
            (!CanEditTransitionEndpoint(region, span + 1, keys.Length) && endValue != end.Value))
        {
            return "The values at adjoining Region boundaries are linked.";
        }
        if (!double.IsFinite(length) || length <= 0)
        {
            return "Enter a finite, positive transition length.";
        }
        if (length != end.Coordinate - start.Coordinate)
        {
            if (span + 1 < keys.Length - 1)
            {
                end = end with { Coordinate = start.Coordinate + length };
                if (!(end.Coordinate > start.Coordinate && end.Coordinate < keys[span + 2].Coordinate))
                { return "Keep the transition end strictly before the next station."; }
            }
            else if (span > 0)
            {
                start = start with { Coordinate = end.Coordinate - length };
                if (!(start.Coordinate > keys[span - 1].Coordinate && start.Coordinate < end.Coordinate))
                { return "Keep the transition start strictly after the previous station."; }
            }
            else
            {
                return "Use Region Properties to resize a transition that spans the entire Region.";
            }
        }
        start = start with { Value = startValue };
        end = end with { Value = endValue };
        if (start == keys[span] && end == keys[span + 1] && shape == shapes[span]) { return null; }
        var updated = (TimelineKeyframe[])keys.Clone();
        updated[span] = start;
        updated[span + 1] = end;
        shapes[span] = shape;
        return ReplaceTimeline(region, channel, updated, shapes);
    }

    internal bool Undo() => Restore(undo, redo);
    internal bool Redo() => Restore(redo, undo);

    private bool Restore(Stack<Snapshot> from, Stack<Snapshot> to)
    {
        if (!from.TryPop(out Snapshot? snapshot))
        {
            return false;
        }
        to.Push(current);
        current = snapshot;
        return true;
    }

    private string? ReplaceTimeline(int regionIndex, string channel, TimelineKeyframe[] keys,
        IReadOnlyList<TransitionShape>? shapes = null)
    {
        AuthoredRegion entry = Regions[regionIndex];
        string? error = BuildEditedRegion(entry, channel, keys, shapes, out AuthoredRegion? updated);
        if (error is not null) { return error; }

        var candidates = (AuthoredRegion[])current.Regions.Clone();
        candidates[regionIndex] = updated!;
        if (regionIndex + 1 < candidates.Length && entry.Keyframes.TryGetValue(channel, out TimelineKeyframe[]? previous)
            && previous[^1].Value != keys[^1].Value)
        {
            AuthoredRegion next = candidates[regionIndex + 1];
            bool mixedTransverse = entry.Mode != next.Mode &&
                (IsModeChannel(entry.Mode, channel) || IsModeChannel(next.Mode, channel));
            // A mixed transverse join is rebound from reconstructed curvature during Commit.
            if (!mixedTransverse && next.Keyframes.TryGetValue(channel, out TimelineKeyframe[]? following))
            {
                TimelineKeyframe[] linked = [following[0] with { Value = keys[^1].Value }, .. following[1..]];
                error = BuildEditedRegion(next, channel, linked, next.Shapes(channel), out AuthoredRegion? rebound);
                if (error is not null) { return $"Linked start of {channel} in '{next.Name}'. {error}"; }
                candidates[regionIndex + 1] = rebound!;
            }
        }
        return Commit(candidates);
    }

    private static bool IsModeChannel(RegionAuthoringMode mode, string channel)
    {
        var (lateral, vertical) = RegionTrackGeometry.TransverseChannels(mode);
        return channel == lateral || channel == vertical;
    }

    private static string? BuildEditedRegion(AuthoredRegion entry, string channel, TimelineKeyframe[] keys,
        IReadOnlyList<TransitionShape>? shapes, out AuthoredRegion? updated)
    {
        updated = null;
        string? error = BuildRegionWithTimeline(entry.Definition, channel, keys, shapes, out Region? definition);
        if (error is not null) { return error; }
        var inputs = new Dictionary<string, TimelineKeyframe[]>(entry.Keyframes) { [channel] = keys };
        updated = entry with
        {
            Definition = definition!,
            Keyframes = inputs
        };
        return null;
    }

    private static string? BuildRegionWithTimeline(Region original, string channel, TimelineKeyframe[] keys,
        IReadOnlyList<TransitionShape>? shapes,
        out Region? updated)
    {
        updated = null;
        string? error = BuildTimeline(original.Domain, keys, out ScalarTimeline? timeline, shapes);
        if (error is not null)
        {
            return error;
        }

        var channels = new Dictionary<string, ScalarTimeline>(original.Channels) { [channel] = timeline! };
        updated = new(original.Domain, original.Start, original.End, channels);
        return null;
    }

    internal static string? BuildTimeline(ProfileDomain domain, TimelineKeyframe[] keys, out ScalarTimeline? timeline,
        IReadOnlyList<TransitionShape>? shapes = null)
    {
        timeline = null;
        if (keys.Any(key => !double.IsFinite(key.Value)))
        {
            return "Use finite channel values.";
        }

        if (keys.Length < 2 || (shapes is not null && shapes.Count != keys.Length - 1))
        {
            return "Supply exactly one transition for each consecutive pair of keyframes.";
        }
        var segments = new FunctionSegment[keys.Length - 1];
        try
        {
            for (int i = 0; i < segments.Length; i++)
            {
                TransitionShape shape = shapes?[i] ?? TransitionShape.Linear;
                if (shape.Kind is null && keys[i].Value != keys[i + 1].Value)
                { return "A Constant transition requires equal endpoint values."; }
                if (shape.Kind is EasingKind.Plateau &&
                    !(0 < shape.HoldStart && shape.HoldStart < shape.HoldEnd && shape.HoldEnd < 1 &&
                        0 < shape.HoldValue && shape.HoldValue < 1))
                { return "Use 0 < hold start < hold end < 1 and a hold value between 0 and 1."; }
                FunctionSegment span = shape.Build(keys[i], keys[i + 1]);
                // Finite keyframes can still produce nonfinite derivatives or unresolved breakpoints.
                double previous = span.Minimum;
                foreach (double point in span.Breakpoints)
                {
                    if (!(point > previous && point < span.Maximum))
                    { return "This transition is too short to resolve its function breakpoints."; }
                    _ = span.Evaluate(point, EvaluationSide.Left);
                    _ = span.Evaluate(point, EvaluationSide.Right);
                    _ = span.Evaluate(previous + (point - previous) / 2);
                    previous = point;
                }
                _ = span.Evaluate(previous + (span.Maximum - previous) / 2);
                _ = span.Evaluate(span.Minimum, EvaluationSide.Right);
                _ = span.Evaluate(span.Maximum, EvaluationSide.Left);
                segments[i] = span;
            }
            timeline = new ScalarTimeline(domain, segments);
        }
        catch (ArgumentException)
        {
            return "This station or value exceeds the timeline's numeric limits.";
        }
        return null;
    }

    private string? Replace(int index, AuthoredRegion entry)
    {
        var updated = (AuthoredRegion[])current.Regions.Clone();
        updated[index] = entry;
        return Commit(updated);
    }

    private string? Commit(AuthoredRegion[] updated)
    {
        if (ValidateSegmentStations(current.Segments, updated) is { } invalid) { return invalid; }
        TrackReconstructionAnchor? anchor = null;
        if (SupportsForceRegions && updated.Length > 0)
        {
            try
            {
                double firstStation = updated[0].Definition.Start;
                double anchorStation = firstStation;
                CurveFrame frame = InitialFrame;
                if (current.Geometry is { } previous)
                {
                    double oldStart = current.Regions[0].Definition.Start;
                    // On prepend or removal of the first Region, preserve the adjoining world pose.
                    anchorStation = System.Math.Max(firstStation, oldStart);
                    frame = previous.UnbankedFrameAt(anchorStation);
                    if (updated.Any(entry => entry.Mode == RegionAuthoringMode.Geometric) ||
                        current.Regions.Any(entry => entry.Mode == RegionAuthoringMode.Geometric))
                    {
                        if (updated[0].Id == current.Regions[0].Id && current.Anchor is { } existing)
                        {
                            // Keep the established heading and anchor when editing a Geometric track.
                            // After prepend, the anchor is at the original track's start.
                            bool atJoin = existing.Station == current.Regions[0].Definition.End;
                            if (!atJoin || updated.Length > 1)
                            {
                                anchor = existing with { Station = atJoin ? updated[0].Definition.End : firstStation };
                            }
                        }
                        anchor ??= new(anchorStation, frame, previous.GeometricHeadingAt(anchorStation));
                    }
                }
                anchor ??= new(anchorStation, frame);
            }
            catch (Exception error) when (error is ArgumentException or InvalidOperationException or ArithmeticException)
            {
                return $"Track geometry could not be generated. {error.Message}";
            }
        }

        if (drafting)
        {
            // A draft stops after validation. Its geometry is built separately by BuildPreview.
            draft = new(updated, anchor);
            return null;
        }
        string? problem = BuildSnapshot(updated, anchor, out Snapshot? snapshot);
        if (problem is not null) { return problem; }
        CommitSnapshot(snapshot! with { Segments = current.Segments });
        return null;
    }

    private void CommitSnapshot(Snapshot snapshot)
    {
        if (drafting) { throw new InvalidOperationException("A draft edit cannot change the session history."); }
        undo.Push(current);
        redo.Clear();
        current = snapshot;
    }

    private string? BuildSnapshot(AuthoredRegion[] updated, TrackReconstructionAnchor? anchor, out Snapshot? snapshot)
    {
        snapshot = null;
        RegionTrackGeometry? geometry = null;
        if (anchor is { } origin && ReconstructionSpeed is double speed)
        {
            try
            {
                geometry = new(updated, speed, origin.Frame, origin.Station, origin.GeometricHeadingRadians);
            }
            catch (Exception error) when (error is ArgumentException or InvalidOperationException or ArithmeticException)
            {
                return $"Track geometry could not be generated. {error.Message}";
            }
        }

        double low = 0;
        double high = 1;
        foreach (TimelineKeyframe key in updated.SelectMany(region => region.Keyframes.Values).SelectMany(keys => keys))
        {
            low = System.Math.Min(low, key.Value);
            high = System.Math.Max(high, key.Value);
        }
        if (!double.IsFinite(high - low))
        {
            return "The resulting value range is too large.";
        }

        snapshot = new(updated, low, high, geometry, anchor);
        return null;
    }
}
