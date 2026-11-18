using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.Linq;
using QuantumDynamix.Math.Functions;

namespace QuantumDynamix.Math.Timelines;

/// <summary>Resolved station snapshot. Stable feature identities are independent of station-table samples.</summary>
public sealed class FeatureStations
{
    public IReadOnlyDictionary<string, double> Stations { get; }
    public FeatureStations(IEnumerable<KeyValuePair<string, double>> stations)
    {
        ArgumentNullException.ThrowIfNull(stations);
        var values = new Dictionary<string, double>(StringComparer.Ordinal);
        foreach (var item in stations)
        {
            if (string.IsNullOrWhiteSpace(item.Key) || !double.IsFinite(item.Value) || item.Value < 0 || !values.TryAdd(item.Key, item.Value))
            { throw new ArgumentException("Feature IDs must be unique and nonblank, with finite nonnegative stations.", nameof(stations)); }
        }
        Stations = new ReadOnlyDictionary<string, double>(values);
    }
    public double Resolve(string id) => Stations.TryGetValue(id, out double station) ? station : throw new KeyNotFoundException($"Unresolved authored feature '{id}'.");
}

public enum SpatialAnchorKind { FeatureRelative, FixedDistance }

public sealed record SpatialAnchor
{
    public SpatialAnchorKind Kind { get; }
    public string? StartFeature { get; }
    public string? EndFeature { get; }
    public double Fraction { get; }
    public double Distance { get; }
    private SpatialAnchor(SpatialAnchorKind kind, string? start, string? end, double fraction, double distance)
    { Kind = kind; StartFeature = start; EndFeature = end; Fraction = fraction; Distance = distance; }

    public static SpatialAnchor BetweenFeatures(string start, string end, double fraction)
    {
        if (string.IsNullOrWhiteSpace(start) || string.IsNullOrWhiteSpace(end) || start == end) { throw new ArgumentException("Supply two distinct stable feature IDs."); }
        if (!double.IsFinite(fraction) || fraction < 0 || fraction > 1) { throw new ArgumentOutOfRangeException(nameof(fraction)); }
        return new(SpatialAnchorKind.FeatureRelative, start, end, fraction, 0);
    }
    public static SpatialAnchor AtDistance(double metres)
    {
        if (!double.IsFinite(metres) || metres < 0) { throw new ArgumentOutOfRangeException(nameof(metres)); }
        return new(SpatialAnchorKind.FixedDistance, null, null, 0, metres);
    }
    public double Resolve(FeatureStations features)
    {
        ArgumentNullException.ThrowIfNull(features);
        if (Kind == SpatialAnchorKind.FixedDistance) { return Distance; }
        double a = features.Resolve(StartFeature!);
        double b = features.Resolve(EndFeature!);
        if (!(b > a)) { throw new InvalidOperationException("Anchor features must resolve in increasing station order."); }
        return a + Fraction * (b - a);
    }
}

public sealed record AnchoredKeyframe(SpatialAnchor Anchor, double Value, IScalarFunction? EasingToNext = null);

/// <summary>Authored anchors are retained; resolving after a geometry edit creates a new timeline snapshot.</summary>
public sealed class AnchoredTimeline
{
    public IReadOnlyList<AnchoredKeyframe> Keyframes { get; }
    public BoundaryPolicy Boundary { get; }
    public AnchoredTimeline(IEnumerable<AnchoredKeyframe> keyframes, BoundaryPolicy boundary = BoundaryPolicy.Reject)
    {
        ArgumentNullException.ThrowIfNull(keyframes);
        AnchoredKeyframe[] keys = keyframes.ToArray();
        if (keys.Length < 2 || keys.Any(k => k is null || k.Anchor is null || !double.IsFinite(k.Value)))
        { throw new ArgumentException("Supply at least two finite anchored keys.", nameof(keyframes)); }
        if (!Enum.IsDefined(boundary)) { throw new ArgumentOutOfRangeException(nameof(boundary)); }
        Keyframes = Array.AsReadOnly(keys);
        Boundary = boundary;
    }
    public ScalarTimeline Resolve(FeatureStations features)
    {
        ArgumentNullException.ThrowIfNull(features);
        return new(ProfileDomain.Distance, Keyframes.Zip(Keyframes.Skip(1), (a, b) =>
            FunctionSegment.Transition(a.Anchor.Resolve(features), b.Anchor.Resolve(features), a.Value, b.Value, a.EasingToNext)), Boundary);
    }
}
