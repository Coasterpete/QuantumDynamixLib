using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.Linq;
using QuantumDynamix.Math;
using QuantumDynamix.Math.Functions;
using QuantumDynamix.Splines.Curves;

namespace QuantumDynamix.Track;

/// <summary>Caller-authored boundaries and named measurement datums in the section plane X=0 (metres).
/// Boundary winding, closure, material sides, and datum semantics belong to the caller.</summary>
public sealed class RailCrossSection
{
    public IReadOnlyList<SplineCurve3d> Boundaries { get; }
    public IReadOnlyDictionary<string, Vector3d> Datums { get; }

    public RailCrossSection(IEnumerable<SplineCurve3d> boundaries, IEnumerable<KeyValuePair<string, Vector3d>>? datums = null)
    {
        ArgumentNullException.ThrowIfNull(boundaries);
        SplineCurve3d[] copy = boundaries.ToArray();
        if (copy.Length == 0 || copy.Any(c => c is null || c.ControlPoints.Any(p => p.X != 0)))
        { throw new ArgumentException("Supply non-null section boundaries whose controls lie in X=0.", nameof(boundaries)); }
        var points = new Dictionary<string, Vector3d>(StringComparer.Ordinal);
        foreach (var datum in datums ?? Array.Empty<KeyValuePair<string, Vector3d>>())
        {
            if (string.IsNullOrWhiteSpace(datum.Key) || datum.Value.X != 0 || !points.TryAdd(datum.Key, datum.Value))
            { throw new ArgumentException("Section datums require unique nonblank names and positions in X=0.", nameof(datums)); }
        }
        Boundaries = Array.AsReadOnly(copy);
        Datums = new ReadOnlyDictionary<string, Vector3d>(points);
    }
}

/// <summary>Spine-relative translations in banked heartline coordinates (metres), and section roll (radians).
/// All profiles use heartline station in metres. No frame algorithm is embedded in this definition.</summary>
public sealed class RailDefinition
{
    public string Id { get; }
    public TrackOffsetProfile Offsets { get; }
    public RailCrossSection CrossSection { get; }
    public IScalarFunction SectionRoll { get; }

    public RailDefinition(string id, TrackOffsetProfile offsets, RailCrossSection crossSection, IScalarFunction sectionRoll)
    {
        ArgumentException.ThrowIfNullOrWhiteSpace(id);
        ArgumentNullException.ThrowIfNull(offsets);
        ArgumentNullException.ThrowIfNull(crossSection);
        ArgumentNullException.ThrowIfNull(sectionRoll);
        Id = id;
        Offsets = offsets;
        CrossSection = crossSection;
        SectionRoll = sectionRoll;
    }
}

/// <summary>Measurement at corresponding heartline stations. BankedLateral is the signed projection B-A
/// onto banked heartline left; Euclidean is the nonnegative distance, not a closest-point search.</summary>
public enum RailSpacingMeasurement { Euclidean, BankedLateral }

/// <summary>Explicit pairwise center spacing, or gauge between two caller-named section datums.
/// Expected is in metres as a function of heartline metres.</summary>
public sealed class RailSpacingDefinition
{
    public string Id { get; }
    public string RailA { get; }
    public string RailB { get; }
    public RailSpacingMeasurement Measurement { get; }
    public IScalarFunction Expected { get; }
    public string? DatumA { get; }
    public string? DatumB { get; }
    public bool UsesSectionDatums => DatumA is not null;

    public RailSpacingDefinition(string id, string railA, string railB, RailSpacingMeasurement measurement,
        IScalarFunction expected, string? datumA = null, string? datumB = null)
    {
        ArgumentException.ThrowIfNullOrWhiteSpace(id);
        ArgumentException.ThrowIfNullOrWhiteSpace(railA);
        ArgumentException.ThrowIfNullOrWhiteSpace(railB);
        ArgumentNullException.ThrowIfNull(expected);
        if (railA == railB || !Enum.IsDefined(measurement) || (datumA is null) != (datumB is null)
            || datumA is not null && (string.IsNullOrWhiteSpace(datumA) || string.IsNullOrWhiteSpace(datumB)))
        { throw new ArgumentException("Select distinct rails, a defined measurement, and either two datums or neither."); }
        Id = id;
        RailA = railA;
        RailB = railB;
        Measurement = measurement;
        Expected = expected;
        DatumA = datumA;
        DatumB = datumB;
    }
}

/// <summary>Any nonempty rail arrangement; no symmetry, gauge, or manufacturer defaults are inferred.</summary>
public sealed class RailArrangement
{
    public IReadOnlyList<RailDefinition> Rails { get; }
    public IReadOnlyList<RailSpacingDefinition> Spacings { get; }
    private readonly Dictionary<string, RailDefinition> byId;

    public RailArrangement(IEnumerable<RailDefinition> rails, IEnumerable<RailSpacingDefinition>? spacings = null)
    {
        ArgumentNullException.ThrowIfNull(rails);
        RailDefinition[] copy = rails.ToArray();
        if (copy.Length == 0 || copy.Any(r => r is null) || copy.Select(r => r.Id).Distinct(StringComparer.Ordinal).Count() != copy.Length)
        { throw new ArgumentException("Supply rails with unique IDs.", nameof(rails)); }
        byId = copy.ToDictionary(r => r.Id, StringComparer.Ordinal);
        RailSpacingDefinition[] measures = spacings?.ToArray() ?? Array.Empty<RailSpacingDefinition>();
        if (measures.Any(m => m is null) || measures.Select(m => m.Id).Distinct(StringComparer.Ordinal).Count() != measures.Length)
        { throw new ArgumentException("Spacing IDs must be unique.", nameof(spacings)); }
        foreach (RailSpacingDefinition measure in measures)
        {
            if (!byId.TryGetValue(measure.RailA, out RailDefinition? a) || !byId.TryGetValue(measure.RailB, out RailDefinition? b)
                || measure.UsesSectionDatums && (!a.CrossSection.Datums.ContainsKey(measure.DatumA!) || !b.CrossSection.Datums.ContainsKey(measure.DatumB!)))
            { throw new ArgumentException("Spacing references an unknown rail or section datum.", nameof(spacings)); }
        }
        Rails = Array.AsReadOnly(copy);
        Spacings = Array.AsReadOnly(measures);
    }

    public RailDefinition GetRail(string id) => byId.TryGetValue(id, out RailDefinition? rail)
        ? rail : throw new KeyNotFoundException($"Unknown rail '{id}'.");
}
