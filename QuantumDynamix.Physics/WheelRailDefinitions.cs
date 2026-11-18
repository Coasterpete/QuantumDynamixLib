using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.Linq;
using QuantumDynamix.Geometry;
using QuantumDynamix.Track;

namespace QuantumDynamix.Physics;

/// <summary>Finite outer tread, revolved about local +X. Dimensions are metres.</summary>
/// <remarks>At axial coordinate a, radius is R-C+sqrt(C*C-a*a). Only smooth tread interiors
/// are supported; this definition supplies neither sidewalls nor an inferred closed wheel solid.</remarks>
public sealed class CrownedWheelTread
{
    public double Radius { get; }
    public double HalfWidth { get; }
    public double CrownRadius { get; }

    public CrownedWheelTread(double radius, double halfWidth, double crownRadius)
    {
        ContactChecks.Positive(radius, nameof(radius));
        ContactChecks.Positive(halfWidth, nameof(halfWidth));
        ContactChecks.Positive(crownRadius, nameof(crownRadius));
        if (halfWidth >= crownRadius
            || !(radius - halfWidth * (halfWidth / crownRadius)
                / (1 + System.Math.Sqrt(1 - (halfWidth / crownRadius) * (halfWidth / crownRadius))) > 0))
        {
            throw new ArgumentException("The crown must have positive radius throughout its finite width and halfWidth < crownRadius.");
        }

        Radius = radius;
        HalfWidth = halfWidth;
        CrownRadius = crownRadius;
    }
}

/// <summary>Owner-authored wheel geometry attached to an existing body's COM frame; no spin coordinate.</summary>
public sealed class WheelDefinition
{
    public string Id { get; }
    public int BodyIndex { get; }
    public RigidPose3d LocalPose { get; }
    public CrownedWheelTread Tread { get; }

    public WheelDefinition(string id, int bodyIndex, RigidPose3d localPose, CrownedWheelTread tread)
    {
        ArgumentException.ThrowIfNullOrWhiteSpace(id);
        ArgumentOutOfRangeException.ThrowIfNegative(bodyIndex);
        ArgumentNullException.ThrowIfNull(tread);
        localPose.Validate(nameof(localPose));
        Id = id;
        BodyIndex = bodyIndex;
        LocalPose = localPose;
        Tread = tread;
    }
}

public enum RailMaterialSide { PositiveAreaVector = 1, NegativeAreaVector = -1 }

/// <summary>Caller-declared regular smooth surface patch. Station bounds are heartline metres;
/// profile bounds use the existing spline parameter. Edges are search boundaries, not contact features.</summary>
/// <remarks>The caller declares that the patch is smooth and has no intersections. Bounded numerical
/// checks cannot certify those properties for arbitrary callbacks. No global material solid is inferred.</remarks>
public sealed class RailContactPatch
{
    public string Id { get; }
    public string RailId { get; }
    public int BoundaryIndex { get; }
    public double StationMinimum { get; }
    public double StationMaximum { get; }
    public double ProfileMinimum { get; }
    public double ProfileMaximum { get; }
    public RailMaterialSide MaterialSide { get; }

    public RailContactPatch(string id, string railId, int boundaryIndex, double stationMinimum,
        double stationMaximum, double profileMinimum, double profileMaximum, RailMaterialSide materialSide)
    {
        ArgumentException.ThrowIfNullOrWhiteSpace(id);
        ArgumentException.ThrowIfNullOrWhiteSpace(railId);
        ArgumentOutOfRangeException.ThrowIfNegative(boundaryIndex);
        ContactChecks.Interval(stationMinimum, stationMaximum);
        ContactChecks.Interval(profileMinimum, profileMaximum);
        if (!Enum.IsDefined(materialSide)) { throw new ArgumentOutOfRangeException(nameof(materialSide)); }
        Id = id;
        RailId = railId;
        BoundaryIndex = boundaryIndex;
        StationMinimum = stationMinimum;
        StationMaximum = stationMaximum;
        ProfileMinimum = profileMinimum;
        ProfileMaximum = profileMaximum;
        MaterialSide = materialSide;
    }
}

/// <summary>An explicit permitted pair. The only initial law is frictionless rigid inelastic contact.</summary>
public sealed record WheelRailPair(string WheelId, string PatchId);

/// <summary>Immutable contact bindings around the existing dynamics system and stationary rails.</summary>
public sealed class WheelRailContactSystem
{
    public MultibodySystem Dynamics { get; }
    public RailGeometry Rails { get; }
    public ReadOnlyCollection<WheelDefinition> Wheels { get; }
    public ReadOnlyCollection<RailContactPatch> Patches { get; }
    public ReadOnlyCollection<WheelRailPair> Pairs { get; }
    internal WheelDefinition Wheel(string id) => Wheels.First(w => w.Id == id);
    internal RailContactPatch Patch(string id) => Patches.First(p => p.Id == id);

    public WheelRailContactSystem(MultibodySystem dynamics, RailGeometry rails,
        IEnumerable<WheelDefinition> wheels, IEnumerable<RailContactPatch> patches, IEnumerable<WheelRailPair> pairs)
    {
        ArgumentNullException.ThrowIfNull(dynamics);
        ArgumentNullException.ThrowIfNull(rails);
        ArgumentNullException.ThrowIfNull(wheels);
        ArgumentNullException.ThrowIfNull(patches);
        ArgumentNullException.ThrowIfNull(pairs);
        WheelDefinition[] wheelArray = wheels.ToArray();
        RailContactPatch[] patchArray = patches.ToArray();
        WheelRailPair[] pairArray = pairs.ToArray();
        if (wheelArray.Any(w => w is null || w.BodyIndex >= dynamics.Bodies.Count)
            || wheelArray.Select(w => w.Id).Distinct(StringComparer.Ordinal).Count() != wheelArray.Length
            || patchArray.Any(p => p is null)
            || patchArray.Select(p => p.Id).Distinct(StringComparer.Ordinal).Count() != patchArray.Length)
        { throw new ArgumentException("Supply non-null, uniquely identified definitions referencing existing bodies."); }
        foreach (RailContactPatch patch in patchArray)
        {
            RailDefinition rail = rails.Arrangement.GetRail(patch.RailId);
            if (patch.BoundaryIndex >= rail.CrossSection.Boundaries.Count || patch.StationMinimum < 0
                || patch.StationMaximum > rails.HeartlineLength)
            { throw new ArgumentException("Patch bounds must lie within the selected rail surface.", nameof(patches)); }
            var boundary = rail.CrossSection.Boundaries[patch.BoundaryIndex];
            if (patch.ProfileMinimum < boundary.Minimum || patch.ProfileMaximum > boundary.Maximum)
            { throw new ArgumentException("Patch profile bounds exceed the boundary domain.", nameof(patches)); }
            // Conservatively require a single declared span. Even smooth interior knots can instead
            // be bound as separate patches; no corner/seam traversal is inferred in this first stage.
            if (rails.Breakpoints.Any(s => s > patch.StationMinimum && s < patch.StationMaximum)
                || boundary.Definition.Breakpoints.Any(v => v > patch.ProfileMinimum && v < patch.ProfileMaximum))
            { throw new ArgumentException("Initial contact patches must not cross known station/profile breakpoints.", nameof(patches)); }
        }
        if (pairArray.Any(p => p is null || !wheelArray.Any(w => w.Id == p.WheelId)
            || !patchArray.Any(s => s.Id == p.PatchId)) || pairArray.Distinct().Count() != pairArray.Length)
        { throw new ArgumentException("Pairs must be unique and reference existing wheels and patches.", nameof(pairs)); }
        Dynamics = dynamics;
        Rails = rails;
        Wheels = Array.AsReadOnly(wheelArray.OrderBy(w => w.Id, StringComparer.Ordinal).ToArray());
        Patches = Array.AsReadOnly(patchArray.OrderBy(p => p.Id, StringComparer.Ordinal).ToArray());
        Pairs = Array.AsReadOnly(pairArray.OrderBy(p => p.WheelId, StringComparer.Ordinal)
            .ThenBy(p => p.PatchId, StringComparer.Ordinal).ToArray());
    }
}

internal static class ContactChecks
{
    internal static void Positive(double value, string name)
    {
        if (!double.IsFinite(value) || value <= 0) { throw new ArgumentOutOfRangeException(name, "Must be finite and positive."); }
    }

    internal static void Interval(double minimum, double maximum)
    {
        if (!double.IsFinite(minimum) || !double.IsFinite(maximum) || !double.IsFinite(maximum - minimum) || maximum <= minimum)
        { throw new ArgumentException("Bounds must form a finite, positive interval."); }
    }
}
