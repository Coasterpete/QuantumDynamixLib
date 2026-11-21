using System;
using System.Collections.Generic;
using QuantumDynamix.Math;
using QuantumDynamix.Math.Functions;
using QuantumDynamix.Track;

namespace QuantumDynamix.Supports;

/// <summary>Evaluates straight support members between authored rail datums and world foundation points.</summary>
public static class SupportMemberGeometry
{
    /// <summary>Resolves a named rail section datum at the supplied heartline station and evaluates the member span.</summary>
    /// <param name="geometry">Rail geometry in the same world coordinates as the foundation point.</param>
    /// <param name="railId">Case-sensitive rail identifier.</param>
    /// <param name="datumName">Case-sensitive datum name within the selected rail section.</param>
    /// <param name="heartlineStation">Distance along the heartline in metres, between zero and its length inclusive.</param>
    /// <param name="foundationPoint">Caller-supplied foundation point in world metres.</param>
    /// <param name="side">Evaluation side for the rail geometry at the station.</param>
    /// <exception cref="KeyNotFoundException">The rail or section datum identifier is unknown.</exception>
    /// <exception cref="InvalidOperationException">The evaluated endpoints coincide.</exception>
    /// <exception cref="RailGeometryException">The rail geometry cannot supply the section point.</exception>
    public static SupportMemberSample Evaluate(
        RailGeometry geometry,
        string railId,
        string datumName,
        double heartlineStation,
        Vector3d foundationPoint,
        EvaluationSide side = EvaluationSide.Automatic)
    {
        ArgumentNullException.ThrowIfNull(geometry);
        ArgumentException.ThrowIfNullOrWhiteSpace(railId);
        ArgumentException.ThrowIfNullOrWhiteSpace(datumName);
        if (!double.IsFinite(heartlineStation) || heartlineStation < 0 || heartlineStation > geometry.HeartlineLength)
        {
            throw new ArgumentOutOfRangeException(nameof(heartlineStation), "Station must lie within the finite heartline distance range.");
        }

        if (!Enum.IsDefined(side))
        {
            throw new ArgumentOutOfRangeException(nameof(side));
        }

        RailDefinition rail = geometry.Arrangement.GetRail(railId);
        if (!rail.CrossSection.Datums.TryGetValue(datumName, out Vector3d localDatum))
        {
            throw new KeyNotFoundException($"Unknown section datum '{datumName}' on rail '{railId}'.");
        }

        RailSample railSample = geometry.Evaluate(railId, heartlineStation, side);
        Vector3d railAttachmentPoint = railSample.TransformSectionPoint(localDatum);
        Vector3d span = railAttachmentPoint - foundationPoint;
        if (span == Vector3d.Zero)
        {
            throw new InvalidOperationException("The foundation point and rail attachment point coincide.");
        }

        return new SupportMemberSample(foundationPoint, railAttachmentPoint, span.Length, span.Normalized());
    }
}
