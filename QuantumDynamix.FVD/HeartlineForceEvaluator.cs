using System;
using QuantumDynamix.Geometry;
using QuantumDynamix.Math;
using QuantumDynamix.Math.Functions;
using QuantumDynamix.Math.Timelines;
using QuantumDynamix.Track;

namespace QuantumDynamix.FVD;

/// <summary>A kinematic sample at a point on the authored heartline.</summary>
/// <param name="HeartlineStation">Heartline arc length in metres.</param>
/// <param name="HeartlineSpeed">Prescribed heartline speed in metres per second.</param>
/// <param name="BankedFrame">World position in metres and banked forward, left, and up axes.</param>
/// <param name="WorldCoordinateAcceleration">Second time derivative of world position in m/s².</param>
/// <param name="TrackSpecificForce">Non-gravitational force per unit mass in m/s², ordered forward, left, up.</param>
public readonly record struct HeartlineForceSample(
    double HeartlineStation,
    double HeartlineSpeed,
    CurveFrame BankedFrame,
    Vector3d WorldCoordinateAcceleration,
    Vector3d TrackSpecificForce);

/// <summary>Evaluates the acceleration and specific force required for prescribed motion along a heartline.</summary>
public static class HeartlineForceEvaluator
{
    /// <summary>Samples an ideal point following the authored heartline.</summary>
    /// <param name="track">Track supplying the heartline and its banked orientation.</param>
    /// <param name="heartlineSpeed">Continuous distance timeline of speed in m/s over the entire heartline.</param>
    /// <param name="heartlineStation">Distance from the heartline start in metres.</param>
    /// <param name="worldGravity">Gravitational acceleration in world coordinates, in m/s².</param>
    /// <param name="side">Limit at an interior join. Track endpoints always use the inward limit.</param>
    /// <remarks>The speed timeline must use <see cref="BoundaryPolicy.Reject"/> with value jumps disabled.
    /// The query requires positive speed, its first station derivative, and the heartline curvature vector.</remarks>
    public static HeartlineForceSample Evaluate(
        HeartlineTrack track,
        ScalarTimeline heartlineSpeed,
        double heartlineStation,
        Vector3d worldGravity,
        EvaluationSide side = EvaluationSide.Automatic)
    {
        ArgumentNullException.ThrowIfNull(track);
        ArgumentNullException.ThrowIfNull(heartlineSpeed);
        if (!Enum.IsDefined(side))
        {
            throw new ArgumentOutOfRangeException(nameof(side));
        }

        double length = track.HeartlineStations.Length;
        if (!double.IsFinite(heartlineStation) || heartlineStation < 0 || heartlineStation > length)
        {
            throw new ArgumentOutOfRangeException(nameof(heartlineStation), "Station must lie within the heartline.");
        }

        if (heartlineSpeed.Domain != ProfileDomain.Distance)
        {
            throw new ArgumentException("Speed must be a function of heartline distance.", nameof(heartlineSpeed));
        }

        if (heartlineSpeed.Boundary != BoundaryPolicy.Reject || heartlineSpeed.AllowsDiscontinuities)
        {
            throw new ArgumentException("Speed must use the Reject boundary policy with value jumps disabled.", nameof(heartlineSpeed));
        }

        if (heartlineSpeed.Minimum > 0 || heartlineSpeed.Maximum < length)
        {
            throw new ArgumentException("Speed must cover the complete heartline distance interval.", nameof(heartlineSpeed));
        }

        EvaluationSide limit = heartlineStation switch
        {
            0 => EvaluationSide.Right,
            _ when heartlineStation == length => EvaluationSide.Left,
            _ => side
        };
        ScalarJet speedJet = heartlineSpeed.Evaluate(heartlineStation, limit);
        double speed = speedJet.Value;
        if (!(speed > 0))
        {
            throw new ArgumentOutOfRangeException(nameof(heartlineSpeed), "Speed must be positive at the requested station.");
        }

        double tangentialAcceleration = speed * speedJet.FirstDerivative;
        double parameter = track.HeartlineStations.Invert(heartlineStation).Parameter;
        CurveGeometrySample geometry = CurveGeometry.Evaluate(track.Heartline, parameter, limit);
        Vector3d worldAcceleration = geometry.Tangent * tangentialAcceleration + geometry.CurvatureVector * speed * speed;

        double bankAngle = track.Banking.Evaluate(heartlineStation, limit).Value;
        CurveFrame bankedFrame = track.Frames.Evaluate(parameter).Banked(bankAngle);
        Vector3d worldSpecificForce = worldAcceleration - worldGravity;
        Vector3d trackSpecificForce = new(
            Vector3d.Dot(worldSpecificForce, bankedFrame.Forward),
            Vector3d.Dot(worldSpecificForce, bankedFrame.Left),
            Vector3d.Dot(worldSpecificForce, bankedFrame.Up));

        return new(heartlineStation, speed, bankedFrame, worldAcceleration, trackSpecificForce);
    }
}
