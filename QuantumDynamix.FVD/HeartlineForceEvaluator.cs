using System;
using QuantumDynamix.Geometry;
using QuantumDynamix.Math;
using QuantumDynamix.Math.Functions;
using QuantumDynamix.Math.Timelines;
using QuantumDynamix.Splines.Curves;
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
    internal const double StandardGravity = 9.80665;

    internal static Vector3d ProjectSpecificForce(CurveFrame frame, Vector3d worldAcceleration, Vector3d worldGravity)
    {
        Vector3d specificForce = worldAcceleration - worldGravity;
        return new(Vector3d.Dot(specificForce, frame.Forward), Vector3d.Dot(specificForce, frame.Left),
            Vector3d.Dot(specificForce, frame.Up));
    }

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

        EvaluationSide limit = GeometryLimit(heartlineStation, length, side);
        ScalarJet speedJet = heartlineSpeed.Evaluate(heartlineStation, limit);
        double speed = speedJet.Value;
        if (!(speed > 0))
        {
            throw new ArgumentOutOfRangeException(nameof(heartlineSpeed), "Speed must be positive at the requested station.");
        }

        double tangentialAcceleration = speed * speedJet.FirstDerivative;
        return Project(track, heartlineStation, speed, tangentialAcceleration, worldGravity, limit);
    }

    /// <summary>Projects prescribed longitudinal motion onto the heartline, including rest and reverse motion.</summary>
    /// <param name="track">Track supplying the heartline and its banked orientation.</param>
    /// <param name="motion">Signed station velocity and longitudinal coordinate acceleration at a heartline station.</param>
    /// <param name="worldGravity">Finite world gravitational acceleration in m/s².</param>
    /// <param name="geometrySide">The geometry limit, independent of the acceleration side. Endpoints use inward limits.</param>
    public static HeartlineForceSample Evaluate(HeartlineTrack track, LongitudinalMotionSample motion,
        Vector3d worldGravity, EvaluationSide geometrySide) => EvaluateMotion(track, motion, worldGravity, geometrySide);

    // Verification supplies the source used to construct this track's fitted banking function.
    internal static HeartlineForceSample EvaluateFitted(HeartlineTrack track, ReconstructedHeartline source,
        LongitudinalMotionSample motion, EvaluationSide geometrySide) =>
        EvaluateMotion(track, motion, Vector3d.Zero, geometrySide, source);

    private static HeartlineForceSample EvaluateMotion(HeartlineTrack track, LongitudinalMotionSample motion,
        Vector3d worldGravity, EvaluationSide geometrySide, ReconstructedHeartline? fittedSource = null)
    {
        ArgumentNullException.ThrowIfNull(track);
        if (!Enum.IsDefined(geometrySide)) { throw new ArgumentOutOfRangeException(nameof(geometrySide)); }

        double station = motion.HeartlineStationMetres;
        double length = track.HeartlineStations.Length;
        if (!double.IsFinite(station) || station < 0 || station > length)
        { throw new ArgumentOutOfRangeException(nameof(motion), "Station must lie within the heartline."); }
        Check.Finite(motion.HeartlineVelocityMetresPerSecond, nameof(motion));
        Check.Finite(motion.LongitudinalAccelerationMetresPerSecondSquared, nameof(motion));
        return Project(track, station, System.Math.Abs(motion.HeartlineVelocityMetresPerSecond),
            motion.LongitudinalAccelerationMetresPerSecondSquared, worldGravity, GeometryLimit(station, length, geometrySide),
            fittedSource);
    }

    private static EvaluationSide GeometryLimit(double station, double length, EvaluationSide side) => station switch
    {
        0 => EvaluationSide.Right,
        _ when station == length => EvaluationSide.Left,
        _ => side
    };

    private static HeartlineForceSample Project(HeartlineTrack track, double heartlineStation, double speed,
        double tangentialAcceleration, Vector3d worldGravity, EvaluationSide limit, ReconstructedHeartline? fittedSource = null)
    {
        double parameter = track.HeartlineStations.Invert(heartlineStation).Parameter;
        CurveJet3d jet = track.Heartline.Evaluate(parameter, limit);
        CurveGeometrySample geometry = CurveGeometry.Evaluate(jet);
        Vector3d worldAcceleration = geometry.Tangent * tangentialAcceleration + geometry.CurvatureVector * speed * speed;

        double bankAngle = fittedSource is null
            ? track.Banking.Evaluate(heartlineStation, limit).Value
            : HeartlineFitMapping.BankAtParameter(fittedSource, track.Heartline, parameter, limit, jet).Value;
        CurveFrame bankedFrame = track.Frames.Evaluate(parameter).Banked(bankAngle);
        return new(heartlineStation, speed, bankedFrame, worldAcceleration,
            ProjectSpecificForce(bankedFrame, worldAcceleration, worldGravity));
    }
}
