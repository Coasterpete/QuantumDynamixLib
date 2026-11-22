using System;
using System.Collections.Generic;
using System.Linq;
using QuantumDynamix.Geometry;
using QuantumDynamix.Math;
using QuantumDynamix.Math.Numerics;
using QuantumDynamix.Math.Timelines;
using QuantumDynamix.Track;

namespace QuantumDynamix.FVD;

/// <summary>Resolved transverse Regions, prescribed motion and initial unbanked pose for a full traversal.</summary>
/// <remarks>
/// Custom acceleration, banking, and motion callbacks must be pure and deterministic. Keep their captured inputs
/// stable for the lifetime of the reconstructed result and any fitted track derived from it. Reconstruction,
/// conversion, verification, and later queries can evaluate the same function again at the same coordinate and side.
/// Declare breakpoints on both sides of localized features in custom transverse or banking profiles.
/// Choose integration steps and fit verification spacing that resolve variation within those intervals.
/// Numerical samples alone cannot detect an arbitrary feature between the sampled coordinates.
/// </remarks>
public sealed class HeartlineReconstructionRequest
{
    /// <param name="regions">Adjacent Regions on one authored axis, in metres or authoring seconds.</param>
    /// <param name="lateralChannelId">Banked left coordinate acceleration in m/s².</param>
    /// <param name="verticalChannelId">Banked up coordinate acceleration in m/s².</param>
    /// <param name="longitudinalTrajectory">M2 motion on the same traversal, with its original station origin.</param>
    /// <param name="initialUnbankedPose">World position in metres and local forward/left/up rotation.</param>
    /// <param name="banking">Continuous unwrapped radians over authored station in metres. Null means zero bank.</param>
    /// <param name="authoringTimeAtStartSeconds">Clock origin for time targets with distance-domain motion.</param>
    /// <param name="timeAccelerationBounds">Conservative m/s² bounds on the given binding over authoring seconds.
    /// Use inward endpoint limits and include roundoff. Callbacks must remain pure and deterministic.</param>
    public HeartlineReconstructionRequest(IEnumerable<Region> regions, string lateralChannelId, string verticalChannelId,
        LongitudinalTrajectory longitudinalTrajectory, RigidPose3d initialUnbankedPose,
        ScalarTimeline? banking = null, double? authoringTimeAtStartSeconds = null,
        Func<LongitudinalAccelerationBinding, double, double, AccelerationRange?>? timeAccelerationBounds = null)
    {
        ArgumentNullException.ThrowIfNull(regions);
        ArgumentException.ThrowIfNullOrWhiteSpace(lateralChannelId);
        ArgumentException.ThrowIfNullOrWhiteSpace(verticalChannelId);
        ArgumentNullException.ThrowIfNull(longitudinalTrajectory);
        Region[] snapshot = regions.ToArray();
        if (snapshot.Length == 0 || snapshot.Any(region => region is null))
        { throw new ArgumentException("Supply at least one non-null Region.", nameof(regions)); }
        if (authoringTimeAtStartSeconds is double time && !double.IsFinite(time))
        { throw new ArgumentOutOfRangeException(nameof(authoringTimeAtStartSeconds)); }
        Regions = Array.AsReadOnly(snapshot);
        LateralChannelId = lateralChannelId;
        VerticalChannelId = verticalChannelId;
        LongitudinalTrajectory = longitudinalTrajectory;
        InitialUnbankedPose = initialUnbankedPose;
        Banking = banking;
        AuthoringTimeAtStartSeconds = authoringTimeAtStartSeconds;
        TimeAccelerationBounds = timeAccelerationBounds;
    }

    public IReadOnlyList<Region> Regions { get; }
    public string LateralChannelId { get; }
    public string VerticalChannelId { get; }
    public LongitudinalTrajectory LongitudinalTrajectory { get; }
    public RigidPose3d InitialUnbankedPose { get; }
    public ScalarTimeline? Banking { get; }
    public double? AuthoringTimeAtStartSeconds { get; }
    public Func<LongitudinalAccelerationBinding, double, double, AccelerationRange?>? TimeAccelerationBounds { get; }
}

public sealed class HeartlineReconstructionSettings
{
    /// <param name="accelerationJoinTolerances">One to four tolerances starting at order zero. Units are m/s²
    /// divided by the corresponding power of the Region coordinate unit.</param>
    /// <param name="permittedCurvatureStepCoordinates">Exact interior transverse breakpoint coordinates
    /// on the authored Region axis, in metres or authoring seconds. Only explicitly permitted transverse value jumps
    /// receive a C1 knot in the fit. Other joins retain the configured continuity and both authored limits.</param>
    public HeartlineReconstructionSettings(IEnumerable<double> accelerationJoinTolerances,
        IEnumerable<double>? permittedCurvatureStepCoordinates = null)
    {
        ArgumentNullException.ThrowIfNull(accelerationJoinTolerances);
        double[] limits = accelerationJoinTolerances.ToArray();
        if (limits.Length is < 1 or > 4 || limits.Any(value => !double.IsFinite(value) || value < 0))
        { throw new ArgumentException("Supply one to four finite nonnegative tolerances.", nameof(accelerationJoinTolerances)); }
        double[] steps = permittedCurvatureStepCoordinates?.ToArray() ?? [];
        if (steps.Any(value => !double.IsFinite(value)) || steps.Distinct().Count() != steps.Length)
        { throw new ArgumentException("Step coordinates must be finite and distinct.", nameof(permittedCurvatureStepCoordinates)); }
        AccelerationJoinTolerances = Array.AsReadOnly(limits);
        PermittedCurvatureStepCoordinates = Array.AsReadOnly(steps);
    }

    public IReadOnlyList<double> AccelerationJoinTolerances { get; }
    public IReadOnlyList<double> PermittedCurvatureStepCoordinates { get; }
    public required double PositionToleranceMetres { get; init; }
    public required double FrameAngleToleranceRadians { get; init; }
    public required double CurvatureTolerancePerMetre { get; init; }
    public required double UnbankedRollRateToleranceRadiansPerMetre { get; init; }
    /// <summary>Allowed difference between the two bank values at a join, in radians. The original values are retained.</summary>
    public required double BankJoinToleranceRadians { get; init; }
    public required double StationMappingToleranceMetres { get; init; }
    public required double TimeMappingToleranceSeconds { get; init; }
    public required double MinimumForwardSpeedMetresPerSecond { get; init; }
    public required double MaximumStepMetres { get; init; }
    /// <summary>Total M3 work per reconstruction or independent sample query.
    /// Each M2 query retains its own existing work limit.</summary>
    public required int MaximumWork { get; init; }

    internal void Validate()
    {
        TrajectoryValidation.Positive(PositionToleranceMetres, nameof(PositionToleranceMetres));
        TrajectoryValidation.Positive(FrameAngleToleranceRadians, nameof(FrameAngleToleranceRadians));
        TrajectoryValidation.Positive(CurvatureTolerancePerMetre, nameof(CurvatureTolerancePerMetre));
        TrajectoryValidation.Positive(UnbankedRollRateToleranceRadiansPerMetre, nameof(UnbankedRollRateToleranceRadiansPerMetre));
        TrajectoryValidation.Nonnegative(BankJoinToleranceRadians, nameof(BankJoinToleranceRadians));
        TrajectoryValidation.Positive(StationMappingToleranceMetres, nameof(StationMappingToleranceMetres));
        TrajectoryValidation.Positive(TimeMappingToleranceSeconds, nameof(TimeMappingToleranceSeconds));
        TrajectoryValidation.Positive(MinimumForwardSpeedMetresPerSecond, nameof(MinimumForwardSpeedMetresPerSecond));
        TrajectoryValidation.Positive(MaximumStepMetres, nameof(MaximumStepMetres));
        ArgumentOutOfRangeException.ThrowIfLessThan(MaximumWork, 1);
    }
}

public sealed class HeartlineFitSettings
{
    public required double PositionToleranceMetres { get; init; }
    public required double TangentToleranceRadians { get; init; }
    public required double CurvatureTolerancePerMetre { get; init; }
    public required double BankedFrameToleranceRadians { get; init; }
    public required double BankRollToleranceRadians { get; init; }
    public required double StationDriftToleranceMetres { get; init; }
    /// <summary>Positive absolute tolerances in m/s², ordered forward, left, up.</summary>
    public required Vector3d CoordinateAccelerationTolerancesMetresPerSecondSquared { get; init; }

    public int MinimumContinuousKnotOrder { get; init; } = 2;
    /// <summary>Positive regularity margin in fitted metres per source metre.</summary>
    public required double MinimumParameterSpeed { get; init; }
    public required QuadratureSettings ArcLengthSettings { get; init; }
    public required double FrameTransportToleranceRadians { get; init; }
    /// <summary>Maximum spacing of independent fit probes in local source metres. Residuals describe sampled stations.</summary>
    public required double MaximumVerificationSpacingMetres { get; init; }
    /// <summary>Total forward verification samples across all candidate fits, counting each side separately.</summary>
    public required int MaximumVerificationSamples { get; init; }
    public required int MaximumControlPoints { get; init; }
    /// <summary>Maximum number of candidate fits, including the initial fit.</summary>
    public required int MaximumRefinementPasses { get; init; }

    internal void Validate()
    {
        TrajectoryValidation.Positive(PositionToleranceMetres, nameof(PositionToleranceMetres));
        TrajectoryValidation.Positive(TangentToleranceRadians, nameof(TangentToleranceRadians));
        if (TangentToleranceRadians >= System.Math.PI)
        { throw new ArgumentOutOfRangeException(nameof(TangentToleranceRadians)); }
        TrajectoryValidation.Positive(CurvatureTolerancePerMetre, nameof(CurvatureTolerancePerMetre));
        TrajectoryValidation.Positive(BankedFrameToleranceRadians, nameof(BankedFrameToleranceRadians));
        TrajectoryValidation.Positive(BankRollToleranceRadians, nameof(BankRollToleranceRadians));
        TrajectoryValidation.Positive(StationDriftToleranceMetres, nameof(StationDriftToleranceMetres));
        Vector3d tolerances = CoordinateAccelerationTolerancesMetresPerSecondSquared;
        if (!(tolerances.X > 0 && tolerances.Y > 0 && tolerances.Z > 0))
        { throw new ArgumentOutOfRangeException(nameof(CoordinateAccelerationTolerancesMetresPerSecondSquared)); }
        if (MinimumContinuousKnotOrder is < 2 or > 4)
        { throw new ArgumentOutOfRangeException(nameof(MinimumContinuousKnotOrder)); }
        TrajectoryValidation.Positive(MinimumParameterSpeed, nameof(MinimumParameterSpeed));
        ArgumentNullException.ThrowIfNull(ArcLengthSettings);
        TrajectoryValidation.Positive(FrameTransportToleranceRadians, nameof(FrameTransportToleranceRadians));
        TrajectoryValidation.Positive(MaximumVerificationSpacingMetres, nameof(MaximumVerificationSpacingMetres));
        ArgumentOutOfRangeException.ThrowIfLessThan(MaximumVerificationSamples, 1);
        ArgumentOutOfRangeException.ThrowIfLessThan(MaximumControlPoints, 1);
        ArgumentOutOfRangeException.ThrowIfLessThan(MaximumRefinementPasses, 1);
    }
}
