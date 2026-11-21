using System;
using System.Collections.Generic;
using System.Linq;
using QuantumDynamix.Math.Timelines;

namespace QuantumDynamix.Track;

/// <summary>Absolute integration targets in metres and metres per second for one time trajectory.</summary>
public sealed class TimeTrajectorySettings
{
    public TimeTrajectorySettings(double stationAbsoluteToleranceMetres, double velocityAbsoluteToleranceMetresPerSecond,
        int maximumDepth, int maximumWork)
    {
        TrajectoryValidation.Positive(stationAbsoluteToleranceMetres, nameof(stationAbsoluteToleranceMetres));
        TrajectoryValidation.Positive(velocityAbsoluteToleranceMetresPerSecond, nameof(velocityAbsoluteToleranceMetresPerSecond));
        ArgumentOutOfRangeException.ThrowIfLessThan(maximumDepth, 1);
        ArgumentOutOfRangeException.ThrowIfLessThan(maximumWork, 1);
        StationAbsoluteToleranceMetres = stationAbsoluteToleranceMetres;
        VelocityAbsoluteToleranceMetresPerSecond = velocityAbsoluteToleranceMetresPerSecond;
        MaximumDepth = maximumDepth;
        MaximumWork = maximumWork;
    }

    public double StationAbsoluteToleranceMetres { get; }
    public double VelocityAbsoluteToleranceMetresPerSecond { get; }
    /// <summary>Subdivision depth per quadrature call, starting at zero.</summary>
    public int MaximumDepth { get; }
    /// <summary>Total evaluations per build or per independent trajectory query.</summary>
    public int MaximumWork { get; }
}

/// <summary>Absolute integration targets in seconds and metres per second for one forward distance trajectory.</summary>
public sealed class DistanceTrajectorySettings
{
    public DistanceTrajectorySettings(double elapsedTimeAbsoluteToleranceSeconds, double velocityAbsoluteToleranceMetresPerSecond,
        int maximumDepth, int maximumWork)
    {
        TrajectoryValidation.Positive(elapsedTimeAbsoluteToleranceSeconds, nameof(elapsedTimeAbsoluteToleranceSeconds));
        TrajectoryValidation.Positive(velocityAbsoluteToleranceMetresPerSecond, nameof(velocityAbsoluteToleranceMetresPerSecond));
        ArgumentOutOfRangeException.ThrowIfLessThan(maximumDepth, 1);
        ArgumentOutOfRangeException.ThrowIfLessThan(maximumWork, 1);
        ElapsedTimeAbsoluteToleranceSeconds = elapsedTimeAbsoluteToleranceSeconds;
        VelocityAbsoluteToleranceMetresPerSecond = velocityAbsoluteToleranceMetresPerSecond;
        MaximumDepth = maximumDepth;
        MaximumWork = maximumWork;
    }

    public double ElapsedTimeAbsoluteToleranceSeconds { get; }
    public double VelocityAbsoluteToleranceMetresPerSecond { get; }
    /// <summary>Subdivision depth per original positivity panel and per quadrature call, starting at zero.</summary>
    public int MaximumDepth { get; }
    /// <summary>Total work per build or per independent trajectory query, including nested evaluations and positivity checks.</summary>
    public int MaximumWork { get; }
}

/// <summary>Estimated absolute integration and accumulation errors. These estimates are not certified error bounds.</summary>
public readonly record struct TrajectoryErrorEstimate
{
    public TrajectoryErrorEstimate(double elapsedTimeSeconds, double stationMetres, double velocityMetresPerSecond)
    {
        TrajectoryValidation.Nonnegative(elapsedTimeSeconds, nameof(elapsedTimeSeconds));
        TrajectoryValidation.Nonnegative(stationMetres, nameof(stationMetres));
        TrajectoryValidation.Nonnegative(velocityMetresPerSecond, nameof(velocityMetresPerSecond));
        ElapsedTimeSeconds = elapsedTimeSeconds;
        StationMetres = stationMetres;
        VelocityMetresPerSecond = velocityMetresPerSecond;
    }

    public double ElapsedTimeSeconds { get; }
    public double StationMetres { get; }
    public double VelocityMetresPerSecond { get; }
}

/// <summary>Inclusive acceleration enclosure in m/s² for a requested subinterval.</summary>
public readonly record struct AccelerationRange(double LowerMetresPerSecondSquared, double UpperMetresPerSecondSquared);

/// <summary>Supplies a conservative enclosure over every real station in one acceleration piece, or null if unavailable.</summary>
/// <remarks>Use inward limits at the two endpoints. The provider must be deterministic and account for roundoff
/// in the returned bounds. The bounds must also contain the evaluated acceleration values.</remarks>
public delegate AccelerationRange? AccelerationRangeProvider(LongitudinalAccelerationBinding binding,
    double startStationMetres, double endStationMetres);

public enum TrajectoryFailureCode
{
    ContinuityRequirementFailed,
    MissingAccelerationBounds,
    InvalidAccelerationBounds,
    NonPositiveSpeed,
    SpeedPositivityUnresolved,
    NumericalBudgetExceeded,
    NumericalRangeExceeded,
    EvaluationFailed
}

/// <summary>A trajectory failure with a coordinate interval in authored seconds or heartline metres.</summary>
public sealed class TrajectoryException : InvalidOperationException
{
    internal TrajectoryException(TrajectoryFailureCode code, string message, ProfileDomain domain,
        double startCoordinate, double endCoordinate, int workUsed, int? regionIndex = null,
        (double Lower, double Upper)? squaredSpeedEnclosure = null, TrajectoryErrorEstimate? estimatedAbsoluteError = null,
        IReadOnlyList<AccelerationJoinDiagnostic>? continuityFailures = null, Exception? innerException = null)
        : base(message, innerException)
    {
        Code = code;
        Domain = domain;
        StartCoordinate = startCoordinate;
        EndCoordinate = endCoordinate;
        WorkUsed = workUsed;
        RegionIndex = regionIndex;
        SquaredSpeedEnclosure = squaredSpeedEnclosure;
        EstimatedAbsoluteError = estimatedAbsoluteError;
        ContinuityFailures = continuityFailures is null
            ? Array.Empty<AccelerationJoinDiagnostic>() : Array.AsReadOnly(continuityFailures.ToArray());
    }

    public TrajectoryFailureCode Code { get; }
    public ProfileDomain Domain { get; }
    public double StartCoordinate { get; }
    public double EndCoordinate { get; }
    public int WorkUsed { get; }
    public int? RegionIndex { get; }
    /// <summary>Squared-speed enclosure in m²/s², when available.</summary>
    public (double Lower, double Upper)? SquaredSpeedEnclosure { get; }
    public TrajectoryErrorEstimate? EstimatedAbsoluteError { get; }
    public IReadOnlyList<AccelerationJoinDiagnostic> ContinuityFailures { get; }
}

/// <summary>Initial heartline station in metres and signed station velocity in metres per second, at profile start.</summary>
public readonly record struct TimeMotionInitialConditions(double HeartlineStationMetres, double HeartlineVelocityMetresPerSecond);

/// <summary>Predicted coordinate motion with elapsed time measured from the start of the profile.</summary>
public readonly record struct LongitudinalMotionSample(double ElapsedSeconds, double HeartlineStationMetres,
    double HeartlineVelocityMetresPerSecond, double LongitudinalAccelerationMetresPerSecondSquared,
    TrajectoryErrorEstimate EstimatedAbsoluteError);

internal static class TrajectoryValidation
{
    internal static void Positive(double value, string name)
    {
        if (!double.IsFinite(value) || value <= 0)
        { throw new ArgumentOutOfRangeException(name, "Supply a finite, strictly positive value."); }
    }

    internal static void Nonnegative(double value, string name)
    {
        if (!double.IsFinite(value) || value < 0)
        { throw new ArgumentOutOfRangeException(name, "Supply a finite, nonnegative value."); }
    }
}
