using System;
using System.Collections.Generic;
using QuantumDynamix.Math;
using QuantumDynamix.Math.Functions;
using QuantumDynamix.Math.Timelines;
using QuantumDynamix.Splines.Fitting;

namespace QuantumDynamix.FVD;

public sealed class HeartlineReconstructionDiagnostics
{
    internal HeartlineReconstructionDiagnostics() { }

    public double ProvenForwardSpeedLowerBoundMetresPerSecond { get; internal init; }
    public double EstimatedPositionErrorMetres { get; internal init; }
    public double EstimatedFrameErrorRadians { get; internal init; }
    public double MaximumObservedCurvatureResidualPerMetre { get; internal init; }
    public double MaximumObservedUnbankedRollRateRadiansPerMetre { get; internal init; }
    public double MaximumEstimatedStationMappingErrorMetres { get; internal init; }
    public double MaximumEstimatedTimeMappingErrorSeconds { get; internal init; }
    public int WorkUsed { get; internal init; }

    public IReadOnlyList<(string ChannelId, double SourceStationMetres, RegionJoinDiagnostic Comparison)>
        AccelerationJoins { get; internal init; } = [];

    public IReadOnlyList<(double SourceStationMetres, RegionJoinDiagnostic Comparison)>
        BankJoins { get; internal init; } = [];
}

/// <summary>Solver and residual diagnostics for the latest candidate, with totals across the fit operation.</summary>
/// <remarks>Residuals are available when the latest candidate completed sampled forward verification.</remarks>
public sealed class HeartlineFitDiagnostics
{
    internal HeartlineFitDiagnostics() { }

    public FitDiagnostics? Solver { get; internal init; }
    public int RefinementPasses { get; internal init; }
    public int VerificationSamples { get; internal init; }
    public double? MaximumPositionErrorMetres { get; internal init; }
    public double? MaximumTangentErrorRadians { get; internal init; }
    public double? MaximumCurvatureErrorPerMetre { get; internal init; }
    public double? MaximumStationDriftMetres { get; internal init; }
    public double? LengthErrorMetres { get; internal init; }
    public Vector3d? MaximumCoordinateAccelerationErrorsMetresPerSecondSquared { get; internal init; }
    public (double ErrorRadians, double SourceStationMetres, EvaluationSide Side)?
        WorstBankedFrameError { get; internal init; }
    public (double SignedErrorRadians, double SourceStationMetres, EvaluationSide Side)?
        WorstBankRollError { get; internal init; }
}

public enum HeartlineFailureCode
{
    InvalidInput,
    ContinuityRequirementFailed,
    MissingAccelerationBounds,
    InvalidAccelerationBounds,
    NonPositiveSpeed,
    SpeedBelowMinimum,
    SpeedPositivityUnresolved,
    SourceTrajectoryFailed,
    MotionMappingFailed,
    EvaluationFailed,
    NumericalRangeExceeded,
    NumericalBudgetExceeded,
    SplineSolveFailed,
    SplineRegularityUnresolved,
    FitToleranceExceeded
}

/// <summary>A reconstruction or fit failure. Source intervals use local numerical metres.</summary>
public sealed record HeartlineFailure(
    HeartlineFailureCode Code,
    string Message,
    (ProfileDomain Domain, double Start, double End)? AuthoredInterval,
    (double StartMetres, double EndMetres)? SourceStationInterval,
    int? RegionIndex,
    string? ChannelId,
    EvaluationSide? Side,
    (string Quantity, string Unit, double Observed, double Limit)? Measurement);

public sealed class HeartlineReconstructionException : InvalidOperationException
{
    internal HeartlineReconstructionException(HeartlineFailure failure, int workUsed, Exception? inner = null)
        : base(failure.Message, inner)
    {
        Failure = failure;
        WorkUsed = workUsed;
    }

    public HeartlineFailure Failure { get; }
    public int WorkUsed { get; }
}

internal sealed class ReconstructionWork(int maximumWork)
{
    internal int Used { get; private set; }
    internal int Remaining => maximumWork - Used;
    internal double StationError { get; set; }
    internal double TimeError { get; set; }

    internal void Charge(double? station = null, int count = 1)
    {
        if (count > Remaining)
        {
            throw Fail(HeartlineFailureCode.NumericalBudgetExceeded, "The reconstruction work limit was exhausted.",
                station is double s ? (s, s) : null);
        }
        Used += count;
    }

    internal HeartlineReconstructionException Fail(HeartlineFailureCode code, string message,
        (double Start, double End)? station = null, (ProfileDomain Domain, double Start, double End)? authored = null,
        int? region = null, string? channel = null, EvaluationSide? side = null,
        (string Quantity, string Unit, double Observed, double Limit)? measurement = null, Exception? inner = null) =>
        new(new(code, message, authored, station, region, channel, side, measurement), Used, inner);
}
