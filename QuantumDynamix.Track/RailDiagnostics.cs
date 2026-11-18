using System;

namespace QuantumDynamix.Track;

public enum RailDiagnosticCode
{
    UnavailableDerivative, SingularCenterline, SingularSectionFrame, DegenerateProfile, SingularSweep,
    PositionDiscontinuity, StationDerivativeDiscontinuity, StationSecondDerivativeDiscontinuity, TangentDiscontinuity, CurvatureDiscontinuity,
    SectionOrientationDiscontinuity, SectionRateDiscontinuity, SpacingMismatch, CoincidentRails,
    NumericalFailure, SamplingLimit, ProfileDiscontinuity, IncompatibleArrangement
}

/// <summary>A located failure or unresolved validation. Residual units are stated in Message.</summary>
public sealed record RailDiagnostic(RailDiagnosticCode Code, string? RailId, double HeartlineStation,
    string Message, double? Residual = null, string? RelatedRailId = null);

/// <summary>A geometric query cannot supply its requested result; the input definition is never repaired.</summary>
public sealed class RailGeometryException : InvalidOperationException
{
    public RailDiagnostic Diagnostic { get; }
    internal RailGeometryException(RailDiagnostic diagnostic) : base(diagnostic.Message) { Diagnostic = diagnostic; }
}

/// <summary>Explicit caller acceptance controls. No physical engineering tolerance is assumed.
/// Sampling controls bound an adaptive inspection, not a continuous-domain certificate.</summary>
public sealed class RailValidationSettings
{
    public double PositionTolerance { get; }
    public double AngularTolerance { get; }
    public double CurvatureTolerance { get; }
    public double StationDerivativeTolerance { get; }
    public double MinimumSpeedRatio { get; }
    public double MinimumFrameProjection { get; }
    public int SamplesPerSpan { get; }
    public int MaximumDepth { get; }
    public int MaximumEvaluations { get; }
    public double SamplingRelativeTolerance { get; }
    public double SamplingAngularTolerance { get; }

    public RailValidationSettings(double positionTolerance, double angularTolerance, double curvatureTolerance,
        double stationDerivativeTolerance, double minimumSpeedRatio, double minimumFrameProjection,
        int samplesPerSpan = 8, int maximumDepth = 8, int maximumEvaluations = 100000,
        double samplingRelativeTolerance = 0.05, double samplingAngularTolerance = 0.1)
    {
        foreach (double value in new[] { positionTolerance, angularTolerance, curvatureTolerance, stationDerivativeTolerance,
            minimumFrameProjection, samplingRelativeTolerance, samplingAngularTolerance })
        {
            if (!double.IsFinite(value) || value <= 0) { throw new ArgumentOutOfRangeException(nameof(positionTolerance), "Tolerances must be positive and finite."); }
        }
        if (!double.IsFinite(minimumSpeedRatio) || minimumSpeedRatio < 0 || minimumFrameProjection >= 1
            || angularTolerance >= System.Math.PI || samplingAngularTolerance >= System.Math.PI
            || samplesPerSpan < 1 || maximumDepth < 0 || maximumEvaluations < 3)
        { throw new ArgumentOutOfRangeException(nameof(minimumSpeedRatio), "Invalid geometric or sampling control."); }
        PositionTolerance = positionTolerance;
        AngularTolerance = angularTolerance;
        CurvatureTolerance = curvatureTolerance;
        StationDerivativeTolerance = stationDerivativeTolerance;
        MinimumSpeedRatio = minimumSpeedRatio;
        MinimumFrameProjection = minimumFrameProjection;
        SamplesPerSpan = samplesPerSpan;
        MaximumDepth = maximumDepth;
        MaximumEvaluations = maximumEvaluations;
        SamplingRelativeTolerance = samplingRelativeTolerance;
        SamplingAngularTolerance = samplingAngularTolerance;
    }
}
