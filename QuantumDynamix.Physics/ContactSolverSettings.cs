using System;

namespace QuantumDynamix.Physics;

/// <summary>Bounded search controls, not certified global collision-detection bounds or engineering tolerances.</summary>
public sealed class ContactGeometrySettings
{
    public double PositionTolerance { get; }
    public double TangencyTolerance { get; }
    public double BoundaryFraction { get; }
    public double DifferenceStep { get; }
    public int SeedsPerAxis { get; }
    public int MaximumIterations { get; }
    public int MaximumEvaluations { get; }

    public ContactGeometrySettings(double positionTolerance = 1e-9, double tangencyTolerance = 1e-8,
        double boundaryFraction = 1e-7, double differenceStep = 1e-4,
        int seedsPerAxis = 3, int maximumIterations = 40, int maximumEvaluations = 20000)
    {
        ContactChecks.Positive(positionTolerance, nameof(positionTolerance));
        ContactChecks.Positive(tangencyTolerance, nameof(tangencyTolerance));
        ContactChecks.Positive(boundaryFraction, nameof(boundaryFraction));
        ContactChecks.Positive(differenceStep, nameof(differenceStep));
        if (tangencyTolerance >= 0.1 || boundaryFraction >= 0.1 || differenceStep >= 0.1
            || seedsPerAxis < 3 || seedsPerAxis > 32 || maximumIterations < 1 || maximumEvaluations < 1)
        { throw new ArgumentOutOfRangeException(nameof(seedsPerAxis), "Invalid search controls."); }
        PositionTolerance = positionTolerance;
        TangencyTolerance = tangencyTolerance;
        BoundaryFraction = boundaryFraction;
        DifferenceStep = differenceStep;
        SeedsPerAxis = seedsPerAxis;
        MaximumIterations = maximumIterations;
        MaximumEvaluations = maximumEvaluations;
    }
}

/// <summary>First-order discrete contact controls. Movement/correction limits are explicitly supplied
/// by the caller, in metres and radians; they do not guarantee continuous collision detection.</summary>
public sealed class ContactSolverSettings
{
    public ContactGeometrySettings Geometry { get; }
    public ConstrainedSolverSettings Joints { get; }
    public double MaximumPointTravel { get; }
    public double MaximumAngularStep { get; }
    public double MaximumPositionCorrection { get; }
    public double MaximumOrientationCorrection { get; }
    public double PositionTolerance { get; }
    public double VelocityTolerance { get; }
    public double ImpulseTolerance { get; }
    public int MaximumActiveSetIterations { get; }
    public int MaximumProjectionIterations { get; }
    public int TravelSamples { get; }

    public ContactSolverSettings(double maximumPointTravel, double maximumAngularStep,
        double maximumPositionCorrection, double maximumOrientationCorrection,
        ContactGeometrySettings? geometry = null, ConstrainedSolverSettings? joints = null,
        double positionTolerance = 1e-8, double velocityTolerance = 1e-8, double impulseTolerance = 1e-10,
        int maximumActiveSetIterations = 128, int maximumProjectionIterations = 12, int travelSamples = 4)
    {
        ContactChecks.Positive(maximumPointTravel, nameof(maximumPointTravel));
        ContactChecks.Positive(maximumAngularStep, nameof(maximumAngularStep));
        ContactChecks.Positive(maximumPositionCorrection, nameof(maximumPositionCorrection));
        ContactChecks.Positive(maximumOrientationCorrection, nameof(maximumOrientationCorrection));
        ContactChecks.Positive(positionTolerance, nameof(positionTolerance));
        ContactChecks.Positive(velocityTolerance, nameof(velocityTolerance));
        ContactChecks.Positive(impulseTolerance, nameof(impulseTolerance));
        if (maximumActiveSetIterations < 1 || maximumProjectionIterations < 1 || travelSamples < 2 || travelSamples > 1024)
        { throw new ArgumentOutOfRangeException(nameof(maximumActiveSetIterations)); }
        Geometry = geometry ?? new();
        Joints = joints ?? ConstrainedSolverSettings.Default;
        if (positionTolerance < Geometry.PositionTolerance)
        { throw new ArgumentException("Contact position tolerance must cover the geometry search tolerance."); }
        MaximumPointTravel = maximumPointTravel;
        MaximumAngularStep = maximumAngularStep;
        MaximumPositionCorrection = maximumPositionCorrection;
        MaximumOrientationCorrection = maximumOrientationCorrection;
        PositionTolerance = positionTolerance;
        VelocityTolerance = velocityTolerance;
        ImpulseTolerance = impulseTolerance;
        MaximumActiveSetIterations = maximumActiveSetIterations;
        MaximumProjectionIterations = maximumProjectionIterations;
        TravelSamples = travelSamples;
    }
}
