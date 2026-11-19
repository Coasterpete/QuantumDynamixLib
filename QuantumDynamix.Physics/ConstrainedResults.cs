using System;
using System.Collections.ObjectModel;
using QuantumDynamix.Math;

namespace QuantumDynamix.Physics;

public enum ConstrainedSolveFailure
{
    SingularConstraints,
    LinearSolve,
    NonlinearSolve,
    PositionProjection,
    VelocityProjection,
    InconsistentInitialState,
    NumericalFailure,
    UnsupportedJointConfiguration
}

/// <summary>A failed calculation never returns a partially advanced state.</summary>
public sealed class ConstrainedSolveException : InvalidOperationException
{
    public ConstrainedSolveFailure Failure { get; }
    public string Stage { get; }
    public int Iterations { get; }
    /// <summary>Residual in the stage's stated units; a singular solve reports its scaled pivot.</summary>
    public double Residual { get; }

    internal ConstrainedSolveException(ConstrainedSolveFailure failure, string stage, int iterations,
        double residual, string explanation, Exception? inner = null)
        : base($"{stage}: {explanation} Iterations={iterations}; residual={residual:E6}.", inner)
    {
        Failure = failure;
        Stage = stage;
        Iterations = iterations;
        Residual = residual;
    }
}

/// <summary>Maximum joint Euclidean errors (m, m/s, m/s^2), and dimensionless linear-solve diagnostics.</summary>
public readonly record struct ConstraintDiagnostics(double MaximumPositionError, double MaximumVelocityError,
    double MaximumAccelerationError, double LinearSolveRelativeResidual, double MinimumScaledPivot)
{
    /// <summary>Maximum actual directed-axle misalignment (rad).</summary>
    public double MaximumAngularPositionError { get; internal init; }
    /// <summary>Maximum norm of a hinge's two alignment-row rates (1/s; forbidden rad/s at alignment).</summary>
    public double MaximumAngularVelocityError { get; internal init; }
    /// <summary>Maximum norm of the two alignment-row acceleration residuals (1/s^2).</summary>
    public double MaximumAngularAccelerationError { get; internal init; }
}

/// <summary>Instantaneous world reaction at the evaluated state: force on A (N), and COM torques (N*m).</summary>
public readonly record struct SphericalJointReaction(Vector3d ForceOnA, Vector3d TorqueOnA, Vector3d TorqueOnB)
{
    public Vector3d ForceOnB => -ForceOnA;
}

public sealed class ConstrainedDynamicsResult
{
    public ReadOnlyCollection<RigidBodyDerivative> Derivatives { get; }
    public ReadOnlyCollection<RigidBodyLoads> ConstraintLoads { get; }
    public ReadOnlyCollection<SphericalJointReaction> JointReactions { get; }
    public ReadOnlyCollection<RevoluteJointReaction> RevoluteJointReactions { get; }
    public ConstraintDiagnostics Diagnostics { get; }

    internal ConstrainedDynamicsResult(RigidBodyDerivative[] derivatives, RigidBodyLoads[] loads,
        SphericalJointReaction[] reactions, RevoluteJointReaction[] revoluteReactions, ConstraintDiagnostics diagnostics)
    {
        Derivatives = Array.AsReadOnly(derivatives);
        ConstraintLoads = Array.AsReadOnly(loads);
        JointReactions = Array.AsReadOnly(reactions);
        RevoluteJointReactions = Array.AsReadOnly(revoluteReactions);
        Diagnostics = diagnostics;
    }
}

/// <summary>Numerical changes from the midpoint endpoint to the projected endpoint; never physical loads.</summary>
/// <param name="PositionChange">World displacement (m).</param>
/// <param name="OrientationCorrection">Right-multiplying unit quaternion: q_after=q_before*correction.</param>
/// <param name="LinearVelocityChange">World velocity change (m/s).</param>
/// <param name="BodyAngularVelocityChange">Change of body-coordinate angular velocity components (rad/s).</param>
public readonly record struct BodyProjectionCorrection(Vector3d PositionChange, Quaterniond OrientationCorrection,
    Vector3d LinearVelocityChange, Vector3d BodyAngularVelocityChange);

/// <summary>Newton residual is dimensionless; pre-projection gap/slip are in m and m/s.</summary>
public readonly record struct ConstrainedStepDiagnostics(int MidpointIterations, double MidpointScaledResidual,
    int PositionProjectionIterations, int VelocityProjectionIterations,
    double BeforeProjectionPositionError, double BeforeProjectionVelocityError)
{
    /// <summary>Actual axle misalignment before numerical projection (rad).</summary>
    public double BeforeProjectionAngularPositionError { get; internal init; }
    /// <summary>Alignment-row rate norm before numerical projection (1/s).</summary>
    public double BeforeProjectionAngularVelocityError { get; internal init; }
}

public sealed class ConstrainedStepResult
{
    internal MultibodyState UnprojectedState { get; }
    public MultibodyState State { get; }
    /// <summary>Recomputed instantaneous reactions/derivatives at the accepted endpoint, not timestep averages.</summary>
    public ConstrainedDynamicsResult AcceptedDynamics { get; }
    public ReadOnlyCollection<BodyProjectionCorrection> ProjectionCorrections { get; }
    public ConstrainedStepDiagnostics Diagnostics { get; }

    internal ConstrainedStepResult(MultibodyState state, ConstrainedDynamicsResult dynamics,
        BodyProjectionCorrection[] corrections, ConstrainedStepDiagnostics diagnostics, MultibodyState unprojectedState)
    {
        State = state;
        AcceptedDynamics = dynamics;
        ProjectionCorrections = Array.AsReadOnly(corrections);
        Diagnostics = diagnostics;
        UnprojectedState = unprojectedState;
    }
}
