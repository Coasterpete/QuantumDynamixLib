using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.Linq;
using QuantumDynamix.Math;

namespace QuantumDynamix.Physics;

public enum ContactSolveFailure
{
    UnsupportedFeature, AmbiguousContact, GeometryUnresolved, InvalidPatch, MaterialSideMismatch,
    InconsistentInitialState, IndeterminateReactions, LinearSolve, ActiveSetLimit, PositionProjection,
    StepResolution, NumericalFailure
}

/// <summary>A failed query or step never returns separation or an advanced state as a substitute.</summary>
public sealed class ContactSolveException : InvalidOperationException
{
    public ContactSolveFailure Failure { get; }
    public string Stage { get; }
    public string? WheelId { get; }
    public string? PatchId { get; }
    public int Iterations { get; }
    public double Residual { get; }
    /// <summary>Ordered bindings for interpreting contact rows after the joint rows in a failed solve.</summary>
    public ReadOnlyCollection<WheelRailPair> InvolvedPairs { get; }
    internal string Explanation { get; }

    internal ContactSolveException(ContactSolveFailure failure, string stage, string message,
        string? wheelId = null, string? patchId = null, int iterations = 0, double residual = 0, Exception? inner = null,
        IEnumerable<WheelRailPair>? involvedPairs = null)
        : base($"{stage}: {message} Wheel={wheelId}; patch={patchId}; iterations={iterations}; residual={residual:E6}.", inner)
    {
        Failure = failure;
        Stage = stage;
        WheelId = wheelId;
        PatchId = patchId;
        Iterations = iterations;
        Residual = residual;
        Explanation = message;
        InvolvedPairs = Array.AsReadOnly(involvedPairs?.ToArray() ?? Array.Empty<WheelRailPair>());
    }
}

/// <summary>A local smooth contact branch. Normal points out of rail material toward the wheel.
/// Gap is signed metres; velocity is wheel material-point velocity (rail stationary), m/s.</summary>
public readonly record struct WheelRailContactSample(string WheelId, string PatchId, string RailId, int BodyIndex,
    double HeartlineStation, double ProfileParameter, Vector3d RailPoint, Vector3d WheelPoint,
    Vector3d Normal, double Gap, Vector3d RelativeVelocity, double NormalVelocity,
    double TangencyResidual, int Evaluations);

public sealed class WheelRailGeometryResult
{
    public ReadOnlyCollection<WheelRailContactSample> Contacts { get; }
    internal WheelRailGeometryResult(WheelRailContactSample[] contacts) => Contacts = Array.AsReadOnly(contacts);
}

/// <summary>Physical discrete impulse on the wheel body in N*s and its world angular impulse about its COM.
/// AverageForce is impulse/step in N, not an instantaneous impact force.</summary>
public readonly record struct WheelRailContactImpulse(WheelRailContactSample Geometry, double NormalImpulse,
    Vector3d WorldImpulse, Vector3d WorldAngularImpulse, Vector3d AverageForce, Vector3d AverageTorque);

/// <summary>Discrete world impulse on joint body A (N*s), with world COM angular impulses (N*m*s).</summary>
public readonly record struct SphericalJointImpulse(Vector3d ImpulseOnA, Vector3d AngularImpulseOnA, Vector3d AngularImpulseOnB)
{
    public Vector3d ImpulseOnB => -ImpulseOnA;
}

/// <summary>Residuals: gap m, joint slip and discrete feasibility m/s, negative impulse N*s;
/// complementarity is scaled, as is the linear backward error. Pivot is dimensionless.</summary>
public readonly record struct ContactStepDiagnostics(int ActiveSetIterations, int PositionProjectionIterations,
    double MinimumGap, double MaximumJointGap, double MaximumJointSlip,
    double MaximumVelocityViolation, double MaximumNegativeImpulse, double ScaledComplementarityResidual,
    double LinearSolveRelativeResidual, double MomentumRelativeResidual, double MinimumScaledPivot);

public sealed class ContactStepResult
{
    public MultibodyState State { get; }
    /// <summary>Geometry at the impulse solve (start configuration), not relocated endpoint impulses.</summary>
    public ReadOnlyCollection<WheelRailContactImpulse> ContactImpulses { get; }
    public ReadOnlyCollection<WheelRailContactSample> AcceptedContacts { get; }
    public ReadOnlyCollection<SphericalJointImpulse> JointImpulses { get; }
    public ReadOnlyCollection<RigidBodyLoads> AverageConstraintLoads { get; }
    /// <summary>Purely numerical endpoint position/orientation/velocity corrections; never physical loads.</summary>
    public ReadOnlyCollection<BodyProjectionCorrection> NumericalCorrections { get; }
    public ContactStepDiagnostics Diagnostics { get; }

    internal ContactStepResult(MultibodyState state, WheelRailContactImpulse[] contacts,
        WheelRailContactSample[] accepted, SphericalJointImpulse[] joints, RigidBodyLoads[] averageLoads,
        BodyProjectionCorrection[] corrections, ContactStepDiagnostics diagnostics)
    {
        State = state;
        ContactImpulses = Array.AsReadOnly(contacts);
        AcceptedContacts = Array.AsReadOnly(accepted);
        JointImpulses = Array.AsReadOnly(joints);
        AverageConstraintLoads = Array.AsReadOnly(averageLoads);
        NumericalCorrections = Array.AsReadOnly(corrections);
        Diagnostics = diagnostics;
    }
}
