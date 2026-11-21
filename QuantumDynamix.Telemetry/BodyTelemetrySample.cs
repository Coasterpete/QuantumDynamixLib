using System;
using QuantumDynamix.Math;
using QuantumDynamix.Physics;

namespace QuantumDynamix.Telemetry;

/// <summary>Center-of-mass telemetry for one body at an accepted constrained step endpoint.</summary>
/// <remarks>Use <see cref="FromAcceptedStep"/> to create a sample from a solver result.
/// The default struct value contains only zero-initialized fields.</remarks>
public readonly record struct BodyTelemetrySample
{
    /// <summary>Accepted simulation time, in seconds.</summary>
    public double AcceptedTimeSeconds { get; }

    /// <summary>Zero-based index in the simulation's body order.</summary>
    public int BodyIndex { get; }

    /// <summary>Center-of-mass position in world coordinates, in metres.</summary>
    public Vector3d WorldCenterOfMassPosition { get; }

    /// <summary>Magnitude of the center-of-mass velocity, in metres per second.</summary>
    public double Speed { get; }

    /// <summary>Instantaneous world linear acceleration at the accepted endpoint, in m/s².</summary>
    public Vector3d WorldLinearAcceleration { get; }

    private BodyTelemetrySample(double acceptedTimeSeconds, int bodyIndex,
        Vector3d worldCenterOfMassPosition, double speed, Vector3d worldLinearAcceleration)
    {
        AcceptedTimeSeconds = acceptedTimeSeconds;
        BodyIndex = bodyIndex;
        WorldCenterOfMassPosition = worldCenterOfMassPosition;
        Speed = speed;
        WorldLinearAcceleration = worldLinearAcceleration;
    }

    /// <summary>Samples one body using the accepted state and its instantaneous endpoint derivative.</summary>
    /// <param name="step">The completed constrained simulation step.</param>
    /// <param name="bodyIndex">The zero-based index of the body to sample.</param>
    /// <param name="acceptedTimeSeconds">The endpoint time supplied by the caller, in seconds.</param>
    public static BodyTelemetrySample FromAcceptedStep(
        ConstrainedStepResult step, int bodyIndex, double acceptedTimeSeconds)
    {
        ArgumentNullException.ThrowIfNull(step);
        if (bodyIndex < 0 || bodyIndex >= step.State.Bodies.Count)
        {
            throw new ArgumentOutOfRangeException(nameof(bodyIndex), "Index must identify a body in the accepted step.");
        }

        if (!double.IsFinite(acceptedTimeSeconds))
        {
            throw new ArgumentOutOfRangeException(nameof(acceptedTimeSeconds), "Endpoint time must be finite.");
        }

        RigidBodyState body = step.State.Bodies[bodyIndex];
        RigidBodyDerivative derivative = step.AcceptedDynamics.Derivatives[bodyIndex];
        return new BodyTelemetrySample(acceptedTimeSeconds, bodyIndex, body.Pose.Position,
            body.LinearVelocity.Length, derivative.LinearAcceleration);
    }
}
