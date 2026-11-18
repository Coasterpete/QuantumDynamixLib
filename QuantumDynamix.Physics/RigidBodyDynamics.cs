using System;
using QuantumDynamix.Math;

namespace QuantumDynamix.Physics;

/// <summary>Unconstrained Newton-Euler equations, independent of any time-stepping method.</summary>
public static class RigidBodyDynamics
{
    public static RigidBodyDerivative EvaluateDerivative(
        RigidBodyMassProperties properties, RigidBodyState state, RigidBodyLoads loads)
    {
        ArgumentNullException.ThrowIfNull(properties);
        state.Validate(nameof(state));

        Vector3d omega = state.BodyAngularVelocity;
        Vector3d bodyMomentum = properties.BodyInertia * omega;
        Vector3d bodyTorque = state.Pose.InverseTransformDirection(loads.Torque);

        // Euler's equation in the body frame: I*omega_dot = torque - omega x (I*omega).
        Vector3d angularAcceleration = properties.SolveBodyInertia(
            bodyTorque - Vector3d.Cross(omega, bodyMomentum));

        // Body-to-world Hamilton quaternion: q_dot = (1/2)*q*(0,omega_body).
        Quaterniond spin = new(0, omega.X, omega.Y, omega.Z);
        Quaterniond orientationRate = (state.Pose.Orientation * spin) * 0.5;
        return new(state.LinearVelocity, orientationRate, loads.Force / properties.Mass, angularAcceleration);
    }

    /// <summary>World angular momentum about the center of mass (kg*m^2/s).</summary>
    public static Vector3d AngularMomentum(RigidBodyMassProperties properties, RigidBodyState state)
    {
        ArgumentNullException.ThrowIfNull(properties);
        state.Validate(nameof(state));
        return state.Pose.TransformDirection(properties.BodyInertia * state.BodyAngularVelocity);
    }

    /// <summary>Translational plus rotational kinetic energy (J); excludes potential energy.</summary>
    public static double KineticEnergy(RigidBodyMassProperties properties, RigidBodyState state)
    {
        ArgumentNullException.ThrowIfNull(properties);
        state.Validate(nameof(state));
        double energy = 0.5 * properties.Mass * Vector3d.Dot(state.LinearVelocity, state.LinearVelocity)
            + 0.5 * Vector3d.Dot(state.BodyAngularVelocity, properties.BodyInertia * state.BodyAngularVelocity);
        if (!double.IsFinite(energy))
        {
            throw new ArithmeticException("Kinetic energy exceeds the finite double range.");
        }

        return energy;
    }
}
