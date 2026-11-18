using System;
using QuantumDynamix.Geometry;
using QuantumDynamix.Math;

namespace QuantumDynamix.Physics;

/// <summary>Fixed-step classical RK4 for one unconstrained body, with quaternion normalization.</summary>
public static class RigidBodyIntegrator
{
    /// <summary>Advances one positive timestep in seconds using four load evaluations.</summary>
    /// <param name="evaluateLoads">Receives each stage's time and physical state, including a normalized orientation.
    /// The function should be deterministic and have no side effects; midpoint times occur twice with different states.</param>
    /// <remarks>Raw quaternion stage sums are retained for RK4 arithmetic; normalized stage views are used by
    /// the equations and load evaluator. This is RK4 on a smooth normalized extension of the unit-quaternion ODE,
    /// followed by projection of the accepted orientation. It is fourth order for smooth loads, but not symplectic
    /// or exactly energy-conserving. No constraints, hidden gravity, or default timestep are supplied.</remarks>
    public static RigidBodyState Step(
        RigidBodyMassProperties properties,
        RigidBodyState state,
        double time,
        double step,
        Func<double, RigidBodyState, RigidBodyLoads> evaluateLoads)
    {
        ArgumentNullException.ThrowIfNull(properties);
        ArgumentNullException.ThrowIfNull(evaluateLoads);
        state.Validate(nameof(state));
        if (!double.IsFinite(time))
        {
            throw new ArgumentOutOfRangeException(nameof(time), "Time must be finite.");
        }

        double halfStep = step / 2;
        double midpoint = time + halfStep;
        double endpoint = time + step;
        if (!double.IsFinite(step) || step <= 0 || !double.IsFinite(endpoint)
            || !(midpoint > time && midpoint < endpoint))
        {
            throw new ArgumentOutOfRangeException(nameof(step),
                "Step must be finite, positive, and resolve distinct finite RK4 stage times.");
        }

        Stage initial = new(state.Pose.Position, state.Pose.Orientation,
            state.LinearVelocity, state.BodyAngularVelocity);

        RigidBodyDerivative k1 = Evaluate(initial, time);
        RigidBodyDerivative k2 = Evaluate(initial.Advance(k1, halfStep), midpoint);
        RigidBodyDerivative k3 = Evaluate(initial.Advance(k2, halfStep), midpoint);
        RigidBodyDerivative k4 = Evaluate(initial.Advance(k3, step), endpoint);

        // Normalize only when constructing physical views; do not normalize quaternion derivatives.
        double sixthStep = step / 6;
        return new Stage(
            initial.Position + Weighted(k1.PositionRate, k2.PositionRate, k3.PositionRate, k4.PositionRate),
            initial.Orientation + k1.OrientationRate * sixthStep + k2.OrientationRate * (step / 3)
                + k3.OrientationRate * (step / 3) + k4.OrientationRate * sixthStep,
            initial.Velocity + Weighted(k1.LinearAcceleration, k2.LinearAcceleration, k3.LinearAcceleration, k4.LinearAcceleration),
            initial.Omega + Weighted(k1.BodyAngularAcceleration, k2.BodyAngularAcceleration, k3.BodyAngularAcceleration, k4.BodyAngularAcceleration))
            .ToPhysicalState();

        RigidBodyDerivative Evaluate(Stage stage, double stageTime)
        {
            RigidBodyState stageState = stage.ToPhysicalState();
            RigidBodyLoads loads = evaluateLoads(stageTime, stageState);
            return RigidBodyDynamics.EvaluateDerivative(properties, stageState, loads);
        }

        Vector3d Weighted(Vector3d first, Vector3d second, Vector3d third, Vector3d fourth) =>
            first * sixthStep + second * (step / 3) + third * (step / 3) + fourth * sixthStep;
    }

    private readonly record struct Stage(Vector3d Position, Quaterniond Orientation, Vector3d Velocity, Vector3d Omega)
    {
        internal Stage Advance(RigidBodyDerivative derivative, double duration) => new(
            Position + derivative.PositionRate * duration,
            Orientation + derivative.OrientationRate * duration,
            Velocity + derivative.LinearAcceleration * duration,
            Omega + derivative.BodyAngularAcceleration * duration);

        internal RigidBodyState ToPhysicalState() => new(new RigidPose3d(Position, Orientation), Velocity, Omega);
    }
}
