using System;
using System.Collections.Generic;
using QuantumDynamix.Math;

namespace QuantumDynamix.Physics;

/// <summary>Acceleration-level bilateral Newton-Euler dynamics, independent of a time integrator.</summary>
public static class ConstrainedMultibodyDynamics
{
    /// <summary>Evaluates instantaneous reactions and derivatives at the supplied snapshot.</summary>
    /// <remarks>This evaluates a smooth acceleration-level extension off the constraint manifold
    /// within each revolute joint's supported directed alignment chart;
    /// it reports position/velocity errors and does not assemble or advance the state. The integrator
    /// requires consistent initial data and accepts endpoints only within its gap/slip tolerances.</remarks>
    public static ConstrainedDynamicsResult Evaluate(MultibodySystem system, MultibodyState state,
        IReadOnlyList<RigidBodyLoads> loads, ConstrainedSolverSettings? settings = null)
    {
        ArgumentNullException.ThrowIfNull(system);
        system.ValidateState(state);
        ArgumentNullException.ThrowIfNull(loads);
        if (loads.Count != system.Bodies.Count)
        {
            throw new ArgumentException("Supply one external load per body.", nameof(loads));
        }

        settings ??= ConstrainedSolverSettings.Default;
        try
        {
            return EvaluateCore(system, state, loads, settings);
        }
        catch (ArithmeticException error)
        {
            throw new ConstrainedSolveException(ConstrainedSolveFailure.NumericalFailure, "Constraint forces", 0,
                double.PositiveInfinity, "Arithmetic is outside the supported finite range.", error);
        }
        catch (ArgumentOutOfRangeException error)
        {
            throw new ConstrainedSolveException(ConstrainedSolveFailure.NumericalFailure, "Constraint forces", 0,
                double.PositiveInfinity, "A derived numerical quantity is outside the supported finite range.", error);
        }
    }

    private static ConstrainedDynamicsResult EvaluateCore(MultibodySystem system, MultibodyState state,
        IReadOnlyList<RigidBodyLoads> loads, ConstrainedSolverSettings settings)
    {
        RigidBodyDerivative[] derivatives = new RigidBodyDerivative[system.Bodies.Count];
        double[] freeAcceleration = new double[6 * system.Bodies.Count];
        for (int i = 0; i < derivatives.Length; i++)
        {
            derivatives[i] = RigidBodyDynamics.EvaluateDerivative(system.Bodies[i], state.Bodies[i], loads[i]);
            ConstraintAssembly.Put(freeAcceleration, 6 * i, derivatives[i].LinearAcceleration);
            ConstraintAssembly.Put(freeAcceleration, 6 * i + 3, derivatives[i].BodyAngularAcceleration);
        }

        ConstraintAssembly assembly = new(system, state, settings, "Constraint forces");
        double[] rhs = ConstrainedLinearAlgebra.Multiply(assembly.Jacobian, freeAcceleration);
        for (int row = 0; row < rhs.Length; row++)
        {
            rhs[row] = -(rhs[row] + assembly.Bias[row]);
        }

        double[] multipliers = assembly.Factor.Solve(rhs, out double relativeResidual);
        RigidBodyLoads[] constraintLoads = new RigidBodyLoads[system.Bodies.Count];
        SphericalJointReaction[] reactions = new SphericalJointReaction[system.Joints.Count];
        for (int j = 0; j < reactions.Length; j++)
        {
            SphericalJoint joint = system.Joints[j];
            Vector3d force = ConstraintAssembly.Get(multipliers, 3 * j);
            Vector3d armA = state.Bodies[joint.BodyA].Pose.TransformDirection(joint.LocalAnchorA);
            Vector3d armB = state.Bodies[joint.BodyB].Pose.TransformDirection(joint.LocalAnchorB);
            reactions[j] = new(force, Vector3d.Cross(armA, force), Vector3d.Cross(armB, -force));
            constraintLoads[joint.BodyA] += new RigidBodyLoads(force, reactions[j].TorqueOnA);
            constraintLoads[joint.BodyB] += new RigidBodyLoads(-force, reactions[j].TorqueOnB);
        }
        RevoluteJointReaction[] revoluteReactions = new RevoluteJointReaction[system.RevoluteJoints.Count];
        for (int j = 0; j < revoluteReactions.Length; j++)
        {
            RevoluteJoint joint = system.RevoluteJoints[j];
            RevoluteJointReaction reaction = assembly.RevoluteReaction(system, state, j, multipliers);
            revoluteReactions[j] = reaction;
            constraintLoads[joint.BodyA] += new RigidBodyLoads(reaction.ForceOnA, reaction.TorqueOnA);
            constraintLoads[joint.BodyB] += new RigidBodyLoads(reaction.ForceOnB, reaction.TorqueOnB);
        }

        double[] acceleration = new double[freeAcceleration.Length];
        for (int i = 0; i < derivatives.Length; i++)
        {
            derivatives[i] = RigidBodyDynamics.EvaluateDerivative(system.Bodies[i], state.Bodies[i], loads[i] + constraintLoads[i]);
            ConstraintAssembly.Put(acceleration, 6 * i, derivatives[i].LinearAcceleration);
            ConstraintAssembly.Put(acceleration, 6 * i + 3, derivatives[i].BodyAngularAcceleration);
        }

        double[] accelerationError = ConstrainedLinearAlgebra.Multiply(assembly.Jacobian, acceleration);
        for (int i = 0; i < accelerationError.Length; i++)
        {
            accelerationError[i] += assembly.Bias[i];
        }

        return new(derivatives, constraintLoads, reactions, revoluteReactions, new(
            assembly.MaximumLinearError(assembly.PositionError),
            assembly.MaximumLinearError(assembly.VelocityError),
            assembly.MaximumLinearError(accelerationError), relativeResidual, assembly.Factor.MinimumPivot)
        {
            MaximumAngularPositionError = assembly.MaximumAlignmentError,
            MaximumAngularVelocityError = assembly.MaximumAngularError(assembly.VelocityError),
            MaximumAngularAccelerationError = assembly.MaximumAngularError(accelerationError)
        });
    }
}
