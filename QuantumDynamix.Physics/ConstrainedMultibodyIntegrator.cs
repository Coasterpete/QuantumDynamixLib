using System;
using System.Collections.Generic;
using QuantumDynamix.Geometry;
using QuantumDynamix.Math;

namespace QuantumDynamix.Physics;

/// <summary>Fixed-step projected implicit midpoint for coupled bilateral rigid-body dynamics.</summary>
/// <remarks>Second order for smooth loads, full-rank constraints, and sufficiently converged solves.
/// Ordinary endpoint projection is not symplectic and does not guarantee energy or momentum conservation.
/// No default step, gravity, compliance, or automatic step retry is supplied.</remarks>
public static class ConstrainedMultibodyIntegrator
{
    /// <summary>Advances the complete system or throws without modifying the input snapshot.</summary>
    /// <param name="evaluateLoads">External world forces and COM torques for every body. Receives normalized
    /// physical poses, including off-manifold midpoint trials. Must be deterministic and side-effect free;
    /// Newton and finite differences can evaluate the same time many times at different states.</param>
    public static ConstrainedStepResult Step(MultibodySystem system, MultibodyState state,
        double time, double step, Func<double, MultibodyState, IReadOnlyList<RigidBodyLoads>> evaluateLoads,
        ConstrainedSolverSettings? settings = null)
    {
        ArgumentNullException.ThrowIfNull(system);
        system.ValidateState(state);
        ArgumentNullException.ThrowIfNull(evaluateLoads);
        double midpoint = time + step / 2;
        double endpoint = time + step;
        if (!double.IsFinite(time) || !double.IsFinite(step) || step <= 0 || !double.IsFinite(endpoint)
            || !(midpoint > time && midpoint < endpoint))
        {
            throw new ArgumentOutOfRangeException(nameof(step), "Resolve distinct finite start, midpoint, and endpoint times with a positive step.");
        }

        settings ??= ConstrainedSolverSettings.Default;
        try
        {
            return StepCore(system, state, time, step, midpoint, endpoint, evaluateLoads, settings);
        }
        catch (ArithmeticException error)
        {
            throw new ConstrainedSolveException(ConstrainedSolveFailure.NumericalFailure, "Constrained step", 0,
                double.PositiveInfinity, "Arithmetic is outside the supported finite range.", error);
        }
        catch (ArgumentOutOfRangeException error)
        {
            throw new ConstrainedSolveException(ConstrainedSolveFailure.NumericalFailure, "Constrained step", 0,
                double.PositiveInfinity, "A numerical calculation or load callback produced an invalid value.", error);
        }
    }

    private static ConstrainedStepResult StepCore(MultibodySystem system, MultibodyState state, double time,
        double step, double midpoint, double endpoint,
        Func<double, MultibodyState, IReadOnlyList<RigidBodyLoads>> evaluateLoads, ConstrainedSolverSettings settings)
    {
        ConstraintAssembly initialAssembly = new(system, state, settings, "Initial constraints");
        double gap = initialAssembly.MaximumLinearError(initialAssembly.PositionError);
        double slip = initialAssembly.MaximumLinearError(initialAssembly.VelocityError);
        if (initialAssembly.PositionNorm(settings) > 1 || initialAssembly.VelocityNorm(settings) > 1)
        {
            throw new ConstrainedSolveException(ConstrainedSolveFailure.InconsistentInitialState, "Initial constraints", 0,
                System.Math.Max(initialAssembly.PositionNorm(settings), initialAssembly.VelocityNorm(settings)),
                $"Initial gap={gap:E6} m, slip={slip:E6} m/s, axle angle={initialAssembly.MaximumAlignmentError:E6} rad, "
                + $"angular row rate={initialAssembly.MaximumAngularError(initialAssembly.VelocityError):E6} 1/s. Supply assembled initial data; residual is tolerance-scaled.");
        }

        ConstrainedDynamicsResult initial = ConstrainedMultibodyDynamics.Evaluate(system, state,
            evaluateLoads(time, state), settings);
        int dimension = 6 * state.Bodies.Count;
        double[] scales = new double[dimension];
        double[] z = new double[dimension];
        for (int body = 0; body < state.Bodies.Count; body++)
        {
            RigidBodyState bodyState = state.Bodies[body];
            RigidBodyDerivative derivative = initial.Derivatives[body];
            double linearScale = System.Math.Max(settings.LinearVelocityScale,
                System.Math.Max(bodyState.LinearVelocity.Length, step * derivative.LinearAcceleration.Length));
            double angularScale = System.Math.Max(settings.AngularVelocityScale,
                System.Math.Max(bodyState.BodyAngularVelocity.Length, step * derivative.BodyAngularAcceleration.Length));
            for (int axis = 0; axis < 3; axis++)
            {
                scales[6 * body + axis] = linearScale;
                scales[6 * body + 3 + axis] = angularScale;
            }

            ConstraintAssembly.Put(z, 6 * body, derivative.LinearAcceleration * (step / linearScale));
            ConstraintAssembly.Put(z, 6 * body + 3, derivative.BodyAngularAcceleration * (step / angularScale));
        }

        double residualNorm = 0;
        int iterations;
        for (iterations = 0; ; iterations++)
        {
            double[] residual = Residual(z);
            residualNorm = ConstrainedLinearAlgebra.MaxAbs(residual);
            if (residualNorm <= settings.NonlinearTolerance)
            {
                break;
            }

            if (iterations == settings.MaxNonlinearIterations)
            {
                throw NonlinearFailure(iterations, residualNorm, "Iteration limit reached; dimensionless velocity residual exceeds tolerance.");
            }

            double[,] jacobian = new double[dimension, dimension];
            for (int column = 0; column < dimension; column++)
            {
                double difference = settings.FiniteDifferenceStep * System.Math.Max(1, System.Math.Abs(z[column]));
                double[] plus = (double[])z.Clone();
                double[] minus = (double[])z.Clone();
                plus[column] += difference;
                minus[column] -= difference;
                double actualDifference = plus[column] - minus[column];
                if (!double.IsFinite(actualDifference) || actualDifference <= 0)
                {
                    throw NonlinearFailure(iterations, residualNorm, "Finite-difference perturbation cannot be resolved.");
                }

                double[] high = Residual(plus);
                double[] low = Residual(minus);
                for (int row = 0; row < dimension; row++)
                {
                    jacobian[row, column] = (high[row] - low[row]) / actualDifference;
                }
            }

            double[] rhs = new double[dimension];
            for (int row = 0; row < dimension; row++)
            {
                rhs[row] = -residual[row];
            }

            double[] correction = ConstrainedLinearAlgebra.SolveNewton(jacobian, rhs, settings, iterations);
            bool improved = false;
            double fraction = 1;
            for (int trial = 0; trial < settings.MaxLineSearchIterations; trial++, fraction /= 2)
            {
                double[] candidate = new double[dimension];
                for (int row = 0; row < dimension; row++)
                {
                    candidate[row] = z[row] + fraction * correction[row];
                }

                double norm = ConstrainedLinearAlgebra.MaxAbs(Residual(candidate));
                if (norm <= settings.NonlinearTolerance || norm < (1 - 1e-4 * fraction) * residualNorm)
                {
                    z = candidate;
                    improved = true;
                    break;
                }
            }

            if (!improved)
            {
                throw NonlinearFailure(iterations, residualNorm, "Damped Newton line search failed to reduce the dimensionless residual.");
            }
        }

        MultibodyState predicted = MidpointKinematics(state, z, scales, step, false);
        ConstraintAssembly before = new(system, predicted, settings, "Before projection");
        MultibodyState accepted = ProjectPositions(system, predicted, settings, out int positionIterations);
        accepted = ProjectVelocities(system, accepted, settings, out int velocityIterations);
        ConstrainedDynamicsResult acceptedDynamics = ConstrainedMultibodyDynamics.Evaluate(system, accepted,
            evaluateLoads(endpoint, accepted), settings);
        if (acceptedDynamics.Diagnostics.MaximumPositionError > settings.PositionTolerance
            || acceptedDynamics.Diagnostics.MaximumVelocityError > settings.VelocityTolerance
            || acceptedDynamics.Diagnostics.MaximumAngularPositionError > settings.AngularConstraints.OrientationTolerance
            || acceptedDynamics.Diagnostics.MaximumAngularVelocityError > settings.AngularConstraints.AngularVelocityTolerance)
        {
            bool mixed = system.RevoluteJoints.Count != 0;
            ConstraintDiagnostics errors = acceptedDynamics.Diagnostics;
            double finalResidual = mixed ? System.Math.Max(
                System.Math.Max(errors.MaximumPositionError / settings.PositionTolerance, errors.MaximumVelocityError / settings.VelocityTolerance),
                System.Math.Max(errors.MaximumAngularPositionError / settings.AngularConstraints.OrientationTolerance,
                    errors.MaximumAngularVelocityError / settings.AngularConstraints.AngularVelocityTolerance)) : errors.MaximumVelocityError;
            throw new ConstrainedSolveException(ConstrainedSolveFailure.VelocityProjection, "Final constraint verification",
                velocityIterations, finalResidual, mixed ? "Projected endpoint failed point/angular verification; residual is tolerance-scaled."
                    : "Projected endpoint failed final gap/slip verification.");
        }

        BodyProjectionCorrection[] corrections = new BodyProjectionCorrection[system.Bodies.Count];
        for (int body = 0; body < corrections.Length; body++)
        {
            RigidBodyState raw = predicted.Bodies[body];
            RigidBodyState final = accepted.Bodies[body];
            corrections[body] = new(final.Pose.Position - raw.Pose.Position,
                (raw.Pose.Orientation.Conjugated() * final.Pose.Orientation).Normalized(),
                final.LinearVelocity - raw.LinearVelocity, final.BodyAngularVelocity - raw.BodyAngularVelocity);
        }

        return new(accepted, acceptedDynamics, corrections, new(iterations, residualNorm,
            positionIterations, velocityIterations, before.MaximumLinearError(before.PositionError),
            before.MaximumLinearError(before.VelocityError))
        {
            BeforeProjectionAngularPositionError = before.MaximumAlignmentError,
            BeforeProjectionAngularVelocityError = before.MaximumAngularError(before.VelocityError)
        }, predicted);

        double[] Residual(double[] increment)
        {
            MultibodyState middle = MidpointKinematics(state, increment, scales, step, true);
            ConstrainedDynamicsResult dynamics = ConstrainedMultibodyDynamics.Evaluate(system, middle,
                evaluateLoads(midpoint, middle), settings);
            double[] result = new double[dimension];
            for (int body = 0; body < state.Bodies.Count; body++)
            {
                ConstraintAssembly.Put(result, 6 * body, dynamics.Derivatives[body].LinearAcceleration);
                ConstraintAssembly.Put(result, 6 * body + 3, dynamics.Derivatives[body].BodyAngularAcceleration);
            }

            for (int i = 0; i < result.Length; i++)
            {
                result[i] = increment[i] - (step / scales[i]) * result[i];
            }

            return result;
        }
    }

    private static MultibodyState MidpointKinematics(MultibodyState initial, double[] increment,
        double[] scales, double step, bool middle)
    {
        RigidBodyState[] states = new RigidBodyState[initial.Bodies.Count];
        for (int body = 0; body < states.Length; body++)
        {
            int offset = 6 * body;
            RigidBodyState start = initial.Bodies[body];
            Vector3d deltaV = ConstraintAssembly.Get(increment, offset) * scales[offset];
            Vector3d deltaOmega = ConstraintAssembly.Get(increment, offset + 3) * scales[offset + 3];
            Vector3d midpointVelocity = start.LinearVelocity + deltaV * 0.5;
            Vector3d midpointOmega = start.BodyAngularVelocity + deltaOmega * 0.5;
            // Eliminate the raw quaternion midpoint equation exactly:
            // q1-q0 = h/4 * (q0+q1) * (0,omega_mid).
            // With a=h*omega_mid/4, q1=q0*(1,a)^2/(1+|a|^2).
            // The normalized arithmetic midpoint orientation is q0*normalize(1,a).
            // This Cayley update is second order and has no zero-axis division.
            Vector3d a = midpointOmega * (step / 4);
            Quaterniond halfRotation = new Quaterniond(1, a.X, a.Y, a.Z).Normalized();
            Quaterniond middleOrientation = start.Pose.Orientation * halfRotation;
            Quaterniond orientation = middle ? middleOrientation : middleOrientation * halfRotation;
            RigidPose3d pose = new(start.Pose.Position + midpointVelocity * (middle ? step / 2 : step), orientation);
            states[body] = new(pose, start.LinearVelocity + deltaV * (middle ? 0.5 : 1),
                start.BodyAngularVelocity + deltaOmega * (middle ? 0.5 : 1));
        }

        return new(states);
    }

    private static MultibodyState ProjectPositions(MultibodySystem system, MultibodyState state,
        ConstrainedSolverSettings settings, out int iterations)
    {
        for (iterations = 0; ; iterations++)
        {
            ConstraintAssembly assembly = new(system, state, settings, "Position projection");
            double norm = assembly.PositionNorm(settings);
            if (norm <= 1)
            {
                return state;
            }

            if (iterations == settings.MaxProjectionIterations)
            {
                throw ProjectionFailure(true, iterations, norm, system, settings, "Position projection iteration limit reached.");
            }

            double[] multipliers = assembly.Factor.Solve(assembly.PositionError, out _);
            double[] displacement = ConstrainedLinearAlgebra.Multiply(assembly.MassResponse, multipliers);
            bool improved = false;
            double fraction = 1;
            for (int trial = 0; trial < settings.MaxLineSearchIterations; trial++, fraction /= 2)
            {
                RigidBodyState[] bodies = new RigidBodyState[state.Bodies.Count];
                for (int body = 0; body < bodies.Length; body++)
                {
                    RigidBodyState current = state.Bodies[body];
                    Vector3d rotation = ConstraintAssembly.Get(displacement, 6 * body + 3) * -fraction;
                    double angle = rotation.Length;
                    Quaterniond change = angle == 0 ? Quaterniond.Identity : Quaterniond.FromAxisAngle(rotation, angle);
                    bodies[body] = new(new(current.Pose.Position - ConstraintAssembly.Get(displacement, 6 * body) * fraction,
                        current.Pose.Orientation * change), current.LinearVelocity, current.BodyAngularVelocity);
                }

                MultibodyState candidate = new(bodies);
                ConstraintAssembly trialAssembly = new(system, candidate, settings, "Position projection trial");
                double trialNorm = trialAssembly.PositionNorm(settings);
                if (trialNorm <= 1 || trialNorm < (1 - 1e-4 * fraction) * norm)
                {
                    state = candidate;
                    improved = true;
                    break;
                }
            }

            if (!improved)
            {
                throw ProjectionFailure(true, iterations, norm, system, settings, "Position projection line search failed.");
            }
        }
    }

    private static MultibodyState ProjectVelocities(MultibodySystem system, MultibodyState state,
        ConstrainedSolverSettings settings, out int iterations)
    {
        for (iterations = 0; ; iterations++)
        {
            ConstraintAssembly assembly = new(system, state, settings, "Velocity projection");
            double norm = assembly.VelocityNorm(settings);
            if (norm <= 1)
            {
                return state;
            }

            if (iterations == settings.MaxProjectionIterations)
            {
                throw ProjectionFailure(false, iterations, norm, system, settings, "Velocity projection iteration limit reached.");
            }

            double[] multipliers = assembly.Factor.Solve(assembly.VelocityError, out _);
            double[] change = ConstrainedLinearAlgebra.Multiply(assembly.MassResponse, multipliers);
            RigidBodyState[] bodies = new RigidBodyState[state.Bodies.Count];
            for (int body = 0; body < bodies.Length; body++)
            {
                RigidBodyState current = state.Bodies[body];
                bodies[body] = new(current.Pose, current.LinearVelocity - ConstraintAssembly.Get(change, 6 * body),
                    current.BodyAngularVelocity - ConstraintAssembly.Get(change, 6 * body + 3));
            }

            state = new(bodies);
        }
    }

    private static ConstrainedSolveException NonlinearFailure(int iterations, double residual, string message) =>
        new(ConstrainedSolveFailure.NonlinearSolve, "Implicit midpoint", iterations, residual, message);

    private static ConstrainedSolveException ProjectionFailure(bool position, int iterations, double residual, string message) =>
        new(position ? ConstrainedSolveFailure.PositionProjection : ConstrainedSolveFailure.VelocityProjection,
            position ? "Position projection" : "Velocity projection", iterations, residual, message);

    private static ConstrainedSolveException ProjectionFailure(bool position, int iterations, double scaledResidual,
        MultibodySystem system, ConstrainedSolverSettings settings, string message) =>
        ProjectionFailure(position, iterations,
            system.RevoluteJoints.Count == 0 ? scaledResidual * (position ? settings.PositionTolerance : settings.VelocityTolerance) : scaledResidual,
            message + (system.RevoluteJoints.Count != 0 ? " Residual is tolerance-scaled across linear/angular blocks."
                : position ? " Residual is in m." : " Residual is in m/s."));
}
