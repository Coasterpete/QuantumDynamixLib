using System;
using System.Collections.Generic;
using System.Linq;
using QuantumDynamix.Geometry;
using QuantumDynamix.Math;

namespace QuantumDynamix.Physics;

/// <summary>First-order semi-implicit rigid inelastic contact stepping, with coupled bilateral joints.
/// Gap/h approximates collision timing. Bounded travel sampling is not continuous collision detection.</summary>
/// <remarks>Uses the caller's external loads at the start snapshot, including Euler gyroscopic terms.
/// Independent wheel spin uses explicit wheel bodies and revolute joints. No hidden compliance,
/// gravity, retry, tangential friction, or guidance is supplied.</remarks>
public static class ContactMultibodyIntegrator
{
    public static ContactStepResult Step(WheelRailContactSystem system, MultibodyState state,
        double time, double step, Func<double, MultibodyState, IReadOnlyList<RigidBodyLoads>> evaluateLoads,
        ContactSolverSettings settings)
    {
        ArgumentNullException.ThrowIfNull(system);
        ArgumentNullException.ThrowIfNull(settings);
        ArgumentNullException.ThrowIfNull(evaluateLoads);
        system.Dynamics.ValidateState(state);
        if (!double.IsFinite(time) || !double.IsFinite(step) || step <= 0
            || !double.IsFinite(time + step) || !(time + step > time))
        { throw new ArgumentOutOfRangeException(nameof(step), "Resolve distinct finite start/end times with a positive step."); }
        using var formulation = new ContactFormulationScope(system, rigid: true);
        try { return StepCore(system, state, time, step, evaluateLoads, settings); }
        catch (ContactSolveException error)
        {
            throw new ContactSolveException(error.Failure, error.Stage, error.Explanation,
                error.WheelId, error.PatchId, error.Iterations, error.Residual, error, system.Pairs);
        }
        catch (ConstrainedSolveException error)
        {
            throw new ContactSolveException(ContactSolveFailure.LinearSolve, error.Stage,
                "The existing bilateral row assembly/solve failed.", iterations: error.Iterations, residual: error.Residual, inner: error);
        }
        catch (Exception error) when (error is ArithmeticException or ArgumentOutOfRangeException)
        {
            throw new ContactSolveException(ContactSolveFailure.NumericalFailure, "Contact step",
                "A derived quantity or load callback exceeded supported arithmetic.", inner: error);
        }
    }

    private static ContactStepResult StepCore(WheelRailContactSystem system, MultibodyState state,
        double time, double step, Func<double, MultibodyState, IReadOnlyList<RigidBodyLoads>> evaluateLoads,
        ContactSolverSettings settings)
    {
        MultibodySystem dynamics = system.Dynamics;
        ConstraintAssembly initial = new(dynamics, state, settings.Joints, "Initial contact joints");
        var contacts = WheelRailContactGeometry.Query(system, state, settings.Geometry).Contacts;
        if (initial.PositionNorm(settings.Joints) > 1 || initial.VelocityNorm(settings.Joints) > 1
            || contacts.Any(c => c.Gap < -settings.PositionTolerance))
        { throw new ContactSolveException(ContactSolveFailure.InconsistentInitialState, "Initial contact state", "Supply assembled joints and nonpenetrating wheels (separate m, m/s, rad and angular-row-rate tolerances)."); }
        IReadOnlyList<RigidBodyLoads> loads = evaluateLoads(time, state);
        ArgumentNullException.ThrowIfNull(loads);
        if (loads.Count != dynamics.Bodies.Count) { throw new ArgumentException("Supply one external load per body.", nameof(evaluateLoads)); }
        double[] free = ConstraintAssembly.Velocities(state);
        for (int body = 0; body < dynamics.Bodies.Count; body++)
        {
            RigidBodyDerivative derivative = RigidBodyDynamics.EvaluateDerivative(dynamics.Bodies[body], state.Bodies[body], loads[body]);
            Add(free, 6 * body, derivative.LinearAcceleration * step);
            Add(free, 6 * body + 3, derivative.BodyAngularAcceleration * step);
        }
        CheckTravel(system, free, step, settings);
        double[,] rows = Rows(initial, state, contacts);
        int equalities = initial.PositionError.Length;
        double[] lower = new double[rows.GetLength(0)];
        for (int i = 0; i < equalities; i++) { lower[i] = -initial.PositionError[i] / step; }
        // Revolute direction/attachment bias is known analytically. Its first-order velocity
        // target makes the physical bearing impulse include turning/gyroscopic reactions.
        // Retain the established spherical-only discrete equations and reaction semantics.
        for (int i = 3 * dynamics.Joints.Count; i < equalities; i++) { lower[i] -= step * initial.Bias[i]; }
        for (int i = 0; i < contacts.Count; i++) { lower[equalities + i] = -contacts[i].Gap / step; }
        var solve = new ContactConstraintSolve(dynamics, rows, equalities, lower, free, settings,
            settings.VelocityTolerance, settings.ImpulseTolerance, "Coupled contact impulses", checkReactionRank: true,
            angularRows: initial.AngularRows, angularFeasibilityTolerance: settings.Joints.AngularConstraints.AngularVelocityTolerance);
        CheckTravel(system, solve.Values, step, settings);
        MultibodyState predicted = Advance(state, solve.Values, step);
        // Bounded checks of the actual proposed discrete path. Thin features between samples can
        // still be missed: travel settings and refinement remain the caller's responsibility.
        for (int sample = 1; sample <= settings.TravelSamples; sample++)
        {
            MultibodyState probe = sample == settings.TravelSamples ? predicted
                : Advance(state, solve.Values, step * sample / settings.TravelSamples);
            var pathContacts = WheelRailContactGeometry.Query(system, probe, settings.Geometry).Contacts;
            foreach (WheelRailContactSample contact in pathContacts)
            {
                if (contact.Gap < -settings.MaximumPositionCorrection)
                { throw new ContactSolveException(ContactSolveFailure.StepResolution, "Contact travel", "Sampled path penetration exceeds the supplied correction limit (m); reduce the step.", contact.WheelId, contact.PatchId, residual: -contact.Gap); }
            }
        }
        MultibodyState accepted = ProjectPositions(system, predicted, settings, out int projectionIterations);
        var acceptedContacts = WheelRailContactGeometry.Query(system, accepted, settings.Geometry).Contacts;
        ConstraintAssembly endpoint = new(dynamics, accepted, settings.Joints, "Endpoint contact joints");
        // Restore joint velocity consistency at corrected poses without worsening any contact's
        // post-impulse normal closing velocity. Gap/h can legitimately leave a new contact closing.
        double[,] endpointRows = Rows(endpoint, accepted, acceptedContacts);
        double[] endpointVelocity = ConstraintAssembly.Velocities(accepted);
        double[] endpointLower = new double[endpointRows.GetLength(0)];
        double[] before = ConstrainedLinearAlgebra.Multiply(endpointRows, endpointVelocity);
        for (int i = equalities; i < endpointLower.Length; i++) { endpointLower[i] = System.Math.Min(0, before[i]); }
        var velocityCorrection = new ContactConstraintSolve(dynamics, endpointRows, equalities, endpointLower,
            endpointVelocity, settings, System.Math.Min(settings.Joints.VelocityTolerance, settings.VelocityTolerance) / 4,
            settings.ImpulseTolerance, "Numerical endpoint velocity correction", angularRows: endpoint.AngularRows,
            angularFeasibilityTolerance: settings.Joints.AngularConstraints.AngularVelocityTolerance / 4);
        accepted = WithVelocities(accepted, velocityCorrection.Values);
        acceptedContacts = WheelRailContactGeometry.Query(system, accepted, settings.Geometry).Contacts;
        endpoint = new(dynamics, accepted, settings.Joints, "Accepted contact joints");
        double gap = endpoint.MaximumLinearError(endpoint.PositionError);
        double slip = endpoint.MaximumLinearError(endpoint.VelocityError);
        if (endpoint.PositionNorm(settings.Joints) > 1 || endpoint.VelocityNorm(settings.Joints) > 1
            || acceptedContacts.Any(c => c.Gap < -settings.PositionTolerance))
        { throw new ContactSolveException(ContactSolveFailure.PositionProjection, "Final contact verification", "Endpoint joint/contact bounds were not met."); }

        RigidBodyLoads[] averageLoads = new RigidBodyLoads[dynamics.Bodies.Count];
        var impulses = new WheelRailContactImpulse[contacts.Count];
        for (int i = 0; i < contacts.Count; i++)
        {
            WheelRailContactSample contact = contacts[i];
            double magnitude = solve.Multipliers[equalities + i];
            Vector3d impulse = contact.Normal * magnitude;
            Vector3d angular = Vector3d.Cross(contact.WheelPoint - state.Bodies[contact.BodyIndex].Pose.Position, impulse);
            impulses[i] = new(contact, magnitude, impulse, angular, impulse / step, angular / step);
            averageLoads[contact.BodyIndex] += new RigidBodyLoads(impulse / step, angular / step);
        }
        var jointImpulses = new SphericalJointImpulse[dynamics.Joints.Count];
        for (int i = 0; i < jointImpulses.Length; i++)
        {
            SphericalJoint joint = dynamics.Joints[i];
            Vector3d impulse = ConstraintAssembly.Get(solve.Multipliers, 3 * i);
            Vector3d angularA = Vector3d.Cross(state.Bodies[joint.BodyA].Pose.TransformDirection(joint.LocalAnchorA), impulse);
            Vector3d angularB = Vector3d.Cross(state.Bodies[joint.BodyB].Pose.TransformDirection(joint.LocalAnchorB), -impulse);
            jointImpulses[i] = new(impulse, angularA, angularB);
            averageLoads[joint.BodyA] += new RigidBodyLoads(impulse / step, angularA / step);
            averageLoads[joint.BodyB] += new RigidBodyLoads(-impulse / step, angularB / step);
        }
        var revoluteImpulses = new RevoluteJointImpulse[dynamics.RevoluteJoints.Count];
        for (int i = 0; i < revoluteImpulses.Length; i++)
        {
            RevoluteJoint joint = dynamics.RevoluteJoints[i];
            RevoluteJointReaction impulse = initial.RevoluteReaction(dynamics, state, i, solve.Multipliers);
            revoluteImpulses[i] = new(impulse.ForceOnA, impulse.CoupleOnA, impulse.TorqueOnA, impulse.TorqueOnB);
            averageLoads[joint.BodyA] += new RigidBodyLoads(impulse.ForceOnA / step, impulse.TorqueOnA / step);
            averageLoads[joint.BodyB] += new RigidBodyLoads(impulse.ForceOnB / step, impulse.TorqueOnB / step);
        }
        var corrections = new BodyProjectionCorrection[dynamics.Bodies.Count];
        for (int i = 0; i < corrections.Length; i++)
        {
            RigidBodyState raw = predicted.Bodies[i];
            RigidBodyState final = accepted.Bodies[i];
            corrections[i] = new(final.Pose.Position - raw.Pose.Position,
                (raw.Pose.Orientation.Conjugated() * final.Pose.Orientation).Normalized(),
                final.LinearVelocity - raw.LinearVelocity, final.BodyAngularVelocity - raw.BodyAngularVelocity);
        }
        return new(accepted, impulses, acceptedContacts.ToArray(), jointImpulses, revoluteImpulses, averageLoads, corrections,
            new(solve.Iterations, projectionIterations, acceptedContacts.Count == 0 ? 0 : acceptedContacts.Min(c => c.Gap),
                gap, slip, solve.FeasibilityError, solve.NegativeImpulse, solve.ComplementarityError,
                solve.LinearError, solve.MomentumError, solve.MinimumPivot)
            {
                MaximumJointAngularGap = endpoint.MaximumAlignmentError,
                MaximumJointAngularSlip = endpoint.MaximumAngularError(endpoint.VelocityError),
                MaximumAngularVelocityViolation = solve.AngularFeasibilityError,
                ScaledVelocityViolation = solve.ScaledFeasibilityError
            });
    }

    private static MultibodyState ProjectPositions(WheelRailContactSystem system, MultibodyState predicted,
        ContactSolverSettings settings, out int iterations)
    {
        MultibodyState state = predicted;
        for (iterations = 0; ; iterations++)
        {
            ConstraintAssembly joints = new(system.Dynamics, state, settings.Joints, "Contact position correction");
            var contacts = WheelRailContactGeometry.Query(system, state, settings.Geometry).Contacts;
            double norm = PositionNorm(joints, contacts, settings);
            if (norm <= 1) { return state; }
            if (iterations >= settings.MaximumProjectionIterations)
            { throw new ContactSolveException(ContactSolveFailure.PositionProjection, "Contact position correction", "Iteration budget exhausted; residual is tolerance-scaled.", iterations: iterations, residual: norm); }
            double[,] rows = Rows(joints, state, contacts);
            int equalities = joints.PositionError.Length;
            double[] lower = new double[rows.GetLength(0)];
            for (int i = 0; i < equalities; i++) { lower[i] = -joints.PositionError[i]; }
            for (int i = 0; i < contacts.Count; i++) { lower[equalities + i] = -contacts[i].Gap; }
            var correction = new ContactConstraintSolve(system.Dynamics, rows, equalities, lower,
                new double[6 * state.Bodies.Count], settings,
                System.Math.Min(settings.Joints.PositionTolerance, settings.PositionTolerance) / 4,
                settings.ImpulseTolerance, "Numerical contact position correction", angularRows: joints.AngularRows,
                angularFeasibilityTolerance: System.Math.Sin(settings.Joints.AngularConstraints.OrientationTolerance) / 4,
                positionCorrection: true);
            bool improved = false;
            double fraction = 1;
            for (int trial = 0; trial < settings.Joints.MaxLineSearchIterations; trial++, fraction /= 2)
            {
                MultibodyState candidate = Displace(state, correction.Values, fraction);
                CheckCorrections(predicted, candidate, settings);
                ConstraintAssembly candidateJoints = new(system.Dynamics, candidate, settings.Joints, "Position correction trial");
                var candidateContacts = WheelRailContactGeometry.Query(system, candidate, settings.Geometry).Contacts;
                double trialNorm = PositionNorm(candidateJoints, candidateContacts, settings);
                if (trialNorm <= 1 || trialNorm < norm * (1 - 1e-4 * fraction))
                { state = candidate; improved = true; break; }
            }
            if (!improved)
            { throw new ContactSolveException(ContactSolveFailure.PositionProjection, "Contact position correction", "Line search failed; residual is tolerance-scaled.", iterations: iterations, residual: norm); }
        }
    }

    private static double PositionNorm(ConstraintAssembly joints, IReadOnlyList<WheelRailContactSample> contacts, ContactSolverSettings settings)
    {
        double norm = joints.PositionNorm(settings.Joints);
        foreach (WheelRailContactSample contact in contacts) { norm = System.Math.Max(norm, -contact.Gap / settings.PositionTolerance); }
        return norm;
    }

    private static double[,] Rows(ConstraintAssembly joints, MultibodyState state, IReadOnlyList<WheelRailContactSample> contacts)
    {
        int equalities = joints.PositionError.Length;
        double[,] rows = new double[equalities + contacts.Count, 6 * state.Bodies.Count];
        for (int i = 0; i < equalities; i++)
        { for (int j = 0; j < rows.GetLength(1); j++) { rows[i, j] = joints.Jacobian[i, j]; } }
        for (int i = 0; i < contacts.Count; i++)
        {
            WheelRailContactSample c = contacts[i];
            RigidPose3d pose = state.Bodies[c.BodyIndex].Pose;
            Vector3d localArm = pose.InverseTransformPoint(c.WheelPoint);
            ContactConstraintSolve.PutRow(rows, equalities + i, 6 * c.BodyIndex, c.Normal);
            ContactConstraintSolve.PutRow(rows, equalities + i, 6 * c.BodyIndex + 3,
                Vector3d.Cross(localArm, pose.InverseTransformDirection(c.Normal)));
        }
        return rows;
    }

    private static void CheckTravel(WheelRailContactSystem system, double[] velocities, double step, ContactSolverSettings settings)
    {
        for (int i = 0; i < system.Dynamics.Bodies.Count; i++)
        {
            double extent = 0;
            foreach (WheelDefinition wheel in system.Wheels.Where(w => w.BodyIndex == i))
            {
                extent = System.Math.Max(extent, wheel.LocalPose.Position.Length
                    + new Vector3d(wheel.Tread.Radius, wheel.Tread.HalfWidth, 0).Length);
            }
            double angle = ConstraintAssembly.Get(velocities, 6 * i + 3).Length * step;
            double travel = ConstraintAssembly.Get(velocities, 6 * i).Length * step + extent * angle;
            if (angle > settings.MaximumAngularStep || travel > settings.MaximumPointTravel)
            { throw new ContactSolveException(ContactSolveFailure.StepResolution, "Contact travel", "Predicted movement exceeds supplied metre/radian limits; reduce the timestep.", residual: travel); }
        }
    }

    private static void CheckCorrections(MultibodyState predicted, MultibodyState candidate, ContactSolverSettings settings)
    {
        for (int i = 0; i < predicted.Bodies.Count; i++)
        {
            RigidPose3d a = predicted.Bodies[i].Pose;
            RigidPose3d b = candidate.Bodies[i].Pose;
            Quaterniond rotation = (a.Orientation.Conjugated() * b.Orientation).Normalized();
            double angle = 2 * System.Math.Atan2(new Vector3d(rotation.X, rotation.Y, rotation.Z).Length, System.Math.Abs(rotation.W));
            if ((b.Position - a.Position).Length > settings.MaximumPositionCorrection || angle > settings.MaximumOrientationCorrection)
            { throw new ContactSolveException(ContactSolveFailure.StepResolution, "Contact position correction", "Numerical correction exceeds supplied metre/radian limits; reduce the timestep."); }
        }
    }

    private static MultibodyState Advance(MultibodyState state, double[] velocities, double step)
    {
        RigidBodyState[] bodies = new RigidBodyState[state.Bodies.Count];
        for (int i = 0; i < bodies.Length; i++)
        {
            Vector3d v = ConstraintAssembly.Get(velocities, 6 * i);
            Vector3d omega = ConstraintAssembly.Get(velocities, 6 * i + 3);
            bodies[i] = new(new(state.Bodies[i].Pose.Position + v * step,
                state.Bodies[i].Pose.Orientation * Rotation(omega * step)), v, omega);
        }
        return new(bodies);
    }

    private static MultibodyState Displace(MultibodyState state, double[] displacement, double fraction)
    {
        RigidBodyState[] bodies = new RigidBodyState[state.Bodies.Count];
        for (int i = 0; i < bodies.Length; i++)
        {
            RigidBodyState body = state.Bodies[i];
            bodies[i] = new(new(body.Pose.Position + ConstraintAssembly.Get(displacement, 6 * i) * fraction,
                body.Pose.Orientation * Rotation(ConstraintAssembly.Get(displacement, 6 * i + 3) * fraction)),
                body.LinearVelocity, body.BodyAngularVelocity);
        }
        return new(bodies);
    }

    private static MultibodyState WithVelocities(MultibodyState state, double[] velocities) => new(
        state.Bodies.Select((body, i) => new RigidBodyState(body.Pose,
            ConstraintAssembly.Get(velocities, 6 * i), ConstraintAssembly.Get(velocities, 6 * i + 3))));

    private static Quaterniond Rotation(Vector3d vector) => vector.Length == 0
        ? Quaterniond.Identity : Quaterniond.FromAxisAngle(vector, vector.Length);

    private static void Add(double[] values, int offset, Vector3d increment) =>
        ConstraintAssembly.Put(values, offset, ConstraintAssembly.Get(values, offset) + increment);
}
