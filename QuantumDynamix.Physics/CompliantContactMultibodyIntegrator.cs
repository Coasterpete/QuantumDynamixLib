using System;
using System.Collections.Generic;
using QuantumDynamix.Geometry;
using QuantumDynamix.Math;

namespace QuantumDynamix.Physics;

/// <summary>Numerical movement/projection limits (m, rad), separate from every physical contact law.
/// Travel sampling is bounded inspection, not continuous collision detection or a force-error bound.</summary>
public sealed class CompliantContactStepSettings
{
    public ContactGeometrySettings Geometry { get; }
    public ConstrainedSolverSettings Joints { get; }
    public double MaximumPointTravel { get; }
    public double MaximumAngularStep { get; }
    public double MaximumPositionCorrection { get; }
    public double MaximumOrientationCorrection { get; }
    public int TravelSamples { get; }

    public CompliantContactStepSettings(double maximumPointTravel, double maximumAngularStep,
        double maximumPositionCorrection, double maximumOrientationCorrection,
        ContactGeometrySettings? geometry = null, ConstrainedSolverSettings? joints = null, int travelSamples = 4)
    {
        ContactChecks.Positive(maximumPointTravel, nameof(maximumPointTravel));
        ContactChecks.Positive(maximumAngularStep, nameof(maximumAngularStep));
        ContactChecks.Positive(maximumPositionCorrection, nameof(maximumPositionCorrection));
        ContactChecks.Positive(maximumOrientationCorrection, nameof(maximumOrientationCorrection));
        if (maximumAngularStep >= System.Math.PI || maximumOrientationCorrection >= System.Math.PI)
        { throw new ArgumentOutOfRangeException(nameof(maximumAngularStep), "Rotation bounds must be below pi to resolve the sampled rotation branch."); }
        if (travelSamples < 2 || travelSamples > 1024) { throw new ArgumentOutOfRangeException(nameof(travelSamples)); }
        MaximumPointTravel = maximumPointTravel;
        MaximumAngularStep = maximumAngularStep;
        MaximumPositionCorrection = maximumPositionCorrection;
        MaximumOrientationCorrection = maximumOrientationCorrection;
        Geometry = geometry ?? new();
        Joints = joints ?? ConstrainedSolverSettings.Default;
        TravelSamples = travelSamples;
    }
}

/// <summary>All quantities are J. External work and physical dissipation use midpoint quadrature,
/// not exact integrals. IntegrationEnergyResidual excludes endpoint joint projection.
/// Projection kinetic/elastic changes are numerical and are never reported as physical loads.</summary>
public readonly record struct CompliantContactEnergyDiagnostics(double InitialMechanicalEnergy,
    double UnprojectedMechanicalEnergy, double AcceptedMechanicalEnergy, double MidpointExternalWork,
    double MidpointDissipation, double IntegrationEnergyResidual, double ProjectionKineticEnergyChange,
    double ProjectionElasticEnergyChange);

public sealed class CompliantContactStepResult
{
    public MultibodyState State => JointStep.State;
    /// <summary>Existing instantaneous endpoint joint reactions/derivatives and numerical corrections.
    /// ConstraintLoads remain bilateral joint loads only.</summary>
    public ConstrainedStepResult JointStep { get; }
    public MultibodyState UnprojectedState { get; }
    public CompliantContactEvaluation InitialContacts { get; }
    public CompliantContactEvaluation MidpointContacts { get; }
    public CompliantContactEvaluation UnprojectedContacts { get; }
    public CompliantContactEvaluation AcceptedContacts { get; }
    public CompliantContactEnergyDiagnostics Energy { get; }

    internal CompliantContactStepResult(ConstrainedStepResult joints, CompliantContactEvaluation initial,
        CompliantContactEvaluation midpoint, CompliantContactEvaluation unprojected,
        CompliantContactEvaluation accepted, CompliantContactEnergyDiagnostics energy)
    {
        JointStep = joints;
        UnprojectedState = joints.UnprojectedState;
        InitialContacts = initial;
        MidpointContacts = midpoint;
        UnprojectedContacts = unprojected;
        AcceptedContacts = accepted;
        Energy = energy;
    }
}

/// <summary>Normal compliance composed with the existing projected implicit-midpoint joint solver.
/// No compliant contact row is rigidly enforced or projected to zero compression.</summary>
/// <remarks>Pure callbacks are evaluated repeatedly at joint-off-manifold Newton trials. Physical-law
/// domain failures also reject trials; no extrapolation, retry, stiffness change, or impact fallback
/// is supplied. Second-order smooth-branch behavior does not establish contact-transition accuracy.</remarks>
public static class CompliantContactMultibodyIntegrator
{
    public static CompliantContactStepResult Step(CompliantWheelRailContactModel model, MultibodyState state,
        double time, double step, Func<double, MultibodyState, IReadOnlyList<RigidBodyLoads>> evaluateLoads,
        CompliantContactStepSettings settings)
    {
        ArgumentNullException.ThrowIfNull(model);
        ArgumentNullException.ThrowIfNull(settings);
        ArgumentNullException.ThrowIfNull(evaluateLoads);
        model.Geometry.Dynamics.ValidateState(state);
        double middleTime = time + step / 2;
        double endTime = time + step;
        if (!double.IsFinite(time) || !double.IsFinite(step) || step <= 0 || !double.IsFinite(endTime)
            || !(middleTime > time && middleTime < endTime))
        { throw new ArgumentOutOfRangeException(nameof(step), "Resolve distinct finite start, midpoint, and endpoint times."); }
        using var formulation = new ContactFormulationScope(model.Geometry, rigid: false);
        try { return StepCore(model, state, time, step, evaluateLoads, settings); }
        catch (Exception error) when (error is ArithmeticException or ArgumentOutOfRangeException)
        {
            throw new ContactSolveException(ContactSolveFailure.NumericalFailure, "Compliant step",
                "A load, movement, or energy quantity exceeded supported arithmetic.",
                inner: error, involvedPairs: model.Geometry.Pairs);
        }
    }

    private static CompliantContactStepResult StepCore(CompliantWheelRailContactModel model, MultibodyState state,
        double time, double step, Func<double, MultibodyState, IReadOnlyList<RigidBodyLoads>> evaluateLoads,
        CompliantContactStepSettings settings)
    {
        WheelRailContactSystem geometry = model.Geometry;
        CompliantContactEvaluation initial = CompliantWheelRailContact.Evaluate(model, state, settings.Geometry);
        ConstrainedStepResult result = ConstrainedMultibodyIntegrator.Step(geometry.Dynamics, state, time, step,
            CombinedLoads, settings.Joints);
        MultibodyState raw = result.UnprojectedState;
        CheckMotion(geometry, state, raw, step, settings);
        CheckMotion(geometry, state, result.State, step, settings);
        foreach (BodyProjectionCorrection correction in result.ProjectionCorrections)
        {
            if (correction.PositionChange.Length > settings.MaximumPositionCorrection
                || RotationAngle(correction.OrientationCorrection) > settings.MaximumOrientationCorrection)
            { throw Resolution("Joint projection exceeds numerical correction bounds; reduce the timestep."); }
        }
        // Interpolate the unprojected discrete path: linear COM and shortest-axis rotation.
        // Its midpoint is the solver's physical midpoint. This is not a continuous trajectory certificate.
        for (int sample = 1; sample < settings.TravelSamples; sample++)
        {
            CompliantWheelRailContact.Evaluate(model, Interpolate(state, raw, (double)sample / settings.TravelSamples), settings.Geometry);
        }
        MultibodyState middle = Interpolate(state, raw, 0.5);
        CompliantContactEvaluation midpoint = CompliantWheelRailContact.Evaluate(model, middle, settings.Geometry);
        CompliantContactEvaluation unprojected = CompliantWheelRailContact.Evaluate(model, raw, settings.Geometry);
        CompliantContactEvaluation accepted = CompliantWheelRailContact.Evaluate(model, result.State, settings.Geometry);
        double initialKinetic = KineticEnergy(geometry.Dynamics, state);
        double rawKinetic = KineticEnergy(geometry.Dynamics, raw);
        double acceptedKinetic = KineticEnergy(geometry.Dynamics, result.State);
        double initialEnergy = initialKinetic + initial.ElasticEnergy;
        double rawEnergy = rawKinetic + unprojected.ElasticEnergy;
        double acceptedEnergy = acceptedKinetic + accepted.ElasticEnergy;
        IReadOnlyList<RigidBodyLoads> external = evaluateLoads(time + step / 2, middle);
        ValidateLoads(external, state.Bodies.Count);
        double power = 0;
        for (int i = 0; i < external.Count; i++)
        {
            power += Vector3d.Dot(external[i].Force, middle.Bodies[i].LinearVelocity)
                + Vector3d.Dot(external[i].Torque, middle.Bodies[i].WorldAngularVelocity);
        }
        double work = step * power;
        double loss = step * midpoint.DissipationPower;
        double residual = rawEnergy - initialEnergy - work + loss;
        var diagnostics = new CompliantContactEnergyDiagnostics(initialEnergy, rawEnergy, acceptedEnergy,
            work, loss, residual, acceptedKinetic - rawKinetic, accepted.ElasticEnergy - unprojected.ElasticEnergy);
        if (!double.IsFinite(initialEnergy) || !double.IsFinite(rawEnergy) || !double.IsFinite(acceptedEnergy)
            || !double.IsFinite(work) || !double.IsFinite(loss) || !double.IsFinite(residual)
            || !double.IsFinite(diagnostics.ProjectionKineticEnergyChange) || !double.IsFinite(diagnostics.ProjectionElasticEnergyChange))
        { throw new ArithmeticException("Energy diagnostics exceed finite range."); }
        return new(result, initial, midpoint, unprojected, accepted, diagnostics);

        IReadOnlyList<RigidBodyLoads> CombinedLoads(double stageTime, MultibodyState stage)
        {
            CompliantContactEvaluation contact = CompliantWheelRailContact.Evaluate(model, stage, settings.Geometry);
            IReadOnlyList<RigidBodyLoads> externalLoads = evaluateLoads(stageTime, stage);
            ValidateLoads(externalLoads, stage.Bodies.Count);
            var combined = new RigidBodyLoads[stage.Bodies.Count];
            for (int i = 0; i < combined.Length; i++) { combined[i] = externalLoads[i] + contact.BodyLoads[i]; }
            return combined;
        }
    }

    private static void ValidateLoads(IReadOnlyList<RigidBodyLoads> loads, int count)
    {
        ArgumentNullException.ThrowIfNull(loads);
        if (loads.Count != count) { throw new ArgumentException("Supply one external force/COM torque per body.", nameof(loads)); }
    }

    private static double KineticEnergy(MultibodySystem system, MultibodyState state)
    {
        double result = 0;
        for (int i = 0; i < state.Bodies.Count; i++)
        { result += RigidBodyDynamics.KineticEnergy(system.Bodies[i], state.Bodies[i]); }
        return result;
    }

    private static void CheckMotion(WheelRailContactSystem system, MultibodyState start, MultibodyState end,
        double step, CompliantContactStepSettings settings)
    {
        for (int i = 0; i < start.Bodies.Count; i++)
        {
            RigidBodyState a = start.Bodies[i];
            RigidBodyState b = end.Bodies[i];
            double angle = step * System.Math.Max(a.BodyAngularVelocity.Length, b.BodyAngularVelocity.Length);
            double extent = 0;
            foreach (WheelDefinition wheel in system.Wheels)
            {
                if (wheel.BodyIndex == i)
                { extent = System.Math.Max(extent, wheel.LocalPose.Position.Length + new Vector3d(wheel.Tread.Radius, wheel.Tread.HalfWidth, 0).Length); }
            }
            double translation = System.Math.Max((b.Pose.Position - a.Pose.Position).Length,
                step * System.Math.Max(a.LinearVelocity.Length, b.LinearVelocity.Length));
            double travel = translation + extent * angle;
            if (!double.IsFinite(travel) || angle > settings.MaximumAngularStep || travel > settings.MaximumPointTravel)
            { throw Resolution("Movement exceeds metre/radian bounds; reduce the timestep."); }
        }
    }

    private static MultibodyState Interpolate(MultibodyState start, MultibodyState end, double fraction)
    {
        var bodies = new RigidBodyState[start.Bodies.Count];
        for (int i = 0; i < bodies.Length; i++)
        {
            RigidBodyState a = start.Bodies[i];
            RigidBodyState b = end.Bodies[i];
            Quaterniond relative = (a.Pose.Orientation.Conjugated() * b.Pose.Orientation).Normalized();
            if (relative.W < 0) { relative *= -1; }
            Vector3d axis = new(relative.X, relative.Y, relative.Z);
            double angle = RotationAngle(relative);
            Quaterniond change = axis.Length == 0 ? Quaterniond.Identity : Quaterniond.FromAxisAngle(axis, angle * fraction);
            bodies[i] = new(new RigidPose3d(a.Pose.Position + (b.Pose.Position - a.Pose.Position) * fraction,
                a.Pose.Orientation * change), a.LinearVelocity + (b.LinearVelocity - a.LinearVelocity) * fraction,
                a.BodyAngularVelocity + (b.BodyAngularVelocity - a.BodyAngularVelocity) * fraction);
        }
        return new(bodies);
    }

    private static double RotationAngle(Quaterniond rotation) =>
        2 * System.Math.Atan2(new Vector3d(rotation.X, rotation.Y, rotation.Z).Length, System.Math.Abs(rotation.W));

    private static ContactSolveException Resolution(string message) =>
        new(ContactSolveFailure.StepResolution, "Compliant step", message);
}
