using System;
using System.Collections.Generic;
using System.Linq;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using QuantumDynamix.Geometry;
using QuantumDynamix.Math;
using QuantumDynamix.Physics;
using QuantumDynamix.Splines.Curves;
using QuantumDynamix.Track;
using static QuantumDynamix.Tests.RailTestFixtures;
using static QuantumDynamix.Tests.RevoluteTestFixtures;

namespace QuantumDynamix.Tests;

// Separate carrier and wheel properties are explicit; no mass is included twice.
[TestClass]
public sealed class RevoluteWheelRailContactTests
{
    public TestContext TestContext { get; set; } = null!;
    private static Quaterniond AxleAlongY => Quaterniond.FromAxisAngle(Vector3d.UnitZ, System.Math.PI / 2);
    private static ContactSolverSettings RigidControls => new(0.5, 0.3, 0.02, 0.05, travelSamples: 2);
    private static CompliantContactStepSettings CompliantControls => new(0.5, 0.3, 0.02, 0.05, travelSamples: 2);

    [TestMethod]
    [DataRow(false, 0.0)]
    [DataRow(true, 0.0)]
    [DataRow(false, 37.0)]
    [DataRow(true, 37.0)]
    [DataRow(false, 500.0)]
    [DataRow(true, 500.0)]
    public void NormalSupportTransmitsBearingLoadAndPreservesArbitraryFrictionlessSpin(bool compliant, double spin)
    {
        var (geometry, state) = ContactPair(gap: compliant ? -0.01 : 0, spin: spin, velocity: new(1.2, 0, 0));
        var model = Model(geometry, 2500);
        const double h = 0.0002;
        // At high spin, witness roundoff perturbs normal point velocity; impulses/h amplify it.
        // These measured-fixture margins are not a general geometry-to-force error guarantee.
        double loadTolerance = spin >= 500 ? 5e-6 : 2e-7;
        double impulseTolerance = System.Math.Max(2e-10, h * loadTolerance);
        double maximumLoadError = 0;
        Assert.AreEqual(2.5, geometry.Dynamics.Bodies.Sum(body => body.Mass));
        for (int i = 0; i < 10; i++)
        {
            if (compliant)
            {
                var result = CompliantContactMultibodyIntegrator.Step(model, state, i * h, h, Gravity, CompliantControls);
                Assert.AreEqual(25, result.AcceptedContacts.Contacts[0].Response.NormalForce, 2e-7);
                TestMath.Near(new(0, 0, 20), result.JointStep.AcceptedDynamics.RevoluteJointReactions[0].ForceOnA, 2e-7);
                Assert.AreEqual(-0.01, result.AcceptedContacts.Contacts[0].Geometry.Gap, 2e-10);
                Assert.AreEqual(0, result.Energy.MidpointDissipation);
                state = result.State;
            }
            else
            {
                var result = ContactMultibodyIntegrator.Step(geometry, state, i * h, h, Gravity, RigidControls);
                Assert.AreEqual(25 * h, result.ContactImpulses[0].NormalImpulse, impulseTolerance);
                TestMath.Near(new(0, 0, 20 * h), result.RevoluteJointImpulses[0].ImpulseOnA, impulseTolerance);
                TestMath.Near(new(0, 0, 20), result.AverageConstraintLoads[0].Force, loadTolerance);
                TestMath.Near(new(0, 0, 5), result.AverageConstraintLoads[1].Force, loadTolerance);
                maximumLoadError = System.Math.Max(maximumLoadError, System.Math.Abs(result.ContactImpulses[0].AverageForce.Z - 25));
                Assert.IsTrue(result.Diagnostics.MaximumJointAngularGap <= 1e-10);
                Assert.IsTrue(result.Diagnostics.MaximumJointAngularSlip <= 1e-10);
                Assert.IsTrue(result.Diagnostics.ScaledVelocityViolation <= 1);
                state = result.State;
            }
        }
        var sample = RevoluteJointKinematics.Evaluate(geometry.Dynamics, state, 0);
        Assert.AreEqual(spin, sample.AngularRate, 2e-9);
        Assert.IsTrue(System.Math.Abs(sample.AngularRate - 1.2 / 0.2) > 1);
        var contact = WheelRailContactGeometry.Query(geometry, state).Contacts[0];
        TestMath.Near(new(1.2 - 0.2 * spin, 0, 0), contact.RelativeVelocity, 2e-7);
        Assert.AreEqual(1.2, state.Bodies[0].LinearVelocity.X, 2e-10);
        TestContext.WriteLine($"compliant={compliant}, spin={spin}: rigid max normal load error={maximumLoadError:E9} N");
    }

    [TestMethod]
    [DataRow(false)]
    [DataRow(true)]
    public void AxlePhasePreservesGeometryAndVelocityFieldForEquivalentWorldMotion(bool transformed)
    {
        Quaterniond rotation = transformed ? Quaterniond.FromAxisAngle(new(1, -2, 3), 0.6) : Quaterniond.Identity;
        Vector3d translation = transformed ? new(10, -7, 3) : Vector3d.Zero;
        var (geometry, _) = ContactPair(rotation: rotation, translation: translation);
        WheelRailContactSample? reference = null;
        Vector3d worldOmega = new(0.4, 5, -0.3), worldVelocity = new(1.1, -0.1, -0.2);
        foreach (double phase in new[] { 0.0, 0.7, 2.8, -3.1 })
        {
            var (_, original) = ContactPair(gap: 0.005, phase: phase);
            RigidBodyState wheel = original.Bodies[1];
            var state = new MultibodyState(new[] { original.Bodies[0], new RigidBodyState(wheel.Pose,
                worldVelocity, wheel.Pose.InverseTransformDirection(worldOmega)) });
            state = Transform(state, rotation, translation);
            var sample = WheelRailContactGeometry.Query(geometry, state).Contacts[0];
            if (reference is WheelRailContactSample expected)
            {
                Assert.AreEqual(expected.Gap, sample.Gap, 2e-10);
                TestMath.Near(expected.WheelPoint, sample.WheelPoint, 1e-8);
                TestMath.Near(expected.RailPoint, sample.RailPoint, 1e-8);
                TestMath.Near(expected.Normal, sample.Normal, 1e-9);
                TestMath.Near(expected.RelativeVelocity, sample.RelativeVelocity, 2e-8);
                Assert.AreEqual(expected.NormalVelocity, sample.NormalVelocity, 2e-8);
            }
            else { reference = sample; }
            TestMath.Near(rotation.Rotate(new(0.1, -0.02, -0.2)), sample.RelativeVelocity, 2e-8);
        }
    }

    [TestMethod]
    public void RigidImpactUsesCombinedMassAndReportsSeparateBearingImpulseAndAverageLoad()
    {
        var (geometry, state) = ContactPair(spin: 37, velocity: new(0, 0, -1));
        var coarse = ContactMultibodyIntegrator.Step(geometry, state, 0, 0.001, NoLoads, RigidControls);
        var fine = ContactMultibodyIntegrator.Step(geometry, state, 0, 0.0005, NoLoads, RigidControls);
        Assert.AreEqual(2.5, coarse.ContactImpulses[0].NormalImpulse, 2e-8);
        Assert.AreEqual(2, coarse.RevoluteJointImpulses[0].ImpulseOnA.Z, 2e-8);
        Assert.AreEqual(2500, coarse.ContactImpulses[0].AverageForce.Z, 2e-5);
        Assert.AreEqual(5000, fine.ContactImpulses[0].AverageForce.Z, 2e-5);
        Assert.AreEqual(37, RevoluteJointKinematics.Evaluate(geometry.Dynamics, coarse.State, 0).AngularRate, 2e-8);
        TestMath.Near(Vector3d.Zero, coarse.State.Bodies[1].LinearVelocity, 2e-8);
        Assert.ThrowsExactly<NotSupportedException>(() => ((IList<RevoluteJointImpulse>)coarse.RevoluteJointImpulses).Clear());
        Assert.AreEqual(coarse.RevoluteJointImpulses[0].ImpulseOnA * -1, coarse.RevoluteJointImpulses[0].ImpulseOnB);
    }

    [TestMethod]
    public void OffsetBearingSeparatesIndependentCoupleFromComMomentAndWheelAxialSpin()
    {
        const double offset = 0.3;
        var (geometry, state) = ContactPair(spin: 37, velocity: new(0, 0, -1), anchor: new(offset, 0, 0));
        var result = ContactMultibodyIntegrator.Step(geometry, state, 0, 0.001, NoLoads, RigidControls);
        double bearingImpulse = 1 / (1 / 2.0 + offset * offset / 2);
        Assert.AreEqual(0.5 + bearingImpulse, result.ContactImpulses[0].NormalImpulse, 2e-8);
        Assert.AreEqual(bearingImpulse, result.RevoluteJointImpulses[0].ImpulseOnA.Z, 2e-8);
        Assert.AreEqual(-offset * bearingImpulse, result.RevoluteJointImpulses[0].AngularImpulseOnA.Y, 2e-8);
        Assert.AreEqual(0, result.RevoluteJointImpulses[0].CoupleImpulseOnA.Y, 2e-10);
        Assert.AreEqual(37, result.State.Bodies[1].WorldAngularVelocity.Y, 2e-8);
        Assert.IsTrue(System.Math.Abs(result.State.Bodies[0].WorldAngularVelocity.Y) > 0.1);
    }

    [TestMethod]
    public void RigidBearingImpulseContainsAnalyticalGyroscopicCoupleBeforeNumericalCorrection()
    {
        var (geometry, initial) = ContactPair(spin: 80);
        var state = WithTurning(initial, 80, 0.7);
        const double h = 0.001;
        var result = ContactMultibodyIntegrator.Step(geometry, state, 0, h, Gravity, RigidControls);
        double couple = 2 * 0.02 * 80 * 0.7 / (2 + 0.03);
        // Axle is +Y: +Z turning cross +Y momentum points along -X on the wheel.
        TestMath.Near(new(-couple, 0, 0), result.RevoluteJointImpulses[0].CoupleImpulseOnB / h, 2e-6);
        Assert.IsTrue(result.Diagnostics.MaximumJointAngularGap <= 1e-10);
        Assert.IsTrue(result.Diagnostics.MaximumJointAngularSlip <= 1e-10);
        Assert.IsTrue(result.Diagnostics.MaximumAngularVelocityViolation <= 1e-10);
        Vector3d sum = result.AverageConstraintLoads[0].Force + result.AverageConstraintLoads[1].Force;
        TestMath.Near(result.ContactImpulses[0].AverageForce, sum, 2e-8);
    }

    [TestMethod]
    public void CompliantTurningWheelReportsProjectionEnergyAndEndpointBearingReaction()
    {
        var (geometry, initial) = ContactPair(gap: -0.001, spin: 80);
        var state = WithTurning(initial, 80, 0.7);
        var model = Model(geometry, 25000);
        var result = CompliantContactMultibodyIntegrator.Step(model, state, 0, 0.001, Gravity, CompliantControls);
        var contacts = CompliantWheelRailContact.Evaluate(model, result.State);
        var gravity = Gravity(0.001, result.State);
        var evaluated = ConstrainedMultibodyDynamics.Evaluate(geometry.Dynamics, result.State,
            new[] { gravity[0] + contacts.BodyLoads[0], gravity[1] + contacts.BodyLoads[1] });
        Assert.AreEqual(evaluated.RevoluteJointReactions[0], result.JointStep.AcceptedDynamics.RevoluteJointReactions[0]);
        Assert.AreEqual(Energy(geometry.Dynamics, result.State) - Energy(geometry.Dynamics, result.UnprojectedState), result.Energy.ProjectionKineticEnergyChange, 1e-13);
        Assert.AreEqual(result.AcceptedContacts.ElasticEnergy - result.UnprojectedContacts.ElasticEnergy, result.Energy.ProjectionElasticEnergyChange, 1e-13);
        Assert.IsTrue(result.JointStep.Diagnostics.PositionProjectionIterations > 0);
        Assert.IsTrue(result.AcceptedContacts.Contacts[0].Geometry.Gap < 0);
        Assert.AreEqual(0, result.Energy.MidpointDissipation);
        TestContext.WriteLine($"Turning compliant wheel: numerical kinetic change={result.Energy.ProjectionKineticEnergyChange:E9} J; elastic change={result.Energy.ProjectionElasticEnergyChange:E9} J; alignment={result.JointStep.AcceptedDynamics.Diagnostics.MaximumAngularPositionError:E6} rad");
    }

    [TestMethod]
    [DataRow(false)]
    [DataRow(true)]
    public void WorldOrientationAndMixedJointsPreserveSupportAndTypedReactionSigns(bool compliant)
    {
        Quaterniond rotation = Quaterniond.FromAxisAngle(new(1, -2, 3), 0.6);
        var (geometry, state) = ContactPair(gap: compliant ? -0.01 : 0, spin: 37, mixed: true,
            rotation: rotation, translation: new(10, -7, 3));
        Vector3d normal = rotation.Rotate(Vector3d.UnitZ);
        IReadOnlyList<RigidBodyLoads> Loads(double time, MultibodyState snapshot) =>
            new[] { new RigidBodyLoads(normal * -20, default), new(normal * -5, default), new(normal * -30, default) };
        if (compliant)
        {
            var result = CompliantContactMultibodyIntegrator.Step(Model(geometry, 5500), state, 0, 0.001, Loads, CompliantControls);
            Assert.AreEqual(55, result.AcceptedContacts.Contacts[0].Response.NormalForce, 2e-6);
            TestMath.Near(normal * 50, result.JointStep.AcceptedDynamics.RevoluteJointReactions[0].ForceOnA, 2e-6);
            TestMath.Near(normal * -30, result.JointStep.AcceptedDynamics.JointReactions[0].ForceOnA, 2e-6);
        }
        else
        {
            var result = ContactMultibodyIntegrator.Step(geometry, state, 0, 0.001, Loads, RigidControls);
            Assert.AreEqual(0.055, result.ContactImpulses[0].NormalImpulse, 2e-8);
            TestMath.Near(normal * 0.05, result.RevoluteJointImpulses[0].ImpulseOnA, 2e-8);
            TestMath.Near(normal * -0.03, result.JointImpulses[0].ImpulseOnA, 2e-8);
            Assert.AreEqual(8, geometry.Dynamics.BilateralConstraintCount);
        }
    }

    [TestMethod]
    public void CompliantWheelAndCarrierOscillatorHasAnalyticalMotionAndSecondOrderRefinement()
    {
        var (geometry, initial) = ContactPair(spin: 37, velocity: new(0, 0, -0.1));
        var model = Model(geometry, 2500);
        double frequency = System.Math.Sqrt(2500 / 2.5), duration = 0.06;
        double expectedGap = -0.1 / frequency * System.Math.Sin(frequency * duration);
        double expectedVelocity = -0.1 * System.Math.Cos(frequency * duration);
        double[] errors = new double[3];
        for (int refinement = 0; refinement < 3; refinement++)
        {
            MultibodyState state = initial;
            int count = 20 << refinement;
            double h = duration / count;
            for (int i = 0; i < count; i++)
            {
                var result = CompliantContactMultibodyIntegrator.Step(model, state, i * h, h, NoLoads, CompliantControls);
                Assert.AreEqual(0, result.Energy.MidpointDissipation);
                state = result.State;
            }
            errors[refinement] = System.Math.Abs((state.Bodies[1].Pose.Position.Z - 0.2 - expectedGap) / 0.01)
                + System.Math.Abs((state.Bodies[1].LinearVelocity.Z - expectedVelocity) / 0.1);
            Assert.AreEqual(37, RevoluteJointKinematics.Evaluate(geometry.Dynamics, state, 0).AngularRate, 2e-8);
        }
        TestContext.WriteLine($"Independent wheel/carrier normal oscillator errors={errors[0]:E9}/{errors[1]:E9}/{errors[2]:E9}");
        Assert.IsTrue(errors[0] / errors[1] > 3.8 && errors[0] / errors[1] < 4.2);
        Assert.IsTrue(errors[1] / errors[2] > 3.8 && errors[1] / errors[2] < 4.2);
    }

    [TestMethod]
    public void RigidHingeTrajectoryConvergesAtFirstOrderAgainstMidpointReference()
    {
        var (dynamics, initial) = GeneralPair();
        var geometry = new WheelRailContactSystem(dynamics, Straight(Rail("rail", default, 10)),
            Array.Empty<WheelDefinition>(), Array.Empty<RailContactPatch>(), Array.Empty<WheelRailPair>());
        MultibodyState reference = initial;
        for (int i = 0; i < 320; i++) { reference = ConstrainedMultibodyIntegrator.Step(dynamics, reference, i * 0.00125, 0.00125, NoLoads).State; }
        double[] errors = new double[3];
        for (int refinement = 0; refinement < 3; refinement++)
        {
            int count = 20 << refinement;
            double h = 0.4 / count;
            MultibodyState state = initial;
            for (int i = 0; i < count; i++) { state = ContactMultibodyIntegrator.Step(geometry, state, i * h, h, NoLoads, RigidControls).State; }
            for (int i = 0; i < 2; i++)
            {
                errors[refinement] += (state.Bodies[i].Pose.Position - reference.Bodies[i].Pose.Position).Length
                    + (state.Bodies[i].LinearVelocity - reference.Bodies[i].LinearVelocity).Length
                    + (state.Bodies[i].WorldAngularVelocity - reference.Bodies[i].WorldAngularVelocity).Length;
            }
        }
        TestContext.WriteLine($"Rigid hinge refinement errors={errors[0]:E9}/{errors[1]:E9}/{errors[2]:E9}");
        Assert.IsTrue(errors[0] / errors[1] > 1.7 && errors[0] / errors[1] < 2.3);
        Assert.IsTrue(errors[1] / errors[2] > 1.7 && errors[1] / errors[2] < 2.3);
    }

    [TestMethod]
    public void SeparateBodiesRetainFormulationExclusionAndExplicitResolutionFailures()
    {
        var (geometry, state) = ContactPair(spin: 500);
        var model = Model(geometry, 2500);
        var error = Assert.ThrowsExactly<ContactSolveException>(() => ContactMultibodyIntegrator.Step(geometry, state, 0, 0.001,
            NoLoads, RigidControls));
        Assert.AreEqual(ContactSolveFailure.StepResolution, error.Failure);
        error = Assert.ThrowsExactly<ContactSolveException>(() => CompliantContactMultibodyIntegrator.Step(model, state, 0, 0.001,
            NoLoads, CompliantControls));
        Assert.AreEqual(ContactSolveFailure.StepResolution, error.Failure);
        var (slowGeometry, slowState) = ContactPair();
        var slowModel = Model(slowGeometry, 2500);
        error = Assert.ThrowsExactly<ContactSolveException>(() => ContactMultibodyIntegrator.Step(slowGeometry, slowState, 0, 0.001,
            (_, snapshot) => CompliantWheelRailContact.Evaluate(slowModel, snapshot).BodyLoads, RigidControls));
        Assert.AreEqual(ContactSolveFailure.MixedFormulations, error.Failure);
        error = Assert.ThrowsExactly<ContactSolveException>(() => CompliantContactMultibodyIntegrator.Step(slowModel, slowState, 0, 0.001,
            (_, snapshot) => ContactMultibodyIntegrator.Step(slowGeometry, snapshot, 0, 0.001, NoLoads, RigidControls).AverageConstraintLoads, CompliantControls));
        Assert.AreEqual(ContactSolveFailure.MixedFormulations, error.Failure);
        Assert.AreEqual(500, RevoluteJointKinematics.Evaluate(geometry.Dynamics, state, 0).AngularRate, 2e-12);
    }

    [TestMethod]
    [DataRow(false)]
    [DataRow(true)]
    public void SeparatingIndependentWheelHasNoAttractionOrAxialSpinCorrection(bool compliant)
    {
        var (geometry, state) = ContactPair(gap: 0.01, spin: 37, velocity: new(0, 0, 0.5));
        MultibodyState accepted;
        if (compliant)
        {
            var result = CompliantContactMultibodyIntegrator.Step(Model(geometry, 2500), state, 0, 0.001, NoLoads, CompliantControls);
            Assert.AreEqual(0, result.AcceptedContacts.Contacts[0].Response.NormalForce);
            accepted = result.State;
        }
        else
        {
            var result = ContactMultibodyIntegrator.Step(geometry, state, 0, 0.001, NoLoads, RigidControls);
            Assert.AreEqual(0, result.ContactImpulses[0].NormalImpulse);
            accepted = result.State;
        }
        Assert.AreEqual(0.0105, WheelRailContactGeometry.Query(geometry, accepted).Contacts[0].Gap, 2e-10);
        Assert.AreEqual(37, RevoluteJointKinematics.Evaluate(geometry.Dynamics, accepted, 0).AngularRate, 2e-9);
    }

    [TestMethod]
    [DataRow(false)]
    [DataRow(true)]
    public void InitialAngularInconsistencyInBothContactPathsFailsBeforeExternalLoads(bool compliant)
    {
        var (geometry, initial) = ContactPair();
        RigidBodyState b = initial.Bodies[1];
        var state = new MultibodyState(new[] { initial.Bodies[0], new RigidBodyState(b.Pose, b.LinearVelocity,
            b.Pose.InverseTransformDirection(new(0, 0, 0.001))) });
        int calls = 0;
        IReadOnlyList<RigidBodyLoads> Loads(double t, MultibodyState s) { calls++; return NoLoads(t, s); }
        if (compliant)
        {
            var error = Assert.ThrowsExactly<ConstrainedSolveException>(() => CompliantContactMultibodyIntegrator.Step(
                Model(geometry, 2500), state, 0, 0.001, Loads, CompliantControls));
            Assert.AreEqual(ConstrainedSolveFailure.InconsistentInitialState, error.Failure);
        }
        else
        {
            var error = Assert.ThrowsExactly<ContactSolveException>(() => ContactMultibodyIntegrator.Step(geometry, state,
                0, 0.001, Loads, RigidControls));
            Assert.AreEqual(ContactSolveFailure.InconsistentInitialState, error.Failure);
        }
        Assert.AreEqual(0, calls);
        Assert.AreEqual(0.001, state.Bodies[1].WorldAngularVelocity.Z, 1e-15);
    }

    private static IReadOnlyList<RigidBodyLoads> Gravity(double time, MultibodyState state) =>
        new[] { new RigidBodyLoads(new(0, 0, -20), default), new(new(0, 0, -5), default) };

    private static CompliantWheelRailContactModel Model(WheelRailContactSystem geometry, double stiffness) =>
        new(geometry, new[] { KeyValuePair.Create(geometry.Pairs[0], new PowerLawNormalContact(stiffness, 1, 0, 0.04)) });

    private static MultibodyState WithTurning(MultibodyState initial, double spin, double turning)
    {
        RigidBodyState a = initial.Bodies[0], b = initial.Bodies[1];
        return new(new[] { new RigidBodyState(a.Pose, a.LinearVelocity, new(0, 0, turning)),
            new RigidBodyState(b.Pose, b.LinearVelocity, b.Pose.InverseTransformDirection(new(0, spin, turning))) });
    }

    private static (WheelRailContactSystem Geometry, MultibodyState State) ContactPair(double gap = 0, double phase = 0,
        double spin = 0, Vector3d velocity = default, Vector3d anchor = default, bool mixed = false,
        Quaterniond? rotation = null, Vector3d translation = default)
    {
        Quaterniond transform = rotation ?? Quaterniond.Identity;
        RigidBodyMassProperties[] bodies = {
            new(2, Matrix3x3d.Diagonal(2, 2, 2)), new(0.5, Matrix3x3d.Diagonal(0.02, 0.03, 0.03)),
            new(3, Matrix3x3d.Identity) };
        var dynamics = new MultibodySystem(mixed ? bodies : bodies.Take(2),
            mixed ? new[] { new SphericalJoint(0, 2, default, default) } : Array.Empty<SphericalJoint>(),
            new[] { new RevoluteJoint(0, 1, new(anchor, AxleAlongY), RigidPose3d.Identity) });
        var line = SplineCurve3d.Clamped(new[] { translation, translation + transform.Rotate(new(10, 0, 0)) }, 1);
        var rails = new RailGeometry(new(line, transform.Rotate(Vector3d.UnitZ), TrackOffsetProfile.Constant(default, 10)),
            new(new[] { Rail("rail", default, 10) }), Settings);
        var wheel = new WheelDefinition("wheel", 1, RigidPose3d.Identity, new(0.2, 0.08, 0.4));
        var geometry = new WheelRailContactSystem(dynamics, rails, new[] { wheel },
            new[] { new RailContactPatch("surface", "rail", 0, 0, 10, 0, 1, RailMaterialSide.PositiveAreaVector) },
            new[] { new WheelRailPair("wheel", "surface") });
        // Keep this isolated witness away from a cached rail-frame node: the existing frame
        // evaluator can reject sub-ulp displacements next to a node at a translated world origin.
        var a = new RigidBodyState(new(new(5.1, 0, 0.2 + gap), Quaterniond.Identity), velocity, default);
        var b = new RigidBodyState(new(a.Pose.Position + anchor, AxleAlongY * Quaterniond.FromAxisAngle(Vector3d.UnitX, phase)), velocity, new(spin, 0, 0));
        return (geometry, Transform(new(mixed ? new[] { a, b, a } : new[] { a, b }), transform, translation));
    }
}
