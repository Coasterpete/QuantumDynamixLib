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

namespace QuantumDynamix.Tests;

// Synthetic SI fixtures only: no manufacturer data or coaster acceptance criteria.
[TestClass]
public sealed class CompliantWheelRailContactTests
{
    public TestContext TestContext { get; set; } = null!;
    private static readonly Quaterniond AxleAlongY = Quaterniond.FromAxisAngle(Vector3d.UnitZ, System.Math.PI / 2);
    private static CompliantContactStepSettings Controls => new(0.5, 0.2, 0.02, 0.05, travelSamples: 2);
    private static RigidBodyMassProperties Mass(double mass = 2) => new(mass, Matrix3x3d.Diagonal(2, 3, 4));
    private static WheelDefinition Wheel(string id = "wheel", int body = 0, Vector3d offset = default) =>
        new(id, body, new(offset, AxleAlongY), new(0.2, 0.08, 0.4));
    private static RailContactPatch Patch(string id = "surface", string rail = "rail", RailMaterialSide side = RailMaterialSide.PositiveAreaVector) =>
        new(id, rail, 0, 0, 10, 0, 1, side);
    private static MultibodyState State(double gap = 0, Vector3d velocity = default) => new(new[] {
        new RigidBodyState(new(new(5, 0, 0.2 + gap), Quaterniond.Identity), velocity, default) });
    private static IReadOnlyList<RigidBodyLoads> NoLoads(double time, MultibodyState state) => new RigidBodyLoads[state.Bodies.Count];
    private static CompliantWheelRailContactModel Model(double coefficient = 2000, double exponent = 1, double dissipation = 0,
        WheelDefinition[]? wheels = null, RigidBodyMassProperties[]? masses = null, SphericalJoint[]? joints = null,
        double maximumCompression = 0.04)
    {
        wheels ??= new[] { Wheel() };
        var geometry = new WheelRailContactSystem(new(masses ?? new[] { Mass() }, joints ?? Array.Empty<SphericalJoint>()),
            Straight(Rail("rail", default, 10)), wheels, new[] { Patch() }, wheels.Select(w => new WheelRailPair(w.Id, "surface")));
        return Assign(geometry, geometry.Pairs.Select(_ => new PowerLawNormalContact(coefficient, exponent, dissipation, maximumCompression)).ToArray());
    }
    private static CompliantWheelRailContactModel Assign(WheelRailContactSystem geometry, params PowerLawNormalContact[] laws) =>
        new(geometry, geometry.Pairs.Select((pair, i) => KeyValuePair.Create(pair, laws[i])));

    [TestMethod]
    [DataRow(1.0)]
    [DataRow(1.5)]
    [DataRow(2.0)]
    public void PowerLawHasAnalyticalForceEnergyTangentAndFiniteZeroLimit(double exponent)
    {
        var law = new PowerLawNormalContact(10000, exponent, 0, 0.02);
        NormalContactResponse response = law.Evaluate(0.01, 3);
        double elastic = 10000 * System.Math.Pow(0.01, exponent);
        Assert.AreEqual(elastic, response.NormalForce, elastic * 1e-14);
        Assert.AreEqual(10000 / (exponent + 1) * System.Math.Pow(0.01, exponent + 1), response.ElasticEnergy, 1e-15);
        Assert.AreEqual(exponent * 10000 * System.Math.Pow(0.01, exponent - 1), response.ElasticTangentStiffness, 1e-10);
        NormalContactResponse zero = law.Evaluate(0, -1e100);
        Assert.AreEqual(0, zero.NormalForce);
        Assert.AreEqual(0, zero.ElasticEnergy);
        Assert.AreEqual(0, zero.DissipationPower);
        Assert.AreEqual(exponent == 1 ? 10000 : 0, zero.ElasticTangentStiffness);
        // Independent central difference of stored energy checks dU/ddelta=Fe.
        double difference = (law.Evaluate(0.0100001, 0).ElasticEnergy - law.Evaluate(0.0099999, 0).ElasticEnergy) / 2e-7;
        Assert.AreEqual(elastic, difference, elastic * 1e-9);
    }

    [TestMethod]
    [DataRow(0.1, 110.0, 1.0)]
    [DataRow(-0.1, 90.0, 1.0)]
    [DataRow(-2.0, 0.0, 200.0)]
    public void LoadingUnloadingAndClippingHaveNoAttractionAndCorrectPower(double rate, double expectedForce, double expectedLoss)
    {
        NormalContactResponse response = new PowerLawNormalContact(10000, 1, 1, 0.02).Evaluate(0.01, rate);
        Assert.AreEqual(expectedForce, response.NormalForce, 1e-12);
        Assert.AreEqual(expectedLoss, response.DissipationPower, 1e-12);
        // d(T+U)/dt=(Fe-Fn)*deltaDot=-physicalLoss, including the clipped branch.
        Assert.AreEqual(-expectedLoss, (response.ElasticForce - response.NormalForce) * rate, 1e-12);
    }

    [TestMethod]
    [DataRow(-1.0)]
    [DataRow(0.0)]
    [DataRow(0.5)]
    [DataRow(double.NaN)]
    [DataRow(double.PositiveInfinity)]
    public void UndefinedOrSingularExponentsAreRejected(double exponent) =>
        Assert.ThrowsExactly<ArgumentOutOfRangeException>(() => new PowerLawNormalContact(2000, exponent, 0, 0.02));

    [TestMethod]
    public void LawRangeAndArithmeticAreExplicitWithoutSilentZeroSupport()
    {
        Assert.ThrowsExactly<ArgumentOutOfRangeException>(() => new PowerLawNormalContact(0, 1, 0, 0.02));
        Assert.ThrowsExactly<ArgumentOutOfRangeException>(() => new PowerLawNormalContact(1, 1, -1, 0.02));
        Assert.ThrowsExactly<ArgumentOutOfRangeException>(() => new PowerLawNormalContact(1, 1, 0, double.NaN));
        Assert.ThrowsExactly<ArgumentException>(() => new PowerLawNormalContact(1e-300, 2, 0, 1e-100));
        Assert.ThrowsExactly<ArgumentException>(() => new PowerLawNormalContact(1e300, 2, 0, 1e100));
        var law = new PowerLawNormalContact(2000, 1.5, 0, 0.02);
        Assert.ThrowsExactly<ArgumentOutOfRangeException>(() => law.Evaluate(-1, 0));
        Assert.ThrowsExactly<ArgumentOutOfRangeException>(() => law.Evaluate(0.021, 0));
        Assert.ThrowsExactly<ArgumentOutOfRangeException>(() => law.Evaluate(0, double.NaN));
        Assert.ThrowsExactly<ArithmeticException>(() => law.Evaluate(1e-300, 0));
        // K*delta^p resolves although delta^p by itself underflows.
        NormalContactResponse scaled = new PowerLawNormalContact(1e300, 4, 0, 0.01).Evaluate(1e-100, 0);
        Assert.AreEqual(1, scaled.NormalForce / 1e-100, 1e-12);
        Assert.AreEqual(1, scaled.ElasticEnergy / 2e-201, 1e-12);
    }

    [TestMethod]
    public void ThreeCollinearSupportsShareLoadByAnalyticalStiffnessAndRetainCompression()
    {
        var geometry = Model(wheels: new[] { Wheel("a", offset: new(-0.5, 0, 0)), Wheel("b"), Wheel("c", offset: new(0.5, 0, 0)) }).Geometry;
        var model = Assign(geometry, new(1e6, 1, 0, 0.02), new(2e6, 1, 0, 0.02), new(1e6, 1, 0, 0.02));
        // Static force/moment balance: delta=4000/(1e6+2e6+1e6)=0.001 m.
        MultibodyState initial = State(-0.001);
        static IReadOnlyList<RigidBodyLoads> Loads(double t, MultibodyState s) => new[] { new RigidBodyLoads(new(0, 0, -4000), default) };
        CompliantContactStepResult result = CompliantContactMultibodyIntegrator.Step(model, initial, 0, 0.001, Loads, Controls);
        double[] expected = { 1000, 2000, 1000 };
        for (int i = 0; i < expected.Length; i++)
        {
            Assert.AreEqual(expected[i], result.AcceptedContacts.Contacts[i].Response.NormalForce, 1e-7);
            Assert.AreEqual(-0.001, result.AcceptedContacts.Contacts[i].Geometry.Gap, 1e-11);
        }
        TestMath.Near(Vector3d.Zero, result.State.Bodies[0].LinearVelocity, 1e-10);
        TestMath.Near(Vector3d.Zero, result.State.Bodies[0].BodyAngularVelocity, 1e-10);
        Assert.AreEqual(2, result.AcceptedContacts.ElasticEnergy, 1e-10);
        Assert.AreEqual(0, result.JointStep.ProjectionCorrections[0].PositionChange.Length);
        ContactSolveException rigid = Assert.ThrowsExactly<ContactSolveException>(() => ContactMultibodyIntegrator.Step(geometry,
            State(), 0, 0.001, Loads, new(0.5, 0.2, 0.02, 0.05)));
        Assert.AreEqual(ContactSolveFailure.IndeterminateReactions, rigid.Failure);
    }

    [TestMethod]
    public void ClearancesUnloadTheMiddleSupportWithoutRedistributingByWheelCount()
    {
        var model = Model(1e6, wheels: new[] { Wheel("a", offset: new(-0.5, 0, 0)),
            Wheel("b", offset: new(0, 0, 0.002)), Wheel("c", offset: new(0.5, 0, 0)) });
        CompliantContactStepResult result = CompliantContactMultibodyIntegrator.Step(model, State(-0.001), 0, 0.001,
            (_, _) => new[] { new RigidBodyLoads(new(0, 0, -2000), default) }, Controls);
        Assert.AreEqual(1000, result.AcceptedContacts.Contacts[0].Response.NormalForce, 1e-7);
        Assert.AreEqual(0, result.AcceptedContacts.Contacts[1].Response.NormalForce);
        Assert.AreEqual(0.001, result.AcceptedContacts.Contacts[1].Geometry.Gap, 1e-10);
        Assert.AreEqual(1000, result.AcceptedContacts.Contacts[2].Response.NormalForce, 1e-7);
    }

    [TestMethod]
    public void OpposingRunningAndUpstopPreloadComesFromInstalledGeometry()
    {
        // Two planar faces 0.1 m apart. Wheel centers are +0.199 and -0.299 m:
        // each 0.2 m tread compresses 0.001 m into its respective face.
        var geometry = new WheelRailContactSystem(new(new[] { Mass() }, Array.Empty<SphericalJoint>()),
            Straight(Rail("top", default, 10), Rail("bottom", new(0, 0, -0.1), 10)),
            new[] { Wheel("running"), Wheel("upstop", offset: new(0, 0, -0.498)) },
            new[] { Patch("top", "top"), Patch("bottom", "bottom", RailMaterialSide.NegativeAreaVector) },
            new[] { new WheelRailPair("running", "top"), new("upstop", "bottom") });
        var model = Assign(geometry, new(1e6, 1, 0, 0.02), new(1e6, 1, 0, 0.02));
        CompliantContactStepResult result = CompliantContactMultibodyIntegrator.Step(model, State(-0.001), 0, 0.001, NoLoads, Controls);
        Assert.AreEqual(1000, result.AcceptedContacts.Contacts[0].WorldForce.Z, 1e-7);
        Assert.AreEqual(-1000, result.AcceptedContacts.Contacts[1].WorldForce.Z, 1e-7);
        TestMath.Near(Vector3d.Zero, result.AcceptedContacts.BodyLoads[0].Force, 1e-7);
        TestMath.Near(Vector3d.Zero, result.State.Bodies[0].LinearVelocity, 1e-9);
        CompliantContactEvaluation raised = CompliantWheelRailContact.Evaluate(model, State(0.0005));
        Assert.AreEqual(0, raised.Contacts[0].Response.NormalForce);
        Assert.AreEqual(2500, raised.Contacts[1].Response.NormalForce, 1e-7);
    }

    [TestMethod]
    public void SphericalJointTransmitsAnalyticalSupportAndEndpointReactions()
    {
        var model = Model(5000, masses: new[] { Mass(2), Mass(3) }, joints: new[] { new SphericalJoint(0, 1, default, default) });
        RigidBodyState body = State(-0.01).Bodies[0];
        var state = new MultibodyState(new[] { body, body });
        CompliantContactStepResult result = CompliantContactMultibodyIntegrator.Step(model, state, 0, 0.001,
            (_, _) => new[] { new RigidBodyLoads(new(0, 0, -20), default), new(new(0, 0, -30), default) }, Controls);
        Assert.AreEqual(50, result.AcceptedContacts.Contacts[0].Response.NormalForce, 1e-9);
        TestMath.Near(new(0, 0, -30), result.JointStep.AcceptedDynamics.JointReactions[0].ForceOnA, 1e-9);
        foreach (RigidBodyState accepted in result.State.Bodies) { TestMath.Near(Vector3d.Zero, accepted.LinearVelocity, 1e-10); }
        Assert.IsTrue(result.JointStep.AcceptedDynamics.Diagnostics.MaximumPositionError <= Controls.Joints.PositionTolerance);
        // Reactions are instantaneous at the accepted compressed geometry, rather than impulses/h.
        var contact = CompliantWheelRailContact.Evaluate(model, result.State);
        var direct = ConstrainedMultibodyDynamics.Evaluate(model.Geometry.Dynamics, result.State,
            new[] { contact.BodyLoads[0] + new RigidBodyLoads(new(0, 0, -20), default), new(new(0, 0, -30), default) });
        Assert.AreEqual(direct.JointReactions[0], result.JointStep.AcceptedDynamics.JointReactions[0]);
    }

    [TestMethod]
    public void ForceAtOffsetHasAnalyticalComTorqueAndNoTangentialForce()
    {
        var model = Model(wheels: new[] { Wheel(offset: new(0.3, 0, 0)) });
        var contact = CompliantWheelRailContact.Evaluate(model, State(-0.01, new(2, 0.1, 0))).Contacts[0];
        TestMath.Near(new(0, 0, 20), contact.WorldForce, 1e-10);
        // Moment resolution includes witness-location error (geometry tolerance is 1e-9 m).
        TestMath.Near(new(0, -6, 0), contact.WorldTorque, 3e-8);
        Assert.AreEqual(0, contact.Response.DissipationPower);
    }

    [TestMethod]
    public void TiltAndMomentLeaveOneWheelUnloadedWithAnalyticalForceAndTorque()
    {
        const double angle = 0.01;
        var model = Model(10000, wheels: new[] { Wheel("front", offset: new(0.5, 0, 0)), Wheel("rear", offset: new(-0.5, 0, 0)) });
        var state = new MultibodyState(new[] { new RigidBodyState(new(new Vector3d(5, 0, 0.2),
            Quaterniond.FromAxisAngle(Vector3d.UnitY, angle)), default, default) });
        // Pitch lowers the front center by a*sin(angle); radius stays 0.2 m because
        // this rotation is about the wheel axle. The rear wheel has positive clearance.
        double force = 10000 * 0.5 * System.Math.Sin(angle);
        double moment = force * 0.5 * System.Math.Cos(angle);
        var result = CompliantContactMultibodyIntegrator.Step(model, state, 0, 0.001,
            (_, _) => new[] { new RigidBodyLoads(new(0, 0, -force), new(0, moment, 0)) }, Controls);
        Assert.AreEqual(force, result.AcceptedContacts.Contacts[0].Response.NormalForce, 1e-7);
        Assert.AreEqual(0, result.AcceptedContacts.Contacts[1].Response.NormalForce);
        TestMath.Near(new(0, -moment, 0), result.AcceptedContacts.BodyLoads[0].Torque, 1e-7);
        TestMath.Near(Vector3d.Zero, result.State.Bodies[0].LinearVelocity, 1e-9);
        TestMath.Near(Vector3d.Zero, result.State.Bodies[0].BodyAngularVelocity, 1e-9);
    }

    [TestMethod]
    public void ClippedUnloadingRemainsForceFreeBeforeGeometricSeparation()
    {
        var model = Model(dissipation: 2);
        var first = CompliantContactMultibodyIntegrator.Step(model, State(-0.001, new(0, 0, 1)), 0, 0.0005, NoLoads, Controls);
        Assert.IsTrue(first.AcceptedContacts.Contacts[0].Response.Compression > 0);
        Assert.AreEqual(0, first.AcceptedContacts.Contacts[0].Response.NormalForce);
        Assert.IsTrue(first.Energy.MidpointDissipation > 0);
        Assert.AreEqual(1, first.State.Bodies[0].LinearVelocity.Z, 1e-12);
        var second = CompliantContactMultibodyIntegrator.Step(model, first.State, 0.0005, 0.001, NoLoads, Controls);
        Assert.IsTrue(second.AcceptedContacts.Contacts[0].Geometry.Gap > 0);
        Assert.AreEqual(0, second.AcceptedContacts.Contacts[0].Response.NormalForce);
        Assert.AreEqual(1, second.State.Bodies[0].LinearVelocity.Z, 1e-12);
    }

    [TestMethod]
    [DataRow(1.0)]
    [DataRow(1.5)]
    public void SingleSupportEquilibriumAcceptsPhysicalCompression(double exponent)
    {
        const double compression = 0.005;
        double weight = 2000 * System.Math.Pow(compression, exponent);
        var model = Model(exponent: exponent);
        CompliantContactStepResult result = CompliantContactMultibodyIntegrator.Step(model, State(-compression), 0, 0.002,
            (_, _) => new[] { new RigidBodyLoads(new(0, 0, -weight), default) }, Controls);
        Assert.AreEqual(weight, result.AcceptedContacts.Contacts[0].Response.NormalForce, 1e-10);
        Assert.AreEqual(-compression, result.AcceptedContacts.Contacts[0].Geometry.Gap, 1e-11);
        TestMath.Near(Vector3d.Zero, result.State.Bodies[0].LinearVelocity, 1e-10);
    }

    [TestMethod]
    public void SmoothElasticOscillationHasAnalyticalPhaseEnergyAndSecondOrderRefinement()
    {
        var model = Model();
        const double duration = 0.06;
        double omega = System.Math.Sqrt(2000.0 / 2);
        double expectedGap = -0.1 / omega * System.Math.Sin(omega * duration);
        double expectedVelocity = -0.1 * System.Math.Cos(omega * duration);
        double[] errors = new double[3];
        for (int refinement = 0; refinement < 3; refinement++)
        {
            int count = 10 << refinement;
            double h = duration / count;
            MultibodyState state = State(velocity: new(0, 0, -0.1));
            for (int i = 0; i < count; i++)
            {
                CompliantContactStepResult result = CompliantContactMultibodyIntegrator.Step(model, state, i * h, h, NoLoads, Controls);
                Assert.AreEqual(0.01, result.Energy.AcceptedMechanicalEnergy, 1e-10);
                Assert.AreEqual(0, result.Energy.MidpointDissipation);
                Assert.AreEqual(0, result.Energy.ProjectionElasticEnergyChange);
                state = result.State;
            }
            errors[refinement] = System.Math.Abs((state.Bodies[0].Pose.Position.Z - 0.2 - expectedGap) / 0.01)
                + System.Math.Abs((state.Bodies[0].LinearVelocity.Z - expectedVelocity) / 0.1);
        }
        TestContext.WriteLine($"Smooth contact errors: {errors[0]:E6}, {errors[1]:E6}, {errors[2]:E6}");
        Assert.IsTrue(errors[0] / errors[1] > 3.8 && errors[0] / errors[1] < 4.2);
        Assert.IsTrue(errors[1] / errors[2] > 3.8 && errors[1] / errors[2] < 4.2);
    }

    [TestMethod]
    [DataRow(1.0, 2000.0)]
    [DataRow(1.5, 20000.0)]
    public void ElasticRecontactUnloadsAndSeparatesWithEnergyBasedPeakCompression(double exponent, double coefficient)
    {
        var model = Model(coefficient, exponent);
        MultibodyState state = State(0.002, new(0, 0, -0.1));
        double maximum = 0;
        double maximumEnergyError = 0;
        CompliantContactStepResult? last = null;
        for (int i = 0; i < 180; i++)
        {
            last = CompliantContactMultibodyIntegrator.Step(model, state, i * 0.001, 0.001, NoLoads, Controls);
            Assert.IsTrue(last.AcceptedContacts.Contacts[0].Response.NormalForce >= 0);
            maximum = System.Math.Max(maximum, last.AcceptedContacts.Contacts[0].Response.Compression);
            maximumEnergyError = System.Math.Max(maximumEnergyError, System.Math.Abs(last.Energy.AcceptedMechanicalEnergy - 0.01));
            if (i == 0) { Assert.AreEqual(0, last.AcceptedContacts.Contacts[0].Response.NormalForce); }
            state = last.State;
        }
        double expected = System.Math.Pow((exponent + 1) * 2 * 0.1 * 0.1 / (2 * coefficient), 1 / (exponent + 1));
        TestContext.WriteLine($"p={exponent}: sampled peak={maximum:E9} m, analytic={expected:E9} m; max energy error={maximumEnergyError:E6} J");
        Assert.AreEqual(expected, maximum, expected * 0.002);
        Assert.IsTrue(maximumEnergyError < 2e-5);
        Assert.IsTrue(last!.AcceptedContacts.Contacts[0].Geometry.Gap > 0);
        Assert.AreEqual(0, last.AcceptedContacts.Contacts[0].Response.NormalForce);
        Assert.AreEqual(0.1, state.Bodies[0].LinearVelocity.Z, 2e-4);
    }

    [TestMethod]
    public void DampingLossAndMidpointEnergyBalanceConvergeUnderRefinement()
    {
        var model = Model(coefficient: 20000, exponent: 1.5, dissipation: 2);
        double initialEnergy = 20000 / 2.5 * System.Math.Pow(0.004, 2.5);
        double[] balanceErrors = new double[2];
        for (int refinement = 0; refinement < 2; refinement++)
        {
            int count = 10 << refinement;
            double h = 0.04 / count;
            MultibodyState state = State(-0.004);
            double loss = 0;
            double finalEnergy = 0;
            for (int i = 0; i < count; i++)
            {
                CompliantContactStepResult result = CompliantContactMultibodyIntegrator.Step(model, state, i * h, h, NoLoads, Controls);
                Assert.IsTrue(result.Energy.MidpointDissipation >= 0);
                Assert.IsTrue(result.Energy.AcceptedMechanicalEnergy <= result.Energy.InitialMechanicalEnergy + 1e-10);
                loss += result.Energy.MidpointDissipation;
                finalEnergy = result.Energy.AcceptedMechanicalEnergy;
                state = result.State;
            }
            Assert.IsTrue(loss > 0 && finalEnergy < initialEnergy);
            balanceErrors[refinement] = System.Math.Abs(finalEnergy - initialEnergy + loss);
        }
        TestContext.WriteLine($"Dissipative energy balance errors: {balanceErrors[0]:E6}, {balanceErrors[1]:E6} J");
        Assert.IsTrue(balanceErrors[0] / balanceErrors[1] > 3.5 && balanceErrors[0] / balanceErrors[1] < 4.5);
        Assert.IsTrue(balanceErrors[1] < 1e-6);
    }

    [TestMethod]
    public void JointProjectionEnergyIsMeasuredSeparatelyFromPhysicalDamping()
    {
        var model = Model(masses: new[] { Mass(2), Mass(3) }, joints: new[] { new SphericalJoint(0, 1, Vector3d.UnitX, -Vector3d.UnitX) });
        var state = new MultibodyState(new[] {
            new RigidBodyState(new(new(4, 0, 0.19), Quaterniond.Identity), default, default),
            new RigidBodyState(new(new(6, 0, 0.19), Quaterniond.Identity), default, default) });
        CompliantContactStepResult result = CompliantContactMultibodyIntegrator.Step(model, state, 0, 0.02,
            (_, _) => new[] { new RigidBodyLoads(new(0, 0, -20), default), new(new(0, 0, -30), default) }, Controls);
        Assert.IsTrue(result.JointStep.Diagnostics.PositionProjectionIterations > 0);
        Assert.IsTrue(result.JointStep.ProjectionCorrections.Any(c => c.PositionChange.Length > 0));
        Assert.AreEqual(0, result.Energy.MidpointDissipation);
        double rawKinetic = Kinetic(model, result.UnprojectedState);
        double acceptedKinetic = Kinetic(model, result.State);
        Assert.AreEqual(acceptedKinetic - rawKinetic, result.Energy.ProjectionKineticEnergyChange, 1e-15);
        double potentialChange = CompliantWheelRailContact.Evaluate(model, result.State).ElasticEnergy
            - CompliantWheelRailContact.Evaluate(model, result.UnprojectedState).ElasticEnergy;
        Assert.AreEqual(potentialChange, result.Energy.ProjectionElasticEnergyChange, 1e-15);
        Assert.IsTrue(System.Math.Abs(potentialChange) > 0);
        Assert.AreEqual(result.Energy.AcceptedMechanicalEnergy - result.Energy.UnprojectedMechanicalEnergy,
            result.Energy.ProjectionKineticEnergyChange + result.Energy.ProjectionElasticEnergyChange, 1e-14);
        TestContext.WriteLine($"Projection: kinetic {result.Energy.ProjectionKineticEnergyChange:E6}, elastic {potentialChange:E6} J; integration residual {result.Energy.IntegrationEnergyResidual:E6} J");
    }

    private static double Kinetic(CompliantWheelRailContactModel model, MultibodyState state) =>
        state.Bodies.Select((b, i) => RigidBodyDynamics.KineticEnergy(model.Geometry.Dynamics.Bodies[i], b)).Sum();

    [TestMethod]
    public void WorldTransformRotatesForcesTorquesAndMotionWithoutChangingScalarLaw()
    {
        var original = Model(dissipation: 1, wheels: new[] { Wheel(offset: new(0.3, 0, 0)) });
        Quaterniond rotation = Quaterniond.FromAxisAngle(new(1, -2, 3), 0.7);
        Vector3d translation = new(10, -7, 3);
        var curve = SplineCurve3d.Clamped(new[] { translation, translation + rotation.Rotate(new(10, 0, 0)) }, 1);
        double length = new ArcLengthMap(curve).Length;
        var rails = new RailGeometry(new(curve, rotation.Rotate(Vector3d.UnitZ), TrackOffsetProfile.Constant(default, length)),
            new(new[] { Rail("rail", default, length) }), Settings);
        var patch = new RailContactPatch("surface", "rail", 0, 0, length, 0, 1, RailMaterialSide.PositiveAreaVector);
        var geometry = new WheelRailContactSystem(original.Geometry.Dynamics, rails, original.Geometry.Wheels,
            new[] { patch }, original.Geometry.Pairs);
        var transformed = Assign(geometry, original.Laws.Single().Value);
        MultibodyState initial = State(-0.004, new(0.1, 0, -0.02));
        var moved = new MultibodyState(new[] { new RigidBodyState(new(translation + rotation.Rotate(initial.Bodies[0].Pose.Position), rotation),
            rotation.Rotate(initial.Bodies[0].LinearVelocity), default) });
        var a = CompliantContactMultibodyIntegrator.Step(original, initial, 0, 0.001, NoLoads, Controls);
        var b = CompliantContactMultibodyIntegrator.Step(transformed, moved, 0, 0.001, NoLoads, Controls);
        Assert.AreEqual(a.AcceptedContacts.Contacts[0].Response.NormalForce, b.AcceptedContacts.Contacts[0].Response.NormalForce, 1e-7);
        TestMath.Near(rotation.Rotate(a.AcceptedContacts.Contacts[0].WorldForce), b.AcceptedContacts.Contacts[0].WorldForce, 1e-7);
        TestMath.Near(rotation.Rotate(a.AcceptedContacts.Contacts[0].WorldTorque), b.AcceptedContacts.Contacts[0].WorldTorque, 1e-7);
        TestMath.Near(translation + rotation.Rotate(a.State.Bodies[0].Pose.Position), b.State.Bodies[0].Pose.Position, 1e-9);
        TestMath.Near(rotation.Rotate(a.State.Bodies[0].LinearVelocity), b.State.Bodies[0].LinearVelocity, 1e-9);
    }

    [TestMethod]
    public void DocumentedSingleSupportExampleBalancesExplicitLoad()
    {
        var line = SplineCurve3d.Clamped(new[] { Vector3d.Zero, new Vector3d(10, 0, 0) }, degree: 1);
        var track = new HeartlineTrack(line, Vector3d.UnitZ, TrackOffsetProfile.Constant(Vector3d.Zero, 10));
        var section = new RailCrossSection(new[] { SplineCurve3d.Clamped(new[] {
            new Vector3d(0, -0.1, 0), new Vector3d(0, 0.1, 0) }, degree: 1) });
        var roll = new QuantumDynamix.Math.Functions.CustomFunction(0, 10, (_, _) => new QuantumDynamix.Math.Functions.ScalarJet(0));
        var rails = new RailGeometry(track, new(new[] { new RailDefinition("rail",
            TrackOffsetProfile.Constant(Vector3d.Zero, 10), section, roll) }), new(1e-8, 1e-7, 1e-7, 1e-7, 1e-10, 1e-8));
        var bodies = new MultibodySystem(new[] { new RigidBodyMassProperties(2, Matrix3x3d.Diagonal(2, 3, 4)) }, Array.Empty<SphericalJoint>());
        var wheel = new WheelDefinition("wheel", 0, new(Vector3d.Zero, Quaterniond.FromAxisAngle(Vector3d.UnitZ, System.Math.PI / 2)), new(0.2, 0.08, 0.4));
        var patch = new RailContactPatch("top", "rail", 0, 0, 10, 0, 1, RailMaterialSide.PositiveAreaVector);
        var pair = new WheelRailPair("wheel", "top");
        var geometry = new WheelRailContactSystem(bodies, rails, new[] { wheel }, new[] { patch }, new[] { pair });
        var model = new CompliantWheelRailContactModel(geometry, new[] {
            KeyValuePair.Create(pair, new PowerLawNormalContact(2000, 1, 0, 0.02)) });
        var state = new MultibodyState(new[] { new RigidBodyState(new(new Vector3d(5, 0, 0.19), Quaterniond.Identity), Vector3d.Zero, Vector3d.Zero) });
        var controls = new CompliantContactStepSettings(0.05, 0.05, 0.001, 0.01);
        var result = CompliantContactMultibodyIntegrator.Step(model, state, 0, 0.001,
            (_, _) => new[] { new RigidBodyLoads(new(0, 0, -20), Vector3d.Zero) }, controls);
        Assert.AreEqual(20, result.AcceptedContacts.Contacts[0].Response.NormalForce, 1e-10);
        Assert.AreEqual(-0.01, result.AcceptedContacts.Contacts[0].Geometry.Gap, 1e-11);
        Assert.AreEqual(0.1, result.AcceptedContacts.ElasticEnergy, 1e-12);
    }

    [TestMethod]
    public void CompleteUniqueLawBindingsAndImmutableSnapshotsAreRequired()
    {
        var original = Model();
        var entries = original.Laws.ToArray();
        var copy = new CompliantWheelRailContactModel(original.Geometry, entries);
        entries[0] = KeyValuePair.Create(new WheelRailPair("unknown", "surface"), new PowerLawNormalContact(1, 1, 0, 1));
        Assert.AreEqual(2000, copy.Laws[original.Geometry.Pairs[0]].Coefficient);
        Assert.ThrowsExactly<NotSupportedException>(() => ((IDictionary<WheelRailPair, PowerLawNormalContact>)copy.Laws).Clear());
        Assert.ThrowsExactly<ArgumentException>(() => new CompliantWheelRailContactModel(original.Geometry, entries));
        Assert.ThrowsExactly<ArgumentException>(() => new CompliantWheelRailContactModel(original.Geometry, Array.Empty<KeyValuePair<WheelRailPair, PowerLawNormalContact>>()));
        Assert.ThrowsExactly<ArgumentException>(() => new CompliantWheelRailContactModel(original.Geometry, original.Laws.Concat(original.Laws)));
        var geometry = original.Geometry;
        var duplicateWheel = new WheelRailContactSystem(geometry.Dynamics, geometry.Rails, geometry.Wheels,
            new[] { Patch("one"), Patch("two") }, new[] { new WheelRailPair("wheel", "one"), new("wheel", "two") });
        Assert.ThrowsExactly<ArgumentException>(() => Assign(duplicateWheel, new(1, 1, 0, 1), new(1, 1, 0, 1)));
        CompliantContactEvaluation snapshot = CompliantWheelRailContact.Evaluate(original, State(-0.01));
        Assert.ThrowsExactly<NotSupportedException>(() => ((IList<WheelRailContactForce>)snapshot.Contacts).Clear());
        Assert.ThrowsExactly<NotSupportedException>(() => ((IList<RigidBodyLoads>)snapshot.BodyLoads).Clear());
    }

    [TestMethod]
    public void SamePairCannotReceiveRigidEnforcementAndCompliantForcesInCallbacks()
    {
        var model = Model();
        var controls = new ContactSolverSettings(0.5, 0.2, 0.02, 0.05);
        ContactSolveException error = Assert.ThrowsExactly<ContactSolveException>(() => ContactMultibodyIntegrator.Step(model.Geometry,
            State(), 0, 0.001, (_, s) => CompliantWheelRailContact.Evaluate(model, s).BodyLoads, controls));
        Assert.AreEqual(ContactSolveFailure.MixedFormulations, error.Failure);
        error = Assert.ThrowsExactly<ContactSolveException>(() => CompliantContactMultibodyIntegrator.Step(model,
            State(), 0, 0.001, (_, s) => ContactMultibodyIntegrator.Step(model.Geometry, s, 0, 0.001, NoLoads, controls).AverageConstraintLoads, Controls));
        Assert.AreEqual(ContactSolveFailure.MixedFormulations, error.Failure);
        // Scope cleanup after either failure permits independent comparisons on the same definitions.
        Assert.AreEqual(0, CompliantWheelRailContact.Evaluate(model, State()).Contacts[0].Response.NormalForce);
        ContactMultibodyIntegrator.Step(model.Geometry, State(), 0, 0.001, NoLoads, controls);
    }

    [TestMethod]
    public void GeometryCompressionAndConvergenceFailuresPreserveInputs()
    {
        var model = Model(maximumCompression: 0.01);
        MultibodyState state = State(-0.02);
        ContactSolveException compression = Assert.ThrowsExactly<ContactSolveException>(() =>
            CompliantContactMultibodyIntegrator.Step(model, state, 0, 0.001, NoLoads, Controls));
        Assert.AreEqual(ContactSolveFailure.CompressionLimit, compression.Failure);
        Assert.AreEqual("wheel", compression.WheelId);
        Assert.AreEqual(State(-0.02).Bodies[0], state.Bodies[0]);
        ContactSolveException geometry = Assert.ThrowsExactly<ContactSolveException>(() => CompliantWheelRailContact.Evaluate(model,
            State(), new(maximumEvaluations: 1)));
        Assert.AreEqual(ContactSolveFailure.GeometryUnresolved, geometry.Failure);
        ContactSolveException travel = Assert.ThrowsExactly<ContactSolveException>(() => CompliantContactMultibodyIntegrator.Step(model,
            State(0.02, new(100, 0, 0)), 0, 0.001, NoLoads, new(0.001, 0.2, 0.02, 0.05)));
        Assert.AreEqual(ContactSolveFailure.StepResolution, travel.Failure);
        var nonlinear = Model(coefficient: 1e5, exponent: 1.5, maximumCompression: 0.1);
        ConstrainedSolveException convergence = Assert.ThrowsExactly<ConstrainedSolveException>(() => CompliantContactMultibodyIntegrator.Step(nonlinear,
            State(-0.01, new(0, 0, -1)), 0, 0.005, NoLoads,
            new(0.5, 0.2, 0.02, 0.05, joints: new(maxNonlinearIterations: 1, nonlinearTolerance: 1e-14))));
        Assert.AreEqual(ConstrainedSolveFailure.NonlinearSolve, convergence.Failure);
    }

    [TestMethod]
    public void RepeatedStepsAreDeterministicAndPhysicalLimitsAreNotSolverTolerances()
    {
        var model = Model(dissipation: 1);
        MultibodyState state = State(-0.004, new(0, 0, -0.1));
        var a = CompliantContactMultibodyIntegrator.Step(model, state, 0, 0.001, NoLoads, Controls);
        var b = CompliantContactMultibodyIntegrator.Step(model, state, 0, 0.001, NoLoads, Controls);
        CollectionAssert.AreEqual(a.State.Bodies, b.State.Bodies);
        CollectionAssert.AreEqual(a.AcceptedContacts.Contacts, b.AcceptedContacts.Contacts);
        Assert.AreEqual(a.Energy, b.Energy);
        Assert.ThrowsExactly<ArgumentOutOfRangeException>(() => new CompliantContactStepSettings(1, System.Math.PI, 1, 1));
        Assert.ThrowsExactly<ArgumentOutOfRangeException>(() => new CompliantContactStepSettings(1, 1, 1, 1, travelSamples: 1));
        int calls = 0;
        Assert.ThrowsExactly<ArgumentOutOfRangeException>(() => CompliantContactMultibodyIntegrator.Step(model, state, 1e20, 0.01,
            (_, _) => { calls++; return new RigidBodyLoads[1]; }, Controls));
        Assert.AreEqual(0, calls);
        Assert.ThrowsExactly<ArgumentException>(() => CompliantContactMultibodyIntegrator.Step(model, state, 0, 0.001,
            (_, _) => Array.Empty<RigidBodyLoads>(), Controls));
    }
}
