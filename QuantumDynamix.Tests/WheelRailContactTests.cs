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

// All dimensions, masses, timesteps, and tolerances below are numerical fixtures,
// not manufacturer data or coaster engineering acceptance criteria.
[TestClass]
public sealed class WheelRailContactTests
{
    private static readonly CrownedWheelTread Tread = new(0.2, 0.08, 0.4);
    private static readonly Quaterniond AxleAlongY = Quaterniond.FromAxisAngle(Vector3d.UnitZ, System.Math.PI / 2);
    private static ContactSolverSettings Controls => new(0.5, 0.2, 0.02, 0.05);
    private static RigidBodyMassProperties Mass(double mass = 2) => new(mass, Matrix3x3d.Diagonal(2, 3, 4));
    private static RailContactPatch Patch(string id = "surface", string rail = "rail", RailMaterialSide side = RailMaterialSide.PositiveAreaVector) =>
        new(id, rail, 0, 0, 10, 0, 1, side);
    private static WheelDefinition Wheel(string id = "wheel", int body = 0, Vector3d offset = default) =>
        new(id, body, new(offset, AxleAlongY), Tread);
    private static MultibodyState State(double gap = 0, Vector3d velocity = default, Vector3d? position = null,
        Quaterniond? orientation = null, Vector3d spin = default) => new(new[] {
            new RigidBodyState(new(position ?? new Vector3d(5, 0, Tread.Radius + gap), orientation ?? Quaterniond.Identity), velocity, spin) });
    private static WheelRailContactSystem SystemFor(WheelDefinition[]? wheels = null,
        RigidBodyMassProperties[]? masses = null, SphericalJoint[]? joints = null, RailGeometry? rails = null,
        RailContactPatch[]? patches = null, WheelRailPair[]? pairs = null)
    {
        wheels ??= new[] { Wheel() };
        patches ??= new[] { Patch() };
        return new(new(masses ?? new[] { Mass() }, joints ?? Array.Empty<SphericalJoint>()),
            rails ?? Straight(Rail("rail", default, 10)), wheels, patches,
            pairs ?? wheels.Select(w => new WheelRailPair(w.Id, patches[0].Id)).ToArray());
    }
    private static IReadOnlyList<RigidBodyLoads> NoLoads(double time, MultibodyState state) => new RigidBodyLoads[state.Bodies.Count];

    [TestMethod]
    [DataRow(-0.005)]
    [DataRow(0.0)]
    [DataRow(0.03)]
    public void SmoothPlaneHasAnalyticSignedGapWitnessesAndMaterialPointVelocity(double gap)
    {
        MultibodyState state = State(gap, new(2, 0, -0.3), spin: new(0, 1, 0));
        WheelRailContactSample c = WheelRailContactGeometry.Query(SystemFor(), state).Contacts.Single();
        Assert.AreEqual(gap, c.Gap, 1e-10);
        TestMath.Near(new(5, 0, 0), c.RailPoint, 1e-8);
        TestMath.Near(new(5, 0, gap), c.WheelPoint, 1e-8);
        TestMath.Near(Vector3d.UnitZ, c.Normal, 1e-10);
        TestMath.Near(new(1.8, 0, -0.3), c.RelativeVelocity, 1e-8);
        Assert.AreEqual(-0.3, c.NormalVelocity, 1e-8);
        Assert.IsTrue(c.TangencyResidual <= 1e-8);
        Assert.IsTrue(c.Evaluations > 9);
    }

    [TestMethod]
    public void AxleTiltChangesCrownedSupportGeometryInsteadOfUsingASphere()
    {
        const double angle = 0.05;
        double support = Tread.CrownRadius + (Tread.Radius - Tread.CrownRadius) * System.Math.Cos(angle);
        MultibodyState state = State(position: new(5, 0, support), orientation: Quaterniond.FromAxisAngle(Vector3d.UnitX, angle));
        WheelRailContactSample c = WheelRailContactGeometry.Query(SystemFor(), state).Contacts[0];
        Assert.AreEqual(0, c.Gap, 1e-10);
        Assert.AreEqual((Tread.Radius - Tread.CrownRadius) * System.Math.Sin(angle), c.RailPoint.Y, 1e-8);
        Assert.IsTrue(System.Math.Abs(support - Tread.Radius) > 1e-5);
    }

    [TestMethod]
    public void RationalCircularRailUsesItsSurfaceNotItsCenterline()
    {
        const double radius = 0.05;
        var boundary = SplineCurve3d.Clamped(new[] { new Vector3d(0, radius, 0), new(0, radius, radius), new(0, 0, radius) },
            2, new[] { 1.0, System.Math.Sqrt(0.5), 1 });
        RailGeometry rails = Straight(Rail("rail", default, 10, new(new[] { boundary })));
        var patch = Patch(side: RailMaterialSide.NegativeAreaVector);
        var wheel = new WheelDefinition("wheel", 0, RigidPose3d.Identity, Tread);
        Vector3d normal = new Vector3d(0, 1, 1).Normalized();
        MultibodyState state = State(position: new Vector3d(5, 0, 0) + normal * (radius + Tread.Radius + 0.01));
        WheelRailContactSample c = WheelRailContactGeometry.Query(SystemFor(new[] { wheel }, rails: rails, patches: new[] { patch }), state).Contacts[0];
        Assert.AreEqual(0.01, c.Gap, 1e-9);
        TestMath.Near(normal, c.Normal, 1e-8);
        TestMath.Near(new Vector3d(5, 0, 0) + normal * radius, c.RailPoint, 1e-8);
    }

    [TestMethod]
    [DataRow(0.0)]
    [DataRow(1.5707963267948966)]
    [DataRow(3.141592653589793)]
    public void RunningSideAndUpstopOrientationsUseTheSameEquations(double angle)
    {
        Quaterniond rotation = Quaterniond.FromAxisAngle(Vector3d.UnitX, angle);
        var track = new HeartlineTrack(Line(), rotation.Rotate(Vector3d.UnitZ), TrackOffsetProfile.Constant(default, 10));
        var rails = new RailGeometry(track, new(new[] { Rail("rail", default, 10) }), Settings);
        Vector3d normal = rotation.Rotate(Vector3d.UnitZ);
        MultibodyState state = State(position: rotation.Rotate(new(5, 0, Tread.Radius)), orientation: rotation);
        ContactStepResult result = ContactMultibodyIntegrator.Step(SystemFor(rails: rails), state, 0, 0.01,
            (_, _) => new[] { new RigidBodyLoads(normal * -20, default) }, Controls);
        Assert.AreEqual(0.2, result.ContactImpulses[0].NormalImpulse, 1e-9);
        TestMath.Near(normal * 20, result.ContactImpulses[0].AverageForce, 1e-7);
        TestMath.Near(state.Bodies[0].Pose.Position, result.State.Bodies[0].Pose.Position, 1e-9);
    }

    [TestMethod]
    public void RestingSupportImpulseBalancesOwnerSuppliedGravity()
    {
        ContactStepResult result = ContactMultibodyIntegrator.Step(SystemFor(), State(), 0, 0.01,
            (_, _) => new[] { new RigidBodyLoads(new(0, 0, -19.62), default) }, Controls);
        Assert.AreEqual(0.1962, result.ContactImpulses[0].NormalImpulse, 1e-10);
        Assert.AreEqual(19.62, result.ContactImpulses[0].AverageForce.Z, 1e-8);
        TestMath.Near(Vector3d.Zero, result.State.Bodies[0].LinearVelocity, 1e-10);
        Assert.IsTrue(result.Diagnostics.MinimumGap >= -Controls.PositionTolerance);
        Assert.IsTrue(result.Diagnostics.MaximumVelocityViolation <= Controls.VelocityTolerance);
        Assert.IsTrue(result.Diagnostics.LinearSolveRelativeResidual <= Controls.Joints.LinearSolveTolerance);
    }

    [TestMethod]
    public void SeparationHasNoAttractionOrTangentialGuidance()
    {
        MultibodyState initial = State(velocity: new(1, 0.2, 0.5));
        ContactStepResult result = ContactMultibodyIntegrator.Step(SystemFor(), initial, 0, 0.01, NoLoads, Controls);
        Assert.AreEqual(0, result.ContactImpulses[0].NormalImpulse);
        TestMath.Near(initial.Bodies[0].LinearVelocity, result.State.Bodies[0].LinearVelocity, 0);
        TestMath.Near(new(5.01, 0.002, 0.205), result.State.Bodies[0].Pose.Position, 1e-12);
        Assert.AreEqual(0.005, result.AcceptedContacts[0].Gap, 1e-9);
    }

    [TestMethod]
    public void CentralImpactReportsImpulseAndStepAverageSeparately()
    {
        MultibodyState incoming = State(velocity: new(0, 0, -1));
        ContactStepResult coarse = ContactMultibodyIntegrator.Step(SystemFor(), incoming, 0, 0.01, NoLoads, Controls);
        ContactStepResult fine = ContactMultibodyIntegrator.Step(SystemFor(), incoming, 0, 0.005, NoLoads, Controls);
        Assert.AreEqual(2, coarse.ContactImpulses[0].NormalImpulse, 1e-9);
        Assert.AreEqual(coarse.ContactImpulses[0].NormalImpulse, fine.ContactImpulses[0].NormalImpulse, 1e-12);
        Assert.AreEqual(200, coarse.ContactImpulses[0].AverageForce.Z, 1e-7);
        Assert.AreEqual(400, fine.ContactImpulses[0].AverageForce.Z, 1e-7);
        TestMath.Near(Vector3d.Zero, coarse.State.Bodies[0].LinearVelocity, 1e-9);
        TestMath.Near(Vector3d.Zero, coarse.NumericalCorrections[0].PositionChange, 1e-12);
    }

    [TestMethod]
    public void GapOverStepRecontactHasDocumentedDiscreteClosingVelocity()
    {
        var system = SystemFor();
        ContactStepResult first = ContactMultibodyIntegrator.Step(system, State(0.01, new(0, 0, -1)), 0, 0.02, NoLoads, Controls);
        Assert.AreEqual(1, first.ContactImpulses[0].NormalImpulse, 1e-8);
        Assert.AreEqual(-0.5, first.State.Bodies[0].LinearVelocity.Z, 1e-8);
        Assert.AreEqual(0, first.AcceptedContacts[0].Gap, 1e-9);
        ContactStepResult second = ContactMultibodyIntegrator.Step(system, first.State, 0.02, 0.02, NoLoads, Controls);
        Assert.AreEqual(1, second.ContactImpulses[0].NormalImpulse, 1e-8);
        Assert.AreEqual(0, second.State.Bodies[0].LinearVelocity.Z, 1e-8);
        Assert.IsTrue(second.AcceptedContacts[0].Gap >= -Controls.PositionTolerance);
    }

    [TestMethod]
    public void OffCenterContactUsesRotationalEffectiveMassAndWorldAngularImpulse()
    {
        var system = SystemFor(new[] { Wheel(offset: new(0.3, 0, 0)) });
        MultibodyState initial = State(velocity: new(0, 0, -1));
        ContactStepResult result = ContactMultibodyIntegrator.Step(system, initial, 0, 0.001, NoLoads, Controls);
        double impulse = 1 / (0.5 + 0.3 * 0.3 / 3);
        Assert.AreEqual(impulse, result.ContactImpulses[0].NormalImpulse, 1e-8);
        TestMath.Near(new(0, -0.3 * impulse, 0), result.ContactImpulses[0].WorldAngularImpulse, 1e-8);
        Assert.AreEqual(-0.3 * impulse / 3, result.State.Bodies[0].BodyAngularVelocity.Y, 1e-8);
    }

    [TestMethod]
    public void SphericalJointAndSupportShareOneCoupledImpulseSolve()
    {
        var system = SystemFor(masses: new[] { Mass(2), Mass(3) }, joints: new[] { new SphericalJoint(0, 1, default, default) });
        MultibodyState one = State();
        MultibodyState state = new(new[] { one.Bodies[0], one.Bodies[0] });
        ContactStepResult result = ContactMultibodyIntegrator.Step(system, state, 0, 0.01,
            (_, _) => new[] { new RigidBodyLoads(new(0, 0, -20), default), new(new(0, 0, -30), default) }, Controls);
        Assert.AreEqual(0.5, result.ContactImpulses[0].NormalImpulse, 1e-9);
        TestMath.Near(new(0, 0, -0.3), result.JointImpulses[0].ImpulseOnA, 1e-9);
        foreach (RigidBodyState body in result.State.Bodies) { TestMath.Near(Vector3d.Zero, body.LinearVelocity, 1e-9); }
        Assert.IsTrue(result.Diagnostics.MaximumJointGap <= Controls.Joints.PositionTolerance);
        Assert.IsTrue(result.Diagnostics.MaximumJointSlip <= Controls.Joints.VelocityTolerance);
    }

    [TestMethod]
    public void SimultaneousSupportsResolveIndependentLoadsAndIndependentStations()
    {
        var system = SystemFor(new[] { Wheel("front", offset: new(0.5, 0, 0)), Wheel("rear", offset: new(-0.5, 0, 0)) });
        ContactStepResult result = ContactMultibodyIntegrator.Step(system, State(), 0, 0.01,
            (_, _) => new[] { new RigidBodyLoads(new(0, 0, -20), new(0, 2, 0)) }, Controls);
        Assert.AreEqual(0.12, result.ContactImpulses[0].NormalImpulse, 1e-8);
        Assert.AreEqual(0.08, result.ContactImpulses[1].NormalImpulse, 1e-8);
        Assert.AreEqual(5.5, result.ContactImpulses[0].Geometry.HeartlineStation, 1e-8);
        Assert.AreEqual(4.5, result.ContactImpulses[1].Geometry.HeartlineStation, 1e-8);
        TestMath.Near(new(0, 0, 20), result.AverageConstraintLoads[0].Force, 1e-7);
        TestMath.Near(new(0, -2, 0), result.AverageConstraintLoads[0].Torque, 1e-7);
    }

    [TestMethod]
    public void RepeatedSteppingAndReorderedDefinitionsAreDeterministic()
    {
        WheelDefinition front = Wheel("front", offset: new(0.5, 0, 0));
        WheelDefinition rear = Wheel("rear", offset: new(-0.5, 0, 0));
        var a = SystemFor(new[] { front, rear });
        var b = SystemFor(new[] { rear, front });
        MultibodyState first = State(velocity: new(0.1, 0, 0));
        MultibodyState second = first;
        for (int i = 0; i < 10; i++)
        {
            static IReadOnlyList<RigidBodyLoads> Loads(double t, MultibodyState s) => new[] { new RigidBodyLoads(new(0, 0, -20), default) };
            ContactStepResult left = ContactMultibodyIntegrator.Step(a, first, i * 0.01, 0.01, Loads, Controls);
            ContactStepResult right = ContactMultibodyIntegrator.Step(b, second, i * 0.01, 0.01, Loads, Controls);
            CollectionAssert.AreEqual(left.State.Bodies, right.State.Bodies);
            CollectionAssert.AreEqual(left.ContactImpulses, right.ContactImpulses);
            Assert.AreEqual(left.Diagnostics, right.Diagnostics);
            first = left.State;
            second = right.State;
        }
    }

    [TestMethod]
    public void RedundantRigidSupportsFailInsteadOfInventingLoadSharing()
    {
        var system = SystemFor(new[] { Wheel("a", offset: new(-0.5, 0, 0)), Wheel("b"), Wheel("c", offset: new(0.5, 0, 0)) });
        MultibodyState state = State();
        ContactSolveException error = Assert.ThrowsExactly<ContactSolveException>(() => ContactMultibodyIntegrator.Step(system, state, 0, 0.01,
            (_, _) => new[] { new RigidBodyLoads(new(0, 0, -20), default) }, Controls));
        Assert.AreEqual(ContactSolveFailure.IndeterminateReactions, error.Failure);
        Assert.AreEqual(State().Bodies[0], state.Bodies[0]);
    }

    [TestMethod]
    public void GeometryFailuresAreExplicitAndNeverSeparation()
    {
        ContactSolveException budget = Assert.ThrowsExactly<ContactSolveException>(() =>
            WheelRailContactGeometry.Query(SystemFor(), State(), new(maximumEvaluations: 1)));
        Assert.AreEqual(ContactSolveFailure.GeometryUnresolved, budget.Failure);
        Assert.AreEqual("wheel", budget.WheelId);
        Assert.AreEqual("surface", budget.PatchId);
        ContactSolveException edge = Assert.ThrowsExactly<ContactSolveException>(() =>
            WheelRailContactGeometry.Query(SystemFor(), State(position: new(0, 0, 0.2))));
        Assert.AreEqual(ContactSolveFailure.UnsupportedFeature, edge.Failure);
        ContactSolveException side = Assert.ThrowsExactly<ContactSolveException>(() => WheelRailContactGeometry.Query(
            SystemFor(patches: new[] { Patch(side: RailMaterialSide.NegativeAreaVector) }), State()));
        Assert.AreEqual(ContactSolveFailure.MaterialSideMismatch, side.Failure);
        const double angle = 0.3;
        ContactSolveException treadEdge = Assert.ThrowsExactly<ContactSolveException>(() => WheelRailContactGeometry.Query(SystemFor(),
            State(position: new(5, 0, 0.4 - 0.2 * System.Math.Cos(angle)), orientation: Quaterniond.FromAxisAngle(Vector3d.UnitX, angle))));
        Assert.AreEqual(ContactSolveFailure.UnsupportedFeature, treadEdge.Failure);
    }

    [TestMethod]
    public void InvalidOrIntersectingDeclaredPatchesFailLocatedQueries()
    {
        var collapsed = new RailCrossSection(new[] { SplineCurve3d.Clamped(new[] { Vector3d.Zero, Vector3d.Zero }, 1) });
        ContactSolveException degenerate = Assert.ThrowsExactly<ContactSolveException>(() => WheelRailContactGeometry.Query(
            SystemFor(rails: Straight(Rail("rail", default, 10, collapsed))), State()));
        Assert.AreEqual(ContactSolveFailure.InvalidPatch, degenerate.Failure);
        var loop = new RailCrossSection(new[] { SplineCurve3d.Clamped(new[] {
            new Vector3d(0, -0.05, 0), new(0, 0.1, 0.02), new(0, 0.1, -0.02), new(0, -0.05, 0) }, 3) });
        ContactSolveException intersection = Assert.ThrowsExactly<ContactSolveException>(() => WheelRailContactGeometry.Query(
            SystemFor(rails: Straight(Rail("rail", default, 10, loop))), State()));
        Assert.AreEqual(ContactSolveFailure.InvalidPatch, intersection.Failure);
    }

    [TestMethod]
    public void InputValidationAndNumericalFailuresPreserveSnapshots()
    {
        Assert.ThrowsExactly<ArgumentException>(() => new CrownedWheelTread(0.1, 0.5, 0.4));
        Assert.ThrowsExactly<ArgumentOutOfRangeException>(() => new CrownedWheelTread(double.NaN, 0.1, 0.4));
        Assert.ThrowsExactly<ArgumentException>(() => SystemFor(pairs: new[] { new WheelRailPair("missing", "surface") }));
        Assert.ThrowsExactly<ArgumentException>(() => SystemFor(pairs: new[] { new WheelRailPair("wheel", "surface"), new("wheel", "surface") }));
        Assert.ThrowsExactly<ArgumentOutOfRangeException>(() => new ContactGeometrySettings(maximumIterations: 0));
        Assert.ThrowsExactly<ArgumentException>(() => new ContactSolverSettings(1, 1, 1, 1, positionTolerance: 1e-12));
        MultibodyState state = State();
        int calls = 0;
        Assert.ThrowsExactly<ArgumentOutOfRangeException>(() => ContactMultibodyIntegrator.Step(SystemFor(), state, 1e20, 0.01,
            (_, _) => { calls++; return new RigidBodyLoads[1]; }, Controls));
        Assert.AreEqual(0, calls);
        ContactSolveException initial = Assert.ThrowsExactly<ContactSolveException>(() => ContactMultibodyIntegrator.Step(SystemFor(), State(-0.01), 0, 0.01,
            (_, _) => { calls++; return new RigidBodyLoads[1]; }, Controls));
        Assert.AreEqual(ContactSolveFailure.InconsistentInitialState, initial.Failure);
        Assert.AreEqual(0, calls);
        ContactSolveException travel = Assert.ThrowsExactly<ContactSolveException>(() => ContactMultibodyIntegrator.Step(SystemFor(), state, 0, 1,
            (_, _) => new[] { new RigidBodyLoads(new(100, 0, 0), default) }, Controls));
        Assert.AreEqual(ContactSolveFailure.StepResolution, travel.Failure);
        Assert.AreEqual(State().Bodies[0], state.Bodies[0]);
        Assert.ThrowsExactly<ArgumentException>(() => ContactMultibodyIntegrator.Step(SystemFor(), state, 0, 0.01,
            (_, _) => Array.Empty<RigidBodyLoads>(), Controls));
    }

    [TestMethod]
    public void ContactDefinitionsDefensivelyCopyCollections()
    {
        WheelDefinition[] wheels = { Wheel() };
        RailContactPatch[] patches = { Patch() };
        WheelRailPair[] pairs = { new("wheel", "surface") };
        var system = SystemFor(wheels, patches: patches, pairs: pairs);
        wheels[0] = Wheel("replacement");
        patches[0] = Patch("replacement");
        pairs[0] = new("replacement", "replacement");
        Assert.AreEqual("wheel", system.Wheels[0].Id);
        Assert.AreEqual("surface", system.Patches[0].Id);
        Assert.AreEqual("wheel", system.Pairs[0].WheelId);
        Assert.ThrowsExactly<NotSupportedException>(() => ((IList<WheelDefinition>)system.Wheels)[0] = wheels[0]);
    }

    [TestMethod]
    public void OffCenterArticulatedPairPreservesJointsWhileTransmittingSupport()
    {
        var system = SystemFor(masses: new[] { Mass(2), Mass(3) },
            joints: new[] { new SphericalJoint(0, 1, Vector3d.UnitX, -Vector3d.UnitX) });
        var state = new MultibodyState(new[] {
            new RigidBodyState(new(new(4, 0, 0.2), Quaterniond.Identity), default, default),
            new RigidBodyState(new(new(6, 0, 0.2), Quaterniond.Identity), default, default) });
        ContactStepResult result = ContactMultibodyIntegrator.Step(system, state, 0, 0.02,
            (_, _) => new[] { new RigidBodyLoads(new(0, 0, -20), default), new(new(0, 0, -30), default) }, Controls);
        Assert.AreEqual(0.6, result.ContactImpulses[0].NormalImpulse, 1e-8);
        TestMath.Near(new(0, 0, -0.2), result.JointImpulses[0].ImpulseOnA, 1e-8);
        Vector3d a = result.State.Bodies[0].Pose.TransformPoint(Vector3d.UnitX);
        Vector3d b = result.State.Bodies[1].Pose.TransformPoint(-Vector3d.UnitX);
        TestMath.Near(a, b, Controls.Joints.PositionTolerance);
        Assert.IsTrue(result.Diagnostics.MaximumJointSlip <= Controls.Joints.VelocityTolerance);
        Assert.IsTrue(result.Diagnostics.PositionProjectionIterations > 0);
        Assert.IsTrue(result.NumericalCorrections.Any(c => c.PositionChange.Length > 0));
        Assert.IsTrue(result.Diagnostics.MomentumRelativeResidual <= Controls.Joints.LinearSolveTolerance);
        // Numerical corrections are not retroactively included in the start-configuration impulse.
        Assert.AreEqual(30, result.ContactImpulses[0].AverageForce.Z, 1e-6);
    }

    [TestMethod]
    public void ThreeNoncollinearContactsHaveDeterminateIndependentSupportLoads()
    {
        var system = SystemFor(new[] { Wheel("a", offset: new(-0.5, -0.03, 0)),
            Wheel("b", offset: new(0.5, -0.03, 0)), Wheel("c", offset: new(0, 0.06, 0)) });
        ContactStepResult result = ContactMultibodyIntegrator.Step(system, State(), 0, 0.01,
            (_, _) => new[] { new RigidBodyLoads(new(0, 0, -20), default) }, Controls);
        foreach (WheelRailContactImpulse impulse in result.ContactImpulses)
        { Assert.AreEqual(0.2 / 3, impulse.NormalImpulse, 1e-8); }
        TestMath.Near(Vector3d.Zero, result.State.Bodies[0].BodyAngularVelocity, 1e-9);
    }

    [TestMethod]
    public void ExcessMomentUnloadsOneWheelRatherThanProducingTension()
    {
        var system = SystemFor(new[] { Wheel("front", offset: new(0.5, 0, 0)), Wheel("rear", offset: new(-0.5, 0, 0)) });
        ContactStepResult result = ContactMultibodyIntegrator.Step(system, State(), 0, 0.01,
            (_, _) => new[] { new RigidBodyLoads(new(0, 0, -20), new(0, 20, 0)) }, Controls);
        Assert.AreEqual((0.1 + 0.5 * 0.2 / 3) / (0.5 + 0.25 / 3), result.ContactImpulses[0].NormalImpulse, 1e-8);
        Assert.AreEqual(0, result.ContactImpulses[1].NormalImpulse);
        Assert.IsTrue(result.AcceptedContacts[1].Gap > 0);
        Assert.IsTrue(result.AcceptedContacts[1].NormalVelocity > 0);
    }

    [TestMethod]
    public void AsymmetricDualRailsRetainExplicitOffsetsAndPairBindings()
    {
        RailGeometry rails = Straight(Rail("left", new(0, -0.3, 0), 10), Rail("right", new(0, 0.4, 0.05), 10));
        var system = SystemFor(new[] { Wheel("a", offset: new(-0.5, -0.3, 0)), Wheel("b", offset: new(0.5, 0.4, 0.05)) },
            rails: rails, patches: new[] { Patch("left", "left"), Patch("right", "right") },
            pairs: new[] { new WheelRailPair("a", "left"), new("b", "right") });
        ContactStepResult result = ContactMultibodyIntegrator.Step(system, State(), 0, 0.001,
            (_, _) => new[] { new RigidBodyLoads(new(0, 0, -20), default) }, Controls);
        Assert.AreEqual("left", result.ContactImpulses[0].Geometry.RailId);
        Assert.AreEqual("right", result.ContactImpulses[1].Geometry.RailId);
        Assert.AreEqual(0.05, result.ContactImpulses[1].Geometry.RailPoint.Z, 1e-10);
        Assert.IsTrue(result.ContactImpulses.All(c => c.NormalImpulse > 0));
        Assert.IsTrue(result.AcceptedContacts.All(c => c.Gap >= -Controls.PositionTolerance));
        Vector3d total = result.ContactImpulses.Aggregate(Vector3d.Zero, (sum, c) => sum + c.WorldImpulse);
        TestMath.Near(total / 0.001, result.AverageConstraintLoads[0].Force, 1e-10);
    }

    [TestMethod]
    public void FirstOrderFreeMotionConvergesWithoutChangingTheExistingIntegrators()
    {
        double coarse = Error(0.1, 10);
        double fine = Error(0.05, 20);
        Assert.AreEqual(2, coarse / fine, 1e-10);
        double Error(double h, int count)
        {
            var system = SystemFor();
            MultibodyState state = State();
            for (int i = 0; i < count; i++)
            {
                state = ContactMultibodyIntegrator.Step(system, state, i * h, h,
                    (_, _) => new[] { new RigidBodyLoads(new(0, 0, 2), default) }, Controls).State;
            }
            // z''=1 m/s^2, z(0)=0.2 m. Semi-implicit Euler error is h/2 at t=1.
            return System.Math.Abs(state.Bodies[0].Pose.Position.Z - 0.7);
        }
    }

    [TestMethod]
    public void GapOverStepImpactTimingConvergesUnderRefinement()
    {
        double coarse = Error(0.02, 5);
        double medium = Error(0.01, 10);
        double fine = Error(0.005, 20);
        Assert.IsTrue(fine < medium && medium < coarse);
        Assert.IsTrue(coarse / medium > 1.5 && medium / fine > 1.5);
        double Error(double h, int count)
        {
            var system = SystemFor();
            MultibodyState state = State(0.037, new(1, 0, -1));
            double firstImpulseStart = double.NaN;
            for (int i = 0; i < count; i++)
            {
                ContactStepResult result = ContactMultibodyIntegrator.Step(system, state, i * h, h, NoLoads, Controls);
                // The first predictive impulse can occur before the exact impact at t=0.037 s.
                if (double.IsNaN(firstImpulseStart) && result.ContactImpulses[0].NormalImpulse > 1e-8)
                { firstImpulseStart = i * h; }
                state = result.State;
            }
            Assert.AreEqual(0, state.Bodies[0].LinearVelocity.Z, 1e-8);
            return System.Math.Abs(firstImpulseStart - 0.037);
        }
    }

    [TestMethod]
    public void SolverAndArithmeticBudgetsFailWithoutReturningPartialAdvances()
    {
        MultibodyState state = State(velocity: new(0, 0, -1));
        ContactSolveException active = Assert.ThrowsExactly<ContactSolveException>(() => ContactMultibodyIntegrator.Step(SystemFor(), state, 0, 0.01,
            NoLoads, new(0.5, 0.2, 0.02, 0.05, maximumActiveSetIterations: 1)));
        Assert.AreEqual(ContactSolveFailure.ActiveSetLimit, active.Failure);
        ContactSolveException arithmetic = Assert.ThrowsExactly<ContactSolveException>(() => ContactMultibodyIntegrator.Step(
            SystemFor(masses: new[] { Mass(1e-308) }), State(), 0, 0.01,
            (_, _) => new[] { new RigidBodyLoads(new(1e308, 0, 0), default) }, Controls));
        Assert.AreEqual(ContactSolveFailure.NumericalFailure, arithmetic.Failure);
        Assert.AreEqual(State(velocity: new(0, 0, -1)).Bodies[0], state.Bodies[0]);
    }

    [TestMethod]
    public void CoincidentPatchBindingsAreAmbiguousRatherThanDoubleCounted()
    {
        var system = SystemFor(patches: new[] { Patch("one"), Patch("two") },
            pairs: new[] { new WheelRailPair("wheel", "one"), new("wheel", "two") });
        ContactSolveException error = Assert.ThrowsExactly<ContactSolveException>(() => WheelRailContactGeometry.Query(system, State()));
        Assert.AreEqual(ContactSolveFailure.AmbiguousContact, error.Failure);
    }

    [TestMethod]
    public void CurvedRailCanSeparateFromFreelyMovingWheelWithoutSplineFollowing()
    {
        const double radius = 10;
        var curve = SplineCurve3d.Clamped(new[] { new Vector3d(radius, 0, 0), new(radius, 0, radius), new(0, 0, radius) },
            2, new[] { 1.0, System.Math.Sqrt(0.5), 1 });
        double length = new ArcLengthMap(curve).Length;
        var rails = new RailGeometry(new(curve, Vector3d.UnitX, TrackOffsetProfile.Constant(default, length)),
            new(new[] { Rail("rail", default, length) }), Settings);
        var patch = new RailContactPatch("surface", "rail", 0, 0, length, 0, 1, RailMaterialSide.PositiveAreaVector);
        Vector3d normal = new Vector3d(1, 0, 1).Normalized();
        Vector3d velocity = new Vector3d(-1, 0, 1).Normalized();
        MultibodyState state = State(position: normal * (radius + Tread.Radius), velocity: velocity);
        ContactStepResult result = ContactMultibodyIntegrator.Step(SystemFor(rails: rails, patches: new[] { patch }),
            state, 0, 0.01, NoLoads, Controls);
        double expectedGap = 0.01 * 0.01 / (System.Math.Sqrt(10.2 * 10.2 + 0.01 * 0.01) + 10.2);
        Assert.AreEqual(0, result.ContactImpulses[0].NormalImpulse, 1e-8);
        Assert.AreEqual(expectedGap, result.AcceptedContacts[0].Gap, 1e-8);
        TestMath.Near(velocity, result.State.Bodies[0].LinearVelocity, 1e-10);
        Assert.AreEqual(Quaterniond.Identity, result.State.Bodies[0].Pose.Orientation);
    }

    [TestMethod]
    public void GeneralWorldTransformRotatesGeometryImpulsesAndMotionConsistently()
    {
        Quaterniond rotation = Quaterniond.FromAxisAngle(new(1, -2, 3), 0.7);
        Vector3d translation = new(100, -70, 30);
        var curve = SplineCurve3d.Clamped(new[] { translation, translation + rotation.Rotate(new(10, 0, 0)) }, 1);
        double length = new ArcLengthMap(curve).Length;
        var track = new HeartlineTrack(curve,
            rotation.Rotate(Vector3d.UnitZ), TrackOffsetProfile.Constant(default, length));
        var rails = new RailGeometry(track, new(new[] { Rail("rail", default, length) }), Settings);
        MultibodyState original = State(velocity: new(0.1, 0, -0.2));
        MultibodyState transformed = State(position: translation + rotation.Rotate(original.Bodies[0].Pose.Position),
            velocity: rotation.Rotate(original.Bodies[0].LinearVelocity), orientation: rotation);
        ContactStepResult a = ContactMultibodyIntegrator.Step(SystemFor(), original, 0, 0.01, NoLoads, Controls);
        var patch = new RailContactPatch("surface", "rail", 0, 0, length, 0, 1, RailMaterialSide.PositiveAreaVector);
        ContactStepResult b = ContactMultibodyIntegrator.Step(SystemFor(rails: rails, patches: new[] { patch }), transformed, 0, 0.01, NoLoads, Controls);
        Assert.AreEqual(a.ContactImpulses[0].NormalImpulse, b.ContactImpulses[0].NormalImpulse, 1e-8);
        TestMath.Near(rotation.Rotate(a.ContactImpulses[0].WorldImpulse), b.ContactImpulses[0].WorldImpulse, 1e-8);
        TestMath.Near(translation + rotation.Rotate(a.State.Bodies[0].Pose.Position), b.State.Bodies[0].Pose.Position, 1e-8);
        TestMath.Near(rotation.Rotate(a.State.Bodies[0].LinearVelocity), b.State.Bodies[0].LinearVelocity, 1e-8);
    }

    [TestMethod]
    public void LargeCrownRadiusRetainsSmallSignedClearance()
    {
        var wheel = new WheelDefinition("wheel", 0, RigidPose3d.Identity, new(0.2, 0.08, 1e10));
        WheelRailContactSample contact = WheelRailContactGeometry.Query(SystemFor(new[] { wheel }), State(1e-6)).Contacts[0];
        Assert.AreEqual(1e-6, contact.Gap, 1e-12);
        TestMath.Near(new(5, 0, 0), contact.RailPoint, 1e-8);
    }

    [TestMethod]
    [DataRow(1e-6)]
    [DataRow(1e6)]
    public void MassScalingPreservesSupportAccelerationAndScalesImpulse(double mass)
    {
        ContactStepResult result = ContactMultibodyIntegrator.Step(SystemFor(masses: new[] { Mass(mass) }), State(), 0, 0.01,
            (_, _) => new[] { new RigidBodyLoads(new(0, 0, -10 * mass), default) }, Controls);
        Assert.AreEqual(0.1, result.ContactImpulses[0].NormalImpulse / mass, 1e-9);
        TestMath.Near(Vector3d.Zero, result.State.Bodies[0].LinearVelocity, 1e-9);
    }
}
