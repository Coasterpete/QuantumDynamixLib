using System;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using QuantumDynamix.Geometry;
using QuantumDynamix.Math;
using QuantumDynamix.Physics;
using static QuantumDynamix.Tests.RevoluteTestFixtures;

namespace QuantumDynamix.Tests;

[TestClass]
public sealed class RevoluteIntegrationTests
{
    public TestContext TestContext { get; set; } = null!;

    [TestMethod]
    public void TwoIndependentWheelsKeepDistinctFreeSpinsAndAcceptedAngleHistory()
    {
        var (pair, _) = Centered();
        var system = new MultibodySystem(new[] { pair.Bodies[0], pair.Bodies[1], pair.Bodies[1] }, Array.Empty<SphericalJoint>(),
            new[] { pair.RevoluteJoints[0], new RevoluteJoint(0, 2, RigidPose3d.Identity, RigidPose3d.Identity) });
        var state = new MultibodyState(new[] {
            new RigidBodyState(RigidPose3d.Identity, default, default),
            new RigidBodyState(new(Vector3d.Zero, Quaterniond.FromAxisAngle(Vector3d.UnitX, 0.2)), default, new(3, 0, 0)),
            new RigidBodyState(new(Vector3d.Zero, Quaterniond.FromAxisAngle(Vector3d.UnitX, -0.4)), default, new(-5, 0, 0)) });
        double initialEnergy = Energy(system, state);
        double[] history = { 0.2, -0.4 };
        for (int i = 0; i < 200; i++)
        {
            var step = ConstrainedMultibodyIntegrator.Step(system, state, i * 0.01, 0.01, NoLoads);
            state = step.State;
            for (int j = 0; j < 2; j++)
            {
                RevoluteJointSample sample = RevoluteJointKinematics.Evaluate(system, state, j);
                Assert.AreEqual(j == 0 ? 3 : -5, sample.AngularRate, 2e-14);
                Assert.AreEqual(0, step.AcceptedDynamics.RevoluteJointReactions[j].CoupleOnA.Length, 1e-13);
                history[j] = RevoluteJointKinematics.UnwrapAngle(history[j], sample.Angle);
            }
        }
        Assert.AreEqual(6.2, history[0], 0.0002);
        Assert.AreEqual(-10.4, history[1], 0.0006);
        Assert.AreEqual(initialEnergy, Energy(system, state), 2e-13);
        Assert.AreEqual(10, system.BilateralConstraintCount);
    }

    [TestMethod]
    [DataRow(1e-10)]
    [DataRow(1e-12)]
    public void RapidSpinAtArbitraryWorldOrientationRefinesAtSecondOrder(double angularTolerance)
    {
        var (system, initial) = Centered(phase: 0.3, spin: 500);
        Quaterniond rotation = Quaterniond.FromAxisAngle(new(1, 2, -3), 0.8);
        initial = Transform(initial, rotation, new(3, -2, 1));
        var settings = new ConstrainedSolverSettings(new AngularConstraintSettings(angularTolerance, angularTolerance));
        double[] errors = new double[3];
        double maximumAngle = 0, maximumRate = 0;
        for (int refinement = 0; refinement < 3; refinement++)
        {
            int count = 100 << refinement;
            double h = 0.08 / count;
            MultibodyState state = initial;
            double angle = 0.3;
            for (int i = 0; i < count; i++)
            {
                var result = ConstrainedMultibodyIntegrator.Step(system, state, i * h, h, NoLoads, settings);
                state = result.State;
                maximumAngle = System.Math.Max(maximumAngle, result.AcceptedDynamics.Diagnostics.MaximumAngularPositionError);
                maximumRate = System.Math.Max(maximumRate, result.AcceptedDynamics.Diagnostics.MaximumAngularVelocityError);
                var sample = RevoluteJointKinematics.Evaluate(system, state, 0);
                angle = RevoluteJointKinematics.UnwrapAngle(angle, sample.Angle);
                Assert.AreEqual(500, sample.AngularRate, 2e-10);
            }
            errors[refinement] = System.Math.Abs(angle - 40.3);
        }
        TestContext.WriteLine($"500 rad/s, angular tolerance={angularTolerance:E2}: phase errors={errors[0]:E9}/{errors[1]:E9}/{errors[2]:E9} rad; alignment={maximumAngle:E6} rad; rate={maximumRate:E6} 1/s");
        Assert.IsTrue(errors[0] / errors[1] > 3.9 && errors[0] / errors[1] < 4.1);
        Assert.IsTrue(errors[1] / errors[2] > 3.9 && errors[1] / errors[2] < 4.1);
        Assert.IsTrue(maximumAngle <= angularTolerance && maximumRate <= angularTolerance);
    }

    [TestMethod]
    [DataRow(1.0)]
    [DataRow(1000.0)]
    [DataRow(1000000.0)]
    public void InternalAxleTorquesConserveMomentumAndResolveLightWheelAtLargeInertiaRatios(double ratio)
    {
        var (original, initial) = Centered(spin: 10);
        var system = new MultibodySystem(new[] {
            new RigidBodyMassProperties(2 * ratio, Matrix3x3d.Diagonal(2 * ratio, 2 * ratio, 2 * ratio)), original.Bodies[1] },
            original.Joints, original.RevoluteJoints);
        Quaterniond rotation = Quaterniond.FromAxisAngle(new(1, -2, 3), 0.9);
        initial = Transform(initial, rotation, new(2, 3, -1));
        Vector3d axis = rotation.Rotate(Vector3d.UnitX);
        Vector3d initialMomentum = AngularMomentum(system, initial);
        MultibodyState state = initial;
        for (int i = 0; i < 20; i++)
        {
            state = ConstrainedMultibodyIntegrator.Step(system, state, i * 0.005, 0.005,
                (_, _) => new[] { new RigidBodyLoads(default, axis * -0.8), new(default, axis * 0.8) }).State;
        }
        Assert.AreEqual(10.4, Vector3d.Dot(axis, state.Bodies[1].WorldAngularVelocity), 2e-11);
        Assert.AreEqual(-0.08 / (2 * ratio), Vector3d.Dot(axis, state.Bodies[0].WorldAngularVelocity), 2e-13);
        TestMath.Near(initialMomentum, AngularMomentum(system, state), 2e-11);
        double angle = RevoluteJointKinematics.Evaluate(system, state, 0).Angle;
        double expected = 1 + 0.5 * 0.8 * (1 / (2 * ratio) + 1 / 0.2) * 0.1 * 0.1;
        Assert.AreEqual(expected, angle, 8e-5);
    }

    [TestMethod]
    public void GeneralFullTensorPairConvergesAndSeparatesProjectionEnergyChanges()
    {
        var (system, initial) = GeneralPair();
        MultibodyState reference = Integrate(0.00125, 320);
        double[] errors = { StateError(Integrate(0.02, 20), reference), StateError(Integrate(0.01, 40), reference), StateError(Integrate(0.005, 80), reference) };
        MultibodyState state = initial;
        double initialEnergy = Energy(system, initial), maxEnergy = 0, maxMomentum = 0;
        Vector3d initialMomentum = AngularMomentum(system, initial);
        bool projectedRotation = false;
        for (int i = 0; i < 200; i++)
        {
            var result = ConstrainedMultibodyIntegrator.Step(system, state, i * 0.005, 0.005, NoLoads);
            state = result.State;
            Assert.IsTrue(result.AcceptedDynamics.Diagnostics.MaximumPositionError <= 1e-10);
            Assert.IsTrue(result.AcceptedDynamics.Diagnostics.MaximumAngularPositionError <= 1e-10);
            Assert.IsTrue(result.AcceptedDynamics.Diagnostics.MaximumAngularVelocityError <= 1e-10);
            maxEnergy = System.Math.Max(maxEnergy, System.Math.Abs(Energy(system, state) / initialEnergy - 1));
            maxMomentum = System.Math.Max(maxMomentum, (AngularMomentum(system, state) - initialMomentum).Length);
            projectedRotation |= result.Diagnostics.BeforeProjectionAngularPositionError > 1e-10;
        }
        TestContext.WriteLine($"Full tensor pair: errors={errors[0]:E6}/{errors[1]:E6}/{errors[2]:E6}; max relative energy={maxEnergy:E6}; angular momentum={maxMomentum:E6} kg*m^2/s");
        Assert.IsTrue(errors[0] / errors[1] > 3.5 && errors[0] / errors[1] < 4.5);
        Assert.IsTrue(errors[1] / errors[2] > 3.5 && errors[1] / errors[2] < 4.8);
        Assert.IsTrue(maxEnergy < 1e-4 && maxMomentum < 1e-4);
        Assert.IsTrue(projectedRotation);

        MultibodyState Integrate(double h, int count)
        {
            MultibodyState current = initial;
            for (int i = 0; i < count; i++) { current = ConstrainedMultibodyIntegrator.Step(system, current, i * h, h, NoLoads).State; }
            return current;
        }
    }

    [TestMethod]
    public void EndpointReactionsAreInstantaneousAndRepeatedLoadsReceiveImmutableTrials()
    {
        var (system, initial) = Centered(spin: 2);
        int calls = 0;
        System.Collections.Generic.IReadOnlyList<RigidBodyLoads> Loads(double time, MultibodyState state)
        {
            calls++;
            foreach (RigidBodyState body in state.Bodies)
            {
                Quaterniond q = body.Pose.Orientation;
                Assert.AreEqual(1, q.W * q.W + q.X * q.X + q.Y * q.Y + q.Z * q.Z, 2e-15);
            }
            return new[] { new RigidBodyLoads(new(time * time, 0, 0), default), default };
        }
        var result = ConstrainedMultibodyIntegrator.Step(system, initial, 0, 0.2, Loads);
        var repeated = ConstrainedMultibodyIntegrator.Step(system, initial, 0, 0.2, Loads);
        Assert.AreEqual(-0.008, result.AcceptedDynamics.RevoluteJointReactions[0].ForceOnA.X, 2e-16);
        var direct = ConstrainedMultibodyDynamics.Evaluate(system, result.State, Loads(0.2, result.State));
        Assert.AreEqual(direct.RevoluteJointReactions[0], result.AcceptedDynamics.RevoluteJointReactions[0]);
        CollectionAssert.AreEqual(result.State.Bodies, repeated.State.Bodies);
        Assert.IsTrue(calls > 8);
        Assert.AreEqual(0, initial.Bodies[0].Pose.Position.Length);
    }

    [TestMethod]
    public void MixedJointIntegrationRetainsTypedReactionIndices()
    {
        var (pair, _) = Centered(spin: 2);
        var system = new MultibodySystem(new[] { pair.Bodies[0], pair.Bodies[1], new RigidBodyMassProperties(3, Matrix3x3d.Identity) },
            new[] { new SphericalJoint(0, 2, default, default) }, pair.RevoluteJoints);
        var rest = new RigidBodyState(RigidPose3d.Identity, default, default);
        var state = new MultibodyState(new[] { rest, new RigidBodyState(RigidPose3d.Identity, default, new(2, 0, 0)), rest });
        var result = ConstrainedMultibodyIntegrator.Step(system, state, 0, 0.2,
            (_, _) => new[] { new RigidBodyLoads(new(11, 0, 0), default), default, default });
        Assert.AreEqual(1, result.AcceptedDynamics.JointReactions.Count);
        Assert.AreEqual(1, result.AcceptedDynamics.RevoluteJointReactions.Count);
        foreach (RigidBodyState body in result.State.Bodies) { TestMath.Near(new(0.04, 0, 0), body.Pose.Position, 2e-14); }
        Assert.AreEqual(2, RevoluteJointKinematics.Evaluate(system, result.State, 0).AngularRate, 2e-14);
    }

    private static double StateError(MultibodyState a, MultibodyState b)
    {
        double result = 0;
        for (int i = 0; i < a.Bodies.Count; i++)
        {
            result += (a.Bodies[i].Pose.Position - b.Bodies[i].Pose.Position).Length;
            result += (a.Bodies[i].LinearVelocity - b.Bodies[i].LinearVelocity).Length;
            result += (a.Bodies[i].BodyAngularVelocity - b.Bodies[i].BodyAngularVelocity).Length;
            result += (a.Bodies[i].Pose.TransformDirection(Vector3d.UnitY) - b.Bodies[i].Pose.TransformDirection(Vector3d.UnitY)).Length;
        }
        return result;
    }
}
