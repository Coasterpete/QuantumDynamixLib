using System;
using System.Collections.Generic;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using QuantumDynamix.Geometry;
using QuantumDynamix.Math;
using QuantumDynamix.Physics;
using static QuantumDynamix.Tests.RevoluteTestFixtures;

namespace QuantumDynamix.Tests;

// All properties, speeds, and tolerances are synthetic SI regression fixtures.
[TestClass]
public sealed class RevoluteDynamicsTests
{
    public TestContext TestContext { get; set; } = null!;

    [TestMethod]
    public void RelativeRateUsesWorldVectorsAndAttachmentFramesDefineSignedAngle()
    {
        var (system, state) = GeneralPair();
        RevoluteJointSample sample = RevoluteJointKinematics.Evaluate(system, state, 0);
        Assert.AreEqual(0.7, sample.Angle, 2e-15);
        Assert.AreEqual(2.3, sample.AngularRate, 3e-15);
        Assert.AreEqual(1, sample.WorldAxis.Length, 1e-15);
        double wrong = Vector3d.Dot(sample.WorldAxis,
            state.Bodies[1].BodyAngularVelocity - state.Bodies[0].BodyAngularVelocity);
        Assert.IsTrue(System.Math.Abs(wrong - sample.AngularRate) > 0.1);
        double h = 1e-6;
        double derivative = (RevoluteJointKinematics.Evaluate(system, Flow(state, h), 0).Angle
            - RevoluteJointKinematics.Evaluate(system, Flow(state, -h), 0).Angle) / (2 * h);
        Assert.AreEqual(sample.AngularRate, derivative, 2e-9);
    }

    [TestMethod]
    [DataRow(0.0, 2.3)]
    [DataRow(0.03, 2.3)]
    [DataRow(0.0, 500.0)]
    public void AnalyticRowsAndBiasMatchIndependentGeometricPerturbations(double tilt, double spin)
    {
        var (system, initial) = GeneralPair(spin: spin);
        RigidBodyState[] bodies = { initial.Bodies[0], initial.Bodies[1] };
        RigidBodyState b = bodies[1];
        bodies[1] = new(new(b.Pose.Position + new Vector3d(tilt, -tilt / 2, 0),
            b.Pose.Orientation * Quaterniond.FromAxisAngle(new(1, 2, 3), tilt)), b.LinearVelocity, b.BodyAngularVelocity);
        MultibodyState state = new(bodies);
        ConstraintAssembly assembly = new(system, state, ConstrainedSolverSettings.Default, "Derivative regression");
        const double epsilon = 2e-5;
        double maxJacobianError = 0;
        for (int column = 0; column < 12; column++)
        {
            double[] plus = GeometricErrors(system.RevoluteJoints[0], Displace(state, column, epsilon));
            double[] minus = GeometricErrors(system.RevoluteJoints[0], Displace(state, column, -epsilon));
            for (int row = 0; row < 5; row++)
            {
                double numerical = (plus[row] - minus[row]) / (2 * epsilon);
                maxJacobianError = System.Math.Max(maxJacobianError, System.Math.Abs(numerical - assembly.Jacobian[row, column]));
                Assert.AreEqual(numerical, assembly.Jacobian[row, column], 2e-9, $"row={row}, column={column}");
            }
        }
        double speed = System.Math.Max(1, System.Math.Max(state.Bodies[0].WorldAngularVelocity.Length, state.Bodies[1].WorldAngularVelocity.Length));
        double h = 0.001 / speed;
        double[] center = GeometricErrors(system.RevoluteJoints[0], state);
        double[] high = GeometricErrors(system.RevoluteJoints[0], Flow(state, h));
        double[] low = GeometricErrors(system.RevoluteJoints[0], Flow(state, -h));
        double[] halfHigh = GeometricErrors(system.RevoluteJoints[0], Flow(state, h / 2));
        double[] halfLow = GeometricErrors(system.RevoluteJoints[0], Flow(state, -h / 2));
        double maxBiasScaledError = 0;
        for (int row = 0; row < 5; row++)
        {
            double coarse = (high[row] - 2 * center[row] + low[row]) / (h * h);
            double fine = (halfHigh[row] - 2 * center[row] + halfLow[row]) / (h * h / 4);
            double numerical = (4 * fine - coarse) / 3;
            // Unit fixture scales: 1 s, with 1 m for point rows and unit alignment
            // for angular rows. The speed-squared term includes that row's length scale.
            double scale = 1 + System.Math.Abs(assembly.Bias[row]) + speed * speed;
            maxBiasScaledError = System.Math.Max(maxBiasScaledError, System.Math.Abs(numerical - assembly.Bias[row]) / scale);
            Assert.AreEqual(numerical, assembly.Bias[row], 2e-7 * scale, $"bias row={row}");
        }
        TestContext.WriteLine($"tilt={tilt}, spin={spin}: max J error={maxJacobianError:E6}; bias scaled error={maxBiasScaledError:E6}");
    }

    [TestMethod]
    public void CenterBearingSharesForceAndTransverseTorqueButPermitsAxialAcceleration()
    {
        var (system, state) = Centered();
        var result = ConstrainedMultibodyDynamics.Evaluate(system, state,
            new[] { new RigidBodyLoads(new(10, 0, 0), new(1, 3, 0)), default });
        TestMath.Near(new(4, 0, 0), result.Derivatives[0].LinearAcceleration, 2e-14);
        TestMath.Near(new(4, 0, 0), result.Derivatives[1].LinearAcceleration, 2e-14);
        RevoluteJointReaction reaction = result.RevoluteJointReactions[0];
        TestMath.Near(new(-2, 0, 0), reaction.ForceOnA, 2e-14);
        TestMath.Near(new(0, -3 * 0.3 / 2.3, 0), reaction.CoupleOnA, 2e-14);
        Assert.AreEqual(0.5, result.Derivatives[0].BodyAngularAcceleration.X, 2e-15);
        Assert.AreEqual(0, result.Derivatives[1].BodyAngularAcceleration.X, 2e-15);
        Assert.AreEqual(0, result.Diagnostics.MaximumAngularAccelerationError, 2e-14);
        Assert.AreEqual(0, result.JointReactions.Count);
    }

    [TestMethod]
    public void InternalAxleTorqueHasAnalyticalRelativeAccelerationAndNoBearingCouple()
    {
        var (system, state) = Centered(spin: 4);
        const double torque = 0.8;
        var result = ConstrainedMultibodyDynamics.Evaluate(system, state,
            new[] { new RigidBodyLoads(default, new(-torque, 0, 0)), new(default, new(torque, 0, 0)) });
        Assert.AreEqual(-torque / 2, result.Derivatives[0].BodyAngularAcceleration.X, 2e-15);
        Assert.AreEqual(torque / 0.2, result.Derivatives[1].BodyAngularAcceleration.X, 2e-15);
        TestMath.Near(Vector3d.Zero, result.RevoluteJointReactions[0].CoupleOnA, 0);
        double acceleration = result.Derivatives[1].BodyAngularAcceleration.X - result.Derivatives[0].BodyAngularAcceleration.X;
        Assert.AreEqual(torque * (1 / 2.0 + 1 / 0.2), acceleration, 2e-15);
    }

    [TestMethod]
    [DataRow(80.0)]
    [DataRow(-80.0)]
    [DataRow(500.0)]
    [DataRow(-500.0)]
    public void FiniteCarrierGyroscopicCoupleAndAccelerationAreAnalytical(double spin)
    {
        const double carrierInertia = 2, axial = 0.2, transverse = 0.3, turning = 0.7;
        var (system, _) = Centered();
        var state = new MultibodyState(new[] {
            new RigidBodyState(RigidPose3d.Identity, default, new(0, 0, turning)),
            new RigidBodyState(RigidPose3d.Identity, default, new(spin, 0, turning)) });
        var result = ConstrainedMultibodyDynamics.Evaluate(system, state, NoLoads(0, state));
        double alpha = -axial * spin * turning / (carrierInertia + transverse);
        double couple = -carrierInertia * alpha;
        TestMath.Near(new(0, alpha, 0), result.Derivatives[0].BodyAngularAcceleration, 2e-12);
        TestMath.Near(new(0, alpha + spin * turning, 0), result.Derivatives[1].BodyAngularAcceleration, 2e-12);
        TestMath.Near(new(0, couple, 0), result.RevoluteJointReactions[0].CoupleOnB, 2e-12);
        Assert.IsTrue(result.Diagnostics.MaximumAngularAccelerationError < 2e-12);
        Assert.AreEqual(0, result.RevoluteJointReactions[0].ForceOnA.Length);
    }

    [TestMethod]
    public void FullTensorsOffCenterBearingReactionsBalanceForceMomentAndPower()
    {
        var (system, state) = GeneralPair();
        var result = ConstrainedMultibodyDynamics.Evaluate(system, state,
            new[] { new RigidBodyLoads(new(2, -3, 4), new(-0.3, 0.4, 0.7)), new(new(-1, 2, -3), new(0.2, -0.6, 0.4)) });
        RevoluteJointReaction reaction = result.RevoluteJointReactions[0];
        Vector3d force = default, moment = default;
        double power = 0;
        for (int i = 0; i < 2; i++)
        {
            RigidBodyLoads load = result.ConstraintLoads[i];
            force += load.Force;
            moment += load.Torque + Vector3d.Cross(state.Bodies[i].Pose.Position, load.Force);
            power += Vector3d.Dot(state.Bodies[i].LinearVelocity, load.Force) + Vector3d.Dot(state.Bodies[i].WorldAngularVelocity, load.Torque);
        }
        TestMath.Near(Vector3d.Zero, force, 0);
        TestMath.Near(Vector3d.Zero, moment, 3e-13);
        Assert.AreEqual(0, power, 3e-13);
        Assert.AreEqual(0, Vector3d.Dot(reaction.CoupleOnA, RevoluteJointKinematics.Evaluate(system, state, 0).WorldAxis), 2e-13);
        RevoluteJoint joint = system.RevoluteJoints[0];
        TestMath.Near(reaction.TorqueOnA, Vector3d.Cross(state.Bodies[0].Pose.TransformDirection(joint.LocalFrameA.Position), reaction.ForceOnA) + reaction.CoupleOnA, 0);
        Assert.IsTrue(result.Diagnostics.MaximumAccelerationError < 2e-13);
        Assert.IsTrue(result.Diagnostics.MaximumAngularAccelerationError < 2e-13);
    }

    [TestMethod]
    public void WorldRotationAndBodyReversalTransformReactionsConsistently()
    {
        var (system, state) = GeneralPair();
        RigidBodyLoads[] loads = { new(new(2, 3, -1), new(0.2, -0.3, 0.4)), default };
        var original = ConstrainedMultibodyDynamics.Evaluate(system, state, loads);
        Quaterniond rotation = Quaterniond.FromAxisAngle(new(2, -1, 3), 0.8);
        var transformed = ConstrainedMultibodyDynamics.Evaluate(system, Transform(state, rotation, new(10, -7, 3)),
            new[] { new RigidBodyLoads(rotation.Rotate(loads[0].Force), rotation.Rotate(loads[0].Torque)), default });
        TestMath.Near(rotation.Rotate(original.RevoluteJointReactions[0].ForceOnA), transformed.RevoluteJointReactions[0].ForceOnA, 3e-13);
        TestMath.Near(rotation.Rotate(original.RevoluteJointReactions[0].CoupleOnA), transformed.RevoluteJointReactions[0].CoupleOnA, 3e-13);
        RevoluteJoint joint = system.RevoluteJoints[0];
        MultibodySystem swapped = new(new[] { system.Bodies[1], system.Bodies[0] }, Array.Empty<SphericalJoint>(),
            new[] { new RevoluteJoint(0, 1, joint.LocalFrameB, joint.LocalFrameA) });
        var reverse = ConstrainedMultibodyDynamics.Evaluate(swapped, new(new[] { state.Bodies[1], state.Bodies[0] }), new[] { loads[1], loads[0] });
        TestMath.Near(original.RevoluteJointReactions[0].ForceOnB, reverse.RevoluteJointReactions[0].ForceOnA, 3e-13);
        TestMath.Near(original.RevoluteJointReactions[0].CoupleOnB, reverse.RevoluteJointReactions[0].CoupleOnA, 3e-13);
        Assert.AreEqual(-0.7, RevoluteJointKinematics.Evaluate(swapped, new(new[] { state.Bodies[1], state.Bodies[0] }), 0).Angle, 2e-15);
    }

    [TestMethod]
    public void MixedSphericalRevoluteAndDisconnectedBodiesUseAllRows()
    {
        var (pair, _) = Centered();
        var system = new MultibodySystem(new[] { pair.Bodies[0], pair.Bodies[1], new RigidBodyMassProperties(3, Matrix3x3d.Identity), pair.Bodies[1] },
            new[] { new SphericalJoint(0, 2, default, default) }, pair.RevoluteJoints);
        var rest = new RigidBodyState(RigidPose3d.Identity, default, default);
        var state = new MultibodyState(new[] { rest, rest, rest, rest });
        var result = ConstrainedMultibodyDynamics.Evaluate(system, state,
            new[] { new RigidBodyLoads(new(11, 0, 0), default), default, default, new(new(0, 1, 0), default) });
        Assert.AreEqual(8, system.BilateralConstraintCount);
        for (int i = 0; i < 3; i++) { TestMath.Near(new(2, 0, 0), result.Derivatives[i].LinearAcceleration, 2e-14); }
        TestMath.Near(new(0, 2, 0), result.Derivatives[3].LinearAcceleration, 0);
        TestMath.Near(new(-6, 0, 0), result.JointReactions[0].ForceOnA, 2e-14);
        TestMath.Near(new(-1, 0, 0), result.RevoluteJointReactions[0].ForceOnA, 2e-14);
    }

    private static double[] GeometricErrors(RevoluteJoint joint, MultibodyState state)
    {
        // Independent geometry-only definition: no production frames, Jacobian, or bias helpers.
        RigidBodyState a = state.Bodies[joint.BodyA], b = state.Bodies[joint.BodyB];
        Vector3d gap = a.Pose.TransformPoint(joint.LocalFrameA.Position) - b.Pose.TransformPoint(joint.LocalFrameB.Position);
        Quaterniond qa = a.Pose.Orientation * joint.LocalFrameA.Orientation;
        Quaterniond qb = b.Pose.Orientation * joint.LocalFrameB.Orientation;
        return new[] { gap.X, gap.Y, gap.Z, Vector3d.Dot(qa.Rotate(Vector3d.UnitY), qb.Rotate(Vector3d.UnitX)),
            Vector3d.Dot(qa.Rotate(Vector3d.UnitZ), qb.Rotate(Vector3d.UnitX)) };
    }

    private static MultibodyState Displace(MultibodyState state, int column, double amount)
    {
        RigidBodyState[] bodies = { state.Bodies[0], state.Bodies[1] };
        int index = column / 6, coordinate = column % 6;
        Vector3d axis = coordinate % 3 == 0 ? Vector3d.UnitX : coordinate % 3 == 1 ? Vector3d.UnitY : Vector3d.UnitZ;
        RigidBodyState body = bodies[index];
        RigidPose3d pose = coordinate < 3 ? new(body.Pose.Position + axis * amount, body.Pose.Orientation)
            : new(body.Pose.Position, body.Pose.Orientation * Quaterniond.FromAxisAngle(axis, amount));
        bodies[index] = new(pose, body.LinearVelocity, body.BodyAngularVelocity);
        return new(bodies);
    }
}

internal static class RevoluteTestFixtures
{
    internal static IReadOnlyList<RigidBodyLoads> NoLoads(double time, MultibodyState state) => new RigidBodyLoads[state.Bodies.Count];

    internal static (MultibodySystem System, MultibodyState State) Centered(double phase = 0, double spin = 0)
    {
        var system = new MultibodySystem(new[] {
            new RigidBodyMassProperties(2, Matrix3x3d.Diagonal(2, 2, 2)),
            new RigidBodyMassProperties(0.5, Matrix3x3d.Diagonal(0.2, 0.3, 0.3)) }, Array.Empty<SphericalJoint>(),
            new[] { new RevoluteJoint(0, 1, RigidPose3d.Identity, RigidPose3d.Identity) });
        var state = new MultibodyState(new[] {
            new RigidBodyState(RigidPose3d.Identity, default, default),
            new RigidBodyState(new(Vector3d.Zero, Quaterniond.FromAxisAngle(Vector3d.UnitX, phase)), default, new(spin, 0, 0)) });
        return (system, state);
    }

    internal static (MultibodySystem System, MultibodyState State) GeneralPair(double spin = 2.3)
    {
        RigidPose3d a = new(new(-0.8, 0.3, -0.2), Quaterniond.FromAxisAngle(new(1, 2, 3), 0.7));
        RigidPose3d b = new(new(0.7, -0.4, 0.5), Quaterniond.FromAxisAngle(new(-2, 1, 3), -0.9));
        Quaterniond frame = Quaterniond.FromAxisAngle(new(1, -2, 3), 0.4);
        Vector3d point = new(0.1, 0.2, -0.1);
        RigidPose3d localA = new(a.InverseTransformPoint(point), a.Orientation.Conjugated() * frame);
        RigidPose3d localB = new(b.InverseTransformPoint(point), b.Orientation.Conjugated() * frame * Quaterniond.FromAxisAngle(Vector3d.UnitX, 0.7));
        Vector3d omegaA = new(0.3, 0.7, -0.2), omegaB = omegaA + frame.Rotate(Vector3d.UnitX) * spin;
        Vector3d velocityA = new(0.2, -0.4, 0.3);
        Vector3d velocityB = velocityA + Vector3d.Cross(omegaA, point - a.Position) - Vector3d.Cross(omegaB, point - b.Position);
        var system = new MultibodySystem(new[] {
            new RigidBodyMassProperties(2, new(4, 0.5, -0.2, 0.5, 5, 0.3, -0.2, 0.3, 6)),
            new RigidBodyMassProperties(3, new(3, -0.3, 0.4, -0.3, 4, 0.2, 0.4, 0.2, 5)) }, Array.Empty<SphericalJoint>(),
            new[] { new RevoluteJoint(0, 1, localA, localB) });
        return (system, new(new[] { new RigidBodyState(a, velocityA, a.InverseTransformDirection(omegaA)),
            new RigidBodyState(b, velocityB, b.InverseTransformDirection(omegaB)) }));
    }

    internal static MultibodyState Flow(MultibodyState state, double time)
    {
        var bodies = new RigidBodyState[state.Bodies.Count];
        for (int i = 0; i < bodies.Length; i++)
        {
            RigidBodyState body = state.Bodies[i];
            double speed = body.BodyAngularVelocity.Length;
            Quaterniond change = speed == 0 ? Quaterniond.Identity : Quaterniond.FromAxisAngle(body.BodyAngularVelocity, speed * time);
            bodies[i] = new(new(body.Pose.Position + body.LinearVelocity * time, body.Pose.Orientation * change), body.LinearVelocity, body.BodyAngularVelocity);
        }
        return new(bodies);
    }

    internal static MultibodyState Transform(MultibodyState state, Quaterniond rotation, Vector3d translation)
    {
        var bodies = new RigidBodyState[state.Bodies.Count];
        for (int i = 0; i < bodies.Length; i++)
        {
            RigidBodyState b = state.Bodies[i];
            bodies[i] = new(new(translation + rotation.Rotate(b.Pose.Position), rotation * b.Pose.Orientation), rotation.Rotate(b.LinearVelocity), b.BodyAngularVelocity);
        }
        return new(bodies);
    }

    internal static double Energy(MultibodySystem system, MultibodyState state)
    {
        double result = 0;
        for (int i = 0; i < state.Bodies.Count; i++) { result += RigidBodyDynamics.KineticEnergy(system.Bodies[i], state.Bodies[i]); }
        return result;
    }

    internal static Vector3d AngularMomentum(MultibodySystem system, MultibodyState state)
    {
        Vector3d result = default;
        for (int i = 0; i < state.Bodies.Count; i++)
        {
            RigidBodyState body = state.Bodies[i];
            result += RigidBodyDynamics.AngularMomentum(system.Bodies[i], body)
                + Vector3d.Cross(body.Pose.Position, body.LinearVelocity * system.Bodies[i].Mass);
        }
        return result;
    }
}
