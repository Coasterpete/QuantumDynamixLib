using System;
using System.Collections.Generic;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using QuantumDynamix.Geometry;
using QuantumDynamix.Math;
using QuantumDynamix.Physics;

namespace QuantumDynamix.Tests;

[TestClass]
public sealed class ConstrainedDynamicsTests
{
    [TestMethod]
    public void CenterAnchorsShareForceAccordingToDistinctMasses()
    {
        MultibodySystem system = new(new[] { Mass(2), Mass(3) }, new[] { new SphericalJoint(0, 1, default, default) });
        MultibodyState state = new(new[] { Rest(), Rest() });
        ConstrainedDynamicsResult result = ConstrainedMultibodyDynamics.Evaluate(system, state,
            new[] { new RigidBodyLoads(new(10, 0, 0), default), default });
        TestMath.Near(new(2, 0, 0), result.Derivatives[0].LinearAcceleration, 2e-15);
        TestMath.Near(new(2, 0, 0), result.Derivatives[1].LinearAcceleration, 2e-15);
        TestMath.Near(new(-6, 0, 0), result.JointReactions[0].ForceOnA, 2e-15);
        Assert.AreEqual(Vector3d.Zero, result.JointReactions[0].TorqueOnA);
        Assert.IsTrue(result.Diagnostics.MaximumAccelerationError < 3e-15);
    }

    [TestMethod]
    public void OffCenterForceCouplesTranslationAndRotationAnalytically()
    {
        MultibodySystem system = new(new[] { Mass(2, new(2, 3, 4)), Mass(3, new(3, 4, 5)) },
            new[] { new SphericalJoint(0, 1, Vector3d.UnitY, -Vector3d.UnitY) });
        MultibodyState state = new(new[] { Rest(new(0, -1, 0)), Rest(new(0, 1, 0)) });
        ConstrainedDynamicsResult result = ConstrainedMultibodyDynamics.Evaluate(system, state,
            new[] { new RigidBodyLoads(new(10, 0, 0), default), default });
        // W_xx=1/2+1/3+1/4+1/5=77/60 kg^-1. lambda_x=-5/W_xx=-300/77 N.
        double force = 300.0 / 77;
        TestMath.Near(new(-force, 0, 0), result.JointReactions[0].ForceOnA, 2e-15);
        TestMath.Near(new(0, 0, force), result.JointReactions[0].TorqueOnA, 2e-15);
        TestMath.Near(new(0, 0, force), result.JointReactions[0].TorqueOnB, 2e-15);
        TestMath.Near(new(0, 0, force / 4), result.Derivatives[0].BodyAngularAcceleration, 2e-15);
        TestMath.Near(new(0, 0, force / 5), result.Derivatives[1].BodyAngularAcceleration, 2e-15);
        TestMath.Near(new(160.0 / 77, 0, 0), AnchorAcceleration(state.Bodies[0], result.Derivatives[0], Vector3d.UnitY), 3e-15);
        TestMath.Near(new(160.0 / 77, 0, 0), AnchorAcceleration(state.Bodies[1], result.Derivatives[1], -Vector3d.UnitY), 3e-15);
    }

    [TestMethod]
    public void RotatingPairRequiresCentripetalReactionWithoutExternalLoads()
    {
        (MultibodySystem system, MultibodyState state) = RotatingPair(speed: 2);
        ConstrainedDynamicsResult result = ConstrainedMultibodyDynamics.Evaluate(system, state, NoLoads(0, state));
        TestMath.Near(new(8, 0, 0), result.JointReactions[0].ForceOnA, 3e-15);
        TestMath.Near(new(4, 0, 0), result.Derivatives[0].LinearAcceleration, 2e-15);
        TestMath.Near(new(-8.0 / 3, 0, 0), result.Derivatives[1].LinearAcceleration, 2e-15);
        Assert.AreEqual(Vector3d.Zero, result.JointReactions[0].TorqueOnA);
        Assert.IsTrue(result.Diagnostics.MaximumAccelerationError < 5e-15);
    }

    [TestMethod]
    public void CenterJointPermitsIndependentOrientationAndSpin()
    {
        RigidBodyState a = new(RigidPose3d.Identity, Vector3d.Zero, new(1, 2, 3));
        RigidBodyState b = new(new(Vector3d.Zero, Quaterniond.FromAxisAngle(new(1, 2, 3), 1.1)), Vector3d.Zero, new(-2, 1, 4));
        MultibodySystem system = new(new[] { Mass(2, new(2, 3, 4)), Mass(5, new(4, 6, 9)) },
            new[] { new SphericalJoint(0, 1, default, default) });
        MultibodyState state = new(new[] { a, b });
        ConstrainedDynamicsResult result = ConstrainedMultibodyDynamics.Evaluate(system, state, NoLoads(0, state));
        Assert.AreEqual(Vector3d.Zero, result.JointReactions[0].ForceOnA);
        Assert.AreEqual(RigidBodyDynamics.EvaluateDerivative(system.Bodies[0], a, default), result.Derivatives[0]);
        Assert.AreEqual(RigidBodyDynamics.EvaluateDerivative(system.Bodies[1], b, default), result.Derivatives[1]);
    }

    [TestMethod]
    public void RotatedFullTensorsSatisfyAnchorAccelerationPowerAndMomentumBalance()
    {
        (MultibodySystem system, MultibodyState state) = GeneralPair();
        ConstrainedDynamicsResult result = ConstrainedMultibodyDynamics.Evaluate(system, state,
            new[] { new RigidBodyLoads(new(2, -3, 4), new(-0.3, 0.4, 0.7)), new(new(-1, 2, -3), new(0.2, -0.6, 0.4)) });
        SphericalJoint joint = system.Joints[0];
        TestMath.Near(AnchorAcceleration(state.Bodies[0], result.Derivatives[0], joint.LocalAnchorA),
            AnchorAcceleration(state.Bodies[1], result.Derivatives[1], joint.LocalAnchorB), 2e-14);
        Vector3d totalForce = Vector3d.Zero;
        Vector3d totalMoment = Vector3d.Zero;
        double power = 0;
        for (int i = 0; i < 2; i++)
        {
            RigidBodyLoads loads = result.ConstraintLoads[i];
            totalForce += loads.Force;
            totalMoment += loads.Torque + Vector3d.Cross(state.Bodies[i].Pose.Position, loads.Force);
            power += Vector3d.Dot(state.Bodies[i].LinearVelocity, loads.Force)
                + Vector3d.Dot(state.Bodies[i].WorldAngularVelocity, loads.Torque);
        }

        TestMath.Near(Vector3d.Zero, totalForce, 0);
        TestMath.Near(Vector3d.Zero, totalMoment, 2e-14);
        Assert.AreEqual(0, power, 2e-14);
        Assert.IsTrue(result.Diagnostics.LinearSolveRelativeResidual < 1e-14);
    }

    [TestMethod]
    public void ThreeBodyChainSolvesSharedBodyCoupling()
    {
        MultibodySystem system = new(new[] { Mass(2), Mass(3), Mass(5) },
            new[] { new SphericalJoint(0, 1, default, default), new SphericalJoint(1, 2, default, default) });
        MultibodyState state = new(new[] { Rest(), Rest(), Rest() });
        ConstrainedDynamicsResult result = ConstrainedMultibodyDynamics.Evaluate(system, state,
            new[] { new RigidBodyLoads(new(10, 0, 0), default), default, default });
        foreach (RigidBodyDerivative derivative in result.Derivatives)
        {
            TestMath.Near(Vector3d.UnitX, derivative.LinearAcceleration, 3e-15);
        }

        TestMath.Near(new(-8, 0, 0), result.JointReactions[0].ForceOnA, 3e-15);
        TestMath.Near(new(-5, 0, 0), result.JointReactions[1].ForceOnA, 3e-15);
    }

    [TestMethod]
    public void WorldRotationAndBodyOrderTransformReactionConsistently()
    {
        (MultibodySystem system, MultibodyState state) = GeneralPair();
        RigidBodyLoads[] loads = { new(new(2, 3, -1), new(0.2, -0.3, 0.4)), default };
        ConstrainedDynamicsResult original = ConstrainedMultibodyDynamics.Evaluate(system, state, loads);
        Quaterniond rotation = Quaterniond.FromAxisAngle(new(2, -1, 3), 0.8);
        RigidBodyState[] rotated = new RigidBodyState[2];
        RigidBodyLoads[] rotatedLoads = new RigidBodyLoads[2];
        for (int i = 0; i < 2; i++)
        {
            RigidBodyState s = state.Bodies[i];
            rotated[i] = new(new(rotation.Rotate(s.Pose.Position), rotation * s.Pose.Orientation),
                rotation.Rotate(s.LinearVelocity), s.BodyAngularVelocity);
            rotatedLoads[i] = new(rotation.Rotate(loads[i].Force), rotation.Rotate(loads[i].Torque));
        }

        ConstrainedDynamicsResult transformed = ConstrainedMultibodyDynamics.Evaluate(system, new(rotated), rotatedLoads);
        TestMath.Near(rotation.Rotate(original.JointReactions[0].ForceOnA), transformed.JointReactions[0].ForceOnA, 2e-14);
        SphericalJoint joint = system.Joints[0];
        MultibodySystem swapped = new(new[] { system.Bodies[1], system.Bodies[0] },
            new[] { new SphericalJoint(0, 1, joint.LocalAnchorB, joint.LocalAnchorA) });
        ConstrainedDynamicsResult reversed = ConstrainedMultibodyDynamics.Evaluate(swapped,
            new(new[] { state.Bodies[1], state.Bodies[0] }), new[] { loads[1], loads[0] });
        TestMath.Near(-original.JointReactions[0].ForceOnA, reversed.JointReactions[0].ForceOnA, 2e-14);
    }

    [TestMethod]
    [DataRow(1e-6)]
    [DataRow(1e6)]
    public void UniformMassInertiaAndForceScalingPreservesAccelerations(double scale)
    {
        (MultibodySystem system, MultibodyState state) = GeneralPair();
        RigidBodyLoads[] loads = { new(new(2, 3, 4), new(0.2, 0.3, 0.4)), default };
        ConstrainedDynamicsResult original = ConstrainedMultibodyDynamics.Evaluate(system, state, loads);
        RigidBodyMassProperties[] masses = new RigidBodyMassProperties[2];
        for (int i = 0; i < 2; i++)
        {
            // Matrix multiplication scales every entry without requiring a new production matrix operator.
            masses[i] = new(system.Bodies[i].Mass * scale,
                Matrix3x3d.Diagonal(scale, scale, scale) * system.Bodies[i].BodyInertia);
        }

        MultibodySystem scaled = new(masses, system.Joints);
        ConstrainedDynamicsResult result = ConstrainedMultibodyDynamics.Evaluate(scaled, state,
            new[] { new RigidBodyLoads(loads[0].Force * scale, loads[0].Torque * scale), default });
        TestMath.Near(original.JointReactions[0].ForceOnA, result.JointReactions[0].ForceOnA / scale, 3e-14);
        for (int i = 0; i < 2; i++)
        {
            TestMath.Near(original.Derivatives[i].LinearAcceleration, result.Derivatives[i].LinearAcceleration, 2e-14);
            TestMath.Near(original.Derivatives[i].BodyAngularAcceleration, result.Derivatives[i].BodyAngularAcceleration, 2e-14);
        }
    }

    internal static RigidBodyMassProperties Mass(double mass, Vector3d? inertia = null)
    {
        Vector3d diagonal = inertia ?? new Vector3d(mass, mass, mass);
        return new(mass, Matrix3x3d.Diagonal(diagonal.X, diagonal.Y, diagonal.Z));
    }

    internal static RigidBodyState Rest(Vector3d position = default) => new(new(position, Quaterniond.Identity), default, default);

    internal static IReadOnlyList<RigidBodyLoads> NoLoads(double time, MultibodyState state) => new RigidBodyLoads[state.Bodies.Count];

    internal static (MultibodySystem, MultibodyState) RotatingPair(double speed = 1, double massRatio = 1.5)
    {
        double a = 1;
        double b = 1 / massRatio;
        MultibodySystem system = new(new[] { Mass(2, new(2, 3, 4)), Mass(2 * massRatio, new Vector3d(3, 4, 5) * massRatio) },
            new[] { new SphericalJoint(0, 1, new(a, 0, 0), new(-b, 0, 0)) });
        MultibodyState state = new(new[]
        {
            new RigidBodyState(new(new(-a, 0, 0), Quaterniond.Identity), new(0, -speed * a, 0), new(0, 0, speed)),
            new RigidBodyState(new(new(b, 0, 0), Quaterniond.Identity), new(0, speed * b, 0), new(0, 0, speed))
        });
        return (system, state);
    }

    internal static (MultibodySystem, MultibodyState) GeneralPair()
    {
        RigidPose3d poseA = new(new(-0.8, 0.3, -0.2), Quaterniond.FromAxisAngle(new(1, 2, 3), 0.7));
        RigidPose3d poseB = new(new(0.7, -0.4, 0.5), Quaterniond.FromAxisAngle(new(-2, 1, 3), -0.9));
        Vector3d jointPoint = new(0.1, 0.2, -0.1);
        Vector3d sA = poseA.InverseTransformPoint(jointPoint);
        Vector3d sB = poseB.InverseTransformPoint(jointPoint);
        Vector3d omegaA = new(0.3, 0.7, -0.2);
        Vector3d omegaB = new(-0.5, 0.2, 0.6);
        Vector3d velocityA = new(0.2, -0.4, 0.3);
        Vector3d velocityB = velocityA + poseA.TransformDirection(Vector3d.Cross(omegaA, sA))
            - poseB.TransformDirection(Vector3d.Cross(omegaB, sB));
        MultibodySystem system = new(new[]
        {
            new RigidBodyMassProperties(2, new(4, 0.5, -0.2, 0.5, 5, 0.3, -0.2, 0.3, 6)),
            new RigidBodyMassProperties(3, new(3, -0.3, 0.4, -0.3, 4, 0.2, 0.4, 0.2, 5))
        }, new[] { new SphericalJoint(0, 1, sA, sB) });
        return (system, new(new[] { new RigidBodyState(poseA, velocityA, omegaA), new RigidBodyState(poseB, velocityB, omegaB) }));
    }

    private static Vector3d AnchorAcceleration(RigidBodyState state, RigidBodyDerivative derivative, Vector3d anchor) =>
        derivative.LinearAcceleration + state.Pose.TransformDirection(
            Vector3d.Cross(derivative.BodyAngularAcceleration, anchor)
            + Vector3d.Cross(state.BodyAngularVelocity, Vector3d.Cross(state.BodyAngularVelocity, anchor)));
}
