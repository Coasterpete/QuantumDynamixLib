using System;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using QuantumDynamix.Geometry;
using QuantumDynamix.Math;
using QuantumDynamix.Physics;

namespace QuantumDynamix.Tests;

[TestClass]
public sealed class DynamicsTests
{
    [TestMethod]
    public void NewtonEulerUsesFullInertiaGyroscopicTermAndWorldTorque()
    {
        RigidBodyMassProperties mass = new(2, new(4, 1, 0.5, 1, 5, -0.25, 0.5, -0.25, 6));
        RigidBodyState state = new(
            new(Vector3d.Zero, Quaterniond.FromAxisAngle(Vector3d.UnitZ, System.Math.PI / 2)),
            new(3, -2, 1), new(1, 2, 3));

        // I*omega=(7.5,10.25,18), omega x I*omega=(5.25,4.5,-4.75).
        // Body torque=(13.75,0.75,14.5) produces alpha_body=(2,-1,3).
        RigidBodyDerivative derivative = RigidBodyDynamics.EvaluateDerivative(
            mass, state, new(new(4, -2, 10), new(-0.75, 13.75, 14.5)));

        Assert.AreEqual(state.LinearVelocity, derivative.PositionRate);
        TestMath.Near(new(2, -1, 5), derivative.LinearAcceleration, 0);
        TestMath.Near(new(2, -1, 3), derivative.BodyAngularAcceleration, 2e-15);
        // Allow a few ulps for quaternion normalization and trigonometric rotation.
        TestMath.Near(new(-2, 1, 3), state.WorldAngularVelocity, 2e-15);
    }

    [TestMethod]
    public void TorqueFreeAsymmetricBodyHasNonzeroAngularAcceleration()
    {
        RigidBodyMassProperties mass = new(1, Matrix3x3d.Diagonal(2, 3, 4));
        RigidBodyState state = new(RigidPose3d.Identity, Vector3d.Zero, new(1, 2, 3));
        RigidBodyDerivative derivative = RigidBodyDynamics.EvaluateDerivative(mass, state, default);
        TestMath.Near(new(-3, 2, -0.5), derivative.BodyAngularAcceleration, 1e-15);
        Assert.AreEqual(new Quaterniond(0, 0.5, 1, 1.5), derivative.OrientationRate);
    }

    [TestMethod]
    public void OffCenterForceCreatesWorldTorqueAndLoadsAdd()
    {
        RigidBodyLoads first = RigidBodyLoads.FromForceAtPoint(new(0, 10, 0), new(7, 0, 0), new(5, 0, 0));
        Assert.AreEqual(new Vector3d(0, 0, 20), first.Torque);
        RigidBodyLoads second = RigidBodyLoads.FromForceAtPoint(new(0, -10, 0), new(3, 0, 0), new(5, 0, 0));
        RigidBodyLoads couple = first + second;
        Assert.AreEqual(Vector3d.Zero, couple.Force);
        Assert.AreEqual(new Vector3d(0, 0, 40), couple.Torque);
    }

    [TestMethod]
    public void EnergyAndMomentumUseCenterOfMassAndConsistentFrames()
    {
        RigidBodyMassProperties mass = new(2, Matrix3x3d.Diagonal(2, 3, 4));
        RigidBodyState state = new(
            new(new(100, 200, 300), Quaterniond.FromAxisAngle(Vector3d.UnitZ, System.Math.PI / 2)),
            new(3, 4, 0), new(1, 2, 3));
        // T = 0.5*2*25 + 0.5*(2+12+36) = 50 J; H_body=(2,6,12).
        Assert.AreEqual(50, RigidBodyDynamics.KineticEnergy(mass, state), 0);
        TestMath.Near(new(-6, 2, 12), RigidBodyDynamics.AngularMomentum(mass, state), 2e-15);
    }

    [TestMethod]
    public void UniformGravityAtCenterOfMassDoesNotCreateAngularAcceleration()
    {
        RigidBodyMassProperties mass = new(10, Matrix3x3d.Diagonal(2, 3, 4));
        RigidBodyState state = new(RigidPose3d.Identity, Vector3d.Zero, Vector3d.Zero);
        RigidBodyLoads gravity = RigidBodyLoads.FromForceAtPoint(new(0, 0, -98), state.Pose.Position, state.Pose.Position);
        RigidBodyDerivative derivative = RigidBodyDynamics.EvaluateDerivative(mass, state, gravity);
        Assert.AreEqual(Vector3d.Zero, derivative.BodyAngularAcceleration);
        TestMath.Near(new(0, 0, -9.8), derivative.LinearAcceleration, 0);
    }
}
