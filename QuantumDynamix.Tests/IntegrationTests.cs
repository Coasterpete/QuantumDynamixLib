using System;
using System.Collections.Generic;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using QuantumDynamix.Geometry;
using QuantumDynamix.Math;
using QuantumDynamix.Physics;

namespace QuantumDynamix.Tests;

[TestClass]
public sealed class IntegrationTests
{
    public TestContext TestContext { get; set; } = null!;

    [TestMethod]
    public void UnloadedBodyTranslatesAtConstantVelocity()
    {
        RigidBodyMassProperties mass = new(3, Matrix3x3d.Identity);
        RigidBodyState initial = new(new(new(3, -2, 7), Quaterniond.Identity), new(4, 5, -1), Vector3d.Zero);
        RigidBodyState result = Integrate(mass, initial, 0.1, 100, NoLoads);
        TestMath.Near(new(43, 48, -3), result.Pose.Position, 2e-12);
        Assert.AreEqual(initial.LinearVelocity, result.LinearVelocity);
        Assert.AreEqual(initial.Pose.Orientation, result.Pose.Orientation);
    }

    [TestMethod]
    public void ConstantForceMatchesAnalyticalParabolicMotion()
    {
        RigidBodyMassProperties mass = new(2, Matrix3x3d.Identity);
        RigidBodyState initial = new(new(new(2, -3, 4), Quaterniond.Identity), new(1, 2, 3), Vector3d.Zero);
        RigidBodyState result = Integrate(mass, initial, 0.01, 200,
            (_, _) => new(new(2, -4, -19.6), Vector3d.Zero));
        TestMath.Near(new(6, -3, -9.6), result.Pose.Position, 1e-12);
        TestMath.Near(new(3, -2, -16.6), result.LinearVelocity, 1e-12);
    }

    [TestMethod]
    public void TimeVaryingForceUsesIntermediateTimes()
    {
        RigidBodyMassProperties mass = new(3, Matrix3x3d.Identity);
        RigidBodyState initial = new(RigidPose3d.Identity, Vector3d.Zero, Vector3d.Zero);
        // F=6*t^2 N, m=3 kg => a=2*t^2, v=2*t^3/3, x=t^4/6.
        RigidBodyState result = Integrate(mass, initial, 0.1, 10,
            (time, _) => new(new(6 * time * time, 0, 0), Vector3d.Zero));
        TestMath.Near(new(1.0 / 6, 0, 0), result.Pose.Position, 2e-15);
        TestMath.Near(new(2.0 / 3, 0, 0), result.LinearVelocity, 2e-15);
    }

    [TestMethod]
    public void StateDependentForceMatchesHarmonicOscillator()
    {
        RigidBodyMassProperties mass = new(2, Matrix3x3d.Identity);
        RigidBodyState initial = new(new(Vector3d.UnitX, Quaterniond.Identity), Vector3d.Zero, Vector3d.Zero);
        // F=-2*x with m=2 gives x''=-x; x(0)=1 and v(0)=0.
        RigidBodyState result = Integrate(mass, initial, 0.02, 100,
            (_, state) => new(-2 * state.Pose.Position, Vector3d.Zero));
        TestMath.Near(new(System.Math.Cos(2), 0, 0), result.Pose.Position, 4e-9);
        TestMath.Near(new(-System.Math.Sin(2), 0, 0), result.LinearVelocity, 4e-9);
    }

    [TestMethod]
    public void LoadEvaluatorReceivesFourPhysicalStageStates()
    {
        RigidBodyMassProperties mass = new(1, Matrix3x3d.Identity);
        RigidBodyState initial = new(new(Vector3d.UnitX, Quaterniond.Identity), Vector3d.Zero, new(0, 0, 2));
        List<double> times = new();
        List<RigidBodyState> states = new();
        RigidBodyIntegrator.Step(mass, initial, 2, 0.1, (time, state) =>
        {
            times.Add(time);
            states.Add(state);
            Quaterniond q = state.Pose.Orientation;
            Assert.AreEqual(1, q.W * q.W + q.X * q.X + q.Y * q.Y + q.Z * q.Z, 5e-16);
            return new(-state.Pose.Position, Vector3d.Zero);
        });

        CollectionAssert.AreEqual(new[] { 2.0, 2.05, 2.05, 2.1 }, times);
        Assert.AreEqual(initial, states[0]);
        Assert.AreNotEqual(states[1], states[2], "The two midpoint evaluations must use distinct stage states.");
        Assert.AreNotEqual(initial.Pose.Position, states[3].Pose.Position);
    }

    [TestMethod]
    public void ConstantTorqueMatchesAngularAccelerationAndAngle()
    {
        RigidBodyMassProperties mass = new(1, Matrix3x3d.Diagonal(2, 2, 2));
        RigidBodyState initial = new(RigidPose3d.Identity, Vector3d.Zero, new(0, 0, 0.3));
        RigidBodyState result = Integrate(mass, initial, 0.0025, 800,
            (_, _) => new(Vector3d.Zero, new(0, 0, 4)));
        // alpha=4/2=2 rad/s^2; at t=2: omega=4.3 rad/s, theta=0.3*2+0.5*2*4=4.6 rad.
        TestMath.Near(new(0, 0, 4.3), result.BodyAngularVelocity, 2e-13);
        TestMath.SameRotation(Quaterniond.FromAxisAngle(Vector3d.UnitZ, 4.6), result.Pose.Orientation, 2e-10);
        Assert.AreEqual(Vector3d.Zero, result.Pose.Position);
    }

    [TestMethod]
    public void TimeVaryingTorqueUsesIntermediateTimes()
    {
        RigidBodyMassProperties mass = new(1, Matrix3x3d.Diagonal(2, 2, 2));
        RigidBodyState initial = new(RigidPose3d.Identity, Vector3d.Zero, Vector3d.Zero);
        // tau_z=4*t, I_zz=2 => omega_z=t^2 and theta=t^3/3, from rest.
        RigidBodyState result = Integrate(mass, initial, 0.005, 400,
            (time, _) => new(Vector3d.Zero, new(0, 0, 4 * time)));
        TestMath.Near(new(0, 0, 4), result.BodyAngularVelocity, 3e-13);
        TestMath.SameRotation(Quaterniond.FromAxisAngle(Vector3d.UnitZ, 8.0 / 3), result.Pose.Orientation, 2e-9);
    }

    [TestMethod]
    public void StateDependentTorqueMatchesExponentialSpinDecay()
    {
        RigidBodyMassProperties mass = new(1, Matrix3x3d.Diagonal(2, 2, 2));
        RigidBodyState initial = new(RigidPose3d.Identity, Vector3d.Zero, Vector3d.UnitZ);
        // tau_world=-2*omega_world with I=2*identity => omega_z=exp(-t), theta=1-exp(-t).
        RigidBodyState result = Integrate(mass, initial, 0.02, 100,
            (_, state) => new(Vector3d.Zero, -2 * state.WorldAngularVelocity));
        double finalSpeed = System.Math.Exp(-2);
        TestMath.Near(new(0, 0, finalSpeed), result.BodyAngularVelocity, 4e-10);
        TestMath.SameRotation(Quaterniond.FromAxisAngle(Vector3d.UnitZ, 1 - finalSpeed), result.Pose.Orientation, 5e-10);
    }

    [TestMethod]
    public void TorqueFreePrincipalAxisSpinUsesBodyToWorldComposition()
    {
        RigidBodyMassProperties mass = new(1, Matrix3x3d.Diagonal(2, 3, 4));
        Quaterniond start = Quaterniond.FromAxisAngle(Vector3d.UnitX, 0.7);
        RigidBodyState initial = new(new(Vector3d.Zero, start), Vector3d.Zero, new(0, 0, 2));
        RigidBodyState result = Integrate(mass, initial, 0.01, 200, NoLoads);
        Assert.AreEqual(initial.BodyAngularVelocity, result.BodyAngularVelocity);
        TestMath.SameRotation(start * Quaterniond.FromAxisAngle(Vector3d.UnitZ, 4), result.Pose.Orientation, 2e-9);
    }

    [TestMethod]
    [DataRow(false)]
    [DataRow(true)]
    public void TorqueFreeAsymmetricTumblingApproximatelyConservesEnergyAndWorldMomentum(bool offDiagonalInertia)
    {
        Matrix3x3d inertia = offDiagonalInertia
            ? new(4, 1, 0.5, 1, 5, -0.25, 0.5, -0.25, 6)
            : Matrix3x3d.Diagonal(2, 3, 4);
        RigidBodyMassProperties mass = new(5, inertia);
        RigidBodyState state = TumblingState();
        Vector3d initialOmega = state.BodyAngularVelocity;
        double initialEnergy = RigidBodyDynamics.KineticEnergy(mass, state);
        Vector3d initialMomentum = RigidBodyDynamics.AngularMomentum(mass, state);
        double maximumEnergyError = 0;
        double maximumMomentumError = 0;
        double maximumNormError = 0;
        for (int index = 0; index < 5000; index++)
        {
            state = RigidBodyIntegrator.Step(mass, state, index * 0.002, 0.002, NoLoads);
            maximumEnergyError = System.Math.Max(maximumEnergyError,
                System.Math.Abs(RigidBodyDynamics.KineticEnergy(mass, state) - initialEnergy) / initialEnergy);
            maximumMomentumError = System.Math.Max(maximumMomentumError,
                (RigidBodyDynamics.AngularMomentum(mass, state) - initialMomentum).Length / initialMomentum.Length);
            Quaterniond q = state.Pose.Orientation;
            maximumNormError = System.Math.Max(maximumNormError,
                System.Math.Abs(q.W * q.W + q.X * q.X + q.Y * q.Y + q.Z * q.Z - 1));
        }

        TestContext.WriteLine($"10 s tumble, h=0.002, off-diagonal={offDiagonalInertia}: maximum relative energy error={maximumEnergyError:E6}, "
            + $"momentum error={maximumMomentumError:E6}, quaternion norm-squared error={maximumNormError:E6}");
        Assert.IsTrue(maximumEnergyError < 1e-9, $"Energy error {maximumEnergyError:E6}");
        Assert.IsTrue(maximumMomentumError < 1e-9, $"Momentum error {maximumMomentumError:E6}");
        Assert.IsTrue(maximumNormError < 1e-15);
        Assert.IsTrue((state.BodyAngularVelocity - initialOmega).Length > 0.1,
            "An asymmetric free body must be allowed to tumble.");
    }

    [TestMethod]
    public void AnalyticalSpinErrorConvergesAtFourthOrder()
    {
        RigidBodyMassProperties mass = new(1, Matrix3x3d.Diagonal(2, 3, 4));
        RigidBodyState initial = new(RigidPose3d.Identity, Vector3d.Zero, new(0, 0, 2));
        Vector3d expected = new(System.Math.Cos(4), System.Math.Sin(4), 0);
        double coarse = Error(0.4, 5);
        double medium = Error(0.2, 10);
        double fine = Error(0.1, 20);
        TestContext.WriteLine($"Analytical spin errors: {coarse:E6}, {medium:E6}, {fine:E6}; "
            + $"ratios: {coarse / medium:F6}, {medium / fine:F6}");
        Assert.IsTrue(coarse / medium > 12 && coarse / medium < 20);
        Assert.IsTrue(medium / fine > 12 && medium / fine < 20);

        double Error(double step, int count) =>
            (Integrate(mass, initial, step, count, NoLoads).Pose.TransformDirection(Vector3d.UnitX) - expected).Length;
    }

    [TestMethod]
    public void CoupledTumblingErrorConvergesAtFourthOrder()
    {
        RigidBodyMassProperties mass = new(5, Matrix3x3d.Diagonal(2, 3, 4));
        RigidBodyState initial = TumblingState();
        RigidBodyState reference = Integrate(mass, initial, 0.00125, 2048, NoLoads);
        double coarse = Error(0.08, 32);
        double medium = Error(0.04, 64);
        double fine = Error(0.02, 128);
        TestContext.WriteLine($"Coupled tumble errors: {coarse:E6}, {medium:E6}, {fine:E6}; "
            + $"ratios: {coarse / medium:F6}, {medium / fine:F6}");
        Assert.IsTrue(coarse / medium > 12 && coarse / medium < 20);
        Assert.IsTrue(medium / fine > 12 && medium / fine < 20);

        double Error(double step, int count)
        {
            RigidBodyState result = Integrate(mass, initial, step, count, NoLoads);
            return (result.BodyAngularVelocity - reference.BodyAngularVelocity).Length
                + (result.Pose.TransformDirection(Vector3d.UnitX) - reference.Pose.TransformDirection(Vector3d.UnitX)).Length;
        }
    }

    [TestMethod]
    public void SimulationRepeatsBitForBitWithSameRuntimeAndInputs()
    {
        RigidBodyMassProperties mass = new(5, Matrix3x3d.Diagonal(2, 3, 4));
        RigidBodyState first = Integrate(mass, TumblingState(), 0.005, 400, Loads);
        RigidBodyState second = Integrate(mass, TumblingState(), 0.005, 400, Loads);
        CollectionAssert.AreEqual(Bits(first), Bits(second));

        static RigidBodyLoads Loads(double time, RigidBodyState state) =>
            new(new Vector3d(System.Math.Sin(time), 0, 0) - 0.2 * state.LinearVelocity, new(0.1, -0.2, 0.3));

        static long[] Bits(RigidBodyState state)
        {
            double[] components = {
                state.Pose.Position.X, state.Pose.Position.Y, state.Pose.Position.Z,
                state.Pose.Orientation.W, state.Pose.Orientation.X, state.Pose.Orientation.Y, state.Pose.Orientation.Z,
                state.LinearVelocity.X, state.LinearVelocity.Y, state.LinearVelocity.Z,
                state.BodyAngularVelocity.X, state.BodyAngularVelocity.Y, state.BodyAngularVelocity.Z
            };
            return Array.ConvertAll(components, BitConverter.DoubleToInt64Bits);
        }
    }

    [TestMethod]
    public void RotatingWorldFrameRotatesTheResultConsistently()
    {
        RigidBodyMassProperties mass = new(5, Matrix3x3d.Diagonal(2, 3, 4));
        RigidBodyState initial = TumblingState();
        Quaterniond rotation = Quaterniond.FromAxisAngle(new(2, -1, 3), 0.9);
        RigidBodyState rotated = new(
            new(rotation.Rotate(initial.Pose.Position), rotation * initial.Pose.Orientation),
            rotation.Rotate(initial.LinearVelocity), initial.BodyAngularVelocity);
        RigidBodyLoads loads = new(new(1, -2, 0.5), new(-0.1, 0.2, 0.3));
        RigidBodyState result = Integrate(mass, initial, 0.01, 100, (_, _) => loads);
        RigidBodyState rotatedResult = Integrate(mass, rotated, 0.01, 100,
            (_, _) => new(rotation.Rotate(loads.Force), rotation.Rotate(loads.Torque)));
        TestMath.Near(rotation.Rotate(result.Pose.Position), rotatedResult.Pose.Position, 3e-14);
        TestMath.Near(rotation.Rotate(result.LinearVelocity), rotatedResult.LinearVelocity, 3e-14);
        TestMath.Near(result.BodyAngularVelocity, rotatedResult.BodyAngularVelocity, 2e-14);
        TestMath.SameRotation(rotation * result.Pose.Orientation, rotatedResult.Pose.Orientation, 2e-14);
    }

    private static RigidBodyState TumblingState() => new(
        new(new(1, 2, -3), Quaterniond.FromAxisAngle(new(1, -2, 3), 0.7)),
        new(0.5, -1, 2), new(0.7, 1.1, 1.9));

    private static RigidBodyLoads NoLoads(double time, RigidBodyState state) => default;

    private static RigidBodyState Integrate(RigidBodyMassProperties mass, RigidBodyState state,
        double step, int count, Func<double, RigidBodyState, RigidBodyLoads> evaluateLoads)
    {
        for (int index = 0; index < count; index++)
        {
            state = RigidBodyIntegrator.Step(mass, state, index * step, step, evaluateLoads);
        }

        return state;
    }
}
