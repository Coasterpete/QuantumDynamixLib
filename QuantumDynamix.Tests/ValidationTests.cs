using System;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using QuantumDynamix.Geometry;
using QuantumDynamix.Math;
using QuantumDynamix.Physics;

namespace QuantumDynamix.Tests;

[TestClass]
public sealed class ValidationTests
{
    [TestMethod]
    [DataRow(0.0)]
    [DataRow(-1.0)]
    [DataRow(double.NaN)]
    [DataRow(double.PositiveInfinity)]
    [DataRow(double.NegativeInfinity)]
    public void MassMustBeFiniteAndPositive(double mass)
    {
        Assert.ThrowsExactly<ArgumentOutOfRangeException>(() => new RigidBodyMassProperties(mass, Matrix3x3d.Identity));
    }

    [TestMethod]
    public void InertiaMustBeSymmetricAndNumericallyPositiveDefinite()
    {
        Matrix3x3d[] invalid = {
            default,
            Matrix3x3d.Diagonal(1, 1, 0),
            Matrix3x3d.Diagonal(-1, 1, 1),
            new(1, 0.1, 0, 0.2, 1, 0, 0, 0, 1),
            new(1, 2, 0, 2, 1, 0, 0, 0, 1),
            new(1, 1, 1, 1, 1, 1, 1, 1, 1)
        };
        foreach (Matrix3x3d inertia in invalid)
        {
            Assert.Throws<ArgumentException>(() => new RigidBodyMassProperties(1, inertia));
        }
    }

    [TestMethod]
    [DataRow(double.NaN)]
    [DataRow(double.PositiveInfinity)]
    [DataRow(double.NegativeInfinity)]
    public void NumericalTypesRejectNonfiniteComponentsInEveryPosition(double invalid)
    {
        Assert.ThrowsExactly<ArgumentOutOfRangeException>(() => new Vector3d(invalid, 0, 0));
        Assert.ThrowsExactly<ArgumentOutOfRangeException>(() => new Vector3d(0, invalid, 0));
        Assert.ThrowsExactly<ArgumentOutOfRangeException>(() => new Vector3d(0, 0, invalid));
        Assert.ThrowsExactly<ArgumentOutOfRangeException>(() => new Quaterniond(invalid, 0, 0, 0));
        Assert.ThrowsExactly<ArgumentOutOfRangeException>(() => new Quaterniond(0, invalid, 0, 0));
        Assert.ThrowsExactly<ArgumentOutOfRangeException>(() => new Quaterniond(0, 0, invalid, 0));
        Assert.ThrowsExactly<ArgumentOutOfRangeException>(() => new Quaterniond(0, 0, 0, invalid));
        for (int index = 0; index < 9; index++)
        {
            double[] entries = new double[9];
            entries[index] = invalid;
            Assert.ThrowsExactly<ArgumentOutOfRangeException>(() => new Matrix3x3d(
                entries[0], entries[1], entries[2], entries[3], entries[4], entries[5], entries[6], entries[7], entries[8]));
        }
    }

    [TestMethod]
    public void ZeroDirectionsOrientationsAndDefaultPhysicalStatesAreRejected()
    {
        Assert.ThrowsExactly<InvalidOperationException>(() => Vector3d.Zero.Normalized());
        Assert.ThrowsExactly<InvalidOperationException>(() => default(Quaterniond).Normalized());
        Assert.ThrowsExactly<InvalidOperationException>(() => new RigidPose3d(Vector3d.Zero, default));
        Assert.ThrowsExactly<ArgumentException>(() => default(RigidPose3d).TransformPoint(Vector3d.UnitX));
        Assert.ThrowsExactly<ArgumentException>(() => new RigidBodyState(default, Vector3d.Zero, Vector3d.Zero));
        RigidBodyMassProperties mass = new(1, Matrix3x3d.Identity);
        Assert.ThrowsExactly<ArgumentException>(() => RigidBodyDynamics.EvaluateDerivative(mass, default, default));
        Assert.ThrowsExactly<ArgumentException>(() => RigidBodyIntegrator.Step(mass, default, 0, 0.1, (_, _) => default));
    }

    [TestMethod]
    public void ArithmeticOverflowAndInvalidScalarsFailExplicitly()
    {
        Assert.ThrowsExactly<ArgumentOutOfRangeException>(() => Vector3d.UnitX * double.NaN);
        Assert.ThrowsExactly<ArgumentOutOfRangeException>(() => Quaterniond.Identity * double.PositiveInfinity);
        Assert.ThrowsExactly<DivideByZeroException>(() => Vector3d.UnitX / 0);
        Assert.ThrowsExactly<DivideByZeroException>(() => Quaterniond.Identity / 0);
        Assert.ThrowsExactly<ArgumentOutOfRangeException>(() => new Vector3d(double.MaxValue, 0, 0) * 2);
        Assert.ThrowsExactly<ArithmeticException>(() => Vector3d.Dot(new(double.MaxValue, 0, 0), new(2, 0, 0)));
        Assert.ThrowsExactly<ArgumentOutOfRangeException>(() => Quaterniond.FromAxisAngle(Vector3d.UnitX, double.NaN));
    }

    [TestMethod]
    [DataRow(0.0)]
    [DataRow(-0.1)]
    [DataRow(double.NaN)]
    [DataRow(double.PositiveInfinity)]
    [DataRow(double.Epsilon)]
    public void StepMustBeFinitePositiveAndResolveIntermediateTimes(double step)
    {
        RigidBodyMassProperties mass = new(1, Matrix3x3d.Identity);
        RigidBodyState state = new(RigidPose3d.Identity, Vector3d.Zero, Vector3d.Zero);
        Assert.ThrowsExactly<ArgumentOutOfRangeException>(() =>
            RigidBodyIntegrator.Step(mass, state, 0, step, (_, _) => default));
    }

    [TestMethod]
    public void TimeOverflowPrecisionLossAndNullDependenciesAreRejectedBeforeLoads()
    {
        RigidBodyMassProperties mass = new(1, Matrix3x3d.Identity);
        RigidBodyState state = new(RigidPose3d.Identity, Vector3d.Zero, Vector3d.Zero);
        int calls = 0;
        RigidBodyLoads CountLoads(double time, RigidBodyState stage)
        {
            calls++;
            return default;
        }

        Assert.ThrowsExactly<ArgumentOutOfRangeException>(() => RigidBodyIntegrator.Step(mass, state, double.NaN, 0.1, CountLoads));
        Assert.ThrowsExactly<ArgumentOutOfRangeException>(() => RigidBodyIntegrator.Step(mass, state, double.MaxValue, double.MaxValue, CountLoads));
        Assert.ThrowsExactly<ArgumentOutOfRangeException>(() => RigidBodyIntegrator.Step(mass, state, 1e20, 0.1, CountLoads));
        Assert.ThrowsExactly<ArgumentNullException>(() => RigidBodyIntegrator.Step(mass, state, 0, 0.1, null!));
        Assert.ThrowsExactly<ArgumentNullException>(() => RigidBodyDynamics.EvaluateDerivative(null!, state, default));
        Assert.ThrowsExactly<ArgumentNullException>(() => RigidBodyIntegrator.Step(null!, state, 0, 0.1, CountLoads));
        Assert.AreEqual(0, calls);
    }

    [TestMethod]
    public void LoadEvaluationCannotIntroduceNonfiniteInputsOrUnrepresentableAcceleration()
    {
        RigidBodyMassProperties mass = new(1, Matrix3x3d.Identity);
        RigidBodyState state = new(RigidPose3d.Identity, Vector3d.Zero, Vector3d.Zero);
        Assert.ThrowsExactly<ArgumentOutOfRangeException>(() => RigidBodyIntegrator.Step(mass, state, 0, 0.1,
            (_, _) => new(new(double.NaN, 0, 0), Vector3d.Zero)));
        RigidBodyMassProperties tinyMass = new(double.Epsilon, Matrix3x3d.Identity);
        Assert.ThrowsExactly<ArgumentOutOfRangeException>(() => RigidBodyDynamics.EvaluateDerivative(
            tinyMass, state, new(Vector3d.UnitX, Vector3d.Zero)));
    }
}
