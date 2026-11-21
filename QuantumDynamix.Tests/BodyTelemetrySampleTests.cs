using System;
using System.Collections.Generic;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using QuantumDynamix.Math;
using QuantumDynamix.Physics;
using QuantumDynamix.Telemetry;
using static QuantumDynamix.Tests.ConstrainedDynamicsTests;

namespace QuantumDynamix.Tests;

[TestClass]
public sealed class BodyTelemetrySampleTests
{
    private const double AnalyticalTolerance = 1e-11;

    [TestMethod]
    public void CollectsOneSamplePerAcceptedStepWithInstantaneousEndpointAcceleration()
    {
        var system = new MultibodySystem(new[] { Mass(2), Mass(3) },
            new[] { new SphericalJoint(0, 1, default, default) });
        var state = new MultibodyState(new[] { Rest(), Rest() });
        var samples = new List<BodyTelemetrySample>();
        const int stepCount = 10;
        const double stepSize = 0.1;

        for (int i = 0; i < stepCount; i++)
        {
            double startTime = (double)i / stepCount;
            ConstrainedStepResult step = ConstrainedMultibodyIntegrator.Step(system, state, startTime, stepSize,
                static (time, _) => new[] { new RigidBodyLoads(new(10 * time, 0, 0), default), default });
            samples.Add(BodyTelemetrySample.FromAcceptedStep(step, 0, startTime + stepSize));
            state = step.State;
        }

        Assert.HasCount(stepCount, samples);
        for (int i = 0; i < stepCount; i++)
        {
            Assert.AreEqual((double)i / stepCount + stepSize, samples[i].AcceptedTimeSeconds);
        }

        // The joined masses accelerate at 2t m/s². The final midpoint gives 1.9 m/s².
        BodyTelemetrySample last = samples[^1];
        Assert.AreEqual(1.0, last.AcceptedTimeSeconds);
        Assert.AreEqual(1.0, last.Speed, AnalyticalTolerance);
        TestMath.Near(new(2, 0, 0), last.WorldLinearAcceleration, AnalyticalTolerance);
    }

    [TestMethod]
    public void SelectsBodyAndReportsWorldCoordinatesAtNegativeEndpointTime()
    {
        var system = new MultibodySystem(new[] { Mass(1), Mass(2) }, Array.Empty<SphericalJoint>());
        var moving = new RigidBodyState(
            new(new(10, 20, 30), Quaterniond.FromAxisAngle(Vector3d.UnitZ, 1)),
            new(3, 2, 0), default);
        var state = new MultibodyState(new[] { Rest(), moving });
        ConstrainedStepResult step = ConstrainedMultibodyIntegrator.Step(system, state, -2, 1,
            static (_, _) => new[] { default, new RigidBodyLoads(new(0, 4, 0), default) });

        BodyTelemetrySample stationary = BodyTelemetrySample.FromAcceptedStep(step, 0, -1);
        BodyTelemetrySample sample = BodyTelemetrySample.FromAcceptedStep(step, 1, -1);

        Assert.AreEqual(0, stationary.BodyIndex);
        Assert.AreEqual(Vector3d.Zero, stationary.WorldCenterOfMassPosition);
        Assert.AreEqual(0.0, stationary.Speed);
        Assert.AreEqual(Vector3d.Zero, stationary.WorldLinearAcceleration);
        Assert.AreEqual(1, sample.BodyIndex);
        Assert.AreEqual(-1.0, sample.AcceptedTimeSeconds);
        TestMath.Near(new(13, 23, 30), sample.WorldCenterOfMassPosition, AnalyticalTolerance);
        Assert.AreEqual(5.0, sample.Speed, AnalyticalTolerance);
        TestMath.Near(new(0, 2, 0), sample.WorldLinearAcceleration, AnalyticalTolerance);
    }

    [TestMethod]
    public void SamplesAcceptedStateAndDynamicsAfterProjection()
    {
        (MultibodySystem system, MultibodyState state) = GeneralPair();
        ConstrainedStepResult step = ConstrainedMultibodyIntegrator.Step(system, state, 0, 0.1, NoLoads);
        Assert.AreNotEqual(Vector3d.Zero, step.ProjectionCorrections[0].PositionChange);
        Assert.AreNotEqual(Vector3d.Zero, step.ProjectionCorrections[0].LinearVelocityChange);

        BodyTelemetrySample sample = BodyTelemetrySample.FromAcceptedStep(step, 0, 0.1);

        Assert.AreEqual(step.State.Bodies[0].Pose.Position, sample.WorldCenterOfMassPosition);
        Assert.AreEqual(step.State.Bodies[0].LinearVelocity.Length, sample.Speed);
        Assert.AreEqual(step.AcceptedDynamics.Derivatives[0].LinearAcceleration, sample.WorldLinearAcceleration);
    }

    [TestMethod]
    public void RejectsNullStepOutOfRangeBodyIndicesAndNonfiniteTimestamps()
    {
        var system = new MultibodySystem(new[] { Mass(1) }, Array.Empty<SphericalJoint>());
        var state = new MultibodyState(new[] { Rest() });
        ConstrainedStepResult step = ConstrainedMultibodyIntegrator.Step(system, state, 0, 0.1, NoLoads);

        ArgumentNullException nullStep = Assert.ThrowsExactly<ArgumentNullException>(
            () => BodyTelemetrySample.FromAcceptedStep(null!, 0, 0.1));
        Assert.AreEqual("step", nullStep.ParamName);

        foreach (int index in new[] { -1, step.State.Bodies.Count })
        {
            ArgumentOutOfRangeException invalidIndex = Assert.ThrowsExactly<ArgumentOutOfRangeException>(
                () => BodyTelemetrySample.FromAcceptedStep(step, index, 0.1));
            Assert.AreEqual("bodyIndex", invalidIndex.ParamName);
        }

        foreach (double time in new[] { double.NaN, double.PositiveInfinity, double.NegativeInfinity })
        {
            ArgumentOutOfRangeException invalidTime = Assert.ThrowsExactly<ArgumentOutOfRangeException>(
                () => BodyTelemetrySample.FromAcceptedStep(step, 0, time));
            Assert.AreEqual("acceptedTimeSeconds", invalidTime.ParamName);
        }
    }
}
