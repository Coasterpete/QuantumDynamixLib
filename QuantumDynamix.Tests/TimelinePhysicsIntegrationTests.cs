using System;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using QuantumDynamix.Geometry;
using QuantumDynamix.Math;
using QuantumDynamix.Math.Functions;
using QuantumDynamix.Math.Timelines;
using QuantumDynamix.Physics;

namespace QuantumDynamix.Tests;

[TestClass]
public sealed class TimelinePhysicsIntegrationTests
{
    [TestMethod]
    public void TimeProfileLoadsIntegrateThroughExistingRk4WithoutOverwritingMotion()
    {
        // Analytical fixture, not a launch actuator: F(t)=6t^2 N, m=2 kg.
        var profile = new ScalarTimeline(ProfileDomain.Time,
            new[] { FunctionSegment.Transition(0, 1, 0, 6, new EasingFunction(EasingKind.Quadratic)) });
        var mass = new RigidBodyMassProperties(2, Matrix3x3d.Diagonal(1, 1, 1));
        var state = new RigidBodyState(RigidPose3d.Identity, default, default);
        for (int i = 0; i < 10; i++)
        {
            state = RigidBodyIntegrator.Step(mass, state, i / 10.0, 0.1,
                (t, _) => new(new(profile.EvaluateAtTime(t, 0).Value, 0, 0), default));
        }
        TestMath.Near(new(0.25, 0, 0), state.Pose.Position, 3e-14);
        TestMath.Near(Vector3d.UnitX, state.LinearVelocity, 3e-14);
    }

    [TestMethod]
    public void TimeProfileLoadsRespectExistingMultibodyReactionsAndSecondOrderIntegration()
    {
        // Analytical fixture: total F(t)=10t N on two center-joined masses totaling 5 kg.
        var profile = ScalarTimeline.FromKeyframes(ProfileDomain.Time, new[] { new TimelineKeyframe(0, 0), new TimelineKeyframe(1, 10) });
        var system = new MultibodySystem(new[] {
            new RigidBodyMassProperties(2, Matrix3x3d.Diagonal(1, 1, 1)),
            new RigidBodyMassProperties(3, Matrix3x3d.Diagonal(1, 1, 1)) },
            new[] { new SphericalJoint(0, 1, default, default) });
        ConstrainedStepResult coarse = Run(10);
        ConstrainedStepResult fine = Run(20);
        double coarseError = System.Math.Abs(coarse.State.Bodies[0].Pose.Position.X - 1.0 / 3);
        double fineError = System.Math.Abs(fine.State.Bodies[0].Pose.Position.X - 1.0 / 3);
        Assert.AreEqual(4, coarseError / fineError, 2e-8);
        TestMath.Near(Vector3d.UnitX, fine.State.Bodies[0].LinearVelocity, 1e-11);
        TestMath.Near(new(-6, 0, 0), fine.AcceptedDynamics.JointReactions[0].ForceOnA, 1e-10);
        TestMath.Near(fine.State.Bodies[0].Pose.Position, fine.State.Bodies[1].Pose.Position, 1e-10);

        ConstrainedStepResult Run(int steps)
        {
            var rest = new RigidBodyState(RigidPose3d.Identity, default, default);
            var state = new MultibodyState(new[] { rest, rest });
            ConstrainedStepResult result = null!;
            for (int i = 0; i < steps; i++)
            {
                result = ConstrainedMultibodyIntegrator.Step(system, state, (double)i / steps, 1.0 / steps,
                    (t, _) => new[] { new RigidBodyLoads(new(profile.EvaluateAtTime(t, 0).Value, 0, 0), default), default });
                state = result.State;
            }
            return result;
        }
    }

    [TestMethod]
    public void StationDerivativeConversionIncludesSpeedChangeAndSignedReversal()
    {
        // q=s^3, s(t)=2+t+t^2 at t=0: q(t)=8+12t+18t^2+13t^3+...
        ScalarJet spatial = new(8, 12, 12, 6);
        ScalarJet temporal = ProfileKinematics.ToTimeDerivatives(spatial, 1, 2, 0);
        Assert.AreEqual(12, temporal.FirstDerivative);
        Assert.AreEqual(36, temporal.SecondDerivative);
        Assert.AreEqual(78, temporal.ThirdDerivative);
        ScalarJet reversed = ProfileKinematics.ToTimeDerivatives(spatial, -1);
        Assert.AreEqual(-12, reversed.FirstDerivative);
        Assert.AreEqual(12, reversed.SecondDerivative);
        Assert.AreEqual(-6, reversed.ThirdDerivative);
        Assert.AreEqual(0, ProfileKinematics.ToTimeDerivatives(new(5, derivativeOrder: 0), 1).DerivativeOrder);
    }
}
