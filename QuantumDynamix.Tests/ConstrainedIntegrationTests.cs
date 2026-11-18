using System;
using System.Collections.Generic;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using QuantumDynamix.Geometry;
using QuantumDynamix.Math;
using QuantumDynamix.Physics;
using static QuantumDynamix.Tests.ConstrainedDynamicsTests;

namespace QuantumDynamix.Tests;

[TestClass]
public sealed class ConstrainedIntegrationTests
{
    public TestContext TestContext { get; set; } = null!;

    [TestMethod]
    public void CenterJointAndDisconnectedBodyFollowAnalyticalConstantLoads()
    {
        MultibodySystem system = new(new[] { Mass(2), Mass(3), Mass(4) },
            new[] { new SphericalJoint(0, 1, default, default) });
        MultibodyState state = new(new[] { Rest(), Rest(), Rest(new(3, 0, 0)) });
        for (int i = 0; i < 20; i++)
        {
            state = ConstrainedMultibodyIntegrator.Step(system, state, i * 0.05, 0.05,
                (_, _) => new[] { new RigidBodyLoads(new(10, 0, 0), default), default, new(new(0, 8, 0), default) }).State;
        }

        TestMath.Near(Vector3d.UnitX, state.Bodies[0].Pose.Position, 2e-13);
        TestMath.Near(state.Bodies[0].Pose.Position, state.Bodies[1].Pose.Position, 1e-13);
        TestMath.Near(new(2, 0, 0), state.Bodies[1].LinearVelocity, 2e-13);
        TestMath.Near(new(3, 1, 0), state.Bodies[2].Pose.Position, 2e-13);
        TestMath.Near(new(0, 2, 0), state.Bodies[2].LinearVelocity, 2e-13);
    }

    [TestMethod]
    public void ReactionsAreInstantaneousAtAcceptedEndpoint()
    {
        MultibodySystem system = new(new[] { Mass(2), Mass(3) }, new[] { new SphericalJoint(0, 1, default, default) });
        MultibodyState initial = new(new[] { Rest(), Rest() });
        static IReadOnlyList<RigidBodyLoads> Loads(double t, MultibodyState s) =>
            new[] { new RigidBodyLoads(new(t * t, 0, 0), default), default };
        ConstrainedStepResult result = ConstrainedMultibodyIntegrator.Step(system, initial, 0, 0.2, Loads);
        TestMath.Near(new(-0.024, 0, 0), result.AcceptedDynamics.JointReactions[0].ForceOnA, 1e-16);
        ConstrainedDynamicsResult direct = ConstrainedMultibodyDynamics.Evaluate(system, result.State, Loads(0.2, result.State));
        Assert.AreEqual(direct.JointReactions[0], result.AcceptedDynamics.JointReactions[0]);
        Assert.AreEqual(Vector3d.Zero, result.ProjectionCorrections[0].PositionChange);
    }

    [TestMethod]
    public void ProjectionCorrectionsAreSeparateAndReactionsAreRecomputed()
    {
        (MultibodySystem system, MultibodyState state) = GeneralPair();
        ConstrainedStepResult result = ConstrainedMultibodyIntegrator.Step(system, state, 0, 0.1, NoLoads);
        Assert.IsTrue(result.Diagnostics.BeforeProjectionPositionError > ConstrainedSolverSettings.Default.PositionTolerance);
        Assert.IsTrue(result.Diagnostics.PositionProjectionIterations > 0);
        Assert.IsTrue(result.Diagnostics.VelocityProjectionIterations > 0);
        Assert.IsTrue(result.ProjectionCorrections[0].PositionChange.Length > 0);
        Assert.IsTrue(result.ProjectionCorrections[0].LinearVelocityChange.Length > 0);
        ConstrainedDynamicsResult direct = ConstrainedMultibodyDynamics.Evaluate(system, result.State, NoLoads(0.1, result.State));
        Assert.AreEqual(direct.JointReactions[0], result.AcceptedDynamics.JointReactions[0]);
        Assert.IsTrue(result.AcceptedDynamics.Diagnostics.MaximumPositionError <= 1e-10);
        Assert.IsTrue(result.AcceptedDynamics.Diagnostics.MaximumVelocityError <= 1e-10);
    }

    [TestMethod]
    public void QuaternionMidpointIsSecondOrderForAnalyticalPrincipalAxisSpin()
    {
        MultibodySystem system = new(new[] { Mass(2, new(2, 3, 4)) }, Array.Empty<SphericalJoint>());
        Quaterniond start = Quaterniond.FromAxisAngle(Vector3d.UnitX, 0.7);
        MultibodyState initial = new(new[] { new RigidBodyState(new(Vector3d.Zero, start), default, new(0, 0, 2)) });
        Quaterniond expected = start * Quaterniond.FromAxisAngle(Vector3d.UnitZ, 4);
        double coarse = Error(0.2, 10);
        double medium = Error(0.1, 20);
        double fine = Error(0.05, 40);
        TestContext.WriteLine($"Quaternion midpoint spin errors={coarse:E6}, {medium:E6}, {fine:E6}; ratios={coarse / medium:F6}, {medium / fine:F6}");
        Assert.IsTrue(coarse / medium > 3.9 && coarse / medium < 4.1);
        Assert.IsTrue(medium / fine > 3.9 && medium / fine < 4.1);

        double Error(double h, int count)
        {
            MultibodyState final = Integrate(system, initial, h, count);
            return (final.Bodies[0].Pose.Orientation.Rotate(Vector3d.UnitX) - expected.Rotate(Vector3d.UnitX)).Length;
        }
    }

    [TestMethod]
    public void CoupledRotatingPairConvergesAtSecondOrderAgainstAnalyticalMotion()
    {
        (MultibodySystem system, MultibodyState initial) = RotatingPair();
        double coarse = Error(0.1, 16);
        double medium = Error(0.05, 32);
        double fine = Error(0.025, 64);
        TestContext.WriteLine($"Coupled analytical errors={coarse:E6}, {medium:E6}, {fine:E6}; ratios={coarse / medium:F6}, {medium / fine:F6}");
        Assert.IsTrue(coarse / medium > 3.5 && coarse / medium < 4.5);
        Assert.IsTrue(medium / fine > 3.5 && medium / fine < 4.5);

        double Error(double h, int count)
        {
            MultibodyState final = Integrate(system, initial, h, count);
            Quaterniond expected = Quaterniond.FromAxisAngle(Vector3d.UnitZ, 1.6);
            double error = 0;
            for (int i = 0; i < 2; i++)
            {
                // This fixture uses 1 m, 1 m/s and 1 rad/s scales: sum dimensionless errors.
                error += (final.Bodies[i].Pose.Position - expected.Rotate(initial.Bodies[i].Pose.Position)).Length;
                error += (final.Bodies[i].LinearVelocity - expected.Rotate(initial.Bodies[i].LinearVelocity)).Length;
                error += (final.Bodies[i].Pose.Orientation.Rotate(Vector3d.UnitX) - expected.Rotate(Vector3d.UnitX)).Length;
                error += (final.Bodies[i].BodyAngularVelocity - initial.Bodies[i].BodyAngularVelocity).Length;
            }

            return error;
        }
    }

    [TestMethod]
    public void LongRotatingPairBoundsDriftAndMeasuresEnergyAndTotalMomentum()
    {
        (MultibodySystem system, MultibodyState state) = RotatingPair();
        double energy = Energy(system, state);
        Vector3d momentum = Momentum(system, state);
        Vector3d angularMomentum = AngularMomentum(system, state);
        double maxEnergy = 0;
        double maxMomentum = 0;
        double maxAngularMomentum = 0;
        double maxGap = 0;
        double maxSlip = 0;
        for (int i = 0; i < 1000; i++)
        {
            ConstrainedStepResult result = ConstrainedMultibodyIntegrator.Step(system, state, i * 0.01, 0.01, NoLoads);
            state = result.State;
            maxEnergy = System.Math.Max(maxEnergy, System.Math.Abs(Energy(system, state) - energy) / energy);
            maxMomentum = System.Math.Max(maxMomentum, (Momentum(system, state) - momentum).Length);
            maxAngularMomentum = System.Math.Max(maxAngularMomentum, (AngularMomentum(system, state) - angularMomentum).Length / angularMomentum.Length);
            maxGap = System.Math.Max(maxGap, result.AcceptedDynamics.Diagnostics.MaximumPositionError);
            maxSlip = System.Math.Max(maxSlip, result.AcceptedDynamics.Diagnostics.MaximumVelocityError);
            foreach (RigidBodyState body in state.Bodies)
            {
                Quaterniond q = body.Pose.Orientation;
                Assert.AreEqual(1, q.W * q.W + q.X * q.X + q.Y * q.Y + q.Z * q.Z, 1e-15);
            }
        }

        TestContext.WriteLine($"10 s pair h=0.01: max relative energy={maxEnergy:E6}, angular momentum={maxAngularMomentum:E6}, linear momentum={maxMomentum:E6} kg*m/s, gap={maxGap:E6} m, slip={maxSlip:E6} m/s");
        Assert.IsTrue(maxEnergy < 1e-6);
        Assert.IsTrue(maxMomentum < 1e-11);
        Assert.IsTrue(maxAngularMomentum < 5e-7);
        Assert.IsTrue(maxGap <= 1e-10);
        Assert.IsTrue(maxSlip <= 1e-10);
    }

    [TestMethod]
    [DataRow(1.0, 0.2)]
    [DataRow(1e3, 2.0)]
    [DataRow(1e6, 10.0)]
    public void MassRatioAndSpeedSensitivityMaintainConstraints(double ratio, double speed)
    {
        (MultibodySystem system, MultibodyState state) = RotatingPair(speed, ratio);
        double initialEnergy = Energy(system, state);
        Vector3d initialMomentum = Momentum(system, state);
        double step = 0.02 / speed;
        double maxEnergy = 0;
        double maxMomentum = 0;
        double maxMotionError = 0;
        double maxReactionError = 0;
        for (int i = 0; i < 100; i++)
        {
            ConstrainedStepResult result = ConstrainedMultibodyIntegrator.Step(system, state, i * step, step, NoLoads);
            state = result.State;
            Assert.IsTrue(result.AcceptedDynamics.Diagnostics.MaximumPositionError <= 1e-10);
            Assert.IsTrue(result.AcceptedDynamics.Diagnostics.MaximumVelocityError <= 1e-10);
            maxEnergy = System.Math.Max(maxEnergy, System.Math.Abs(Energy(system, state) / initialEnergy - 1));
            maxMomentum = System.Math.Max(maxMomentum, (Momentum(system, state) - initialMomentum).Length);
            Quaterniond expectedRotation = Quaterniond.FromAxisAngle(Vector3d.UnitZ, (i + 1) * step * speed);
            // Check the light body's analytical motion and reaction separately: the heavy body's
            // spin energy can dominate total energy and conceal errors at large mass ratios.
            maxMotionError = System.Math.Max(maxMotionError,
                (state.Bodies[0].Pose.Position - expectedRotation.Rotate(new(-1, 0, 0))).Length
                + (state.Bodies[0].LinearVelocity / speed - expectedRotation.Rotate(new(0, -1, 0))).Length);
            maxReactionError = System.Math.Max(maxReactionError,
                (result.AcceptedDynamics.JointReactions[0].ForceOnA / (2 * speed * speed)
                - expectedRotation.Rotate(Vector3d.UnitX)).Length);
        }

        TestContext.WriteLine($"ratio={ratio:E3}, speed={speed}: relative energy={maxEnergy:E6}, momentum={maxMomentum:E6} kg*m/s, motion error={maxMotionError:E6}, reaction error={maxReactionError:E6}");
        Assert.IsTrue(maxEnergy < 5e-6);
        Assert.IsTrue(maxMomentum < 1e-10);
        Assert.IsTrue(maxMotionError < 1e-3);
        Assert.IsTrue(maxReactionError < 1e-3);
    }

    [TestMethod]
    public void GeneralTumblingPairShowsConvergenceAndEnergyMomentumSensitivity()
    {
        (MultibodySystem system, MultibodyState initial) = GeneralPair();
        MultibodyState coarse = Integrate(system, initial, 0.04, 25);
        MultibodyState medium = Integrate(system, initial, 0.02, 50);
        MultibodyState fine = Integrate(system, initial, 0.01, 100);
        MultibodyState reference = Integrate(system, initial, 0.0025, 400);
        double coarseError = StateError(coarse, reference);
        double mediumError = StateError(medium, reference);
        double fineError = StateError(fine, reference);
        double initialEnergy = Energy(system, initial);
        double coarseEnergy = System.Math.Abs(Energy(system, coarse) / initialEnergy - 1);
        double fineEnergy = System.Math.Abs(Energy(system, fine) / initialEnergy - 1);
        double fineAngular = (AngularMomentum(system, fine) - AngularMomentum(system, initial)).Length;
        TestContext.WriteLine($"Tumble errors={coarseError:E6}, {mediumError:E6}, {fineError:E6}; relative energy coarse={coarseEnergy:E6}, fine={fineEnergy:E6}; angular momentum error={fineAngular:E6} kg*m^2/s");
        Assert.IsTrue(coarseError / mediumError > 3.5 && coarseError / mediumError < 4.5);
        Assert.IsTrue(mediumError / fineError > 3.5 && mediumError / fineError < 4.8);
        Assert.IsTrue(fineEnergy < coarseEnergy / 8);
        Assert.IsTrue(fineEnergy < 1e-6);
        Assert.IsTrue(fineAngular < 2e-5);
        TestMath.Near(Momentum(system, initial), Momentum(system, fine), 1e-11);
    }

    [TestMethod]
    public void NonlinearWholeSystemLoadsReceiveNormalizedSnapshotsAndRepeatDeterministically()
    {
        (MultibodySystem system, MultibodyState initial) = GeneralPair();
        int calls = 0;
        IReadOnlyList<RigidBodyLoads> Loads(double time, MultibodyState snapshot)
        {
            calls++;
            Assert.AreEqual(2, snapshot.Bodies.Count);
            foreach (RigidBodyState body in snapshot.Bodies)
            {
                Quaterniond q = body.Pose.Orientation;
                Assert.AreEqual(1, q.W * q.W + q.X * q.X + q.Y * q.Y + q.Z * q.Z, 1e-15);
            }

            Vector3d force = (snapshot.Bodies[1].LinearVelocity - snapshot.Bodies[0].LinearVelocity) * 0.2;
            return new[] { new RigidBodyLoads(force + new Vector3d(time, 0, 0), default), new(-force, default) };
        }

        ConstrainedStepResult first = ConstrainedMultibodyIntegrator.Step(system, initial, 0, 0.05, Loads);
        ConstrainedStepResult second = ConstrainedMultibodyIntegrator.Step(system, initial, 0, 0.05, Loads);
        CollectionAssert.AreEqual(first.State.Bodies, second.State.Bodies);
        CollectionAssert.AreEqual(first.AcceptedDynamics.JointReactions, second.AcceptedDynamics.JointReactions);
        Assert.IsTrue(calls > 8);
        Assert.AreEqual(initial.Bodies[0], GeneralPair().Item2.Bodies[0]);
    }

    [TestMethod]
    public void NonlinearDampingConvergesAgainstAnalyticalSpinAndAngle()
    {
        MultibodySystem system = new(new[] { Mass(1) }, Array.Empty<SphericalJoint>());
        MultibodyState initial = new(new[] { new RigidBodyState(RigidPose3d.Identity, default, new(0, 0, 2)) });
        double coarse = Error(0.05, 20);
        double medium = Error(0.025, 40);
        double fine = Error(0.0125, 80);
        TestContext.WriteLine($"Nonlinear damping errors={coarse:E6}, {medium:E6}, {fine:E6}; ratios={coarse / medium:F6}, {medium / fine:F6}");
        Assert.IsTrue(coarse / medium > 3.7 && coarse / medium < 4.3);
        Assert.IsTrue(medium / fine > 3.7 && medium / fine < 4.3);

        double Error(double h, int count)
        {
            MultibodyState current = initial;
            for (int i = 0; i < count; i++)
            {
                current = ConstrainedMultibodyIntegrator.Step(system, current, i * h, h, (_, s) =>
                    new[] { new RigidBodyLoads(default, new(0, 0, -System.Math.Pow(s.Bodies[0].BodyAngularVelocity.Z, 3))) }).State;
            }

            // w'=-w^3, w(0)=2 => w(t)=2/sqrt(1+8t), theta(t)=(sqrt(1+8t)-1)/2.
            return System.Math.Abs(current.Bodies[0].BodyAngularVelocity.Z - 2.0 / 3)
                + (current.Bodies[0].Pose.Orientation.Rotate(Vector3d.UnitX)
                - Quaterniond.FromAxisAngle(Vector3d.UnitZ, 1).Rotate(Vector3d.UnitX)).Length;
        }
    }

    [TestMethod]
    public void IntegratedThreeBodyChainUsesAllCoupledRows()
    {
        MultibodySystem system = new(new[] { Mass(2), Mass(3), Mass(5) },
            new[] { new SphericalJoint(0, 1, default, default), new SphericalJoint(1, 2, default, default) });
        MultibodyState initial = new(new[] { Rest(), Rest(), Rest() });
        ConstrainedStepResult result = ConstrainedMultibodyIntegrator.Step(system, initial, 0, 0.2,
            (_, _) => new[] { new RigidBodyLoads(new(10, 0, 0), default), default, default });
        foreach (RigidBodyState body in result.State.Bodies)
        {
            TestMath.Near(new(0.02, 0, 0), body.Pose.Position, 1e-15);
            TestMath.Near(new(0.2, 0, 0), body.LinearVelocity, 1e-15);
        }
    }

    internal static MultibodyState Integrate(MultibodySystem system, MultibodyState state, double step, int count,
        ConstrainedSolverSettings? settings = null)
    {
        for (int i = 0; i < count; i++)
        {
            state = ConstrainedMultibodyIntegrator.Step(system, state, i * step, step, NoLoads, settings).State;
        }

        return state;
    }

    private static double Energy(MultibodySystem system, MultibodyState state)
    {
        double total = 0;
        for (int i = 0; i < state.Bodies.Count; i++)
        {
            total += RigidBodyDynamics.KineticEnergy(system.Bodies[i], state.Bodies[i]);
        }

        return total;
    }

    private static Vector3d Momentum(MultibodySystem system, MultibodyState state)
    {
        Vector3d total = Vector3d.Zero;
        for (int i = 0; i < state.Bodies.Count; i++)
        {
            total += state.Bodies[i].LinearVelocity * system.Bodies[i].Mass;
        }

        return total;
    }

    private static Vector3d AngularMomentum(MultibodySystem system, MultibodyState state)
    {
        Vector3d total = Vector3d.Zero;
        for (int i = 0; i < state.Bodies.Count; i++)
        {
            total += RigidBodyDynamics.AngularMomentum(system.Bodies[i], state.Bodies[i])
                + Vector3d.Cross(state.Bodies[i].Pose.Position, state.Bodies[i].LinearVelocity * system.Bodies[i].Mass);
        }

        return total;
    }

    private static double StateError(MultibodyState actual, MultibodyState expected)
    {
        double total = 0;
        for (int i = 0; i < actual.Bodies.Count; i++)
        {
            total += (actual.Bodies[i].Pose.Position - expected.Bodies[i].Pose.Position).Length;
            total += (actual.Bodies[i].LinearVelocity - expected.Bodies[i].LinearVelocity).Length;
            total += (actual.Bodies[i].BodyAngularVelocity - expected.Bodies[i].BodyAngularVelocity).Length;
            total += (actual.Bodies[i].Pose.Orientation.Rotate(Vector3d.UnitX)
                - expected.Bodies[i].Pose.Orientation.Rotate(Vector3d.UnitX)).Length;
        }

        return total;
    }
}
