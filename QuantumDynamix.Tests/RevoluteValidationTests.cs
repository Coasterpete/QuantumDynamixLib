using System;
using System.Collections.Generic;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using QuantumDynamix.Geometry;
using QuantumDynamix.Math;
using QuantumDynamix.Physics;
using static QuantumDynamix.Tests.RevoluteTestFixtures;

namespace QuantumDynamix.Tests;

[TestClass]
public sealed class RevoluteValidationTests
{
    [TestMethod]
    public void InvalidFramesBodyReferencesAndNullDefinitionsAreRejected()
    {
        var (pair, _) = Centered();
        Assert.ThrowsExactly<ArgumentNullException>(() => new MultibodySystem(pair.Bodies, pair.Joints, null!));
        foreach (RevoluteJoint joint in new[] {
            new RevoluteJoint(-1, 1, RigidPose3d.Identity, RigidPose3d.Identity),
            new(0, 2, RigidPose3d.Identity, RigidPose3d.Identity), new(0, 0, RigidPose3d.Identity, RigidPose3d.Identity),
            new(0, 1, default, RigidPose3d.Identity), new(0, 1, RigidPose3d.Identity, default) })
        {
            Assert.ThrowsExactly<ArgumentException>(() => new MultibodySystem(pair.Bodies, pair.Joints, new[] { joint }));
        }
    }

    [TestMethod]
    public void DefinitionsAndReactionSnapshotsAreCopiedAndReadOnly()
    {
        var (pair, state) = Centered();
        RevoluteJoint[] definitions = { pair.RevoluteJoints[0] };
        var system = new MultibodySystem(pair.Bodies, pair.Joints, definitions);
        definitions[0] = default;
        Assert.AreEqual(1, system.RevoluteJoints[0].BodyB);
        Assert.ThrowsExactly<NotSupportedException>(() => ((IList<RevoluteJoint>)system.RevoluteJoints).Clear());
        var result = ConstrainedMultibodyDynamics.Evaluate(system, state, NoLoads(0, state));
        Assert.ThrowsExactly<NotSupportedException>(() => ((IList<RevoluteJointReaction>)result.RevoluteJointReactions).Clear());
        var legacy = new MultibodySystem(pair.Bodies, pair.Joints);
        Assert.AreEqual(0, legacy.RevoluteJoints.Count);
        Assert.AreEqual(0, legacy.BilateralConstraintCount);
        // Existing public positional constructors and deconstruction remain source compatible.
        var (gap, slip, acceleration, linear, pivot) = new ConstraintDiagnostics(1, 2, 3, 4, 5);
        Assert.AreEqual(15, gap + slip + acceleration + linear + pivot);
    }

    [TestMethod]
    [DataRow(0.0)]
    [DataRow(-1.0)]
    [DataRow(double.NaN)]
    [DataRow(double.PositiveInfinity)]
    public void InvalidAngularControlsAreRejected(double value)
    {
        Assert.ThrowsExactly<ArgumentOutOfRangeException>(() => new AngularConstraintSettings(orientationTolerance: value));
        Assert.ThrowsExactly<ArgumentOutOfRangeException>(() => new AngularConstraintSettings(angularVelocityTolerance: value));
    }

    [TestMethod]
    public void AngularSettingsAreSeparateAndValidateTheirChartRange()
    {
        Assert.ThrowsExactly<ArgumentOutOfRangeException>(() => new AngularConstraintSettings(System.Math.PI / 2));
        Assert.ThrowsExactly<ArgumentNullException>(() => new ConstrainedSolverSettings(null!));
        var angular = new AngularConstraintSettings(1e-4, 2e-4);
        var settings = new ConstrainedSolverSettings(angular, positionTolerance: 3e-7, velocityTolerance: 4e-7);
        Assert.AreSame(angular, settings.AngularConstraints);
        Assert.AreEqual(3e-7, settings.PositionTolerance);
        var (system, _) = Centered();
        var rest = new RigidBodyState(RigidPose3d.Identity, default, default);
        var state = new MultibodyState(new[] { rest, new RigidBodyState(new(Vector3d.Zero,
            Quaterniond.FromAxisAngle(Vector3d.UnitZ, 5e-5)), default, new(0, 0, 5e-5)) });
        var evaluated = ConstrainedMultibodyDynamics.Evaluate(system, state, NoLoads(0, state), settings);
        Assert.AreEqual(0, evaluated.Diagnostics.MaximumPositionError);
        Assert.AreEqual(0, evaluated.Diagnostics.MaximumVelocityError);
        Assert.AreEqual(5e-5, evaluated.Diagnostics.MaximumAngularPositionError, 1e-15);
        ConstrainedMultibodyIntegrator.Step(system, state, 0, 0.001, NoLoads, settings);
        Assert.ThrowsExactly<ConstrainedSolveException>(() => ConstrainedMultibodyIntegrator.Step(system, state, 0, 0.001, NoLoads));
    }

    [TestMethod]
    [DataRow(0)]
    [DataRow(1)]
    [DataRow(2)]
    [DataRow(3)]
    public void InconsistentInitialGapTiltLinearOrAngularVelocityFailsBeforeLoads(int kind)
    {
        var (system, initial) = Centered();
        var b = new RigidBodyState(new(kind == 0 ? new Vector3d(0.001, 0, 0) : Vector3d.Zero,
            kind == 1 ? Quaterniond.FromAxisAngle(Vector3d.UnitY, 0.001) : Quaterniond.Identity),
            kind == 2 ? new Vector3d(0, 0.001, 0) : Vector3d.Zero, kind == 3 ? new Vector3d(0, 0, 0.001) : Vector3d.Zero);
        var state = new MultibodyState(new[] { initial.Bodies[0], b });
        int calls = 0;
        var error = Assert.ThrowsExactly<ConstrainedSolveException>(() => ConstrainedMultibodyIntegrator.Step(system, state,
            0, 0.01, (_, _) => { calls++; return new RigidBodyLoads[2]; }));
        Assert.AreEqual(ConstrainedSolveFailure.InconsistentInitialState, error.Failure);
        Assert.AreEqual(0, calls);
        Assert.AreEqual(b, state.Bodies[1]);
    }

    [TestMethod]
    public void AntiparallelAndSingularChartsFailWhileArbitraryAxlePhaseIsAllowed()
    {
        var (system, initial) = Centered();
        foreach (double angle in new[] { System.Math.PI, System.Math.PI / 2 })
        {
            var state = new MultibodyState(new[] { initial.Bodies[0], new RigidBodyState(new(Vector3d.Zero,
                Quaterniond.FromAxisAngle(Vector3d.UnitY, angle)), default, default) });
            var error = Assert.ThrowsExactly<ConstrainedSolveException>(() => ConstrainedMultibodyDynamics.Evaluate(system, state, NoLoads(0, state)));
            Assert.IsTrue(error.Failure is ConstrainedSolveFailure.UnsupportedJointConfiguration or ConstrainedSolveFailure.SingularConstraints);
            Assert.ThrowsExactly<ConstrainedSolveException>(() => RevoluteJointKinematics.Evaluate(system, state, 0));
        }
        foreach (double angle in new[] { -20.0, -System.Math.PI, 0, System.Math.PI, 37.0 })
        {
            var (_, state) = Centered(phase: angle, spin: 123);
            var sample = RevoluteJointKinematics.Evaluate(system, state, 0);
            Assert.IsTrue(sample.Angle > -System.Math.PI && sample.Angle <= System.Math.PI);
            Assert.AreEqual(123, sample.AngularRate, 2e-13);
            var dynamics = ConstrainedMultibodyDynamics.Evaluate(system, state, NoLoads(0, state));
            Assert.AreEqual(0, dynamics.Diagnostics.MaximumAngularPositionError, 1e-15);
        }
    }

    [TestMethod]
    public void DuplicateOrOverconstrainedJointsFailWithoutRegularization()
    {
        var (pair, state) = Centered();
        foreach (MultibodySystem system in new[] {
            new MultibodySystem(pair.Bodies, pair.Joints, new[] { pair.RevoluteJoints[0], pair.RevoluteJoints[0] }),
            new(pair.Bodies, new[] { new SphericalJoint(0, 1, default, default) }, pair.RevoluteJoints) })
        {
            var error = Assert.ThrowsExactly<ConstrainedSolveException>(() => ConstrainedMultibodyDynamics.Evaluate(system, state, NoLoads(0, state)));
            Assert.AreEqual(ConstrainedSolveFailure.SingularConstraints, error.Failure);
        }
    }

    [TestMethod]
    public void FailedNonlinearAndAngularProjectionSolvesPreserveInitialState()
    {
        var (system, state) = GeneralPair();
        var error = Assert.ThrowsExactly<ConstrainedSolveException>(() => ConstrainedMultibodyIntegrator.Step(system, state, 0, 0.1,
            NoLoads, new(maxNonlinearIterations: 1, nonlinearTolerance: 1e-14)));
        Assert.AreEqual(ConstrainedSolveFailure.NonlinearSolve, error.Failure);
        // Exact initial axes/rates avoid rejecting the fixture before the projection under test.
        var (centered, _) = Centered();
        var turning = new MultibodyState(new[] {
            new RigidBodyState(RigidPose3d.Identity, default, new(0, 0, 0.7)),
            new RigidBodyState(RigidPose3d.Identity, default, new(80, 0, 0.7)) });
        error = Assert.ThrowsExactly<ConstrainedSolveException>(() => ConstrainedMultibodyIntegrator.Step(centered, turning, 0, 0.001,
            NoLoads, new(new AngularConstraintSettings(1e-30, 1e-30), maxProjectionIterations: 1)));
        Assert.IsTrue(error.Failure is ConstrainedSolveFailure.PositionProjection or ConstrainedSolveFailure.VelocityProjection);
        Assert.IsTrue(error.Residual > 1);
        Assert.AreEqual(GeneralPair().State.Bodies[1], state.Bodies[1]);
    }

    [TestMethod]
    public void WrappedAngleIsQuaternionSignInvariantAndHistoryCrossesBothCuts()
    {
        var (system, state) = Centered(phase: 3.1, spin: -7);
        RigidBodyState b = state.Bodies[1];
        var oppositeSign = new MultibodyState(new[] { state.Bodies[0], new RigidBodyState(new(b.Pose.Position,
            b.Pose.Orientation * -1), b.LinearVelocity, b.BodyAngularVelocity) });
        Assert.AreEqual(RevoluteJointKinematics.Evaluate(system, state, 0), RevoluteJointKinematics.Evaluate(system, oppositeSign, 0));
        Assert.AreEqual(2 * System.Math.PI - 3, RevoluteJointKinematics.UnwrapAngle(3.1, -3), 1e-15);
        Assert.AreEqual(-2 * System.Math.PI + 3, RevoluteJointKinematics.UnwrapAngle(-3.1, 3), 1e-15);
        Assert.ThrowsExactly<ArgumentException>(() => RevoluteJointKinematics.UnwrapAngle(0, System.Math.PI));
        Assert.ThrowsExactly<ArgumentOutOfRangeException>(() => RevoluteJointKinematics.UnwrapAngle(double.NaN, 0));
        Assert.ThrowsExactly<ArgumentOutOfRangeException>(() => RevoluteJointKinematics.UnwrapAngle(0, -System.Math.PI));
        Assert.ThrowsExactly<ArgumentOutOfRangeException>(() => RevoluteJointKinematics.Evaluate(system, state, -1));
        Assert.ThrowsExactly<ArgumentOutOfRangeException>(() => RevoluteJointKinematics.Evaluate(system, state, 1));
    }

    [TestMethod]
    public void UnrepresentableGyroscopicArithmeticFailsExplicitly()
    {
        var (system, initial) = Centered();
        var state = new MultibodyState(new[] { initial.Bodies[0], new RigidBodyState(RigidPose3d.Identity, default, new(1e200, 1e200, 0)) });
        var error = Assert.ThrowsExactly<ConstrainedSolveException>(() => ConstrainedMultibodyDynamics.Evaluate(system, state, NoLoads(0, state)));
        Assert.AreEqual(ConstrainedSolveFailure.NumericalFailure, error.Failure);
        Assert.IsNotNull(error.InnerException);
        Assert.AreEqual(new Vector3d(1e200, 1e200, 0), state.Bodies[1].BodyAngularVelocity);
    }
}
