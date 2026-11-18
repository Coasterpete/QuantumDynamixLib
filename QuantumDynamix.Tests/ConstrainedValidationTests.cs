using System;
using System.Collections.Generic;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using QuantumDynamix.Geometry;
using QuantumDynamix.Math;
using QuantumDynamix.Physics;
using static QuantumDynamix.Tests.ConstrainedDynamicsTests;

namespace QuantumDynamix.Tests;

[TestClass]
public sealed class ConstrainedValidationTests
{
    [TestMethod]
    public void DefinitionsAndSnapshotsDefensivelyCopyInputsAndExposeReadOnlyCollections()
    {
        RigidBodyMassProperties[] masses = { Mass(2), Mass(3) };
        SphericalJoint[] joints = { new(0, 1, default, default) };
        RigidBodyState[] bodies = { Rest(), Rest() };
        MultibodySystem system = new(masses, joints);
        MultibodyState state = new(bodies);
        masses[0] = Mass(100);
        joints[0] = new(1, 1, default, default);
        bodies[0] = Rest(Vector3d.UnitX);
        Assert.AreEqual(2, system.Bodies[0].Mass);
        Assert.AreEqual(0, system.Joints[0].BodyA);
        Assert.AreEqual(Vector3d.Zero, state.Bodies[0].Pose.Position);
        Assert.ThrowsExactly<NotSupportedException>(() => ((IList<RigidBodyState>)state.Bodies)[0] = bodies[0]);
        Assert.ThrowsExactly<NotSupportedException>(() => ((IList<SphericalJoint>)system.Joints)[0] = joints[0]);
    }

    [TestMethod]
    public void InvalidDefinitionsStateCountsAndLoadsAreRejected()
    {
        Assert.ThrowsExactly<ArgumentNullException>(() => new MultibodySystem(null!, Array.Empty<SphericalJoint>()));
        Assert.ThrowsExactly<ArgumentNullException>(() => new MultibodySystem(new[] { Mass(1) }, null!));
        Assert.ThrowsExactly<ArgumentException>(() => new MultibodySystem(Array.Empty<RigidBodyMassProperties>(), Array.Empty<SphericalJoint>()));
        Assert.ThrowsExactly<ArgumentNullException>(() => new MultibodySystem(new RigidBodyMassProperties[] { null! }, Array.Empty<SphericalJoint>()));
        foreach (SphericalJoint joint in new[] { new SphericalJoint(-1, 1, default, default), new(0, 2, default, default), new(0, 0, default, default) })
        {
            Assert.ThrowsExactly<ArgumentException>(() => new MultibodySystem(new[] { Mass(2), Mass(3) }, new[] { joint }));
        }

        Assert.ThrowsExactly<ArgumentException>(() => new MultibodyState(Array.Empty<RigidBodyState>()));
        Assert.ThrowsExactly<ArgumentException>(() => new MultibodyState(new RigidBodyState[1]));
        Assert.ThrowsExactly<ArgumentNullException>(() => new MultibodyState(null!));
        (MultibodySystem system, MultibodyState state) = RotatingPair();
        Assert.ThrowsExactly<ArgumentException>(() => ConstrainedMultibodyDynamics.Evaluate(system, new(new[] { Rest() }), new RigidBodyLoads[2]));
        Assert.ThrowsExactly<ArgumentException>(() => ConstrainedMultibodyDynamics.Evaluate(system, state, new RigidBodyLoads[1]));
        Assert.ThrowsExactly<ArgumentNullException>(() => ConstrainedMultibodyDynamics.Evaluate(system, state, null!));
        Assert.ThrowsExactly<ArgumentNullException>(() => ConstrainedMultibodyIntegrator.Step(system, state, 0, 0.1, null!));
        Assert.ThrowsExactly<ArgumentException>(() => ConstrainedMultibodyIntegrator.Step(system, state, 0, 0.1, (_, _) => new RigidBodyLoads[1]));
    }

    [TestMethod]
    [DataRow(0.0)]
    [DataRow(-1.0)]
    [DataRow(double.NaN)]
    [DataRow(double.PositiveInfinity)]
    public void InvalidSettingsAreRejected(double value)
    {
        Assert.ThrowsExactly<ArgumentOutOfRangeException>(() => new ConstrainedSolverSettings(positionTolerance: value));
        Assert.ThrowsExactly<ArgumentOutOfRangeException>(() => new ConstrainedSolverSettings(velocityTolerance: value));
        Assert.ThrowsExactly<ArgumentOutOfRangeException>(() => new ConstrainedSolverSettings(nonlinearTolerance: value));
        Assert.ThrowsExactly<ArgumentOutOfRangeException>(() => new ConstrainedSolverSettings(linearSolveTolerance: value));
        Assert.ThrowsExactly<ArgumentOutOfRangeException>(() => new ConstrainedSolverSettings(minimumScaledPivot: value));
        Assert.ThrowsExactly<ArgumentOutOfRangeException>(() => new ConstrainedSolverSettings(linearVelocityScale: value));
        Assert.ThrowsExactly<ArgumentOutOfRangeException>(() => new ConstrainedSolverSettings(angularVelocityScale: value));
        Assert.ThrowsExactly<ArgumentOutOfRangeException>(() => new ConstrainedSolverSettings(finiteDifferenceStep: value));
    }

    [TestMethod]
    public void InvalidIterationAndDifferenceControlsAreRejected()
    {
        Assert.ThrowsExactly<ArgumentOutOfRangeException>(() => new ConstrainedSolverSettings(maxNonlinearIterations: 0));
        Assert.ThrowsExactly<ArgumentOutOfRangeException>(() => new ConstrainedSolverSettings(maxProjectionIterations: 0));
        Assert.ThrowsExactly<ArgumentOutOfRangeException>(() => new ConstrainedSolverSettings(maxLineSearchIterations: 0));
        Assert.ThrowsExactly<ArgumentOutOfRangeException>(() => new ConstrainedSolverSettings(minimumScaledPivot: 1));
        Assert.ThrowsExactly<ArgumentOutOfRangeException>(() => new ConstrainedSolverSettings(finiteDifferenceStep: 1));
    }

    [TestMethod]
    [DataRow(0.0)]
    [DataRow(-0.1)]
    [DataRow(double.NaN)]
    [DataRow(double.PositiveInfinity)]
    [DataRow(double.Epsilon)]
    public void InvalidStepsAreRejectedBeforeLoadEvaluation(double step)
    {
        (MultibodySystem system, MultibodyState state) = RotatingPair();
        int calls = 0;
        Assert.ThrowsExactly<ArgumentOutOfRangeException>(() => ConstrainedMultibodyIntegrator.Step(system, state,
            0, step, (_, _) => { calls++; return new RigidBodyLoads[2]; }));
        Assert.AreEqual(0, calls);
    }

    [TestMethod]
    [DataRow(double.NaN, 0.1)]
    [DataRow(double.MaxValue, double.MaxValue)]
    [DataRow(1e20, 0.1)]
    public void UnrepresentableTimesAreRejected(double time, double step)
    {
        (MultibodySystem system, MultibodyState state) = RotatingPair();
        Assert.ThrowsExactly<ArgumentOutOfRangeException>(() => ConstrainedMultibodyIntegrator.Step(system, state, time, step, NoLoads));
    }

    [TestMethod]
    [DataRow(false)]
    [DataRow(true)]
    public void DuplicateAndGeometricallyRedundantConstraintsFailExplicitly(bool distinctAnchors)
    {
        Vector3d anchor = distinctAnchors ? Vector3d.UnitX : Vector3d.Zero;
        MultibodySystem system = new(new[] { Mass(2), Mass(3) }, new[]
        {
            new SphericalJoint(0, 1, default, default), new SphericalJoint(0, 1, anchor, anchor)
        });
        MultibodyState state = new(new[] { Rest(), Rest() });
        ConstrainedSolveException error = Assert.ThrowsExactly<ConstrainedSolveException>(() =>
            ConstrainedMultibodyDynamics.Evaluate(system, state, NoLoads(0, state)));
        Assert.AreEqual(ConstrainedSolveFailure.SingularConstraints, error.Failure);
        Assert.IsTrue(error.Residual <= 1e-12);
        int calls = 0;
        error = Assert.ThrowsExactly<ConstrainedSolveException>(() =>
            ConstrainedMultibodyIntegrator.Step(system, state, 0, 0.1, (_, _) => { calls++; return new RigidBodyLoads[2]; }));
        Assert.AreEqual(ConstrainedSolveFailure.SingularConstraints, error.Failure);
        Assert.AreEqual("Initial constraints", error.Stage);
        Assert.AreEqual(0, calls);
    }

    [TestMethod]
    [DataRow(false)]
    [DataRow(true)]
    public void InconsistentInitialGapOrSlipFailsWithoutAdvancing(bool velocity)
    {
        MultibodySystem system = new(new[] { Mass(2), Mass(3) }, new[] { new SphericalJoint(0, 1, default, default) });
        MultibodyState state = new(new[] { Rest(), new RigidBodyState(new(velocity ? default : new Vector3d(0.001, 0, 0), Quaterniond.Identity),
            velocity ? new Vector3d(0.001, 0, 0) : default, default) });
        int calls = 0;
        ConstrainedSolveException error = Assert.ThrowsExactly<ConstrainedSolveException>(() =>
            ConstrainedMultibodyIntegrator.Step(system, state, 0, 0.1, (_, _) => { calls++; return new RigidBodyLoads[2]; }));
        Assert.AreEqual(ConstrainedSolveFailure.InconsistentInitialState, error.Failure);
        Assert.IsTrue(error.Residual > 1);
        Assert.AreEqual(0, calls);
    }

    [TestMethod]
    public void FailedNonlinearSolveProvidesDiagnosticsAndPreservesSnapshot()
    {
        MultibodySystem system = new(new[] { Mass(1) }, Array.Empty<SphericalJoint>());
        RigidBodyState body = new(RigidPose3d.Identity, default, new(0, 0, 2));
        MultibodyState state = new(new[] { body });
        ConstrainedSolveException error = Assert.ThrowsExactly<ConstrainedSolveException>(() =>
            ConstrainedMultibodyIntegrator.Step(system, state, 0, 0.4, (_, s) =>
                new[] { new RigidBodyLoads(default, new(0, 0, -System.Math.Pow(s.Bodies[0].BodyAngularVelocity.Z, 3))) },
                new(maxNonlinearIterations: 1, nonlinearTolerance: 1e-14)));
        Assert.AreEqual(ConstrainedSolveFailure.NonlinearSolve, error.Failure);
        Assert.AreEqual(1, error.Iterations);
        Assert.IsTrue(error.Residual > 1e-14);
        Assert.AreEqual(body, state.Bodies[0]);
    }

    [TestMethod]
    public void ImpossiblePositionToleranceFailsProjectionWithDiagnostics()
    {
        (MultibodySystem system, MultibodyState state) = RotatingPair();
        ConstrainedSolveException error = Assert.ThrowsExactly<ConstrainedSolveException>(() =>
            ConstrainedMultibodyIntegrator.Step(system, state, 0, 0.2, NoLoads,
                new(positionTolerance: 1e-30, maxProjectionIterations: 1)));
        Assert.AreEqual(ConstrainedSolveFailure.PositionProjection, error.Failure);
        Assert.IsTrue(error.Residual > 1e-30);
        Assert.AreEqual("Position projection", error.Stage);
    }

    [TestMethod]
    public void ImpossibleVelocityToleranceFailsProjectionWithDiagnostics()
    {
        (MultibodySystem system, MultibodyState state) = RotatingPair();
        ConstrainedSolveException error = Assert.ThrowsExactly<ConstrainedSolveException>(() =>
            ConstrainedMultibodyIntegrator.Step(system, state, 0, 0.2, NoLoads,
                new(velocityTolerance: 1e-30)));
        Assert.AreEqual(ConstrainedSolveFailure.VelocityProjection, error.Failure);
        Assert.IsTrue(error.Residual > 1e-30);
        Assert.AreEqual("Velocity projection", error.Stage);
    }

    [TestMethod]
    public void NonfiniteDerivedAccelerationReportsNumericalFailure()
    {
        MultibodySystem system = new(new[] { Mass(double.Epsilon) }, Array.Empty<SphericalJoint>());
        MultibodyState state = new(new[] { Rest() });
        ConstrainedSolveException error = Assert.ThrowsExactly<ConstrainedSolveException>(() =>
            ConstrainedMultibodyDynamics.Evaluate(system, state, new[] { new RigidBodyLoads(Vector3d.UnitX, default) }));
        Assert.AreEqual(ConstrainedSolveFailure.NumericalFailure, error.Failure);
        Assert.IsNotNull(error.InnerException);
    }
}
