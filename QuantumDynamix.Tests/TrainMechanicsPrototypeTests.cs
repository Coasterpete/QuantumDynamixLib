using Microsoft.VisualStudio.TestTools.UnitTesting;
using QuantumDynamix.Geometry;
using QuantumDynamix.Math;
using QuantumDynamix.Physics;
using static QuantumDynamix.Tests.ConstrainedDynamicsTests;

namespace QuantumDynamix.Tests;

[TestClass]
public sealed class TrainMechanicsPrototypeTests
{
    // Synthetic offsets from the car center of mass in metres. X is forward, Y is left, Z is up.
    private static readonly RigidPose3d LeftCarrier = new(new(0, 0.5, -0.25),
        Quaterniond.FromAxisAngle(Vector3d.UnitZ, System.Math.PI / 2));
    private static readonly RigidPose3d RightCarrier = new(new(0, -0.5, -0.25),
        Quaterniond.FromAxisAngle(Vector3d.UnitZ, -System.Math.PI / 2));
    private static readonly RigidPose3d FrontConnector = new(Vector3d.UnitX, Quaterniond.Identity);
    private static readonly RigidPose3d RearConnector = new(-Vector3d.UnitX,
        Quaterniond.FromAxisAngle(Vector3d.UnitZ, System.Math.PI));

    public TestContext TestContext { get; set; } = null!;

    [TestMethod]
    public void SingleCarCarriesAllFourAttachmentFramesThroughTranslationAndRotation()
    {
        RigidBodyMassProperties mass = Mass(2, new(2, 3, 4));
        Quaterniond startRotation = Quaterniond.FromAxisAngle(Vector3d.UnitZ, System.Math.PI / 2);
        RigidBodyState initial = new(new(new(2, -3, 1), startRotation), new(1, -2, 0.5), Vector3d.UnitX);
        RigidBodyState state = initial;
        const double step = 0.01;
        const int stepCount = 40;
        for (int i = 0; i < stepCount; i++)
        {
            state = RigidBodyIntegrator.Step(mass, state, i * step, step, (_, _) => default);
        }

        // With zero loads, this principal-axis spin and constant translation have an analytical endpoint.
        double elapsed = stepCount * step;
        TestMath.Near(initial.Pose.Position + initial.LinearVelocity * elapsed, state.Pose.Position, 1e-14);
        TestMath.SameRotation(startRotation * Quaterniond.FromAxisAngle(Vector3d.UnitX, elapsed),
            state.Pose.Orientation, 2e-9);

        RigidPose3d[] localFrames = [LeftCarrier, RightCarrier, FrontConnector, RearConnector];
        foreach (RigidPose3d localFrame in localFrames)
        {
            AssertAttachmentFollows(initial.Pose, localFrame);
            AssertAttachmentFollows(state.Pose, localFrame);
        }
    }

    [TestMethod]
    [DataRow(0.0, 0.0)]
    [DataRow(0.65, 0.0)]
    [DataRow(-0.55, 0.45)]
    public void TwoCarsArticulateOffRailAndRefineConsistently(double yaw, double pitch)
    {
        SphericalJoint joint = new(0, 1, RearConnector.Position, FrontConnector.Position);
        MultibodySystem system = new([Mass(2, new(2, 3, 4)), Mass(3, new(3, 4, 5))], [joint]);
        Quaterniond orientationA = Quaterniond.FromAxisAngle(new(1, 2, 3), 0.7);
        Quaterniond orientationB = orientationA * Quaterniond.FromAxisAngle(Vector3d.UnitZ, yaw)
            * Quaterniond.FromAxisAngle(Vector3d.UnitY, pitch);
        Vector3d connectorPoint = new(0.1, 0.2, -0.1);
        Vector3d connectorVelocity = new(0.2, -0.4, 0.3);
        MultibodyState initial = new([
            AssembleCar(orientationA, joint.LocalAnchorA, new(0.3, 0.7, -0.2), connectorPoint, connectorVelocity),
            AssembleCar(orientationB, joint.LocalAnchorB, new(-0.5, 0.2, 0.6), connectorPoint, connectorVelocity)
        ]);
        ConstrainedSolverSettings settings = ConstrainedSolverSettings.Default;
        Quaterniond initialRelativeRotation = RelativeRotation(initial);

        // Compare endpoints at the same time against the finest run, as in the existing tumbling-pair test.
        MultibodyState reference = Run(0.0025, 400);
        (double Position, double Angle) coarse = EndpointError(Run(0.04, 25), reference);
        (double Position, double Angle) medium = EndpointError(Run(0.02, 50), reference);
        (double Position, double Angle) fine = EndpointError(Run(0.01, 100), reference);
        TestContext.WriteLine($"Endpoint differences vs 0.0025 s, coarse/medium/fine: "
            + $"position={coarse.Position:E6}/{medium.Position:E6}/{fine.Position:E6} m, "
            + $"orientation={coarse.Angle:E6}/{medium.Angle:E6}/{fine.Angle:E6} rad");
        Assert.IsTrue(fine.Position < medium.Position && medium.Position < coarse.Position);
        Assert.IsTrue(fine.Angle < medium.Angle && medium.Angle < coarse.Angle);

        MultibodyState Run(double step, int stepCount)
        {
            MultibodyState state = initial;
            double maxGap = 0;
            double maxSlip = 0;
            double maxAccelerationError = 0;
            double maxLinearResidual = 0;
            for (int i = 0; i < stepCount; i++)
            {
                ConstrainedStepResult result = ConstrainedMultibodyIntegrator.Step(
                    system, state, i * step, step, NoLoads, settings);
                state = result.State;
                RigidBodyState bodyA = state.Bodies[joint.BodyA];
                RigidBodyState bodyB = state.Bodies[joint.BodyB];
                Vector3d anchorA = bodyA.Pose.TransformPoint(joint.LocalAnchorA);
                Vector3d anchorB = bodyB.Pose.TransformPoint(joint.LocalAnchorB);
                Vector3d velocityA = bodyA.LinearVelocity
                    + Vector3d.Cross(bodyA.WorldAngularVelocity, anchorA - bodyA.Pose.Position);
                Vector3d velocityB = bodyB.LinearVelocity
                    + Vector3d.Cross(bodyB.WorldAngularVelocity, anchorB - bodyB.Pose.Position);
                ConstraintDiagnostics diagnostics = result.AcceptedDynamics.Diagnostics;
                maxGap = System.Math.Max(maxGap,
                    System.Math.Max((anchorA - anchorB).Length, diagnostics.MaximumPositionError));
                maxSlip = System.Math.Max(maxSlip,
                    System.Math.Max((velocityA - velocityB).Length, diagnostics.MaximumVelocityError));
                maxAccelerationError = System.Math.Max(maxAccelerationError, diagnostics.MaximumAccelerationError);
                maxLinearResidual = System.Math.Max(maxLinearResidual, diagnostics.LinearSolveRelativeResidual);

                foreach (RigidBodyState body in state.Bodies)
                {
                    Vector3d p = body.Pose.Position;
                    Quaterniond q = body.Pose.Orientation;
                    Assert.IsTrue(double.IsFinite(p.X) && double.IsFinite(p.Y) && double.IsFinite(p.Z));
                    Assert.AreEqual(1, q.W * q.W + q.X * q.X + q.Y * q.Y + q.Z * q.Z, 1e-15);
                }
            }

            double relativeRotation = RotationDistance(initialRelativeRotation, RelativeRotation(state));
            TestContext.WriteLine($"yaw={yaw:F2}, pitch={pitch:F2} rad, dt={step:G} s: "
                + $"max gap={maxGap:E6} m, velocity mismatch={maxSlip:E6} m/s, "
                + $"acceleration residual={maxAccelerationError:E6} m/s^2, "
                + $"scaled linear residual={maxLinearResidual:E6}, relative rotation change={relativeRotation:F6} rad");
            Assert.IsTrue(maxGap <= settings.PositionTolerance);
            Assert.IsTrue(maxSlip <= settings.VelocityTolerance);
            Assert.IsTrue(double.IsFinite(maxAccelerationError));
            Assert.IsTrue(maxLinearResidual <= settings.LinearSolveTolerance);
            // Guard against a locked connector in this fixture, where the bodies start with distinct spin.
            Assert.IsTrue(relativeRotation > 0.1);
            return state;
        }
    }

    private static void AssertAttachmentFollows(RigidPose3d car, RigidPose3d localFrame)
    {
        RigidPose3d worldFrame = new(car.TransformPoint(localFrame.Position), car.Orientation * localFrame.Orientation);
        TestMath.Near(localFrame.Position, car.InverseTransformPoint(worldFrame.Position), 1e-14);
        Vector3d[] axes = [Vector3d.UnitX, Vector3d.UnitY, Vector3d.UnitZ];
        foreach (Vector3d axis in axes)
        {
            TestMath.Near(localFrame.TransformDirection(axis),
                car.InverseTransformDirection(worldFrame.TransformDirection(axis)), 1e-14);
        }
    }

    private static RigidBodyState AssembleCar(Quaterniond orientation, Vector3d localAnchor, Vector3d bodySpin,
        Vector3d connectorPoint, Vector3d connectorVelocity)
    {
        Vector3d position = connectorPoint - orientation.Rotate(localAnchor);
        Vector3d velocity = connectorVelocity - orientation.Rotate(Vector3d.Cross(bodySpin, localAnchor));
        return new(new(position, orientation), velocity, bodySpin);
    }

    private static Quaterniond RelativeRotation(MultibodyState state) =>
        state.Bodies[0].Pose.Orientation.Conjugated() * state.Bodies[1].Pose.Orientation;

    private static double RotationDistance(Quaterniond from, Quaterniond to)
    {
        Quaterniond difference = from.Conjugated() * to;
        // The absolute scalar gives the shorter angle and makes the result independent of quaternion sign.
        Vector3d vector = new(difference.X, difference.Y, difference.Z);
        return 2 * System.Math.Atan2(vector.Length, System.Math.Abs(difference.W));
    }

    private static (double Position, double Angle) EndpointError(MultibodyState state, MultibodyState reference)
    {
        double position = 0;
        double angle = 0;
        for (int i = 0; i < state.Bodies.Count; i++)
        {
            RigidPose3d actual = state.Bodies[i].Pose;
            RigidPose3d expected = reference.Bodies[i].Pose;
            position = System.Math.Max(position, (actual.Position - expected.Position).Length);
            angle = System.Math.Max(angle, RotationDistance(expected.Orientation, actual.Orientation));
        }

        return (position, angle);
    }
}
