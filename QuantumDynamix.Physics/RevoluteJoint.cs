using System;
using QuantumDynamix.Geometry;
using QuantumDynamix.Math;

namespace QuantumDynamix.Physics;

/// <summary>Five bilateral constraints joining two body-local attachment frames. Origins are COM
/// offsets (m); local +X is the directed axle and Y/Z define zero relative angle.</summary>
/// <remarks>An ideal revolute joint permits axial rotation and transmits force and a transverse
/// couple. Definitions are validated when copied into a MultibodySystem.</remarks>
public readonly record struct RevoluteJoint(int BodyA, int BodyB, RigidPose3d LocalFrameA, RigidPose3d LocalFrameB);

/// <summary>Relative angle in (-pi,pi] rad, projected relative angular rate (rad/s), and world axle A.
/// AngularRate equals the angle derivative on an assembled joint. Off manifold it is a projected
/// diagnostic extension, not the derivative of the off-manifold angle chart.</summary>
public readonly record struct RevoluteJointSample(double Angle, double AngularRate, Vector3d WorldAxis);

/// <summary>Pure queries using body-attached frames and world angular velocities.</summary>
public static class RevoluteJointKinematics
{
    public static RevoluteJointSample Evaluate(MultibodySystem system, MultibodyState state, int jointIndex)
    {
        ArgumentNullException.ThrowIfNull(system);
        system.ValidateState(state);
        ArgumentOutOfRangeException.ThrowIfNegative(jointIndex);
        if (jointIndex >= system.RevoluteJoints.Count) { throw new ArgumentOutOfRangeException(nameof(jointIndex)); }
        RevoluteJoint joint = system.RevoluteJoints[jointIndex];
        RevoluteFrames frames = new(joint, state, "Revolute kinematics");
        double angle = System.Math.Atan2(Vector3d.Dot(frames.ZA, frames.YB), Vector3d.Dot(frames.YA, frames.YB));
        if (angle <= -System.Math.PI) { angle = System.Math.PI; }
        double rate = Vector3d.Dot(frames.AxisA,
            state.Bodies[joint.BodyB].WorldAngularVelocity - state.Bodies[joint.BodyA].WorldAngularVelocity);
        return new(angle, rate, frames.AxisA);
    }

    /// <summary>Chooses the current angle's nearest 2*pi branch to previousUnwrappedAngle (rad).</summary>
    /// <remarks>Cumulative revolutions require caller-maintained history updated only after accepted
    /// states. The true relative phase advance between samples must be strictly below pi. Endpoints
    /// cannot detect missed turns; this helper cannot certify step resolution. Never accumulate
    /// history in a load callback: Newton evaluates repeated/off-manifold trials. Very large histories
    /// lose phase precision; use an application turn counter if that range is required.</remarks>
    public static double UnwrapAngle(double previousUnwrappedAngle, double currentWrappedAngle)
    {
        if (!double.IsFinite(previousUnwrappedAngle)) { throw new ArgumentOutOfRangeException(nameof(previousUnwrappedAngle)); }
        if (!double.IsFinite(currentWrappedAngle) || currentWrappedAngle <= -System.Math.PI || currentWrappedAngle > System.Math.PI)
        { throw new ArgumentOutOfRangeException(nameof(currentWrappedAngle), "Supply an angle in (-pi,pi]."); }
        double previousPhase = System.Math.IEEERemainder(previousUnwrappedAngle, 2 * System.Math.PI);
        double change = System.Math.IEEERemainder(currentWrappedAngle - previousPhase, 2 * System.Math.PI);
        if (System.Math.Abs(change) == System.Math.PI)
        { throw new ArgumentException("A half-turn advance has no uniquely resolved branch.", nameof(currentWrappedAngle)); }
        double result = previousUnwrappedAngle + change;
        if (!double.IsFinite(result) || change != 0 && result == previousUnwrappedAngle)
        { throw new ArithmeticException("The accumulated angle change is not representable."); }
        return result;
    }
}

internal readonly struct RevoluteFrames
{
    internal Vector3d AxisA { get; }
    internal Vector3d AxisB { get; }
    internal Vector3d YA { get; }
    internal Vector3d ZA { get; }
    internal Vector3d YB { get; }
    internal double AlignmentError { get; }

    internal RevoluteFrames(RevoluteJoint joint, MultibodyState state, string stage)
    {
        RigidPose3d a = state.Bodies[joint.BodyA].Pose;
        RigidPose3d b = state.Bodies[joint.BodyB].Pose;
        AxisA = a.TransformDirection(joint.LocalFrameA.TransformDirection(Vector3d.UnitX));
        AxisB = b.TransformDirection(joint.LocalFrameB.TransformDirection(Vector3d.UnitX));
        YA = a.TransformDirection(joint.LocalFrameA.TransformDirection(Vector3d.UnitY));
        ZA = a.TransformDirection(joint.LocalFrameA.TransformDirection(Vector3d.UnitZ));
        YB = b.TransformDirection(joint.LocalFrameB.TransformDirection(Vector3d.UnitY));
        double dot = Vector3d.Dot(AxisA, AxisB);
        if (dot <= 0)
        {
            throw new ConstrainedSolveException(ConstrainedSolveFailure.UnsupportedJointConfiguration, stage, 0, dot,
                "Revolute axles must occupy the same directed hemisphere; antiparallel/perpendicular branches are unsupported (residual is axis dot product).");
        }
        // At a perpendicular axle the two dot constraints lose rank. Guard the chart itself
        // at double-precision resolution; scaled Cholesky can reject poor conditioning sooner.
        if (dot * dot <= 2.220446049250313e-16)
        {
            throw new ConstrainedSolveException(ConstrainedSolveFailure.SingularConstraints, stage, 0, dot,
                "The revolute alignment chart is numerically singular (residual is axis dot product).");
        }
        AlignmentError = System.Math.Atan2(Vector3d.Cross(AxisA, AxisB).Length, dot);
    }
}

/// <summary>Instantaneous world bearing force (N), independent couple (N*m), and total COM torques
/// (N*m). On an assembled ideal hinge CoupleOnA is perpendicular to the axle.</summary>
public readonly record struct RevoluteJointReaction(Vector3d ForceOnA, Vector3d CoupleOnA,
    Vector3d TorqueOnA, Vector3d TorqueOnB)
{
    public Vector3d ForceOnB => -ForceOnA;
    public Vector3d CoupleOnB => -CoupleOnA;
}

/// <summary>Discrete world force impulse (N*s), independent couple impulse (N*m*s), and total COM
/// angular impulses (N*m*s), at the rigid contact solve's start configuration.</summary>
public readonly record struct RevoluteJointImpulse(Vector3d ImpulseOnA, Vector3d CoupleImpulseOnA,
    Vector3d AngularImpulseOnA, Vector3d AngularImpulseOnB)
{
    public Vector3d ImpulseOnB => -ImpulseOnA;
    public Vector3d CoupleImpulseOnB => -CoupleImpulseOnA;
}
