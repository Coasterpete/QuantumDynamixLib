using QuantumDynamix.Geometry;
using QuantumDynamix.Math;

namespace QuantumDynamix.Physics;

/// <summary>Center-of-mass pose, world linear velocity (m/s), and body-frame angular velocity (rad/s).</summary>
/// <remarks>Angular velocity is expressed in the rotating body frame. Default state has an invalid orientation.</remarks>
public readonly record struct RigidBodyState
{
    public RigidPose3d Pose { get; }
    public Vector3d LinearVelocity { get; }
    public Vector3d BodyAngularVelocity { get; }

    public RigidBodyState(RigidPose3d pose, Vector3d linearVelocity, Vector3d bodyAngularVelocity)
    {
        pose.Validate(nameof(pose));
        Pose = pose;
        LinearVelocity = linearVelocity;
        BodyAngularVelocity = bodyAngularVelocity;
    }

    public Vector3d WorldAngularVelocity => Pose.TransformDirection(BodyAngularVelocity);

    internal void Validate(string parameterName) => Pose.Validate(parameterName);
}
