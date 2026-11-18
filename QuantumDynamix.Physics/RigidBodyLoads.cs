using QuantumDynamix.Math;

namespace QuantumDynamix.Physics;

/// <summary>World resultant force (N) and world torque about the center of mass (N*m).</summary>
/// <remarks>Zero/default means no applied loads. Gravity, damping, and other loads must be supplied explicitly.</remarks>
public readonly record struct RigidBodyLoads(Vector3d Force, Vector3d Torque)
{
    /// <summary>Creates the force and its moment for a world-space application point.</summary>
    public static RigidBodyLoads FromForceAtPoint(
        Vector3d force, Vector3d worldPoint, Vector3d worldCenterOfMass) =>
        new(force, Vector3d.Cross(worldPoint - worldCenterOfMass, force));

    public static RigidBodyLoads operator +(RigidBodyLoads left, RigidBodyLoads right) =>
        new(left.Force + right.Force, left.Torque + right.Torque);
}
