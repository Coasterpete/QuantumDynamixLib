using QuantumDynamix.Math;

namespace QuantumDynamix.Physics;

/// <summary>Time derivatives of position, quaternion components, world velocity, and body angular velocity.</summary>
/// <remarks>OrientationRate is a quaternion derivative, not an orientation, and must not be normalized.</remarks>
public readonly record struct RigidBodyDerivative(
    Vector3d PositionRate,
    Quaterniond OrientationRate,
    Vector3d LinearAcceleration,
    Vector3d BodyAngularAcceleration);
