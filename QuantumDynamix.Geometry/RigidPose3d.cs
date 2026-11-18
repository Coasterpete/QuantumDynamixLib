using System;
using QuantumDynamix.Math;

namespace QuantumDynamix.Geometry;

/// <summary>World position in metres and a unit quaternion mapping body directions into world directions.</summary>
/// <remarks>Construction normalizes a nonzero orientation. The default struct value is not a valid pose.</remarks>
public readonly record struct RigidPose3d
{
    public Vector3d Position { get; }
    public Quaterniond Orientation { get; }

    public RigidPose3d(Vector3d position, Quaterniond orientation)
    {
        Position = position;
        Orientation = orientation.Normalized();
    }

    public static RigidPose3d Identity => new(Vector3d.Zero, Quaterniond.Identity);

    public Vector3d TransformPoint(Vector3d bodyPoint) => Position + TransformDirection(bodyPoint);

    public Vector3d TransformDirection(Vector3d bodyDirection)
    {
        Validate(nameof(Orientation));
        return Orientation.Rotate(bodyDirection);
    }

    public Vector3d InverseTransformPoint(Vector3d worldPoint) =>
        InverseTransformDirection(worldPoint - Position);

    public Vector3d InverseTransformDirection(Vector3d worldDirection)
    {
        Validate(nameof(Orientation));
        return Orientation.Conjugated().Rotate(worldDirection);
    }

    internal void Validate(string parameterName)
    {
        // All explicit constructors normalize; default(T) bypasses struct constructors.
        if (Orientation == default)
        {
            throw new ArgumentException("Pose must have a nonzero orientation; use RigidPose3d.Identity for identity.",
                parameterName);
        }
    }
}
