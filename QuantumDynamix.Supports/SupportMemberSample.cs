using QuantumDynamix.Math;

namespace QuantumDynamix.Supports;

/// <summary>The evaluated geometry of one straight support member.</summary>
/// <remarks>Use <see cref="SupportMemberGeometry.Evaluate"/> to obtain a sample. The default value is zero-initialized.</remarks>
public readonly record struct SupportMemberSample
{
    /// <summary>The caller-supplied foundation point in world metres.</summary>
    public Vector3d FoundationPoint { get; }

    /// <summary>The evaluated rail section datum in world metres.</summary>
    public Vector3d RailAttachmentPoint { get; }

    /// <summary>Straight-line distance between the endpoints in metres.</summary>
    public double Length { get; }

    /// <summary>Dimensionless world-space unit vector from the foundation toward the rail attachment.</summary>
    public Vector3d Direction { get; }

    internal SupportMemberSample(Vector3d foundationPoint, Vector3d railAttachmentPoint, double length, Vector3d direction)
    {
        FoundationPoint = foundationPoint;
        RailAttachmentPoint = railAttachmentPoint;
        Length = length;
        Direction = direction;
    }
}
