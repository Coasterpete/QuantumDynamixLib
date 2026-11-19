using System;

namespace QuantumDynamix.Physics;

/// <summary>Provisional angular numerical controls, independent of metre-valued attachment controls.
/// OrientationTolerance is actual axle misalignment in rad. AngularVelocityTolerance bounds the norm
/// of the two alignment-row rates (1/s, equal to forbidden angular speed on an aligned hinge).</summary>
public sealed class AngularConstraintSettings
{
    public static AngularConstraintSettings Default { get; } = new();
    public double OrientationTolerance { get; }
    public double AngularVelocityTolerance { get; }

    public AngularConstraintSettings(double orientationTolerance = 1e-10, double angularVelocityTolerance = 1e-10)
    {
        if (!double.IsFinite(orientationTolerance) || orientationTolerance <= 0 || orientationTolerance >= System.Math.PI / 2)
        { throw new ArgumentOutOfRangeException(nameof(orientationTolerance), "Supply a positive angular tolerance below pi/2 rad."); }
        if (!double.IsFinite(angularVelocityTolerance) || angularVelocityTolerance <= 0)
        { throw new ArgumentOutOfRangeException(nameof(angularVelocityTolerance)); }
        OrientationTolerance = orientationTolerance;
        AngularVelocityTolerance = angularVelocityTolerance;
    }
}
