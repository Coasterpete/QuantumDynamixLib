using System;
using QuantumDynamix.Math;

namespace QuantumDynamix.FVD;

internal static class HeartlineFrameMath
{
    internal static Quaterniond AlignSign(Quaterniond rotation, Quaterniond reference) =>
        rotation.W * reference.W + rotation.X * reference.X + rotation.Y * reference.Y + rotation.Z * reference.Z < 0
            ? rotation * -1 : rotation;

    internal static double PrincipalAngle(Quaterniond a, Quaterniond b)
    {
        Quaterniond relative = (a.Normalized().Conjugated() * b.Normalized()).Normalized();
        return 2 * System.Math.Atan2(new Vector3d(relative.X, relative.Y, relative.Z).Length, System.Math.Abs(relative.W));
    }
}
