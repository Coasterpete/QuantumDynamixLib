using System;
using QuantumDynamix.Geometry;
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

    internal static (double Tangent, double Frame, double? Roll) Compare(CurveFrame source, CurveFrame fitted)
    {
        // Rebuild orthonormal bases before measuring their relative rotation.
        Vector3d ts = source.Forward.Normalized();
        Vector3d ls = Vector3d.Cross(source.Up, ts).Normalized();
        Vector3d us = Vector3d.Cross(ts, ls);
        Vector3d tf = fitted.Forward.Normalized();
        Vector3d lf = Vector3d.Cross(fitted.Up, tf).Normalized();
        Vector3d uf = Vector3d.Cross(tf, lf);
        Quaterniond relative = FromBasis(new(
            Vector3d.Dot(ts, tf), Vector3d.Dot(ts, lf), Vector3d.Dot(ts, uf),
            Vector3d.Dot(ls, tf), Vector3d.Dot(ls, lf), Vector3d.Dot(ls, uf),
            Vector3d.Dot(us, tf), Vector3d.Dot(us, lf), Vector3d.Dot(us, uf)));

        Vector3d axis = Vector3d.Cross(ts, tf);
        double sine = axis.Length;
        double cosine = Vector3d.Dot(ts, tf);
        double tangent = System.Math.Atan2(sine, cosine);
        double frame = PrincipalAngle(Quaterniond.Identity, relative);
        if (sine == 0 && cosine < 0) { return (tangent, frame, null); }
        Vector3d alignedUp = sine == 0 ? us : Quaterniond.FromAxisAngle(axis, tangent).Rotate(us);
        double roll = System.Math.Atan2(Vector3d.Dot(tf, Vector3d.Cross(alignedUp, uf)), Vector3d.Dot(alignedUp, uf));
        return (tangent, frame, roll);
    }

    internal static Quaterniond FromBasis(Matrix3x3d m)
    {
        double trace = m.M11 + m.M22 + m.M33;
        if (trace > 0)
        {
            double scale = 2 * System.Math.Sqrt(1 + trace);
            return new(scale / 4, (m.M32 - m.M23) / scale, (m.M13 - m.M31) / scale, (m.M21 - m.M12) / scale);
        }
        // The largest diagonal branch avoids division by the small scalar part near a half-turn.
        if (m.M11 >= m.M22 && m.M11 >= m.M33)
        {
            double scale = 2 * System.Math.Sqrt(1 + m.M11 - m.M22 - m.M33);
            return new((m.M32 - m.M23) / scale, scale / 4, (m.M12 + m.M21) / scale, (m.M13 + m.M31) / scale);
        }
        if (m.M22 >= m.M33)
        {
            double scale = 2 * System.Math.Sqrt(1 + m.M22 - m.M11 - m.M33);
            return new((m.M13 - m.M31) / scale, (m.M12 + m.M21) / scale, scale / 4, (m.M23 + m.M32) / scale);
        }
        double zScale = 2 * System.Math.Sqrt(1 + m.M33 - m.M11 - m.M22);
        return new((m.M21 - m.M12) / zScale, (m.M13 + m.M31) / zScale, (m.M23 + m.M32) / zScale, zScale / 4);
    }
}
