using System;

namespace QuantumDynamix.Math;

/// <summary>A finite quaternion in scalar-first (W,X,Y,Z) order using Hamilton multiplication.</summary>
/// <remarks>General algebra permits non-unit values. Rotation operations normalize a nonzero quaternion.</remarks>
public readonly record struct Quaterniond
{
    public double W { get; }
    public double X { get; }
    public double Y { get; }
    public double Z { get; }

    public Quaterniond(double w, double x, double y, double z)
    {
        if (!double.IsFinite(w) || !double.IsFinite(x) || !double.IsFinite(y) || !double.IsFinite(z))
        {
            throw new ArgumentOutOfRangeException(nameof(w), "Quaternion components must all be finite.");
        }

        W = w;
        X = x;
        Y = y;
        Z = z;
    }

    public static Quaterniond Identity => new(1, 0, 0, 0);

    public Quaterniond Conjugated() => new(W, -X, -Y, -Z);

    public Quaterniond Normalized()
    {
        double scale = System.Math.Max(System.Math.Max(System.Math.Abs(W), System.Math.Abs(X)),
            System.Math.Max(System.Math.Abs(Y), System.Math.Abs(Z)));
        if (scale == 0)
        {
            throw new InvalidOperationException("The zero quaternion does not represent an orientation.");
        }

        Quaterniond scaled = this / scale;
        double norm = System.Math.Sqrt(scaled.W * scaled.W + scaled.X * scaled.X
            + scaled.Y * scaled.Y + scaled.Z * scaled.Z);
        return scaled / norm;
    }

    /// <summary>Creates an active right-handed rotation; angle is in radians.</summary>
    public static Quaterniond FromAxisAngle(Vector3d axis, double angle)
    {
        if (!double.IsFinite(angle))
        {
            throw new ArgumentOutOfRangeException(nameof(angle));
        }

        Vector3d unitAxis = axis.Normalized();
        double halfAngle = angle / 2;
        double sine = System.Math.Sin(halfAngle);
        return new Quaterniond(System.Math.Cos(halfAngle),
            unitAxis.X * sine, unitAxis.Y * sine, unitAxis.Z * sine).Normalized();
    }

    public Vector3d Rotate(Vector3d vector) => ToRotationMatrix() * vector;

    public Matrix3x3d ToRotationMatrix()
    {
        Quaterniond q = Normalized();
        return new(
            1 - 2 * (q.Y * q.Y + q.Z * q.Z), 2 * (q.X * q.Y - q.W * q.Z), 2 * (q.X * q.Z + q.W * q.Y),
            2 * (q.X * q.Y + q.W * q.Z), 1 - 2 * (q.X * q.X + q.Z * q.Z), 2 * (q.Y * q.Z - q.W * q.X),
            2 * (q.X * q.Z - q.W * q.Y), 2 * (q.Y * q.Z + q.W * q.X), 1 - 2 * (q.X * q.X + q.Y * q.Y));
    }

    public static Quaterniond operator *(Quaterniond left, Quaterniond right) => new(
        left.W * right.W - left.X * right.X - left.Y * right.Y - left.Z * right.Z,
        left.W * right.X + left.X * right.W + left.Y * right.Z - left.Z * right.Y,
        left.W * right.Y - left.X * right.Z + left.Y * right.W + left.Z * right.X,
        left.W * right.Z + left.X * right.Y - left.Y * right.X + left.Z * right.W);

    public static Quaterniond operator +(Quaterniond left, Quaterniond right) => new(
        left.W + right.W, left.X + right.X, left.Y + right.Y, left.Z + right.Z);

    public static Quaterniond operator *(Quaterniond value, double scalar)
    {
        if (!double.IsFinite(scalar))
        {
            throw new ArgumentOutOfRangeException(nameof(scalar));
        }

        return new(value.W * scalar, value.X * scalar, value.Y * scalar, value.Z * scalar);
    }

    public static Quaterniond operator *(double scalar, Quaterniond value) => value * scalar;

    public static Quaterniond operator /(Quaterniond value, double scalar)
    {
        if (!double.IsFinite(scalar))
        {
            throw new ArgumentOutOfRangeException(nameof(scalar));
        }

        if (scalar == 0)
        {
            throw new DivideByZeroException();
        }

        return new(value.W / scalar, value.X / scalar, value.Y / scalar, value.Z / scalar);
    }
}
