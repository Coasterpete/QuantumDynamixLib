using System;

namespace QuantumDynamix.Math;

/// <summary>A finite, double-precision vector in a right-handed coordinate system.</summary>
public readonly record struct Vector3d
{
    public double X { get; }
    public double Y { get; }
    public double Z { get; }

    public Vector3d(double x, double y, double z)
    {
        if (!double.IsFinite(x) || !double.IsFinite(y) || !double.IsFinite(z))
        {
            throw new ArgumentOutOfRangeException(nameof(x), "Vector components must all be finite.");
        }

        X = x;
        Y = y;
        Z = z;
    }

    public static Vector3d Zero => default;
    public static Vector3d UnitX => new(1, 0, 0);
    public static Vector3d UnitY => new(0, 1, 0);
    public static Vector3d UnitZ => new(0, 0, 1);

    public double Length
    {
        get
        {
            double scale = MaxAbsComponent;
            if (scale == 0)
            {
                return 0;
            }

            Vector3d scaled = this / scale;
            double length = scale * System.Math.Sqrt(Dot(scaled, scaled));
            if (!double.IsFinite(length))
            {
                throw new ArithmeticException("Vector length exceeds the finite double range.");
            }

            return length;
        }
    }

    /// <summary>Normalizes using scaling to avoid squaring extremely large or small components.</summary>
    public Vector3d Normalized()
    {
        double scale = MaxAbsComponent;
        if (scale == 0)
        {
            throw new InvalidOperationException("The zero vector has no direction.");
        }

        Vector3d scaled = this / scale;
        return scaled / System.Math.Sqrt(Dot(scaled, scaled));
    }

    public static double Dot(Vector3d left, Vector3d right)
    {
        double result = left.X * right.X + left.Y * right.Y + left.Z * right.Z;
        if (!double.IsFinite(result))
        {
            throw new ArithmeticException("Dot product exceeds the finite double range.");
        }

        return result;
    }

    public static Vector3d Cross(Vector3d left, Vector3d right) => new(
        left.Y * right.Z - left.Z * right.Y,
        left.Z * right.X - left.X * right.Z,
        left.X * right.Y - left.Y * right.X);

    public static Vector3d operator +(Vector3d left, Vector3d right) =>
        new(left.X + right.X, left.Y + right.Y, left.Z + right.Z);

    public static Vector3d operator -(Vector3d left, Vector3d right) =>
        new(left.X - right.X, left.Y - right.Y, left.Z - right.Z);

    public static Vector3d operator -(Vector3d value) => new(-value.X, -value.Y, -value.Z);

    public static Vector3d operator *(Vector3d value, double scalar)
    {
        RequireFiniteScalar(scalar);
        return new(value.X * scalar, value.Y * scalar, value.Z * scalar);
    }

    public static Vector3d operator *(double scalar, Vector3d value) => value * scalar;

    public static Vector3d operator /(Vector3d value, double scalar)
    {
        RequireFiniteScalar(scalar);
        if (scalar == 0)
        {
            throw new DivideByZeroException();
        }

        return new(value.X / scalar, value.Y / scalar, value.Z / scalar);
    }

    private double MaxAbsComponent => System.Math.Max(System.Math.Abs(X),
        System.Math.Max(System.Math.Abs(Y), System.Math.Abs(Z)));

    private static void RequireFiniteScalar(double scalar)
    {
        if (!double.IsFinite(scalar))
        {
            throw new ArgumentOutOfRangeException(nameof(scalar), "Scalar must be finite.");
        }
    }
}
