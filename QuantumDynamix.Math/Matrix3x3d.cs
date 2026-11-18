using System;

namespace QuantumDynamix.Math;

/// <summary>A finite 3-by-3 matrix; Mij denotes row i, column j. Vectors are columns.</summary>
public readonly record struct Matrix3x3d
{
    public double M11 { get; }
    public double M12 { get; }
    public double M13 { get; }
    public double M21 { get; }
    public double M22 { get; }
    public double M23 { get; }
    public double M31 { get; }
    public double M32 { get; }
    public double M33 { get; }

    public Matrix3x3d(
        double m11, double m12, double m13,
        double m21, double m22, double m23,
        double m31, double m32, double m33)
    {
        if (!double.IsFinite(m11) || !double.IsFinite(m12) || !double.IsFinite(m13)
            || !double.IsFinite(m21) || !double.IsFinite(m22) || !double.IsFinite(m23)
            || !double.IsFinite(m31) || !double.IsFinite(m32) || !double.IsFinite(m33))
        {
            throw new ArgumentOutOfRangeException(nameof(m11), "Matrix entries must all be finite.");
        }

        M11 = m11;
        M12 = m12;
        M13 = m13;
        M21 = m21;
        M22 = m22;
        M23 = m23;
        M31 = m31;
        M32 = m32;
        M33 = m33;
    }

    public static Matrix3x3d Identity => Diagonal(1, 1, 1);

    public static Matrix3x3d Diagonal(double x, double y, double z) => new(
        x, 0, 0,
        0, y, 0,
        0, 0, z);

    public Matrix3x3d Transposed() => new(
        M11, M21, M31,
        M12, M22, M32,
        M13, M23, M33);

    /// <summary>Solves A*x=b by scaled Cholesky factorization; A must be exactly symmetric and positive definite.</summary>
    public Vector3d SolvePositiveDefinite(Vector3d rightHandSide) =>
        new Cholesky3x3(this).Solve(rightHandSide);

    public static Vector3d operator *(Matrix3x3d matrix, Vector3d vector) => new(
        matrix.M11 * vector.X + matrix.M12 * vector.Y + matrix.M13 * vector.Z,
        matrix.M21 * vector.X + matrix.M22 * vector.Y + matrix.M23 * vector.Z,
        matrix.M31 * vector.X + matrix.M32 * vector.Y + matrix.M33 * vector.Z);

    public static Matrix3x3d operator *(Matrix3x3d left, Matrix3x3d right) => new(
        left.M11 * right.M11 + left.M12 * right.M21 + left.M13 * right.M31,
        left.M11 * right.M12 + left.M12 * right.M22 + left.M13 * right.M32,
        left.M11 * right.M13 + left.M12 * right.M23 + left.M13 * right.M33,
        left.M21 * right.M11 + left.M22 * right.M21 + left.M23 * right.M31,
        left.M21 * right.M12 + left.M22 * right.M22 + left.M23 * right.M32,
        left.M21 * right.M13 + left.M22 * right.M23 + left.M23 * right.M33,
        left.M31 * right.M11 + left.M32 * right.M21 + left.M33 * right.M31,
        left.M31 * right.M12 + left.M32 * right.M22 + left.M33 * right.M32,
        left.M31 * right.M13 + left.M32 * right.M23 + left.M33 * right.M33);
}

/// <summary>Stores A/scale=L*L^T so constant inertia can be factored once.</summary>
internal readonly struct Cholesky3x3
{
    private readonly double scale;
    private readonly double l11;
    private readonly double l21;
    private readonly double l22;
    private readonly double l31;
    private readonly double l32;
    private readonly double l33;

    internal Cholesky3x3(Matrix3x3d matrix)
    {
        if (matrix.M12 != matrix.M21 || matrix.M13 != matrix.M31 || matrix.M23 != matrix.M32)
        {
            throw new ArgumentException("Matrix must be exactly symmetric.", nameof(matrix));
        }

        // For a positive-definite matrix the largest magnitude entry lies on the diagonal.
        scale = System.Math.Max(matrix.M11, System.Math.Max(matrix.M22, matrix.M33));
        if (!(scale > 0))
        {
            throw new ArgumentException("Matrix must be positive definite.", nameof(matrix));
        }

        l11 = PositiveSquareRoot(matrix.M11 / scale);
        l21 = (matrix.M21 / scale) / l11;
        l31 = (matrix.M31 / scale) / l11;
        l22 = PositiveSquareRoot(matrix.M22 / scale - l21 * l21);
        l32 = (matrix.M32 / scale - l31 * l21) / l22;
        l33 = PositiveSquareRoot(matrix.M33 / scale - l31 * l31 - l32 * l32);
    }

    internal Vector3d Solve(Vector3d rightHandSide)
    {
        // Forward substitution: L*y=b/scale. Back substitution: L^T*x=y.
        Vector3d scaled = rightHandSide / scale;
        double y1 = scaled.X / l11;
        double y2 = (scaled.Y - l21 * y1) / l22;
        double y3 = (scaled.Z - l31 * y1 - l32 * y2) / l33;
        double x3 = y3 / l33;
        double x2 = (y2 - l32 * x3) / l22;
        double x1 = (y1 - l21 * x2 - l31 * x3) / l11;
        return new(x1, x2, x3);
    }

    private static double PositiveSquareRoot(double value)
    {
        if (!(value > 0) || !double.IsFinite(value))
        {
            throw new ArgumentException("Matrix must be numerically positive definite.");
        }

        return System.Math.Sqrt(value);
    }
}
