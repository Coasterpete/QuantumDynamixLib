using System;
using QuantumDynamix.Geometry;
using QuantumDynamix.Math;
using QuantumDynamix.Math.Numerics;

namespace QuantumDynamix.FVD;

/// <summary>One normalized Hermite orientation segment in a basis relative to its starting rotation.</summary>
internal readonly struct HeartlineRotationPiece
{
    private readonly Quaterniond relativeEnd;
    private readonly Quaterniond linear;
    private readonly Quaterniond quadratic;
    private readonly Quaterniond cubic;

    internal double Start { get; }
    internal double End { get; }
    internal Quaterniond StartRotation { get; }
    internal Quaterniond EndRotation => relativeEnd == Quaterniond.Identity ? StartRotation : (StartRotation * relativeEnd).Normalized();
    internal Vector3d StartRate { get; }
    internal Vector3d EndRate { get; }

    internal HeartlineRotationPiece(double start, double end, Quaterniond rotation, Quaterniond relativeEnd,
        Vector3d startRate, Vector3d endRate)
    {
        Start = start;
        End = end;
        StartRotation = rotation;
        this.relativeEnd = HeartlineFrameMath.AlignSign(relativeEnd.Normalized(), Quaterniond.Identity);
        StartRate = startRate;
        EndRate = endRate;
        double span = end - start;
        linear = Rate(Quaterniond.Identity, startRate) * span;
        Quaterniond slopeEnd = Rate(this.relativeEnd, endRate) * span;
        Quaterniond change = this.relativeEnd + Quaterniond.Identity * -1;
        quadratic = change * 3 + linear * -2 + slopeEnd * -1;
        cubic = change * -2 + linear + slopeEnd;
    }

    internal CurveFrame Frame(double station, Vector3d position)
    {
        Matrix3x3d rotation = Orientation(station).ToRotationMatrix();
        return new(position, rotation * Vector3d.UnitX, rotation * Vector3d.UnitY, rotation * Vector3d.UnitZ);
    }

    internal Quaterniond Orientation(double station)
    {
        Quaterniond relative = RelativeOrientation(station);
        return relative == Quaterniond.Identity ? StartRotation : (StartRotation * relative).Normalized();
    }

    internal Quaterniond RelativeOrientation(double station) => station == Start ? Quaterniond.Identity
        : station == End ? relativeEnd : Polynomial((station - Start) / (End - Start)).Normalized();

    internal Vector3d AngularRate(double station)
    {
        if (station == Start) { return StartRate; }
        if (station == End) { return EndRate; }
        double z = (station - Start) / (End - Start);
        Quaterniond p = Polynomial(z);
        Quaterniond dp = (linear + (quadratic * 2 + cubic * (3 * z)) * z) / (End - Start);
        Quaterniond product = p.Conjugated() * dp;
        Quaterniond square = p.Conjugated() * p;
        return new Vector3d(product.X, product.Y, product.Z) * (2 / square.W);
    }

    internal (Vector3d Displacement, double Error) Displacement(double end, double tolerance, ReconstructionWork work)
    {
        if (end == Start) { return (Vector3d.Zero, 0); }
        HeartlineRotationPiece piece = this;
        try
        {
            QuadratureResult x = Integrate(Vector3d.UnitX);
            QuadratureResult y = Integrate(Vector3d.UnitY);
            QuadratureResult z = Integrate(Vector3d.UnitZ);
            return (new(x.Value, y.Value, z.Value), x.EstimatedAbsoluteError + y.EstimatedAbsoluteError + z.EstimatedAbsoluteError);
        }
        catch (NumericalConvergenceException ex)
        {
            throw work.Fail(HeartlineFailureCode.NumericalBudgetExceeded,
                "Position quadrature exhausted its budget.", (Start, end),
                measurement: ("Position quadrature error estimate", "m", ex.Residual, tolerance), inner: ex);
        }

        QuadratureResult Integrate(Vector3d axis)
        {
            if (!(tolerance / 3 > 0) || work.Remaining < 15)
            { throw work.Fail(HeartlineFailureCode.NumericalBudgetExceeded, "Insufficient work or tolerance for position quadrature.",
                (piece.Start, end)); }
            return AdaptiveGaussKronrod.Integrate(s =>
            {
                work.Charge(s);
                Vector3d tangent = piece.Orientation(s).Rotate(Vector3d.UnitX);
                return Vector3d.Dot(axis, tangent);
            }, piece.Start, end, new QuadratureSettings(tolerance / 3, 0, int.MaxValue, work.Remaining));
        }
    }

    private Quaterniond Polynomial(double z) => Quaterniond.Identity + (linear + (quadratic + cubic * z) * z) * z;

    internal static Quaterniond Rate(Quaterniond q, Vector3d omega) =>
        q * new Quaterniond(0, omega.X, omega.Y, omega.Z) * 0.5;
}
