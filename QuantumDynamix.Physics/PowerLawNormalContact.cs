using System;

namespace QuantumDynamix.Physics;

/// <summary>Owner-supplied normal law F=K*delta^p*max(0,1+alpha*deltaDot).
/// K is N/m^p, p is dimensionless, alpha is s/m, and maximum compression is m.</summary>
/// <remarks>No material, Hertz coefficient, restitution, preload, or numerical softness is inferred.
/// p>=1 gives finite nonnegative elastic tangent stiffness at zero. Positive quantities that cannot
/// be represented, including underflow, fail explicitly rather than silently removing support.</remarks>
public sealed class PowerLawNormalContact
{
    public double Coefficient { get; }
    public double Exponent { get; }
    public double Dissipation { get; }
    public double MaximumCompression { get; }

    public PowerLawNormalContact(double coefficient, double exponent, double dissipation, double maximumCompression)
    {
        ContactChecks.Positive(coefficient, nameof(coefficient));
        ContactChecks.Positive(maximumCompression, nameof(maximumCompression));
        if (!double.IsFinite(exponent) || exponent < 1 || !double.IsFinite(exponent + 1))
        { throw new ArgumentOutOfRangeException(nameof(exponent), "Supply a finite exponent >=1 with finite p+1."); }
        if (!double.IsFinite(dissipation) || dissipation < 0)
        { throw new ArgumentOutOfRangeException(nameof(dissipation), "Dissipation must be finite and nonnegative."); }
        Coefficient = coefficient;
        Exponent = exponent;
        Dissipation = dissipation;
        MaximumCompression = maximumCompression;
        try { Evaluate(maximumCompression, 0); }
        catch (ArithmeticException error)
        { throw new ArgumentException("Force, elastic energy, and tangent stiffness must resolve at maximum compression.", nameof(maximumCompression), error); }
    }

    /// <summary>Evaluates compression (m) and its signed rate (m/s); positive rate is loading.
    /// At zero compression force, energy, and dissipation are zero for every finite rate.</summary>
    public NormalContactResponse Evaluate(double compression, double compressionRate)
    {
        if (!double.IsFinite(compression) || compression < 0 || compression > MaximumCompression)
        { throw new ArgumentOutOfRangeException(nameof(compression), "Compression must lie within the physical law's declared range."); }
        if (!double.IsFinite(compressionRate)) { throw new ArgumentOutOfRangeException(nameof(compressionRate)); }
        if (compression == 0)
        { return new(0, compressionRate, 0, 0, 0, Exponent == 1 ? Coefficient : 0, 0); }

        double power = System.Math.Pow(compression, Exponent);
        double elastic = Coefficient * power;
        if (!(elastic > 0 && double.IsFinite(elastic) && power > 0 && double.IsFinite(power)))
        { elastic = FromLog(System.Math.Log(Coefficient) + Exponent * System.Math.Log(compression)); }
        double stiffness = Scale(elastic, Exponent, compression);
        double energy = Scale(elastic, compression, Exponent + 1);
        double force = elastic;
        double loss = 0;
        if (Dissipation > 0 && compressionRate != 0)
        {
            double speed = System.Math.Abs(compressionRate);
            // Clipped unloading has F=0 and loses stored elastic energy at Fe*|deltaDot|.
            if (compressionRate < 0 && speed >= 1 / Dissipation)
            {
                force = 0;
                loss = Scale(elastic, speed);
            }
            else
            {
                double dampingForce = Scale(Scale(elastic, Dissipation), speed);
                force = compressionRate > 0 ? elastic + dampingForce : System.Math.Max(0, elastic - dampingForce);
                loss = Scale(dampingForce, speed);
            }
        }
        if (!double.IsFinite(force)) { throw new ArithmeticException("Normal force exceeds finite double range."); }
        return new(compression, compressionRate, elastic, force, energy, stiffness, loss);
    }

    private static double Scale(double value, double factor, double divisor = 1)
    {
        double result = value * (factor / divisor);
        return result > 0 && double.IsFinite(result) ? result
            : FromLog(System.Math.Log(value) + System.Math.Log(factor) - System.Math.Log(divisor));
    }

    private static double FromLog(double logarithm)
    {
        double result = System.Math.Exp(logarithm);
        if (!(result > 0 && double.IsFinite(result)))
        { throw new ArithmeticException("A positive contact-law quantity overflows or underflows double precision."); }
        return result;
    }
}

/// <summary>Instantaneous values: compression m, rate m/s, forces N, energy J,
/// elastic tangent stiffness N/m, and nonnegative physical dissipation power W.</summary>
public readonly record struct NormalContactResponse(double Compression, double CompressionRate,
    double ElasticForce, double NormalForce, double ElasticEnergy, double ElasticTangentStiffness,
    double DissipationPower);
