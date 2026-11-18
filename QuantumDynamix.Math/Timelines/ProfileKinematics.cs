using QuantumDynamix.Math.Functions;

namespace QuantumDynamix.Math.Timelines;

/// <summary>Chain-rule conversions using caller-supplied motion. These operations do not integrate or prescribe motion.</summary>
public static class ProfileKinematics
{
    /// <summary>For q(s(t)), converts derivatives per metre into derivatives per second, including signed reversal.</summary>
    public static ScalarJet ToTimeDerivatives(ScalarJet spatial, double stationRate, double stationAcceleration = 0, double stationJerk = 0)
    {
        Check.Finite(stationRate, nameof(stationRate));
        Check.Finite(stationAcceleration, nameof(stationAcceleration));
        Check.Finite(stationJerk, nameof(stationJerk));
        double first = spatial.DerivativeOrder >= 1 ? spatial.FirstDerivative * stationRate : 0;
        double second = spatial.DerivativeOrder >= 2 ? spatial.SecondDerivative * stationRate * stationRate + spatial.FirstDerivative * stationAcceleration : 0;
        double third = spatial.DerivativeOrder >= 3 ? spatial.ThirdDerivative * stationRate * stationRate * stationRate
            + 3 * spatial.SecondDerivative * stationRate * stationAcceleration + spatial.FirstDerivative * stationJerk : 0;
        return new(spatial.Value, first, second, third, spatial.DerivativeOrder);
    }
}
