using System;
using QuantumDynamix.Math.Functions;
using QuantumDynamix.Math.Numerics;
using QuantumDynamix.Math.Timelines;

namespace QuantumDynamix.Track;

internal sealed class TrajectoryWork(ProfileDomain domain, int maximumDepth, int maximumWork)
{
    internal int MaximumDepth => maximumDepth;
    private int Used { get; set; }
    private int Remaining => maximumWork - Used;

    internal void Charge(double start, double end, int regionIndex, bool checkingPositivity = false)
    {
        if (Remaining == 0)
        {
            TrajectoryFailureCode code = checkingPositivity ? TrajectoryFailureCode.SpeedPositivityUnresolved
                : TrajectoryFailureCode.NumericalBudgetExceeded;
            throw Failure(code, start, end, regionIndex, "The trajectory work limit was exhausted.");
        }
        Used++;
    }

    internal ScalarJet Evaluate(double coordinate, int regionIndex, Func<ScalarJet> evaluate, bool checkingPositivity = false)
    {
        Charge(coordinate, coordinate, regionIndex, checkingPositivity);
        return EvaluateCharged(coordinate, regionIndex, evaluate);
    }

    internal ScalarJet EvaluateCharged(double coordinate, int regionIndex, Func<ScalarJet> evaluate)
    {
        try { return evaluate(); }
        catch (Exception exception) when (exception is not OutOfMemoryException)
        {
            throw Failure(TrajectoryFailureCode.EvaluationFailed, coordinate, coordinate, regionIndex,
                "The authored acceleration failed to evaluate.", inner: exception);
        }
    }

    internal QuadratureResult Integrate(TrajectoryPanel panel, double end, double absoluteTolerance,
        Func<double, double> integrand)
    {
        if (Remaining < 15 || !(absoluteTolerance > 0) || !double.IsFinite(absoluteTolerance))
        {
            throw Failure(TrajectoryFailureCode.NumericalBudgetExceeded, panel.Start, end, panel.RegionIndex,
                "The remaining work or allocated absolute tolerance cannot support quadrature on this interval.");
        }

        try
        {
            return AdaptiveGaussKronrod.Integrate(coordinate =>
            {
                Charge(panel.Start, end, panel.RegionIndex);
                return integrand(coordinate);
            }, panel.Start, end, new QuadratureSettings(absoluteTolerance, 0, maximumDepth, Remaining));
        }
        catch (NumericalConvergenceException exception)
        {
            throw Failure(TrajectoryFailureCode.NumericalBudgetExceeded, panel.Start, end, panel.RegionIndex,
                "Quadrature exhausted its subdivision or evaluation budget before meeting the allocated accuracy.", inner: exception);
        }
        catch (ArithmeticException exception)
        {
            throw Failure(TrajectoryFailureCode.NumericalRangeExceeded, panel.Start, end, panel.RegionIndex,
                "Quadrature arithmetic exceeded finite double range.", inner: exception);
        }
    }

    internal TrajectoryException Failure(TrajectoryFailureCode code, TrajectoryPanel panel, string message,
        (double Lower, double Upper)? enclosure = null, Exception? inner = null) =>
        Failure(code, panel.Start, panel.End, panel.RegionIndex, message, enclosure, inner: inner);

    internal TrajectoryException Failure(TrajectoryFailureCode code, double start, double end, int regionIndex, string message,
        (double Lower, double Upper)? enclosure = null, TrajectoryErrorEstimate? error = null, Exception? inner = null) =>
        new(code, message, domain, start, end, Used, regionIndex, enclosure, error, innerException: inner);
}
