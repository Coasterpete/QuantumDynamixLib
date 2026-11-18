using System;

namespace QuantumDynamix.Physics;

/// <summary>Immutable provisional numerical controls; these do not establish application accuracy requirements.</summary>
/// <remarks>Position/velocity tolerances bound each joint's Euclidean error in m and m/s. Newton uses a
/// dimensionless infinity norm, scaled per body by the maximum of its velocity scale floor, initial speed,
/// and predicted velocity change. Linear tolerance is a scaled normwise backward error. MinimumScaledPivot
/// bounds Cholesky pivots after unit-diagonal scaling and relative LU pivots after row scaling.</remarks>
public sealed class ConstrainedSolverSettings
{
    public static ConstrainedSolverSettings Default { get; } = new();

    public double PositionTolerance { get; }
    public double VelocityTolerance { get; }
    public double NonlinearTolerance { get; }
    public double LinearSolveTolerance { get; }
    public double MinimumScaledPivot { get; }
    public double LinearVelocityScale { get; }
    public double AngularVelocityScale { get; }
    public double FiniteDifferenceStep { get; }
    public int MaxNonlinearIterations { get; }
    public int MaxProjectionIterations { get; }
    public int MaxLineSearchIterations { get; }

    public ConstrainedSolverSettings(
        double positionTolerance = 1e-10,
        double velocityTolerance = 1e-10,
        double nonlinearTolerance = 1e-11,
        double linearSolveTolerance = 1e-11,
        double minimumScaledPivot = 1e-12,
        double linearVelocityScale = 1,
        double angularVelocityScale = 1,
        double finiteDifferenceStep = 6e-6,
        int maxNonlinearIterations = 20,
        int maxProjectionIterations = 12,
        int maxLineSearchIterations = 12)
    {
        RequirePositive(positionTolerance, nameof(positionTolerance));
        RequirePositive(velocityTolerance, nameof(velocityTolerance));
        RequirePositive(nonlinearTolerance, nameof(nonlinearTolerance));
        RequirePositive(linearSolveTolerance, nameof(linearSolveTolerance));
        RequirePositive(minimumScaledPivot, nameof(minimumScaledPivot));
        RequirePositive(linearVelocityScale, nameof(linearVelocityScale));
        RequirePositive(angularVelocityScale, nameof(angularVelocityScale));
        RequirePositive(finiteDifferenceStep, nameof(finiteDifferenceStep));
        if (minimumScaledPivot >= 1)
        {
            throw new ArgumentOutOfRangeException(nameof(minimumScaledPivot), "The scaled pivot floor must be below one.");
        }

        if (finiteDifferenceStep >= 1)
        {
            throw new ArgumentOutOfRangeException(nameof(finiteDifferenceStep), "The relative difference step must be below one.");
        }

        ArgumentOutOfRangeException.ThrowIfNegativeOrZero(maxNonlinearIterations);
        ArgumentOutOfRangeException.ThrowIfNegativeOrZero(maxProjectionIterations);
        ArgumentOutOfRangeException.ThrowIfNegativeOrZero(maxLineSearchIterations);
        PositionTolerance = positionTolerance;
        VelocityTolerance = velocityTolerance;
        NonlinearTolerance = nonlinearTolerance;
        LinearSolveTolerance = linearSolveTolerance;
        MinimumScaledPivot = minimumScaledPivot;
        LinearVelocityScale = linearVelocityScale;
        AngularVelocityScale = angularVelocityScale;
        FiniteDifferenceStep = finiteDifferenceStep;
        MaxNonlinearIterations = maxNonlinearIterations;
        MaxProjectionIterations = maxProjectionIterations;
        MaxLineSearchIterations = maxLineSearchIterations;
    }

    private static void RequirePositive(double value, string name)
    {
        if (!double.IsFinite(value) || value <= 0)
        {
            throw new ArgumentOutOfRangeException(name, "Control must be finite and positive.");
        }
    }
}
