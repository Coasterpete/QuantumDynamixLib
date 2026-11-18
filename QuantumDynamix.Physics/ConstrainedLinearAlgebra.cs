using System;

namespace QuantumDynamix.Physics;

// Deliberately internal: joint row assembly does not expose a general-purpose public matrix API.
internal static class ConstrainedLinearAlgebra
{
    internal static double MaxAbs(double[] values)
    {
        double norm = 0;
        foreach (double value in values)
        {
            if (!double.IsFinite(value))
            {
                throw new ArithmeticException("A constrained calculation exceeded the finite double range.");
            }

            norm = System.Math.Max(norm, System.Math.Abs(value));
        }

        return norm;
    }

    internal static double[] Multiply(double[,] matrix, double[] vector)
    {
        double[] result = new double[matrix.GetLength(0)];
        for (int row = 0; row < result.Length; row++)
        {
            for (int column = 0; column < vector.Length; column++)
            {
                result[row] += matrix[row, column] * vector[column];
            }
        }

        MaxAbs(result);
        return result;
    }

    internal static double BackwardError(double[,] matrix, double[] solution, double[] rhs)
    {
        double xNorm = MaxAbs(solution);
        double bNorm = MaxAbs(rhs);
        double scale = System.Math.Max(xNorm, bNorm);
        if (scale == 0)
        {
            return 0;
        }

        double matrixNorm = 0;
        double residualNorm = 0;
        for (int i = 0; i < rhs.Length; i++)
        {
            double sum = 0;
            double residual = -rhs[i] / scale;
            for (int j = 0; j < solution.Length; j++)
            {
                sum += System.Math.Abs(matrix[i, j]);
                residual += matrix[i, j] * (solution[j] / scale);
            }

            matrixNorm = System.Math.Max(matrixNorm, sum);
            residualNorm = System.Math.Max(residualNorm, System.Math.Abs(residual));
        }

        double error = residualNorm / (matrixNorm * (xNorm / scale) + bNorm / scale);
        if (!double.IsFinite(error))
        {
            throw new ArithmeticException("Linear residual is not representable.");
        }

        return error;
    }

    // Row scaling and partial pivoting for the dimensionless midpoint Newton equations.
    internal static double[] SolveNewton(double[,] matrix, double[] rhs, ConstrainedSolverSettings settings, int iteration)
    {
        int n = rhs.Length;
        double[,] a = (double[,])matrix.Clone();
        double[] b = (double[])rhs.Clone();
        for (int i = 0; i < n; i++)
        {
            double scale = 0;
            for (int j = 0; j < n; j++)
            {
                scale = System.Math.Max(scale, System.Math.Abs(a[i, j]));
            }

            if (!double.IsFinite(scale) || scale == 0)
            {
                throw new ConstrainedSolveException(ConstrainedSolveFailure.LinearSolve, "Midpoint Newton matrix",
                    iteration, scale, "Zero or nonfinite row scale.");
            }

            b[i] /= scale;
            for (int j = 0; j < n; j++)
            {
                a[i, j] /= scale;
            }
        }

        for (int k = 0; k < n; k++)
        {
            int pivotRow = k;
            for (int i = k + 1; i < n; i++)
            {
                if (System.Math.Abs(a[i, k]) > System.Math.Abs(a[pivotRow, k]))
                {
                    pivotRow = i;
                }
            }

            double pivot = System.Math.Abs(a[pivotRow, k]);
            if (!double.IsFinite(pivot) || pivot <= settings.MinimumScaledPivot)
            {
                throw new ConstrainedSolveException(ConstrainedSolveFailure.LinearSolve, "Midpoint Newton matrix",
                    iteration, pivot, "Numerically singular scaled LU pivot.");
            }

            if (pivotRow != k)
            {
                for (int j = 0; j < n; j++)
                {
                    (a[k, j], a[pivotRow, j]) = (a[pivotRow, j], a[k, j]);
                }

                (b[k], b[pivotRow]) = (b[pivotRow], b[k]);
            }

            for (int i = k + 1; i < n; i++)
            {
                double multiplier = a[i, k] / a[k, k];
                for (int j = k + 1; j < n; j++)
                {
                    a[i, j] -= multiplier * a[k, j];
                }

                b[i] -= multiplier * b[k];
            }
        }

        double[] solution = new double[n];
        for (int i = n - 1; i >= 0; i--)
        {
            double value = b[i];
            for (int j = i + 1; j < n; j++)
            {
                value -= a[i, j] * solution[j];
            }

            solution[i] = value / a[i, i];
        }

        double error = BackwardError(matrix, solution, rhs);
        if (error > settings.LinearSolveTolerance)
        {
            throw new ConstrainedSolveException(ConstrainedSolveFailure.LinearSolve, "Midpoint Newton matrix",
                iteration, error, "Scaled normwise backward error exceeds tolerance.");
        }

        return solution;
    }
}

internal sealed class ConstraintCholesky
{
    private readonly double[,] scaled;
    private readonly double[,] lower;
    private readonly double[] diagonalScale;
    private readonly ConstrainedSolverSettings settings;
    private readonly string stage;

    internal double MinimumPivot { get; }

    internal ConstraintCholesky(double[,] matrix, ConstrainedSolverSettings settings, string stage)
    {
        this.settings = settings;
        this.stage = stage;
        int n = matrix.GetLength(0);
        diagonalScale = new double[n];
        scaled = new double[n, n];
        lower = new double[n, n];
        for (int i = 0; i < n; i++)
        {
            double diagonal = matrix[i, i];
            if (!double.IsFinite(diagonal) || diagonal <= 0)
            {
                throw Singular(diagonal);
            }

            diagonalScale[i] = System.Math.Sqrt(diagonal);
        }

        double minimumPivot = 1;
        for (int i = 0; i < n; i++)
        {
            for (int j = 0; j <= i; j++)
            {
                // Sequential division avoids overflow/underflow in a product of scale factors.
                double entry = i == j ? 1 : (matrix[i, j] / diagonalScale[i]) / diagonalScale[j];
                scaled[i, j] = entry;
                scaled[j, i] = entry;
                double value = entry;
                for (int k = 0; k < j; k++)
                {
                    value -= lower[i, k] * lower[j, k];
                }

                if (i == j)
                {
                    if (!double.IsFinite(value) || value <= settings.MinimumScaledPivot)
                    {
                        throw Singular(value);
                    }

                    minimumPivot = System.Math.Min(minimumPivot, value);
                    lower[i, j] = System.Math.Sqrt(value);
                }
                else
                {
                    lower[i, j] = value / lower[j, j];
                }
            }
        }

        MinimumPivot = minimumPivot;
    }

    internal double[] Solve(double[] rhs, out double error)
    {
        int n = rhs.Length;
        double[] b = new double[n];
        double[] z = new double[n];
        for (int i = 0; i < n; i++)
        {
            b[i] = rhs[i] / diagonalScale[i];
            double value = b[i];
            for (int j = 0; j < i; j++)
            {
                value -= lower[i, j] * z[j];
            }

            z[i] = value / lower[i, i];
        }

        for (int i = n - 1; i >= 0; i--)
        {
            double value = z[i];
            for (int j = i + 1; j < n; j++)
            {
                value -= lower[j, i] * z[j];
            }

            z[i] = value / lower[i, i];
        }

        error = ConstrainedLinearAlgebra.BackwardError(scaled, z, b);
        if (error > settings.LinearSolveTolerance)
        {
            throw new ConstrainedSolveException(ConstrainedSolveFailure.LinearSolve, stage, 0, error,
                "Constraint solve scaled backward error exceeds tolerance.");
        }

        double[] solution = new double[n];
        for (int i = 0; i < n; i++)
        {
            solution[i] = z[i] / diagonalScale[i];
        }

        ConstrainedLinearAlgebra.MaxAbs(solution);
        return solution;
    }

    private ConstrainedSolveException Singular(double pivot) => new(
        ConstrainedSolveFailure.SingularConstraints, stage, 0, pivot,
        "Constraint rows are redundant, singular, or numerically ill-conditioned; no compliance was added.");
}
