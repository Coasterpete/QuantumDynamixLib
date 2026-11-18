using System;
using System.Collections.Generic;
using System.Linq;

namespace QuantumDynamix.Math.Numerics;

internal readonly record struct LinearEquation(double[] Row, double Target);
internal readonly record struct LeastSquaresSolution(double[]? Values, bool Infeasible, int Rank, int ConstraintRank,
    double Residual, double ConstraintResidual, string Message);

/// <summary>Equality constraints are eliminated with rank-revealing QR of C^T, then the reduced
/// least-squares problem is solved by pivoted Householder QR. No normal equations are formed.</summary>
internal static class ConstrainedLeastSquares
{
    internal static LeastSquaresSolution Solve(double[,] matrix, double[] rhs, IReadOnlyList<LinearEquation> constraints,
        IReadOnlyDictionary<int, double>? fixedValues = null, double tolerance = 1e-9)
    {
        int rows = matrix.GetLength(0);
        int count = matrix.GetLength(1);
        fixedValues ??= new Dictionary<int, double>();
        if (fixedValues.Any(p => p.Key < 0 || p.Key >= count || !double.IsFinite(p.Value)))
        { throw new ArgumentException("Invalid locked coefficient.", nameof(fixedValues)); }
        int[] free = Enumerable.Range(0, count).Where(i => !fixedValues.ContainsKey(i)).ToArray();
        var equations = new List<LinearEquation>();
        foreach (LinearEquation equation in constraints)
        {
            double target = equation.Target;
            foreach (var locked in fixedValues) { target -= equation.Row[locked.Key] * locked.Value; }
            double[] row = free.Select(i => equation.Row[i]).ToArray();
            double norm = Norm(row);
            if (norm == 0)
            {
                if (System.Math.Abs(target) > tolerance * System.Math.Max(1, System.Math.Abs(equation.Target)))
                { return new(null, true, 0, 0, 0, System.Math.Abs(target), "A constraint conflicts with locked coefficients."); }
            }
            else
            {
                if (!double.IsFinite(target / norm)) { throw new ArithmeticException("Normalized constraint exceeds finite range."); }
                equations.Add(new(row.Select(x => x / norm).ToArray(), target / norm));
            }
        }
        int n = free.Length;
        double[,] transpose = new double[n, equations.Count];
        for (int j = 0; j < equations.Count; j++) { for (int i = 0; i < n; i++) { transpose[i, j] = equations[j].Row[i]; } }
        var constraintQr = new PivotedQr(transpose);
        int rank = constraintQr.Rank;
        double[] y = new double[rank];
        for (int i = 0; i < rank; i++)
        {
            double value = equations[constraintQr.Permutation[i]].Target;
            for (int j = 0; j < i; j++) { value -= constraintQr.R[j, i] * y[j]; }
            y[i] = value / constraintQr.R[i, i];
            if (!double.IsFinite(y[i])) { throw new ArithmeticException("Constraint elimination exceeds finite range."); }
        }
        double[] particular = new double[n];
        for (int i = 0; i < n; i++) { for (int j = 0; j < rank; j++) { particular[i] += constraintQr.Q[i, j] * y[j]; } }
        foreach (LinearEquation equation in equations)
        {
            double error = System.Math.Abs(Dot(equation.Row, particular) - equation.Target);
            if (error > tolerance * System.Math.Max(1, System.Math.Abs(equation.Target)))
            { return new(null, true, 0, rank, 0, error, "Dependent equality constraints are inconsistent."); }
        }
        int nullity = n - rank;
        double[,] reduced = new double[rows, nullity];
        double[] reducedRhs = new double[rows];
        for (int row = 0; row < rows; row++)
        {
            double target = rhs[row];
            foreach (var locked in fixedValues) { target -= matrix[row, locked.Key] * locked.Value; }
            for (int i = 0; i < n; i++) { target -= matrix[row, free[i]] * particular[i]; }
            reducedRhs[row] = target;
            for (int j = 0; j < nullity; j++)
            {
                for (int i = 0; i < n; i++) { reduced[row, j] += matrix[row, free[i]] * constraintQr.Q[i, rank + j]; }
            }
        }
        var qr = new PivotedQr(reduced, false);
        if (qr.Rank != nullity) { return new(null, false, qr.Rank, rank, 0, 0, "The data and constraints do not determine a unique, numerically resolved fit."); }
        double[] solution = qr.Solve(reducedRhs);
        double[] values = new double[count];
        foreach (var locked in fixedValues) { values[locked.Key] = locked.Value; }
        for (int i = 0; i < n; i++)
        {
            double value = particular[i];
            for (int j = 0; j < nullity; j++) { value += constraintQr.Q[i, rank + j] * solution[j]; }
            if (!double.IsFinite(value)) { throw new ArithmeticException("Fitting solution exceeds finite range."); }
            values[free[i]] = value;
        }
        double maximum = 0;
        foreach (LinearEquation equation in constraints)
        {
            double residual = System.Math.Abs(Dot(equation.Row, values) - equation.Target);
            double normalized = residual / System.Math.Max(1, System.Math.Abs(equation.Target));
            maximum = System.Math.Max(maximum, normalized);
        }
        if (maximum > tolerance) { return new(null, true, qr.Rank, rank, 0, maximum, "Solved coefficients fail the requested constraint residual tolerance."); }
        double[] residuals = new double[rows];
        for (int i = 0; i < rows; i++)
        {
            double value = -rhs[i];
            for (int j = 0; j < count; j++) { value += matrix[i, j] * values[j]; }
            residuals[i] = value;
        }
        return new(values, false, qr.Rank, rank, Norm(residuals), maximum, "Constrained least squares converged.");
    }

    internal static double Dot(double[] a, double[] b)
    {
        double value = 0;
        for (int i = 0; i < a.Length; i++) { value += a[i] * b[i]; }
        if (!double.IsFinite(value)) { throw new ArithmeticException("Least-squares dot product exceeds finite range."); }
        return value;
    }

    internal static double Norm(IEnumerable<double> values)
    {
        double scale = 0;
        double sum = 1;
        foreach (double value in values)
        {
            if (!double.IsFinite(value)) { throw new ArithmeticException("Least-squares data are not finite."); }
            double absolute = System.Math.Abs(value);
            if (absolute == 0) { continue; }
            if (scale < absolute) { sum = 1 + sum * (scale / absolute) * (scale / absolute); scale = absolute; }
            else { sum += (absolute / scale) * (absolute / scale); }
        }
        double result = scale == 0 ? 0 : scale * System.Math.Sqrt(sum);
        if (!double.IsFinite(result)) { throw new ArithmeticException("Least-squares norm exceeds finite range."); }
        return result;
    }

    private sealed class PivotedQr
    {
        internal double[,] R { get; }
        internal double[,] Q { get; }
        internal int[] Permutation { get; }
        internal int Rank { get; }
        private readonly List<double[]> reflectors = new();
        internal PivotedQr(double[,] matrix, bool formOrthogonal = true)
        {
            int m = matrix.GetLength(0);
            int n = matrix.GetLength(1);
            R = (double[,])matrix.Clone();
            Q = formOrthogonal ? new double[m, m] : new double[0, 0];
            if (formOrthogonal) { for (int i = 0; i < m; i++) { Q[i, i] = 1; } }
            Permutation = Enumerable.Range(0, n).ToArray();
            double initial = 0;
            int rank = 0;
            for (int k = 0; k < System.Math.Min(m, n); k++)
            {
                int best = k;
                double largest = -1;
                for (int j = k; j < n; j++)
                {
                    double norm = Norm(Enumerable.Range(k, m - k).Select(i => R[i, j]));
                    if (norm > largest) { largest = norm; best = j; }
                }
                if (k == 0) { initial = largest; }
                if (largest == 0 || largest <= 1e-12 * initial) { break; }
                if (best != k)
                {
                    for (int i = 0; i < m; i++) { (R[i, k], R[i, best]) = (R[i, best], R[i, k]); }
                    (Permutation[k], Permutation[best]) = (Permutation[best], Permutation[k]);
                }
                double[] v = Enumerable.Range(k, m - k).Select(i => R[i, k] / largest).ToArray();
                v[0] += v[0] >= 0 ? 1 : -1;
                double normV = Norm(v);
                for (int i = 0; i < v.Length; i++) { v[i] /= normV; }
                reflectors.Add(v);
                for (int j = k; j < n; j++)
                {
                    double dot = 0;
                    for (int i = k; i < m; i++) { dot += v[i - k] * R[i, j]; }
                    for (int i = k; i < m; i++) { R[i, j] -= 2 * v[i - k] * dot; }
                }
                for (int i = 0; formOrthogonal && i < m; i++)
                {
                    double dot = 0;
                    for (int j = k; j < m; j++) { dot += Q[i, j] * v[j - k]; }
                    for (int j = k; j < m; j++) { Q[i, j] -= 2 * dot * v[j - k]; }
                }
                rank++;
            }
            Rank = rank;
        }

        internal double[] Solve(double[] rhs)
        {
            int n = R.GetLength(1);
            double[] transformed = (double[])rhs.Clone();
            for (int k = 0; k < reflectors.Count; k++)
            {
                double[] v = reflectors[k];
                double dot = 0;
                for (int j = k; j < transformed.Length; j++) { dot += v[j - k] * transformed[j]; }
                for (int j = k; j < transformed.Length; j++) { transformed[j] -= 2 * dot * v[j - k]; }
            }
            double[] permuted = new double[n];
            for (int i = n - 1; i >= 0; i--)
            {
                double value = transformed[i];
                for (int j = i + 1; j < n; j++) { value -= R[i, j] * permuted[j]; }
                permuted[i] = value / R[i, i];
            }
            double[] result = new double[n];
            for (int i = 0; i < n; i++) { result[Permutation[i]] = permuted[i]; }
            return result;
        }
    }
}
