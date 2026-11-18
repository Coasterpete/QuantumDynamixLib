using System;
using System.Collections.Generic;
using System.Linq;
using QuantumDynamix.Math;

namespace QuantumDynamix.Physics;

// Dense dual active-set solve for min 1/2 (u-free)^T M (u-free), equality rows first,
// remaining rows lower <= J*u. There is no diagonal regularization or force redistribution.
internal sealed class ContactConstraintSolve
{
    internal double[] Values { get; }
    internal double[] Multipliers { get; }
    internal int Iterations { get; }
    internal double FeasibilityError { get; }
    internal double NegativeImpulse { get; }
    internal double ComplementarityError { get; }
    internal double LinearError { get; }
    internal double MomentumError { get; }
    internal double MinimumPivot { get; }

    internal ContactConstraintSolve(MultibodySystem system, double[,] rows, int equalities, double[] lower,
        double[] free, ContactSolverSettings settings, double feasibilityTolerance, double dualTolerance,
        string stage, bool checkReactionRank = false)
    {
        int count = rows.GetLength(0);
        int dimension = free.Length;
        double[,] response = new double[dimension, count];
        for (int body = 0; body < system.Bodies.Count; body++)
        {
            for (int row = 0; row < count; row++)
            {
                Vector3d linear = GetRow(rows, row, 6 * body) / system.Bodies[body].Mass;
                Vector3d angular = system.Bodies[body].SolveBodyInertia(GetRow(rows, row, 6 * body + 3));
                PutColumn(response, 6 * body, row, linear);
                PutColumn(response, 6 * body + 3, row, angular);
            }
        }
        double[,] w = new double[count, count];
        for (int i = 0; i < count; i++)
        {
            for (int j = 0; j <= i; j++)
            {
                double value = 0;
                for (int k = 0; k < dimension; k++) { value += rows[i, k] * response[k, j]; }
                w[i, j] = value;
                w[j, i] = value;
            }
        }
        double[] offset = ConstrainedLinearAlgebra.Multiply(rows, free);
        for (int i = 0; i < count; i++) { offset[i] -= lower[i]; }
        double[] p = new double[count];
        var active = new SortedSet<int>();
        double linearError = 0;
        double minimumPivot = 1;
        int iteration = 0;
        while (true)
        {
            if (++iteration > settings.MaximumActiveSetIterations)
            { throw new ContactSolveException(ContactSolveFailure.ActiveSetLimit, stage, "Active-set budget exhausted.", iterations: iteration - 1); }
            int[] selected = Enumerable.Range(0, equalities).Concat(active).ToArray();
            double[] candidate = Solve(selected);
            int blocker = -1;
            double fraction = 1;
            foreach (int row in active)
            {
                if (candidate[row] < 0)
                {
                    double amount = p[row] / (p[row] - candidate[row]);
                    if (amount < fraction || amount == fraction && (blocker < 0 || row < blocker))
                    { fraction = amount; blocker = row; }
                }
            }
            if (blocker >= 0)
            {
                for (int i = 0; i < count; i++) { p[i] += fraction * (candidate[i] - p[i]); }
                p[blocker] = 0;
                active.Remove(blocker);
                continue;
            }
            p = candidate;
            double[] slack = ConstrainedLinearAlgebra.Multiply(w, p);
            int entering = -1;
            double violation = feasibilityTolerance;
            for (int i = equalities; i < count; i++)
            {
                slack[i] += offset[i];
                if (!active.Contains(i) && -slack[i] > violation)
                { entering = i; violation = -slack[i]; }
            }
            if (entering >= 0) { active.Add(entering); continue; }
            break;
        }
        double[] change = ConstrainedLinearAlgebra.Multiply(response, p);
        double[] values = new double[dimension];
        for (int i = 0; i < dimension; i++) { values[i] = free[i] + change[i]; }
        double[] errors = ConstrainedLinearAlgebra.Multiply(rows, values);
        double feasibility = 0;
        double negative = 0;
        double complementarity = 0;
        var tight = new List<int>(Enumerable.Range(0, equalities));
        double pScale = System.Math.Max(dualTolerance, ConstrainedLinearAlgebra.MaxAbs(p));
        double vScale = System.Math.Max(feasibilityTolerance, ConstrainedLinearAlgebra.MaxAbs(offset));
        for (int i = 0; i < count; i++)
        {
            errors[i] -= lower[i];
            feasibility = System.Math.Max(feasibility, i < equalities ? System.Math.Abs(errors[i]) : -errors[i]);
            if (i >= equalities)
            {
                negative = System.Math.Max(negative, -p[i]);
                complementarity = System.Math.Max(complementarity, System.Math.Abs((p[i] / pScale) * (errors[i] / vScale)));
                if (p[i] > dualTolerance && System.Math.Abs(errors[i]) > feasibilityTolerance)
                { throw new ContactSolveException(ContactSolveFailure.LinearSolve, stage, "Active contact fails complementarity feasibility (stage units).", residual: errors[i]); }
                if (System.Math.Abs(errors[i]) <= feasibilityTolerance) { tight.Add(i); }
            }
        }
        if (feasibility > feasibilityTolerance || negative > dualTolerance)
        { throw new ContactSolveException(ContactSolveFailure.LinearSolve, stage, "Discrete constraint feasibility failed (stage units).", residual: feasibility); }
        // A conservative rank policy: every tight potential support must be independently resolved.
        // Do not publish an arbitrary basis-dependent distribution among redundant rigid contacts.
        if (checkReactionRank) { Factor(tight.ToArray()); }
        double momentumError = 0;
        for (int body = 0; body < system.Bodies.Count; body++)
        {
            Vector3d linear = (ConstraintAssembly.Get(values, 6 * body) - ConstraintAssembly.Get(free, 6 * body)) * system.Bodies[body].Mass;
            Vector3d angular = system.Bodies[body].BodyInertia
                * (ConstraintAssembly.Get(values, 6 * body + 3) - ConstraintAssembly.Get(free, 6 * body + 3));
            Vector3d expectedLinear = default;
            Vector3d expectedAngular = default;
            double linearScale = system.Bodies[body].Mass * (ConstraintAssembly.Get(values, 6 * body).Length
                + ConstraintAssembly.Get(free, 6 * body).Length);
            double angularScale = (system.Bodies[body].BodyInertia * ConstraintAssembly.Get(values, 6 * body + 3)).Length
                + (system.Bodies[body].BodyInertia * ConstraintAssembly.Get(free, 6 * body + 3)).Length;
            for (int row = 0; row < count; row++)
            {
                expectedLinear += GetRow(rows, row, 6 * body) * p[row];
                expectedAngular += GetRow(rows, row, 6 * body + 3) * p[row];
                linearScale += (GetRow(rows, row, 6 * body) * p[row]).Length;
                angularScale += (GetRow(rows, row, 6 * body + 3) * p[row]).Length;
            }
            momentumError = System.Math.Max(momentumError, RelativeError(linear, expectedLinear, linearScale));
            momentumError = System.Math.Max(momentumError, RelativeError(angular, expectedAngular, angularScale));
        }
        if (momentumError > settings.Joints.LinearSolveTolerance)
        { throw new ContactSolveException(ContactSolveFailure.LinearSolve, stage, "Velocity increments fail relative impulse/momentum balance.", residual: momentumError); }
        Values = values;
        Multipliers = p;
        Iterations = iteration;
        FeasibilityError = feasibility;
        NegativeImpulse = negative;
        ComplementarityError = complementarity;
        LinearError = linearError;
        MomentumError = momentumError;
        MinimumPivot = minimumPivot;

        ConstraintCholesky Factor(int[] indices)
        {
            double[,] matrix = new double[indices.Length, indices.Length];
            for (int i = 0; i < indices.Length; i++)
            { for (int j = 0; j < indices.Length; j++) { matrix[i, j] = w[indices[i], indices[j]]; } }
            try
            {
                var factor = new ConstraintCholesky(matrix, settings.Joints, stage);
                minimumPivot = System.Math.Min(minimumPivot, factor.MinimumPivot);
                return factor;
            }
            catch (ConstrainedSolveException error)
            {
                throw new ContactSolveException(ContactSolveFailure.IndeterminateReactions, stage,
                    $"Required/tight rows [{string.Join(",", indices)}] are dependent or numerically unresolved; no load distribution or compliance was added.",
                    iterations: iteration, residual: error.Residual, inner: error);
            }
        }

        double[] Solve(int[] indices)
        {
            ConstraintCholesky factor = Factor(indices);
            double[] rhs = indices.Select(i => -offset[i]).ToArray();
            double[] solution;
            try { solution = factor.Solve(rhs, out double error); linearError = System.Math.Max(linearError, error); }
            catch (ConstrainedSolveException error)
            { throw new ContactSolveException(ContactSolveFailure.LinearSolve, stage, "Scaled linear solve failed.", iterations: iteration, residual: error.Residual, inner: error); }
            double[] result = new double[count];
            for (int i = 0; i < indices.Length; i++) { result[indices[i]] = solution[i]; }
            return result;
        }
    }

    private static double RelativeError(Vector3d actual, Vector3d expected, double scale)
    {
        if (!double.IsFinite(scale)) { throw new ArithmeticException("Momentum residual scale exceeds finite range."); }
        return scale == 0 ? 0 : ((actual - expected) / scale).Length;
    }

    internal static Vector3d GetRow(double[,] matrix, int row, int offset) => new(matrix[row, offset], matrix[row, offset + 1], matrix[row, offset + 2]);
    internal static void PutRow(double[,] matrix, int row, int offset, Vector3d value)
    { matrix[row, offset] = value.X; matrix[row, offset + 1] = value.Y; matrix[row, offset + 2] = value.Z; }
    private static void PutColumn(double[,] matrix, int offset, int column, Vector3d value)
    { matrix[offset, column] = value.X; matrix[offset + 1, column] = value.Y; matrix[offset + 2, column] = value.Z; }
}
