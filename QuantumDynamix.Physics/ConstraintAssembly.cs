using System;
using QuantumDynamix.Math;

namespace QuantumDynamix.Physics;

// Each joint contributes rows to the same J and bias. Additional constraint types can supply
// their own rows without changing the mass operator, Schur solver, or integration equations.
internal sealed class ConstraintAssembly
{
    internal double[,] Jacobian { get; }
    internal double[,] MassResponse { get; }
    internal double[] PositionError { get; }
    internal double[] VelocityError { get; }
    internal double[] Bias { get; }
    internal ConstraintCholesky Factor { get; }

    internal ConstraintAssembly(MultibodySystem system, MultibodyState state,
        ConstrainedSolverSettings settings, string stage)
    {
        int rows = checked(3 * system.Joints.Count);
        int speeds = checked(6 * system.Bodies.Count);
        Jacobian = new double[rows, speeds];
        MassResponse = new double[speeds, rows];
        PositionError = new double[rows];
        Bias = new double[rows];
        for (int jointIndex = 0; jointIndex < system.Joints.Count; jointIndex++)
        {
            SphericalJoint joint = system.Joints[jointIndex];
            RigidBodyState a = state.Bodies[joint.BodyA];
            RigidBodyState b = state.Bodies[joint.BodyB];
            Put(PositionError, 3 * jointIndex,
                a.Pose.TransformPoint(joint.LocalAnchorA) - b.Pose.TransformPoint(joint.LocalAnchorB));
            Vector3d centripetalA = a.Pose.TransformDirection(Vector3d.Cross(a.BodyAngularVelocity,
                Vector3d.Cross(a.BodyAngularVelocity, joint.LocalAnchorA)));
            Vector3d centripetalB = b.Pose.TransformDirection(Vector3d.Cross(b.BodyAngularVelocity,
                Vector3d.Cross(b.BodyAngularVelocity, joint.LocalAnchorB)));
            Put(Bias, 3 * jointIndex, centripetalA - centripetalB);
            for (int axis = 0; axis < 3; axis++)
            {
                Vector3d direction = axis == 0 ? Vector3d.UnitX : axis == 1 ? Vector3d.UnitY : Vector3d.UnitZ;
                int row = 3 * jointIndex + axis;
                Jacobian[row, 6 * joint.BodyA + axis] = 1;
                Jacobian[row, 6 * joint.BodyB + axis] = -1;
                // J_A=[1,-R_A*[s_A]x], J_B=[-1,R_B*[s_B]x].
                PutRow(Jacobian, row, 6 * joint.BodyA + 3,
                    Vector3d.Cross(joint.LocalAnchorA, a.Pose.InverseTransformDirection(direction)));
                PutRow(Jacobian, row, 6 * joint.BodyB + 3,
                    -Vector3d.Cross(joint.LocalAnchorB, b.Pose.InverseTransformDirection(direction)));
            }
        }

        VelocityError = ConstrainedLinearAlgebra.Multiply(Jacobian, Velocities(state));
        for (int body = 0; body < system.Bodies.Count; body++)
        {
            RigidBodyMassProperties properties = system.Bodies[body];
            for (int row = 0; row < rows; row++)
            {
                Vector3d linear = GetRow(Jacobian, row, 6 * body) / properties.Mass;
                Vector3d angular = properties.SolveBodyInertia(GetRow(Jacobian, row, 6 * body + 3));
                PutColumn(MassResponse, 6 * body, row, linear);
                PutColumn(MassResponse, 6 * body + 3, row, angular);
            }
        }

        double[,] schur = new double[rows, rows];
        for (int i = 0; i < rows; i++)
        {
            for (int j = 0; j <= i; j++)
            {
                double value = 0;
                for (int k = 0; k < speeds; k++)
                {
                    value += Jacobian[i, k] * MassResponse[k, j];
                }

                // Assemble one triangle and mirror; never require floating products to be exactly symmetric.
                schur[i, j] = value;
                schur[j, i] = value;
            }
        }

        Factor = new ConstraintCholesky(schur, settings, stage);
    }

    internal static double MaximumJointError(double[] errors)
    {
        double maximum = 0;
        for (int i = 0; i < errors.Length; i += 3)
        {
            maximum = System.Math.Max(maximum, Get(errors, i).Length);
        }

        return maximum;
    }

    internal static double[] Velocities(MultibodyState state)
    {
        double[] result = new double[6 * state.Bodies.Count];
        for (int body = 0; body < state.Bodies.Count; body++)
        {
            Put(result, 6 * body, state.Bodies[body].LinearVelocity);
            Put(result, 6 * body + 3, state.Bodies[body].BodyAngularVelocity);
        }

        return result;
    }

    internal static Vector3d Get(double[] values, int offset) => new(values[offset], values[offset + 1], values[offset + 2]);

    internal static void Put(double[] values, int offset, Vector3d value)
    {
        values[offset] = value.X;
        values[offset + 1] = value.Y;
        values[offset + 2] = value.Z;
    }

    private static Vector3d GetRow(double[,] matrix, int row, int offset) =>
        new(matrix[row, offset], matrix[row, offset + 1], matrix[row, offset + 2]);

    private static void PutRow(double[,] matrix, int row, int offset, Vector3d value)
    {
        matrix[row, offset] = value.X;
        matrix[row, offset + 1] = value.Y;
        matrix[row, offset + 2] = value.Z;
    }

    private static void PutColumn(double[,] matrix, int offset, int column, Vector3d value)
    {
        matrix[offset, column] = value.X;
        matrix[offset + 1, column] = value.Y;
        matrix[offset + 2, column] = value.Z;
    }
}
