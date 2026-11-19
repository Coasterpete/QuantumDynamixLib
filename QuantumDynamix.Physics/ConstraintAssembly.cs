using System;
using System.Collections.Generic;
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
    internal bool[] AngularRows { get; }
    internal double MaximumAlignmentError { get; }
    private readonly ConstraintRowBlock[] blocks;
    private readonly Vector3d[] coupleDirections;
    private readonly int sphericalRows;

    internal ConstraintAssembly(MultibodySystem system, MultibodyState state,
        ConstrainedSolverSettings settings, string stage)
    {
        int rows = system.BilateralConstraintCount;
        sphericalRows = checked(3 * system.Joints.Count);
        int speeds = checked(6 * system.Bodies.Count);
        Jacobian = new double[rows, speeds];
        MassResponse = new double[speeds, rows];
        PositionError = new double[rows];
        Bias = new double[rows];
        AngularRows = new bool[rows];
        coupleDirections = new Vector3d[2 * system.RevoluteJoints.Count];
        var rowBlocks = new List<ConstraintRowBlock>();
        for (int jointIndex = 0; jointIndex < system.Joints.Count; jointIndex++)
        {
            SphericalJoint joint = system.Joints[jointIndex];
            AddPointRows(3 * jointIndex, joint.BodyA, joint.BodyB, joint.LocalAnchorA, joint.LocalAnchorB);
        }
        double maximumAlignment = 0;
        for (int j = 0; j < system.RevoluteJoints.Count; j++)
        {
            RevoluteJoint joint = system.RevoluteJoints[j];
            int offset = RevoluteRow(j);
            AddPointRows(offset, joint.BodyA, joint.BodyB, joint.LocalFrameA.Position, joint.LocalFrameB.Position);
            RevoluteFrames frames = new(joint, state, stage);
            maximumAlignment = System.Math.Max(maximumAlignment, frames.AlignmentError);
            Vector3d omegaA = state.Bodies[joint.BodyA].WorldAngularVelocity;
            Vector3d omegaB = state.Bodies[joint.BodyB].WorldAngularVelocity;
            rowBlocks.Add(new(offset + 3, true));
            for (int i = 0; i < 2; i++)
            {
                int row = offset + 3 + i;
                AngularRows[row] = true;
                Vector3d transverse = i == 0 ? frames.YA : frames.ZA;
                Vector3d direction = Vector3d.Cross(transverse, frames.AxisB);
                coupleDirections[2 * j + i] = direction;
                PositionError[row] = Vector3d.Dot(transverse, frames.AxisB);
                PutRow(Jacobian, row, 6 * joint.BodyA + 3, state.Bodies[joint.BodyA].Pose.InverseTransformDirection(direction));
                PutRow(Jacobian, row, 6 * joint.BodyB + 3, -state.Bodies[joint.BodyB].Pose.InverseTransformDirection(direction));
                // d^2(t_A dot a_B)/dt^2 = J*uDot + b. Both directions rotate.
                Bias[row] = Vector3d.Dot(Vector3d.Cross(omegaA, Vector3d.Cross(omegaA, transverse)), frames.AxisB)
                    + 2 * Vector3d.Dot(Vector3d.Cross(omegaA, transverse), Vector3d.Cross(omegaB, frames.AxisB))
                    + Vector3d.Dot(transverse, Vector3d.Cross(omegaB, Vector3d.Cross(omegaB, frames.AxisB)));
            }
        }
        blocks = rowBlocks.ToArray();
        MaximumAlignmentError = maximumAlignment;

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

        void AddPointRows(int offset, int bodyA, int bodyB, Vector3d anchorA, Vector3d anchorB)
        {
            rowBlocks.Add(new(offset, false));
            RigidBodyState a = state.Bodies[bodyA];
            RigidBodyState b = state.Bodies[bodyB];
            Put(PositionError, offset, a.Pose.TransformPoint(anchorA) - b.Pose.TransformPoint(anchorB));
            Vector3d centripetalA = a.Pose.TransformDirection(Vector3d.Cross(a.BodyAngularVelocity,
                Vector3d.Cross(a.BodyAngularVelocity, anchorA)));
            Vector3d centripetalB = b.Pose.TransformDirection(Vector3d.Cross(b.BodyAngularVelocity,
                Vector3d.Cross(b.BodyAngularVelocity, anchorB)));
            Put(Bias, offset, centripetalA - centripetalB);
            for (int axis = 0; axis < 3; axis++)
            {
                Vector3d direction = axis == 0 ? Vector3d.UnitX : axis == 1 ? Vector3d.UnitY : Vector3d.UnitZ;
                int row = offset + axis;
                Jacobian[row, 6 * bodyA + axis] = 1;
                Jacobian[row, 6 * bodyB + axis] = -1;
                PutRow(Jacobian, row, 6 * bodyA + 3, Vector3d.Cross(anchorA, a.Pose.InverseTransformDirection(direction)));
                PutRow(Jacobian, row, 6 * bodyB + 3, -Vector3d.Cross(anchorB, b.Pose.InverseTransformDirection(direction)));
            }
        }
    }

    internal int RevoluteRow(int index) => checked(sphericalRows + 5 * index);

    internal double MaximumLinearError(double[] errors) => MaximumError(errors, false);

    internal double MaximumAngularError(double[] errors) => MaximumError(errors, true);

    private double MaximumError(double[] errors, bool angular)
    {
        double maximum = 0;
        foreach (ConstraintRowBlock block in blocks)
        {
            if (block.Angular != angular) { continue; }
            Vector3d error = angular ? new(errors[block.Offset], errors[block.Offset + 1], 0) : Get(errors, block.Offset);
            maximum = System.Math.Max(maximum, error.Length);
        }
        return maximum;
    }

    internal double PositionNorm(ConstrainedSolverSettings settings) => System.Math.Max(
        MaximumLinearError(PositionError) / settings.PositionTolerance,
        MaximumAlignmentError / settings.AngularConstraints.OrientationTolerance);

    internal double VelocityNorm(ConstrainedSolverSettings settings) => System.Math.Max(
        MaximumLinearError(VelocityError) / settings.VelocityTolerance,
        MaximumAngularError(VelocityError) / settings.AngularConstraints.AngularVelocityTolerance);

    internal RevoluteJointReaction RevoluteReaction(MultibodySystem system, MultibodyState state, int index, double[] multipliers)
    {
        RevoluteJoint joint = system.RevoluteJoints[index];
        int offset = RevoluteRow(index);
        Vector3d force = Get(multipliers, offset);
        Vector3d couple = coupleDirections[2 * index] * multipliers[offset + 3]
            + coupleDirections[2 * index + 1] * multipliers[offset + 4];
        Vector3d armA = state.Bodies[joint.BodyA].Pose.TransformDirection(joint.LocalFrameA.Position);
        Vector3d armB = state.Bodies[joint.BodyB].Pose.TransformDirection(joint.LocalFrameB.Position);
        return new(force, couple, Vector3d.Cross(armA, force) + couple, Vector3d.Cross(armB, -force) - couple);
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

// Angular blocks contain two dimensionless alignment rows; point blocks contain three metre rows.
internal readonly record struct ConstraintRowBlock(int Offset, bool Angular);
