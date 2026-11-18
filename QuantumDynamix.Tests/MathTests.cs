using System;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using QuantumDynamix.Geometry;
using QuantumDynamix.Math;

namespace QuantumDynamix.Tests;

[TestClass]
public sealed class MathTests
{
    [TestMethod]
    public void VectorArithmeticUsesRightHandedCrossProduct()
    {
        Assert.AreEqual(Vector3d.UnitZ, Vector3d.Cross(Vector3d.UnitX, Vector3d.UnitY));
        Assert.AreEqual(-Vector3d.UnitZ, Vector3d.Cross(Vector3d.UnitY, Vector3d.UnitX));
        Assert.AreEqual(32, Vector3d.Dot(new(1, 2, 3), new(4, 5, 6)));
        Assert.AreEqual(new Vector3d(3, 6, 9), 2 * new Vector3d(1, 2, 3) + new Vector3d(1, 2, 3));
        Assert.AreEqual(13, new Vector3d(3, 4, 12).Length, 1e-14);
    }

    [TestMethod]
    [DataRow(1e300)]
    [DataRow(1e-300)]
    public void NormalizationHandlesLargeAndSmallFiniteScales(double scale)
    {
        TestMath.Near(new(0.6, 0.8, 0), new Vector3d(3 * scale, 4 * scale, 0).Normalized(), 2e-16);
        Quaterniond q = new Quaterniond(scale, scale, scale, scale).Normalized();
        Assert.AreEqual(new Quaterniond(0.5, 0.5, 0.5, 0.5), q);
    }

    [TestMethod]
    public void HamiltonProductComposesActiveRotationsInOrder()
    {
        Quaterniond z = Quaterniond.FromAxisAngle(Vector3d.UnitZ, System.Math.PI / 2);
        Quaterniond x = Quaterniond.FromAxisAngle(Vector3d.UnitX, System.Math.PI / 2);
        TestMath.Near(Vector3d.UnitY, z.Rotate(Vector3d.UnitX), 3e-16);
        TestMath.Near(Vector3d.UnitZ, (x * z).Rotate(Vector3d.UnitX), 5e-16);
        TestMath.Near(Vector3d.UnitY, (z * x).Rotate(Vector3d.UnitX), 5e-16);
        TestMath.SameRotation(z, z * -1, 0);
    }

    [TestMethod]
    public void PoseDistinguishesPointsFromDirectionsAndRoundTrips()
    {
        RigidPose3d pose = new(new(10, -4, 2), Quaterniond.FromAxisAngle(Vector3d.UnitZ, System.Math.PI / 2));
        TestMath.Near(new(10, -3, 2), pose.TransformPoint(Vector3d.UnitX), 1e-14);
        TestMath.Near(Vector3d.UnitY, pose.TransformDirection(Vector3d.UnitX), 3e-16);
        Vector3d point = new(2, -3, 5);
        TestMath.Near(point, pose.InverseTransformPoint(pose.TransformPoint(point)), 3e-15);
        TestMath.Near(point, pose.InverseTransformDirection(pose.TransformDirection(point)), 3e-15);
    }

    [TestMethod]
    public void FullMatrixProductAndTransposeAgreeWithRotations()
    {
        Matrix3x3d rotation = Quaterniond.FromAxisAngle(new(1, 2, -3), 1.2).ToRotationMatrix();
        Vector3d vector = new(3, -4, 2);
        TestMath.Near(vector, (rotation.Transposed() * rotation) * vector, 4e-15);
        Assert.AreEqual(Matrix3x3d.Identity, Matrix3x3d.Identity * Matrix3x3d.Identity);
    }

    [TestMethod]
    public void CholeskySolvesFullTensorWithOffDiagonalTerms()
    {
        Matrix3x3d matrix = new(4, 1, 0.5, 1, 5, -0.25, 0.5, -0.25, 6);
        TestMath.Near(new(2, -1, 3), matrix.SolvePositiveDefinite(new(8.5, -3.75, 19.25)), 1e-15);
    }

    [TestMethod]
    [DataRow(1e300)]
    [DataRow(1e-300)]
    public void CholeskyHandlesUniformlyScaledTensors(double scale)
    {
        Matrix3x3d matrix = new(4 * scale, scale, 0, scale, 3 * scale, 0, 0, 0, 2 * scale);
        TestMath.Near(new(1, 2, -3), matrix.SolvePositiveDefinite(new(6 * scale, 7 * scale, -6 * scale)), 2e-15);
    }

    [TestMethod]
    public void PoseNormalizesExplicitOrientations()
    {
        RigidPose3d pose = new(Vector3d.Zero, new(10, 0, 0, 0));
        Assert.AreEqual(Quaterniond.Identity, pose.Orientation);
    }
}
