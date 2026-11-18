using Microsoft.VisualStudio.TestTools.UnitTesting;
using QuantumDynamix.Math;

namespace QuantumDynamix.Tests;

internal static class TestMath
{
    internal static void Near(Vector3d expected, Vector3d actual, double tolerance)
    {
        Assert.AreEqual(expected.X, actual.X, tolerance, "X component");
        Assert.AreEqual(expected.Y, actual.Y, tolerance, "Y component");
        Assert.AreEqual(expected.Z, actual.Z, tolerance, "Z component");
    }

    internal static void SameRotation(Quaterniond expected, Quaterniond actual, double tolerance)
    {
        // Compare rotated axes so q and -q correctly represent the same orientation.
        Near(expected.Rotate(Vector3d.UnitX), actual.Rotate(Vector3d.UnitX), tolerance);
        Near(expected.Rotate(Vector3d.UnitY), actual.Rotate(Vector3d.UnitY), tolerance);
        Near(expected.Rotate(Vector3d.UnitZ), actual.Rotate(Vector3d.UnitZ), tolerance);
    }
}
