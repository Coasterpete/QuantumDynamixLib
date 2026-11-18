using System.Collections.Generic;
using QuantumDynamix.Math;
using QuantumDynamix.Math.Functions;
using QuantumDynamix.Splines.Curves;
using QuantumDynamix.Track;

namespace QuantumDynamix.Tests;

internal static class RailTestFixtures
{
    // Numerical regression controls for these fixtures, not coaster engineering acceptance criteria.
    internal static RailValidationSettings Settings => new(1e-8, 1e-7, 1e-7, 1e-7, 1e-10, 1e-8);
    internal static IScalarFunction Constant(double value, double length) => new CustomFunction(0, length, (_, _) => new(value));
    internal static SplineCurve3d Line(double length = 10, Vector3d origin = default) =>
        SplineCurve3d.Clamped(new[] { origin, origin + new Vector3d(length, 0, 0) }, 1);
    internal static RailCrossSection Section(double halfWidth = 0.1) => new(new[] {
        SplineCurve3d.Clamped(new[] { new Vector3d(0, -halfWidth, 0), new Vector3d(0, halfWidth, 0) }, 1) },
        new Dictionary<string, Vector3d> { ["negative"] = new(0, -halfWidth, 0), ["positive"] = new(0, halfWidth, 0) });
    internal static RailDefinition Rail(string id, Vector3d offset, double length, RailCrossSection? section = null, IScalarFunction? roll = null) =>
        new(id, TrackOffsetProfile.Constant(offset, length), section ?? Section(), roll ?? Constant(0, length));
    internal static RailGeometry Straight(params RailDefinition[] rails) => new(
        new HeartlineTrack(Line(), Vector3d.UnitZ, TrackOffsetProfile.Constant(default, 10)), new(rails), Settings);
}
