using System;
using System.Linq;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using QuantumDynamix.Geometry;
using QuantumDynamix.Math;
using QuantumDynamix.Math.Functions;
using QuantumDynamix.Math.Numerics;
using QuantumDynamix.Splines;
using QuantumDynamix.Splines.Curves;
using QuantumDynamix.Track;

namespace QuantumDynamix.Tests;

[TestClass]
public sealed class GeometryTrackFoundationTests
{
    [TestMethod]
    public void ParabolaCurvatureAndLengthMatchClosedFormMathematics()
    {
        var parabola = SplineCurve3d.Clamped(new[] { Vector3d.Zero, new Vector3d(0.5, 0, 0), new Vector3d(1, 1, 0) }, 2);
        foreach (double u in new[] { 0.0, 0.2, 0.5, 1 })
        {
            CurveGeometrySample geometry = CurveGeometry.Evaluate(parabola, u);
            Assert.AreEqual(2 / System.Math.Pow(1 + 4 * u * u, 1.5), geometry.Curvature, 2e-15);
            Assert.AreEqual(0, geometry.Torsion!.Value, 1e-15);
        }
        var map = new ArcLengthMap(parabola);
        Assert.AreEqual(System.Math.Sqrt(5) / 2 + System.Math.Asinh(2) / 4, map.Length, 2e-12);
    }

    [TestMethod]
    [DataRow(0.01)]
    [DataRow(1.0)]
    [DataRow(1000.0)]
    public void RationalCircleGeometryLengthAndInversionAreAccurateAcrossScales(double radius)
    {
        SplineCurve3d circle = SplineFoundationTests.QuarterCircle(radius);
        var map = new ArcLengthMap(circle);
        Assert.AreEqual(System.Math.PI * radius / 2, map.Length, 2e-10 * System.Math.Max(1, radius));
        foreach (double u in new[] { 0, 0.1, 0.35, 0.7, 1 })
        {
            CurveGeometrySample geometry = CurveGeometry.Evaluate(circle, u);
            Assert.AreEqual(1 / radius, geometry.Curvature, 2e-12 / radius);
            double angle = System.Math.Atan2(geometry.Position.Y, geometry.Position.X);
            Assert.AreEqual(radius * angle, map.StationAt(u), 2e-9 * System.Math.Max(1, radius));
            ArcLengthInversion inverted = map.Invert(radius * angle);
            Assert.AreEqual(u, inverted.Parameter, 2e-6 * System.Math.Max(1, 0.01 / radius));
            Assert.IsTrue(System.Math.Abs(inverted.StationResidual) + inverted.EstimatedLengthError <= 1e-8 + 1e-10 * map.Length);
        }
    }

    [TestMethod]
    public void CurvatureVectorStationDerivativeAgreesWithIndependentDifference()
    {
        SplineCurve3d curve = SplineCurve3d.Clamped(new[] { Vector3d.Zero, new Vector3d(1, 2, 1), new Vector3d(2, -1, 2), new Vector3d(4, 3, 5) }, 3);
        const double u = 0.4;
        const double h = 1e-5;
        CurveGeometrySample center = CurveGeometry.Evaluate(curve, u);
        Vector3d difference = (CurveGeometry.Evaluate(curve, u + h).CurvatureVector - CurveGeometry.Evaluate(curve, u - h).CurvatureVector) / (2 * h * center.ParameterSpeed);
        TestMath.Near(difference, center.CurvatureStationDerivative!.Value, 3e-9);
    }

    [TestMethod]
    public void InversionSafeguardsAZeroEndpointSpeed()
    {
        SplineCurve3d curve = SplineCurve3d.Clamped(new[] { Vector3d.Zero, Vector3d.Zero, Vector3d.Zero, Vector3d.UnitX }, 3);
        var map = new ArcLengthMap(curve);
        foreach (double station in new[] { 1e-6, 0.01, 0.5, 0.999 })
        {
            ArcLengthInversion inverse = map.Invert(station);
            Assert.AreEqual(station, System.Math.Pow(inverse.Parameter, 3), 1.1e-8);
        }
        Assert.ThrowsExactly<InvalidOperationException>(() => CurveGeometry.Evaluate(curve, 0));
        Assert.ThrowsExactly<ArgumentOutOfRangeException>(() => map.Invert(2));
    }

    [TestMethod]
    public void QuadratureHandlesSmoothAndPiecewiseFunctionsAndReportsBudgetExhaustion()
    {
        Assert.AreEqual(2, AdaptiveGaussKronrod.Integrate(System.Math.Sin, 0, System.Math.PI).Value, 2e-14);
        Assert.AreEqual(System.Math.E - 1, AdaptiveGaussKronrod.Integrate(System.Math.Exp, 0, 1).Value, 3e-14);
        Assert.AreEqual(-2, AdaptiveGaussKronrod.Integrate(System.Math.Sin, System.Math.PI, 0).Value, 2e-14);
        Assert.AreEqual((0.3 * 0.3 + 0.7 * 0.7) / 2,
            AdaptiveGaussKronrod.Integrate(x => System.Math.Abs(x - 0.3), 0, 1).Value, 2e-8);
        Assert.ThrowsExactly<NumericalConvergenceException>(() => AdaptiveGaussKronrod.Integrate(x => System.Math.Sin(1000 * x), 0, 1,
            new QuadratureSettings(1e-12, maximumEvaluations: 15)));
        Assert.ThrowsExactly<ArithmeticException>(() => AdaptiveGaussKronrod.Integrate(_ => double.NaN, 0, 1));
    }

    [TestMethod]
    public void PlanarFrameTransportPreservesAnExplicitBinormalAndRightHandedAxes()
    {
        SplineCurve3d curve = SplineFoundationTests.QuarterCircle();
        var frames = new RotationMinimizingFrames(curve, Vector3d.UnitZ);
        foreach (double u in Enumerable.Range(0, 51).Select(i => i / 50.0))
        {
            CurveFrame frame = frames.Evaluate(u);
            TestMath.Near(Vector3d.UnitZ, frame.Up, 2e-14);
            TestMath.Near(frame.Up, Vector3d.Cross(frame.Forward, frame.Left), 2e-14);
            Assert.AreEqual(1, frame.Left.Length, 2e-14);
            Assert.AreEqual(0, Vector3d.Dot(frame.Forward, frame.Up), 2e-14);
        }
        Assert.IsNull(frames.ClosureTwist);
        CurveFrame bank = frames.Evaluate(0.4).Banked(System.Math.PI / 2);
        TestMath.Near(Vector3d.UnitZ, bank.Left, 2e-14);
    }

    [TestMethod]
    public void SpatialFramesAreRotationEquivariantAndConvergeWithTighterAngularBudget()
    {
        Vector3d[] controls = { new(0, 0, 0), new(1, 2, -1), new(3, 2, 2), new(5, -1, 4), new(6, 2, 3), new(7, 0, 6) };
        var curve = SplineCurve3d.Clamped(controls);
        Quaterniond rotation = Quaterniond.FromAxisAngle(new(1, 2, 3), 0.7);
        var rotated = SplineCurve3d.Clamped(controls.Select(rotation.Rotate));
        var a = new RotationMinimizingFrames(curve, Vector3d.UnitZ, 1e-7);
        var b = new RotationMinimizingFrames(rotated, rotation.Rotate(Vector3d.UnitZ), 1e-7);
        var tighter = new RotationMinimizingFrames(curve, Vector3d.UnitZ, 1e-9);
        TestMath.Near(rotation.Rotate(a.Evaluate(1).Up), b.Evaluate(1).Up, 2e-8);
        TestMath.Near(tighter.Evaluate(1).Up, a.Evaluate(1).Up, 2e-7);
        Assert.IsTrue(a.EstimatedAccumulatedAngularError <= 1e-7);
    }

    [TestMethod]
    public void PeriodicPlanarFrameReportsItsClosureTwist()
    {
        var curve = SplineCurve3d.Periodic(Enumerable.Range(0, 9).Select(i =>
            new Vector3d(System.Math.Cos(2 * System.Math.PI * i / 9), System.Math.Sin(2 * System.Math.PI * i / 9), 0)));
        var frames = new RotationMinimizingFrames(curve, Vector3d.UnitZ);
        Assert.AreEqual(0, frames.ClosureTwist!.Value, 1e-12);
    }

    [TestMethod]
    public void CircularHeartlineAndOffsetSpineHaveDifferentKnownLengthsAndCurvatures()
    {
        SplineCurve3d circle = SplineFoundationTests.QuarterCircle(10);
        var map = new ArcLengthMap(circle);
        var track = new HeartlineTrack(circle, Vector3d.UnitZ, TrackOffsetProfile.Constant(new(0, 2, 0), map.Length),
            features: new[] { new TrackFeature("start", 0), new TrackFeature("end", 1) });
        HeartlineTrackSample sample = track.Evaluate(map.Length / 2);
        Assert.AreEqual(10, sample.HeartlinePosition.Length, 1e-13);
        Assert.AreEqual(8, sample.SpinePosition.Length, 1e-12);
        Assert.AreEqual(0.8, sample.SpineSpeedPerHeartlineMetre, 2e-14);
        Assert.AreEqual(1.0 / 8, sample.SpineCurvature, 2e-14);
        Assert.AreEqual(4 * System.Math.PI, track.IntegrateSpineLength().Value, 2e-8);
        Assert.AreEqual(map.Length, track.ResolveFeatures().Resolve("end"));
    }

    [TestMethod]
    public void BankedOffsetAroundStraightHeartlineMatchesAnalyticHelix()
    {
        var line = SplineCurve3d.Clamped(new[] { Vector3d.Zero, new Vector3d(10, 0, 0) }, 1);
        double length = new ArcLengthMap(line).Length;
        const double k = 0.2;
        const double h = 2;
        var bank = new CustomFunction(0, length, (s, _) => new(k * s, k));
        var track = new HeartlineTrack(line, Vector3d.UnitZ, TrackOffsetProfile.Constant(new(0, 0, h), length), bank);
        const double s = 3;
        HeartlineTrackSample sample = track.Evaluate(s);
        TestMath.Near(new(s, -h * System.Math.Sin(k * s), h * System.Math.Cos(k * s)), sample.SpinePosition, 2e-13);
        TestMath.Near(new(1, -h * k * System.Math.Cos(k * s), -h * k * System.Math.Sin(k * s)), sample.SpineStationDerivative, 2e-13);
        TestMath.Near(new(0, h * k * k * System.Math.Sin(k * s), -h * k * k * System.Math.Cos(k * s)), sample.SpineStationSecondDerivative, 2e-13);
        Assert.AreEqual(length * System.Math.Sqrt(1 + h * h * k * k), track.IntegrateSpineLength().Value, 3e-8);
    }

    [TestMethod]
    public void VariableHeartlineOffsetsContributeTheirOwnDerivatives()
    {
        var line = SplineCurve3d.Clamped(new[] { Vector3d.Zero, new Vector3d(10, 0, 0) }, 1);
        double length = new ArcLengthMap(line).Length;
        var zero = new CustomFunction(0, length, (_, _) => new(0));
        var height = new CustomFunction(0, length, (s, _) => new(s * s, 2 * s, 2));
        var track = new HeartlineTrack(line, Vector3d.UnitZ, new(zero, zero, height));
        HeartlineTrackSample sample = track.Evaluate(2);
        TestMath.Near(new(2, 0, 4), sample.SpinePosition, 1e-13);
        TestMath.Near(new(1, 0, 4), sample.SpineStationDerivative, 1e-13);
        TestMath.Near(new(0, 0, 2), sample.SpineStationSecondDerivative, 1e-13);
    }

    [TestMethod]
    public void SingularAndDiscontinuousGeometricInputsFailExplicitly()
    {
        var point = SplineCurve3d.Clamped(new[] { Vector3d.Zero, Vector3d.Zero }, 1);
        Assert.ThrowsExactly<ArgumentException>(() => new ArcLengthMap(point));
        var jump = new SplineCurve3d(new SplineDefinition(1, new[] { 0.0, 0, 0.5, 0.5, 1, 1 }),
            new[] { Vector3d.Zero, Vector3d.UnitX, new Vector3d(2, 0, 0), new Vector3d(3, 0, 0) });
        Assert.ThrowsExactly<ArgumentException>(() => new ArcLengthMap(jump));
        var line = SplineCurve3d.Clamped(new[] { Vector3d.Zero, Vector3d.UnitX }, 1);
        Assert.ThrowsExactly<ArgumentException>(() => new RotationMinimizingFrames(line, Vector3d.UnitX));
        Assert.ThrowsExactly<ArgumentOutOfRangeException>(() => new QuadratureSettings(double.NaN));
        Assert.ThrowsExactly<ArgumentOutOfRangeException>(() => new RotationMinimizingFrames(line, Vector3d.UnitZ, double.NaN));
        var translated = SplineCurve3d.Clamped(new[] { new Vector3d(1e12, 0, 0), new Vector3d(1e12 + 1, 0, 0) }, 1);
        Assert.IsNull(new RotationMinimizingFrames(translated, Vector3d.UnitZ).ClosureTwist);
    }
}
