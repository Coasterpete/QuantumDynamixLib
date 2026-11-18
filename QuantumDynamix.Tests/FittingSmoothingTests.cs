using System;
using System.Linq;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using QuantumDynamix.Math;
using QuantumDynamix.Math.Functions;
using QuantumDynamix.Splines;
using QuantumDynamix.Splines.Curves;
using QuantumDynamix.Splines.Fitting;

namespace QuantumDynamix.Tests;

[TestClass]
public sealed class FittingSmoothingTests
{
    [TestMethod]
    public void ConstrainedFitRecoversAnAnalyticCubicAndItsEndpointSlopes()
    {
        var definition = SplineDefinition.Clamped(4, 3);
        ScalarFitSample[] samples = Enumerable.Range(0, 21).Select(i => i / 20.0)
            .Select(x => new ScalarFitSample(x, 1 + 2 * x - 3 * x * x + 4 * x * x * x)).ToArray();
        ScalarFitResult result = ConstrainedSplineFitter.FitScalar(definition, samples,
            new[] { new ScalarFitConstraint(0, 0, 1), new ScalarFitConstraint(1, 0, 4), new ScalarFitConstraint(0, 1, 2), new ScalarFitConstraint(1, 1, 8) });
        Assert.AreEqual(FitStatus.Success, result.Diagnostics.Status);
        Assert.IsNotNull(result.Curve);
        Assert.AreEqual(1.75, result.Curve.Evaluate(0.5).Value, 2e-13);
        Assert.AreEqual(24, result.Curve.Evaluate(0.3).ThirdDerivative, 1e-11);
        Assert.IsTrue(result.Diagnostics.WeightedResidualNorm < 1e-11);
    }

    [TestMethod]
    public void SpatialFittingReconstructsRationalCircleWithFixedWeights()
    {
        SplineCurve3d source = SplineFoundationTests.QuarterCircle(3);
        CurveFitResult fit = ConstrainedSplineFitter.FitCurve(source.Definition, Enumerable.Range(0, 31)
            .Select(i => new CurveFitSample(i / 30.0, source.Evaluate(i / 30.0).Position)));
        Assert.AreEqual(FitStatus.Success, fit.Diagnostics.Status);
        Assert.IsNotNull(fit.Curve);
        foreach (double u in new[] { 0, 0.13, 0.7, 1 }) { Assert.AreEqual(3, fit.Curve.Evaluate(u).Position.Length, 2e-13); }
    }

    [TestMethod]
    public void ContradictoryConstraintsAndUnderdeterminedFitsHaveDifferentDiagnostics()
    {
        SplineDefinition definition = SplineDefinition.Clamped(4, 3);
        ScalarFitSample[] samples = { new(0, 0), new(1, 1) };
        ScalarFitResult impossible = ConstrainedSplineFitter.FitScalar(definition, samples,
            new[] { new ScalarFitConstraint(0, 0, 0), new ScalarFitConstraint(0, 0, 1) });
        Assert.AreEqual(FitStatus.Infeasible, impossible.Diagnostics.Status);
        Assert.IsNull(impossible.Curve);
        ScalarFitResult insufficient = ConstrainedSplineFitter.FitScalar(definition, samples);
        Assert.AreEqual(FitStatus.RankDeficient, insufficient.Diagnostics.Status);
        Assert.IsNull(insufficient.Curve);
    }

    [TestMethod]
    public void RedundantConsistentConstraintsDoNotCreateFalseInfeasibility()
    {
        ScalarFitResult result = ConstrainedSplineFitter.FitScalar(SplineDefinition.Clamped(2, 1),
            new[] { new ScalarFitSample(0, 2), new ScalarFitSample(0.5, 3), new ScalarFitSample(1, 4) },
            new[] { new ScalarFitConstraint(0, 0, 2), new ScalarFitConstraint(0, 0, 2), new ScalarFitConstraint(1, 0, 4) });
        Assert.AreEqual(FitStatus.Success, result.Diagnostics.Status);
        Assert.AreEqual(3, result.Curve!.Evaluate(0.5).Value, 1e-14);
    }

    [TestMethod]
    public void ScalarSmoothingReducesRoughnessPreservesLocksAndLeavesSourceUntouched()
    {
        ScalarSpline source = NoisyScalar();
        double[] original = source.Coefficients.ToArray();
        ScalarSmoothingResult result = SplineSmoothing.SmoothScalar(source, new(strength: 1e-5, roughnessOrder: 2), new[] { 8 },
            new[] { new ScalarFitConstraint(0.5, 0, source.Evaluate(0.5).Value) });
        Assert.AreEqual(FitStatus.Success, result.Diagnostics.Status, result.Diagnostics.Message);
        Assert.IsNotNull(result.Candidate);
        CollectionAssert.AreEqual(original, source.Coefficients.ToArray());
        Assert.AreEqual(original[8], result.Candidate.Coefficients[8], 0);
        Assert.AreEqual(source.Evaluate(0.5).Value, result.Candidate.Evaluate(0.5).Value, 2e-10);
        for (int d = 0; d <= 3; d++)
        {
            Assert.AreEqual(source.Evaluate(0).Derivative(d), result.Candidate.Evaluate(0).Derivative(d), 2e-8);
            Assert.AreEqual(source.Evaluate(1).Derivative(d), result.Candidate.Evaluate(1).Derivative(d), 2e-8);
        }
        Assert.IsTrue(result.Metrics.CandidateRoughness < result.Metrics.OriginalRoughness);
        Assert.IsTrue(result.Metrics.MaximumSampleDeviation > 0);
        Assert.IsTrue(result.Comparison.Count > 10);
    }

    [TestMethod]
    public void SelectedRegionPreservesEverythingOutsideTheSelectionExactly()
    {
        ScalarSpline source = NoisyScalar();
        ScalarSmoothingResult result = SplineSmoothing.SmoothScalar(source,
            new(strength: 1e-4, roughnessOrder: 2, regionMinimum: 0.2, regionMaximum: 0.8));
        Assert.AreEqual(FitStatus.Success, result.Diagnostics.Status, result.Diagnostics.Message);
        Assert.IsNotNull(result.Candidate);
        foreach (double u in Enumerable.Range(0, 101).Select(i => i / 100.0).Where(u => u <= 0.2 || u >= 0.8))
        {
            ScalarJet a = source.Evaluate(u, EvaluationSide.Right);
            ScalarJet b = result.Candidate.Evaluate(u, EvaluationSide.Right);
            for (int d = 0; d <= 3; d++) { Assert.AreEqual(a.Derivative(d), b.Derivative(d), 0); }
        }
        Assert.IsTrue(result.Metrics.FrozenControlCount > 0);
        Assert.IsTrue(result.Metrics.CandidateRoughness < result.Metrics.OriginalRoughness);
    }

    [TestMethod]
    public void ZeroStrengthIsExactAndStillChecksConstraintFeasibility()
    {
        ScalarSpline source = NoisyScalar();
        ScalarSmoothingResult same = SplineSmoothing.SmoothScalar(source);
        Assert.AreSame(source, same.Candidate);
        Assert.AreEqual(0, same.Metrics.MaximumSampleDeviation);
        ScalarSmoothingResult conflict = SplineSmoothing.SmoothScalar(source,
            constraints: new[] { new ScalarFitConstraint(0.5, 0, 1000) });
        Assert.AreEqual(FitStatus.Infeasible, conflict.Diagnostics.Status);
        Assert.IsNull(conflict.Candidate);
    }

    [TestMethod]
    public void StrongerSmoothingReducesRoughnessWithoutWeakeningHardLocks()
    {
        ScalarSpline source = NoisyScalar();
        ScalarSmoothingResult weak = SplineSmoothing.SmoothScalar(source, new(strength: 1e-7, roughnessOrder: 2), new[] { 10 });
        ScalarSmoothingResult strong = SplineSmoothing.SmoothScalar(source, new(strength: 1e-3, roughnessOrder: 2), new[] { 10 });
        Assert.AreEqual(FitStatus.Success, weak.Diagnostics.Status, weak.Diagnostics.Message);
        Assert.AreEqual(FitStatus.Success, strong.Diagnostics.Status, strong.Diagnostics.Message);
        Assert.IsTrue(strong.Metrics.CandidateRoughness <= weak.Metrics.CandidateRoughness);
        Assert.AreEqual(source.Coefficients[10], strong.Candidate!.Coefficients[10], 0);
    }

    [TestMethod]
    public void ControlLockConflictsAreReportedWithoutReturningAPartialCandidate()
    {
        ScalarSpline source = NoisyScalar();
        ScalarSmoothingResult result = SplineSmoothing.SmoothScalar(source, new(strength: 1), new[] { 0 },
            new[] { new ScalarFitConstraint(0, 0, source.Coefficients[0] + 1) });
        Assert.AreEqual(FitStatus.Infeasible, result.Diagnostics.Status);
        Assert.IsNull(result.Candidate);
    }

    [TestMethod]
    public void SpatialSmoothingPreservesFeaturePositionsAndReturnsCurvatureComparison()
    {
        SplineCurve3d source = NoisyCurve();
        Vector3d feature = source.Evaluate(0.5).Position;
        CurveSmoothingResult result = SplineSmoothing.SmoothCurve(source,
            new(strength: 1e-7, roughnessOrder: 3), new[] { 10 }, new[] { new CurveFitConstraint(0.5, 0, feature) });
        Assert.AreEqual(FitStatus.Success, result.Diagnostics.Status, result.Diagnostics.Message);
        Assert.IsNotNull(result.Candidate);
        TestMath.Near(feature, result.Candidate.Evaluate(0.5).Position, 2e-9);
        Assert.AreEqual(source.ControlPoints[10], result.Candidate.ControlPoints[10]);
        Assert.IsTrue(result.Metrics.CandidateRoughness < result.Metrics.OriginalRoughness);
        Assert.IsTrue(result.Comparison.All(s => s.OriginalCurvature.HasValue && s.CandidateCurvature.HasValue));
        Assert.AreSame(source.Definition, result.Candidate.Definition);
    }

    [TestMethod]
    public void GeometricFairingUsesOriginalArcLengthRatherThanPenalizingParameterSpeed()
    {
        SplineCurve3d source = SplineCurve3d.Clamped(new[] { Vector3d.Zero, new Vector3d(0.1, 0, 0), new Vector3d(3, 0, 0), new Vector3d(10, 0, 0) }, 3);
        CurveSmoothingResult result = SplineSmoothing.SmoothCurve(source, new(strength: 1, lockBoundaries: false));
        Assert.AreEqual(FitStatus.Success, result.Diagnostics.Status, result.Diagnostics.Message);
        Assert.IsTrue(result.Metrics.OriginalRoughness < 1e-14);
        Assert.IsTrue(result.Metrics.MaximumSampleDeviation < 1e-7);
    }

    [TestMethod]
    public void QuadratureRefinementAndValueScalingPreserveTheScalarFit()
    {
        ScalarSpline source = NoisyScalar();
        ScalarSmoothingResult first = SplineSmoothing.SmoothScalar(source, new(strength: 1e-5, roughnessOrder: 2, valueScale: 1));
        ScalarSmoothingResult second = SplineSmoothing.SmoothScalar(source, new(strength: 1e-5, roughnessOrder: 2, valueScale: 2, quadratureSubdivisions: 4));
        Assert.AreEqual(FitStatus.Success, first.Diagnostics.Status, first.Diagnostics.Message);
        Assert.AreEqual(FitStatus.Success, second.Diagnostics.Status, second.Diagnostics.Message);
        for (int i = 0; i < source.Coefficients.Count; i++)
        { Assert.AreEqual(first.Candidate!.Coefficients[i], second.Candidate!.Coefficients[i], 1e-9); }
        Assert.AreEqual(first.Metrics.CandidateRoughness / 4, second.Metrics.CandidateRoughness, 1e-5);
    }

    [TestMethod]
    public void CustomRoughnessWeightsAreSupportedAndValidated()
    {
        ScalarSpline source = NoisyScalar();
        ScalarSmoothingResult result = SplineSmoothing.SmoothScalar(source,
            new(strength: 1e-3, roughnessOrder: 2, roughnessWeight: x => x < 0.5 ? 0.2 : 1));
        Assert.AreEqual(FitStatus.Success, result.Diagnostics.Status, result.Diagnostics.Message);
        Assert.ThrowsExactly<ArgumentException>(() => SplineSmoothing.SmoothScalar(source, new(strength: 1, roughnessWeight: _ => -1)));
    }

    [TestMethod]
    public void PeriodicFittingAndSmoothingMaintainTheSeam()
    {
        ScalarSpline source = ScalarSpline.Periodic(Enumerable.Range(0, 12).Select(i => System.Math.Sin(2 * System.Math.PI * i / 12)), 3);
        ScalarSmoothingResult result = SplineSmoothing.SmoothScalar(source, new(strength: 1e-4, roughnessOrder: 2, lockBoundaries: false));
        Assert.AreEqual(FitStatus.Success, result.Diagnostics.Status, result.Diagnostics.Message);
        Assert.AreEqual(0, result.Metrics.FrozenControlCount);
        for (int order = 0; order <= 2; order++)
        { Assert.AreEqual(result.Candidate!.Evaluate(0, EvaluationSide.Right).Derivative(order), result.Candidate.Evaluate(1, EvaluationSide.Left).Derivative(order), 1e-8); }
    }

    [TestMethod]
    public void InvalidFittingAndSmoothingInputsAreRejected()
    {
        Assert.ThrowsExactly<ArgumentOutOfRangeException>(() => new SmoothingSettings(double.NaN));
        Assert.ThrowsExactly<ArgumentException>(() => new SmoothingSettings(regionMinimum: 0.2));
        Assert.ThrowsExactly<ArgumentOutOfRangeException>(() => SplineSmoothing.SmoothScalar(NoisyScalar(), lockedControlPoints: new[] { -1 }));
        Assert.ThrowsExactly<ArgumentOutOfRangeException>(() => ConstrainedSplineFitter.FitScalar(SplineDefinition.Clamped(2, 1), new[] { new ScalarFitSample(0, 0, 0) }));
        var definition = new SplineDefinition(2, new[] { 0.0, 0, 0, 0.5, 0.5, 1, 1, 1 });
        Assert.ThrowsExactly<ArgumentException>(() => ConstrainedSplineFitter.FitScalar(definition, Array.Empty<ScalarFitSample>(), new[] { new ScalarFitConstraint(0.5, 1, 0) }));
    }

    [TestMethod]
    public void NonfiniteDerivedFittingArithmeticReturnsNumericalFailure()
    {
        ScalarFitResult result = ConstrainedSplineFitter.FitScalar(SplineDefinition.Clamped(2, 1),
            new[] { new ScalarFitSample(0, 1e308, 1e308), new ScalarFitSample(1, 1e308, 1e308) });
        Assert.AreEqual(FitStatus.NumericalFailure, result.Diagnostics.Status);
        Assert.IsNull(result.Curve);
    }

    [TestMethod]
    public void SampleWeightsMultiplySquaredResiduals()
    {
        ScalarFitResult result = ConstrainedSplineFitter.FitScalar(SplineDefinition.Clamped(2, 1),
            new[] { new ScalarFitSample(0, 0), new ScalarFitSample(1, 1), new ScalarFitSample(0.5, 10, 100) });
        Assert.AreEqual(FitStatus.Success, result.Diagnostics.Status);
        Assert.AreEqual(1001.0 / 102, result.Curve!.Evaluate(0.5).Value, 1e-12);
        Assert.AreEqual(1, result.Curve.Evaluate(0.5).FirstDerivative, 1e-12);
    }

    [TestMethod]
    public void ASelectionWithNoInteriorSupportReportsAllCoefficientsFrozen()
    {
        ScalarSpline source = NoisyScalar();
        ScalarSmoothingResult result = SplineSmoothing.SmoothScalar(source,
            new(strength: 1, regionMinimum: 0.49, regionMaximum: 0.51));
        Assert.AreEqual(FitStatus.Success, result.Diagnostics.Status, result.Diagnostics.Message);
        Assert.AreEqual(source.Coefficients.Count, result.Metrics.FrozenControlCount);
        Assert.AreEqual(0, result.Metrics.MaximumSampleDeviation);
        StringAssert.Contains(result.Diagnostics.Message, "frozen");
    }

    private static ScalarSpline NoisyScalar() => ScalarSpline.Clamped(Enumerable.Range(0, 24)
        .Select(i => 0.3 * System.Math.Sin(i * 2.1) + System.Math.Sin(System.Math.PI * i / 23)), 5);

    private static SplineCurve3d NoisyCurve() => SplineCurve3d.Clamped(Enumerable.Range(0, 24).Select(i =>
        new Vector3d(i * 2, 0.15 * System.Math.Sin(i * 2.1), 0.1 * System.Math.Cos(i * 1.7))), 5);
}
