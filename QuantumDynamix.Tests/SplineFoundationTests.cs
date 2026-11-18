using System;
using System.Collections.Generic;
using System.Linq;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using QuantumDynamix.Math;
using QuantumDynamix.Math.Functions;
using QuantumDynamix.Splines;
using QuantumDynamix.Splines.Curves;

namespace QuantumDynamix.Tests;

[TestClass]
public sealed class SplineFoundationTests
{
    [TestMethod]
    [DataRow(1)]
    [DataRow(2)]
    [DataRow(3)]
    [DataRow(5)]
    [DataRow(7)]
    public void LocalBasisHasPartitionOfUnityAndZeroDerivativeSums(int degree)
    {
        var definition = SplineDefinition.Clamped(degree + 5, degree);
        foreach (double parameter in Enumerable.Range(0, 41).Select(i => i / 40.0))
        {
            SplineBasisSample basis = definition.EvaluateBasis(parameter);
            Assert.AreEqual(degree + 1, basis.Count);
            for (int order = 0; order <= 3; order++)
            {
                double sum = Enumerable.Range(0, basis.Count).Sum(j => basis[order, j]);
                Assert.AreEqual(order == 0 ? 1 : 0, sum, order == 3 ? 3e-9 : 3e-11);
            }
            for (int j = 0; j < basis.Count; j++) { Assert.IsTrue(basis[0, j] >= -1e-15); }
        }
    }

    [TestMethod]
    [DataRow(1)]
    [DataRow(3)]
    [DataRow(5)]
    public void PolynomialReproductionMatchesAnalyticFirstThroughThirdDerivatives(int power)
    {
        const int degree = 5;
        // A monomial u^k has clamped single-span B-spline coefficients (i choose k)/(p choose k).
        double[] controls = Enumerable.Range(0, degree + 1).Select(i => Choose(i, power) / Choose(degree, power)).ToArray();
        var spline = ScalarSpline.Clamped(controls, degree);
        foreach (double x in new[] { 0, 0.13, 0.5, 0.91, 1 })
        {
            ScalarJet jet = spline.Evaluate(x);
            for (int order = 0; order <= 3; order++)
            {
                double factor = 1;
                for (int k = 0; k < order; k++) { factor *= power - k; }
                double expected = order > power ? 0 : factor * System.Math.Pow(x, power - order);
                Assert.AreEqual(expected, jet.Derivative(order), 3e-12);
            }
        }
    }

    [TestMethod]
    public void RationalLineHasNonzeroDerivativesAbovePolynomialDegree()
    {
        var spline = ScalarSpline.Clamped(new[] { 0.0, 1 }, 1, new[] { 1.0, 2 });
        foreach (double x in new[] { 0, 0.2, 0.5, 1 })
        {
            ScalarJet jet = spline.Evaluate(x);
            Assert.AreEqual(2 * x / (1 + x), jet.Value, 1e-14);
            Assert.AreEqual(2 / System.Math.Pow(1 + x, 2), jet.FirstDerivative, 1e-13);
            Assert.AreEqual(-4 / System.Math.Pow(1 + x, 3), jet.SecondDerivative, 1e-13);
            Assert.AreEqual(12 / System.Math.Pow(1 + x, 4), jet.ThirdDerivative, 1e-12);
        }
    }

    [TestMethod]
    public void RationalQuadraticRepresentsAnExactQuarterCircle()
    {
        SplineCurve3d circle = QuarterCircle();
        for (int i = 0; i <= 100; i++)
        {
            CurveJet3d sample = circle.Evaluate(i / 100.0);
            Assert.AreEqual(1, sample.Position.Length, 4e-15);
            Assert.AreEqual(0, Vector3d.Dot(sample.Position, sample.FirstDerivative), 3e-15);
            Assert.AreEqual(-Vector3d.Dot(sample.FirstDerivative, sample.FirstDerivative), Vector3d.Dot(sample.Position, sample.SecondDerivative), 2e-14);
        }
    }

    [TestMethod]
    public void EqualAndCommonlyScaledWeightsPreserveTheCurve()
    {
        double[] coefficients = { 1, -2, 5, 9, 3, 2, 4 };
        var plain = ScalarSpline.Clamped(coefficients, 3);
        var scaled = ScalarSpline.Clamped(coefficients, 3, Enumerable.Repeat(1e250, coefficients.Length));
        for (int i = 0; i <= 40; i++)
        {
            for (int d = 0; d <= 3; d++)
            { Assert.AreEqual(plain.Evaluate(i / 40.0, EvaluationSide.Right).Derivative(d), scaled.Evaluate(i / 40.0, EvaluationSide.Right).Derivative(d), 0); }
        }
    }

    [TestMethod]
    public void RepeatedKnotsExposeOneSidedDerivativesAndPreserveEndpointConventions()
    {
        var definition = new SplineDefinition(2, new[] { 0.0, 0, 0, 0.5, 0.5, 1, 1, 1 });
        var curve = new ScalarSpline(definition, new[] { 0.0, 1, 0, 2, 3 });
        Assert.AreEqual(0, definition.ContinuityAt(0.5));
        Assert.AreEqual(0, curve.Evaluate(0.5).DerivativeOrder);
        Assert.ThrowsExactly<InvalidOperationException>(() => curve.Evaluate(0.5).Derivative(1));
        Assert.AreEqual(-4, curve.Evaluate(0.5, EvaluationSide.Left).FirstDerivative, 1e-14);
        Assert.AreEqual(8, curve.Evaluate(0.5, EvaluationSide.Right).FirstDerivative, 1e-14);
        Assert.AreEqual(0, curve.Evaluate(0).Value);
        Assert.AreEqual(3, curve.Evaluate(1).Value);
    }

    [TestMethod]
    public void FullMultiplicityHasExplicitRightHandValue()
    {
        var definition = new SplineDefinition(1, new[] { 0.0, 0, 0.5, 0.5, 1, 1 });
        var curve = new ScalarSpline(definition, new[] { 0.0, 1, 4, 5 });
        Assert.AreEqual(-1, definition.ContinuityAt(0.5));
        Assert.AreEqual(1, curve.Evaluate(0.5, EvaluationSide.Left).Value);
        Assert.AreEqual(4, curve.Evaluate(0.5).Value);
        Assert.AreEqual(0, curve.Evaluate(0.5).DerivativeOrder);
    }

    [TestMethod]
    public void NonuniformKnotsUseTheirSuppliedDomainAndGrevilleLinearPrecision()
    {
        var definition = new SplineDefinition(3, new[] { 2.0, 2, 2, 2, 2.1, 4.8, 5, 5, 5, 5 });
        double[] coefficients = Enumerable.Range(0, definition.ControlPointCount)
            .Select(i => (definition.Knots[i + 1] + definition.Knots[i + 2] + definition.Knots[i + 3]) / 3).ToArray();
        var curve = new ScalarSpline(definition, coefficients);
        foreach (double x in new[] { 2, 2.05, 2.1, 3, 4.8, 5 })
        {
            ScalarJet jet = curve.Evaluate(x, EvaluationSide.Right);
            Assert.AreEqual(x, jet.Value, 2e-14);
            Assert.AreEqual(1, jet.FirstDerivative, 1e-13);
            Assert.AreEqual(0, jet.SecondDerivative, 2e-12);
        }
        Assert.ThrowsExactly<ArgumentOutOfRangeException>(() => curve.Evaluate(0.5));
    }

    [TestMethod]
    public void EndpointSpanSearchHandlesRepeatedKnotsAtUnclampedDomainEdges()
    {
        var definition = new SplineDefinition(2, new[] { -1.0, 0, 0, 0, 1, 1, 2 });
        var curve = new ScalarSpline(definition, new[] { 10.0, 20, 30, 40 });
        Assert.IsTrue(double.IsFinite(curve.Evaluate(0).Value));
        Assert.IsTrue(double.IsFinite(curve.Evaluate(1).Value));
        Assert.AreEqual(20, curve.Evaluate(0).Value, 1e-14);
    }

    [TestMethod]
    [DataRow(1)]
    [DataRow(3)]
    [DataRow(5)]
    public void ExplicitPeriodicCurvesCloseAndWrapWithoutImplicitExtrapolation(int degree)
    {
        var points = Enumerable.Range(0, 9).Select(i => new Vector3d(System.Math.Cos(2 * System.Math.PI * i / 9),
            System.Math.Sin(2 * System.Math.PI * i / 9), 0.1 * System.Math.Sin(4 * System.Math.PI * i / 9))).ToArray();
        SplineCurve3d curve = SplineCurve3d.Periodic(points, degree, Enumerable.Range(0, 9).Select(i => 1 + 0.1 * i));
        for (int order = 0; order <= System.Math.Min(3, degree - 1); order++)
        { TestMath.Near(curve.Evaluate(0, EvaluationSide.Right).Derivative(order), curve.Evaluate(1, EvaluationSide.Left).Derivative(order), 1e-8); }
        Assert.AreEqual(0.25, curve.Definition.WrapParameter(-1.75), 1e-15);
        Assert.ThrowsExactly<ArgumentOutOfRangeException>(() => curve.Evaluate(1.25));
        ScalarSpline scalar = ScalarSpline.Periodic(Enumerable.Range(0, 9).Select(i => (double)i), degree);
        Assert.AreEqual(scalar.Evaluate(0).Value, scalar.Evaluate(1).Value, 1e-12);
    }

    [TestMethod]
    public void KnotInsertionIdentityPreservesAnExistingPiecewiseLinearCurve()
    {
        var original = ScalarSpline.Clamped(new[] { 2.0, 8 }, 1);
        var refined = new ScalarSpline(new SplineDefinition(1, new[] { 0.0, 0, 0.3, 1, 1 }), new[] { 2.0, 3.8, 8 });
        for (int i = 0; i <= 50; i++) { Assert.AreEqual(original.Evaluate(i / 50.0).Value, refined.Evaluate(i / 50.0).Value, 2e-15); }
    }

    [TestMethod]
    public void DefinitionsAndCurvesCopyInputsAndRejectInvalidNumericalData()
    {
        double[] knots = { 0, 0, 1, 1 };
        double[] weights = { 1, 2 };
        double[] coefficients = { 0, 1 };
        var definition = new SplineDefinition(1, knots, weights);
        var curve = new ScalarSpline(definition, coefficients);
        knots[0] = 99;
        weights[0] = 99;
        coefficients[0] = 99;
        Assert.AreEqual(0, curve.Evaluate(0).Value);
        Assert.ThrowsExactly<NotSupportedException>(() => ((IList<double>)definition.Knots)[0] = 4);
        Assert.ThrowsExactly<ArgumentException>(() => new SplineDefinition(1, new[] { 0.0, 0, 1, 1 }, new[] { 0.0, 1 }));
        Assert.ThrowsExactly<ArgumentException>(() => new SplineDefinition(1, new[] { 0.0, 0, 0, 1, 1 }));
        Assert.ThrowsExactly<ArgumentException>(() => new SplineDefinition(1, new[] { 0.0, 1, 0, 1 }));
        Assert.ThrowsExactly<ArgumentOutOfRangeException>(() => definition.EvaluateBasis(double.NaN));
        Assert.ThrowsExactly<ArgumentOutOfRangeException>(() => definition.EvaluateBasis(0.5, side: (EvaluationSide)99));
        Assert.ThrowsExactly<ArgumentException>(() => new ScalarSpline(definition, new[] { double.PositiveInfinity, 1 }));
        Assert.ThrowsExactly<ArgumentException>(() => new SplineCurve3d(new SplineDefinition(1, new[] { 0.0, 0, 1, 1 }, isPeriodic: true), new[] { Vector3d.Zero, Vector3d.UnitX }));
    }

    [TestMethod]
    public void LargeCoordinateOriginsDoNotConcealAnOpenPeriodicSeam()
    {
        var definition = new SplineDefinition(1, new[] { 0.0, 0, 1, 1 }, isPeriodic: true);
        Assert.ThrowsExactly<ArgumentException>(() => new SplineCurve3d(definition, new[] { new Vector3d(1e12, 0, 0), new Vector3d(1e12 + 1, 0, 0) }));
        Assert.ThrowsExactly<ArgumentException>(() => new ScalarSpline(definition, new[] { 1e12, 1e12 + 1 }));
    }

    internal static SplineCurve3d QuarterCircle(double radius = 1) => SplineCurve3d.Clamped(
        new[] { new Vector3d(radius, 0, 0), new Vector3d(radius, radius, 0), new Vector3d(0, radius, 0) }, 2,
        new[] { 1, System.Math.Sqrt(0.5), 1 });

    private static double Choose(int n, int k)
    {
        if (n < k) { return 0; }
        double result = 1;
        for (int i = 1; i <= k; i++) { result *= (double)(n - i + 1) / i; }
        return result;
    }
}
