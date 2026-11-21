using System;
using System.Collections.Generic;
using System.Linq;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using QuantumDynamix.Math.Functions;
using QuantumDynamix.Math.Timelines;

namespace QuantumDynamix.Tests;

[TestClass]
public sealed class EasingTimelineTests
{
    [TestMethod]
    [DataRow(EasingKind.Linear)]
    [DataRow(EasingKind.Quadratic)]
    [DataRow(EasingKind.Cubic)]
    [DataRow(EasingKind.Quartic)]
    [DataRow(EasingKind.Quintic)]
    [DataRow(EasingKind.Sine)]
    [DataRow(EasingKind.Cosine)]
    public void BuiltInDirectionsHaveNormalizedEndpointsAndAnalyticDerivatives(EasingKind kind)
    {
        foreach (EasingDirection direction in Enum.GetValues<EasingDirection>())
        {
            var easing = new EasingFunction(kind, direction);
            Assert.AreEqual(0, easing.Evaluate(0).Value, 1e-15);
            Assert.AreEqual(1, easing.Evaluate(1).Value, 1e-15);
            const double x = 0.31;
            const double h = 1e-5;
            ScalarJet sample = easing.Evaluate(x);
            Assert.AreEqual((easing.Evaluate(x + h).Value - easing.Evaluate(x - h).Value) / (2 * h), sample.FirstDerivative, 2e-8);
            Assert.AreEqual((easing.Evaluate(x + h).FirstDerivative - easing.Evaluate(x - h).FirstDerivative) / (2 * h), sample.SecondDerivative, 2e-7);
            Assert.AreEqual((easing.Evaluate(x + h).SecondDerivative - easing.Evaluate(x - h).SecondDerivative) / (2 * h), sample.ThirdDerivative, 2e-6);
            double previous = -1;
            for (int i = 0; i <= 50; i++) { double current = easing.Evaluate(i / 50.0).Value; Assert.IsTrue(current >= previous); previous = current; }
        }
    }

    [TestMethod]
    public void PowerInOutReportsItsSecondDerivativeJump()
    {
        var easing = new EasingFunction(EasingKind.Quintic, EasingDirection.InOut);
        Assert.AreEqual(1, easing.Evaluate(0.5).DerivativeOrder);
        Assert.AreEqual(40, easing.Evaluate(0.5, EvaluationSide.Left).SecondDerivative);
        Assert.AreEqual(-40, easing.Evaluate(0.5, EvaluationSide.Right).SecondDerivative);
        Assert.ThrowsExactly<InvalidOperationException>(() => easing.Evaluate(0.5).ThirdDerivative);
    }

    [TestMethod]
    public void SineAndCosineAreSeparatePresetsWithTheApprovedSharedFormulas()
    {
        var sine = new EasingFunction(EasingKind.Sine, EasingDirection.InOut);
        var cosine = new EasingFunction(EasingKind.Cosine, EasingDirection.InOut);
        Assert.AreNotEqual(sine.Kind, cosine.Kind);
        Assert.AreEqual(3, sine.Evaluate(0.5).DerivativeOrder);
        Assert.AreEqual((1 - System.Math.Cos(System.Math.PI * 0.3)) / 2, sine.Evaluate(0.3).Value, 2e-16);
        Assert.AreEqual(sine.Evaluate(0.3), cosine.Evaluate(0.3));
    }

    [TestMethod]
    public void PlateauHasTheSpecifiedIntermediateHoldAndC3Shoulders()
    {
        var plateau = new EasingFunction(EasingKind.Plateau);
        Assert.AreEqual(0, plateau.Evaluate(0).Value);
        Assert.AreEqual(1, plateau.Evaluate(1).Value);
        foreach (double x in new[] { 0, 0.25, 0.5, 0.75, 1 })
        {
            ScalarJet value = plateau.Evaluate(x);
            for (int d = 1; d <= 3; d++) { Assert.AreEqual(0, value.Derivative(d), 0); }
        }
        Assert.AreEqual(0.5, plateau.Evaluate(0.4).Value);
        Assert.AreEqual(0.25, plateau.Evaluate(0.125).Value, 1e-15);
        Assert.AreEqual(0.75, plateau.Evaluate(0.875).Value, 1e-15);
        var outward = new EasingFunction(EasingKind.Plateau, EasingDirection.Out, 0.2, 0.6, 0.3);
        Assert.AreEqual(0.7, outward.Evaluate(0.5).Value, 1e-15);
        var composed = new EasingFunction(EasingKind.Plateau, EasingDirection.InOut);
        Assert.AreEqual(0.25, composed.Evaluate(0.25).Value, 1e-15);
        Assert.AreEqual(0.75, composed.Evaluate(0.75).Value, 1e-15);
        Assert.AreEqual(3, composed.Evaluate(0.5).DerivativeOrder);
    }

    [TestMethod]
    public void RiseHoldFallIsAnEnvelopeRatherThanAnEndpointTransition()
    {
        var envelope = new RiseHoldFallEnvelope();
        Assert.AreEqual(0, envelope.Evaluate(0).Value);
        Assert.AreEqual(0, envelope.Evaluate(1).Value);
        Assert.AreEqual(1, envelope.Evaluate(0.5).Value);
        Assert.IsTrue(envelope.Evaluate(0.1).FirstDerivative > 0);
        Assert.IsTrue(envelope.Evaluate(0.9).FirstDerivative < 0);
        Assert.ThrowsExactly<ArgumentException>(() => FunctionSegment.Transition(0, 2, 0, 10, envelope));
        var profile = new ScalarTimeline(ProfileDomain.Time, new[] { new FunctionSegment(10, 14, envelope, 5) }, BoundaryPolicy.Zero);
        Assert.AreEqual(5, profile.Evaluate(12).Value);
        Assert.AreEqual(0, profile.Evaluate(20).Value);
        Assert.AreEqual(0, profile.ValidateContinuity(3).Count);
    }

    [TestMethod]
    public void CustomFunctionsDeclareTheirDerivativeAvailabilityAndDirections()
    {
        var custom = EasingFunction.Custom((x, _) => new(x * x, 2 * x, 2, 0));
        Assert.AreEqual(0.09, custom.Evaluate(0.3).Value, 1e-15);
        var valueOnly = EasingFunction.Custom((x, _) => new(x, derivativeOrder: 0), EasingDirection.Out);
        Assert.AreEqual(0, valueOnly.Evaluate(0.4).DerivativeOrder);
        Assert.ThrowsExactly<InvalidOperationException>(() => valueOnly.Evaluate(0.4).FirstDerivative);
        Assert.ThrowsExactly<ArgumentException>(() => EasingFunction.Custom((_, _) => new(2)));
    }

    [TestMethod]
    public void PhysicalDomainMappingScalesEachDerivativeAndSupportsDecreasingValues()
    {
        var segment = FunctionSegment.Transition(2, 6, 10, -6, new EasingFunction(EasingKind.Cubic));
        ScalarJet sample = segment.Evaluate(3);
        Assert.AreEqual(9.75, sample.Value, 1e-14);
        Assert.AreEqual(-0.75, sample.FirstDerivative, 1e-14);
        Assert.AreEqual(-1.5, sample.SecondDerivative, 1e-14);
        Assert.AreEqual(-1.5, sample.ThirdDerivative, 1e-14);
    }

    [TestMethod]
    public void SegmentUsesAuthoredNormalizedBreakpointOnlyAtItsExactMappedCoordinate()
    {
        var segment = new FunctionSegment(10, 14, new CustomFunction(0, 1, Kink, new[] { 0.2 }));
        var timeline = new ScalarTimeline(ProfileDomain.Distance, new[] { segment });
        double point = timeline.Breakpoints.Single();

        Assert.AreEqual(10.8, point);
        Assert.AreNotEqual(0.2, (point - 10) / 4);
        Assert.AreEqual(new ScalarJet(0, 0), timeline.Evaluate(point, EvaluationSide.Left));
        Assert.AreEqual(new ScalarJet(0, 0.25), timeline.Evaluate(point, EvaluationSide.Right));
        Assert.AreEqual(0, timeline.Evaluate(point).DerivativeOrder);
        Assert.AreEqual(new ContinuityDiagnostic(point, 0), timeline.ValidateContinuity(1).Single());
        Assert.AreEqual(0, timeline.Evaluate(System.Math.BitDecrement(point), EvaluationSide.Right).FirstDerivative);
        Assert.AreEqual(0.25, timeline.Evaluate(System.Math.BitIncrement(point), EvaluationSide.Left).FirstDerivative);

        static ScalarJet Kink(double x, EvaluationSide side)
        {
            const double breakpoint = 0.2;
            if (x == breakpoint && side == EvaluationSide.Automatic) { return new(0, derivativeOrder: 0); }
            return x > breakpoint || (x == breakpoint && side == EvaluationSide.Right) ? new(x - breakpoint, 1) : new(0);
        }
    }

    [TestMethod]
    [DataRow(EasingDirection.Out, 0.8)]
    [DataRow(EasingDirection.InOut, 0.9)]
    public void ReflectedCustomEasingPreservesExactBreakpointAndReversesLimits(EasingDirection direction, double point)
    {
        var easing = EasingFunction.Custom(Kink, direction, new[] { 0.2 });
        var segment = FunctionSegment.Transition(10, 14, 0, 1, easing);
        double station = segment.Breakpoints[^1];

        Assert.AreEqual(point, easing.Breakpoints[^1]);
        Assert.AreEqual(new ScalarJet(1, 1.25), easing.Evaluate(point, EvaluationSide.Left));
        Assert.AreEqual(new ScalarJet(1, 0), easing.Evaluate(point, EvaluationSide.Right));
        Assert.AreEqual(new ScalarJet(1, 0.3125), segment.Evaluate(station, EvaluationSide.Left));
        Assert.AreEqual(new ScalarJet(1, 0), segment.Evaluate(station, EvaluationSide.Right));
        Assert.AreEqual(0, segment.Evaluate(station).DerivativeOrder);

        static ScalarJet Kink(double x, EvaluationSide side)
        {
            const double breakpoint = 0.2;
            const double slope = 1.25;
            if (x == breakpoint && side == EvaluationSide.Automatic) { return new(0, derivativeOrder: 0); }
            return x > breakpoint || (x == breakpoint && side == EvaluationSide.Right)
                ? new((x - breakpoint) * slope, slope) : new(0);
        }
    }

    [TestMethod]
    public void RoundedBreakpointCollisionsKeepOrdinaryEndpointAndCoordinateEvaluation()
    {
        double middle = System.Math.BitIncrement(1);
        double end = System.Math.BitIncrement(middle);
        var shape = new CustomFunction(0, 1, (u, _) => new(u, 1), new[] { 0.1, 0.4, 0.6, 0.9 });
        var segment = new FunctionSegment(1, end, shape);

        CollectionAssert.AreEqual(new[] { 1, middle, middle, end }, segment.Breakpoints.ToArray());
        Assert.AreEqual(0, segment.Evaluate(1).Value);
        Assert.AreEqual(0.5, segment.Evaluate(middle).Value);
        Assert.AreEqual(1, segment.Evaluate(end).Value);
    }

    [TestMethod]
    public void TimelineReportsInternalEasingJoinsAsWellAsKeyframeJoins()
    {
        var timeline = new ScalarTimeline(ProfileDomain.Distance, new[] {
            FunctionSegment.Transition(0, 10, 0, 1, new EasingFunction(EasingKind.Cubic, EasingDirection.InOut)),
            FunctionSegment.Transition(10, 20, 1, 3) });
        Assert.IsTrue(timeline.ValidateContinuity(2).Any(j => j.Coordinate == 5 && j.ContinuousDerivativeOrder == 1));
        Assert.IsTrue(timeline.ValidateContinuity(1).Any(j => j.Coordinate == 10 && j.ContinuousDerivativeOrder == 0));
        Assert.AreEqual(0, timeline.Evaluate(10).DerivativeOrder);
    }

    [TestMethod]
    public void DiscontinuitiesNeedExplicitPermissionAndReturnRightHandValues()
    {
        IScalarFunction[] segments = { FunctionSegment.Transition(0, 1, 0, 1), FunctionSegment.Transition(1, 2, 4, 5) };
        Assert.ThrowsExactly<ArgumentException>(() => new ScalarTimeline(ProfileDomain.Time, segments));
        var allowed = new ScalarTimeline(ProfileDomain.Time, segments, allowDiscontinuities: true);
        Assert.AreEqual(4, allowed.Evaluate(1).Value);
        Assert.AreEqual(1, allowed.Evaluate(1, EvaluationSide.Left).Value);
        Assert.AreEqual(-1, allowed.ValidateContinuity(0).Single().ContinuousDerivativeOrder);
    }

    [TestMethod]
    public void HoldZeroRejectAndRepeatHaveExplicitBoundaryDerivativeSemantics()
    {
        var hold = ScalarTimeline.FromKeyframes(ProfileDomain.Time, new[] { new TimelineKeyframe(0, 2), new TimelineKeyframe(1, 4) }, BoundaryPolicy.Hold);
        Assert.AreEqual(2, hold.Evaluate(-1).Value);
        Assert.AreEqual(0, hold.Evaluate(0).DerivativeOrder);
        Assert.ThrowsExactly<ArgumentException>(() => new ScalarTimeline(ProfileDomain.Time, hold.Segments, BoundaryPolicy.Zero));
        Assert.ThrowsExactly<ArgumentOutOfRangeException>(() => new ScalarTimeline(ProfileDomain.Time, hold.Segments).Evaluate(-1));
        var repeat = ScalarTimeline.FromKeyframes(ProfileDomain.Distance, new[] { new TimelineKeyframe(0, 0), new TimelineKeyframe(1, 1), new TimelineKeyframe(2, 0) }, BoundaryPolicy.Repeat);
        Assert.AreEqual(0.25, repeat.Evaluate(-0.25).Value, 1e-15);
        Assert.AreEqual(0.25, repeat.Evaluate(2.25).Value, 1e-15);
        Assert.AreEqual(-1, repeat.Evaluate(0, EvaluationSide.Left).FirstDerivative);
        Assert.AreEqual(1, repeat.Evaluate(2, EvaluationSide.Right).FirstDerivative);
        Assert.AreEqual(0, repeat.Evaluate(0).DerivativeOrder);
    }

    [TestMethod]
    public void AnchorsResolveAgainstFreshFeaturesWithoutMutatingAuthoredKeys()
    {
        var keys = new[] {
            new AnchoredKeyframe(SpatialAnchor.BetweenFeatures("entry", "exit", 0), 0),
            new AnchoredKeyframe(SpatialAnchor.BetweenFeatures("entry", "exit", 0.5), 10),
            new AnchoredKeyframe(SpatialAnchor.BetweenFeatures("entry", "exit", 1), 0) };
        var source = new AnchoredTimeline(keys);
        ScalarTimeline first = source.Resolve(Stations(10, 30));
        ScalarTimeline second = source.Resolve(Stations(20, 60));
        Assert.AreEqual(10, first.Evaluate(20).Value);
        Assert.AreEqual(10, second.Evaluate(40).Value);
        Assert.AreEqual(0.5, keys[1].Anchor.Fraction);
        Assert.AreEqual(7, SpatialAnchor.AtDistance(7).Resolve(Stations(20, 60)));
        Assert.ThrowsExactly<KeyNotFoundException>(() => source.Resolve(new FeatureStations(Array.Empty<KeyValuePair<string, double>>())));
        Assert.ThrowsExactly<InvalidOperationException>(() => source.Resolve(Stations(30, 10)));
    }

    [TestMethod]
    public void TimeOriginEvaluationIsStatelessAcrossRepeatedTrialQueries()
    {
        var timeline = ScalarTimeline.FromKeyframes(ProfileDomain.Time, new[] { new TimelineKeyframe(0, 0), new TimelineKeyframe(2, 10) });
        Assert.AreEqual(5, timeline.EvaluateAtTime(101, 100).Value);
        timeline.EvaluateAtTime(101.5, 100);
        Assert.AreEqual(5, timeline.EvaluateAtTime(101, 100).Value);
    }

    [TestMethod]
    public void InvalidDomainsKeysAndFunctionContractsAreRejected()
    {
        Assert.ThrowsExactly<ArgumentException>(() => FunctionSegment.Transition(1, 1, 0, 1));
        Assert.ThrowsExactly<ArgumentException>(() => ScalarTimeline.FromKeyframes(ProfileDomain.Time, new[] { new TimelineKeyframe(1, 0), new TimelineKeyframe(0, 1) }));
        Assert.ThrowsExactly<ArgumentException>(() => new ScalarTimeline(ProfileDomain.Time, new[] { FunctionSegment.Transition(0, 1, 0, 1), FunctionSegment.Transition(2, 3, 1, 2) }));
        Assert.ThrowsExactly<ArgumentOutOfRangeException>(() => new EasingFunction().Evaluate(double.NaN));
        Assert.ThrowsExactly<ArgumentException>(() => new CustomFunction(0, 1, (x, _) => new(x), new[] { 0.5, 0.2 }));
        Assert.ThrowsExactly<ArgumentOutOfRangeException>(() => SpatialAnchor.BetweenFeatures("a", "b", double.NaN));
        Assert.ThrowsExactly<ArgumentException>(() => Stations(double.PositiveInfinity, 1));
        Assert.ThrowsExactly<ArgumentOutOfRangeException>(() => new RiseHoldFallEnvelope().Evaluate(1.1));
    }

    private static FeatureStations Stations(double a, double b) => new(new Dictionary<string, double> { ["entry"] = a, ["exit"] = b });
}
