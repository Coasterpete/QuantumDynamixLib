using System;
using System.Collections.Generic;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using QuantumDynamix.Math.Functions;
using QuantumDynamix.Math.Timelines;
using QuantumDynamix.Track;

namespace QuantumDynamix.Tests;

[TestClass]
public sealed class LongitudinalAccelerationBindingTests
{
    private const string AccelerationChannelId = "acceleration";

    [TestMethod]
    public void CustomTimePulseReturnsAvailableTimeDerivativesAcrossOutOfOrderQueries()
    {
        var pulse = new CustomFunction(0, 1, (u, _) =>
            new(4 * u * (1 - u), 4 - 8 * u, -8, derivativeOrder: 2));
        Region region = CreateRegion(new(ProfileDomain.Time,
            new[] { new FunctionSegment(-2, 2, pulse, scale: -2) }));
        var binding = new LongitudinalAccelerationBinding(region, AccelerationChannelId);

        Assert.AreEqual(ProfileDomain.Time, binding.Domain);
        ScalarJet sample = binding.Evaluate(-1);
        Assert.AreEqual(new ScalarJet(-1.5, -1, 1, derivativeOrder: 2), sample);
        Assert.AreEqual(2, sample.DerivativeOrder);
        Assert.ThrowsExactly<InvalidOperationException>(() => sample.Derivative(3));
        Assert.AreEqual(new ScalarJet(-1.5, 1, 1, derivativeOrder: 2), binding.Evaluate(1));
        Assert.AreEqual(-2.0, binding.Evaluate(0).Value);
        Assert.AreEqual(sample, binding.Evaluate(-1));
    }

    [TestMethod]
    public void DistanceCubicReturnsStationDerivativesAndInwardEndpointLimits()
    {
        var cubic = FunctionSegment.Transition(10, 18, 1, -15, new EasingFunction(EasingKind.Cubic));
        Region region = CreateRegion(new(ProfileDomain.Distance, new[] { cubic }));
        var binding = new LongitudinalAccelerationBinding(region, AccelerationChannelId);

        Assert.AreEqual(ProfileDomain.Distance, binding.Domain);
        Assert.AreEqual(new ScalarJet(-1, -1.5, -0.75, -0.1875), binding.Evaluate(14));
        foreach (EvaluationSide side in Enum.GetValues<EvaluationSide>())
        {
            Assert.AreEqual(new ScalarJet(1, 0, 0, -0.1875), binding.Evaluate(10, side));
            Assert.AreEqual(new ScalarJet(-15, -6, -1.5, -0.1875), binding.Evaluate(18, side));
        }

        Assert.ThrowsExactly<ArgumentOutOfRangeException>(() => binding.Evaluate(System.Math.BitDecrement(10)));
        Assert.ThrowsExactly<ArgumentOutOfRangeException>(() => binding.Evaluate(System.Math.BitIncrement(18)));
    }

    [TestMethod]
    public void InteriorJoinsPreserveSidedJetsAndExplicitValueJumps()
    {
        var incoming = FunctionSegment.Transition(0, 1, 0, 1);
        Region continuous = CreateRegion(new(ProfileDomain.Time,
            new[] { incoming, FunctionSegment.Transition(1, 2, 1, 3) }));
        Region stepped = CreateRegion(new(ProfileDomain.Time,
            new[] { incoming, FunctionSegment.Transition(1, 2, 4, 6) }, allowDiscontinuities: true));
        var continuousBinding = new LongitudinalAccelerationBinding(continuous, AccelerationChannelId);
        var steppedBinding = new LongitudinalAccelerationBinding(stepped, AccelerationChannelId);

        Assert.AreEqual(new ScalarJet(1, 1), continuousBinding.Evaluate(1, EvaluationSide.Left));
        Assert.AreEqual(new ScalarJet(1, 2), continuousBinding.Evaluate(1, EvaluationSide.Right));
        ScalarJet continuousJoin = continuousBinding.Evaluate(1);
        Assert.AreEqual(1.0, continuousJoin.Value);
        Assert.AreEqual(0, continuousJoin.DerivativeOrder);

        Assert.AreEqual(new ScalarJet(1, 1), steppedBinding.Evaluate(1, EvaluationSide.Left));
        Assert.AreEqual(new ScalarJet(4, 2), steppedBinding.Evaluate(1, EvaluationSide.Right));
        ScalarJet steppedJoin = steppedBinding.Evaluate(1);
        Assert.AreEqual(4.0, steppedJoin.Value);
        Assert.AreEqual(0, steppedJoin.DerivativeOrder);
    }

    [TestMethod]
    public void BindingSelectsExactChannelIdAndRejectsInvalidInputs()
    {
        var channels = new Dictionary<string, ScalarTimeline>
        {
            ["drive"] = new(ProfileDomain.Time, new[] { FunctionSegment.Transition(0, 1, -2, -2) }),
            ["Drive"] = new(ProfileDomain.Time, new[] { FunctionSegment.Transition(0, 1, 3, 3) })
        };
        var region = new Region(ProfileDomain.Time, 0, 1, channels);
        var drive = new LongitudinalAccelerationBinding(region, "drive");
        var capitalizedDrive = new LongitudinalAccelerationBinding(region, "Drive");

        Assert.AreSame(region, drive.Region);
        Assert.AreEqual("drive", drive.ChannelId);
        Assert.AreEqual(-2.0, drive.Evaluate(0.5).Value);
        Assert.AreEqual(3.0, capitalizedDrive.Evaluate(0.5).Value);

        Assert.ThrowsExactly<ArgumentNullException>(() => new LongitudinalAccelerationBinding(null!, "drive"));
        Assert.ThrowsExactly<ArgumentNullException>(() => new LongitudinalAccelerationBinding(region, null!));
        Assert.ThrowsExactly<ArgumentException>(() => new LongitudinalAccelerationBinding(region, ""));
        Assert.ThrowsExactly<ArgumentException>(() => new LongitudinalAccelerationBinding(region, " \t"));
        Assert.ThrowsExactly<ArgumentException>(() => new LongitudinalAccelerationBinding(region, "DRIVE"));
        foreach (double coordinate in new[] { double.NaN, double.NegativeInfinity, double.PositiveInfinity })
        {
            Assert.ThrowsExactly<ArgumentOutOfRangeException>(() => drive.Evaluate(coordinate));
        }

        Assert.ThrowsExactly<ArgumentOutOfRangeException>(() => drive.Evaluate(0.5, (EvaluationSide)99));
    }

    [TestMethod]
    public void CustomEasingRetainsSidedLimitsAtItsMappedBreakpoint()
    {
        var easing = EasingFunction.Custom((u, side) =>
        {
            if (u == 0.25 && side == EvaluationSide.Automatic)
            {
                return new(0.625, derivativeOrder: 0);
            }

            return u < 0.25 || (u == 0.25 && side == EvaluationSide.Left)
                ? new(2.5 * u, 2.5)
                : new(0.625 + 0.5 * (u - 0.25), 0.5);
        }, breakpoints: new[] { 0.25 });
        var segment = FunctionSegment.Transition(2, 10, -6, 2, easing);
        Region region = CreateRegion(new(ProfileDomain.Distance, new[] { segment }));
        var binding = new LongitudinalAccelerationBinding(region, AccelerationChannelId);

        Assert.AreEqual(new ScalarJet(-1, 2.5), binding.Evaluate(4, EvaluationSide.Left));
        Assert.AreEqual(new ScalarJet(-1, 0.5), binding.Evaluate(4, EvaluationSide.Right));
        Assert.AreEqual(new ScalarJet(-1, derivativeOrder: 0), binding.Evaluate(4));
    }

    [TestMethod]
    public void AdjacentAccelerationRegionsSampleAndDiagnoseMatchingAndMismatchedLimits()
    {
        var rise = FunctionSegment.Transition(0, 4, -3, 1, new EasingFunction(EasingKind.Quadratic, EasingDirection.Out));
        var fall = FunctionSegment.Transition(4, 8, 1, -3, new EasingFunction(EasingKind.Quadratic));
        Region first = CreateRegion(new(ProfileDomain.Distance, new[] { rise }));
        Region second = CreateRegion(new(ProfileDomain.Distance, new[] { fall }));
        var incoming = new LongitudinalAccelerationBinding(first, AccelerationChannelId);
        var outgoing = new LongitudinalAccelerationBinding(second, AccelerationChannelId);

        Assert.AreEqual(new ScalarJet(-3, 2, -0.5), incoming.Evaluate(0));
        Assert.AreEqual(new ScalarJet(0, 1, -0.5), incoming.Evaluate(2));
        Assert.AreEqual(new ScalarJet(0, -1, -0.5), outgoing.Evaluate(6));
        Assert.AreEqual(new ScalarJet(-3, -2, -0.5), outgoing.Evaluate(8));

        double[] exactTolerances = { 0, 0, 0, 0 };
        var commonLimit = new ScalarJet(1, 0, -0.5);
        RegionJoinDiagnostic matching = incoming.Region.DiagnoseJoin(outgoing.Region, incoming.ChannelId, exactTolerances);
        Assert.AreEqual(new RegionJoinDiagnostic(4, commonLimit, commonLimit, 3, 3), matching);
        Assert.AreEqual(matching.Left, incoming.Evaluate(4, EvaluationSide.Left));
        Assert.AreEqual(matching.Right, outgoing.Evaluate(4, EvaluationSide.Right));

        var linearFall = FunctionSegment.Transition(4, 8, 1, -3);
        Region mismatchedSecond = CreateRegion(new(ProfileDomain.Distance, new[] { linearFall }));
        var mismatchedOutgoing = new LongitudinalAccelerationBinding(mismatchedSecond, AccelerationChannelId);
        RegionJoinDiagnostic mismatch = incoming.Region.DiagnoseJoin(
            mismatchedOutgoing.Region, incoming.ChannelId, exactTolerances);
        Assert.AreEqual(new RegionJoinDiagnostic(4, commonLimit, new ScalarJet(1, -1), 3, 0), mismatch);
        Assert.AreEqual(mismatch.Right, mismatchedOutgoing.Evaluate(4, EvaluationSide.Right));
    }

    private static Region CreateRegion(ScalarTimeline timeline) =>
        new(timeline.Domain, timeline.Minimum, timeline.Maximum,
            new Dictionary<string, ScalarTimeline> { [AccelerationChannelId] = timeline });
}
