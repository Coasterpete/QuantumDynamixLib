using System;
using System.Collections.Generic;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using QuantumDynamix.Geometry;
using QuantumDynamix.Math;
using QuantumDynamix.Math.Functions;
using QuantumDynamix.Math.Timelines;
using QuantumDynamix.Splines.Curves;
using QuantumDynamix.Track;

namespace QuantumDynamix.Tests;

[TestClass]
public sealed class RegionTests
{
    [TestMethod]
    public void ChannelsKeepIndependentBreakpointsAndPhysicalDerivatives()
    {
        var cubic = new ScalarTimeline(ProfileDomain.Distance,
            new[] { FunctionSegment.Transition(2, 6, 10, -6, new EasingFunction(EasingKind.Cubic)) });
        var steppedSlope = new ScalarTimeline(ProfileDomain.Distance, new[]
        {
            FunctionSegment.Transition(2, 3, 0, 1),
            FunctionSegment.Transition(3, 6, 1, 1)
        });
        var region = new Region(ProfileDomain.Distance, 2, 6,
            new Dictionary<string, ScalarTimeline> { ["cubic"] = cubic, ["steppedSlope"] = steppedSlope });

        Assert.AreEqual(ProfileDomain.Distance, region.Domain);
        Assert.AreEqual(2, region.Start);
        Assert.AreEqual(6, region.End);
        Assert.AreEqual(0, region.Channels["cubic"].Breakpoints.Count);
        Assert.AreEqual(1, region.Channels["steppedSlope"].Breakpoints.Count);
        Assert.AreEqual(3, region.Channels["steppedSlope"].Breakpoints[0]);
        Assert.AreEqual(new ScalarJet(8, -3, -3, -1.5), region.Channels["cubic"].Evaluate(4));
        Assert.AreEqual(1, region.Channels["steppedSlope"].Evaluate(3, EvaluationSide.Left).FirstDerivative);
        Assert.AreEqual(0, region.Channels["steppedSlope"].Evaluate(3, EvaluationSide.Right).FirstDerivative);
        Assert.AreEqual(0, region.Channels["steppedSlope"].Evaluate(3).DerivativeOrder);
    }

    [TestMethod]
    public void AnchoredBoundsResolveAgainstChangedHeartlineGeometry()
    {
        SpatialAnchor start = SpatialAnchor.BetweenFeatures("entry", "exit", 0.25);
        SpatialAnchor end = SpatialAnchor.BetweenFeatures("entry", "exit", 0.75);
        var authored = new AnchoredTimeline(new[] { new AnchoredKeyframe(start, 0), new AnchoredKeyframe(end, 10) });
        Region original = Resolve(40);
        Region edited = Resolve(80);

        Assert.AreEqual(15, original.Start, 1e-12);
        Assert.AreEqual(25, original.End, 1e-12);
        Assert.AreEqual(30, edited.Start, 1e-12);
        Assert.AreEqual(50, edited.End, 1e-12);
        Assert.AreEqual(5, original.Channels["q"].Evaluate(20).Value, 1e-12);
        Assert.AreEqual(5, edited.Channels["q"].Evaluate(40).Value, 1e-12);
        Assert.AreEqual(1, original.Channels["q"].Evaluate(20).FirstDerivative, 1e-12);
        Assert.AreEqual(0.5, edited.Channels["q"].Evaluate(40).FirstDerivative, 1e-12);

        Region Resolve(double length)
        {
            var curve = SplineCurve3d.Clamped(new[] { Vector3d.Zero, new Vector3d(length, 0, 0) }, 1);
            var track = new HeartlineTrack(curve, Vector3d.UnitZ,
                TrackOffsetProfile.Constant(Vector3d.Zero, new ArcLengthMap(curve).Length),
                features: new[] { new TrackFeature("entry", 0.25), new TrackFeature("exit", 0.75) });
            FeatureStations stations = track.ResolveFeatures();
            return new Region(ProfileDomain.Distance, start.Resolve(stations), end.Resolve(stations),
                new Dictionary<string, ScalarTimeline> { ["q"] = authored.Resolve(stations) });
        }
    }

    [TestMethod]
    public void JoinComparesSidedDerivativesConsecutivelyAndUsesAbsoluteTolerances()
    {
        Region first = WithChannel(ProfileDomain.Distance, FunctionSegment.Transition(0, 5, 0, 5));
        Region next = WithChannel(ProfileDomain.Distance, FunctionSegment.Transition(5, 10, 5, 15));

        RegionJoinDiagnostic strict = first.DiagnoseJoin(next, "q", new[] { 0.0, 0, 0, 0 });
        Assert.AreEqual(new RegionJoinDiagnostic(5, new(5, 1), new(5, 2), 3, 0), strict);

        RegionJoinDiagnostic slopeWithinTolerance = first.DiagnoseJoin(next, "q", new[] { 0.0, 1, 0, 0 });
        Assert.AreEqual(3, slopeWithinTolerance.MatchingOrder);
        Assert.AreEqual(strict.Left, slopeWithinTolerance.Left);
        Assert.AreEqual(strict.Right, slopeWithinTolerance.Right);

        RegionJoinDiagnostic valuesOnly = first.DiagnoseJoin(next, "q", new[] { 0.0 });
        Assert.AreEqual(0, valuesOnly.RequiredOrder);
        Assert.AreEqual(0, valuesOnly.MatchingOrder);
    }

    [TestMethod]
    public void JoinReportsValueMismatchBeforeComparingHigherOrders()
    {
        Region first = WithChannel(ProfileDomain.Distance, FunctionSegment.Transition(0, 5, 0, 5));
        Region next = WithChannel(ProfileDomain.Distance, FunctionSegment.Transition(5, 10, 6, 11));

        RegionJoinDiagnostic result = first.DiagnoseJoin(next, "q", new[] { 0.0, 0 });
        Assert.AreEqual(new RegionJoinDiagnostic(5, new(5, 1), new(6, 1), 1, -1), result);
    }

    [TestMethod]
    public void JoinLeavesUnavailableDerivativesUnverifiedAndPreservesBothJets()
    {
        Region first = WithChannel(ProfileDomain.Distance, FunctionSegment.Transition(0, 5, 0, 5));
        Region next = WithChannel(ProfileDomain.Distance, new CustomFunction(5, 10, (_, _) => new(5, derivativeOrder: 0)));

        RegionJoinDiagnostic result = first.DiagnoseJoin(next, "q", new[] { 0.0, 0, 0, 0 });
        Assert.AreEqual(new RegionJoinDiagnostic(5, new(5, 1), new(5, derivativeOrder: 0), 3, 0), result);
    }

    [TestMethod]
    public void TimeRegionEvaluatesCustomAccelerationPulseAcrossCallerOrigin()
    {
        var shape = new CustomFunction(0, 1, (u, _) => new(4 * u * (1 - u), 4 - 8 * u, -8));
        Region region = WithChannel(ProfileDomain.Time, new FunctionSegment(-2, 2, shape, scale: 2), "acceleration");
        ScalarTimeline acceleration = region.Channels["acceleration"];

        Assert.AreEqual(0, acceleration.Evaluate(-2).Value);
        Assert.AreEqual(0, acceleration.Evaluate(2).Value);
        Assert.AreEqual(2, acceleration.EvaluateAtTime(100, 100).Value);
        Assert.AreEqual(new ScalarJet(1.5, 1, -1), acceleration.EvaluateAtTime(99, 100));
        Assert.ThrowsExactly<ArgumentOutOfRangeException>(() => acceleration.Evaluate(System.Math.BitDecrement(-2)));
    }

    [TestMethod]
    public void ChannelCollectionIsReadOnlyOrdinalAndPreservesDiscontinuities()
    {
        var jump = new ScalarTimeline(ProfileDomain.Time, new[]
        {
            FunctionSegment.Transition(0, 1, 0, 1),
            FunctionSegment.Transition(1, 2, 4, 5)
        }, allowDiscontinuities: true);
        var continuous = new ScalarTimeline(ProfileDomain.Time, new[] { FunctionSegment.Transition(0, 2, 0, 0) });
        var supplied = new Dictionary<string, ScalarTimeline> { ["Bank"] = jump, ["bank"] = continuous };
        var region = new Region(ProfileDomain.Time, 0, 2, supplied);
        supplied.Clear();

        Assert.AreEqual(2, region.Channels.Count);
        Assert.AreSame(jump, region.Channels["Bank"]);
        Assert.AreSame(continuous, region.Channels["bank"]);
        Assert.IsFalse(region.Channels.ContainsKey("BANK"));
        Assert.IsTrue(region.Channels["Bank"].AllowsDiscontinuities);
        Assert.AreEqual(1, region.Channels["Bank"].Evaluate(1, EvaluationSide.Left).Value);
        Assert.AreEqual(4, region.Channels["Bank"].Evaluate(1, EvaluationSide.Right).Value);
        Assert.ThrowsExactly<NotSupportedException>(() => ((IDictionary<string, ScalarTimeline>)region.Channels).Clear());
    }

    [TestMethod]
    [DataRow((ProfileDomain)99, 0.0, 1.0)]
    [DataRow(ProfileDomain.Distance, -1.0, 1.0)]
    [DataRow(ProfileDomain.Time, double.NaN, 1.0)]
    [DataRow(ProfileDomain.Time, 0.0, double.PositiveInfinity)]
    [DataRow(ProfileDomain.Time, 1.0, 0.0)]
    [DataRow(ProfileDomain.Time, 1.0, 1.0)]
    [DataRow(ProfileDomain.Time, -double.MaxValue, double.MaxValue)]
    public void ConstructorRejectsInvalidDomainsAndBounds(ProfileDomain domain, double start, double end)
    {
        Region valid = WithChannel(ProfileDomain.Time, FunctionSegment.Transition(0, 1, 0, 0));
        Assert.ThrowsExactly<ArgumentOutOfRangeException>(() => new Region(domain, start, end, valid.Channels));
    }

    [TestMethod]
    public void ConstructorRejectsInvalidChannelCollections()
    {
        Region valid = WithChannel(ProfileDomain.Time, FunctionSegment.Transition(0, 1, 0, 0));
        ScalarTimeline channel = valid.Channels["q"];
        Assert.ThrowsExactly<ArgumentNullException>(() => new Region(ProfileDomain.Time, 0, 1, null!));
        Assert.ThrowsExactly<ArgumentException>(() => new Region(ProfileDomain.Time, 0, 1, Array.Empty<KeyValuePair<string, ScalarTimeline>>()));
        Assert.ThrowsExactly<ArgumentException>(() => new Region(ProfileDomain.Time, 0, 1,
            new Dictionary<string, ScalarTimeline> { [" "] = channel }));
        Assert.ThrowsExactly<ArgumentException>(() => new Region(ProfileDomain.Time, 0, 1,
            new Dictionary<string, ScalarTimeline> { ["q"] = null! }));
        var pair = new KeyValuePair<string, ScalarTimeline>("q", channel);
        Assert.ThrowsExactly<ArgumentException>(() => new Region(ProfileDomain.Time, 0, 1, new[] { pair, pair }));
    }

    [TestMethod]
    public void ConstructorRequiresMatchingDomainExactEndpointsAndRejectBoundaryPolicy()
    {
        Region valid = WithChannel(ProfileDomain.Time, FunctionSegment.Transition(2, 6, 0, 0));
        Assert.ThrowsExactly<ArgumentException>(() => new Region(ProfileDomain.Distance, 2, 6, valid.Channels));
        Assert.ThrowsExactly<ArgumentException>(() => new Region(ProfileDomain.Time, System.Math.BitDecrement(2), 6, valid.Channels));
        Assert.ThrowsExactly<ArgumentException>(() => new Region(ProfileDomain.Time, 2, System.Math.BitDecrement(6), valid.Channels));

        foreach (BoundaryPolicy boundary in new[] { BoundaryPolicy.Hold, BoundaryPolicy.Zero, BoundaryPolicy.Repeat })
        {
            var channel = new ScalarTimeline(ProfileDomain.Time, valid.Channels["q"].Segments, boundary);
            Assert.ThrowsExactly<ArgumentException>(() => new Region(ProfileDomain.Time, 2, 6,
                new Dictionary<string, ScalarTimeline> { ["q"] = channel }));
        }
    }

    [TestMethod]
    public void JoinRequiresSameDomainExactAdjacencyAndACommonChannel()
    {
        Region first = WithChannel(ProfileDomain.Distance, FunctionSegment.Transition(0, 5, 0, 0));
        Region next = WithChannel(ProfileDomain.Distance, FunctionSegment.Transition(5, 10, 0, 0));
        Region otherChannel = WithChannel(ProfileDomain.Distance, FunctionSegment.Transition(5, 10, 0, 0), "Q");
        Region otherDomain = WithChannel(ProfileDomain.Time, FunctionSegment.Transition(5, 10, 0, 0));
        Region gap = WithChannel(ProfileDomain.Distance, FunctionSegment.Transition(System.Math.BitIncrement(5), 10, 0, 0));
        Region overlap = WithChannel(ProfileDomain.Distance, FunctionSegment.Transition(System.Math.BitDecrement(5), 10, 0, 0));
        double[] tolerances = { 0 };

        Assert.ThrowsExactly<ArgumentNullException>(() => first.DiagnoseJoin(null!, "q", tolerances));
        Assert.ThrowsExactly<ArgumentNullException>(() => first.DiagnoseJoin(next, null!, tolerances));
        Assert.ThrowsExactly<ArgumentException>(() => first.DiagnoseJoin(otherDomain, "q", tolerances));
        Assert.ThrowsExactly<ArgumentException>(() => first.DiagnoseJoin(gap, "q", tolerances));
        Assert.ThrowsExactly<ArgumentException>(() => first.DiagnoseJoin(overlap, "q", tolerances));
        Assert.ThrowsExactly<ArgumentException>(() => first.DiagnoseJoin(otherChannel, "q", tolerances));
        Assert.ThrowsExactly<ArgumentException>(() => first.DiagnoseJoin(otherChannel, "Q", tolerances));
    }

    [TestMethod]
    public void JoinRejectsInvalidToleranceInputs()
    {
        Region first = WithChannel(ProfileDomain.Time, FunctionSegment.Transition(0, 1, 0, 0));
        Region next = WithChannel(ProfileDomain.Time, FunctionSegment.Transition(1, 2, 0, 0));
        Assert.ThrowsExactly<ArgumentNullException>(() => first.DiagnoseJoin(next, "q", null!));
        Assert.ThrowsExactly<ArgumentException>(() => first.DiagnoseJoin(next, "q", Array.Empty<double>()));
        Assert.ThrowsExactly<ArgumentException>(() => first.DiagnoseJoin(next, "q", new double[5]));
        foreach (double invalid in new[] { -1, double.NaN, double.PositiveInfinity })
        {
            Assert.ThrowsExactly<ArgumentOutOfRangeException>(() => first.DiagnoseJoin(next, "q", new[] { 0.0, invalid }));
        }
    }

    private static Region WithChannel(ProfileDomain domain, IScalarFunction segment, string id = "q") =>
        new(domain, segment.Minimum, segment.Maximum,
            new Dictionary<string, ScalarTimeline> { [id] = new(domain, new[] { segment }) });
}
