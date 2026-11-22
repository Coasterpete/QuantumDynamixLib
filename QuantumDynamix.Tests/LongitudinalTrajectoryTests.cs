using System;
using System.Collections.Generic;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using QuantumDynamix.FVD;
using QuantumDynamix.Geometry;
using QuantumDynamix.Math;
using QuantumDynamix.Math.Functions;
using QuantumDynamix.Math.Timelines;
using QuantumDynamix.Splines;
using QuantumDynamix.Splines.Curves;
using QuantumDynamix.Track;

namespace QuantumDynamix.Tests;

[TestClass]
public sealed class LongitudinalTrajectoryTests
{
    private const double Tolerance = 1e-9;
    private const string Channel = "acceleration";
    private static readonly TimeTrajectorySettings TimeSettings = new(Tolerance, Tolerance, 20, 100_000);
    private static readonly DistanceTrajectorySettings DistanceSettings = new(Tolerance, Tolerance, 20, 100_000);

    [TestMethod]
    public void SignedTimeMotionStopsReversesAndCanStartFromRest()
    {
        LongitudinalTrajectory reversing = ConstantTime(3, -2).IntegrateTime(new(10, 4), TimeSettings);
        AssertMotion(reversing.Sample(2), 2, 14, 0, -2);
        AssertMotion(reversing.Sample(3), 3, 13, -2, -2);

        LongitudinalTrajectory stationary = ConstantTime(3, 0).IntegrateTime(new(10, 0), TimeSettings);
        AssertMotion(stationary.Sample(3), 3, 10, 0, 0);
        LongitudinalTrajectory starting = ConstantTime(3, 2).IntegrateTime(new(10, 0), TimeSettings);
        AssertMotion(starting.Sample(3), 3, 19, 6, 2);
        LongitudinalTrajectory backing = ConstantTime(3, 0).IntegrateTime(new(0, -2), TimeSettings);
        AssertMotion(backing.Sample(3), 3, -6, -2, 0);
    }

    [TestMethod]
    public void TimeRegionsUseIndependentLinearAndQuadraticAuthoringAndExactJoins()
    {
        LongitudinalAccelerationBinding first = Bind(ProfileDomain.Time, 10, 12, new IScalarFunction[]
        {
            FunctionSegment.Transition(10, 11, 0, 2),
            FunctionSegment.Transition(11, 12, 2, 0, new EasingFunction(EasingKind.Quadratic))
        });
        var profile = Profile(-1, first, Constant(ProfileDomain.Time, 12, 13, -2));
        LongitudinalTrajectory trajectory = profile.IntegrateTime(new(100, 3), TimeSettings);

        CollectionAssert.AreEqual(new[] { 11.0, 12 }, new List<double>(profile.Breakpoints));
        Assert.AreEqual(0, profile.Joins[0].Comparison.MatchingOrder);
        Assert.AreEqual(-1, profile.Joins[1].Comparison.MatchingOrder);
        Assert.AreEqual(profile.Joins[0].LeftRegionIndex, profile.Joins[0].RightRegionIndex);
        Assert.AreEqual(3, profile.Joins[1].Comparison.RequiredOrder);
        Assert.AreEqual(new ScalarJet(2, 2), profile.Evaluate(11, EvaluationSide.Left));
        Assert.AreEqual(new ScalarJet(2, 0, -4), profile.Evaluate(11, EvaluationSide.Right));
        ScalarJet automatic = profile.Evaluate(11);
        Assert.AreEqual(2.0, automatic.Value);
        Assert.AreEqual(0, automatic.DerivativeOrder);
        Assert.ThrowsExactly<InvalidOperationException>(() => automatic.Derivative(1));
        Assert.AreEqual(new ScalarJet(0, -4, -4), profile.Evaluate(12, EvaluationSide.Left));
        Assert.AreEqual(new ScalarJet(-2, derivativeOrder: 0), trajectory.EvaluateTimeAcceleration(12));
        Assert.AreEqual(profile.Evaluate(11.5), trajectory.EvaluateTimeAcceleration(11.5));
        Assert.AreEqual(1.5, profile.Evaluate(11.5).Value);

        AssertMotion(trajectory.Sample(10, EvaluationSide.Left), 0, 100, 3, 0);
        AssertMotion(trajectory.Sample(11), 1, 310.0 / 3, 4, 2);
        LongitudinalMotionSample incoming = trajectory.Sample(12, EvaluationSide.Left);
        LongitudinalMotionSample outgoing = trajectory.Sample(12, EvaluationSide.Right);
        AssertMotion(incoming, 2, 649.0 / 6, 16.0 / 3, 0);
        Assert.AreEqual(incoming with { LongitudinalAccelerationMetresPerSecondSquared = -2 }, outgoing);
        Assert.AreEqual(outgoing, trajectory.Sample(12));
        AssertMotion(trajectory.Sample(13, EvaluationSide.Right), 3, 112.5, 10.0 / 3, -2);
        Assert.IsTrue(profile.Evaluate(System.Math.BitDecrement(12)).Value > 0);
        Assert.AreEqual(-2.0, profile.Evaluate(System.Math.BitIncrement(12)).Value);
        Assert.ThrowsExactly<ArgumentOutOfRangeException>(() => trajectory.Sample(System.Math.BitIncrement(13)));
    }

    [TestMethod]
    public void RequiredContinuityReportsAllFailuresAndLeavesProfileInspectable()
    {
        var ramp = Bind(ProfileDomain.Time, 1, 3, new IScalarFunction[]
        {
            FunctionSegment.Transition(1, 2, 2, 3), FunctionSegment.Transition(2, 3, 3, 3)
        });
        var profile = Profile(1, Constant(ProfileDomain.Time, 0, 1, 1), ramp);
        Assert.AreEqual(2, profile.Joins.Count);
        Assert.AreEqual(2.0, profile.Evaluate(1).Value);
        var failure = Assert.ThrowsExactly<TrajectoryException>(() => profile.IntegrateTime(new(0, 0), TimeSettings));
        Assert.AreEqual(TrajectoryFailureCode.ContinuityRequirementFailed, failure.Code);
        Assert.AreEqual(0, failure.WorkUsed);
        Assert.AreEqual(-1, failure.ContinuityFailures[0].Comparison.MatchingOrder);
        Assert.AreEqual(0, failure.ContinuityFailures[1].Comparison.MatchingOrder);
    }

    [TestMethod]
    public void ProfileUsesAbsoluteContinuityTolerancesAndPreservesUnavailableDerivatives()
    {
        var left = Constant(ProfileDomain.Time, 0, 1, 1024);
        var right = Bind(ProfileDomain.Time, 1, 2,
            new[] { new CustomFunction(1, 2, (_, _) => new(1024.125, derivativeOrder: 0)) });
        var profile = new LongitudinalAccelerationProfile(new[] { left, right }, 0, new[] { 0.125, 0.0, 0, 0 });
        var join = profile.Joins[0].Comparison;
        Assert.AreEqual(0, join.MatchingOrder);
        Assert.AreEqual(3, join.Left.DerivativeOrder);
        Assert.AreEqual(0, join.Right.DerivativeOrder);
        Assert.AreEqual(new ScalarJet(1024.125, derivativeOrder: 0), profile.Evaluate(1));
        AssertMotion(profile.IntegrateTime(new(0, 0), TimeSettings).Sample(1), 1, 512, 1024, 1024.125);
    }

    [TestMethod]
    public void ProfileRejectsInvalidCompositionAndSnapshotsBindings()
    {
        var first = Constant(ProfileDomain.Time, 0, 1, 0);
        var next = Constant(ProfileDomain.Time, 1, 2, 0);
        var ordered = new[] { first, next };
        LongitudinalAccelerationProfile profile = Profile(0, ordered);
        ordered[0] = next;
        Assert.AreSame(first, profile.Regions[0]);
        Assert.ThrowsExactly<ArgumentException>(() => Profile(0));
        Assert.ThrowsExactly<ArgumentException>(() => Profile(0, next, first));
        Assert.ThrowsExactly<ArgumentException>(() => Profile(0, first, Constant(ProfileDomain.Time, 0.5, 2, 0)));
        Assert.ThrowsExactly<ArgumentException>(() => Profile(0, first, Constant(ProfileDomain.Time, System.Math.BitIncrement(1), 2, 0)));
        Assert.ThrowsExactly<ArgumentException>(() => Profile(0, first, Constant(ProfileDomain.Distance, 1, 2, 0)));
        Assert.ThrowsExactly<ArgumentException>(() => Profile(0, first,
            Bind(ProfileDomain.Time, 1, 2, new[] { FunctionSegment.Transition(1, 2, 0, 0) }, channel: "Acceleration")));
        Assert.ThrowsExactly<ArgumentOutOfRangeException>(() => Profile(-2, first));
        Assert.ThrowsExactly<ArgumentException>(() => new LongitudinalAccelerationProfile(new[] { first }, 1, new[] { 0.0 }));
        Assert.ThrowsExactly<InvalidOperationException>(() => profile.IntegrateDistance(1, DistanceSettings));
    }

    [TestMethod]
    public void CustomEasingBreakpointIsSidedAndIntegratedAcrossItsMappedCoordinate()
    {
        const double breakpoint = 0.3;
        var easing = EasingFunction.Custom((x, side) => x < breakpoint || x == breakpoint && side == EvaluationSide.Left
            ? new(0, derivativeOrder: 0) : new(1, derivativeOrder: 0), breakpoints: new[] { breakpoint });
        var segment = FunctionSegment.Transition(10, 12, 0, 2, easing);
        var profile = Profile(-1, Bind(ProfileDomain.Time, 10, 12, new[] { segment }, allowSteps: true));
        double join = segment.Breakpoints[0];
        Assert.AreEqual(join, profile.Breakpoints[0]);
        Assert.AreEqual(0.0, profile.Evaluate(join, EvaluationSide.Left).Value);
        Assert.AreEqual(2.0, profile.Evaluate(join).Value);
        AssertMotion(profile.IntegrateTime(new(0, 0), TimeSettings).Sample(12), 2, (12 - join) * (12 - join), 2 * (12 - join), 2);
    }

    [TestMethod]
    public void ForwardTrajectoryCrossesExactAccelerationStepWithAnalyticalMotion()
    {
        LongitudinalTrajectory trajectory = ForwardExample().IntegrateDistance(1, DistanceSettings);
        AssertMotion(trajectory.Sample(10, EvaluationSide.Left), 0, 10, 1, 2);
        LongitudinalMotionSample incoming = trajectory.Sample(12, EvaluationSide.Left);
        LongitudinalMotionSample outgoing = trajectory.Sample(12, EvaluationSide.Right);
        AssertMotion(incoming, 1, 12, 3, 2);
        Assert.AreEqual(incoming with { LongitudinalAccelerationMetresPerSecondSquared = -1 }, outgoing);
        Assert.AreEqual(outgoing, trajectory.Sample(12));
        LongitudinalMotionSample end = trajectory.Sample(16, EvaluationSide.Right);
        AssertMotion(end, 3, 16, 1, -1);
        Assert.AreEqual(0.0, end.EstimatedAbsoluteError.StationMetres);
        Assert.IsTrue(System.Math.Abs(end.ElapsedSeconds - 3) <= end.EstimatedAbsoluteError.ElapsedTimeSeconds);
        Assert.IsTrue(System.Math.Abs(end.HeartlineVelocityMetresPerSecond - 1) <= end.EstimatedAbsoluteError.VelocityMetresPerSecond);
    }

    [TestMethod]
    public void DistanceAccelerationDerivativesUseSolvedSpeedAndChainRule()
    {
        var ramp = FunctionSegment.Transition(0, 2, 1, 3);
        var profile = Profile(0, Bind(ProfileDomain.Distance, 0, 2, new[] { ramp }));
        LongitudinalTrajectory trajectory = profile.IntegrateDistance(2, DistanceSettings);
        Assert.AreEqual(new ScalarJet(1, 1), profile.Evaluate(0));
        Assert.AreEqual(new ScalarJet(1, 2, 1, 2), trajectory.EvaluateTimeAcceleration(0));
        double station = 1;
        double expectedSpeed = System.Math.Sqrt(7);
        Assert.AreEqual(expectedSpeed, trajectory.Sample(station).HeartlineVelocityMetresPerSecond, Tolerance);
        Assert.AreEqual(expectedSpeed, trajectory.EvaluateTimeAcceleration(station).FirstDerivative, Tolerance);
    }

    [TestMethod]
    [DataRow(EasingKind.Linear, EasingDirection.Out)]
    [DataRow(EasingKind.Quadratic, EasingDirection.In)]
    [DataRow(EasingKind.Cubic, EasingDirection.InOut)]
    [DataRow(EasingKind.Quartic, EasingDirection.Out)]
    [DataRow(EasingKind.Quintic, EasingDirection.In)]
    [DataRow(EasingKind.Sine, EasingDirection.In)]
    [DataRow(EasingKind.Cosine, EasingDirection.InOut)]
    [DataRow(EasingKind.Plateau, EasingDirection.InOut)]
    public void BuiltInEasingSupportsForwardMotionWithoutCallerBounds(EasingKind kind, EasingDirection direction)
    {
        var segment = FunctionSegment.Transition(0, 2, -1, 1, new EasingFunction(kind, direction));
        var profile = Profile(0, Bind(ProfileDomain.Distance, 0, 2, new[] { segment }));
        LongitudinalMotionSample result = profile.IntegrateDistance(3, DistanceSettings).Sample(2);
        Assert.AreEqual(2.0, result.HeartlineStationMetres);
        Assert.IsTrue(result.HeartlineVelocityMetresPerSecond > 0);
        Assert.AreEqual(1.0, result.LongitudinalAccelerationMetresPerSecondSquared);
        if (direction == EasingDirection.InOut || kind == EasingKind.Linear)
        { Assert.AreEqual(3.0, result.HeartlineVelocityMetresPerSecond, Tolerance); }
    }

    [TestMethod]
    public void ForwardValidationRefinesAConservativeBuiltInRangeToProvePositiveSpeed()
    {
        var segment = FunctionSegment.Transition(0, 2, -1, 1);
        var profile = Profile(0, Bind(ProfileDomain.Distance, 0, 2, new[] { segment }));
        // q(s) = 2.25 - 2s + s², with a positive minimum of 1.25 at s = 1.
        LongitudinalTrajectory trajectory = profile.IntegrateDistance(1.5, DistanceSettings);
        Assert.AreEqual(System.Math.Sqrt(1.25), trajectory.Sample(1).HeartlineVelocityMetresPerSecond, Tolerance);
        Assert.AreEqual(1.5, trajectory.Sample(2).HeartlineVelocityMetresPerSecond, Tolerance);
    }

    [TestMethod]
    public void RiseHoldFallEnclosuresHandleBothSlopesAndScaleSign()
    {
        var envelope = new FunctionSegment(0, 4, new RiseHoldFallEnvelope(), scale: -1);
        var profile = Profile(0, Bind(ProfileDomain.Distance, 0, 4, new[] { envelope }));
        // Each shoulder has area 1/2 by reflection, and the hold has length 2.
        LongitudinalMotionSample end = profile.IntegrateDistance(3, DistanceSettings).Sample(4);
        Assert.AreEqual(System.Math.Sqrt(3), end.HeartlineVelocityMetresPerSecond, Tolerance);
    }

    [TestMethod]
    public void DistanceAcceptsBrakingBeforeStopAndRejectsTheTerminalZero()
    {
        LongitudinalTrajectory moving = Profile(0, Constant(ProfileDomain.Distance, 0, 3, -2)).IntegrateDistance(4, DistanceSettings);
        AssertMotion(moving.Sample(3), 1, 3, 2, -2);
        var failure = Assert.ThrowsExactly<TrajectoryException>(() =>
            Profile(0, Constant(ProfileDomain.Distance, 0, 4, -2)).IntegrateDistance(4, DistanceSettings));
        Assert.AreEqual(TrajectoryFailureCode.NonPositiveSpeed, failure.Code);
        Assert.AreEqual(4.0, failure.StartCoordinate);
        Assert.AreEqual(4.0, failure.EndCoordinate);
        Assert.AreEqual((0.0, 0.0), failure.SquaredSpeedEnclosure!.Value);
    }

    [TestMethod]
    [DataRow(2.0, 0.0)]
    [DataRow(3.0, -2.0)]
    public void DistanceDetectsInteriorZeroAndNegativeSquaredSpeedDespiteRecovery(double accelerationMagnitude, double expectedSquaredSpeed)
    {
        var profile = Profile(-1, Constant(ProfileDomain.Distance, 0, 1, -accelerationMagnitude),
            Constant(ProfileDomain.Distance, 1, 2, accelerationMagnitude));
        var failure = Assert.ThrowsExactly<TrajectoryException>(() => profile.IntegrateDistance(2, DistanceSettings));
        Assert.AreEqual(TrajectoryFailureCode.NonPositiveSpeed, failure.Code);
        Assert.AreEqual(1.0, failure.StartCoordinate);
        Assert.AreEqual((expectedSquaredSpeed, expectedSquaredSpeed), failure.SquaredSpeedEnclosure!.Value);
    }

    [TestMethod]
    public void InteriorNegativeSpeedSquaredCanBeEstablishedInsideOneSmoothEasing()
    {
        var segment = FunctionSegment.Transition(0, 2, -2, 2);
        var profile = Profile(0, Bind(ProfileDomain.Distance, 0, 2, new[] { segment }));
        var failure = Assert.ThrowsExactly<TrajectoryException>(() => profile.IntegrateDistance(1, DistanceSettings));
        Assert.AreEqual(TrajectoryFailureCode.NonPositiveSpeed, failure.Code);
        Assert.IsTrue(failure.StartCoordinate > 0 && failure.EndCoordinate < 2);
        Assert.IsTrue(failure.SquaredSpeedEnclosure!.Value.Upper <= 0);
    }

    [TestMethod]
    public void ExactSquaredSpeedCheckKeepsRoundingFromInventingAStop()
    {
        double forwardSpeed = 1 + System.Math.ScaleB(1, -27);
        double roundedSquare = forwardSpeed * forwardSpeed;
        var profile = Profile(0, Constant(ProfileDomain.Distance, 0, 1, -roundedSquare / 2));
        var failure = Assert.ThrowsExactly<TrajectoryException>(() => profile.IntegrateDistance(forwardSpeed, DistanceSettings));
        // The mathematical endpoint q is 2^-54, even though the rounded input square cancels the exact integral.
        Assert.AreEqual(TrajectoryFailureCode.NumericalBudgetExceeded, failure.Code);
        Assert.IsTrue(failure.SquaredSpeedEnclosure!.Value.Lower > 0);
    }

    [TestMethod]
    public void NearZeroWithLooseValidRangesRemainsUnresolved()
    {
        double acceleration = -(1 - System.Math.ScaleB(1, -40)) / 2;
        var settings = new DistanceTrajectorySettings(Tolerance, Tolerance, 3, 2_000);
        var profile = CustomDistance(new CustomFunction(0, 1, (_, _) => new(acceleration)));
        var failure = Assert.ThrowsExactly<TrajectoryException>(() => profile.IntegrateDistance(1, settings, (_, _, _) => new(-0.5, 0)));
        Assert.AreEqual(TrajectoryFailureCode.SpeedPositivityUnresolved, failure.Code);
        Assert.AreEqual(1.0, failure.EndCoordinate);
        Assert.AreEqual(0.0, failure.SquaredSpeedEnclosure!.Value.Lower);
        Assert.IsTrue(failure.SquaredSpeedEnclosure.Value.Upper > 0);
    }

    [TestMethod]
    public void CustomEasingNeedsBoundsAndCanDriveAnAnalyticalDistanceTrajectory()
    {
        var custom = EasingFunction.Custom((x, _) => new(x, 1));
        var profile = Profile(0, Bind(ProfileDomain.Distance, 0, 1, new[] { FunctionSegment.Transition(0, 1, 0, 1, custom) }));
        var missing = Assert.ThrowsExactly<TrajectoryException>(() => profile.IntegrateDistance(1, DistanceSettings));
        Assert.AreEqual(TrajectoryFailureCode.MissingAccelerationBounds, missing.Code);
        var end = profile.IntegrateDistance(1, DistanceSettings, (_, left, right) => new(left, right)).Sample(1);
        AssertMotion(end, System.Math.Asinh(1), 1, System.Math.Sqrt(2), 1);
    }

    [TestMethod]
    public void RangeDiagnosticsIdentifyMissingInvalidAndContradictoryBounds()
    {
        var segment = new CustomFunction(0, 1, (s, _) => new(4 * s * (1 - s)));
        var profile = CustomDistance(segment);
        CheckFailure((_, _, _) => null, TrajectoryFailureCode.MissingAccelerationBounds);
        CheckFailure((_, _, _) => new(1, 0), TrajectoryFailureCode.InvalidAccelerationBounds);
        CheckFailure((_, _, _) => new(double.NaN, 1), TrajectoryFailureCode.InvalidAccelerationBounds);
        var inner = new InvalidOperationException("provider failed");
        Assert.AreSame(inner, CheckFailure((_, _, _) => throw inner, TrajectoryFailureCode.InvalidAccelerationBounds).InnerException);
        TrajectoryException inconsistent = CheckFailure((_, _, _) => new(0, 0.5), TrajectoryFailureCode.InvalidAccelerationBounds);
        Assert.AreEqual(0.5, inconsistent.StartCoordinate);
        Assert.AreEqual(0.5, inconsistent.EndCoordinate);

        TrajectoryException CheckFailure(AccelerationRangeProvider provider, TrajectoryFailureCode code)
        {
            var failure = Assert.ThrowsExactly<TrajectoryException>(() => profile.IntegrateDistance(1, DistanceSettings, provider));
            Assert.AreEqual(code, failure.Code);
            Assert.AreEqual(0, failure.RegionIndex);
            return failure;
        }
    }

    [TestMethod]
    public void NestedDistanceQuadraturePreservesTheAuthoredException()
    {
        // The first elapsed-time node is 0.5. Its squared-speed integral first evaluates at 0.25.
        var callbackError = new ArithmeticException("acceleration unavailable at the nested node");
        var profile = CustomDistance(new CustomFunction(0, 1, (s, _) =>
            s == 0.25 ? throw callbackError : new(0)));

        var failure = Assert.ThrowsExactly<TrajectoryException>(() =>
            profile.IntegrateDistance(2, DistanceSettings, (_, _, _) => new(0, 0)));

        Assert.AreEqual(TrajectoryFailureCode.EvaluationFailed, failure.Code);
        Assert.AreEqual(ProfileDomain.Distance, failure.Domain);
        Assert.AreEqual(0, failure.RegionIndex);
        Assert.AreEqual(0.25, failure.StartCoordinate);
        Assert.AreEqual(0.25, failure.EndCoordinate);
        Assert.AreEqual(20, failure.WorkUsed); // Three proof charges, 15 endpoint nodes, and one node in each nested rule.
        Assert.AreSame(callbackError, failure.InnerException);
    }

    [TestMethod]
    public void NestedDistanceQuadratureRejectsAnObservationOutsideTheSuppliedRange()
    {
        var profile = CustomDistance(new CustomFunction(0, 1, (s, _) => new(s == 0.25 ? 1 : 0)));

        var failure = Assert.ThrowsExactly<TrajectoryException>(() =>
            profile.IntegrateDistance(2, DistanceSettings, (_, _, _) => new(0, 0)));

        Assert.AreEqual(TrajectoryFailureCode.InvalidAccelerationBounds, failure.Code);
        Assert.AreEqual(ProfileDomain.Distance, failure.Domain);
        Assert.AreEqual(0, failure.RegionIndex);
        Assert.AreEqual(0.25, failure.StartCoordinate);
        Assert.AreEqual(0.25, failure.EndCoordinate);
        Assert.AreEqual(20, failure.WorkUsed);
        Assert.IsNull(failure.InnerException);
    }

    [TestMethod]
    public void NestedDistanceQuadratureRequiresEnoughWorkForTheNextRule()
    {
        int interiorEvaluations = 0;
        var profile = CustomDistance(new CustomFunction(0, 1, (s, _) =>
        {
            if (s > 0 && s < 1) { interiorEvaluations++; }
            return new(0);
        }));
        var settings = new DistanceTrajectorySettings(Tolerance, Tolerance, 20, 33);

        var failure = Assert.ThrowsExactly<TrajectoryException>(() =>
            profile.IntegrateDistance(2, settings, (_, _, _) => new(0, 0)));

        Assert.AreEqual(TrajectoryFailureCode.NumericalBudgetExceeded, failure.Code);
        Assert.AreEqual(ProfileDomain.Distance, failure.Domain);
        Assert.AreEqual(0, failure.RegionIndex);
        Assert.AreEqual(0.0, failure.StartCoordinate);
        Assert.AreEqual(0.5, failure.EndCoordinate);
        Assert.AreEqual(19, failure.WorkUsed); // The first outer node leaves 14 charges for the 15-node inner rule.
        Assert.AreEqual(15, interiorEvaluations);
        Assert.IsNull(failure.InnerException);
    }

    [TestMethod]
    public void DistanceQueriesCheckDirectlyObservedAccelerationAgainstTheProviderRange()
    {
        // One deliberately false range, contradicted at a specific coordinate outside the construction nodes.
        const double point = 0.123456789;
        var profile = CustomDistance(new CustomFunction(0, 1, (s, _) => new(s == point ? 1 : 0, derivativeOrder: 0)));
        LongitudinalTrajectory trajectory = profile.IntegrateDistance(2, DistanceSettings, (_, _, _) => new(0, 0));
        var failure = Assert.ThrowsExactly<TrajectoryException>(() => trajectory.Sample(point));
        Assert.AreEqual(TrajectoryFailureCode.InvalidAccelerationBounds, failure.Code);
        Assert.AreEqual(1, failure.WorkUsed);
    }

    [TestMethod]
    public void QueryOrderAndSidedSamplesDoNotMutateTrajectoryPrefixes()
    {
        LongitudinalTrajectory distance = ForwardExample().IntegrateDistance(1, DistanceSettings);
        LongitudinalTrajectory time = ConstantTime(3, -2).IntegrateTime(new(10, 4), TimeSettings);
        AssertReproducible(time, new[] { 0.5, 1.25, 2.875, 3 });
        AssertReproducible(distance, new[] { 10.625, 12, 14.25, 16 });

        static void AssertReproducible(LongitudinalTrajectory trajectory, double[] coordinates)
        {
            var reference = new List<LongitudinalMotionSample>();
            foreach (double coordinate in coordinates) { reference.Add(trajectory.Sample(coordinate)); }
            for (int i = coordinates.Length - 1; i >= 0; i--)
            {
                _ = trajectory.Sample(coordinates[i], EvaluationSide.Left);
                Assert.AreEqual(reference[i], trajectory.Sample(coordinates[i]));
            }
        }
    }

    [TestMethod]
    public void TimeAndNestedDistanceIntegrationShareTheWholeWorkLimit()
    {
        var timeFailure = Assert.ThrowsExactly<TrajectoryException>(() =>
            ConstantTime(1, 1).IntegrateTime(new(0, 0), new(Tolerance, Tolerance, 20, 29)));
        Assert.AreEqual(TrajectoryFailureCode.NumericalBudgetExceeded, timeFailure.Code);
        Assert.AreEqual(15, timeFailure.WorkUsed);

        var distance = Profile(0, Constant(ProfileDomain.Distance, 0, 1, 0));
        var distanceFailure = Assert.ThrowsExactly<TrajectoryException>(() =>
            distance.IntegrateDistance(1, new(Tolerance, Tolerance, 20, 40)));
        Assert.AreEqual(TrajectoryFailureCode.NumericalBudgetExceeded, distanceFailure.Code);
        Assert.AreEqual(33, distanceFailure.WorkUsed);

        var rangeFailure = Assert.ThrowsExactly<TrajectoryException>(() =>
            CustomDistance(new CustomFunction(0, 1, (_, _) => new(0))).IntegrateDistance(1,
                new(Tolerance, Tolerance, 20, 2), (_, _, _) => new(0, 0)));
        Assert.AreEqual(TrajectoryFailureCode.SpeedPositivityUnresolved, rangeFailure.Code);
        Assert.AreEqual(2, rangeFailure.WorkUsed);
    }

    [TestMethod]
    public void TimeErrorEstimatesTrackAnalyticalSinusoidalMotionAndAccumulationRoundoff()
    {
        var sinusoid = new CustomFunction(0, 20, (t, _) => new(System.Math.Sin(t), System.Math.Cos(t)));
        var profile = Profile(0, Bind(ProfileDomain.Time, 0, 20, new[] { sinusoid }));
        LongitudinalMotionSample end = profile.IntegrateTime(new(50, 3), TimeSettings).Sample(20);
        Assert.IsTrue(System.Math.Abs(end.HeartlineStationMetres - (130 - System.Math.Sin(20))) <= end.EstimatedAbsoluteError.StationMetres);
        Assert.IsTrue(System.Math.Abs(end.HeartlineVelocityMetresPerSecond - (4 - System.Math.Cos(20))) <= end.EstimatedAbsoluteError.VelocityMetresPerSecond);
        Assert.AreEqual(0.0, end.EstimatedAbsoluteError.ElapsedTimeSeconds);
        Assert.IsTrue(end.EstimatedAbsoluteError.StationMetres <= TimeSettings.StationAbsoluteToleranceMetres);
        Assert.IsTrue(end.EstimatedAbsoluteError.VelocityMetresPerSecond <= TimeSettings.VelocityAbsoluteToleranceMetresPerSecond);

        var failure = Assert.ThrowsExactly<TrajectoryException>(() => ConstantTime(1, 0).IntegrateTime(new(1e16, 1), TimeSettings));
        Assert.AreEqual(TrajectoryFailureCode.NumericalBudgetExceeded, failure.Code);
        Assert.IsTrue(failure.EstimatedAbsoluteError!.Value.StationMetres >= 1);
    }

    [TestMethod]
    public void EvaluationFailureAndUnrepresentableSquaredSpeedHaveTypedDiagnostics()
    {
        var callbackError = new InvalidOperationException("acceleration unavailable");
        var bad = new CustomFunction(0, 1, (_, _) => throw callbackError);
        var failure = Assert.ThrowsExactly<TrajectoryException>(() =>
            Profile(0, Bind(ProfileDomain.Time, 0, 1, new[] { bad })).IntegrateTime(new(1, 1), TimeSettings));
        Assert.AreEqual(TrajectoryFailureCode.EvaluationFailed, failure.Code);
        Assert.AreSame(callbackError, failure.InnerException);
        Assert.AreEqual(0.5, failure.StartCoordinate);

        var profile = Profile(0, Constant(ProfileDomain.Distance, 0, 1, 0));
        Assert.AreEqual(TrajectoryFailureCode.NumericalRangeExceeded,
            Assert.ThrowsExactly<TrajectoryException>(() => profile.IntegrateDistance(double.Epsilon, DistanceSettings)).Code);
    }

    [TestMethod]
    public void FvdUsesTrajectoryMotionAndSeparatesGravityFromCoordinateAcceleration()
    {
        var incline = CreateTrack(SplineCurve3d.Clamped(new[] { Vector3d.Zero, new Vector3d(30, 0, 40) }, degree: 1));
        Vector3d gravity = new(0, 0, -10);
        var forward = ForwardExample().IntegrateDistance(1, DistanceSettings);
        HeartlineForceSample force = HeartlineForceEvaluator.Evaluate(incline, forward.Sample(12), gravity, EvaluationSide.Automatic);
        Assert.AreEqual(3.0, force.HeartlineSpeed, Tolerance);
        TestMath.Near(new(-0.6, 0, -0.8), force.WorldCoordinateAcceleration, Tolerance);
        TestMath.Near(new(7, 0, 6), force.TrackSpecificForce, Tolerance);

        var coast = ConstantTime(2, 0).IntegrateTime(new(10, -2), TimeSettings);
        HeartlineForceSample coasting = HeartlineForceEvaluator.Evaluate(incline, coast.Sample(1), gravity, EvaluationSide.Right);
        TestMath.Near(Vector3d.Zero, coasting.WorldCoordinateAcceleration, Tolerance);
        TestMath.Near(new(8, 0, 6), coasting.TrackSpecificForce, Tolerance);
    }

    [TestMethod]
    public void FvdCurvatureIsIndependentOfVelocitySignAndSupportsRest()
    {
        var curve = CreateTrack(SplineFoundationTests.QuarterCircle(10));
        LongitudinalAccelerationProfile profile = ConstantTime(1, 1);
        foreach (double initialVelocity in new[] { -10.0, 0, 10 })
        {
            LongitudinalMotionSample motion = profile.IntegrateTime(new(0, initialVelocity), TimeSettings).Sample(0);
            HeartlineForceSample result = HeartlineForceEvaluator.Evaluate(curve, motion, Vector3d.Zero, EvaluationSide.Left);
            Assert.AreEqual(System.Math.Abs(initialVelocity), result.HeartlineSpeed);
            TestMath.Near(new(-initialVelocity * initialVelocity / 10, 1, 0), result.WorldCoordinateAcceleration, Tolerance);
        }
    }

    [TestMethod]
    public void FvdGeometrySideIsIndependentOfAuthoringSideAndRejectsOffTrackMotion()
    {
        var curve = CreateTrack(new SplineCurve3d(new SplineDefinition(2, new[] { 0.0, 0, 0, 1, 2, 2, 2 }),
            new[] { Vector3d.Zero, new Vector3d(1, 0, 0), new Vector3d(3, 0, 0), new Vector3d(4, 1, 0) }));
        double knot = curve.HeartlineStations.StationAt(1);
        var trajectory = Profile(-1, Constant(ProfileDomain.Distance, 0, knot, 1),
            Constant(ProfileDomain.Distance, knot, 4, -1)).IntegrateDistance(2, DistanceSettings);
        LongitudinalMotionSample incoming = trajectory.Sample(knot, EvaluationSide.Left);
        LongitudinalMotionSample outgoing = trajectory.Sample(knot, EvaluationSide.Right);
        HeartlineForceSample outgoingGeometry = HeartlineForceEvaluator.Evaluate(curve, incoming, Vector3d.Zero, EvaluationSide.Right);
        HeartlineForceSample incomingGeometry = HeartlineForceEvaluator.Evaluate(curve, outgoing, Vector3d.Zero, EvaluationSide.Left);
        TestMath.Near(new(1, (4 + 2 * knot) / 2, 0), outgoingGeometry.WorldCoordinateAcceleration, Tolerance);
        TestMath.Near(new(-1, 0, 0), incomingGeometry.WorldCoordinateAcceleration, Tolerance);

        var offTrack = ConstantTime(2, 0).IntegrateTime(new(curve.HeartlineStations.Length - 1, 2), TimeSettings).Sample(1);
        Assert.ThrowsExactly<ArgumentOutOfRangeException>(() =>
            HeartlineForceEvaluator.Evaluate(curve, offTrack, Vector3d.Zero, EvaluationSide.Automatic));
    }

    [TestMethod]
    public void SettingsAndInitialConditionsRequireFiniteValuesAndExplicitBudgets()
    {
        Assert.ThrowsExactly<ArgumentOutOfRangeException>(() => new TimeTrajectorySettings(double.NaN, 1, 1, 1));
        Assert.ThrowsExactly<ArgumentOutOfRangeException>(() => new DistanceTrajectorySettings(1, 0, 1, 1));
        Assert.ThrowsExactly<ArgumentOutOfRangeException>(() => new DistanceTrajectorySettings(1, 1, 0, 1));
        Assert.ThrowsExactly<ArgumentOutOfRangeException>(() => new TimeTrajectorySettings(1, 1, 1, 0));
        Assert.ThrowsExactly<ArgumentOutOfRangeException>(() => new TrajectoryErrorEstimate(0, -1, 0));
        Assert.ThrowsExactly<ArgumentOutOfRangeException>(() => ConstantTime(1, 0).IntegrateTime(new(-1, 0), TimeSettings));
        Assert.ThrowsExactly<ArgumentOutOfRangeException>(() => ConstantTime(1, 0).IntegrateTime(new(0, double.PositiveInfinity), TimeSettings));
        Assert.ThrowsExactly<ArgumentOutOfRangeException>(() => ForwardExample().IntegrateDistance(0, DistanceSettings));
        Assert.ThrowsExactly<ArgumentOutOfRangeException>(() => ForwardExample().IntegrateDistance(-1, DistanceSettings));
    }

    private static LongitudinalAccelerationProfile ForwardExample() => Profile(-1,
        Constant(ProfileDomain.Distance, 10, 12, 2), Constant(ProfileDomain.Distance, 12, 16, -1));

    private static LongitudinalAccelerationProfile ConstantTime(double end, double acceleration) =>
        Profile(0, Constant(ProfileDomain.Time, 0, end, acceleration));

    private static LongitudinalAccelerationProfile CustomDistance(CustomFunction function) =>
        Profile(0, Bind(ProfileDomain.Distance, function.Minimum, function.Maximum, new[] { function }));

    private static LongitudinalAccelerationProfile Profile(int requiredOrder, params LongitudinalAccelerationBinding[] bindings) =>
        new(bindings, requiredOrder, new[] { 0.0, 0, 0, 0 });

    private static LongitudinalAccelerationBinding Constant(ProfileDomain domain, double start, double end, double acceleration) =>
        Bind(domain, start, end, new[] { FunctionSegment.Transition(start, end, acceleration, acceleration) });

    private static LongitudinalAccelerationBinding Bind(ProfileDomain domain, double start, double end,
        IEnumerable<IScalarFunction> segments, bool allowSteps = false, string channel = Channel)
    {
        var timeline = new ScalarTimeline(domain, segments, allowDiscontinuities: allowSteps);
        return new(new Region(domain, start, end, new Dictionary<string, ScalarTimeline> { [channel] = timeline }), channel);
    }

    private static HeartlineTrack CreateTrack(SplineCurve3d curve) =>
        new(curve, Vector3d.UnitZ, TrackOffsetProfile.Constant(Vector3d.Zero, new ArcLengthMap(curve).Length));

    private static void AssertMotion(LongitudinalMotionSample sample, double elapsed, double station, double velocity, double acceleration)
    {
        Assert.AreEqual(elapsed, sample.ElapsedSeconds, Tolerance);
        Assert.AreEqual(station, sample.HeartlineStationMetres, Tolerance);
        Assert.AreEqual(velocity, sample.HeartlineVelocityMetresPerSecond, Tolerance);
        Assert.AreEqual(acceleration, sample.LongitudinalAccelerationMetresPerSecondSquared, Tolerance);
    }
}
