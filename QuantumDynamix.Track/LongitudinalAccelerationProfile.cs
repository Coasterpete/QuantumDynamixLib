using System;
using System.Collections.Generic;
using System.Linq;
using QuantumDynamix.Math.Functions;
using QuantumDynamix.Math.Timelines;

namespace QuantumDynamix.Track;

/// <summary>One acceleration comparison. Equal Region indices identify a breakpoint inside a Region.</summary>
public readonly record struct AccelerationJoinDiagnostic(int LeftRegionIndex, int RightRegionIndex, RegionJoinDiagnostic Comparison);

/// <summary>Adjacent authored Regions on one time or heartline-station axis, bound to the same acceleration channel.</summary>
public sealed class LongitudinalAccelerationProfile
{
    private readonly AccelerationJoinDiagnostic[] joins;
    private readonly double[] breakpoints;
    private readonly TrajectoryPanel[] panels;
    private readonly int requiredContinuityOrder;

    public ProfileDomain Domain { get; }
    public double Start { get; }
    public double End { get; }
    public IReadOnlyList<LongitudinalAccelerationBinding> Regions { get; }
    /// <summary>Interior Region boundaries and selected-channel breakpoints, in ascending coordinate order.</summary>
    public IReadOnlyList<double> Breakpoints { get; }
    /// <summary>Sided jets compared through order three, subject to derivative availability.</summary>
    public IReadOnlyList<AccelerationJoinDiagnostic> Joins { get; }

    /// <param name="regions">Ordered bindings on the same coordinate axis, with exactly adjacent Region boundaries.</param>
    /// <param name="requiredContinuityOrder">The lowest matching order required to integrate. -1 allows acceleration steps.</param>
    /// <param name="absoluteContinuityTolerances">Four finite, nonnegative tolerances in native jet units, starting with acceleration.</param>
    public LongitudinalAccelerationProfile(IEnumerable<LongitudinalAccelerationBinding> regions,
        int requiredContinuityOrder, IReadOnlyList<double> absoluteContinuityTolerances)
    {
        ArgumentNullException.ThrowIfNull(regions);
        ArgumentNullException.ThrowIfNull(absoluteContinuityTolerances);
        if (requiredContinuityOrder is < -1 or > 3) { throw new ArgumentOutOfRangeException(nameof(requiredContinuityOrder)); }
        double[] tolerances = absoluteContinuityTolerances.ToArray();
        if (tolerances.Length != 4 || tolerances.Any(value => !double.IsFinite(value) || value < 0))
        { throw new ArgumentException("Supply four finite, nonnegative derivative-order tolerances.", nameof(absoluteContinuityTolerances)); }

        LongitudinalAccelerationBinding[] bindings = regions.ToArray();
        if (bindings.Length == 0 || bindings.Any(binding => binding is null))
        { throw new ArgumentException("Supply one or more non-null acceleration bindings.", nameof(regions)); }
        Domain = bindings[0].Domain;
        Start = bindings[0].Region.Start;
        End = bindings[^1].Region.End;
        string channelId = bindings[0].ChannelId;
        for (int i = 1; i < bindings.Length; i++)
        {
            if (bindings[i].Domain != Domain || !string.Equals(bindings[i].ChannelId, channelId, StringComparison.Ordinal)
                || bindings[i - 1].Region.End != bindings[i].Region.Start)
            { throw new ArgumentException("Bindings must have a common domain and channel ID and be ordered and exactly adjacent.", nameof(regions)); }
        }
        if (!double.IsFinite(End - Start)) { throw new ArgumentException("The profile span must be finite.", nameof(regions)); }

        this.requiredContinuityOrder = requiredContinuityOrder;
        Regions = Array.AsReadOnly(bindings);
        var panelList = new List<TrajectoryPanel>();
        var comparisons = new List<AccelerationJoinDiagnostic>();
        for (int i = 0; i < bindings.Length; i++)
        {
            LongitudinalAccelerationBinding binding = bindings[i];
            if (i > 0)
            { comparisons.Add(new(i - 1, i, bindings[i - 1].Region.DiagnoseJoin(binding.Region, channelId, tolerances))); }

            ScalarTimeline timeline = binding.Region.Channels[channelId];
            double start = binding.Region.Start;
            int segmentIndex = 0;
            foreach (double end in timeline.Breakpoints.Append(binding.Region.End))
            {
                while (timeline.Segments[segmentIndex].Maximum <= start) { segmentIndex++; }
                panelList.Add(new(start, end, i, binding, timeline.Segments[segmentIndex]));
                if (end < binding.Region.End)
                {
                    ScalarJet left = binding.Evaluate(end, EvaluationSide.Left);
                    ScalarJet right = binding.Evaluate(end, EvaluationSide.Right);
                    comparisons.Add(new(i, i, Region.CompareLimits(end, left, right, tolerances)));
                }
                start = end;
            }
        }

        panels = panelList.ToArray();
        joins = comparisons.ToArray();
        breakpoints = joins.Select(join => join.Comparison.Coordinate).ToArray();
        Joins = Array.AsReadOnly(joins);
        Breakpoints = Array.AsReadOnly(breakpoints);
    }

    /// <summary>Evaluates acceleration in m/s² and available derivatives per authored second or per heartline metre.</summary>
    /// <remarks>Automatic uses the right value and matching derivative order at a join. Both endpoints use inward limits.</remarks>
    public ScalarJet Evaluate(double coordinate, EvaluationSide side = EvaluationSide.Automatic)
    {
        Check.Argument(coordinate, Start, End, side);
        int index = Array.BinarySearch(breakpoints, coordinate);
        if (index < 0) { return panels[~index].Binding.Evaluate(coordinate, side); }
        RegionJoinDiagnostic comparison = joins[index].Comparison;
        return side switch
        {
            EvaluationSide.Left => comparison.Left,
            EvaluationSide.Right => comparison.Right,
            _ => comparison.Right.LimitOrder(System.Math.Max(0, comparison.MatchingOrder))
        };
    }

    /// <summary>Integrates from profile start with signed velocity, including rest and reversal.</summary>
    public LongitudinalTrajectory IntegrateTime(TimeMotionInitialConditions initial, TimeTrajectorySettings settings)
    {
        ArgumentNullException.ThrowIfNull(settings);
        TrajectoryValidation.Nonnegative(initial.HeartlineStationMetres, nameof(initial));
        Check.Finite(initial.HeartlineVelocityMetresPerSecond, nameof(initial));
        RequireIntegrable(ProfileDomain.Time);
        return LongitudinalTrajectory.BuildTime(this, panels, initial, settings);
    }

    /// <summary>Integrates a strictly positive forward traversal from the first to the last authored station.</summary>
    public LongitudinalTrajectory IntegrateDistance(double initialForwardSpeedMetresPerSecond, DistanceTrajectorySettings settings,
        AccelerationRangeProvider? customRanges = null)
    {
        ArgumentNullException.ThrowIfNull(settings);
        TrajectoryValidation.Positive(initialForwardSpeedMetresPerSecond, nameof(initialForwardSpeedMetresPerSecond));
        RequireIntegrable(ProfileDomain.Distance);
        return LongitudinalTrajectory.BuildDistance(this, panels, initialForwardSpeedMetresPerSecond, settings, customRanges);
    }

    internal int RegionIndexAt(double coordinate, EvaluationSide side)
    {
        int index = Array.BinarySearch(breakpoints, coordinate);
        if (index < 0) { return panels[~index].RegionIndex; }
        return side == EvaluationSide.Left ? joins[index].LeftRegionIndex : joins[index].RightRegionIndex;
    }

    private void RequireIntegrable(ProfileDomain domain)
    {
        if (Domain != domain) { throw new InvalidOperationException($"This integration method requires a {domain} profile."); }
        AccelerationJoinDiagnostic[] failures = joins.Where(join => join.Comparison.MatchingOrder < requiredContinuityOrder).ToArray();
        if (failures.Length != 0)
        {
            throw new TrajectoryException(TrajectoryFailureCode.ContinuityRequirementFailed,
                "The acceleration profile fails its required continuity order.", Domain,
                failures[0].Comparison.Coordinate, failures[^1].Comparison.Coordinate, 0, failures[0].LeftRegionIndex,
                continuityFailures: failures);
        }
    }
}

internal readonly record struct TrajectoryPanel(double Start, double End, int RegionIndex,
    LongitudinalAccelerationBinding Binding, IScalarFunction Segment)
{
    internal ScalarJet Evaluate(double coordinate) => Binding.Evaluate(coordinate, coordinate == End ? EvaluationSide.Left : EvaluationSide.Right);
}
