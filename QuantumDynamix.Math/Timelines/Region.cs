using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using QuantumDynamix.Math.Functions;

namespace QuantumDynamix.Math.Timelines;

/// <summary>The limits of one channel at a shared Region boundary and their verified continuity.</summary>
/// <param name="Coordinate">The shared boundary in heartline metres or authoring seconds.</param>
/// <param name="Left">The incoming limit from smaller coordinates.</param>
/// <param name="Right">The outgoing limit from larger coordinates.</param>
/// <param name="RequiredOrder">The highest derivative order requested, with zero denoting the value.</param>
/// <param name="MatchingOrder">The highest consecutive order verified within tolerance, or -1 for a value mismatch.</param>
public readonly record struct RegionJoinDiagnostic(
    double Coordinate, ScalarJet Left, ScalarJet Right, int RequiredOrder, int MatchingOrder);

/// <summary>A resolved interval of heartline metres or caller-relative authoring seconds with named scalar channels.</summary>
public sealed class Region
{
    public ProfileDomain Domain { get; }
    public double Start { get; }
    public double End { get; }
    public IReadOnlyDictionary<string, ScalarTimeline> Channels { get; }

    public Region(ProfileDomain domain, double start, double end,
        IEnumerable<KeyValuePair<string, ScalarTimeline>> channels)
    {
        if (!Enum.IsDefined(domain)) { throw new ArgumentOutOfRangeException(nameof(domain)); }
        Check.Finite(start, nameof(start));
        if (domain == ProfileDomain.Distance && start < 0)
        {
            throw new ArgumentOutOfRangeException(nameof(start), "Heartline station must be nonnegative.");
        }
        if (!double.IsFinite(end) || end <= start || !double.IsFinite(end - start))
        {
            throw new ArgumentOutOfRangeException(nameof(end), "The Region must have a finite, positive span.");
        }
        ArgumentNullException.ThrowIfNull(channels);

        var copy = new Dictionary<string, ScalarTimeline>(StringComparer.Ordinal);
        foreach ((string id, ScalarTimeline timeline) in channels)
        {
            if (string.IsNullOrWhiteSpace(id))
            {
                throw new ArgumentException("Channel identifiers must be nonblank.", nameof(channels));
            }
            if (timeline is null || timeline.Domain != domain || timeline.Boundary != BoundaryPolicy.Reject
                || timeline.Minimum != start || timeline.Maximum != end)
            {
                throw new ArgumentException($"Channel '{id}' must have the Region's domain and exact endpoints, and use Reject boundaries.", nameof(channels));
            }
            if (!copy.TryAdd(id, timeline))
            {
                throw new ArgumentException($"Duplicate channel identifier '{id}'.", nameof(channels));
            }
        }
        if (copy.Count == 0) { throw new ArgumentException("Supply at least one named channel.", nameof(channels)); }

        Domain = domain;
        Start = start;
        End = end;
        Channels = new ReadOnlyDictionary<string, ScalarTimeline>(copy);
    }

    /// <summary>Compares the inward endpoint limits of a common channel across adjacent Regions.</summary>
    /// <param name="next">The Region starting exactly at this Region's end on the same domain and coordinate axis.</param>
    /// <param name="channelId">The case-sensitive identifier of the same quantity in both Regions.</param>
    /// <param name="absoluteTolerances">One to four finite, nonnegative tolerances, in derivative order starting with the value.</param>
    /// <remarks>The caller supplies compatible coordinates and channel units. Unavailable derivatives are left unverified.</remarks>
    public RegionJoinDiagnostic DiagnoseJoin(Region next, string channelId, IReadOnlyList<double> absoluteTolerances)
    {
        ArgumentNullException.ThrowIfNull(next);
        ArgumentNullException.ThrowIfNull(channelId);
        ArgumentNullException.ThrowIfNull(absoluteTolerances);
        if (Domain != next.Domain || End != next.Start)
        {
            throw new ArgumentException("The next Region must start exactly at this Region's end on the same domain.", nameof(next));
        }
        if (!Channels.TryGetValue(channelId, out ScalarTimeline? leftChannel)
            || !next.Channels.TryGetValue(channelId, out ScalarTimeline? rightChannel))
        {
            throw new ArgumentException($"Channel '{channelId}' must be present in both Regions.", nameof(channelId));
        }

        int count = absoluteTolerances.Count;
        if (count is < 1 or > 4)
        {
            throw new ArgumentException("Supply one to four tolerances, starting with the value tolerance.", nameof(absoluteTolerances));
        }
        var tolerances = new double[count];
        for (int order = 0; order < count; order++)
        {
            double tolerance = absoluteTolerances[order];
            if (!double.IsFinite(tolerance) || tolerance < 0)
            {
                throw new ArgumentOutOfRangeException(nameof(absoluteTolerances), $"Tolerance at order {order} must be finite and nonnegative.");
            }
            tolerances[order] = tolerance;
        }

        ScalarJet left = leftChannel.Evaluate(End, EvaluationSide.Left);
        ScalarJet right = rightChannel.Evaluate(next.Start, EvaluationSide.Right);
        return CompareLimits(End, left, right, tolerances);
    }

    internal static RegionJoinDiagnostic CompareLimits(double coordinate, ScalarJet left, ScalarJet right,
        ReadOnlySpan<double> absoluteTolerances)
    {
        int requiredOrder = absoluteTolerances.Length - 1;
        int lastComparableOrder = System.Math.Min(requiredOrder, System.Math.Min(left.DerivativeOrder, right.DerivativeOrder));
        int matchingOrder = -1;
        for (int order = 0; order <= lastComparableOrder; order++)
        {
            if (System.Math.Abs(left.Derivative(order) - right.Derivative(order)) > absoluteTolerances[order]) { break; }
            matchingOrder = order;
        }
        return new(coordinate, left, right, requiredOrder, matchingOrder);
    }
}
