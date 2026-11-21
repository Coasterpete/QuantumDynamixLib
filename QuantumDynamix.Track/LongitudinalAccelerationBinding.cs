using System;
using QuantumDynamix.Math.Functions;
using QuantumDynamix.Math.Timelines;

namespace QuantumDynamix.Track;

/// <summary>Binds a Region channel as signed longitudinal coordinate acceleration in m/s².</summary>
/// <remarks>Positive acceleration points toward increasing heartline station regardless of the direction of travel.</remarks>
public sealed class LongitudinalAccelerationBinding
{
    private readonly ScalarTimeline timeline;

    public Region Region { get; }
    public string ChannelId { get; }
    public ProfileDomain Domain => Region.Domain;

    public LongitudinalAccelerationBinding(Region region, string channelId)
    {
        ArgumentNullException.ThrowIfNull(region);
        ArgumentException.ThrowIfNullOrWhiteSpace(channelId);
        if (!region.Channels.TryGetValue(channelId, out ScalarTimeline? selectedTimeline))
        {
            throw new ArgumentException($"Channel '{channelId}' must be present in the Region.", nameof(channelId));
        }

        Region = region;
        ChannelId = channelId;
        timeline = selectedTimeline;
    }

    /// <summary>Returns acceleration and its available derivatives with respect to the Region coordinate.</summary>
    /// <param name="coordinate">Caller-relative authoring seconds or heartline metres, according to <see cref="Domain"/>.</param>
    /// <param name="side">The limit at an interior join. Region endpoints use the inward limit.</param>
    public ScalarJet Evaluate(double coordinate, EvaluationSide side = EvaluationSide.Automatic) =>
        timeline.Evaluate(coordinate, side);
}
