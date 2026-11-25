using System;
using System.Collections.Generic;
using System.Linq;
using QuantumDynamix.Geometry;
using QuantumDynamix.Math.Timelines;
using QuantumDynamix.Track;

namespace QuantumDynamix.App;

/// <summary>Explicit association of a distance channel in radians with an authored track.</summary>
/// <remarks>The constructor callback supplies the same geometry and numerical settings for each bank profile.</remarks>
internal sealed record TrackBankingBinding(Guid RegionId, string ChannelId, Func<ScalarTimeline, HeartlineTrack> WithBanking);

internal sealed class TrackWireframeSource
{
    private readonly ArcLengthMap heartline;
    private readonly HeartlineTrack? spine;
    private readonly RailGeometry? rails;
    private readonly TrackBankingBinding? banking;
    private readonly WireframeSupport[] supports;

    internal TrackWireframeSource(ArcLengthMap heartline)
    {
        this.heartline = heartline;
        supports = [];
    }

    internal TrackWireframeSource(HeartlineTrack spine, RailGeometry? rails = null, TrackBankingBinding? banking = null,
        IEnumerable<WireframeSupport>? supports = null)
    {
        if (rails is not null && !ReferenceEquals(rails.Track, spine))
        {
            throw new ArgumentException("Rail geometry must reference the supplied track.", nameof(rails));
        }

        heartline = spine.HeartlineStations;
        this.spine = spine;
        this.rails = rails;
        this.banking = banking;
        this.supports = supports?.ToArray() ?? [];
    }

    internal TrackWireframe Sample(IReadOnlyList<AuthoredRegion> regions, int stationIntervals)
    {
        if (banking is null)
        {
            return TrackWireframeSampler.Sample(heartline, spine, rails, regions, supports, stationIntervals);
        }

        AuthoredRegion? region = regions.FirstOrDefault(entry => entry.Id == banking.RegionId);
        if (region is null || !region.Definition.Channels.TryGetValue(banking.ChannelId, out ScalarTimeline? bank))
        {
            return HeartlineOnly($"Offset geometry requires the bound '{banking.ChannelId}' banking profile.");
        }
        if (bank.Domain != ProfileDomain.Distance || bank.Minimum != 0 || bank.Maximum != heartline.Length)
        {
            return HeartlineOnly($"The bound banking profile must span 0 to {heartline.Length:G} heartline metres.");
        }

        HeartlineTrack edited;
        RailGeometry? editedRails;
        try
        {
            edited = ReferenceEquals(bank, spine!.Banking) ? spine : banking.WithBanking(bank);
            if (!ReferenceEquals(edited.Heartline, heartline.Curve) || !ReferenceEquals(edited.Offsets, spine!.Offsets)
                || !ReferenceEquals(edited.Banking, bank) || edited.HeartlineStations.Length != heartline.Length)
            {
                return HeartlineOnly("The banking binding must keep the authored heartline and offsets and apply the selected timeline.");
            }
            editedRails = rails is null || ReferenceEquals(edited, spine)
                ? rails : new RailGeometry(edited, rails.Arrangement, rails.Settings);
        }
        catch (Exception error) when (error is ArgumentException or InvalidOperationException)
        {
            return HeartlineOnly($"Offset geometry could not be evaluated with the bound banking profile. {error.Message}");
        }

        return TrackWireframeSampler.Sample(heartline, edited, editedRails, regions, supports, stationIntervals);

        TrackWireframe HeartlineOnly(string message) =>
            TrackWireframeSampler.Sample(heartline, null, null, regions, [], stationIntervals, message);
    }
}
