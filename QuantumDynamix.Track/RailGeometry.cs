using System;
using System.Collections.Generic;
using System.Linq;
using QuantumDynamix.Math;
using QuantumDynamix.Math.Functions;
using QuantumDynamix.Math.Numerics;
using QuantumDynamix.Math.Timelines;

namespace QuantumDynamix.Track;

/// <summary>Stationary physical rail geometry. Queries do not assemble, constrain, or advance a vehicle.</summary>
public sealed partial class RailGeometry
{
    public HeartlineTrack Track { get; }
    public RailArrangement Arrangement { get; }
    public RailValidationSettings Settings { get; }
    public IReadOnlyList<double> Breakpoints { get; }
    public double HeartlineLength => Track.HeartlineStations.Length;

    public RailGeometry(HeartlineTrack track, RailArrangement arrangement, RailValidationSettings settings)
    {
        ArgumentNullException.ThrowIfNull(track);
        ArgumentNullException.ThrowIfNull(arrangement);
        ArgumentNullException.ThrowIfNull(settings);
        Track = track;
        Arrangement = arrangement;
        Settings = settings;
        IScalarFunction[] functions = new[] { track.Banking }.Concat(TrackOffsetGeometry.Functions(track.Offsets))
            .Concat(arrangement.Rails.SelectMany(r => TrackOffsetGeometry.Functions(r.Offsets).Append(r.SectionRoll)))
            .Concat(arrangement.Spacings.Select(m => m.Expected)).ToArray();
        foreach (IScalarFunction function in functions)
        {
            if (!double.IsFinite(function.Minimum) || !double.IsFinite(function.Maximum)
                || function.Minimum > 0 || function.Maximum < HeartlineLength
                || function is ScalarTimeline { Domain: ProfileDomain.Time })
            { throw new ArgumentException("All rail, bank, spine, and measurement profiles must cover the heartline distance domain."); }
            if (function.Breakpoints.Any(s => !double.IsFinite(s) || s <= function.Minimum || s >= function.Maximum))
            { throw new ArgumentException("Profile breakpoints must be finite and strictly inside their function domain."); }
        }
        double[] points = functions.SelectMany(f => f.Breakpoints)
            .Concat(track.Heartline.Definition.Breakpoints.Select(track.HeartlineStations.StationAt))
            .Concat(track.ResolveFeatures().Stations.Values).Where(s => s > 0 && s < HeartlineLength).Distinct().Order().ToArray();
        Breakpoints = Array.AsReadOnly(points);
    }

    public RailSample Evaluate(string railId, double heartlineStation, EvaluationSide side = EvaluationSide.Automatic)
    {
        RailDefinition rail = Arrangement.GetRail(railId);
        return new(railId, heartlineStation, TrackOffsetGeometry.Evaluate(Track, rail.Offsets, heartlineStation, side),
            rail.SectionRoll.Evaluate(heartlineStation, side), Settings);
    }

    /// <summary>First surface partials, suitable for later contact geometry. No surface Hessian or contact law is supplied.</summary>
    public RailSurfaceSample EvaluateSurface(string railId, double heartlineStation, int boundaryIndex, double profileParameter,
        EvaluationSide stationSide = EvaluationSide.Automatic, EvaluationSide profileSide = EvaluationSide.Automatic)
    {
        RailDefinition rail = Arrangement.GetRail(railId);
        RailSample sample = Evaluate(railId, heartlineStation, stationSide);
        return Surface(rail, sample, boundaryIndex, profileParameter, profileSide);
    }

    private static RailSurfaceSample Surface(RailDefinition rail, RailSample sample, int boundaryIndex,
        double profileParameter, EvaluationSide profileSide)
    {
        var profile = rail.CrossSection.Boundaries[boundaryIndex].Evaluate(profileParameter, profileSide);
        Vector3d displacement = sample.SectionFrame.Transform(profile.Position);
        return new(sample.Position + displacement, sample.Centerline.FirstDerivative + Vector3d.Cross(sample.SectionAngularRate, displacement),
            sample.SectionFrame.Transform(profile.FirstDerivative));
    }

    public double MeasureSpacing(string spacingId, double heartlineStation, EvaluationSide side = EvaluationSide.Automatic)
    {
        RailSpacingDefinition measure = Arrangement.Spacings.FirstOrDefault(m => m.Id == spacingId)
            ?? throw new KeyNotFoundException($"Unknown spacing '{spacingId}'.");
        RailSample a = Evaluate(measure.RailA, heartlineStation, side);
        RailSample b = Evaluate(measure.RailB, heartlineStation, side);
        return Measure(measure, a, b);
    }

    private double Measure(RailSpacingDefinition measure, RailSample a, RailSample b)
    {
        Vector3d delta = Difference(a, b);
        if (measure.UsesSectionDatums)
        {
            delta += b.SectionFrame.Transform(Arrangement.GetRail(measure.RailB).CrossSection.Datums[measure.DatumB!])
                - a.SectionFrame.Transform(Arrangement.GetRail(measure.RailA).CrossSection.Datums[measure.DatumA!]);
        }
        return measure.Measurement == RailSpacingMeasurement.Euclidean ? delta.Length : Vector3d.Dot(delta, a.BankedReferenceFrame.Left);
    }

    internal static Vector3d Difference(RailSample a, RailSample b) =>
        (b.BankedReferenceFrame.Position - a.BankedReferenceFrame.Position) + (b.ReferenceDisplacement - a.ReferenceDisplacement);

    public TrackDistanceMap CreateRailDistanceMap(string railId, QuadratureSettings? settings = null)
    {
        Arrangement.GetRail(railId);
        return new(this, railId, settings);
    }

    public TrackDistanceMap CreateSpineDistanceMap(QuadratureSettings? settings = null) => new(this, null, settings);

    internal TrackOffsetSample EvaluateCenterline(string? railId, double station, EvaluationSide side) =>
        TrackOffsetGeometry.Evaluate(Track, railId is null ? null : Arrangement.GetRail(railId).Offsets, station, side);
}
