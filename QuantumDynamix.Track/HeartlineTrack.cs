using System;
using System.Collections.Generic;
using System.Linq;
using QuantumDynamix.Geometry;
using QuantumDynamix.Math;
using QuantumDynamix.Math.Functions;
using QuantumDynamix.Math.Numerics;
using QuantumDynamix.Math.Timelines;
using QuantumDynamix.Splines.Curves;

namespace QuantumDynamix.Track;

/// <summary>A stable authored location on the reference curve; callers preserve/update its ID during edits.</summary>
public sealed record TrackFeature(string Id, double Parameter);

/// <summary>Externally supplied offsets in the banked forward/left/up frame, as functions of heartline metres.</summary>
public sealed class TrackOffsetProfile
{
    public IScalarFunction Forward { get; }
    public IScalarFunction Left { get; }
    public IScalarFunction Up { get; }
    public TrackOffsetProfile(IScalarFunction forward, IScalarFunction left, IScalarFunction up)
    {
        ArgumentNullException.ThrowIfNull(forward);
        ArgumentNullException.ThrowIfNull(left);
        ArgumentNullException.ThrowIfNull(up);
        Forward = forward;
        Left = left;
        Up = up;
    }
    public static TrackOffsetProfile Constant(Vector3d offset, double maximumStation)
    {
        return new(new CustomFunction(0, maximumStation, (_, _) => new(offset.X)),
            new CustomFunction(0, maximumStation, (_, _) => new(offset.Y)),
            new CustomFunction(0, maximumStation, (_, _) => new(offset.Z)));
    }
    internal Vector3d Derivative(double station, int order, EvaluationSide side) => new(
        Forward.Evaluate(station, side).Derivative(order), Left.Evaluate(station, side).Derivative(order), Up.Evaluate(station, side).Derivative(order));
}

/// <summary>Spine derivatives are with respect to heartline station, not spine arc length.</summary>
public readonly record struct HeartlineTrackSample(double HeartlineStation, double Parameter, Vector3d HeartlinePosition,
    CurveFrame BankedFrame, Vector3d SpinePosition, Vector3d SpineStationDerivative, Vector3d SpineStationSecondDerivative)
{
    public double SpineSpeedPerHeartlineMetre => SpineStationDerivative.Length;
    public double SpineCurvature
    {
        get
        {
            double speed = SpineStationDerivative.Length;
            if (!(speed > 0)) { throw new InvalidOperationException("The derived spine is singular."); }
            Vector3d tangent = SpineStationDerivative / speed;
            Vector3d scaled = SpineStationSecondDerivative / speed;
            return ((scaled - tangent * Vector3d.Dot(tangent, scaled)) / speed).Length;
        }
    }
}

/// <summary>Authored heartline and mathematical offsets. No gauge, rails, vehicle guidance, or physical motion is inferred.</summary>
public sealed class HeartlineTrack
{
    public SplineCurve3d Heartline { get; }
    public ArcLengthMap HeartlineStations { get; }
    public RotationMinimizingFrames Frames { get; }
    public IScalarFunction Banking { get; }
    public TrackOffsetProfile Offsets { get; }
    public IReadOnlyList<TrackFeature> Features { get; }

    public HeartlineTrack(SplineCurve3d heartline, Vector3d initialUp, TrackOffsetProfile offsets,
        IScalarFunction? banking = null, IEnumerable<TrackFeature>? features = null,
        QuadratureSettings? lengthSettings = null, double frameAngularTolerance = 1e-8)
    {
        ArgumentNullException.ThrowIfNull(heartline);
        ArgumentNullException.ThrowIfNull(offsets);
        Heartline = heartline;
        HeartlineStations = new(heartline, lengthSettings);
        Frames = new(heartline, initialUp, frameAngularTolerance);
        Banking = banking ?? new CustomFunction(0, HeartlineStations.Length, (_, _) => new(0));
        Offsets = offsets;
        foreach (IScalarFunction function in new[] { Banking, offsets.Forward, offsets.Left, offsets.Up })
        {
            if (function.Minimum > 0 || function.Maximum < HeartlineStations.Length)
            { throw new ArgumentException("Bank and offset domains must cover the complete heartline station range."); }
        }
        TrackFeature[] items = features?.ToArray() ?? Array.Empty<TrackFeature>();
        if (items.Any(f => f is null || string.IsNullOrWhiteSpace(f.Id) || !double.IsFinite(f.Parameter)
            || f.Parameter < heartline.Minimum || f.Parameter > heartline.Maximum)
            || items.Select(f => f.Id).Distinct(StringComparer.Ordinal).Count() != items.Length)
        { throw new ArgumentException("Features require unique nonblank IDs and valid reference parameters.", nameof(features)); }
        Features = Array.AsReadOnly(items);
    }

    public FeatureStations ResolveFeatures() => new(Features.Select(f =>
        new KeyValuePair<string, double>(f.Id, HeartlineStations.StationAt(f.Parameter))));

    public HeartlineTrackSample Evaluate(double heartlineStation, EvaluationSide side = EvaluationSide.Automatic)
    {
        TrackOffsetSample sample = TrackOffsetGeometry.Evaluate(this, null, heartlineStation, side);
        if (sample.Curve.DerivativeOrder < 2)
        { throw new InvalidOperationException("Spine second derivatives require a third heartline derivative or an explicit sided limit."); }
        return new(heartlineStation, sample.Parameter, sample.Frame.Position, sample.Frame,
            sample.Curve.Position, sample.Curve.FirstDerivative, sample.Curve.SecondDerivative);
    }

    /// <summary>Length of the derived spine, not heartline length or rail length. The quadrature error estimate
    /// excludes frame-transport and heartline-inversion approximation errors.</summary>
    public QuadratureResult IntegrateSpineLength(QuadratureSettings? settings = null)
    {
        settings ??= new();
        double length = HeartlineStations.Length;
        double[] breaks = Heartline.Definition.Breakpoints.Select(HeartlineStations.StationAt)
            .Concat(Banking.Breakpoints).Concat(Offsets.Forward.Breakpoints).Concat(Offsets.Left.Breakpoints).Concat(Offsets.Up.Breakpoints)
            .Where(s => s > 0 && s < length).Prepend(0).Append(length).Distinct().Order().ToArray();
        double total = 0;
        double error = 0;
        int evaluations = 0;
        int intervals = 0;
        for (int i = 1; i < breaks.Length; i++)
        {
            var local = new QuadratureSettings(System.Math.Max(double.Epsilon, settings.AbsoluteTolerance * ((breaks[i] - breaks[i - 1]) / length)),
                settings.RelativeTolerance, settings.MaximumDepth, settings.MaximumEvaluations);
            QuadratureResult result = AdaptiveGaussKronrod.Integrate(s => Evaluate(s, EvaluationSide.Right).SpineSpeedPerHeartlineMetre,
                breaks[i - 1], breaks[i], local);
            total += result.Value;
            error += result.EstimatedAbsoluteError;
            evaluations = checked(evaluations + result.Evaluations);
            intervals = checked(intervals + result.Subintervals);
        }
        return new(total, error, evaluations, intervals);
    }
}
