using System;
using System.Collections.Generic;
using System.Linq;
using QuantumDynamix.FVD;
using QuantumDynamix.Geometry;
using QuantumDynamix.Math;
using QuantumDynamix.Math.Functions;
using QuantumDynamix.Math.Timelines;
using QuantumDynamix.Track;

namespace QuantumDynamix.App;

/// <summary>Samples the reconstruction directly so the editor uses the original authored heartline metres.</summary>
internal sealed class RegionTrackGeometry
{
    internal const int DisplayIntervals = 160;
    internal const string LateralChannel = "Left accel. (m/s²)";
    internal const string VerticalChannel = "Up accel. (m/s²)";
    internal const string LateralForceChannel = "Lateral force (g)";
    internal const string NormalForceChannel = "Normal force (g)";
    internal const string YawChangeChannel = "Yaw Change (°/m)";
    internal const string PitchChangeChannel = "Pitch Change (°/m)";
    internal static readonly Vector3d WorldGravity = -Vector3d.UnitZ * HeartlineForceEvaluator.StandardGravity;
    // Treat only directions indistinguishable from vertical at floating-point precision as poles.
    private static readonly double VerticalResolution = 32 * (System.Math.BitIncrement(1.0) - 1);

    // Use the existing analytic reconstruction tests' numerical settings for this viewport workflow.
    private static readonly HeartlineReconstructionSettings Settings = ReconstructionSettings(2_000_000);

    private static HeartlineReconstructionSettings ReconstructionSettings(int work) => new([1e-10])
    {
        PositionToleranceMetres = 2e-6,
        FrameAngleToleranceRadians = 2e-6,
        CurvatureTolerancePerMetre = 2e-7,
        UnbankedRollRateToleranceRadiansPerMetre = 2e-7,
        BankJoinToleranceRadians = 1e-10,
        StationMappingToleranceMetres = 1e-8,
        TimeMappingToleranceSeconds = 1e-8,
        MinimumForwardSpeedMetresPerSecond = 1e-4,
        MaximumStepMetres = 1,
        MaximumWork = work
    };

    // Generic upright preview dimensions in metres. See docs/ForceRegionAuthoring.md.
    private const double RailCenterHalfSpacing = .5;
    private const double HeartlineHeightAboveRailCenters = .75;
    private const double SpineDepthBelowRailCenters = .25;

    private readonly ReconstructedHeartline[] spans;
    private readonly double[] spanStarts;
    private readonly CurveFrame? sourceAnchor;
    private readonly CurveFrame worldAnchor;
    private readonly double length;

    internal TrackWireframe DisplayScene { get; }

    internal RegionTrackGeometry(AuthoredRegion[] authored, double speed, CurveFrame worldAnchor, double anchorStation,
        double? anchorHeading = null)
    {
        double start = authored[0].Definition.Start;
        double end = authored[^1].Definition.End;
        length = end - start;
        this.worldAnchor = worldAnchor;
        var bank = new ScalarTimeline(ProfileDomain.Distance,
            authored.SelectMany(entry => entry.Definition.Channels[TrackDocument.BankingChannel].Segments),
            continuityTolerance: Settings.BankJoinToleranceRadians);
        if (authored.All(entry => entry.Mode == RegionAuthoringMode.CoordinateAcceleration))
        {
            ReconstructedHeartline heartline = Reconstruct(authored, speed, RigidPose3d.Identity, bank, Settings);
            spans = [heartline];
            sourceAnchor = heartline.Evaluate(anchorStation - start).UnbankedFrame;
        }
        else
        {
            spans = ReconstructWorldSpans(authored, speed, worldAnchor, anchorStation, bank, anchorHeading);
        }
        spanStarts = spans.Select(span => span.AuthoredStartStationMetres).ToArray();
        DisplayScene = Sample(authored, DisplayIntervals);
    }

    private static ReconstructedHeartline[] ReconstructWorldSpans(AuthoredRegion[] authored, double speed,
        CurveFrame anchor, double anchorStation, ScalarTimeline bank, double? anchorHeading)
    {
        var spans = new List<ReconstructedHeartline>();
        int work = Settings.MaximumWork;
        int index = 0;
        if (anchorStation > authored[0].Definition.Start)
        {
            // Prepend adds one Region before the existing world pose. Integrate toward decreasing station
            // with the same forward-facing rider frame and fixed world gravity.
            spans.Add(ReconstructSpan(authored[..1], anchor, fromEnd: true, anchorHeading));
            index = 1;
        }

        CurveFrame frame = anchor;
        while (index < authored.Length)
        {
            RegionAuthoringMode mode = authored[index].Mode;
            ReconstructedHeartlineSample? incoming = null;
            bool mixed = index > 0 && authored[index - 1].Mode != mode;
            if (index > 0)
            {
                ReconstructedHeartline previous = spans[^1];
                incoming = previous.Evaluate(previous.LengthMetres, EvaluationSide.Left);
                if (mixed)
                {
                    double? heading = authored[index].Definition.Start == anchorStation ? anchorHeading : null;
                    authored[index] = BindStart(authored[index], TransverseInMode(incoming.Value, mode, heading));
                }
                else
                {
                    // Prepending creates a reverse span even when the two Regions use the same channels.
                    var (lateral, vertical) = TransverseChannels(mode);
                    double tolerance = Settings.AccelerationJoinTolerances[0] /
                        (mode == RegionAuthoringMode.Force ? HeartlineForceEvaluator.StandardGravity : 1);
                    Region preceding = authored[index - 1].Definition;
                    Region following = authored[index].Definition;
                    if (preceding.DiagnoseJoin(following, lateral, [tolerance]).MatchingOrder < 0 ||
                        preceding.DiagnoseJoin(following, vertical, [tolerance]).MatchingOrder < 0)
                    { throw new InvalidOperationException("The transverse values at the reconstruction anchor do not match."); }
                }
            }
            int end = index + 1;
            while (end < authored.Length && authored[end].Mode == mode) { end++; }
            ReconstructedHeartline span = ReconstructSpan(authored[index..end], frame, fromEnd: false,
                authored[index].Definition.Start == anchorStation ? anchorHeading : null);
            if (incoming is { CurvatureVectorPerMetre: { } before })
            {
                Vector3d after = span.Evaluate(0, EvaluationSide.Right).CurvatureVectorPerMetre!.Value;
                if ((after - before).Length > Settings.CurvatureTolerancePerMetre)
                { throw new InvalidOperationException("Curvature at the adjoining Region boundary exceeds the join tolerance."); }
            }
            spans.Add(span);
            frame = span.Evaluate(span.LengthMetres, EvaluationSide.Left).UnbankedFrame;
            index = end;
        }
        return spans.ToArray();

        ReconstructedHeartline ReconstructSpan(AuthoredRegion[] entries, CurveFrame initial, bool fromEnd, double? heading)
        {
            if (work == 0) { throw new InvalidOperationException("The reconstruction work limit was exhausted."); }
            Vector3d forward = initial.Forward;
            Vector3d left = initial.Left;
            Vector3d up = initial.Up;
            var basis = new Matrix3x3d(forward.X, left.X, up.X, forward.Y, left.Y, up.Y, forward.Z, left.Z, up.Z);
            var pose = new RigidPose3d(initial.Position, HeartlineFrameMath.FromBasis(basis));
            ReconstructedHeartline span = Reconstruct(entries, speed, pose, bank, ReconstructionSettings(work), fromEnd, heading);
            work -= span.Diagnostics.WorkUsed;
            return span;
        }
    }

    private static ReconstructedHeartline Reconstruct(AuthoredRegion[] entries, double speed, RigidPose3d pose,
        ScalarTimeline bank, HeartlineReconstructionSettings settings, bool fromEnd = false, double? heading = null)
    {
        Region[] regions = entries.Select(entry => entry.Definition).ToArray();
        double start = regions[0].Start;
        double end = regions[^1].End;
        const string longitudinalChannel = "Prescribed forward acceleration";
        var zero = ScalarTimeline.FromKeyframes(ProfileDomain.Distance, [new(start, 0), new(end, 0)]);
        var motionRegion = new Region(ProfileDomain.Distance, start, end, [KeyValuePair.Create(longitudinalChannel, zero)]);
        var motion = new LongitudinalAccelerationProfile([new(motionRegion, longitudinalChannel)], 0, [0, 0, 0, 0]);
        LongitudinalTrajectory trajectory = motion.IntegrateDistance(speed, new(1e-9, 1e-9, 32, 100_000));

        RegionAuthoringMode mode = entries[0].Mode;
        var (lateral, vertical) = TransverseChannels(mode);
        return HeartlineReconstructor.Reconstruct(new(regions, lateral, vertical, trajectory, pose, bank)
        {
            SpecificForceWorldGravity = mode == RegionAuthoringMode.Force ? WorldGravity : null,
            GeometricHeadingAtAnchorRadians = mode == RegionAuthoringMode.Geometric
                ? ResolveAnchorHeading(pose, heading, settings.FrameAngleToleranceRadians) : null,
            IntegrateFromEnd = fromEnd
        }, settings);
    }

    internal static bool SupportsSpeed(double speed) => double.IsFinite(speed) && speed >= Settings.MinimumForwardSpeedMetresPerSecond;

    internal CurveFrame UnbankedFrameAt(double station) =>
        ToWorld(Evaluate(station, EvaluationSide.Right).UnbankedFrame);

    internal static (string Lateral, string Vertical) TransverseChannels(RegionAuthoringMode mode) => mode switch
    {
        RegionAuthoringMode.Force => (LateralForceChannel, NormalForceChannel),
        RegionAuthoringMode.Geometric => (YawChangeChannel, PitchChangeChannel),
        _ => (LateralChannel, VerticalChannel)
    };

    internal static bool IsTransverseChannel(string channel) =>
        channel is LateralChannel or VerticalChannel or LateralForceChannel or NormalForceChannel;

    internal Vector3d TransverseAt(double station, EvaluationSide side, RegionAuthoringMode mode)
    {
        ReconstructedHeartlineSample sample = Evaluate(station, side);
        return TransverseInMode(sample with
        {
            BankedFrame = ToWorld(sample.BankedFrame),
            UnbankedFrame = ToWorld(sample.UnbankedFrame),
            CurvatureVectorPerMetre = sample.CurvatureVectorPerMetre is { } curvature ? ToWorldDirection(curvature) : null
        }, mode);
    }

    private static Vector3d TransverseInMode(ReconstructedHeartlineSample sample, RegionAuthoringMode mode, double? heading = null)
    {
        if (mode == RegionAuthoringMode.Geometric)
        {
            CurveFrame frame = sample.UnbankedFrame;
            Vector3d curvature = sample.CurvatureVectorPerMetre!.Value;
            double horizontal = HorizontalLength(frame.Forward);
            var (yaw, pitch, missing) = GroundRates(frame, curvature, heading);
            if (horizontal <= VerticalResolution && System.Math.Abs(missing) > Settings.CurvatureTolerancePerMetre)
            {
                throw new InvalidOperationException("The vertical mixed-mode join has sideways curvature that its carried heading cannot represent with Pitch Change and Yaw Change.");
            }
            return new(0, yaw ?? 0, pitch);
        }
        var acceleration = new Vector3d(0, sample.LateralCoordinateAccelerationMetresPerSecondSquared,
            sample.VerticalCoordinateAccelerationMetresPerSecondSquared);
        return mode == RegionAuthoringMode.Force ? (acceleration - ProjectGravity(sample.BankedFrame)) /
            HeartlineForceEvaluator.StandardGravity : acceleration;
    }

    private static double HorizontalLength(Vector3d forward) => new Vector3d(forward.X, forward.Y, 0).Length;

    private static double GroundHeading(Vector3d forward, Vector3d left) => HorizontalLength(forward) > VerticalResolution
        ? System.Math.Atan2(forward.Y, forward.X)
        : System.Math.Atan2(-left.X, left.Y);

    private static double ResolveAnchorHeading(RigidPose3d pose, double? carriedHeading, double tolerance)
    {
        Vector3d forward = pose.TransformDirection(Vector3d.UnitX);
        if (carriedHeading is not double heading)
        { return GroundHeading(forward, pose.TransformDirection(Vector3d.UnitY)); }
        var (sine, cosine) = System.Math.SinCos(heading);
        if (System.Math.Abs(forward.Y * cosine - forward.X * sine) > tolerance)
        { throw new InvalidOperationException("The carried heading does not match the geometric reconstruction anchor's tangent."); }
        return heading;
    }

    private static (double? Yaw, double Pitch, double SidewaysCurvature) GroundRates(CurveFrame frame, Vector3d curvature,
        double? carriedHeading = null)
    {
        double horizontal = HorizontalLength(frame.Forward);
        var (sine, cosine) = System.Math.SinCos(carriedHeading ?? GroundHeading(frame.Forward, frame.Left));
        var headingLeft = new Vector3d(-sine, cosine, 0);
        Vector3d pitchAxis = Vector3d.Cross(frame.Forward, headingLeft);
        double pitch = Vector3d.Dot(curvature, pitchAxis) / HeartlineGeometricHeading.RadiansPerDegree;
        double sideways = Vector3d.Dot(curvature, headingLeft);
        double? yaw = horizontal > VerticalResolution
            ? sideways / (frame.Forward.X * cosine + frame.Forward.Y * sine) / HeartlineGeometricHeading.RadiansPerDegree : null;
        return (yaw, pitch, sideways);
    }

    internal static Vector3d ProjectGravity(CurveFrame frame) => new(0,
        Vector3d.Dot(WorldGravity, frame.Left), Vector3d.Dot(WorldGravity, frame.Up));

    private static AuthoredRegion BindStart(AuthoredRegion entry, Vector3d value)
    {
        var (lateral, vertical) = TransverseChannels(entry.Mode);
        if (entry.Keyframes[lateral][0].Value == value.Y && entry.Keyframes[vertical][0].Value == value.Z)
        { return entry; }
        var keys = new Dictionary<string, TimelineKeyframe[]>(entry.Keyframes);
        var channels = new Dictionary<string, ScalarTimeline>(entry.Definition.Channels);
        Bind(lateral, value.Y);
        Bind(vertical, value.Z);
        Region definition = entry.Definition;
        return entry with { Definition = new(definition.Domain, definition.Start, definition.End, channels), Keyframes = keys };

        void Bind(string channel, double start)
        {
            TimelineKeyframe[] original = keys[channel];
            if (original[0].Value == start) { return; }
            TransitionShape[] shapes = entry.Shapes(channel);
            // Avoid perturbing explicitly equal Constant endpoints during a numerically equivalent conversion.
            double magnitude = System.Math.Max(1, System.Math.Max(System.Math.Abs(start), System.Math.Abs(original[0].Value)));
            if (shapes[0].Kind is null && System.Math.Abs(original[0].Value - start) <= VerticalResolution * magnitude) { return; }
            TimelineKeyframe[] updated = [original[0] with { Value = start }, .. original[1..]];
            string? error = TimelineEditorSession.BuildTimeline(entry.Definition.Domain, updated,
                out ScalarTimeline? timeline, shapes);
            if (error is not null) { throw new InvalidOperationException($"Linked start of {channel} in '{entry.Name}'. {error}"); }
            keys[channel] = updated;
            channels[channel] = timeline!;
        }
    }

    internal Vector3d CalculateSpecificForceG(double station, EvaluationSide side)
    {
        ReconstructedHeartlineSample sample = Evaluate(station, side);
        if (!sample.CurvatureVectorPerMetre.HasValue)
        { throw new InvalidOperationException("Select an inward side at this station to calculate specific force."); }
        double speed = sample.Motion.HeartlineVelocityMetresPerSecond;
        Vector3d acceleration = ToWorldDirection(sample.CurvatureVectorPerMetre.Value) * speed * speed;
        return HeartlineForceEvaluator.ProjectSpecificForce(ToWorld(sample.BankedFrame), acceleration, WorldGravity) /
            HeartlineForceEvaluator.StandardGravity;
    }

    internal (double Pitch, double? Heading, double? PitchRate, double? YawRate, Vector3d Curvature) CalculateGeometry(
        double station, EvaluationSide side)
    {
        ReconstructedHeartlineSample sample = Evaluate(station, side);
        CurveFrame frame = ToWorld(sample.UnbankedFrame);
        Vector3d curvature = ToWorldDirection(sample.CurvatureVectorPerMetre!.Value);
        double horizontal = HorizontalLength(frame.Forward);
        double degrees = 1 / HeartlineGeometricHeading.RadiansPerDegree;
        double pitch = System.Math.Atan2(frame.Forward.Z, horizontal) * degrees;
        if (horizontal <= VerticalResolution) { return (pitch, null, null, null, curvature); }
        var (yaw, pitchRate, _) = GroundRates(frame, curvature);
        return (pitch, GroundHeading(frame.Forward, frame.Left) * degrees, pitchRate, yaw, curvature);
    }

    internal double? GeometricHeadingAt(double station) => SpanAt(station, EvaluationSide.Right).GeometricHeadingAt(station);

    private ReconstructedHeartlineSample Evaluate(double station, EvaluationSide side)
    {
        ReconstructedHeartline span = SpanAt(station, side);
        return span.Evaluate(station - span.AuthoredStartStationMetres, side);
    }

    private ReconstructedHeartline SpanAt(double station, EvaluationSide side)
    {
        int index = Array.BinarySearch(spanStarts, station);
        if (index < 0) { index = ~index - 1; }
        else if (index > 0 && side == EvaluationSide.Left) { index--; }
        return spans[System.Math.Max(0, index)];
    }

    private CurveFrame ToWorld(CurveFrame frame)
    {
        if (sourceAnchor is not { } source) { return frame; }
        Vector3d displacement = ToWorldDirection(frame.Position - source.Position);
        Vector3d position = worldAnchor.Position + displacement;
        if ((position - worldAnchor.Position - displacement).Length > Settings.PositionToleranceMetres)
        {
            throw new InvalidOperationException("The world position is too large to represent the reconstructed track at this precision.");
        }
        return new(position, ToWorldDirection(frame.Forward), ToWorldDirection(frame.Left), ToWorldDirection(frame.Up));
    }

    private Vector3d ToWorldDirection(Vector3d direction) => sourceAnchor is { } source
        ? worldAnchor.Transform(new(Vector3d.Dot(source.Forward, direction),
            Vector3d.Dot(source.Left, direction), Vector3d.Dot(source.Up, direction)))
        : direction;

    internal TrackWireframe Sample(IReadOnlyList<AuthoredRegion> regions, int stationIntervals)
    {
        ArgumentOutOfRangeException.ThrowIfNegativeOrZero(stationIntervals);
        var paths = new List<WireframePath>();
        var boundaries = new List<WireframeBoundary>();
        foreach (AuthoredRegion entry in regions)
        {
            Region region = entry.Definition;
            var centre = new List<WireframeVertex>();
            var spine = new List<WireframeVertex>();
            var left = new List<WireframeVertex>();
            var right = new List<WireframeVertex>();
            var (lateral, vertical) = TransverseChannels(entry.Mode);
            string[] channels = [TrackDocument.BankingChannel, lateral, vertical];
            double[] stations = channels.SelectMany(channel => region.Channels[channel].Breakpoints)
                .Distinct().Order().Prepend(region.Start).Append(region.End).ToArray();
            AddVertex(region.Start);
            for (int span = 1; span < stations.Length; span++)
            {
                double start = stations[span - 1];
                double end = stations[span];
                int intervals = (int)System.Math.Ceiling((end - start) / length * stationIntervals);
                for (int i = 1; i < intervals; i++)
                {
                    double station = start + (end - start) * ((double)i / intervals);
                    if (station > centre[^1].HeartlineStation && station < end)
                    {
                        AddVertex(station);
                    }
                }
                AddVertex(end);
            }
            paths.Add(new(WireframePathKind.Heartline, "Heartline", centre.AsReadOnly(), entry.Id));
            paths.Add(new(WireframePathKind.Spine, "Spine", spine.AsReadOnly(), entry.Id));
            paths.Add(new(WireframePathKind.Rail, "left", left.AsReadOnly(), entry.Id));
            paths.Add(new(WireframePathKind.Rail, "right", right.AsReadOnly(), entry.Id));

            void AddVertex(double station)
            {
                EvaluationSide side = station == region.End ? EvaluationSide.Left : EvaluationSide.Right;
                CurveFrame frame = ToWorld(Evaluate(station, side).BankedFrame);
                Vector3d railCenter = frame.Position - frame.Up * HeartlineHeightAboveRailCenters;
                Vector3d spinePosition = railCenter - frame.Up * SpineDepthBelowRailCenters;
                Vector3d railOffset = frame.Left * RailCenterHalfSpacing;
                centre.Add(new(frame.Position, station));
                spine.Add(new(spinePosition, station));
                left.Add(new(railCenter + railOffset, station));
                right.Add(new(railCenter - railOffset, station));
                if (station == region.Start || station == region.End)
                {
                    boundaries.Add(new(entry.Id, entry.Name, station == region.Start, station, frame.Position, frame));
                }
            }
        }
        return new(paths.AsReadOnly(), [], boundaries.AsReadOnly(), []);
    }
}
