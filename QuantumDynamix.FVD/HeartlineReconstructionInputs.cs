using System;
using System.Collections.Generic;
using System.Linq;
using QuantumDynamix.Math;
using QuantumDynamix.Math.Functions;
using QuantumDynamix.Math.Timelines;
using QuantumDynamix.Track;

namespace QuantumDynamix.FVD;

internal sealed class HeartlineReconstructionInputs
{
    private readonly HeartlineReconstructionRequest request;
    private readonly ResolvedAxis transverse;
    private readonly ResolvedAxis? banking;
    private readonly HeartlineGeometricHeading? heading;
    private readonly double[] regionEnds;
    private readonly Dictionary<double, (double Native, double? AuthoredTime)> motionPins = new();

    internal HeartlineMotionMap Motion { get; }
    internal bool UsesSpecificForce => request.SpecificForceWorldGravity.HasValue;
    internal bool RequiresOrientation => UsesSpecificForce || heading is not null;
    internal double? GeometricHeadingAt(double authoredStation, ReconstructionWork work) => heading?.At(authoredStation, work);
    internal ProfileDomain TransverseDomain { get; }
    internal double[] Boundaries { get; }
    internal double[] CurvatureSteps { get; }
    internal IReadOnlyList<(string, double, RegionJoinDiagnostic)> AccelerationJoins { get; }
    internal IReadOnlyList<(double, RegionJoinDiagnostic)> BankJoins { get; }

    internal HeartlineReconstructionInputs(HeartlineReconstructionRequest request,
        HeartlineReconstructionSettings settings, ReconstructionWork work)
    {
        this.request = request;
        TransverseDomain = request.Regions[0].Domain;
        if (request.InitialUnbankedPose.Orientation == default)
        { throw work.Fail(HeartlineFailureCode.InvalidInput, "Supply a valid initial unbanked pose."); }
        if (request.LateralChannelId == request.VerticalChannelId)
        { throw work.Fail(HeartlineFailureCode.InvalidInput, "The two transverse channel identifiers must be distinct."); }

        string[] channels = [request.LateralChannelId, request.VerticalChannelId];
        regionEnds = new double[request.Regions.Count];
        var breakpoints = new SortedSet<double>();
        for (int i = 0; i < request.Regions.Count; i++)
        {
            work.Charge();
            Region region = request.Regions[i];
            if (region.Domain != TransverseDomain || (i > 0 && regionEnds[i - 1] != region.Start))
            { throw work.Fail(HeartlineFailureCode.InvalidInput, "Transverse Regions must be exactly adjacent on one authored axis.",
                authored: (region.Domain, region.Start, region.End), region: i); }
            foreach (string channel in channels)
            {
                if (!region.Channels.TryGetValue(channel, out ScalarTimeline? timeline))
                { throw work.Fail(HeartlineFailureCode.InvalidInput, "The required transverse channel is missing.",
                    authored: (region.Domain, region.Start, region.End), region: i, channel: channel); }
                foreach (double coordinate in timeline.Breakpoints) { work.Charge(); breakpoints.Add(coordinate); }
            }
            regionEnds[i] = region.End;
            if (i + 1 < regionEnds.Length) { breakpoints.Add(region.End); }
        }

        Motion = new(request, settings, TransverseDomain, work);
        foreach (HeartlineMotionMap.MotionNode node in Motion.Nodes) { AddMotionPin(node.Station, node.Native, work); }
        transverse = ResolveAxis(TransverseDomain, request.Regions[0].Start, regionEnds[^1], breakpoints, work);
        if (request.Banking is ScalarTimeline bank)
        {
            if (bank.Domain != ProfileDomain.Distance)
            { throw work.Fail(HeartlineFailureCode.InvalidInput, "Banking requires a timeline on the authored station axis."); }
            banking = ResolveAxis(ProfileDomain.Distance, bank.Minimum, bank.Maximum, bank.Breakpoints, work);
        }

        var permitted = new HashSet<double>(settings.PermittedCurvatureStepCoordinates);
        if (permitted.Any(coordinate => !transverse.InteriorCoordinates.Contains(coordinate)))
        { throw work.Fail(HeartlineFailureCode.InvalidInput, "Step permissions must name interior transverse breakpoints on the traversal."); }

        var steps = new SortedSet<double>();
        var accelerationJoins = new List<(string, double, RegionJoinDiagnostic)>();
        double[] tolerances = UsesSpecificForce
            ? settings.AccelerationJoinTolerances.Select(tolerance => tolerance / HeartlineForceEvaluator.StandardGravity).ToArray()
            : settings.AccelerationJoinTolerances.ToArray();
        foreach ((double station, double coordinate) in transverse.Interior)
        {
            int left = RegionIndex(coordinate, EvaluationSide.Left);
            int right = RegionIndex(coordinate, EvaluationSide.Right);
            foreach (string channel in channels)
            {
                ScalarTimeline incoming = request.Regions[left].Channels[channel];
                if (left == right && !incoming.Breakpoints.Contains(coordinate)) { continue; }
                ScalarJet a = Evaluate(incoming, coordinate, EvaluationSide.Left, station, TransverseDomain, work, left, channel);
                ScalarJet b = Evaluate(request.Regions[right].Channels[channel], coordinate, EvaluationSide.Right, station,
                    TransverseDomain, work, right, channel);
                RegionJoinDiagnostic comparison = Region.CompareLimits(coordinate, a, b, tolerances);
                accelerationJoins.Add((channel, station, comparison));
                if (permitted.Contains(coordinate))
                {
                    if (a.Value != b.Value) { steps.Add(station); }
                }
                else if (comparison.MatchingOrder < comparison.RequiredOrder)
                {
                    throw work.Fail(HeartlineFailureCode.ContinuityRequirementFailed,
                        "A transverse channel does not meet the required join continuity.", (station, station),
                        (TransverseDomain, coordinate, coordinate), right, channel);
                }
            }
        }

        var bankJoins = new List<(double, RegionJoinDiagnostic)>();
        if (banking is not null)
        {
            foreach ((double station, double coordinate) in banking.Interior)
            {
                ScalarJet a = Evaluate(request.Banking!, coordinate, EvaluationSide.Left, station, ProfileDomain.Distance, work);
                ScalarJet b = Evaluate(request.Banking!, coordinate, EvaluationSide.Right, station, ProfileDomain.Distance, work);
                RegionJoinDiagnostic comparison = Region.CompareLimits(coordinate, a, b, [settings.BankJoinToleranceRadians]);
                bankJoins.Add((station, comparison));
                if (comparison.MatchingOrder < 0)
                { throw work.Fail(HeartlineFailureCode.ContinuityRequirementFailed, "Bank values must meet the join tolerance.",
                    (station, station), (ProfileDomain.Distance, coordinate, coordinate), channel: "Banking"); }
            }
        }

        Boundaries = motionPins.Keys.Concat(transverse.Stations).Concat(banking?.Stations ?? [])
            .Distinct().Order().ToArray();
        if (request.GeometricHeadingAtAnchorRadians is double initialHeading)
        {
            if (TransverseDomain != ProfileDomain.Distance || UsesSpecificForce || !double.IsFinite(initialHeading))
            { throw work.Fail(HeartlineFailureCode.InvalidInput, "Geometric pitch and yaw require distance profiles and a finite heading."); }
            heading = new(request.Regions.Select(region => region.Channels[request.LateralChannelId]),
                initialHeading, request.IntegrateFromEnd, work);
        }
        CurvatureSteps = steps.ToArray();
        AccelerationJoins = accelerationJoins.AsReadOnly();
        BankJoins = bankJoins.AsReadOnly();
    }

    internal ReconstructionTarget Target(double station, EvaluationSide side, ReconstructionWork work)
    {
        HeartlineMotionMap.MotionQuery? retryStartMotion = null;
        return Target(station, side, work, ref retryStartMotion);
    }

    internal ReconstructionTarget Target(double station, EvaluationSide side, ReconstructionWork work,
        ref HeartlineMotionMap.MotionQuery? retryStartMotion)
    {
        bool pinned = motionPins.TryGetValue(station, out var pin);
        HeartlineMotionMap.MappedMotion mapped = Motion.AtStation(station, pinned ? pin.Native : null,
            pin.AuthoredTime, side, work, in retryStartMotion, out HeartlineMotionMap.MotionQuery sampled);
        double space = Motion.OriginStation + station;
        var (coordinate, transverseSide) = transverse.Resolve(station,
            TransverseDomain == ProfileDomain.Time ? mapped.Time : space, side);
        int regionIndex = RegionIndex(coordinate, transverseSide);
        Region region = request.Regions[regionIndex];
        double left = Evaluate(region.Channels[request.LateralChannelId], coordinate, transverseSide,
            station, TransverseDomain, work, regionIndex, request.LateralChannelId).Value;
        double up = Evaluate(region.Channels[request.VerticalChannelId], coordinate, transverseSide,
            station, TransverseDomain, work, regionIndex, request.VerticalChannelId).Value;
        var (bankJet, bankStationError) = EvaluateBank(station, side, work);
        double stationError = System.Math.Max(mapped.StationError, bankStationError);
        double timeError = mapped.TimeError;
        if (TransverseDomain == ProfileDomain.Time) { timeError += System.Math.Abs(coordinate - mapped.Time); }
        else { stationError = System.Math.Max(stationError, StationResidual(coordinate, station)); }
        Motion.RecordErrors(stationError, timeError, station, work);
        retryStartMotion = sampled;
        var target = new ReconstructionTarget(coordinate, mapped.Motion, stationError, timeError, mapped.VelocityError,
            bankJet.Value, left, up, Vector3d.Zero);
        if (heading is { } geometric)
        {
            var (sine, cosine) = System.Math.SinCos(geometric.At(coordinate, work));
            double pitch = up * HeartlineGeometricHeading.RadiansPerDegree;
            double yaw = left * HeartlineGeometricHeading.RadiansPerDegree;
            // World yaw is about +Z, and positive pitch rotates about the negative heading-left axis.
            return target with { AngularVelocity = new(sine * pitch, -cosine * pitch, yaw) };
        }
        return UsesSpecificForce ? target : ResolveCurvature(target, null, station, side, work);
    }

    internal ReconstructionTarget ResolveOrientedTarget(ReconstructionTarget target, Quaterniond orientation,
        double station, EvaluationSide side, ReconstructionWork work)
    {
        if (heading is null) { return ResolveCurvature(target, orientation, station, side, work); }

        Vector3d local = orientation.Conjugated().Rotate(target.AngularVelocity);
        // The forward component does not turn the tangent. Dropping it transports the unbanked frame
        // without roll, including at the vertical where a ground yaw changes only the carried heading.
        var (sine, cosine) = System.Math.SinCos(target.Bank);
        double speed = target.Motion.HeartlineVelocityMetresPerSecond;
        double left = (cosine * local.Z - sine * local.Y) * speed * speed;
        double up = (-sine * local.Z - cosine * local.Y) * speed * speed;
        if (!double.IsFinite(left) || !double.IsFinite(up))
        { throw work.Fail(HeartlineFailureCode.NumericalRangeExceeded, "Geometric coordinate acceleration exceeds finite range.", (station, station)); }
        return target with { Left = left, Up = up, AngularVelocity = new(0, local.Y, local.Z) };
    }

    private ReconstructionTarget ResolveCurvature(ReconstructionTarget target, Quaterniond? orientation,
        double station, EvaluationSide side, ReconstructionWork work)
    {
        try
        {
            var (sine, cosine) = System.Math.SinCos(target.Bank);
            double left = target.Left;
            double up = target.Up;
            if (orientation is Quaterniond rotation && request.SpecificForceWorldGravity is Vector3d gravity)
            {
                Vector3d localGravity = rotation.Conjugated().Rotate(gravity);
                left = left * HeartlineForceEvaluator.StandardGravity + cosine * localGravity.Y + sine * localGravity.Z;
                up = up * HeartlineForceEvaluator.StandardGravity - sine * localGravity.Y + cosine * localGravity.Z;
            }

            double speed = target.Motion.HeartlineVelocityMetresPerSecond;
            double kl = left / speed / speed;
            double ku = up / speed / speed;
            if ((left != 0 && kl == 0) || (up != 0 && ku == 0))
            {
                throw work.Fail(HeartlineFailureCode.NumericalRangeExceeded, "A nonzero curvature component underflows double precision.",
                    (station, station), (TransverseDomain, target.Coordinate, target.Coordinate),
                    RegionIndex(target.Coordinate, side), side: side);
            }
            var omega = new Vector3d(0, -sine * kl - cosine * ku, cosine * kl - sine * ku);
            return target with { Left = left, Up = up, AngularVelocity = omega };
        }
        catch (ArgumentOutOfRangeException ex)
        { throw work.Fail(HeartlineFailureCode.NumericalRangeExceeded, "The transverse acceleration or curvature exceeds finite range.",
            (station, station), (TransverseDomain, target.Coordinate, target.Coordinate),
            RegionIndex(target.Coordinate, side), side: side, inner: ex); }
    }

    internal ScalarJet BankAt(double station, EvaluationSide side, ReconstructionWork work)
    {
        var (jet, error) = EvaluateBank(station, side, work);
        Motion.RecordErrors(error, 0, station, work);
        return jet;
    }

    private (ScalarJet Jet, double StationError) EvaluateBank(double station, EvaluationSide side, ReconstructionWork work)
    {
        if (banking is null) { return (new(0), 0); }
        var (coordinate, limit) = banking.Resolve(station, Motion.OriginStation + station, side);
        return (Evaluate(request.Banking!, coordinate, limit, station, ProfileDomain.Distance, work, channelId: "Banking"),
            StationResidual(coordinate, station));
    }

    private double StationResidual(double authored, double local) => System.Math.Abs((authored - Motion.OriginStation) - local);

    private ResolvedAxis ResolveAxis(ProfileDomain domain, double minimum, double maximum,
        IEnumerable<double> breakpoints, ReconstructionWork work)
    {
        bool time = domain == ProfileDomain.Time;
        double first = time ? Motion.First.Time : Motion.First.Motion.HeartlineStationMetres;
        double last = time ? Motion.Last.Time : Motion.Last.Motion.HeartlineStationMetres;
        TrajectoryErrorEstimate error = Motion.Last.Motion.EstimatedAbsoluteError;
        double endpointError = time
            ? error.ElapsedTimeSeconds + System.Math.Abs((last - first) - Motion.Last.Motion.ElapsedSeconds)
            : error.StationMetres;
        if (minimum > first || (maximum < last && last - maximum > endpointError) || maximum <= first)
        { throw work.Fail(HeartlineFailureCode.InvalidInput, "The profile does not cover the complete resolved traversal.",
            authored: (domain, first, last)); }
        // An endpoint within the M2 estimate is the same resolved event. Retain its exact authored limit.
        double lastCoordinate = System.Math.Abs(last - maximum) <= endpointError ? maximum : last;
        var points = new List<(double Station, double Coordinate)> { (0, first) };
        foreach (double coordinate in breakpoints)
        {
            work.Charge();
            if (coordinate >= last && coordinate < lastCoordinate)
            { throw work.Fail(HeartlineFailureCode.MotionMappingFailed,
                "A breakpoint lies within the unresolved traversal endpoint interval.",
                (Motion.Length, Motion.Length), (domain, last, lastCoordinate)); }
            if (!(coordinate > first && coordinate < lastCoordinate && coordinate < last)) { continue; }
            double station;
            if (time)
            {
                HeartlineMotionMap.MappedMotion mapped = Motion.AtTime(coordinate, EvaluationSide.Right, work);
                station = mapped.Motion.HeartlineStationMetres - Motion.OriginStation;
                AddMotionPin(station, mapped.Native, work, coordinate);
            }
            else
            {
                station = coordinate - Motion.OriginStation;
                if (Motion.Domain == ProfileDomain.Distance) { AddMotionPin(station, coordinate, work); }
            }
            if (!(station > points[^1].Station && station < Motion.Length))
            { throw work.Fail(HeartlineFailureCode.MotionMappingFailed, "Distinct profile breakpoints cannot be resolved as distinct stations.",
                (station, station), (domain, coordinate, coordinate)); }
            points.Add((station, coordinate));
        }
        if (time && Motion.Domain == ProfileDomain.Distance && lastCoordinate == maximum)
        { AddMotionPin(Motion.Length, Motion.Last.Native, work, lastCoordinate); }
        points.Add((Motion.Length, lastCoordinate));
        return new(points);
    }

    private void AddMotionPin(double station, double coordinate, ReconstructionWork work, double? authoredTime = null)
    {
        if (motionPins.TryAdd(station, (coordinate, authoredTime))) { return; }
        var existing = motionPins[station];
        if (existing.Native != coordinate || (existing.AuthoredTime.HasValue && authoredTime.HasValue
            && existing.AuthoredTime != authoredTime))
        { throw work.Fail(HeartlineFailureCode.MotionMappingFailed, "Distinct motion coordinates resolve to the same station.",
            (station, station), (Motion.Domain, existing.Native, coordinate)); }
        if (authoredTime.HasValue) { motionPins[station] = (coordinate, authoredTime); }
    }

    private int RegionIndex(double coordinate, EvaluationSide side)
    {
        int index = Array.BinarySearch(regionEnds, coordinate);
        return index < 0 ? ~index : side == EvaluationSide.Left ? index : System.Math.Min(index + 1, regionEnds.Length - 1);
    }

    private static ScalarJet Evaluate(ScalarTimeline channel, double coordinate, EvaluationSide side, double station,
        ProfileDomain domain, ReconstructionWork work, int? region = null, string? channelId = null)
    {
        work.Charge(station);
        try { return channel.Evaluate(coordinate, side); }
        catch (Exception ex) when (ex is not OutOfMemoryException)
        { throw work.Fail(HeartlineFailureCode.EvaluationFailed, "An authored channel failed to evaluate.", (station, station),
            (domain, coordinate, coordinate), region, channelId, side, inner: ex); }
    }

    private sealed class ResolvedAxis
    {
        private readonly double[] coordinates;
        internal double[] Stations { get; }
        internal IEnumerable<(double Station, double Coordinate)> Interior => Stations.Zip(coordinates).Skip(1).SkipLast(1);
        internal IEnumerable<double> InteriorCoordinates => coordinates.Skip(1).SkipLast(1);

        internal ResolvedAxis(List<(double Station, double Coordinate)> points)
        {
            Stations = points.Select(point => point.Station).ToArray();
            coordinates = points.Select(point => point.Coordinate).ToArray();
        }

        internal (double Coordinate, EvaluationSide Side) Resolve(double station, double coordinate, EvaluationSide side)
        {
            int index = Array.BinarySearch(Stations, station);
            if (index >= 0) { return (coordinates[index], side); }
            int upper = ~index;
            if (coordinate <= coordinates[upper - 1]) { return (coordinates[upper - 1], EvaluationSide.Right); }
            if (coordinate >= coordinates[upper]) { return (coordinates[upper], EvaluationSide.Left); }
            return (coordinate, side);
        }
    }
}

internal readonly record struct ReconstructionTarget(double Coordinate, LongitudinalMotionSample Motion,
    double StationError, double TimeError, double VelocityError, double Bank, double Left, double Up, Vector3d AngularVelocity);
