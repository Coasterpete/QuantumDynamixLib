using System;
using System.Linq;
using QuantumDynamix.Math.Functions;
using QuantumDynamix.Math.Timelines;
using QuantumDynamix.Track;

namespace QuantumDynamix.FVD;

/// <summary>Maps the original M2 clock/station pair to local numerical heartline stations.</summary>
internal sealed class HeartlineMotionMap
{
    private readonly HeartlineReconstructionSettings settings;
    private readonly LongitudinalTrajectory trajectory;
    private readonly MotionNode[] nodes;
    private readonly double[] inverseAxis;
    private readonly double[] stationAxis;
    private readonly TimeForwardPanel[]? timeProof;
    private readonly double[]? timeProofEnds;
    private readonly double accelerationMagnitude;

    internal ProfileDomain Domain => trajectory.Profile.Domain;
    internal double OriginStation { get; }
    internal double OriginTime { get; }
    internal double Length { get; }
    internal double LowerSpeed { get; }
    internal MotionNode First => nodes[0];
    internal MotionNode Last => nodes[^1];
    internal ReadOnlySpan<MotionNode> Nodes => nodes;

    internal HeartlineMotionMap(HeartlineReconstructionRequest request, HeartlineReconstructionSettings settings,
        ProfileDomain transverseDomain, ReconstructionWork work)
    {
        this.settings = settings;
        trajectory = request.LongitudinalTrajectory;
        OriginTime = request.AuthoringTimeAtStartSeconds ?? (Domain == ProfileDomain.Time ? trajectory.Profile.Start : 0);
        if ((Domain == ProfileDomain.Time && OriginTime != trajectory.Profile.Start)
            || (Domain == ProfileDomain.Distance && transverseDomain == ProfileDomain.Time
                && request.AuthoringTimeAtStartSeconds is null))
        { throw work.Fail(HeartlineFailureCode.InvalidInput, "The authoring clock must agree with the time trajectory or be supplied for time targets."); }

        double first = trajectory.Profile.Start;
        LongitudinalMotionSample initial = Sample(first, EvaluationSide.Right, work);
        OriginStation = initial.HeartlineStationMetres;
        double speed = initial.HeartlineVelocityMetresPerSecond;
        if (speed <= 0 || speed < settings.MinimumForwardSpeedMetresPerSecond)
        {
            throw work.Fail(speed <= 0 ? HeartlineFailureCode.NonPositiveSpeed : HeartlineFailureCode.SpeedBelowMinimum,
                "The starting velocity cannot support forward reconstruction.", (0, 0), (Domain, first, first),
                measurement: ("Forward speed", "m/s", speed, settings.MinimumForwardSpeedMetresPerSecond));
        }

        if (Domain == ProfileDomain.Time)
        {
            var proofWork = new TrajectoryWork(ProfileDomain.Time, trajectory.TimeSubdivisionDepth, work.Remaining);
            AccelerationRangeProvider? ranges = request.TimeAccelerationBounds is { } callback ? callback.Invoke : null;
            try { timeProof = trajectory.ProveForwardTime(settings.MinimumForwardSpeedMetresPerSecond, ranges, proofWork); }
            catch (TrajectoryException ex)
            {
                work.Charge(count: proofWork.Used);
                throw Translate(ex, work);
            }
            work.Charge(count: proofWork.Used);
            LowerSpeed = timeProof.Min(panel => panel.SpeedLower);
            accelerationMagnitude = timeProof.Max(panel => System.Math.Max(System.Math.Abs(panel.Range.LowerMetresPerSecondSquared),
                System.Math.Abs(panel.Range.UpperMetresPerSecondSquared)));
            timeProofEnds = timeProof.Select(panel => panel.Panel.End).ToArray();
        }
        else
        {
            double enclosedMinimum = trajectory.DistanceSpeedLowerBound;
            accelerationMagnitude = trajectory.DistanceAccelerationMagnitude;
            if (!trajectory.DistanceMeetsSpeedFloor(settings.MinimumForwardSpeedMetresPerSecond))
            {
                throw work.Fail(HeartlineFailureCode.SpeedPositivityUnresolved,
                    "The M2 enclosure does not establish the configured minimum forward speed.",
                    authored: (Domain, first, trajectory.Profile.End),
                    measurement: ("Enclosed minimum forward speed", "m/s", enclosedMinimum, settings.MinimumForwardSpeedMetresPerSecond));
            }
            LowerSpeed = System.Math.Max(enclosedMinimum, settings.MinimumForwardSpeedMetresPerSecond);
        }

        double[] coordinates = trajectory.Profile.Breakpoints.Prepend(first).Append(trajectory.Profile.End).ToArray();
        nodes = new MotionNode[coordinates.Length];
        inverseAxis = new double[coordinates.Length];
        for (int i = 0; i < coordinates.Length; i++)
        {
            double coordinate = coordinates[i];
            LongitudinalMotionSample motion = i == 0 ? initial :
                Sample(coordinate, i == coordinates.Length - 1 ? EvaluationSide.Left : EvaluationSide.Right, work);
            double station = motion.HeartlineStationMetres - OriginStation;
            double time = Domain == ProfileDomain.Time ? coordinate : OriginTime + motion.ElapsedSeconds;
            if (!double.IsFinite(station) || !double.IsFinite(time)
                || (i > 0 && (!(station > nodes[i - 1].Station) || !(time > nodes[i - 1].Time))))
            { throw MappingFailure(work, station, "Motion boundaries are not distinguishable on the station or time axis."); }
            nodes[i] = new(station, coordinate, time, motion);
            inverseAxis[i] = Domain == ProfileDomain.Time ? station : time;
        }
        stationAxis = nodes.Select(node => node.Station).ToArray();
        Length = Last.Station;
        CheckErrors(0, First.Motion, 0, First.Native, work);
        CheckErrors(0, Last.Motion, Length, Last.Native, work);
    }

    internal MappedMotion AtStation(double station, double? pinnedCoordinate, double? pinnedAuthoredTime, EvaluationSide side,
        ReconstructionWork work)
    {
        if (pinnedCoordinate is double exact)
        {
            LongitudinalMotionSample motion = Sample(exact, side, work);
            double residual = System.Math.Abs((motion.HeartlineStationMetres - OriginStation) - station);
            return CheckErrors(residual, motion, station, exact, work, pinnedAuthoredTime);
        }
        if (Domain == ProfileDomain.Time) { return Invert(station, side, work); }
        double authored = OriginStation + station;
        if (!double.IsFinite(authored) || authored < First.Native || authored > Last.Native)
        { throw MappingFailure(work, station, "Local station cannot be resolved on the authored station axis."); }
        int index = Array.BinarySearch(stationAxis, station);
        if (index < 0)
        {
            int upper = ~index;
            double left = nodes[upper - 1].Native;
            double right = nodes[upper].Native;
            if (authored <= left) { authored = left; side = EvaluationSide.Right; }
            if (authored >= right) { authored = right; side = EvaluationSide.Left; }
        }
        return CheckErrors(System.Math.Abs((authored - OriginStation) - station),
            Sample(authored, side, work), station, authored, work);
    }

    internal MappedMotion AtTime(double time, EvaluationSide side, ReconstructionWork work)
    {
        if (Domain == ProfileDomain.Distance) { return Invert(time, side, work); }
        LongitudinalMotionSample motion = Sample(time, side, work);
        return CheckErrors(0, motion, motion.HeartlineStationMetres - OriginStation, time, work);
    }

    private MappedMotion Invert(double target, EvaluationSide side, ReconstructionWork work)
    {
        bool byStation = Domain == ProfileDomain.Time;
        int index = Array.BinarySearch(inverseAxis, target);
        if (index >= 0)
        {
            MotionNode node = nodes[index];
            return CheckErrors(0, Sample(node.Native, side, work), node.Station, node.Native, work,
                byStation ? null : target);
        }
        int upper = ~index;
        if (upper == 0 || upper == nodes.Length)
        { throw MappingFailure(work, byStation ? target : null, "The inversion target is outside the traversal."); }

        double low = nodes[upper - 1].Native;
        double high = nodes[upper].Native;
        double x = low + (high - low) * ((target - inverseAxis[upper - 1]) /
            (inverseAxis[upper] - inverseAxis[upper - 1]));
        while (true)
        {
            EvaluationSide limit = x == low ? EvaluationSide.Right : x == high ? EvaluationSide.Left : side;
            LongitudinalMotionSample motion = Sample(x, limit, work);
            double station = motion.HeartlineStationMetres - OriginStation;
            double residual = (byStation ? station : OriginTime + motion.ElapsedSeconds) - target;
            double speed = motion.HeartlineVelocityMetresPerSecond;
            double nativeError = byStation ? motion.EstimatedAbsoluteError.StationMetres
                : motion.EstimatedAbsoluteError.ElapsedTimeSeconds + ClockRoundoff(motion.ElapsedSeconds);
            double error = System.Math.Abs(residual) + nativeError;
            double stationError = byStation ? error : StationErrorFromTime(error);
            double timeError = byStation ? error / LowerSpeed : error;
            if (stationError <= settings.StationMappingToleranceMetres && timeError <= settings.TimeMappingToleranceSeconds)
            {
                RecordErrors(stationError, timeError, byStation ? target : station, work);
                return WithErrors(x, byStation ? x : target, motion, stationError, timeError, station, work);
            }
            if (System.Math.Abs(residual) <= nativeError)
            { throw MappingFailure(work, station, "The M2 error estimate prevents resolving the inverse within tolerance."); }
            if (residual < 0) { low = x; } else { high = x; }
            double next = x - (byStation ? residual / speed : residual * speed);
            if (!(next > low && next < high)) { next = low + (high - low) / 2; }
            if (!(next > low && next < high))
            { throw MappingFailure(work, station, "The inverse exhausted representable bracket subdivisions."); }
            x = next;
        }
    }

    private double StationErrorFromTime(double timeError)
    {
        // For distance inversion use a full-traversal speed upper bound, not a local slope estimate.
        return timeError * trajectory.DistanceSpeedUpperBound;
    }

    private MappedMotion CheckErrors(double stationResidual, LongitudinalMotionSample motion,
        double station, double native, ReconstructionWork work, double? authoredTime = null)
    {
        double stationError = stationResidual + motion.EstimatedAbsoluteError.StationMetres;
        double timeError = Domain == ProfileDomain.Time ? stationError / LowerSpeed :
            motion.EstimatedAbsoluteError.ElapsedTimeSeconds + ClockRoundoff(motion.ElapsedSeconds);
        double time = Domain == ProfileDomain.Time ? native : OriginTime + motion.ElapsedSeconds;
        if (Domain == ProfileDomain.Distance && authoredTime is double eventTime)
        {
            // The pin retains the authored event, even when the accepted station only approximates its time.
            timeError += System.Math.Abs(time - eventTime);
            stationError += StationErrorFromTime(timeError);
            time = eventTime;
        }
        RecordErrors(stationError, timeError, station, work);
        return WithErrors(native, time, motion, stationError, timeError, station, work);
    }

    private double ClockRoundoff(double elapsed) => System.Math.Abs((OriginTime + elapsed - OriginTime) - elapsed);

    private MappedMotion WithErrors(double native, double time, LongitudinalMotionSample motion,
        double stationError, double timeError, double station, ReconstructionWork work)
    {
        double timeUncertainty = Domain == ProfileDomain.Time ? timeError : stationError / LowerSpeed;
        double velocityError = motion.EstimatedAbsoluteError.VelocityMetresPerSecond + accelerationMagnitude * timeUncertainty;
        if (!(velocityError < motion.HeartlineVelocityMetresPerSecond))
        {
            throw work.Fail(HeartlineFailureCode.SpeedPositivityUnresolved, "The mapped velocity error leaves no usable speed margin.",
                (station, station), measurement: ("Velocity error estimate", "m/s", velocityError, motion.HeartlineVelocityMetresPerSecond));
        }
        return new(native, time, motion, stationError, timeError, velocityError);
    }

    internal void RecordErrors(double stationError, double timeError, double station, ReconstructionWork work)
    {
        if (!(stationError <= settings.StationMappingToleranceMetres))
        { throw MappingFailure(work, station, "Station mapping exceeds its tolerance.", ("Station error", "m", stationError, settings.StationMappingToleranceMetres)); }
        if (!(timeError <= settings.TimeMappingToleranceSeconds))
        { throw MappingFailure(work, station, "Time mapping exceeds its tolerance.", ("Time error", "s", timeError, settings.TimeMappingToleranceSeconds)); }
        work.StationError = System.Math.Max(work.StationError, stationError);
        work.TimeError = System.Math.Max(work.TimeError, timeError);
    }

    private LongitudinalMotionSample Sample(double coordinate, EvaluationSide side, ReconstructionWork work)
    {
        work.Charge();
        try
        {
            LongitudinalMotionSample sample = trajectory.Sample(coordinate, side);
            if (timeProof is not null)
            {
                int index = Array.BinarySearch(timeProofEnds!, coordinate);
                index = index < 0 ? ~index : side == EvaluationSide.Left ? index : System.Math.Min(index + 1, timeProof.Length - 1);
                TimeForwardPanel proof = timeProof[index];
                double acceleration = sample.LongitudinalAccelerationMetresPerSecondSquared;
                if (proof.IsProvided && (acceleration < proof.Range.LowerMetresPerSecondSquared
                    || acceleration > proof.Range.UpperMetresPerSecondSquared))
                { throw work.Fail(HeartlineFailureCode.InvalidAccelerationBounds, "Observed time acceleration contradicts its enclosure.",
                    authored: (Domain, coordinate, coordinate), region: proof.Panel.RegionIndex, side: side); }
                double v = sample.HeartlineVelocityMetresPerSecond;
                double error = sample.EstimatedAbsoluteError.VelocityMetresPerSecond;
                if (v + error < proof.SpeedLower || v - error > proof.SpeedUpper)
                { throw work.Fail(HeartlineFailureCode.InvalidAccelerationBounds, "The M2 velocity estimate contradicts the propagated time enclosure.",
                    authored: (Domain, coordinate, coordinate), region: proof.Panel.RegionIndex, side: side); }
            }
            double speed = sample.HeartlineVelocityMetresPerSecond;
            if (speed > 0 && speed <= sample.EstimatedAbsoluteError.VelocityMetresPerSecond)
            { throw work.Fail(HeartlineFailureCode.SpeedPositivityUnresolved, "The numerical velocity cannot be resolved above zero.",
                authored: (Domain, coordinate, coordinate)); }
            if (timeProof is not null && !(speed > 0))
            { throw work.Fail(HeartlineFailureCode.NonPositiveSpeed, "A time trajectory sample is nonpositive.", authored: (Domain, coordinate, coordinate)); }
            return sample;
        }
        catch (TrajectoryException ex) { throw Translate(ex, work); }
    }

    private static HeartlineReconstructionException Translate(TrajectoryException ex, ReconstructionWork work)
    {
        HeartlineFailureCode code = ex.Code switch
        {
            TrajectoryFailureCode.MissingAccelerationBounds => HeartlineFailureCode.MissingAccelerationBounds,
            TrajectoryFailureCode.InvalidAccelerationBounds => HeartlineFailureCode.InvalidAccelerationBounds,
            TrajectoryFailureCode.NonPositiveSpeed => HeartlineFailureCode.NonPositiveSpeed,
            TrajectoryFailureCode.SpeedPositivityUnresolved => HeartlineFailureCode.SpeedPositivityUnresolved,
            _ => HeartlineFailureCode.SourceTrajectoryFailed
        };
        return work.Fail(code, ex.Message, authored: (ex.Domain, ex.StartCoordinate, ex.EndCoordinate),
            region: ex.RegionIndex, inner: ex);
    }

    private static HeartlineReconstructionException MappingFailure(ReconstructionWork work, double? station, string message,
        (string, string, double, double)? measurement = null) =>
        work.Fail(HeartlineFailureCode.MotionMappingFailed, message, station is double s && double.IsFinite(s) ? (s, s) : null,
            measurement: measurement);

    internal readonly record struct MotionNode(double Station, double Native, double Time, LongitudinalMotionSample Motion);
    internal readonly record struct MappedMotion(double Native, double Time, LongitudinalMotionSample Motion,
        double StationError, double TimeError, double VelocityError);
}
