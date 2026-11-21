using System;
using System.Linq;
using QuantumDynamix.Math.Functions;
using QuantumDynamix.Math.Numerics;
using QuantumDynamix.Math.Timelines;

namespace QuantumDynamix.Track;

/// <summary>Integrated longitudinal motion. Queries use immutable prefixes and receive independent work budgets.</summary>
public sealed partial class LongitudinalTrajectory
{
    private readonly double[] coordinates;
    private readonly TimeSolution? time;
    private readonly DistanceSolution? distance;

    private LongitudinalTrajectory(LongitudinalAccelerationProfile profile, double[] coordinates,
        TimeSolution? time, DistanceSolution? distance)
    {
        Profile = profile;
        this.coordinates = coordinates;
        this.time = time;
        this.distance = distance;
    }

    public LongitudinalAccelerationProfile Profile { get; }

    /// <summary>Returns motion at the requested authoring second or heartline metre, according to the profile domain.</summary>
    public LongitudinalMotionSample Sample(double coordinate, EvaluationSide side = EvaluationSide.Automatic)
    {
        Check.Argument(coordinate, Profile.Start, Profile.End, side);
        TrajectoryWork work = QueryWork();
        ScalarJet acceleration = Acceleration(coordinate, side, work);
        int index = Array.BinarySearch(coordinates, coordinate);
        if (time is not null)
        {
            TimePrefix state = time.At(coordinate, index, work);
            return new(coordinate - Profile.Start, state.Station.Value, state.Velocity.Value, acceleration.Value, state.Error);
        }

        DistanceSolution forward = distance!;
        forward.CheckAcceleration(coordinate, index, side, acceleration.Value, work);
        DistancePrefix distanceState = forward.At(coordinate, index, work);
        double velocity = index == 0 ? forward.InitialSpeed : System.Math.Sqrt(distanceState.SquaredSpeed.Value);
        return new(distanceState.ElapsedTime.Value, coordinate, velocity, acceleration.Value, distanceState.Error);
    }

    /// <summary>Returns acceleration in m/s² and available derivatives with respect to time.</summary>
    public ScalarJet EvaluateTimeAcceleration(double coordinate, EvaluationSide side = EvaluationSide.Automatic)
    {
        Check.Argument(coordinate, Profile.Start, Profile.End, side);
        TrajectoryWork work = QueryWork();
        ScalarJet acceleration = Acceleration(coordinate, side, work);
        if (distance is null) { return acceleration; }

        int index = Array.BinarySearch(coordinates, coordinate);
        distance.CheckAcceleration(coordinate, index, side, acceleration.Value, work);
        if (acceleration.DerivativeOrder == 0) { return acceleration; }
        DistancePrefix state = distance.At(coordinate, index, work);
        double velocity = index == 0 ? distance.InitialSpeed : System.Math.Sqrt(state.SquaredSpeed.Value);
        try
        {
            double jerk = acceleration.DerivativeOrder >= 3 ? acceleration.FirstDerivative * velocity : 0;
            return ProfileKinematics.ToTimeDerivatives(acceleration, velocity, acceleration.Value, jerk);
        }
        catch (ArgumentOutOfRangeException exception)
        {
            throw work.Failure(TrajectoryFailureCode.NumericalRangeExceeded, coordinate, coordinate,
                Profile.RegionIndexAt(coordinate, side), "The time acceleration derivatives exceed finite range.", inner: exception);
        }
    }

    private ScalarJet Acceleration(double coordinate, EvaluationSide side, TrajectoryWork work) =>
        work.Evaluate(coordinate, Profile.RegionIndexAt(coordinate, side), () => Profile.Evaluate(coordinate, side));

    private TrajectoryWork QueryWork() => time is not null
        ? new(Profile.Domain, time.Settings.MaximumDepth, time.Settings.MaximumWork)
        : new(Profile.Domain, distance!.Settings.MaximumDepth, distance.Settings.MaximumWork);

    internal static LongitudinalTrajectory BuildTime(LongitudinalAccelerationProfile profile, TrajectoryPanel[] panels,
        TimeMotionInitialConditions initial, TimeTrajectorySettings settings)
    {
        var work = new TrajectoryWork(ProfileDomain.Time, settings.MaximumDepth, settings.MaximumWork);
        var solution = new TimeSolution(panels, profile.End - profile.Start, initial, settings, work);
        return new(profile, panels.Select(panel => panel.End).Prepend(profile.Start).ToArray(), solution, null);
    }

    private sealed class TimeSolution
    {
        private readonly TrajectoryPanel[] panels;
        private readonly TimePrefix[] prefixes;
        private readonly double duration;
        private readonly double velocityQuadratureBudget;
        internal TimeTrajectorySettings Settings { get; }

        internal TimeSolution(TrajectoryPanel[] panels, double duration, TimeMotionInitialConditions initial,
            TimeTrajectorySettings settings, TrajectoryWork work)
        {
            this.panels = panels;
            this.duration = duration;
            Settings = settings;
            // Velocity errors carry forward into station for at most the complete duration.
            velocityQuadratureBudget = System.Math.Min(settings.VelocityAbsoluteToleranceMetresPerSecond,
                settings.StationAbsoluteToleranceMetres / 2 / duration) / 2;
            prefixes = new TimePrefix[panels.Length + 1];
            prefixes[0] = new(new(initial.HeartlineStationMetres), new(initial.HeartlineVelocityMetresPerSecond), 0, 0);
            for (int i = 0; i < panels.Length; i++)
            { prefixes[i + 1] = Advance(panels[i], panels[i].End, prefixes[i], work); }
        }

        internal TimePrefix At(double coordinate, int index, TrajectoryWork work)
        {
            if (index >= 0) { return prefixes[index]; }
            int lower = ~index - 1;
            return Advance(panels[lower], coordinate, prefixes[lower], work);
        }

        private TimePrefix Advance(TrajectoryPanel panel, double end, TimePrefix prefix, TrajectoryWork work)
        {
            double span = end - panel.Start;
            double fraction = span / duration;
            QuadratureResult velocityChange = work.Integrate(panel, end, velocityQuadratureBudget * fraction, AccelerationAt);
            QuadratureResult displacement = work.Integrate(panel, end, Settings.StationAbsoluteToleranceMetres / 4 * fraction,
                coordinate => (end - coordinate) * AccelerationAt(coordinate));

            double drift = span * prefix.Velocity.Value;
            Accumulated velocity = prefix.Velocity.Add(velocityChange.Value);
            Accumulated station = prefix.Station.Add(drift).Add(displacement.Value);
            double velocityError = prefix.VelocityError + velocityChange.EstimatedAbsoluteError + Roundoff(velocityChange.Value);
            double stationError = prefix.StationError + span * (prefix.VelocityError + prefix.Velocity.OutputRounding)
                + displacement.EstimatedAbsoluteError + 2 * Roundoff(drift) + Roundoff(displacement.Value);
            var next = new TimePrefix(station, velocity, stationError, velocityError);
            if (!double.IsFinite(station.Value) || !double.IsFinite(velocity.Value)
                || !double.IsFinite(stationError + station.OutputRounding) || !double.IsFinite(velocityError + velocity.OutputRounding))
            {
                throw work.Failure(TrajectoryFailureCode.NumericalRangeExceeded, panel.Start, end, panel.RegionIndex,
                    "Time trajectory state or error exceeds finite range.");
            }
            TrajectoryErrorEstimate error = next.Error;
            if (error.StationMetres > Settings.StationAbsoluteToleranceMetres
                || error.VelocityMetresPerSecond > Settings.VelocityAbsoluteToleranceMetresPerSecond)
            {
                throw work.Failure(TrajectoryFailureCode.NumericalBudgetExceeded, panel.Start, end, panel.RegionIndex,
                    "The accumulated station or velocity error estimate exceeds its absolute budget.", error: error);
            }
            return next;

            double AccelerationAt(double coordinate) => work.EvaluateCharged(coordinate, panel.RegionIndex,
                () => panel.Evaluate(coordinate)).Value;
        }
    }

    private readonly record struct TimePrefix(Accumulated Station, Accumulated Velocity, double StationError, double VelocityError)
    {
        internal TrajectoryErrorEstimate Error => new(0, StationError + Station.OutputRounding, VelocityError + Velocity.OutputRounding);
    }

    // Retain low bits when a small panel increment is added to a large prefix.
    private readonly record struct Accumulated(double High, double Low = 0)
    {
        internal double Value => High + Low;
        internal double OutputRounding => System.Math.Abs(Low - (Value - High));

        internal Accumulated Add(double value)
        {
            double sum = High + value;
            double offset = sum - High;
            double error = (High - (sum - offset)) + (value - offset);
            double tail = Low + error;
            double result = sum + tail;
            double remainder = result - sum;
            return new(result, (sum - (result - remainder)) + (tail - remainder));
        }
    }

    private static double Roundoff(double value) => System.Math.Abs(value) * 1.1102230246251565e-16;
}
