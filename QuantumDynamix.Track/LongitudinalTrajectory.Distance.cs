using System;
using System.Linq;
using System.Numerics;
using QuantumDynamix.Math.Functions;
using QuantumDynamix.Math.Numerics;
using QuantumDynamix.Math.Timelines;

namespace QuantumDynamix.Track;

public sealed partial class LongitudinalTrajectory
{
    internal static LongitudinalTrajectory BuildDistance(LongitudinalAccelerationProfile profile, TrajectoryPanel[] panels,
        double initialSpeed, DistanceTrajectorySettings settings, AccelerationRangeProvider? customRanges)
    {
        var work = new TrajectoryWork(ProfileDomain.Distance, settings.MaximumDepth, settings.MaximumWork);
        ForwardPanel[] forwardPanels = ForwardSpeedValidation.Validate(panels, initialSpeed, customRanges, work,
            out BigInteger minimumSquaredSpeed);
        var solution = new DistanceSolution(forwardPanels, profile.End - profile.Start, initialSpeed, minimumSquaredSpeed, settings, work);
        return new(profile, forwardPanels.Select(panel => panel.Panel.End).Prepend(profile.Start).ToArray(), null, solution);
    }

    private sealed class DistanceSolution
    {
        private readonly ForwardPanel[] panels;
        private readonly DistancePrefix[] prefixes;
        private readonly double length;
        private readonly double squaredSpeedBudget;
        private readonly BigInteger exactMinimumSquaredSpeed;
        internal DistanceTrajectorySettings Settings { get; }
        internal double InitialSpeed { get; }
        internal double SpeedLowerBound { get; }
        internal double SpeedUpperBound { get; }
        internal double AccelerationMagnitude { get; }

        internal DistanceSolution(ForwardPanel[] panels, double length, double initialSpeed, BigInteger exactMinimumSquaredSpeed,
            DistanceTrajectorySettings settings, TrajectoryWork work)
        {
            this.panels = panels;
            this.length = length;
            Settings = settings;
            InitialSpeed = initialSpeed;
            this.exactMinimumSquaredSpeed = exactMinimumSquaredSpeed;

            double minimumSquaredSpeed = panels.Min(panel => panel.SquaredSpeedLower);
            SpeedLowerBound = System.Math.BitDecrement(System.Math.Sqrt(minimumSquaredSpeed));
            SpeedUpperBound = System.Math.BitIncrement(System.Math.Sqrt(panels.Max(panel => panel.SquaredSpeedUpper)));
            AccelerationMagnitude = panels.Max(panel => System.Math.Max(System.Math.Abs(panel.Range.LowerMetresPerSecondSquared),
                System.Math.Abs(panel.Range.UpperMetresPerSecondSquared)));
            double halfMinimum = minimumSquaredSpeed / 2;
            double rootHalfMinimum = System.Math.Sqrt(halfMinimum);
            // A q error of at most half the proved minimum leaves a positive margin for the reciprocal square root.
            // The elapsed-time share for q error is half the total target, using the worst sensitivity over the traversal.
            double elapsedLimit = settings.ElapsedTimeAbsoluteToleranceSeconds / length * rootHalfMinimum * halfMinimum;
            squaredSpeedBudget = System.Math.Min(halfMinimum,
                System.Math.Min(settings.VelocityAbsoluteToleranceMetresPerSecond * System.Math.Sqrt(minimumSquaredSpeed), elapsedLimit));

            double initialSquaredSpeed = initialSpeed * initialSpeed;
            double initialError = System.Math.Abs(System.Math.FusedMultiplyAdd(initialSpeed, initialSpeed, -initialSquaredSpeed));
            if (!(initialSquaredSpeed > 0) || !double.IsFinite(initialSquaredSpeed))
            {
                throw work.Failure(TrajectoryFailureCode.NumericalRangeExceeded, panels[0].Panel,
                    "Initial squared speed exceeds usable double range.");
            }
            prefixes = new DistancePrefix[panels.Length + 1];
            prefixes[0] = new(new(initialSquaredSpeed), initialError, default, 0, default);
            for (int i = 0; i < panels.Length; i++)
            { prefixes[i + 1] = Advance(panels[i], panels[i].Panel.End, prefixes[i], work); }
        }

        internal bool MeetsSpeedFloor(double minimumSpeed) =>
            ForwardSpeedValidation.MeetsSquaredSpeedFloor(exactMinimumSquaredSpeed, minimumSpeed);

        internal DistancePrefix At(double coordinate, int index, TrajectoryWork work)
        {
            if (index >= 0) { return prefixes[index]; }
            int lower = ~index - 1;
            return Advance(panels[lower], coordinate, prefixes[lower], work);
        }

        internal void CheckAcceleration(double coordinate, int index, EvaluationSide side, double acceleration, TrajectoryWork work)
        {
            int panelIndex = index switch
            {
                < 0 => ~index - 1,
                0 => 0,
                _ when index == panels.Length || side == EvaluationSide.Left => index - 1,
                _ => index
            };
            ForwardPanel panel = panels[panelIndex];
            if (panel.ProvidedRange is AccelerationRange range)
            { ForwardSpeedValidation.CheckObserved(work, panel.Panel, range, coordinate, acceleration); }
        }

        private DistancePrefix Advance(ForwardPanel forward, double end, DistancePrefix prefix, TrajectoryWork work)
        {
            TrajectoryPanel panel = forward.Panel;
            double span = end - panel.Start;
            double fraction = span / length;
            double squaredSpeedTolerance = squaredSpeedBudget / 2 * fraction;
            double halfLower = forward.SquaredSpeedLower / 2;
            double errorAllowance = prefix.SquaredSpeedError + squaredSpeedTolerance + 4 * Roundoff(forward.SquaredSpeedUpper);
            if (!(errorAllowance <= squaredSpeedBudget) || !(halfLower > 0))
            {
                throw work.Failure(TrajectoryFailureCode.NumericalBudgetExceeded, panel.Start, end, panel.RegionIndex,
                    "The allocated squared-speed accuracy cannot support elapsed-time integration at this speed margin.",
                    (forward.SquaredSpeedLower, forward.SquaredSpeedUpper));
            }

            var (squaredSpeed, squaredSpeedError) = SquaredSpeedAt(end);
            QuadratureResult timeChange = work.Integrate(panel, end, Settings.ElapsedTimeAbsoluteToleranceSeconds / 4 * fraction,
                coordinate =>
                {
                    double value = coordinate == end ? squaredSpeed.Value : SquaredSpeedAt(coordinate).SquaredSpeed.Value;
                    return 1 / System.Math.Sqrt(value);
                });

            Accumulated elapsedTime = prefix.ElapsedTime.Add(timeChange.Value);
            double timeFromSquaredSpeedError = errorAllowance / halfLower / System.Math.Sqrt(halfLower) / 2 * span;
            double elapsedTimeError = prefix.ElapsedTimeError + timeChange.EstimatedAbsoluteError + timeFromSquaredSpeedError
                + Roundoff(timeChange.Value);
            double velocity = System.Math.Sqrt(squaredSpeed.Value);
            double velocityError = (squaredSpeedError + squaredSpeed.OutputRounding)
                / (velocity + System.Math.Sqrt(forward.SquaredSpeedLower)) + Roundoff(velocity);
            double reportedTimeError = elapsedTimeError + elapsedTime.OutputRounding;
            if (!double.IsFinite(elapsedTime.Value) || !double.IsFinite(reportedTimeError) || !double.IsFinite(velocityError))
            {
                throw work.Failure(TrajectoryFailureCode.NumericalRangeExceeded, panel.Start, end, panel.RegionIndex,
                    "Distance trajectory state or error exceeds finite range.");
            }
            var error = new TrajectoryErrorEstimate(reportedTimeError, 0, velocityError);
            if (error.ElapsedTimeSeconds > Settings.ElapsedTimeAbsoluteToleranceSeconds
                || error.VelocityMetresPerSecond > Settings.VelocityAbsoluteToleranceMetresPerSecond)
            {
                throw work.Failure(TrajectoryFailureCode.NumericalBudgetExceeded, panel.Start, end, panel.RegionIndex,
                    "The accumulated elapsed-time or velocity error estimate exceeds its absolute budget.", error: error);
            }
            return new(squaredSpeed, squaredSpeedError, elapsedTime, elapsedTimeError, error);

            (Accumulated SquaredSpeed, double EstimatedError) SquaredSpeedAt(double coordinate)
            {
                Accumulated value = prefix.SquaredSpeed;
                double estimate = prefix.SquaredSpeedError;
                if (coordinate != panel.Start)
                {
                    QuadratureResult change = work.Integrate(panel, coordinate, squaredSpeedTolerance, station =>
                    {
                        double acceleration = work.EvaluateCharged(station, in panel).Value;
                        if (forward.ProvidedRange is AccelerationRange range)
                        { ForwardSpeedValidation.CheckObserved(work, panel, range, station, acceleration); }
                        return 2 * acceleration;
                    });
                    value = value.Add(change.Value);
                    estimate += change.EstimatedAbsoluteError + Roundoff(change.Value);
                }
                if (!double.IsFinite(value.Value) || !double.IsFinite(estimate + value.OutputRounding))
                {
                    throw work.Failure(TrajectoryFailureCode.NumericalRangeExceeded, panel.Start, coordinate, panel.RegionIndex,
                        "Integrated squared speed or its error estimate exceeds finite range.");
                }
                if (value.Value < halfLower || estimate + value.OutputRounding > errorAllowance)
                {
                    throw work.Failure(TrajectoryFailureCode.NumericalBudgetExceeded, panel.Start, coordinate, panel.RegionIndex,
                        "The numerical squared-speed value cannot support the allocated positive-speed margin or error budget.",
                        (forward.SquaredSpeedLower, forward.SquaredSpeedUpper));
                }
                return (value, estimate);
            }
        }
    }

    private readonly record struct DistancePrefix(Accumulated SquaredSpeed, double SquaredSpeedError,
        Accumulated ElapsedTime, double ElapsedTimeError, TrajectoryErrorEstimate Error);
}
