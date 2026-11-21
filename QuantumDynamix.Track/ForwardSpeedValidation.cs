using System;
using System.Collections.Generic;
using System.Numerics;
using QuantumDynamix.Math.Functions;

namespace QuantumDynamix.Track;

internal readonly record struct ForwardPanel(TrajectoryPanel Panel, double SquaredSpeedLower, double SquaredSpeedUpper,
    AccelerationRange? ProvidedRange);

/// <summary>Establishes positive squared speed from analytic or caller-supplied acceleration enclosures.</summary>
internal static class ForwardSpeedValidation
{
    internal static ForwardPanel[] Validate(TrajectoryPanel[] panels, double initialSpeed, AccelerationRangeProvider? customRanges,
        TrajectoryWork work)
    {
        BigInteger initial = DoubleUnits(initialSpeed);
        BigInteger initialSquaredSpeed = initial * initial;
        var cells = new List<Cell>(panels.Length);
        foreach (TrajectoryPanel panel in panels) { cells.Add(CreateCell(panel, 0, customRanges, work)); }

        while (true)
        {
            PropagatedRange[] ranges = Propagate(cells, initialSquaredSpeed, work, out int firstInconclusive);
            if (firstInconclusive < 0) { return ToForwardPanels(cells, ranges, work); }

            int lastInconclusive = Array.FindLastIndex(ranges, range => range.Lower.Sign <= 0);
            List<Cell> refined = Refine(cells, lastInconclusive, customRanges, work);
            if (refined.Count == cells.Count)
            {
                TrajectoryPanel panel = cells[firstInconclusive].Panel;
                PropagatedRange range = ranges[firstInconclusive];
                throw work.Failure(TrajectoryFailureCode.SpeedPositivityUnresolved, panel,
                    "The acceleration enclosures could not establish strictly positive squared speed over this interval.",
                    enclosure: ToEnclosure(range.Lower, range.Upper));
            }
            cells = refined;
        }
    }

    internal static void CheckObserved(TrajectoryWork work, TrajectoryPanel panel, AccelerationRange range,
        double coordinate, double observed)
    {
        if (observed < range.LowerMetresPerSecondSquared || observed > range.UpperMetresPerSecondSquared)
        {
            throw work.Failure(TrajectoryFailureCode.InvalidAccelerationBounds, coordinate, coordinate, panel.RegionIndex,
                $"Acceleration {observed:R} m/s² lies outside the supplied interval " +
                $"[{range.LowerMetresPerSecondSquared:R}, {range.UpperMetresPerSecondSquared:R}] m/s².");
        }
    }

    private static Cell CreateCell(TrajectoryPanel panel, int depth, AccelerationRangeProvider? customRanges, TrajectoryWork work,
        AccelerationRange? containingRange = null)
    {
        work.Charge(panel.Start, panel.End, panel.RegionIndex, checkingPositivity: true);
        AccelerationRange? builtIn = AccelerationEnclosures.ForBuiltIn(panel.Segment, panel.Start, panel.End);
        if (builtIn is AccelerationRange known)
        {
            if (!IsValid(known))
            { throw work.Failure(TrajectoryFailureCode.NumericalRangeExceeded, panel, "The built-in acceleration enclosure exceeds finite range."); }
            return new(panel, known, IsProvided: false, depth);
        }

        AccelerationRange? supplied;
        try { supplied = customRanges?.Invoke(panel.Binding, panel.Start, panel.End); }
        catch (Exception exception) when (exception is not OutOfMemoryException)
        {
            throw work.Failure(TrajectoryFailureCode.InvalidAccelerationBounds, panel,
                "The acceleration range provider failed.", inner: exception);
        }
        if (supplied is not AccelerationRange range)
        { throw work.Failure(TrajectoryFailureCode.MissingAccelerationBounds, panel, "This acceleration piece requires a caller-supplied enclosure."); }
        if (!IsValid(range))
        { throw work.Failure(TrajectoryFailureCode.InvalidAccelerationBounds, panel, "Acceleration bounds must be finite and ordered."); }
        if (containingRange is AccelerationRange parent)
        {
            range = new(System.Math.Max(range.LowerMetresPerSecondSquared, parent.LowerMetresPerSecondSquared),
                System.Math.Min(range.UpperMetresPerSecondSquared, parent.UpperMetresPerSecondSquared));
            if (!IsValid(range))
            { throw work.Failure(TrajectoryFailureCode.InvalidAccelerationBounds, panel, "The supplied subinterval bounds contradict the enclosing bounds."); }
        }

        CheckObserved(work, panel, range, panel.Start, work.Evaluate(panel.Start, panel.RegionIndex,
            () => panel.Binding.Evaluate(panel.Start, EvaluationSide.Right), checkingPositivity: true).Value);
        CheckObserved(work, panel, range, panel.End, work.Evaluate(panel.End, panel.RegionIndex,
            () => panel.Binding.Evaluate(panel.End, EvaluationSide.Left), checkingPositivity: true).Value);
        return new(panel, range, IsProvided: true, depth);
    }

    private static bool IsValid(AccelerationRange range) => double.IsFinite(range.LowerMetresPerSecondSquared)
        && double.IsFinite(range.UpperMetresPerSecondSquared)
        && range.LowerMetresPerSecondSquared <= range.UpperMetresPerSecondSquared;

    private static PropagatedRange[] Propagate(List<Cell> cells, BigInteger initialSquaredSpeed, TrajectoryWork work,
        out int firstInconclusive)
    {
        var ranges = new PropagatedRange[cells.Count];
        BigInteger lower = initialSquaredSpeed;
        BigInteger upper = initialSquaredSpeed;
        firstInconclusive = -1;
        for (int i = 0; i < cells.Count; i++)
        {
            Cell cell = cells[i];
            TrajectoryPanel panel = cell.Panel;
            if (cell.Visited) { work.Charge(panel.Start, panel.End, panel.RegionIndex, checkingPositivity: true); }
            cells[i] = cell with { Visited = true };
            // Each double is an integer multiple of 2^-1074. Products and prefix sums below are exact in 2^-2148 units.
            BigInteger twiceWidth = (DoubleUnits(panel.End) - DoubleUnits(panel.Start)) << 1;
            BigInteger endLower = lower + twiceWidth * DoubleUnits(cell.Range.LowerMetresPerSecondSquared);
            BigInteger endUpper = upper + twiceWidth * DoubleUnits(cell.Range.UpperMetresPerSecondSquared);
            if (endUpper.Sign <= 0)
            {
                throw work.Failure(TrajectoryFailureCode.NonPositiveSpeed, panel.End, panel.End, panel.RegionIndex,
                    "The enclosed squared speed is nonpositive at this station.", enclosure: ToEnclosure(endLower, endUpper));
            }
            BigInteger throughoutLower = BigInteger.Min(lower, endLower);
            ranges[i] = new(throughoutLower, BigInteger.Max(upper, endUpper));
            if (firstInconclusive < 0 && throughoutLower.Sign <= 0) { firstInconclusive = i; }
            lower = endLower;
            upper = endUpper;
        }
        return ranges;
    }

    private static List<Cell> Refine(List<Cell> cells, int lastInconclusive, AccelerationRangeProvider? customRanges,
        TrajectoryWork work)
    {
        var refined = new List<Cell>(cells.Count);
        for (int i = 0; i < cells.Count; i++)
        {
            Cell cell = cells[i];
            TrajectoryPanel panel = cell.Panel;
            double middle = panel.Start + (panel.End - panel.Start) / 2;
            if (i > lastInconclusive || cell.Depth >= work.MaximumDepth || middle <= panel.Start || middle >= panel.End
                || cell.Range.LowerMetresPerSecondSquared == cell.Range.UpperMetresPerSecondSquared)
            {
                refined.Add(cell);
                continue;
            }
            AccelerationRange? parent = cell.IsProvided ? cell.Range : null;
            refined.Add(CreateCell(panel with { End = middle }, cell.Depth + 1, customRanges, work, parent));
            refined.Add(CreateCell(panel with { Start = middle }, cell.Depth + 1, customRanges, work, parent));
        }
        return refined;
    }

    private static ForwardPanel[] ToForwardPanels(List<Cell> cells, PropagatedRange[] ranges, TrajectoryWork work)
    {
        var result = new ForwardPanel[cells.Count];
        for (int i = 0; i < result.Length; i++)
        {
            Cell cell = cells[i];
            var enclosure = ToEnclosure(ranges[i].Lower, ranges[i].Upper);
            if (enclosure is not { Lower: > 0 })
            { throw work.Failure(TrajectoryFailureCode.NumericalRangeExceeded, cell.Panel, "The positive squared-speed enclosure exceeds usable double range.", enclosure: enclosure); }
            result[i] = new(cell.Panel, enclosure.Value.Lower, enclosure.Value.Upper, cell.IsProvided ? cell.Range : null);
        }
        return result;
    }

    private static BigInteger DoubleUnits(double value)
    {
        ulong bits = BitConverter.DoubleToUInt64Bits(value);
        ulong significand = bits & ((1UL << 52) - 1);
        int exponent = (int)((bits >> 52) & 0x7ff);
        if (exponent != 0) { significand |= 1UL << 52; }
        BigInteger units = new BigInteger(significand) << System.Math.Max(0, exponent - 1);
        return (bits >> 63) == 0 ? units : -units;
    }

    private static (double Lower, double Upper)? ToEnclosure(BigInteger lower, BigInteger upper)
    {
        double low = RoundOutward(lower, upward: false);
        double high = RoundOutward(upper, upward: true);
        return double.IsFinite(low) && double.IsFinite(high) ? (low, high) : null;
    }

    private static double RoundOutward(BigInteger units, bool upward)
    {
        BigInteger magnitude = BigInteger.Abs(units);
        int shift = System.Math.Max(1074, (int)magnitude.GetBitLength() - 53);
        BigInteger truncated = magnitude >> shift;
        if (upward != (units.Sign < 0) && (truncated << shift) != magnitude) { truncated++; }
        double rounded = System.Math.ScaleB((double)truncated, shift - 2148);
        return units.Sign < 0 ? -rounded : rounded;
    }

    private readonly record struct Cell(TrajectoryPanel Panel, AccelerationRange Range, bool IsProvided, int Depth, bool Visited = false);
    private readonly record struct PropagatedRange(BigInteger Lower, BigInteger Upper);
}
