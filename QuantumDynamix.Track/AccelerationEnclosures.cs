using System;
using QuantumDynamix.Math.Functions;
using QuantumDynamix.Math.Timelines;

namespace QuantumDynamix.Track;

/// <summary>Encloses the mathematical values of the existing built-in shapes using outward-rounded arithmetic.</summary>
internal static class AccelerationEnclosures
{
    internal static AccelerationRange? ForBuiltIn(IScalarFunction segment, double start, double end)
    {
        IScalarFunction shape = segment;
        double scale = 1;
        double offset = 0;
        var argument = new Interval(start, end);
        if (segment is FunctionSegment mapped)
        {
            if (mapped.Scale == 0) { return new(mapped.Offset, mapped.Offset); }
            shape = mapped.Function;
            scale = mapped.Scale;
            offset = mapped.Offset;
            Interval width = Interval.Point(mapped.Maximum) - Interval.Point(mapped.Minimum);
            argument = Normalize(argument, mapped.Minimum, width);
        }

        Interval range;
        if (shape is EasingFunction easing && easing.Kind != EasingKind.Custom)
        {
            // All built-in easing directions are nondecreasing on [0,1].
            range = new(EasingAt(easing, argument.Low).Low, EasingAt(easing, argument.High).High);
        }
        else if (shape is RiseHoldFallEnvelope envelope)
        {
            Interval left = EnvelopeAt(envelope, argument.Low);
            Interval right = EnvelopeAt(envelope, argument.High);
            double upper = argument.Low <= envelope.FallStart && argument.High >= envelope.RiseEnd
                ? 1 : System.Math.Max(left.High, right.High);
            range = new(System.Math.Min(left.Low, right.Low), upper);
        }
        else { return null; }

        Interval physical = Interval.Point(offset) + Interval.Point(scale) * range;
        return new(physical.Low, physical.High);
    }

    private static Interval EasingAt(EasingFunction easing, double x)
    {
        if (x is 0 or 1) { return Interval.Point(x); }
        Interval argument = Interval.Point(x);
        return easing.Direction switch
        {
            EasingDirection.In => Base(easing, argument),
            EasingDirection.Out => (Interval.One - Base(easing, Interval.One - argument)).Unit(),
            _ when x <= 0.5 => (Interval.Half * Base(easing, Interval.Two * argument)).Unit(),
            _ => (Interval.One - Interval.Half * Base(easing, Interval.Two * (Interval.One - argument))).Unit()
        };
    }

    private static Interval Base(EasingFunction easing, Interval argument)
    {
        argument = argument.Unit();
        if (easing.Kind == EasingKind.Plateau)
        {
            var (start, end, hold) = easing.PlateauParameters;
            return new(PlateauAt(argument.Low, start, end, hold).Low, PlateauAt(argument.High, start, end, hold).High);
        }
        if (easing.Kind is EasingKind.Sine or EasingKind.Cosine)
        {
            Interval angle = new Interval(System.Math.PI, System.Math.BitIncrement(System.Math.PI)) * Interval.Point(0.25) * argument;
            Interval squared = angle * angle;
            Interval term = angle;
            Interval sum = angle;
            // On [0, pi/4], consecutive partial sums enclose sine by the alternating-series remainder.
            for (int index = 1; index <= 8; index++)
            {
                term = term * squared / Interval.Point(2 * index * (2 * index + 1));
                sum = index % 2 == 0 ? sum + term : sum - term;
            }
            Interval nextTerm = term * squared / Interval.Point(18 * 19);
            var sine = new Interval(System.Math.Max(0, (sum - nextTerm).Low), sum.High);
            return (Interval.Two * sine * sine).Unit();
        }

        int power = easing.Kind switch
        {
            EasingKind.Quadratic => 2,
            EasingKind.Cubic => 3,
            EasingKind.Quartic => 4,
            EasingKind.Quintic => 5,
            _ => 1
        };
        Interval result = argument;
        for (int i = 1; i < power; i++) { result = result * argument; }
        return result.Unit();
    }

    private static Interval PlateauAt(double x, double start, double end, double value)
    {
        Interval hold = Interval.Point(value);
        if (x < start) { return (hold * Shoulder(Interval.Point(x) / Interval.Point(start))).Unit(); }
        if (x <= end) { return hold; }
        Interval position = Normalize(Interval.Point(x), end, Interval.One - Interval.Point(end));
        return (hold + (Interval.One - hold) * Shoulder(position)).Unit();
    }

    private static Interval EnvelopeAt(RiseHoldFallEnvelope envelope, double x)
    {
        if (x < envelope.RiseEnd) { return Shoulder(Interval.Point(x) / Interval.Point(envelope.RiseEnd)); }
        if (x <= envelope.FallStart) { return Interval.One; }
        return Shoulder(Normalize(Interval.Point(-x), -1, Interval.One - Interval.Point(envelope.FallStart)));
    }

    private static Interval Normalize(Interval x, double origin, Interval width) => width.Low > 0
        ? ((x - Interval.Point(origin)) / width).Unit() : new(0, 1);

    private static Interval Shoulder(Interval x)
    {
        x = x.Unit();
        return new(ShoulderAt(x.Low).Low, ShoulderAt(x.High).High);
    }

    private static Interval ShoulderAt(double x)
    {
        if (x is 0 or 1) { return Interval.Point(x); }
        if (x > 0.5)
        {
            // For x > 0.5, 1 - x is exactly representable by Sterbenz's lemma.
            return (Interval.One - ShoulderAt(1 - x)).Unit();
        }
        Interval u = Interval.Point(x);
        Interval square = u * u;
        Interval polynomial = Interval.Point(35) + u * (Interval.Point(-84) + u * (Interval.Point(70) - Interval.Point(20) * u));
        return (square * square * polynomial).Unit();
    }

    private readonly record struct Interval(double Low, double High)
    {
        internal static Interval Half => Point(0.5);
        internal static Interval One => Point(1);
        internal static Interval Two => Point(2);
        internal static Interval Point(double value) => new(value, value);
        internal Interval Unit() => new(System.Math.Clamp(Low, 0, 1), System.Math.Clamp(High, 0, 1));

        public static Interval operator +(Interval a, Interval b)
        {
            if (a == Point(0)) { return b; }
            if (b == Point(0)) { return a; }
            return Outward(a.Low + b.Low, a.High + b.High);
        }

        public static Interval operator -(Interval a, Interval b) => a + new Interval(-b.High, -b.Low);

        public static Interval operator *(Interval a, Interval b)
        {
            if (a == Point(0) || b == Point(0)) { return Point(0); }
            if (a == One) { return b; }
            if (b == One) { return a; }
            return Hull(a.Low * b.Low, a.Low * b.High, a.High * b.Low, a.High * b.High);
        }

        public static Interval operator /(Interval numerator, Interval positiveDenominator) =>
            Hull(numerator.Low / positiveDenominator.Low, numerator.Low / positiveDenominator.High,
                numerator.High / positiveDenominator.Low, numerator.High / positiveDenominator.High);

        private static Interval Hull(double a, double b, double c, double d) =>
            Outward(System.Math.Min(System.Math.Min(a, b), System.Math.Min(c, d)), System.Math.Max(System.Math.Max(a, b), System.Math.Max(c, d)));

        private static Interval Outward(double lower, double upper) => new(System.Math.BitDecrement(lower), System.Math.BitIncrement(upper));
    }
}
