using System;
using QuantumDynamix.Math.Functions;
using QuantumDynamix.Math.Timelines;

namespace QuantumDynamix.App;

internal readonly record struct TransitionShape(EasingKind? Kind, EasingDirection Direction = EasingDirection.In,
    double HoldStart = .25, double HoldEnd = .75, double HoldValue = .5)
{
    internal static readonly TransitionShape Linear = new(EasingKind.Linear);

    // Preserve the explicit Constant choice even when a Linear span also has equal endpoints.
    private static readonly IScalarFunction Constant = new CustomFunction(0, 1, (_, _) => new(0));

    internal string Name => Kind?.ToString() ?? "Constant";

    /// <summary>The function and, where it changes the shape, its direction.</summary>
    internal string Description => Kind is null or EasingKind.Linear ? Name : $"{Name} {Direction}";

    /// <summary>True when the span's endpoint values are equal, so y₀ + (y₁ − y₀)E(u) is flat for every function.</summary>
    internal static bool IsFlat(TimelineKeyframe start, TimelineKeyframe end) => start.Value == end.Value;

    internal static TransitionShape FromSegment(IScalarFunction segment)
    {
        if (segment is FunctionSegment span)
        {
            if (ReferenceEquals(span.Function, Constant)) { return new(null); }
            if (span.Function is EasingFunction easing && easing.Kind != EasingKind.Custom)
            {
                var (start, end, value) = easing.PlateauParameters;
                return new(easing.Kind, easing.Direction, start, end, value);
            }
        }
        throw new InvalidOperationException("This transition function is not supported by the project editor.");
    }

    internal FunctionSegment Build(TimelineKeyframe start, TimelineKeyframe end)
    {
        if (Kind is null)
        {
            if (start.Value != end.Value)
            {
                throw new ArgumentException("A Constant transition requires equal endpoint values.");
            }
            return new(start.Coordinate, end.Coordinate, Constant, scale: 0, offset: start.Value);
        }
        return FunctionSegment.Transition(start.Coordinate, end.Coordinate, start.Value, end.Value,
            new EasingFunction(Kind.Value, Direction, HoldStart, HoldEnd, HoldValue));
    }
}
