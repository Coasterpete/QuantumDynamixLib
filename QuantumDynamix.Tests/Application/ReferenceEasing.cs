using System;
using QuantumDynamix.Math.Functions;

namespace QuantumDynamix.Tests.Application;

/// <summary>
/// Independent restatement of the documented Transition functions in docs/MathematicalTransitionAuthoring.md.
/// It does not call the library, so a shape regression in the editor or the library is detected.
/// </summary>
internal static class ReferenceEasing
{
    internal static double Evaluate(EasingKind kind, EasingDirection direction, double u,
        double holdStart = .25, double holdEnd = .75, double holdValue = .5) => direction switch
    {
        EasingDirection.In => In(kind, u, holdStart, holdEnd, holdValue),
        EasingDirection.Out => 1 - In(kind, 1 - u, holdStart, holdEnd, holdValue),
        _ => u < .5 ? In(kind, 2 * u, holdStart, holdEnd, holdValue) / 2
            : 1 - In(kind, 2 - 2 * u, holdStart, holdEnd, holdValue) / 2
    };

    private static double In(EasingKind kind, double u, double a, double b, double h) => kind switch
    {
        EasingKind.Linear => u,
        EasingKind.Quadratic => u * u,
        EasingKind.Cubic => u * u * u,
        EasingKind.Quartic => u * u * u * u,
        EasingKind.Quintic => u * u * u * u * u,
        EasingKind.Sine or EasingKind.Cosine => 1 - System.Math.Cos(System.Math.PI * u / 2),
        EasingKind.Plateau => u < a ? h * Shoulder(u / a) : u <= b ? h : h + (1 - h) * Shoulder((u - b) / (1 - b)),
        _ => throw new ArgumentOutOfRangeException(nameof(kind))
    };

    // Seventh-degree smoothstep whose first three derivatives vanish at both ends.
    private static double Shoulder(double x) => x * x * x * x * (35 - 84 * x + 70 * x * x - 20 * x * x * x);
}
