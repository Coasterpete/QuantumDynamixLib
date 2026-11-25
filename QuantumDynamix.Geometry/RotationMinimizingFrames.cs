using System;
using System.Collections.Generic;
using System.Linq;
using QuantumDynamix.Math;
using QuantumDynamix.Math.Functions;
using QuantumDynamix.Math.Numerics;
using QuantumDynamix.Splines.Curves;

namespace QuantumDynamix.Geometry;

/// <summary>Right-handed forward/left/up axes. Positive bank rotates left toward up about forward.</summary>
public readonly record struct CurveFrame(Vector3d Position, Vector3d Forward, Vector3d Left, Vector3d Up)
{
    public Vector3d Transform(Vector3d local) => Forward * local.X + Left * local.Y + Up * local.Z;
    public CurveFrame Banked(double radians)
    {
        Check.Finite(radians, nameof(radians));
        double c = System.Math.Cos(radians);
        double s = System.Math.Sin(radians);
        return new(Position, Forward, Left * c + Up * s, Up * c - Left * s);
    }
}

public readonly record struct FrameNode(double Parameter, CurveFrame Frame);

/// <summary>Deterministic double-reflection transport with adaptive angular error estimates.
/// No automatic distribution of closed-loop twist; initial up is explicitly supplied.</summary>
public sealed class RotationMinimizingFrames
{
    private readonly FrameNode[] nodes;
    private readonly double angularTolerance;
    private readonly int maximumDepth;
    public SplineCurve3d Curve { get; }
    public IReadOnlyList<FrameNode> Nodes { get; }
    public double? ClosureTwist { get; }
    public double EstimatedAccumulatedAngularError { get; }

    public RotationMinimizingFrames(SplineCurve3d curve, Vector3d initialUp, double angularTolerance = 1e-8, int maximumDepth = 20)
    {
        ArgumentNullException.ThrowIfNull(curve);
        if (!double.IsFinite(angularTolerance) || angularTolerance <= 0 || maximumDepth < 1) { throw new ArgumentOutOfRangeException(nameof(angularTolerance)); }
        Curve = curve;
        this.angularTolerance = angularTolerance;
        this.maximumDepth = maximumDepth;
        if (curve.Definition.Breakpoints.Any(x => curve.Definition.ContinuityAt(x) < 1))
        { throw new ArgumentException("Frame transport requires at least C1 continuity at interior knots.", nameof(curve)); }
        CurveJet3d start = curve.Evaluate(curve.Minimum, EvaluationSide.Right);
        Vector3d tangent = start.FirstDerivative.Normalized();
        Vector3d seed = initialUp.Normalized();
        Vector3d projected = seed - tangent * Vector3d.Dot(seed, tangent);
        if (projected.Length <= 1e-12) { throw new ArgumentException("Initial up must not be parallel to the starting tangent.", nameof(initialUp)); }
        Vector3d up = projected.Normalized();
        CurveFrame initial = new(start.Position, tangent, Vector3d.Cross(up, tangent).Normalized(), up);
        var output = new List<FrameNode> { new(curve.Minimum, initial) };
        double error = 0;
        var seeds = new SortedSet<double>(curve.Definition.Breakpoints) { curve.Minimum, curve.Maximum };
        for (int i = 1; i < 64; i++)
        {
            double parameter = curve.Minimum + (curve.Maximum - curve.Minimum) * i / 64;
            // Leave room for the adaptive midpoint on each side of an optional seed.
            // Mandatory boundaries are inserted first and are always retained.
            if (parameter > curve.Minimum && parameter < curve.Maximum
                && !seeds.Contains(System.Math.BitDecrement(parameter))
                && !seeds.Contains(System.Math.BitIncrement(parameter)))
            { seeds.Add(parameter); }
        }
        foreach (double endpoint in seeds.Skip(1)) { Advance(output[^1].Parameter, output[^1].Frame, endpoint, 0, output, ref error); }
        nodes = output.ToArray();
        Nodes = Array.AsReadOnly(nodes);
        EstimatedAccumulatedAngularError = error;
        CurveFrame last = nodes[^1].Frame;
        if ((curve.Definition.IsPeriodic || initial.Position == last.Position)
            && Angle(initial.Forward, last.Forward) <= angularTolerance)
        { ClosureTwist = System.Math.Atan2(Vector3d.Dot(initial.Forward, Vector3d.Cross(initial.Up, last.Up)), Vector3d.Dot(initial.Up, last.Up)); }
    }

    public CurveFrame Evaluate(double parameter)
    {
        Check.Argument(parameter, Curve.Minimum, Curve.Maximum, EvaluationSide.Automatic);
        int low = 0;
        int high = nodes.Length;
        while (low + 1 < high)
        {
            int middle = low + (high - low) / 2;
            if (nodes[middle].Parameter <= parameter) { low = middle; } else { high = middle; }
        }
        if (nodes[low].Parameter == parameter) { return nodes[low].Frame; }
        if (System.Math.BitIncrement(nodes[low].Parameter) == parameter)
        { return EvaluateAdjacent(parameter, nodes[low]); }
        if (high < nodes.Length && System.Math.BitIncrement(parameter) == nodes[high].Parameter)
        { return EvaluateAdjacent(parameter, nodes[high]); }
        var output = new List<FrameNode>();
        double error = 0;
        Advance(nodes[low].Parameter, nodes[low].Frame, parameter, 0, output, ref error);
        return output[^1].Frame;
    }

    private CurveFrame EvaluateAdjacent(double parameter, FrameNode node)
    {
        CurveFrame reference = node.Frame;
        CurveJet3d jet = Curve.Evaluate(parameter, EvaluationSide.Right);
        Vector3d tangent = jet.FirstDerivative.Normalized();
        Vector3d bisector = reference.Forward + tangent;
        if (bisector == Vector3d.Zero)
        {
            double a = System.Math.Min(parameter, node.Parameter);
            double b = System.Math.Max(parameter, node.Parameter);
            throw new NumericalConvergenceException(FormattableString.Invariant(
                $"Frame transport for an adjacent query over spline parameters [{a:R}, {b:R}] found opposite tangents ({System.Math.PI:R} rad apart)."),
                0, System.Math.PI) { ParameterInterval = (a, b) };
        }

        // On the normal plane, the shortest tangent rotation agrees with reflection in the plane
        // perpendicular to the tangent bisector. This avoids relying on a possibly rounded-to-zero chord.
        Vector3d normal = bisector.Normalized();
        Vector3d transported = reference.Up - normal * (2 * Vector3d.Dot(normal, reference.Up));
        Vector3d up = (transported - tangent * Vector3d.Dot(tangent, transported)).Normalized();
        return new(jet.Position, tangent, Vector3d.Cross(up, tangent).Normalized(), up);
    }

    private void Advance(double a, CurveFrame start, double b, int depth, List<FrameNode> output, ref double error)
    {
        double middle = a + (b - a) / 2;
        if (!(middle > a && middle < b))
        {
            throw new NumericalConvergenceException(FormattableString.Invariant(
                $"Frame transport cannot resolve a midpoint over spline parameters [{a:R}, {b:R}] at subdivision depth {depth}. The interval width is {b - a:R} in spline parameter units."),
                depth, b - a) { ParameterInterval = (a, b) };
        }
        CurveJet3d midpoint = Curve.Evaluate(middle, EvaluationSide.Right);
        CurveJet3d endpoint = Curve.Evaluate(b, EvaluationSide.Left);
        CurveFrame half = Transport(start, midpoint, a, middle, depth);
        CurveFrame fine = Transport(half, endpoint, middle, b, depth);
        double estimate;
        Vector3d chord = endpoint.Position - start.Position;
        if (chord.Length == 0) { estimate = double.PositiveInfinity; }
        else { estimate = Angle(Transport(start, endpoint, a, b, depth).Up, fine.Up); }
        double budget = angularTolerance * ((b - a) / (Curve.Maximum - Curve.Minimum));
        if (estimate <= budget)
        {
            output.Add(new(middle, half));
            output.Add(new(b, fine));
            error += estimate;
            return;
        }
        if (depth >= maximumDepth)
        {
            bool hasEstimate = double.IsFinite(estimate);
            string detail = hasEstimate
                ? FormattableString.Invariant($"The angular error estimate is {estimate:R} rad against a limit of {budget:R} rad.")
                : "The end-to-end chord rounded to zero, leaving the angular error estimate unavailable.";
            throw new NumericalConvergenceException(FormattableString.Invariant(
                $"Frame transport exhausted the angular error budget over spline parameters [{a:R}, {b:R}] at subdivision depth {depth}. {detail}"),
                depth, estimate)
            {
                ParameterInterval = (a, b),
                ResidualBudget = hasEstimate ? ("Frame angular error estimate", "rad", budget) : null
            };
        }
        Advance(a, start, middle, depth + 1, output, ref error);
        Advance(middle, output[^1].Frame, b, depth + 1, output, ref error);
    }

    private static CurveFrame Transport(CurveFrame start, CurveJet3d endpoint, double a, double b, int depth)
    {
        if (endpoint.FirstDerivative == Vector3d.Zero)
        {
            throw new InvalidOperationException(FormattableString.Invariant(
                $"Frame transport over spline parameters [{a:R}, {b:R}] at subdivision depth {depth} reached a zero tangent at {b:R}."));
        }
        Vector3d tangent = endpoint.FirstDerivative.Normalized();
        Vector3d displacement = endpoint.Position - start.Position;
        if (displacement == Vector3d.Zero)
        {
            throw new InvalidOperationException(FormattableString.Invariant(
                $"Frame transport encountered a zero displacement over spline parameters [{a:R}, {b:R}] at subdivision depth {depth}."));
        }
        Vector3d chord = displacement.Normalized();
        Vector3d reflectedUp = start.Up - chord * (2 * Vector3d.Dot(chord, start.Up));
        Vector3d reflectedTangent = start.Forward - chord * (2 * Vector3d.Dot(chord, start.Forward));
        Vector3d difference = tangent - reflectedTangent;
        Vector3d transported = reflectedUp;
        if (difference.Length > 1e-14)
        {
            Vector3d normal = difference.Normalized();
            transported -= normal * (2 * Vector3d.Dot(normal, reflectedUp));
        }
        Vector3d up = (transported - tangent * Vector3d.Dot(tangent, transported)).Normalized();
        return new(endpoint.Position, tangent, Vector3d.Cross(up, tangent).Normalized(), up);
    }

    private static double Angle(Vector3d a, Vector3d b) => System.Math.Atan2(Vector3d.Cross(a, b).Length, Vector3d.Dot(a, b));
}
