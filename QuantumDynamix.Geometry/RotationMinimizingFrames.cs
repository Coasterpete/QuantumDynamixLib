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
        double[] seeds = Enumerable.Range(0, 65).Select(i => i == 64 ? curve.Maximum : curve.Minimum + (curve.Maximum - curve.Minimum) * i / 64)
            .Concat(curve.Definition.Breakpoints).Distinct().Order().ToArray();
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
        var output = new List<FrameNode>();
        double error = 0;
        Advance(nodes[low].Parameter, nodes[low].Frame, parameter, 0, output, ref error);
        return output[^1].Frame;
    }

    private void Advance(double a, CurveFrame start, double b, int depth, List<FrameNode> output, ref double error)
    {
        double middle = a + (b - a) / 2;
        if (!(middle > a && middle < b)) { throw new NumericalConvergenceException("Frame parameter subdivision cannot be resolved.", depth, b - a); }
        CurveJet3d midpoint = Curve.Evaluate(middle, EvaluationSide.Right);
        CurveJet3d endpoint = Curve.Evaluate(b, EvaluationSide.Left);
        CurveFrame half = Transport(start, midpoint.Position, midpoint.FirstDerivative.Normalized());
        CurveFrame fine = Transport(half, endpoint.Position, endpoint.FirstDerivative.Normalized());
        double estimate;
        Vector3d chord = endpoint.Position - start.Position;
        if (chord.Length == 0) { estimate = double.PositiveInfinity; }
        else { estimate = Angle(Transport(start, endpoint.Position, endpoint.FirstDerivative.Normalized()).Up, fine.Up); }
        double budget = angularTolerance * ((b - a) / (Curve.Maximum - Curve.Minimum));
        if (estimate <= budget)
        {
            output.Add(new(middle, half));
            output.Add(new(b, fine));
            error += estimate;
            return;
        }
        if (depth >= maximumDepth) { throw new NumericalConvergenceException("Frame angular error estimate exceeds its budget (radians).", depth, estimate); }
        Advance(a, start, middle, depth + 1, output, ref error);
        Advance(middle, output[^1].Frame, b, depth + 1, output, ref error);
    }

    private static CurveFrame Transport(CurveFrame start, Vector3d position, Vector3d tangent)
    {
        Vector3d chord = (position - start.Position).Normalized();
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
        return new(position, tangent, Vector3d.Cross(up, tangent).Normalized(), up);
    }

    private static double Angle(Vector3d a, Vector3d b) => System.Math.Atan2(Vector3d.Cross(a, b).Length, Vector3d.Dot(a, b));
}
