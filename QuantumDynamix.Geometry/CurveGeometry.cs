using System;
using QuantumDynamix.Math;
using QuantumDynamix.Math.Functions;
using QuantumDynamix.Splines.Curves;

namespace QuantumDynamix.Geometry;

/// <summary>Spatial differential geometry. CurvatureVector=dT/ds and its derivative has units 1/m^2.</summary>
public readonly record struct CurveGeometrySample(Vector3d Position, Vector3d Tangent, double ParameterSpeed,
    Vector3d CurvatureVector, Vector3d? CurvatureStationDerivative, double? Torsion)
{
    public double Curvature => CurvatureVector.Length;
}

public static class CurveGeometry
{
    public static CurveGeometrySample Evaluate(SplineCurve3d curve, double parameter, EvaluationSide side = EvaluationSide.Automatic)
    {
        ArgumentNullException.ThrowIfNull(curve);
        return Evaluate(curve.Evaluate(parameter, side));
    }

    internal static CurveGeometrySample Evaluate(CurveJet3d jet)
    {
        Vector3d d1 = jet.FirstDerivative;
        Vector3d d2 = jet.SecondDerivative;
        double speed = d1.Length;
        if (!(speed > 0)) { throw new InvalidOperationException("A stationary curve parameter has no defined tangent or curvature."); }
        Vector3d tangent = d1 / speed;
        double speedRate = Vector3d.Dot(tangent, d2);
        Vector3d tangentRate = d2 / speed - tangent * (speedRate / speed);
        Vector3d curvature = tangentRate / speed;
        Vector3d? curvatureRate = null;
        double? torsion = null;
        if (jet.DerivativeOrder >= 3)
        {
            Vector3d d3 = jet.ThirdDerivative;
            double speedSecond = Vector3d.Dot(tangentRate, d2) + Vector3d.Dot(tangent, d3);
            curvatureRate = ((d3 / speed - tangentRate * (speedRate / speed) - tangent * (speedSecond / speed)) / speed
                - curvature * (2 * speedRate / speed)) / speed;
            double d2Scale = d2.Length;
            double d3Scale = d3.Length;
            if (d2Scale > 0)
            {
                Vector3d cross = Vector3d.Cross(tangent, d2 / d2Scale);
                double square = Vector3d.Dot(cross, cross);
                if (square > 0)
                {
                    torsion = d3Scale == 0 ? 0 : Vector3d.Dot(cross, d3 / d3Scale) / square * (d3Scale / speed / d2Scale);
                    if (!double.IsFinite(torsion.Value)) { throw new ArithmeticException("Torsion exceeds finite range."); }
                }
            }
        }
        return new(jet.Position, tangent, speed, curvature, curvatureRate, torsion);
    }
}
