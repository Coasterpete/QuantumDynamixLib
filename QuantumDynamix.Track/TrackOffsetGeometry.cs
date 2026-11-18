using System;
using System.Collections.Generic;
using System.Linq;
using QuantumDynamix.Geometry;
using QuantumDynamix.Math;
using QuantumDynamix.Math.Functions;
using QuantumDynamix.Splines.Curves;

namespace QuantumDynamix.Track;

internal readonly record struct TrackOffsetSample(double Parameter, CurveFrame Frame, CurveJet3d Curve,
    Vector3d FrameAngularRate, Vector3d Displacement);

/// <summary>Shared analytic offset construction; all derivatives are with respect to heartline metres.</summary>
internal static class TrackOffsetGeometry
{
    internal static IEnumerable<IScalarFunction> Functions(TrackOffsetProfile offsets) => new[] { offsets.Forward, offsets.Left, offsets.Up };

    internal static TrackOffsetSample Evaluate(HeartlineTrack track, TrackOffsetProfile? additional,
        double station, EvaluationSide side)
    {
        double parameter = track.HeartlineStations.Invert(station).Parameter;
        CurveJet3d heart = track.Heartline.Evaluate(parameter, side);
        ScalarJet bank = track.Banking.Evaluate(station, side);
        CurveFrame frame = track.Frames.Evaluate(parameter).Banked(bank.Value);
        ScalarJet[] offsets = Functions(track.Offsets).Select(f => f.Evaluate(station, side)).ToArray();
        ScalarJet[]? extra = additional is null ? null : Functions(additional).Select(f => f.Evaluate(station, side)).ToArray();
        int order = System.Math.Min(2, System.Math.Min(heart.DerivativeOrder - 1, bank.DerivativeOrder));
        order = System.Math.Min(order, offsets.Min(j => j.DerivativeOrder));
        if (extra is not null) { order = System.Math.Min(order, extra.Min(j => j.DerivativeOrder)); }
        Vector3d Local(int derivative) => new(
            offsets[0].Derivative(derivative) + (extra?[0].Derivative(derivative) ?? 0),
            offsets[1].Derivative(derivative) + (extra?[1].Derivative(derivative) ?? 0),
            offsets[2].Derivative(derivative) + (extra?[2].Derivative(derivative) ?? 0));
        Vector3d displacement = frame.Transform(Local(0));
        Vector3d first = default;
        Vector3d second = default;
        Vector3d omega = default;
        if (order >= 1)
        {
            CurveGeometrySample geometry = CurveGeometry.Evaluate(track.Heartline, parameter, side);
            omega = Vector3d.Cross(geometry.Tangent, geometry.CurvatureVector) + geometry.Tangent * bank.FirstDerivative;
            Vector3d offsetRate = frame.Transform(Local(1));
            first = geometry.Tangent + Vector3d.Cross(omega, displacement) + offsetRate;
            if (order >= 2)
            {
                Vector3d omegaRate = Vector3d.Cross(geometry.Tangent, geometry.CurvatureStationDerivative!.Value)
                    + geometry.CurvatureVector * bank.FirstDerivative + geometry.Tangent * bank.SecondDerivative;
                second = geometry.CurvatureVector + Vector3d.Cross(omegaRate, displacement)
                    + Vector3d.Cross(omega, Vector3d.Cross(omega, displacement))
                    + Vector3d.Cross(omega, offsetRate) * 2 + frame.Transform(Local(2));
            }
        }
        return new(parameter, frame, new(heart.Position + displacement, first, second, default, System.Math.Max(0, order)), omega, displacement);
    }
}
