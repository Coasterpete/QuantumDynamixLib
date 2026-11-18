using System;
using QuantumDynamix.Geometry;
using QuantumDynamix.Math;
using QuantumDynamix.Math.Functions;
using QuantumDynamix.Splines.Curves;

namespace QuantumDynamix.Track;

/// <summary>Geometric sample, never a vehicle state. Centerline derivatives use heartline metres;
/// curvature uses rail metres. Section forward follows the rail tangent, left is projected banked
/// heartline left, up=forward cross left, then positive section roll rotates left toward up.</summary>
public readonly record struct RailSample
{
    private readonly ScalarJet roll;
    private readonly Vector3d referenceAngularRate;
    private readonly RailValidationSettings settings;
    public string RailId { get; }
    public double HeartlineStation { get; }
    public double Parameter { get; }
    public CurveJet3d Centerline { get; }
    public CurveFrame BankedReferenceFrame { get; }
    internal Vector3d ReferenceDisplacement { get; }
    public Vector3d Position => Centerline.Position;
    public int DerivativeOrder => Centerline.DerivativeOrder;

    internal RailSample(string id, double station, TrackOffsetSample sample, ScalarJet roll, RailValidationSettings settings)
    {
        RailId = id;
        HeartlineStation = station;
        Parameter = sample.Parameter;
        Centerline = sample.Curve;
        BankedReferenceFrame = sample.Frame;
        ReferenceDisplacement = sample.Displacement;
        referenceAngularRate = sample.FrameAngularRate;
        this.roll = roll;
        this.settings = settings;
    }

    public double SpeedPerHeartlineMetre
    {
        get { RequireOrder(1); return Centerline.FirstDerivative.Length; }
    }

    public Vector3d Tangent
    {
        get
        {
            double speed = SpeedPerHeartlineMetre;
            if (!(speed > settings.MinimumSpeedRatio))
            { throw Failure(RailDiagnosticCode.SingularCenterline, "Rail speed ds_rail/ds_heartline is at or below the supplied regularity floor.", speed); }
            return Centerline.FirstDerivative / speed;
        }
    }

    public Vector3d CurvatureVector
    {
        get
        {
            RequireOrder(2);
            Vector3d tangent = Tangent;
            Vector3d scaled = Centerline.SecondDerivative / SpeedPerHeartlineMetre;
            return (scaled - tangent * Vector3d.Dot(tangent, scaled)) / SpeedPerHeartlineMetre;
        }
    }

    public double Curvature => CurvatureVector.Length;

    public CurveFrame SectionFrame
    {
        get
        {
            Vector3d tangent = Tangent;
            Vector3d projected = ProjectedLeft(tangent);
            Vector3d left = projected.Normalized();
            return new CurveFrame(Position, tangent, left, Vector3d.Cross(tangent, left)).Banked(roll.Value);
        }
    }

    /// <summary>World angular change per heartline metre, obtained by differentiating the projected-left
    /// construction analytically. Not angular velocity; multiply by a supplied station speed for that.</summary>
    public Vector3d SectionAngularRate
    {
        get
        {
            RequireOrder(2);
            if (roll.DerivativeOrder < 1)
            { throw Failure(RailDiagnosticCode.UnavailableDerivative, "Section angular rate requires the section-roll first derivative."); }
            Vector3d tangent = Tangent;
            Vector3d tangentRate = CurvatureVector * SpeedPerHeartlineMetre;
            Vector3d referenceLeft = BankedReferenceFrame.Left;
            Vector3d referenceLeftRate = Vector3d.Cross(referenceAngularRate, referenceLeft);
            double alignment = Vector3d.Dot(referenceLeft, tangent);
            Vector3d projected = ProjectedLeft(tangent);
            Vector3d left = projected.Normalized();
            Vector3d projectedRate = referenceLeftRate - tangentRate * alignment
                - tangent * (Vector3d.Dot(referenceLeftRate, tangent) + Vector3d.Dot(referenceLeft, tangentRate));
            Vector3d leftRate = (projectedRate - left * Vector3d.Dot(left, projectedRate)) / projected.Length;
            Vector3d up = Vector3d.Cross(tangent, left);
            Vector3d upRate = Vector3d.Cross(tangentRate, left) + Vector3d.Cross(tangent, leftRate);
            return (Vector3d.Cross(tangent, tangentRate) + Vector3d.Cross(left, leftRate) + Vector3d.Cross(up, upRate)) / 2
                + tangent * roll.FirstDerivative;
        }
    }

    public Vector3d TransformSectionPoint(Vector3d local)
    {
        if (local.X != 0) { throw new ArgumentException("A cross-section point must lie in X=0.", nameof(local)); }
        return Position + SectionFrame.Transform(local);
    }

    private Vector3d ProjectedLeft(Vector3d tangent)
    {
        // The vector triple product is the projected left direction. This form avoids cancellation
        // along the tangent and preserves perpendicularity close to the configured singularity floor.
        Vector3d projected = Vector3d.Cross(Vector3d.Cross(tangent, BankedReferenceFrame.Left), tangent);
        if (!(projected.Length > settings.MinimumFrameProjection))
        { throw Failure(RailDiagnosticCode.SingularSectionFrame, "Projected banked-left length is at or below the supplied frame floor (dimensionless).", projected.Length); }
        return projected;
    }

    private void RequireOrder(int order)
    {
        if (DerivativeOrder < order)
        { throw Failure(RailDiagnosticCode.UnavailableDerivative, $"Rail derivative order {order} is unavailable; request a sided limit where applicable."); }
    }

    private RailGeometryException Failure(RailDiagnosticCode code, string message, double? residual = null) =>
        new(new(code, RailId, HeartlineStation, message, residual));
}

/// <summary>Partials of X(s,v)=r(s)+Q(s)c(v). The area vector has no inferred material-side orientation.</summary>
public readonly record struct RailSurfaceSample(Vector3d Position, Vector3d HeartlineStationDerivative, Vector3d ProfileDerivative)
{
    public Vector3d AreaVector => Vector3d.Cross(HeartlineStationDerivative, ProfileDerivative);
}
