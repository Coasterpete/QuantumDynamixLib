using System;
using System.Collections.Generic;
using System.Linq;
using QuantumDynamix.Geometry;
using QuantumDynamix.Math;
using QuantumDynamix.Math.Functions;
using QuantumDynamix.Math.Timelines;
using QuantumDynamix.Track;

namespace QuantumDynamix.FVD;

/// <summary>One numerical heartline sample in SI units. Motion retains the original authored station.</summary>
/// <param name="AuthoredCoordinate">Transverse profile coordinate, in authored metres or seconds.</param>
/// <param name="Motion">The M2 sample at the corresponding authored coordinate, retaining original authored station S.</param>
/// <param name="UnbankedFrame">Integrated position, tangent and unbanked normal-plane axes.</param>
/// <param name="BankedFrame">The same frame rotated around its forward axis by the authored bank.</param>
/// <param name="BankRadians">Unwrapped authored bank in radians.</param>
/// <param name="CurvatureVectorPerMetre">World curvature per metre, or null for an ambiguous automatic limit.</param>
/// <param name="LateralCoordinateAccelerationMetresPerSecondSquared">Coordinate acceleration used in reconstruction along banked left.</param>
/// <param name="VerticalCoordinateAccelerationMetresPerSecondSquared">Coordinate acceleration used in reconstruction along banked up.</param>
public readonly record struct ReconstructedHeartlineSample(double AuthoredCoordinate, LongitudinalMotionSample Motion,
    CurveFrame UnbankedFrame, CurveFrame BankedFrame, double BankRadians, Vector3d? CurvatureVectorPerMetre,
    double LateralCoordinateAccelerationMetresPerSecondSquared, double VerticalCoordinateAccelerationMetresPerSecondSquared)
{
    private readonly double? localStation;
    internal double StationCorrespondenceError { get; }
    internal double TimeCorrespondenceError { get; }
    internal double VelocityCorrespondenceError { get; }

    internal ReconstructedHeartlineSample(double station, ReconstructionTarget target, CurveFrame unbanked,
        Vector3d? curvature) : this(target.Coordinate, target.Motion, unbanked, unbanked.Banked(target.Bank),
        target.Bank, curvature, target.Left, target.Up)
    {
        localStation = station;
        StationCorrespondenceError = target.StationError;
        TimeCorrespondenceError = target.TimeError;
        VelocityCorrespondenceError = target.VelocityError;
    }

    /// <summary>Local numerical metres from the reconstructed start. Directly constructed samples assume a zero origin.</summary>
    public double StationMetres => localStation ?? Motion.HeartlineStationMetres;
}

/// <summary>Immutable progressive heartline in local arc-length metres, independent of its spline conversion.</summary>
/// <remarks>Sample queries can reevaluate custom functions retained from the reconstruction request.
/// See <see cref="HeartlineReconstructionRequest"/> for callback lifetime requirements.</remarks>
public sealed class ReconstructedHeartline
{
    private readonly HeartlineReconstructionInputs inputs;
    private readonly HeartlineReconstructionSettings settings;
    private readonly Vector3d origin;
    private readonly double integrationPositionError;
    private readonly HeartlineIntegratedPiece[] pieces;
    private readonly double[] pieceEnds;

    internal ReconstructedHeartline(HeartlineReconstructionInputs inputs, HeartlineReconstructionSettings settings,
        Vector3d origin, double integrationPositionError, HeartlineIntegratedPiece[] pieces,
        HeartlineReconstructionDiagnostics diagnostics)
    {
        this.inputs = inputs;
        this.settings = settings;
        this.origin = origin;
        this.integrationPositionError = integrationPositionError;
        this.pieces = pieces;
        pieceEnds = pieces.Select(piece => System.Math.Max(piece.Rotation.Start, piece.Rotation.End)).ToArray();
        BoundaryStationsMetres = Array.AsReadOnly(inputs.Boundaries);
        Diagnostics = diagnostics;
    }

    public double LengthMetres => inputs.Motion.Length;
    /// <summary>Original M2 station at local station zero, in metres.</summary>
    public double AuthoredStartStationMetres => inputs.Motion.OriginStation;
    internal double? GeometricHeadingAt(double authoredStation) =>
        inputs.GeometricHeadingAt(authoredStation, new ReconstructionWork(settings.MaximumWork));
    public ProfileDomain TransverseDomain => inputs.TransverseDomain;
    /// <summary>Resolved breakpoints and endpoints in local numerical metres.</summary>
    public IReadOnlyList<double> BoundaryStationsMetres { get; }
    public HeartlineReconstructionDiagnostics Diagnostics { get; }

    internal IReadOnlyList<double> CurvatureSteps => inputs.CurvatureSteps;
    internal ScalarJet BankAt(double station, EvaluationSide side) =>
        inputs.BankAt(station, side, new ReconstructionWork(settings.MaximumWork));

    /// <summary>Evaluates the numerical heartline at local station s = S - S0.</summary>
    /// <param name="sourceStationMetres">Local arc length in metres, measured from the reconstructed start.</param>
    /// <param name="side">Explicit limit at a join, or Automatic for outgoing values and common curvature.</param>
    public ReconstructedHeartlineSample Evaluate(double sourceStationMetres, EvaluationSide side = EvaluationSide.Automatic)
    {
        Check.Argument(sourceStationMetres, 0, LengthMetres, side);
        double s = sourceStationMetres;
        EvaluationSide limit = s == LengthMetres ? EvaluationSide.Left
            : s == 0 || side == EvaluationSide.Automatic ? EvaluationSide.Right : side;
        var work = new ReconstructionWork(settings.MaximumWork);
        int exact = Array.BinarySearch(pieceEnds, s);
        bool interiorJoin = exact >= 0 && exact < pieceEnds.Length - 1;
        int index = exact < 0 ? ~exact : interiorJoin && limit == EvaluationSide.Right ? exact + 1 : exact;
        HeartlineIntegratedPiece piece = pieces[index];
        ReconstructionTarget target = inputs.Target(s, limit, work);
        try
        {
            double error = 0;
            Vector3d offset;
            if (s == piece.Rotation.Start) { offset = piece.StartOffset; }
            else if (s == piece.Rotation.End) { offset = piece.EndOffset; }
            else
            {
                var (change, estimate) = piece.Rotation.Displacement(s, piece.QuadratureTolerance, work);
                offset = piece.StartOffset + change;
                error = estimate + ((offset - piece.StartOffset) - change).Length;
            }
            Vector3d position = origin + offset;
            error += ((position - origin) - offset).Length + integrationPositionError;
            if (!(error <= settings.PositionToleranceMetres))
            { throw work.Fail(HeartlineFailureCode.NumericalBudgetExceeded, "The numerical position query exceeds its tolerance.",
                (s, s), measurement: ("Position error estimate", "m", error, settings.PositionToleranceMetres)); }

            CurveFrame frame = piece.Rotation.Frame(s, position);
            if (inputs.RequiresOrientation)
            {
                target = inputs.ResolveOrientedTarget(target, piece.Rotation.Orientation(s), s, limit, work);
            }
            Vector3d rate = piece.Rotation.AngularRate(s);
            Vector3d? curvature = frame.Left * rate.Z - frame.Up * rate.Y;
            if (side == EvaluationSide.Automatic && interiorJoin)
            {
                Vector3d incoming = pieces[exact].Rotation.AngularRate(s);
                Vector3d outgoing = pieces[exact + 1].Rotation.AngularRate(s);
                if (incoming.Y != outgoing.Y || incoming.Z != outgoing.Z) { curvature = null; }
            }
            return new(s, target, frame, curvature);
        }
        catch (Exception ex) when (ex is ArithmeticException or ArgumentOutOfRangeException)
        {
            throw work.Fail(HeartlineFailureCode.NumericalRangeExceeded, "Numerical heartline evaluation exceeds finite range.",
                (s, s), side: limit, inner: ex);
        }
    }
}
