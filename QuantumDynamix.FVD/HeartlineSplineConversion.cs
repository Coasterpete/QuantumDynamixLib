using System;
using System.Collections.Generic;
using System.Linq;
using QuantumDynamix.Geometry;
using QuantumDynamix.Math;
using QuantumDynamix.Math.Functions;
using QuantumDynamix.Math.Numerics;
using QuantumDynamix.Splines;
using QuantumDynamix.Splines.Curves;
using QuantumDynamix.Splines.Fitting;
using QuantumDynamix.Track;

namespace QuantumDynamix.FVD;

internal sealed class HeartlineSplineConversion(ReconstructedHeartline source, HeartlineFitSettings settings)
{
    private const int Degree = 5;
    private const int EndpointMultiplicity = Degree + 1;
    private readonly SortedDictionary<double, int> knots = new();
    private int passes;
    private int verificationSamples;
    private FitDiagnostics? solver;
    private HeartlineFitVerification? completedVerification;
    private string stage = "Spline fit setup";

    internal HeartlineFitResult Fit()
    {
        try { return FitWithinLimits(); }
        catch (HeartlineReconstructionException ex)
        { return Result(null, ex.Failure with { Message = $"{stage}. {ex.Failure.Message}" }); }
        catch (NumericalConvergenceException ex)
        {
            string message = $"{stage}. {ex.Message}";
            if (ex.ParameterInterval.HasValue) { message += " Spline parameters are in local source metres."; }
            return Result(null, Failure(HeartlineFailureCode.NumericalBudgetExceeded, message,
                ex.ParameterInterval, measurement: ex.ResidualBudget is { } budget
                    ? (budget.Quantity, budget.Unit, ex.Residual, budget.Limit) : null));
        }
        catch (Exception ex) when (ex is ArithmeticException or ArgumentException or InvalidOperationException)
        { return Result(null, Failure(HeartlineFailureCode.EvaluationFailed, $"{stage}. {ex.Message}")); }
    }

    private HeartlineFitResult FitWithinLimits()
    {
        var steps = new HashSet<double>(source.CurvatureSteps);
        IReadOnlyList<double> sourceBoundaries = source.BoundaryStationsMetres;
        knots.Add(0, EndpointMultiplicity);
        knots.Add(source.LengthMetres, EndpointMultiplicity);
        if (!CanAdd(0))
        { return Result(null, Failure(HeartlineFailureCode.NumericalBudgetExceeded, "A degree-five spline requires at least six control points.")); }
        foreach (double boundary in sourceBoundaries.Skip(1).SkipLast(1))
        {
            int multiplicity = steps.Contains(boundary) ? Degree - 1 : Degree - settings.MinimumContinuousKnotOrder;
            if (!CanAdd(multiplicity))
            { return Result(null, Failure(HeartlineFailureCode.NumericalBudgetExceeded, "Required knots exceed the control-point limit.")); }
            knots.Add(boundary, multiplicity);
        }

        ReconstructedHeartlineSample start = source.Evaluate(0, EvaluationSide.Right);
        Vector3d origin = start.UnbankedFrame.Position;
        var constraints = new List<CurveFitConstraint>();
        foreach (double boundary in sourceBoundaries)
        {
            bool endpoint = boundary == 0 || boundary == source.LengthMetres;
            bool step = steps.Contains(boundary);
            EvaluationSide limit = boundary == source.LengthMetres ? EvaluationSide.Left : EvaluationSide.Right;
            ReconstructedHeartlineSample sample = source.Evaluate(boundary, limit);
            constraints.Add(new(boundary, 0, sample.UnbankedFrame.Position - origin));
            constraints.Add(new(boundary, 1, sample.UnbankedFrame.Forward));
            if (endpoint || step)
            { constraints.Add(new(boundary, 2, sample.CurvatureVectorPerMetre!.Value, limit)); }
            if (endpoint) { continue; }

            Vector3d leftCurvature = source.Evaluate(boundary, EvaluationSide.Left).CurvatureVectorPerMetre!.Value;
            if (step) { constraints.Add(new(boundary, 2, leftCurvature, EvaluationSide.Left)); }
            else
            {
                // At a C2 or smoother knot, one fitted curvature must serve both source limits.
                double minimumError = (leftCurvature - sample.CurvatureVectorPerMetre!.Value).Length / 2;
                if (minimumError > settings.CurvatureTolerancePerMetre)
                {
                    return Result(null, Failure(HeartlineFailureCode.FitToleranceExceeded,
                        $"A C{settings.MinimumContinuousKnotOrder} knot cannot match both source curvature limits within tolerance.",
                        (boundary, boundary), measurement: ("Minimum sided curvature error", "1/m", minimumError,
                            settings.CurvatureTolerancePerMetre)));
                }
            }
        }

        while (ControlCount < constraints.Count)
        {
            double[] boundaries = knots.Keys.ToArray();
            var spans = Enumerable.Range(0, boundaries.Length - 1).OrderByDescending(i => boundaries[i + 1] - boundaries[i]);
            if (!Subdivide(boundaries, spans.Take(1)))
            { return Result(null, Failure(HeartlineFailureCode.NumericalBudgetExceeded, "The control-point or station resolution limit prevents fitting the required constraints.")); }
        }

        HashSet<double> requiredKnots = sourceBoundaries.ToHashSet();
        HashSet<double> paddingKnots = knots.Keys.Where(knot => !requiredKnots.Contains(knot)).ToHashSet();
        Queue<(double Start, double End)>? retrySpans = null;
        HeartlineFailure? lastFailure = null;
        for (int pass = 0; pass < settings.MaximumRefinementPasses; pass++)
        {
            passes++;
            solver = null;
            completedVerification = null;
            double[] boundaries = knots.Keys.ToArray();
            stage = "Spline solve";
            var definition = new SplineDefinition(Degree, knots.SelectMany(knot => Enumerable.Repeat(knot.Key, knot.Value)));
            CurveFitResult fit = ConstrainedSplineFitter.FitCurve(definition, FitSamples(boundaries, origin), constraints);
            solver = fit.Diagnostics;
            if (fit.Curve is null)
            {
                lastFailure = Failure(HeartlineFailureCode.SplineSolveFailed, $"{stage}. {fit.Diagnostics.Message}",
                    (definition.Minimum, definition.Maximum));
                if (passes == settings.MaximumRefinementPasses || settings.MinimumContinuousKnotOrder != Degree - 1
                    || fit.Diagnostics.Status is not (FitStatus.Infeasible or FitStatus.RankDeficient)) { break; }

                retrySpans ??= new(sourceBoundaries.Zip(sourceBoundaries.Skip(1), (a, b) => (Start: a, End: b))
                    .OrderBy(span => span.End - span.Start));
                if (RetryClusteredConstraints(boundaries, paddingKnots, retrySpans)) { continue; }
                break;
            }

            // Use the actual exported controls for the regularity and FVD checks, including translation roundoff.
            stage = "Fitted spline regularity";
            var curve = new SplineCurve3d(definition, fit.Curve.ControlPoints.Select(point => origin + point));
            int[] irregular = IrregularSpans(curve, boundaries).ToArray();
            IReadOnlyCollection<int> subdivisions;
            if (irregular.Length != 0)
            {
                int first = irregular[0];
                lastFailure = Failure(HeartlineFailureCode.SplineRegularityUnresolved,
                    "A fitted span cannot establish the required positive parameter-speed margin.", (boundaries[first], boundaries[first + 1]));
                subdivisions = irregular;
            }
            else
            {
                stage = "Fitted arc-length map";
                var map = new ArcLengthMap(curve, settings.ArcLengthSettings);
                stage = "Fitted frame transport";
                var track = new HeartlineTrack(map, start.UnbankedFrame.Up, TrackOffsetProfile.Constant(Vector3d.Zero, map.Length),
                    HeartlineFitMapping.Bank(source, map), null, settings.FrameTransportToleranceRadians);
                stage = "Forward FVD verification";
                var verification = new HeartlineFitVerification(source, settings, track, boundaries);
                bool finished = verification.Verify(ref verificationSamples);
                if (!finished)
                { return Result(null, verification.IncompleteVerification); }

                completedVerification = verification;
                if (verification.WorstFailure is null) { return Result(track, null); }
                lastFailure = verification.WorstFailure;
                subdivisions = verification.FailingSpans;
            }

            if (passes == settings.MaximumRefinementPasses || !Subdivide(boundaries, subdivisions)) { break; }
        }
        return Result(null, lastFailure);
    }

    private int ControlCount => knots.Values.Sum() - EndpointMultiplicity;
    private bool CanAdd(int count) => count <= settings.MaximumControlPoints - ControlCount;

    private bool RetryClusteredConstraints(double[] boundaries, HashSet<double> paddingKnots,
        Queue<(double Start, double End)> spans)
    {
        // Try short intervals between constraints first. A knot in a distant span gives the cluster little local freedom.
        while (spans.TryDequeue(out var span))
        {
            double middle = span.Start + (span.End - span.Start) / 2;
            if (!(middle > span.Start && middle < span.End) || knots.ContainsKey(middle)) { continue; }

            // Move only a knot placed to meet the initial control count. Required and previously refined knots stay put.
            double donor = Enumerable.Range(1, boundaries.Length - 2)
                .Where(i => paddingKnots.Contains(boundaries[i]) && (boundaries[i] < span.Start || boundaries[i] > span.End))
                .OrderByDescending(i => boundaries[i + 1] - boundaries[i - 1])
                .Select(i => boundaries[i]).FirstOrDefault(double.NaN);
            if (double.IsFinite(donor))
            {
                knots.Remove(donor);
                paddingKnots.Remove(donor);
            }
            else if (!CanAdd(1)) { continue; }

            knots.Add(middle, 1);
            return true;
        }
        return false;
    }

    private bool Subdivide(double[] boundaries, IEnumerable<int> spans)
    {
        bool added = false;
        foreach (int span in spans)
        {
            if (!CanAdd(1)) { break; }
            double a = boundaries[span];
            double b = boundaries[span + 1];
            double mid = a + (b - a) / 2;
            if (mid > a && mid < b) { added |= knots.TryAdd(mid, 1); }
        }
        return added;
    }

    private IEnumerable<CurveFitSample> FitSamples(double[] boundaries, Vector3d origin)
    {
        for (int span = 0; span + 1 < boundaries.Length; span++)
        {
            double a = boundaries[span];
            double b = boundaries[span + 1];
            for (int i = 1; i <= 4; i++)
            {
                double station = a + (b - a) * (i / 5.0);
                if (station > a && station < b)
                { yield return new(station, source.Evaluate(station).UnbankedFrame.Position - origin); }
            }
        }
    }

    private IEnumerable<int> IrregularSpans(SplineCurve3d curve, double[] boundaries)
    {
        int knotIndex = Degree;
        for (int span = 0; span + 1 < boundaries.Length; span++)
        {
            double a = boundaries[span];
            double b = boundaries[span + 1];
            double middle = a + (b - a) / 2;
            Vector3d derivative = curve.Evaluate(middle, EvaluationSide.Right).FirstDerivative;
            // A slightly shortened reference direction has norm at most one despite normalization roundoff.
            if (derivative == Vector3d.Zero
                || !HasPositiveProjection(curve, knotIndex, derivative.Normalized() * (1 - 8 * (System.Math.BitIncrement(1) - 1))))
            { yield return span; }
            knotIndex += knots[b];
        }
    }

    private bool HasPositiveProjection(SplineCurve3d curve, int span, Vector3d direction)
    {
        IReadOnlyList<double> parameters = curve.Definition.Knots;
        for (int i = span - Degree; i < span; i++)
        {
            Vector3d a = curve.ControlPoints[i];
            Vector3d b = curve.ControlPoints[i + 1];
            double x = LowerProduct(direction.X, b.X - a.X);
            double y = LowerProduct(direction.Y, b.Y - a.Y);
            double z = LowerProduct(direction.Z, b.Z - a.Z);
            double projected = System.Math.BitDecrement(System.Math.BitDecrement(x + y) + z);
            double width = System.Math.BitIncrement(parameters[i + Degree + 1] - parameters[i + 1]);
            double lower = System.Math.BitDecrement(System.Math.BitDecrement(Degree * projected) / width);
            if (!(projected > 0 && double.IsFinite(lower) && lower >= settings.MinimumParameterSpeed)) { return false; }
        }
        // The polynomial derivative basis is nonnegative and sums to one across the span.
        return true;

        static double LowerProduct(double multiplier, double difference) => System.Math.BitDecrement(multiplier *
            (multiplier < 0 ? System.Math.BitIncrement(difference) : System.Math.BitDecrement(difference)));
    }

    private HeartlineFitResult Result(HeartlineTrack? track, HeartlineFailure? failure) => new(source, track,
        completedVerification?.Diagnostics(solver, passes, verificationSamples) ?? new HeartlineFitDiagnostics
        { Solver = solver, RefinementPasses = passes, VerificationSamples = verificationSamples }, failure);

    internal static HeartlineFailure Failure(HeartlineFailureCode code, string message,
        (double, double)? interval = null, EvaluationSide? side = null, (string, string, double, double)? measurement = null) =>
        new(code, message, null, interval, null, null, side, measurement);
}
