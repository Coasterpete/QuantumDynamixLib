using System;
using System.Collections.Generic;
using System.Linq;
using QuantumDynamix.Geometry;
using QuantumDynamix.Math;
using QuantumDynamix.Math.Functions;
using QuantumDynamix.Splines.Curves;

namespace QuantumDynamix.Track;

public sealed partial class RailGeometry
{
    /// <summary>Bounded adaptive inspection, including both sides of known joins. Success is not a
    /// proof of global injectivity, clearance, or continuous-domain regularity. No input is changed.</summary>
    public IReadOnlyList<RailDiagnostic> Validate(bool closedCircuit = false)
    {
        var diagnostics = new HashSet<RailDiagnostic>();
        var cache = new Dictionary<(double, EvaluationSide), RailSample?[]>();
        int evaluations = 0;
        bool exhausted = false;
        foreach (RailDefinition rail in Arrangement.Rails)
        {
            foreach (SplineCurve3d boundary in rail.CrossSection.Boundaries)
            {
                foreach (double u in boundary.Definition.Breakpoints)
                {
                    Attempt(rail.Id, 0, () =>
                    {
                        double gap = (boundary.Evaluate(u, EvaluationSide.Left).Position - boundary.Evaluate(u, EvaluationSide.Right).Position).Length;
                        if (gap > Settings.PositionTolerance)
                        { diagnostics.Add(new(RailDiagnosticCode.ProfileDiscontinuity, rail.Id, 0, $"Cross-section position jump at profile parameter {u:R} (metres).", gap)); }
                    }, diagnostics);
                }
            }
        }
        foreach (double point in Breakpoints)
        {
            CompareAt(this, point, point, "Interior join", diagnostics);
        }
        if (closedCircuit) { CompareAt(this, HeartlineLength, 0, "Closed-circuit seam", diagnostics); }
        double[] partition = Breakpoints.Prepend(0).Append(HeartlineLength).ToArray();
        for (int span = 1; span < partition.Length && !exhausted; span++)
        {
            double start = partition[span - 1];
            double end = partition[span];
            for (int i = 0; i < Settings.SamplesPerSpan && !exhausted; i++)
            {
                double a = i == 0 ? start : start + (end - start) * i / Settings.SamplesPerSpan;
                double b = i + 1 == Settings.SamplesPerSpan ? end : start + (end - start) * (i + 1) / Settings.SamplesPerSpan;
                Inspect(a, b, 0);
            }
        }
        return Array.AsReadOnly(diagnostics.OrderBy(d => d.HeartlineStation).ThenBy(d => d.RailId, StringComparer.Ordinal).ThenBy(d => d.Code).ToArray());

        RailSample?[] Probe(double s, EvaluationSide side)
        {
            if (cache.TryGetValue((s, side), out RailSample?[]? cached)) { return cached; }
            var samples = new RailSample?[Arrangement.Rails.Count];
            if (exhausted) { return samples; }
            if (evaluations > Settings.MaximumEvaluations - samples.Length)
            {
                diagnostics.Add(new(RailDiagnosticCode.SamplingLimit, null, s, "Validation exhausted its rail-station evaluation budget.", evaluations));
                exhausted = true;
                return samples;
            }
            for (int index = 0; index < samples.Length; index++)
            {
                RailDefinition rail = Arrangement.Rails[index];
                evaluations++;
                Attempt(rail.Id, s, () =>
                {
                    RailSample sample = Evaluate(rail.Id, s, side);
                    samples[index] = sample;
                    InspectSample(rail, sample, diagnostics);
                }, diagnostics);
            }
            for (int i = 0; i < samples.Length; i++)
            {
                for (int j = i + 1; j < samples.Length; j++)
                {
                    if (samples[i] is RailSample a && samples[j] is RailSample b)
                    {
                        Attempt(a.RailId, s, () =>
                        {
                            double separation = Difference(a, b).Length;
                            if (separation <= Settings.PositionTolerance)
                            { diagnostics.Add(new(RailDiagnosticCode.CoincidentRails, a.RailId, s, "Rail center separation is at or below the position tolerance (metres).", separation, b.RailId)); }
                        }, diagnostics);
                    }
                }
            }
            foreach (RailSpacingDefinition spacing in Arrangement.Spacings)
            {
                Attempt(spacing.RailA, s, () =>
                {
                    RailSample? a = samples[FindRail(spacing.RailA)];
                    RailSample? b = samples[FindRail(spacing.RailB)];
                    if (a is null || b is null) { return; }
                    double measured = Measure(spacing, a.Value, b.Value);
                    double expected = spacing.Expected.Evaluate(s, side).Value;
                    double residual = System.Math.Abs(measured - expected);
                    if (residual > Settings.PositionTolerance || spacing.Measurement == RailSpacingMeasurement.Euclidean && expected < 0)
                    { diagnostics.Add(new(RailDiagnosticCode.SpacingMismatch, spacing.RailA, s, $"Spacing '{spacing.Id}' disagrees with its explicit expectation (metres).", residual, spacing.RailB)); }
                }, diagnostics);
            }
            cache.Add((s, side), samples);
            return samples;
        }

        void Inspect(double a, double b, int depth)
        {
            double middle = a + (b - a) / 2;
            if (!(middle > a && middle < b))
            {
                diagnostics.Add(new(RailDiagnosticCode.SamplingLimit, null, a, "Validation cannot resolve a distinct interval midpoint (heartline metres).", b - a));
                return;
            }
            RailSample?[] left = Probe(a, EvaluationSide.Right);
            RailSample?[] center = Probe(middle, EvaluationSide.Right);
            RailSample?[] right = Probe(b, EvaluationSide.Left);
            if (exhausted) { return; }
            bool refine = false;
            for (int i = 0; i < left.Length; i++)
            {
                if (left[i] is not RailSample x || center[i] is not RailSample y || right[i] is not RailSample z
                    || x.DerivativeOrder < 1 || y.DerivativeOrder < 1 || z.DerivativeOrder < 1) { continue; }
                Attempt(x.RailId, middle, () =>
                {
                    double vx = x.SpeedPerHeartlineMetre;
                    double vy = y.SpeedPerHeartlineMetre;
                    double vz = z.SpeedPerHeartlineMetre;
                    if (System.Math.Min(vx, System.Math.Min(vy, vz)) <= Settings.MinimumSpeedRatio) { return; }
                    refine |= System.Math.Abs(vy - (vx / 2 + vz / 2)) > Settings.SamplingRelativeTolerance * System.Math.Max(vx, System.Math.Max(vy, vz))
                        || Angle(x.Tangent, y.Tangent) > Settings.SamplingAngularTolerance || Angle(y.Tangent, z.Tangent) > Settings.SamplingAngularTolerance;
                }, diagnostics);
                Attempt(x.RailId, middle, () =>
                {
                    refine |= Angle(x.SectionFrame.Left, y.SectionFrame.Left) > Settings.SamplingAngularTolerance
                        || Angle(y.SectionFrame.Left, z.SectionFrame.Left) > Settings.SamplingAngularTolerance;
                    // Endpoint orientations alone alias complete turns. Analytic angular rates also drive inspection.
                    double angularRate = System.Math.Max(x.SectionAngularRate.Length,
                        System.Math.Max(y.SectionAngularRate.Length, z.SectionAngularRate.Length));
                    refine |= angularRate * ((b - a) / 2) > Settings.SamplingAngularTolerance;
                }, diagnostics);
            }
            if (!refine) { return; }
            if (depth >= Settings.MaximumDepth)
            {
                diagnostics.Add(new(RailDiagnosticCode.SamplingLimit, null, middle, "Validation reached its adaptive subdivision limit; this interval remains unresolved.", depth));
                return;
            }
            Inspect(a, middle, depth + 1);
            if (!exhausted) { Inspect(middle, b, depth + 1); }
        }
    }

    /// <summary>Checks end-to-start joins by stable rail ID. Profile boundaries correspond by index and
    /// normalized parameter; no stitching, reparameterization, frame correction, or topology is inferred.</summary>
    public IReadOnlyList<RailDiagnostic> ValidateJoin(RailGeometry next)
    {
        ArgumentNullException.ThrowIfNull(next);
        var diagnostics = new HashSet<RailDiagnostic>();
        CompareAt(next, HeartlineLength, 0, "Section join", diagnostics);
        return Array.AsReadOnly(diagnostics.ToArray());
    }

    private int FindRail(string id)
    {
        for (int i = 0; i < Arrangement.Rails.Count; i++) { if (Arrangement.Rails[i].Id == id) { return i; } }
        throw new KeyNotFoundException(id);
    }

    private void InspectSample(RailDefinition rail, RailSample sample, ISet<RailDiagnostic> diagnostics)
    {
        // Independent queries preserve all applicable diagnostics instead of stopping at the first failure.
        Attempt(rail.Id, sample.HeartlineStation, () => { _ = sample.CurvatureVector; }, diagnostics);
        Attempt(rail.Id, sample.HeartlineStation, () => { _ = sample.SectionFrame; }, diagnostics);
        Attempt(rail.Id, sample.HeartlineStation, () =>
        {
            CurveFrame frame = sample.SectionFrame;
            Vector3d omega = sample.SectionAngularRate;
            foreach (SplineCurve3d boundary in rail.CrossSection.Boundaries)
            {
                foreach (var point in ProfilePoints(boundary))
                {
                    CurveJet3d profile = boundary.Evaluate(point.Parameter, point.Side);
                    double profileSpeed = profile.FirstDerivative.Length;
                    if (!(profileSpeed > 0))
                    {
                        diagnostics.Add(new(RailDiagnosticCode.DegenerateProfile, rail.Id, sample.HeartlineStation,
                            $"Zero cross-section tangent at profile parameter {point.Parameter:R}."));
                        continue;
                    }
                    Vector3d offset = frame.Transform(profile.Position);
                    Vector3d stationRate = sample.Centerline.FirstDerivative + Vector3d.Cross(omega, offset);
                    Vector3d profileRate = frame.Transform(profile.FirstDerivative / profileSpeed);
                    double stationSpeed = stationRate.Length;
                    double sine = stationSpeed > 0 ? Vector3d.Cross(stationRate / stationSpeed, profileRate).Length : 0;
                    if (stationSpeed <= Settings.MinimumSpeedRatio || sine <= Settings.MinimumFrameProjection)
                    {
                        diagnostics.Add(new(RailDiagnosticCode.SingularSweep, rail.Id, sample.HeartlineStation,
                        $"Swept-section Jacobian is singular or below supplied floors at profile parameter {point.Parameter:R} (normalized area).", sine));
                    }
                }
            }
        }, diagnostics);
    }

    private IEnumerable<(double Parameter, EvaluationSide Side)> ProfilePoints(SplineCurve3d boundary)
    {
        double[] partition = boundary.Definition.Breakpoints.Prepend(boundary.Minimum).Append(boundary.Maximum).ToArray();
        for (int span = 1; span < partition.Length; span++)
        {
            double a = partition[span - 1];
            double b = partition[span];
            for (int i = 0; i <= Settings.SamplesPerSpan; i++)
            {
                yield return (i == Settings.SamplesPerSpan ? b : a + (b - a) * i / Settings.SamplesPerSpan,
                    i == Settings.SamplesPerSpan ? EvaluationSide.Left : EvaluationSide.Right);
            }
        }
    }

    private void CompareAt(RailGeometry next, double end, double start, string context, ISet<RailDiagnostic> diagnostics)
    {
        if (!Arrangement.Rails.Select(r => r.Id).Order(StringComparer.Ordinal)
            .SequenceEqual(next.Arrangement.Rails.Select(r => r.Id).Order(StringComparer.Ordinal)))
        { diagnostics.Add(new(RailDiagnosticCode.IncompatibleArrangement, null, end, $"{context}: rail IDs do not match.")); }
        foreach (RailDefinition rail in Arrangement.Rails)
        {
            if (!next.Arrangement.Rails.Any(r => r.Id == rail.Id)) { continue; }
            Attempt(rail.Id, end, () =>
            {
                RailSample a = Evaluate(rail.Id, end, EvaluationSide.Left);
                RailSample b = next.Evaluate(rail.Id, start, EvaluationSide.Right);
                void Check(RailDiagnosticCode code, double residual, double tolerance, string units)
                {
                    if (residual > tolerance)
                    { diagnostics.Add(new(code, rail.Id, end, $"{context}: {code} ({units}).", residual)); }
                }
                Check(RailDiagnosticCode.PositionDiscontinuity, Difference(a, b).Length, Settings.PositionTolerance, "metres");
                Attempt(rail.Id, end, () =>
                {
                    if (a.DerivativeOrder < 1 || b.DerivativeOrder < 1)
                    { throw new RailGeometryException(new(RailDiagnosticCode.UnavailableDerivative, rail.Id, end, $"{context}: first rail derivatives unavailable.")); }
                    Check(RailDiagnosticCode.StationDerivativeDiscontinuity, (a.Centerline.FirstDerivative - b.Centerline.FirstDerivative).Length,
                        Settings.StationDerivativeTolerance, "metres per heartline metre");
                    Check(RailDiagnosticCode.TangentDiscontinuity, Angle(a.Tangent, b.Tangent), Settings.AngularTolerance, "radians");
                }, diagnostics);
                Attempt(rail.Id, end, () =>
                {
                    if (a.DerivativeOrder < 2 || b.DerivativeOrder < 2)
                    { throw new RailGeometryException(new(RailDiagnosticCode.UnavailableDerivative, rail.Id, end, $"{context}: second rail derivatives unavailable.")); }
                    Check(RailDiagnosticCode.StationSecondDerivativeDiscontinuity,
                        (a.Centerline.SecondDerivative - b.Centerline.SecondDerivative).Length, Settings.CurvatureTolerance, "1/metres");
                    Check(RailDiagnosticCode.CurvatureDiscontinuity,
                        (a.CurvatureVector - b.CurvatureVector).Length, Settings.CurvatureTolerance, "1/metres");
                }, diagnostics);
                Attempt(rail.Id, end, () =>
                {
                    CurveFrame fa = a.SectionFrame;
                    CurveFrame fb = b.SectionFrame;
                    Check(RailDiagnosticCode.SectionOrientationDiscontinuity,
                        System.Math.Max(Angle(fa.Left, fb.Left), System.Math.Max(Angle(fa.Up, fb.Up), Angle(fa.Forward, fb.Forward))), Settings.AngularTolerance, "radians");
                    Check(RailDiagnosticCode.SectionRateDiscontinuity, (a.SectionAngularRate - b.SectionAngularRate).Length, Settings.CurvatureTolerance, "radians/metre");
                    CompareProfiles(rail, next.Arrangement.GetRail(rail.Id), a, b, context, diagnostics);
                }, diagnostics);
            }, diagnostics);
        }
    }

    private void CompareProfiles(RailDefinition a, RailDefinition b, RailSample sa, RailSample sb, string context, ISet<RailDiagnostic> diagnostics)
    {
        if (a.CrossSection.Boundaries.Count != b.CrossSection.Boundaries.Count)
        { diagnostics.Add(new(RailDiagnosticCode.ProfileDiscontinuity, a.Id, sa.HeartlineStation, $"{context}: cross-section boundary counts differ.")); return; }
        if (ReferenceEquals(a.CrossSection, b.CrossSection)) { return; }
        for (int i = 0; i < a.CrossSection.Boundaries.Count; i++)
        {
            SplineCurve3d ca = a.CrossSection.Boundaries[i];
            SplineCurve3d cb = b.CrossSection.Boundaries[i];
            var points = ProfilePoints(ca).Select(p => ((p.Parameter - ca.Minimum) / (ca.Maximum - ca.Minimum), p.Side))
                .Concat(ProfilePoints(cb).Select(p => ((p.Parameter - cb.Minimum) / (cb.Maximum - cb.Minimum), p.Side))).Distinct();
            foreach (var (fraction, side) in points)
            {
                Vector3d pa = ca.Evaluate(ca.Minimum + (ca.Maximum - ca.Minimum) * fraction, side).Position;
                Vector3d pb = cb.Evaluate(cb.Minimum + (cb.Maximum - cb.Minimum) * fraction, side).Position;
                double gap = (Difference(sa, sb) + sb.SectionFrame.Transform(pb) - sa.SectionFrame.Transform(pa)).Length;
                if (gap > Settings.PositionTolerance)
                { diagnostics.Add(new(RailDiagnosticCode.ProfileDiscontinuity, a.Id, sa.HeartlineStation, $"{context}: section boundary {i} differs at normalized parameter {fraction:R} (metres).", gap)); }
            }
        }
    }

    private static double Angle(Vector3d a, Vector3d b) => System.Math.Atan2(Vector3d.Cross(a, b).Length, Vector3d.Dot(a, b));

    private static void Attempt(string? id, double station, Action action, ISet<RailDiagnostic> diagnostics)
    {
        try { action(); }
        catch (RailGeometryException error) { diagnostics.Add(error.Diagnostic); }
        catch (Exception error) when (error is ArithmeticException or InvalidOperationException or ArgumentException)
        { diagnostics.Add(new(RailDiagnosticCode.NumericalFailure, id, station, error.Message)); }
    }
}
