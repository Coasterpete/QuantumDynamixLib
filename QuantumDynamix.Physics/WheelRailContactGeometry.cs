using System;
using System.Collections.Generic;
using System.Linq;
using QuantumDynamix.Geometry;
using QuantumDynamix.Math;
using QuantumDynamix.Math.Functions;
using QuantumDynamix.Math.Numerics;
using QuantumDynamix.Track;

namespace QuantumDynamix.Physics;

/// <summary>Bounded local signed-gap queries against declared smooth rail patches. No global solid,
/// contact force, vehicle guidance, continuous collision detection, or geometry repair is inferred.</summary>
public static class WheelRailContactGeometry
{
    public static WheelRailGeometryResult Query(WheelRailContactSystem system, MultibodyState state,
        ContactGeometrySettings? settings = null)
    {
        ArgumentNullException.ThrowIfNull(system);
        system.Dynamics.ValidateState(state);
        settings ??= new();
        var samples = new List<WheelRailContactSample>();
        foreach (WheelRailPair pair in system.Pairs)
        {
            WheelDefinition wheel = system.Wheel(pair.WheelId);
            RailContactPatch patch = system.Patch(pair.PatchId);
            try
            {
                samples.Add(new Search(system.Rails, wheel, patch, state.Bodies[wheel.BodyIndex], settings).Run());
            }
            catch (ContactSolveException) { throw; }
            catch (Exception error) when (error is ArithmeticException or ArgumentException or InvalidOperationException)
            {
                throw new ContactSolveException(ContactSolveFailure.GeometryUnresolved, "Contact geometry",
                    "A surface/derivative query or numerical operation failed; separation is not established.",
                    wheel.Id, patch.Id, inner: error);
            }
        }
        // Overlapping bindings must not create two copies of the same physical contact.
        for (int i = 0; i < samples.Count; i++)
        {
            for (int j = 0; j < i; j++)
            {
                if (samples[i].WheelId == samples[j].WheelId
                    && (samples[i].RailPoint - samples[j].RailPoint).Length <= settings.PositionTolerance)
                {
                    throw new ContactSolveException(ContactSolveFailure.AmbiguousContact, "Contact geometry",
                        $"Two patch bindings identify coincident witnesses, including '{samples[j].PatchId}'.",
                        samples[i].WheelId, samples[i].PatchId);
                }
            }
        }
        return new(samples.ToArray());
    }

    private readonly record struct Probe(double S, double V, RailSurfaceSample Surface, Vector3d WheelLocalPoint,
        Vector3d WheelOutwardNormal, double Gap, double Gs, double Gv, double Residual);

    private sealed class Search
    {
        private readonly RailGeometry rails;
        private readonly WheelDefinition wheel;
        private readonly RailContactPatch patch;
        private readonly RigidBodyState body;
        private readonly ContactGeometrySettings settings;
        private readonly RigidPose3d pose;
        private readonly double stationRange;
        private readonly double profileRange;
        private int evaluations;

        internal Search(RailGeometry rails, WheelDefinition wheel, RailContactPatch patch,
            RigidBodyState body, ContactGeometrySettings settings)
        {
            this.rails = rails;
            this.wheel = wheel;
            this.patch = patch;
            this.body = body;
            this.settings = settings;
            pose = new(body.Pose.TransformPoint(wheel.LocalPose.Position),
                body.Pose.Orientation * wheel.LocalPose.Orientation);
            stationRange = patch.StationMaximum - patch.StationMinimum;
            profileRange = patch.ProfileMaximum - patch.ProfileMinimum;
        }

        internal WheelRailContactSample Run()
        {
            var candidates = new List<Probe>();
            var grid = new List<Probe>();
            int count = settings.SeedsPerAxis;
            for (int i = 0; i < count; i++)
            {
                for (int j = 0; j < count; j++)
                {
                    Probe seed = Evaluate((double)i / (count - 1), (double)j / (count - 1));
                    for (int k = 0; k < grid.Count; k++)
                    {
                        int otherI = k / count;
                        int otherJ = k % count;
                        if (System.Math.Abs(i - otherI) + System.Math.Abs(j - otherJ) > 1
                            && (seed.Surface.Position - grid[k].Surface.Position).Length <= settings.PositionTolerance)
                        { throw Fail(ContactSolveFailure.InvalidPatch, "Sampled patch intersection/coincidence is unsupported."); }
                    }
                    grid.Add(seed);
                }
            }
            foreach (Probe seed in grid)
            {
                Probe result = Minimize(seed);
                if (!candidates.Any(c => System.Math.Abs(c.S - result.S) < settings.BoundaryFraction * 10
                    && System.Math.Abs(c.V - result.V) < settings.BoundaryFraction * 10
                    && (c.Surface.Position - result.Surface.Position).Length <= 4 * settings.PositionTolerance))
                { candidates.Add(result); }
            }
            if (candidates.Count != 1)
            { throw Fail(ContactSolveFailure.AmbiguousContact, "Search seeds resolve distinct contact branches; split/restrict the declared patch.", candidates.Count); }
            Probe best = candidates[0];
            if (best.S <= settings.BoundaryFraction || best.S >= 1 - settings.BoundaryFraction
                || best.V <= settings.BoundaryFraction || best.V >= 1 - settings.BoundaryFraction)
            { throw Fail(ContactSolveFailure.UnsupportedFeature, "Closest branch reaches a patch edge/search boundary; edge/corner contact is unsupported."); }
            if (System.Math.Abs(best.WheelLocalPoint.X) >= wheel.Tread.HalfWidth - settings.PositionTolerance)
            { throw Fail(ContactSolveFailure.UnsupportedFeature, "Closest branch reaches or exceeds the finite tread edge."); }
            Vector3d normal = best.Surface.AreaVector.Normalized() * (int)patch.MaterialSide;
            double mismatch = (normal + best.WheelOutwardNormal).Length;
            if (mismatch > 10 * settings.TangencyTolerance)
            { throw Fail(ContactSolveFailure.MaterialSideMismatch, "The declared rail material normal does not oppose the wheel tread normal.", mismatch); }
            // Curvature of the reduced objective must resolve a strict local minimum. Flat/line
            // contacts and saddle points need a contact manifold, outside this first-stage model.
            var h = Hessian(best);
            if (!(h.A > 0 && h.D > 0 && h.A * h.D - h.B * h.B > 1e-12 * h.A * h.D))
            { throw Fail(ContactSolveFailure.AmbiguousContact, "The local contact minimum is not numerically isolated."); }
            Vector3d wheelPoint = pose.TransformPoint(best.WheelLocalPoint);
            Vector3d arm = wheelPoint - body.Pose.Position;
            Vector3d velocity = body.LinearVelocity + Vector3d.Cross(body.WorldAngularVelocity, arm);
            return new(wheel.Id, patch.Id, patch.RailId, wheel.BodyIndex,
                patch.StationMinimum + best.S * stationRange, patch.ProfileMinimum + best.V * profileRange,
                best.Surface.Position, wheelPoint, normal, best.Gap, velocity, Vector3d.Dot(normal, velocity),
                best.Residual, evaluations);
        }

        private Probe Evaluate(double s, double v)
        {
            if (++evaluations > settings.MaximumEvaluations)
            { throw Fail(ContactSolveFailure.GeometryUnresolved, "Search evaluation budget exhausted.", evaluations); }
            double station = patch.StationMinimum + s * stationRange;
            double profile = patch.ProfileMinimum + v * profileRange;
            RailSurfaceSample surface = rails.EvaluateSurface(patch.RailId, station, patch.BoundaryIndex, profile,
                s == 1 ? EvaluationSide.Left : EvaluationSide.Right, v == 1 ? EvaluationSide.Left : EvaluationSide.Right);
            Vector3d ds = surface.HeartlineStationDerivative;
            Vector3d dv = surface.ProfileDerivative;
            if (!(ds.Length > 0 && dv.Length > 0)
                || Vector3d.Cross(ds.Normalized(), dv.Normalized()).Length <= settings.TangencyTolerance)
            { throw Fail(ContactSolveFailure.InvalidPatch, "Degenerate or unresolved surface Jacobian."); }
            Vector3d point = pose.InverseTransformPoint(surface.Position);
            double radial = new Vector3d(0, point.Y, point.Z).Length;
            double radialGap = radial - wheel.Tread.Radius;
            double offset = radialGap + wheel.Tread.CrownRadius;
            double length = new Vector3d(point.X, offset, 0).Length;
            if (!(radial > 0 && offset > 0 && length > 0))
            { throw Fail(ContactSolveFailure.UnsupportedFeature, "Rail sample lies outside the regular outer-crown distance chart; no solid sign is inferred."); }
            double axialNormal = point.X / length;
            double radialNormal = offset / length;
            Vector3d outwardLocal = new(axialNormal, radialNormal * (point.Y / radial), radialNormal * (point.Z / radial));
            Vector3d outward = pose.TransformDirection(outwardLocal);
            double radius = wheel.Tread.Radius - (wheel.Tread.CrownRadius * axialNormal) * axialNormal / (1 + radialNormal);
            // Search seeds may lie far outside the trimmed axial width. The analytic meridian
            // extension supplies a search objective only; Run requires the final witness to lie
            // strictly inside the finite positive-radius tread before publishing any geometry.
            Vector3d witness = new(wheel.Tread.CrownRadius * axialNormal,
                radius * (point.Y / radial), radius * (point.Z / radial));
            double gs = Vector3d.Dot(outward, ds);
            double gv = Vector3d.Dot(outward, dv);
            double residual = System.Math.Max(System.Math.Abs(gs) / ds.Length, System.Math.Abs(gv) / dv.Length);
            double denominator = length + wheel.Tread.CrownRadius;
            if (!double.IsFinite(denominator)) { throw new ArithmeticException("Crown distance scale exceeds finite range."); }
            // Rationalized L-C avoids subtracting two almost equal crown-radius-sized numbers.
            double gap = (point.X / denominator) * point.X
                + (radialGap / denominator) * offset + (radialGap / denominator) * wheel.Tread.CrownRadius;
            return new(s, v, surface, witness, outward, gap,
                gs * stationRange, gv * profileRange, residual);
        }

        private Probe Minimize(Probe current)
        {
            for (int iteration = 0; iteration < settings.MaximumIterations; iteration++)
            {
                double gs = Projected(current.Gs, current.S);
                double gv = Projected(current.Gv, current.V);
                if (gs == 0 && gv == 0) { return current; }
                var h = Hessian(current);
                double determinant = h.A * h.D - h.B * h.B;
                double ds;
                double dv;
                if (h.A > 0 && h.D > 0 && determinant > 1e-12 * h.A * h.D)
                {
                    ds = (-h.D * gs + h.B * gv) / determinant;
                    dv = (h.B * gs - h.A * gv) / determinant;
                }
                else
                {
                    // A search direction only; this does not modify the contact law or inertia.
                    double scale = System.Math.Max(1, System.Math.Max(System.Math.Abs(h.A), System.Math.Abs(h.D)));
                    ds = -gs / scale;
                    dv = -gv / scale;
                }
                double locationCorrection = System.Math.Abs(ds) * stationRange * current.Surface.HeartlineStationDerivative.Length
                    + System.Math.Abs(dv) * profileRange * current.Surface.ProfileDerivative.Length;
                if (current.Residual <= settings.TangencyTolerance && locationCorrection <= settings.PositionTolerance)
                { return current; }
                double maximum = System.Math.Max(System.Math.Abs(ds), System.Math.Abs(dv));
                if (maximum > 0.5) { ds *= 0.5 / maximum; dv *= 0.5 / maximum; }
                bool improved = false;
                NumericalConvergenceException? failedTrial = null;
                double fraction = 1;
                for (int trial = 0; trial < 24; trial++, fraction /= 2)
                {
                    double s = System.Math.Clamp(current.S + fraction * ds, 0, 1);
                    double v = System.Math.Clamp(current.V + fraction * dv, 0, 1);
                    if (s == current.S && v == current.V) { continue; }
                    Probe candidate;
                    try { candidate = Evaluate(s, v); }
                    catch (NumericalConvergenceException error)
                    {
                        // A trial can land next to an unresolvable cached frame parameter.
                        // Backtrack within the same budget; never replace a failed query by a gap.
                        failedTrial = error;
                        continue;
                    }
                    double slope = current.Gs * (s - current.S) + current.Gv * (v - current.V);
                    if (candidate.Gap < current.Gap + 1e-4 * slope
                        || candidate.Residual < current.Residual * (1 - 1e-4 * fraction))
                    { current = candidate; improved = true; break; }
                }
                if (!improved)
                {
                    throw new ContactSolveException(ContactSolveFailure.GeometryUnresolved, "Contact geometry",
                        "Search line search stalled; separation is not established.", wheel.Id, patch.Id, iteration, current.Residual, failedTrial);
                }
            }
            throw Fail(ContactSolveFailure.GeometryUnresolved, "Search iteration budget exhausted.", current.Residual, settings.MaximumIterations);
        }

        private (double A, double B, double D) Hessian(Probe point)
        {
            var s = Difference(point, true);
            var v = Difference(point, false);
            return (s.S, (s.V + v.S) / 2, v.V);
        }

        private (double S, double V) Difference(Probe point, bool station)
        {
            NumericalConvergenceException? failedStencil = null;
            double step = settings.DifferenceStep;
            for (int attempt = 0; attempt < 8; attempt++, step /= 2)
            {
                double coordinate = station ? point.S : point.V;
                double low = System.Math.Max(0, coordinate - step);
                double high = System.Math.Min(1, coordinate + step);
                if (!(high > low)) { break; }
                try
                {
                    Probe a = Evaluate(station ? low : point.S, station ? point.V : low);
                    Probe b = Evaluate(station ? high : point.S, station ? point.V : high);
                    return ((b.Gs - a.Gs) / (high - low), (b.Gv - a.Gv) / (high - low));
                }
                catch (NumericalConvergenceException error) { failedStencil = error; }
            }
            throw new ContactSolveException(ContactSolveFailure.GeometryUnresolved, "Contact geometry",
                "No resolved finite-difference search stencil within the bounded attempts.", wheel.Id, patch.Id, inner: failedStencil);
        }

        private static double Projected(double gradient, double parameter) =>
            parameter == 0 && gradient > 0 || parameter == 1 && gradient < 0 ? 0 : gradient;

        private ContactSolveException Fail(ContactSolveFailure failure, string message, double residual = 0, int iterations = 0) =>
            new(failure, "Contact geometry", message, wheel.Id, patch.Id, iterations, residual);
    }
}
