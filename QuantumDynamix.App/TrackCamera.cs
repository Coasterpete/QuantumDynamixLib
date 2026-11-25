using System.Collections.Generic;
using QuantumDynamix.Math;
using SystemMath = System.Math;

namespace QuantumDynamix.App;

internal enum TrackView { Front, Back, Left, Right, Top, Bottom, Isometric, Perspective }

internal sealed class TrackCamera
{
    private static readonly double HalfFieldOfViewTangent = SystemMath.Tan(SystemMath.PI / 8);
    private Vector3d target;
    private Vector3d forward;
    private Vector3d right;
    private Vector3d up;
    private bool perspective;
    private double halfHeight = 1;
    private double minimumHeight = 1e-6;
    private double maximumHeight = 1e6;

    internal TrackCamera() => SetView(TrackView.Perspective);

    internal void SetView(TrackView view)
    {
        // World +Z is up. Front faces +Y, and the horizontal plan views keep +Y at the top.
        (Vector3d direction, Vector3d screenUp) = view switch
        {
            TrackView.Front => (Vector3d.UnitY, Vector3d.UnitZ),
            TrackView.Back => (-Vector3d.UnitY, Vector3d.UnitZ),
            TrackView.Left => (Vector3d.UnitX, Vector3d.UnitZ),
            TrackView.Right => (-Vector3d.UnitX, Vector3d.UnitZ),
            TrackView.Top => (-Vector3d.UnitZ, Vector3d.UnitY),
            TrackView.Bottom => (Vector3d.UnitZ, Vector3d.UnitY),
            _ => (new(-1, 1, -1), Vector3d.UnitZ)
        };
        Orient(direction, screenUp, view == TrackView.Perspective);
    }

    private void Orient(Vector3d direction, Vector3d screenUp, bool perspective)
    {
        forward = direction.Normalized();
        right = Vector3d.Cross(forward, screenUp).Normalized();
        up = Vector3d.Cross(right, forward);
        this.perspective = perspective;
    }

    internal void Frame(IReadOnlyList<Vector3d> points, double width, double height)
    {
        if (points.Count == 0 || width <= 0 || height <= 0)
        {
            return;
        }

        // Use a local origin so camera fitting works with translated survey coordinates.
        Vector3d origin = points[0];
        Vector3d lower = default;
        Vector3d upper = default;
        foreach (Vector3d point in points)
        {
            Vector3d local = ToView(point - origin);
            lower = new(SystemMath.Min(lower.X, local.X), SystemMath.Min(lower.Y, local.Y), SystemMath.Min(lower.Z, local.Z));
            upper = new(SystemMath.Max(upper.X, local.X), SystemMath.Max(upper.Y, local.Y), SystemMath.Max(upper.Z, local.Z));
        }
        Vector3d middle = lower * .5 + upper * .5;
        target = origin + (right * middle.X + up * middle.Y + forward * middle.Z);

        double aspect = width / height;
        const double padding = 1.16;
        double depth = upper.Z - middle.Z;
        double fit = perspective ? depth * HalfFieldOfViewTangent * 1.05 : depth * .03;
        foreach (Vector3d point in points)
        {
            Vector3d local = ToView(point - origin) - middle;
            double extent = padding * SystemMath.Max(SystemMath.Abs(local.Y), SystemMath.Abs(local.X) / aspect);
            // In perspective the point's distance from the eye changes with its own view-space Z.
            fit = SystemMath.Max(fit, perspective ? extent - local.Z * HalfFieldOfViewTangent : extent);
        }
        halfHeight = fit > 0 ? fit : 1;
        minimumHeight = SystemMath.Max(double.Epsilon, halfHeight * 1e-4);
        maximumHeight = SystemMath.Min(double.MaxValue, halfHeight * 1e4);
    }

    internal void Pan(double dx, double dy, double viewportHeight)
    {
        double metresPerDip = 2 * halfHeight / viewportHeight;
        target += right * (-dx * metresPerDip) + up * (dy * metresPerDip);
    }

    internal void Zoom(double factor) => halfHeight = SystemMath.Clamp(halfHeight * factor, minimumHeight, maximumHeight);

    internal void Orbit(double yawDelta, double pitchDelta)
    {
        double yaw = SystemMath.Atan2(right.X, -right.Y) + yawDelta;
        double pitch = SystemMath.Asin(SystemMath.Clamp(forward.Z, -1, 1)) + pitchDelta;
        pitch = SystemMath.Clamp(pitch, -SystemMath.PI / 2 + .01, SystemMath.PI / 2 - .01);
        double horizontal = SystemMath.Cos(pitch);
        Orient(new(horizontal * SystemMath.Cos(yaw), horizontal * SystemMath.Sin(yaw), SystemMath.Sin(pitch)), Vector3d.UnitZ,
            perspective: true);
    }

    internal CameraProjection ProjectTo(double width, double height) =>
        new(target, right, up, forward, perspective, halfHeight, HalfFieldOfViewTangent, width, height);

    private Vector3d ToView(Vector3d relative) => new(Vector3d.Dot(relative, right), Vector3d.Dot(relative, up),
        Vector3d.Dot(relative, forward));
}

internal readonly record struct ProjectedPoint(double X, double Y, double Depth);

/// <summary>One camera snapshot shared by drawing and picking. Screen coordinates are in DIPs.</summary>
internal readonly struct CameraProjection(Vector3d target, Vector3d right, Vector3d up, Vector3d forward,
    bool perspective, double halfHeight, double halfFovTangent, double width, double height)
{
    private double Distance => halfHeight / halfFovTangent;
    private double Near => double.Max(double.Epsilon, Distance * 1e-5);

    internal bool TryProject(Vector3d world, out ProjectedPoint point)
    {
        point = default;
        if (width <= 0 || height <= 0)
        {
            return false;
        }

        Vector3d local = ToView(world);
        double y = HalfHeightAt(local.Z);
        double x = y * (width / height);
        if ((perspective && local.Z < Near) || double.Abs(local.X) > x || double.Abs(local.Y) > y)
        {
            return false;
        }
        point = ToScreen(local);
        return true;
    }

    internal bool TryProjectSegment(Vector3d start, Vector3d end, out ProjectedPoint a, out ProjectedPoint b)
    {
        a = b = default;
        if (width <= 0 || height <= 0)
        {
            return false;
        }

        Vector3d first = ToView(start);
        Vector3d last = ToView(end);
        double enter = 0, exit = 1;
        double aspect = width / height;
        double ay = HalfHeightAt(first.Z), by = HalfHeightAt(last.Z);
        double ax = ay * aspect, bx = by * aspect;

        // The signed distance to each frustum plane is linear along the segment.
        if ((perspective && !Clip(first.Z - Near, last.Z - Near, ref enter, ref exit)) ||
            !Clip(ax - first.X, bx - last.X, ref enter, ref exit) || !Clip(ax + first.X, bx + last.X, ref enter, ref exit) ||
            !Clip(ay - first.Y, by - last.Y, ref enter, ref exit) || !Clip(ay + first.Y, by + last.Y, ref enter, ref exit))
        {
            return false;
        }

        Vector3d span = last - first;
        Vector3d clippedFirst = first + span * enter;
        Vector3d clippedLast = last - span * (1 - exit);
        a = ToScreen(clippedFirst);
        b = ToScreen(clippedLast);
        return true;
    }

    internal double DepthAt(ProjectedPoint start, ProjectedPoint end, double fraction) => perspective
        ? 1 / ((1 - fraction) / start.Depth + fraction / end.Depth)
        : start.Depth + fraction * (end.Depth - start.Depth);

    private double HalfHeightAt(double depth) => perspective ? depth * halfFovTangent : halfHeight;

    private Vector3d ToView(Vector3d point)
    {
        Vector3d relative = point - target;
        return new(Vector3d.Dot(relative, right), Vector3d.Dot(relative, up), Vector3d.Dot(relative, forward) + Distance);
    }

    private ProjectedPoint ToScreen(Vector3d point)
    {
        double depth = perspective ? double.Max(Near, point.Z) : point.Z;
        double scale = height / (2 * HalfHeightAt(depth));
        return new(width / 2 + point.X * scale, height / 2 - point.Y * scale, depth);
    }

    private static bool Clip(double start, double end, ref double enter, ref double exit)
    {
        if (start < 0 && end < 0)
        {
            return false;
        }
        if (start < 0) { enter = double.Max(enter, start / (start - end)); }
        if (end < 0) { exit = double.Min(exit, start / (start - end)); }
        return enter <= exit;
    }
}
