using System;
using System.Collections.Generic;
using QuantumDynamix.Geometry;
using QuantumDynamix.Math;

namespace QuantumDynamix.App;

internal enum WireframeElement { Heartline, Spine, Rail, Support, ControlPoint, RegionBoundary, Forward, Left, Up, OperationalSegment }

internal readonly record struct WireframeSelection(WireframeElement Element, string Id, string Label,
    Guid? RegionId = null, Guid? SegmentId = null)
{
    internal static WireframeSelection ForBoundary(WireframeBoundary boundary)
    {
        string edge = boundary.IsStart ? "start" : "end";
        return new(WireframeElement.RegionBoundary, edge,
            $"{boundary.RegionName} {edge} · {boundary.HeartlineStation:G5} m", boundary.RegionId);
    }
}
internal readonly record struct ProjectedLine(ProjectedPoint Start, ProjectedPoint End, WireframeSelection Selection,
    WireframeElement Appearance);
internal readonly record struct ProjectedMarker(ProjectedPoint Point, WireframeSelection Selection);

/// <summary>Projects the same world-space primitives for drawing and hit testing.</summary>
internal sealed class ProjectedWireframe
{
    private readonly CameraProjection projection;
    private readonly List<ProjectedLine> lines = new();
    private readonly List<ProjectedMarker> markers = new();

    internal IReadOnlyList<ProjectedLine> Lines => lines;
    internal IReadOnlyList<ProjectedMarker> Markers => markers;

    internal ProjectedWireframe(TrackWireframe scene, CameraProjection projection, double axisLength,
        IReadOnlyList<WireframePath>? operations = null)
    {
        this.projection = projection;
        foreach (WireframePath path in scene.Paths) { AddPath(path); }
        foreach (WireframePath path in operations ?? []) { AddPath(path); }

        foreach (WireframeControlPoint control in scene.ControlPoints)
        {
            AddMarker(control.Position, new(WireframeElement.ControlPoint, control.Index.ToString(), $"Control point {control.Index + 1}"));
        }
        foreach (WireframeBoundary boundary in scene.Boundaries)
        {
            WireframeSelection selection = WireframeSelection.ForBoundary(boundary);
            AddMarker(boundary.Position, selection);
            if (boundary.Orientation is CurveFrame frame && axisLength > 0)
            {
                // The axis length is a display scale. Frame directions come from the evaluated track.
                AddLine(boundary.Position, boundary.Position + frame.Forward * axisLength, selection, WireframeElement.Forward);
                AddLine(boundary.Position, boundary.Position + frame.Left * axisLength, selection, WireframeElement.Left);
                AddLine(boundary.Position, boundary.Position + frame.Up * axisLength, selection, WireframeElement.Up);
            }
        }
    }

    internal static WireframeElement ElementFor(WireframePathKind kind) => kind switch
    {
        WireframePathKind.Rail => WireframeElement.Rail,
        WireframePathKind.Spine => WireframeElement.Spine,
        WireframePathKind.Support => WireframeElement.Support,
        WireframePathKind.OperationalSegment => WireframeElement.OperationalSegment,
        _ => WireframeElement.Heartline
    };

    internal WireframeSelection? Pick(double x, double y, double radius, bool operationsOnly = false)
    {
        double bestDistance = radius * radius;
        double bestDepth = double.PositiveInfinity;
        WireframeSelection? result = null;

        // Give visible point markers priority at their centers, even where a path passes through them.
        foreach (ProjectedMarker marker in markers)
        {
            if (operationsOnly) { break; }
            Consider(marker.Point.X, marker.Point.Y, marker.Point.Depth, marker.Selection);
        }
        if (result is not null)
        {
            return result;
        }

        foreach (ProjectedLine line in lines)
        {
            if (operationsOnly != line.Selection.SegmentId.HasValue) { continue; }
            double dx = line.End.X - line.Start.X;
            double dy = line.End.Y - line.Start.Y;
            double lengthSquared = dx * dx + dy * dy;
            double fraction = lengthSquared == 0
                ? (line.Start.Depth <= line.End.Depth ? 0 : 1)
                : double.Clamp(((x - line.Start.X) * dx + (y - line.Start.Y) * dy) / lengthSquared, 0, 1);
            Consider(line.Start.X + dx * fraction, line.Start.Y + dy * fraction,
                projection.DepthAt(line.Start, line.End, fraction), line.Selection);
        }
        return result;

        void Consider(double px, double py, double depth, WireframeSelection target)
        {
            double dx = px - x;
            double dy = py - y;
            double distance = dx * dx + dy * dy;
            const double tie = 1e-8;
            if (distance <= radius * radius &&
                (distance < bestDistance - tie || (double.Abs(distance - bestDistance) <= tie && depth < bestDepth)))
            {
                bestDistance = distance;
                bestDepth = depth;
                result = target;
            }
        }
    }

    private void AddPath(WireframePath path)
    {
        WireframeElement element = ElementFor(path.Kind);
        string label = element is WireframeElement.Rail or WireframeElement.Support ? $"{element} {path.Id}" : path.Id;
        var selection = new WireframeSelection(element, path.Id, label, path.RegionId, path.SegmentId);
        for (int i = 1; i < path.Vertices.Count; i++)
        { AddLine(path.Vertices[i - 1].Position, path.Vertices[i].Position, selection, element); }
    }

    private void AddLine(Vector3d start, Vector3d end, WireframeSelection selection, WireframeElement appearance)
    {
        if (projection.TryProjectSegment(start, end, out ProjectedPoint a, out ProjectedPoint b))
        {
            lines.Add(new(a, b, selection, appearance));
        }
    }

    private void AddMarker(Vector3d world, WireframeSelection selection)
    {
        if (projection.TryProject(world, out ProjectedPoint point))
        {
            markers.Add(new(point, selection));
        }
    }
}
