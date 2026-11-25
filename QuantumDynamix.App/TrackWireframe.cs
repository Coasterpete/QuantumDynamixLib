using System;
using System.Collections.Generic;
using System.Linq;
using QuantumDynamix.Geometry;
using QuantumDynamix.Math;
using QuantumDynamix.Math.Functions;
using QuantumDynamix.Math.Timelines;
using QuantumDynamix.Supports;
using QuantumDynamix.Track;

namespace QuantumDynamix.App;

internal enum WireframePathKind { Heartline, Spine, Rail, Support }

internal readonly record struct WireframeVertex(Vector3d Position, double? HeartlineStation);
internal sealed record WireframePath(WireframePathKind Kind, string Id, IReadOnlyList<WireframeVertex> Vertices, Guid? RegionId = null);
internal readonly record struct WireframeControlPoint(int Index, Vector3d Position);
internal readonly record struct WireframeBoundary(Guid RegionId, string RegionName, bool IsStart,
    double HeartlineStation, Vector3d Position, CurveFrame? Orientation);

// An existing support attachment and its authored foundation point, in world metres.
internal sealed record WireframeSupport(string Id, string RailId, string DatumName, double HeartlineStation,
    Vector3d FoundationPoint, EvaluationSide Side = EvaluationSide.Automatic);

internal sealed record TrackWireframe(IReadOnlyList<WireframePath> Paths,
    IReadOnlyList<WireframeControlPoint> ControlPoints, IReadOnlyList<WireframeBoundary> Boundaries,
    IReadOnlyList<string> Diagnostics)
{
    internal static TrackWireframe Empty(string message) => new([], [], [], [message]);
}

/// <summary>Samples evaluated engineering centerlines into independent display spans in world metres.</summary>
internal static class TrackWireframeSampler
{
    internal static TrackWireframe Sample(ArcLengthMap heartline, HeartlineTrack? spine, RailGeometry? rails,
        IReadOnlyList<AuthoredRegion> regions, IReadOnlyList<WireframeSupport> supports, int stationIntervals,
        string? bankingDiagnostic = null)
    {
        ArgumentOutOfRangeException.ThrowIfNegativeOrZero(stationIntervals);
        double length = heartline.Length;
        var paths = new List<WireframePath>();
        var boundaries = new List<WireframeBoundary>();
        var diagnostics = new List<string>();
        if (bankingDiagnostic is not null)
        {
            diagnostics.Add(bankingDiagnostic);
        }

        double[] knots = heartline.Nodes.Select(node => node.Station).ToArray();
        Vector3d HeartlineAt(double station, EvaluationSide side) =>
            heartline.Curve.Evaluate(heartline.Invert(station).Parameter, side).Position;

        AddPath(WireframePathKind.Heartline, "Heartline", knots, HeartlineAt);
        if (spine is not null)
        {
            IEnumerable<double> breaks = knots.Concat(spine.Banking.Breakpoints).Concat(spine.Offsets.Forward.Breakpoints)
                .Concat(spine.Offsets.Left.Breakpoints).Concat(spine.Offsets.Up.Breakpoints);
            AddPath(WireframePathKind.Spine, "Spine", Stations(breaks, length),
                (station, side) => spine.Evaluate(station, side).SpinePosition);
        }
        else if (bankingDiagnostic is null)
        {
            diagnostics.Add("This track has no spine offset definition.");
        }

        if (rails is not null)
        {
            double[] railStations = Stations(rails.Breakpoints, length);
            foreach (RailDefinition rail in rails.Arrangement.Rails)
            {
                AddPath(WireframePathKind.Rail, rail.Id, railStations,
                    (station, side) => rails.Evaluate(rail.Id, station, side).Position);
            }
            foreach (WireframeSupport support in supports)
            {
                try
                {
                    SupportMemberSample sample = SupportMemberGeometry.Evaluate(rails, support.RailId, support.DatumName,
                        support.HeartlineStation, support.FoundationPoint, support.Side);
                    paths.Add(new(WireframePathKind.Support, support.Id,
                        [new(sample.FoundationPoint, null), new(sample.RailAttachmentPoint, support.HeartlineStation)]));
                }
                catch (Exception error) when (error is ArgumentException or InvalidOperationException or KeyNotFoundException)
                {
                    diagnostics.Add($"Support {support.Id}: {error.Message}");
                }
            }
        }
        else if (bankingDiagnostic is null)
        {
            diagnostics.Add("This track has no running rail definitions.");
        }

        foreach (AuthoredRegion region in regions.Where(entry => entry.Definition.Domain == ProfileDomain.Distance))
        {
            AddBoundary(region, region.Definition.Start, isStart: true, EvaluationSide.Right);
            AddBoundary(region, region.Definition.End, isStart: false, EvaluationSide.Left);
        }

        var controls = heartline.Curve.ControlPoints.Select((point, index) => new WireframeControlPoint(index, point));
        return new(paths.AsReadOnly(), Array.AsReadOnly(controls.ToArray()), boundaries.AsReadOnly(), diagnostics.AsReadOnly());

        void AddPath(WireframePathKind kind, string id, double[] breaks, Func<double, EvaluationSide, Vector3d> evaluate)
        {
            var spans = new List<WireframePath>();
            try
            {
                for (int i = 1; i < breaks.Length; i++)
                {
                    double start = breaks[i - 1];
                    double end = breaks[i];
                    int intervals = (int)System.Math.Ceiling((end - start) / length * stationIntervals);
                    var vertices = new List<WireframeVertex> { new(evaluate(start, EvaluationSide.Right), start) };
                    for (int j = 1; j < intervals; j++)
                    {
                        double station = start + (end - start) * ((double)j / intervals);
                        if (station > vertices[^1].HeartlineStation && station < end)
                        {
                            vertices.Add(new(evaluate(station, EvaluationSide.Right), station));
                        }
                    }
                    vertices.Add(new(evaluate(end, EvaluationSide.Left), end));
                    spans.Add(new(kind, id, vertices.AsReadOnly()));
                }
                paths.AddRange(spans);
            }
            catch (Exception error) when (error is ArgumentException or InvalidOperationException)
            {
                diagnostics.Add($"{id}: {error.Message}");
            }
        }

        void AddBoundary(AuthoredRegion region, double station, bool isStart, EvaluationSide side)
        {
            if (station < 0 || station > length)
            {
                diagnostics.Add($"Region '{region.Name}' has a boundary outside the heartline station range.");
                return;
            }

            try
            {
                double parameter = heartline.Invert(station).Parameter;
                Vector3d position = heartline.Curve.Evaluate(parameter, side).Position;
                CurveFrame? orientation = spine?.Frames.Evaluate(parameter).Banked(spine.Banking.Evaluate(station, side).Value);
                boundaries.Add(new(region.Id, region.Name, isStart, station, position, orientation));
            }
            catch (Exception error) when (error is ArgumentException or InvalidOperationException)
            {
                diagnostics.Add($"Region '{region.Name}': {error.Message}");
            }
        }
    }

    private static double[] Stations(IEnumerable<double> breaks, double length) =>
        breaks.Where(station => station > 0 && station < length).Distinct().Order().Prepend(0).Append(length).ToArray();
}
