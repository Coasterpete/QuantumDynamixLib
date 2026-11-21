using System;
using System.Collections.Generic;
using System.Linq;

namespace QuantumDynamix.Operations;

public enum RouteTopology { Open, Closed }

/// <summary>Occupancy of a block by the train supplied to the query.</summary>
public enum BlockOccupancy { Unknown, Unoccupied, Occupied }

/// <summary>A half-open block in heartline metres. A descending pair wraps on a closed route.</summary>
public sealed record BlockSection(string Id, double StartStationMetres, double EndStationMetres);

/// <summary>The physical rear and front of one train at the same instant, in heartline metres.</summary>
/// <param name="RouteId">The route and station frame used to localize both ends.</param>
/// <param name="RearStationMetres">The station of the train's physical rear.</param>
/// <param name="FrontStationMetres">The station of the train's physical front.</param>
/// <param name="WrapsAtSeam">
/// Selects the arc from the higher station through the closed-route seam to the lower station.
/// Closed-route endpoints must be in [0, length), and the extent must be shorter than one lap.
/// </param>
public readonly record struct TrainRouteExtent(
    string RouteId, double RearStationMetres, double FrontStationMetres, bool WrapsAtSeam);

public readonly record struct BlockOccupancySample(string BlockId, BlockOccupancy Occupancy);

/// <summary>An immutable set of disjoint block interiors on one route in heartline metres.</summary>
public sealed class BlockOccupancyMap
{
    public string RouteId { get; }
    public double LengthMetres { get; }
    public RouteTopology Topology { get; }
    public IReadOnlyList<BlockSection> Blocks { get; }

    public BlockOccupancyMap(
        string routeId, double lengthMetres, RouteTopology topology, IEnumerable<BlockSection> blocks)
    {
        if (string.IsNullOrWhiteSpace(routeId))
        {
            throw new ArgumentException("A route ID is required.", nameof(routeId));
        }

        if (!double.IsFinite(lengthMetres) || lengthMetres <= 0)
        {
            throw new ArgumentOutOfRangeException(nameof(lengthMetres), "Route length must be finite and positive.");
        }

        if (topology is not (RouteTopology.Open or RouteTopology.Closed))
        {
            throw new ArgumentOutOfRangeException(nameof(topology));
        }

        ArgumentNullException.ThrowIfNull(blocks);
        BlockSection[] definitions = blocks.ToArray();
        if (definitions.Length == 0)
        {
            throw new ArgumentException("Supply at least one block.", nameof(blocks));
        }

        RouteId = routeId;
        LengthMetres = lengthMetres;
        Topology = topology;

        var ids = new HashSet<string>(StringComparer.Ordinal);
        var intervals = new List<(StationInterval Interval, string BlockId)>();
        foreach (BlockSection block in definitions)
        {
            if (block is null || string.IsNullOrWhiteSpace(block.Id) || !ids.Add(block.Id))
            {
                throw new ArgumentException("Blocks must have nonblank, unique IDs.", nameof(blocks));
            }

            double start = block.StartStationMetres;
            double end = block.EndStationMetres;
            if (!double.IsFinite(start) || !double.IsFinite(end) || start < 0 || start >= lengthMetres
                || end < 0 || end > lengthMetres || start == end || (topology == RouteTopology.Open && start > end))
            {
                throw new ArgumentException($"Block '{block.Id}' has invalid station bounds.", nameof(blocks));
            }

            var (first, second) = SplitAtSeam(start, end);
            intervals.Add((first, block.Id));
            if (second.Start < second.End)
            {
                intervals.Add((second, block.Id));
            }
        }

        intervals.Sort(static (left, right) => left.Interval.Start.CompareTo(right.Interval.Start));
        for (int i = 1; i < intervals.Count; i++)
        {
            var previous = intervals[i - 1];
            var current = intervals[i];
            if (current.Interval.Start < previous.Interval.End)
            {
                throw new ArgumentException($"Blocks '{previous.BlockId}' and '{current.BlockId}' overlap.", nameof(blocks));
            }
        }

        Blocks = Array.AsReadOnly(definitions);
    }

    /// <summary>Returns one sample per block in definition order for a single train snapshot.</summary>
    /// <remarks>A missing or invalid extent yields Unknown for every block.</remarks>
    public IReadOnlyList<BlockOccupancySample> Evaluate(TrainRouteExtent? extent)
    {
        if (extent is not TrainRouteExtent train || !StringComparer.Ordinal.Equals(train.RouteId, RouteId)
            || !IsTrainStation(train.RearStationMetres) || !IsTrainStation(train.FrontStationMetres)
            || train.RearStationMetres == train.FrontStationMetres || (train.WrapsAtSeam && Topology == RouteTopology.Open))
        {
            return Array.AsReadOnly(Blocks.Select(block => new BlockOccupancySample(block.Id, BlockOccupancy.Unknown)).ToArray());
        }

        double low = System.Math.Min(train.RearStationMetres, train.FrontStationMetres);
        double high = System.Math.Max(train.RearStationMetres, train.FrontStationMetres);
        var (trainFirst, trainSecond) = train.WrapsAtSeam ? SplitAtSeam(high, low) : SplitAtSeam(low, high);

        var samples = new BlockOccupancySample[Blocks.Count];
        for (int i = 0; i < Blocks.Count; i++)
        {
            BlockSection block = Blocks[i];
            var (blockFirst, blockSecond) = SplitAtSeam(block.StartStationMetres, block.EndStationMetres);
            bool intersects = trainFirst.Overlaps(blockFirst) || trainFirst.Overlaps(blockSecond)
                || trainSecond.Overlaps(blockFirst) || trainSecond.Overlaps(blockSecond);
            samples[i] = new(block.Id, intersects ? BlockOccupancy.Occupied : BlockOccupancy.Unoccupied);
        }

        return Array.AsReadOnly(samples);
    }

    private bool IsTrainStation(double station) => double.IsFinite(station) && station >= 0
        && (Topology == RouteTopology.Closed ? station < LengthMetres : station <= LengthMetres);

    private (StationInterval First, StationInterval Second) SplitAtSeam(double start, double end) =>
        start < end ? (new(start, end), default) : (new(start, LengthMetres), new(0, end));

    private readonly record struct StationInterval(double Start, double End)
    {
        public bool Overlaps(StationInterval other) =>
            System.Math.Max(Start, other.Start) < System.Math.Min(End, other.End);
    }
}
