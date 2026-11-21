using System;
using System.Linq;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using QuantumDynamix.Operations;
using static QuantumDynamix.Operations.BlockOccupancy;

namespace QuantumDynamix.Tests;

[TestClass]
public sealed class BlockOccupancyMapTests
{
    private const string RouteId = "Route";
    private const double RouteLength = 100;

    [TestMethod]
    [DataRow(30.0, 40.0, Occupied, Unoccupied, Unoccupied)]
    [DataRow(40.0, 55.0, Unoccupied, Occupied, Unoccupied)]
    [DataRow(40.0, 70.0, Unoccupied, Occupied, Unoccupied)]
    [DataRow(35.0, 45.0, Occupied, Occupied, Unoccupied)]
    [DataRow(35.0, 75.0, Occupied, Occupied, Occupied)]
    [DataRow(45.0, 35.0, Occupied, Occupied, Unoccupied)]
    [DataRow(0.0, 10.0, Occupied, Unoccupied, Unoccupied)]
    [DataRow(90.0, 100.0, Unoccupied, Unoccupied, Occupied)]
    public void OpenRouteReportsPositiveLengthOverlap(
        double rear, double front, BlockOccupancy a, BlockOccupancy b, BlockOccupancy c)
    {
        AssertOccupancy(OpenRoute(), new TrainRouteExtent(RouteId, rear, front, false), a, b, c);
    }

    [TestMethod]
    [DataRow(96.0, 4.0, true, Occupied, Unoccupied, Occupied)]
    [DataRow(4.0, 96.0, true, Occupied, Unoccupied, Occupied)]
    [DataRow(96.0, 4.0, false, Occupied, Occupied, Occupied)]
    [DataRow(96.0, 0.0, true, Unoccupied, Unoccupied, Occupied)]
    [DataRow(0.0, 4.0, false, Occupied, Unoccupied, Unoccupied)]
    public void ClosedRouteUsesTheExplicitArc(
        double rear, double front, bool wrapsAtSeam, BlockOccupancy a, BlockOccupancy b, BlockOccupancy c)
    {
        AssertOccupancy(ClosedRoute(), new TrainRouteExtent(RouteId, rear, front, wrapsAtSeam), a, b, c);
    }

    [TestMethod]
    public void WrappedBlockReportsOneSampleWhenBothPiecesOverlap()
    {
        var map = new BlockOccupancyMap(RouteId, RouteLength, RouteTopology.Closed,
            new BlockSection[] { new("S", 90, 10), new("M", 10, 90) });

        AssertOccupancy(map, new TrainRouteExtent(RouteId, 96, 4, true), Occupied, Unoccupied);
        AssertOccupancy(map, new TrainRouteExtent(RouteId, 10, 90, false), Unoccupied, Occupied);
        AssertOccupancy(map, new TrainRouteExtent(RouteId, 4, 96, false), Occupied, Occupied);
    }

    [TestMethod]
    public void ClosedBlockCanEndAtZeroWithoutOccupyingTheFirstStations()
    {
        var map = new BlockOccupancyMap(RouteId, RouteLength, RouteTopology.Closed,
            new BlockSection[] { new("Last", 90, 0), new("First", 0, 90) });

        AssertOccupancy(map, new TrainRouteExtent(RouteId, 96, 0, true), Occupied, Unoccupied);
        AssertOccupancy(map, new TrainRouteExtent(RouteId, 0, 4, false), Unoccupied, Occupied);
    }

    [TestMethod]
    public void ClosedBlockCanCoverTheFullRoute()
    {
        var map = new BlockOccupancyMap(RouteId, RouteLength, RouteTopology.Closed,
            new BlockSection[] { new("All", 0, RouteLength) });

        AssertOccupancy(map, new TrainRouteExtent(RouteId, 96, 4, true), Occupied);
    }

    [TestMethod]
    public void OneRepresentableStepPastTheBoundaryOccupiesTheNextBlock()
    {
        double front = System.Math.BitIncrement(40.0);

        AssertOccupancy(OpenRoute(), new TrainRouteExtent(RouteId, 30, front, false), Occupied, Occupied, Unoccupied);
    }

    [TestMethod]
    public void TrainWithinAGapOccupiesNeitherAdjacentBlock()
    {
        var map = new BlockOccupancyMap(RouteId, RouteLength, RouteTopology.Open,
            new BlockSection[] { new("A", 0, 20), new("B", 40, 70) });

        AssertOccupancy(map, new TrainRouteExtent(RouteId, 20, 40, false), Unoccupied, Unoccupied);
    }

    [TestMethod]
    public void BlockDefinitionsAreCopiedInInputOrderWithCaseSensitiveIds()
    {
        BlockSection[] definitions = { new("A", 40, 100), new("a", 0, 40) };
        var map = new BlockOccupancyMap(RouteId, RouteLength, RouteTopology.Open, definitions);
        definitions[0] = new("Replacement", 0, 100);

        CollectionAssert.AreEqual(new[] { new BlockSection("A", 40, 100), new BlockSection("a", 0, 40) }, map.Blocks.ToArray());
        AssertOccupancy(map, new TrainRouteExtent(RouteId, 20, 30, false), Unoccupied, Occupied);
    }

    [TestMethod]
    public void AbsentLocalizationReturnsUnknownForEveryBlock()
    {
        AssertOccupancy(OpenRoute(), null, Unknown, Unknown, Unknown);
    }

    [TestMethod]
    [DataRow("route")]
    [DataRow("Other")]
    public void DifferentRouteIdReturnsUnknownForEveryBlock(string routeId)
    {
        AssertOccupancy(OpenRoute(), new TrainRouteExtent(routeId, 35, 45, false), Unknown, Unknown, Unknown);
    }

    [TestMethod]
    [DataRow(RouteTopology.Open, 10.0, 10.0, false)]
    [DataRow(RouteTopology.Open, -1.0, 10.0, false)]
    [DataRow(RouteTopology.Open, 90.0, 101.0, false)]
    [DataRow(RouteTopology.Open, 90.0, 10.0, true)]
    [DataRow(RouteTopology.Open, double.NaN, 45.0, false)]
    [DataRow(RouteTopology.Open, 35.0, double.PositiveInfinity, false)]
    [DataRow(RouteTopology.Open, double.NegativeInfinity, 45.0, false)]
    [DataRow(RouteTopology.Closed, 96.0, 100.0, true)]
    [DataRow(RouteTopology.Closed, 100.0, 4.0, true)]
    [DataRow(RouteTopology.Closed, 0.0, 0.0, true)]
    [DataRow(RouteTopology.Closed, 10.0, 10.0, false)]
    [DataRow(RouteTopology.Closed, 96.0, -1.0, true)]
    public void InvalidStationsOrWrapFlagReturnUnknownForEveryBlock(
        RouteTopology topology, double rear, double front, bool wrapsAtSeam)
    {
        BlockOccupancyMap map = topology == RouteTopology.Open ? OpenRoute() : ClosedRoute();

        AssertOccupancy(map, new TrainRouteExtent(RouteId, rear, front, wrapsAtSeam), Unknown, Unknown, Unknown);
    }

    [TestMethod]
    [DataRow(0.0)]
    [DataRow(-1.0)]
    [DataRow(double.NaN)]
    [DataRow(double.PositiveInfinity)]
    public void InvalidRouteLengthIsRejected(double length)
    {
        Assert.ThrowsExactly<ArgumentOutOfRangeException>(() => new BlockOccupancyMap(RouteId, length, RouteTopology.Open,
            new BlockSection[] { new("A", 0, 40) }));
    }

    [TestMethod]
    public void InvalidRouteIdOrTopologyIsRejected()
    {
        BlockSection[] blocks = { new("A", 0, 40) };

        Assert.ThrowsExactly<ArgumentException>(() => new BlockOccupancyMap(" ", RouteLength, RouteTopology.Open, blocks));
        Assert.ThrowsExactly<ArgumentOutOfRangeException>(() => new BlockOccupancyMap(RouteId, RouteLength, (RouteTopology)99, blocks));
    }

    [TestMethod]
    [DataRow(RouteTopology.Open, -1.0, 40.0)]
    [DataRow(RouteTopology.Open, 0.0, 101.0)]
    [DataRow(RouteTopology.Open, 40.0, 40.0)]
    [DataRow(RouteTopology.Open, 90.0, 10.0)]
    [DataRow(RouteTopology.Open, double.NaN, 40.0)]
    [DataRow(RouteTopology.Open, 0.0, double.PositiveInfinity)]
    [DataRow(RouteTopology.Closed, 100.0, 10.0)]
    [DataRow(RouteTopology.Closed, 90.0, -1.0)]
    [DataRow(RouteTopology.Closed, 0.0, 0.0)]
    [DataRow(RouteTopology.Closed, 0.0, 101.0)]
    public void InvalidBlockBoundsAreRejected(RouteTopology topology, double start, double end)
    {
        Assert.ThrowsExactly<ArgumentException>(() => new BlockOccupancyMap(RouteId, RouteLength, topology,
            new BlockSection[] { new("A", start, end) }));
    }

    [TestMethod]
    public void InvalidBlockCollectionsAreRejected()
    {
        Assert.ThrowsExactly<ArgumentNullException>(() => new BlockOccupancyMap(RouteId, RouteLength, RouteTopology.Open, null!));
        Assert.ThrowsExactly<ArgumentException>(() => new BlockOccupancyMap(RouteId, RouteLength, RouteTopology.Open,
            Array.Empty<BlockSection>()));
        Assert.ThrowsExactly<ArgumentException>(() => new BlockOccupancyMap(RouteId, RouteLength, RouteTopology.Open,
            new BlockSection[] { null! }));
        Assert.ThrowsExactly<ArgumentException>(() => new BlockOccupancyMap(RouteId, RouteLength, RouteTopology.Open,
            new BlockSection[] { new(" ", 0, 40) }));
        Assert.ThrowsExactly<ArgumentException>(() => new BlockOccupancyMap(RouteId, RouteLength, RouteTopology.Open,
            new BlockSection[] { new("A", 0, 40), new("A", 40, 70) }));
    }

    [TestMethod]
    [DataRow(RouteTopology.Open, 20.0, 60.0, 0.0, 40.0)]
    [DataRow(RouteTopology.Closed, 90.0, 10.0, 0.0, 20.0)]
    [DataRow(RouteTopology.Closed, 90.0, 10.0, 80.0, 100.0)]
    [DataRow(RouteTopology.Closed, 90.0, 10.0, 95.0, 5.0)]
    public void OverlappingBlockInteriorsAreRejected(
        RouteTopology topology, double firstStart, double firstEnd, double secondStart, double secondEnd)
    {
        Assert.ThrowsExactly<ArgumentException>(() => new BlockOccupancyMap(RouteId, RouteLength, topology,
            new BlockSection[] { new("A", firstStart, firstEnd), new("B", secondStart, secondEnd) }));
    }

    private static BlockOccupancyMap OpenRoute() => new(RouteId, RouteLength, RouteTopology.Open,
        new BlockSection[] { new("A", 0, 40), new("B", 40, 70), new("C", 70, 100) });

    private static BlockOccupancyMap ClosedRoute() => new(RouteId, RouteLength, RouteTopology.Closed,
        new BlockSection[] { new("A", 0, 20), new("B", 20, 80), new("C", 80, 100) });

    private static void AssertOccupancy(BlockOccupancyMap map, TrainRouteExtent? extent, params BlockOccupancy[] expected)
    {
        var samples = map.Evaluate(extent);

        CollectionAssert.AreEqual(map.Blocks.Select(block => block.Id).ToArray(), samples.Select(sample => sample.BlockId).ToArray());
        CollectionAssert.AreEqual(expected, samples.Select(sample => sample.Occupancy).ToArray());
    }
}
