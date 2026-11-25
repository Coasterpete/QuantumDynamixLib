using System.Collections.Generic;
using System.Linq;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using QuantumDynamix.App;
using QuantumDynamix.Math.Timelines;
using SystemMath = System.Math;

namespace QuantumDynamix.Tests.Application;

[TestClass]
public sealed class TransitionGraphFixtureTests
{
    [TestMethod]
    public void FourDistanceRegionsShareFiveContinuousChannels()
    {
        IReadOnlyList<Region> regions = TransitionGraphFixture.CreateRegions();
        CollectionAssert.AreEqual(new[] { 0.0, 25, 55, 80, 100 }, regions.Select(r => r.Start).Append(regions[^1].End).ToArray());
        string[] channels = ["Blue", "Red", "Green", "Purple", "Gold"];
        for (int i = 0; i < regions.Count; i++)
        {
            Region region = regions[i];
            Assert.AreEqual(ProfileDomain.Distance, region.Domain);
            CollectionAssert.AreEquivalent(channels, region.Channels.Keys.ToArray());
            if (i + 1 < regions.Count)
            {
                Assert.AreEqual(region.End, regions[i + 1].Start);
                foreach (string channel in channels)
                {
                    RegionJoinDiagnostic join = region.DiagnoseJoin(regions[i + 1], channel, [1e-12, 1e-12]);
                    Assert.AreEqual(1, join.MatchingOrder, $"{channel} at {region.End} m");
                }
            }
        }
    }

    [TestMethod]
    public void QuarterStationMatchesIndependentReferencePolynomials()
    {
        Region region = TransitionGraphFixture.CreateRegions()[0];
        const double t = .25;
        Assert.AreEqual(.12 + .8 * t, region.Channels["Blue"].Evaluate(25).Value, 1e-12);
        Assert.AreEqual(.15 + .8 * (3 * t * t - 2 * SystemMath.Pow(t, 3)), region.Channels["Red"].Evaluate(25).Value, 1e-12);
        Assert.AreEqual(.09 + .8 * (10 * SystemMath.Pow(t, 3) - 15 * SystemMath.Pow(t, 4) + 6 * SystemMath.Pow(t, 5)),
            region.Channels["Green"].Evaluate(25).Value, 1e-12);
        Assert.AreEqual(.18 + .8 * (35 * SystemMath.Pow(t, 4) - 84 * SystemMath.Pow(t, 5)
            + 70 * SystemMath.Pow(t, 6) - 20 * SystemMath.Pow(t, 7)),
            region.Channels["Purple"].Evaluate(25).Value, 1e-12);
        Assert.AreEqual(.06 + .4 * (1 - SystemMath.Cos(SystemMath.PI * t)), region.Channels["Gold"].Evaluate(25).Value, 1e-12);
    }
}
