using System;
using System.Collections.Generic;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using QuantumDynamix.Math;
using QuantumDynamix.Math.Functions;
using QuantumDynamix.Supports;
using QuantumDynamix.Track;
using static QuantumDynamix.Tests.RailTestFixtures;

namespace QuantumDynamix.Tests;

[TestClass]
public sealed class SupportMemberGeometryTests
{
    private const double Tolerance = 1e-12;

    [TestMethod]
    public void StraightUnbankedRailResolvesNamedDatumAboveFoundation()
    {
        RailGeometry geometry = Straight(Rail("rail", new(0, 0, 5), 10, Section(1)));
        Vector3d foundation = new(4, 1, 0);

        SupportMemberSample member = SupportMemberGeometry.Evaluate(geometry, "rail", "positive", 4, foundation);

        Assert.AreEqual(foundation, member.FoundationPoint);
        TestMath.Near(new(4, 1, 5), member.RailAttachmentPoint, Tolerance);
        Assert.AreEqual(5, member.Length, Tolerance);
        TestMath.Near(Vector3d.UnitZ, member.Direction, Tolerance);
    }

    [TestMethod]
    public void TranslatedRailAppliesBankedOffsetsAndSectionRoll()
    {
        const double quarterTurn = System.Math.PI / 2;
        var track = new HeartlineTrack(Line(origin: new(10, 20, 30)), Vector3d.UnitZ,
            TrackOffsetProfile.Constant(new(0, 1, 0), 10), Constant(quarterTurn, 10));
        RailDefinition rail = Rail("rail", new(0, 0, 2), 10, Section(1), Constant(quarterTurn, 10));
        var geometry = new RailGeometry(track, new(new[] { rail }), Settings);
        Vector3d foundation = new(14, 14, 27);

        SupportMemberSample member = SupportMemberGeometry.Evaluate(geometry, "rail", "positive", 4, foundation);

        // Banking sends the spine offset toward +Z and the rail offset toward -Y.
        // The separate section roll sends the positive-left datum 1 m farther toward -Y.
        Assert.AreEqual(foundation, member.FoundationPoint);
        TestMath.Near(new(14, 17, 31), member.RailAttachmentPoint, Tolerance);
        Assert.AreEqual(5, member.Length, Tolerance);
        TestMath.Near(new(0, 0.6, 0.8), member.Direction, Tolerance);
    }

    [TestMethod]
    public void ExplicitSidesResolveDifferentDatumPositionsAtRollJoin()
    {
        const double station = 5;
        const double quarterTurn = System.Math.PI / 2;
        var roll = new CustomFunction(0, 10,
            (s, side) => new(s < station || (s == station && side == EvaluationSide.Left) ? 0 : quarterTurn),
            new[] { station });
        RailGeometry geometry = Straight(Rail("rail", default, 10, Section(1), roll));
        Vector3d foundation = new(5, 0, -1);

        SupportMemberSample left = SupportMemberGeometry.Evaluate(geometry, "rail", "positive", station, foundation, EvaluationSide.Left);
        SupportMemberSample right = SupportMemberGeometry.Evaluate(geometry, "rail", "positive", station, foundation, EvaluationSide.Right);

        TestMath.Near(new(5, 1, 0), left.RailAttachmentPoint, Tolerance);
        TestMath.Near(new(5, 0, 1), right.RailAttachmentPoint, Tolerance);
        Assert.AreEqual(System.Math.Sqrt(2), left.Length, Tolerance);
        Assert.AreEqual(2, right.Length, Tolerance);
    }

    [TestMethod]
    public void BothHeartlineStationBoundariesAreAccepted()
    {
        RailGeometry geometry = StraightMemberRail();

        SupportMemberSample start = SupportMemberGeometry.Evaluate(geometry, "rail", "positive", 0, default);
        SupportMemberSample end = SupportMemberGeometry.Evaluate(geometry, "rail", "positive", geometry.HeartlineLength, default);

        TestMath.Near(new(0, 1, 0), start.RailAttachmentPoint, Tolerance);
        TestMath.Near(new(10, 1, 0), end.RailAttachmentPoint, Tolerance);
    }

    [TestMethod]
    [DataRow(-0.1)]
    [DataRow(10.1)]
    [DataRow(double.NaN)]
    [DataRow(double.NegativeInfinity)]
    [DataRow(double.PositiveInfinity)]
    public void InvalidHeartlineStationIsRejected(double station)
    {
        RailGeometry geometry = StraightMemberRail();

        var error = Assert.ThrowsExactly<ArgumentOutOfRangeException>(() =>
            SupportMemberGeometry.Evaluate(geometry, "rail", "positive", station, default));

        Assert.AreEqual("heartlineStation", error.ParamName);
    }

    [TestMethod]
    public void UndefinedEvaluationSideIsRejected()
    {
        RailGeometry geometry = StraightMemberRail();

        var error = Assert.ThrowsExactly<ArgumentOutOfRangeException>(() =>
            SupportMemberGeometry.Evaluate(geometry, "rail", "positive", 4, default, (EvaluationSide)99));

        Assert.AreEqual("side", error.ParamName);
    }

    [TestMethod]
    public void NullGeometryAndBlankIdentifiersAreRejected()
    {
        RailGeometry geometry = StraightMemberRail();

        Assert.ThrowsExactly<ArgumentNullException>(() => SupportMemberGeometry.Evaluate(null!, "rail", "positive", 4, default));
        Assert.ThrowsExactly<ArgumentException>(() => SupportMemberGeometry.Evaluate(geometry, " ", "positive", 4, default));
        Assert.ThrowsExactly<ArgumentException>(() => SupportMemberGeometry.Evaluate(geometry, "rail", "", 4, default));
    }

    [TestMethod]
    [DataRow("Rail", "positive")]
    [DataRow("rail", "Positive")]
    public void RailAndDatumIdentifiersAreCaseSensitive(string railId, string datumName)
    {
        RailGeometry geometry = StraightMemberRail();

        Assert.ThrowsExactly<KeyNotFoundException>(() => SupportMemberGeometry.Evaluate(geometry, railId, datumName, 4, default));
    }

    [TestMethod]
    public void CoincidentEndpointsAreRejected()
    {
        RailGeometry geometry = StraightMemberRail();

        Assert.ThrowsExactly<InvalidOperationException>(() =>
            SupportMemberGeometry.Evaluate(geometry, "rail", "positive", 0, new(0, 1, 0)));
    }

    [TestMethod]
    public void ShortNonzeroSpanHasItsActualLengthAndDirection()
    {
        const double length = 1e-12;
        RailGeometry geometry = StraightMemberRail();

        SupportMemberSample member = SupportMemberGeometry.Evaluate(geometry, "rail", "positive", 0, new(0, 1, -length));

        Assert.AreEqual(length, member.Length);
        Assert.AreEqual(Vector3d.UnitZ, member.Direction);
    }

    [TestMethod]
    public void UnavailableSectionFramePreservesRailDiagnostic()
    {
        const double station = 4;
        var valueOnly = new CustomFunction(0, 10, (_, _) => new(0, derivativeOrder: 0));
        var offsets = new TrackOffsetProfile(Constant(0, 10), valueOnly, Constant(0, 10));
        RailGeometry geometry = Straight(new RailDefinition("rail", offsets, Section(1), Constant(0, 10)));

        var error = Assert.ThrowsExactly<RailGeometryException>(() =>
            SupportMemberGeometry.Evaluate(geometry, "rail", "positive", station, default));

        Assert.AreEqual(RailDiagnosticCode.UnavailableDerivative, error.Diagnostic.Code);
        Assert.AreEqual("rail", error.Diagnostic.RailId);
        Assert.AreEqual(station, error.Diagnostic.HeartlineStation);
    }

    private static RailGeometry StraightMemberRail() => Straight(Rail("rail", default, 10, Section(1)));
}
