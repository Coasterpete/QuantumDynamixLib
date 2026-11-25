using Microsoft.VisualStudio.TestTools.UnitTesting;
using QuantumDynamix.App;
using QuantumDynamix.Math.Timelines;

namespace QuantumDynamix.Tests.Application;

[TestClass]
public sealed class TransitionEditStateTests
{
    [TestMethod]
    public void KeyframeCandidateKeepsItsIdentityAndRejectsInvalidCoordinates()
    {
        var state = new TransitionEditState();
        var keyframe = new TimelineKeyframe(16, .25);
        var target = new TransitionDragTarget(10, 25, (origin, pointer) => new TransitionEdit.Keyframe("Red", keyframe,
            new(keyframe.Coordinate + (pointer.Distance - origin.Distance), keyframe.Value + (pointer.Value - origin.Value))));

        Assert.IsTrue(state.TryBegin(target, new(15, .5)));
        Assert.IsFalse(state.TryBegin(target, new(22, .8)));
        state.Update(new(18, .75));
        Assert.AreEqual(new TransitionEdit.Keyframe("Red", keyframe, new(19, .5)), state.Candidate);
        state.Update(new(24, .75));
        Assert.IsNull(state.Candidate); // The candidate would coincide with the next keyframe.
        Assert.IsNull(state.Finish(new(18, double.NaN)));
        Assert.IsNull(state.Candidate);
    }

    [TestMethod]
    public void BoundaryCandidateMustLeaveBothNeighboringSpansPositive()
    {
        var state = new TransitionEditState();
        var target = new TransitionDragTarget(0, 55, (_, pointer) => new TransitionEdit.Boundary(25, pointer.Distance));

        Assert.IsFalse(state.TryBegin(target, new(double.PositiveInfinity, 0)));
        Assert.IsTrue(state.TryBegin(target, new(25, 0)));
        state.Update(new(0, 0));
        Assert.IsNull(state.Candidate);
        state.Update(new(56, 0));
        Assert.IsNull(state.Candidate);
        Assert.AreEqual(new TransitionEdit.Boundary(25, 31), state.Finish(new(31, 0)));
        Assert.IsNull(state.Candidate);
        Assert.IsTrue(state.TryBegin(target, new(25, 0)));
        state.Update(new(34, 0));
        Assert.IsNotNull(state.Candidate);
        state.Cancel();
        Assert.IsNull(state.Finish(new(34, 0)));
    }
}
