using System.Linq;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using QuantumDynamix.App;
using QuantumDynamix.Geometry;
using QuantumDynamix.Math;

namespace QuantumDynamix.Tests.Application;

[TestClass]
public sealed class TrackCameraTests
{
    [TestMethod]
    public void LevelRegionShowsTheUprightCrossSectionInTopSideAndEndViews()
    {
        TrackWireframe scene = LevelRegion();
        Vector3d[] points = scene.Paths.SelectMany(path => path.Vertices.Select(vertex => vertex.Position)).ToArray();
        var camera = new TrackCamera();

        var (heartline, spine, left, right) = CrossSection(ProjectView(TrackView.Top));
        Assert.AreEqual(heartline.X, spine.X, 1e-10);
        Assert.AreEqual(heartline.Y, spine.Y, 1e-10);
        Assert.AreEqual(heartline.X, left.X, 1e-10);
        Assert.AreEqual(heartline.X, right.X, 1e-10);
        Assert.IsTrue(left.Y < heartline.Y && heartline.Y < right.Y);
        Assert.AreEqual(heartline.Y, (left.Y + right.Y) / 2, 1e-10);

        // At zero heading the Front camera sees the side of the track and Left sees it end-on.
        (heartline, spine, left, right) = CrossSection(ProjectView(TrackView.Front));
        Assert.AreEqual(left.X, right.X, 1e-10);
        Assert.AreEqual(left.Y, right.Y, 1e-10);
        Assert.IsTrue(heartline.Y < left.Y && left.Y < spine.Y);
        Assert.AreEqual(3 * (spine.Y - left.Y), left.Y - heartline.Y, 1e-10);

        ProjectedWireframe end = ProjectView(TrackView.Left);
        (heartline, spine, left, right) = CrossSection(end);
        Assert.IsTrue(left.X < heartline.X && heartline.X < right.X);
        Assert.AreEqual(heartline.X, (left.X + right.X) / 2, 1e-10);
        Assert.AreEqual(heartline.X, spine.X, 1e-10);
        Assert.AreEqual(left.Y, right.Y, 1e-10);
        Assert.IsTrue(heartline.Y < left.Y && left.Y < spine.Y);
        Assert.AreEqual(right.X - left.X, spine.Y - heartline.Y, 1e-10);
        foreach (ProjectedLine line in end.Lines)
        {
            Assert.AreEqual(line.Start.X, line.End.X, 1e-10);
            Assert.AreEqual(line.Start.Y, line.End.Y, 1e-10);
        }

        ProjectedWireframe ProjectView(TrackView view)
        {
            camera.SetView(view);
            camera.Frame(points, 640, 480);
            return new(scene, camera.ProjectTo(640, 480), 0);
        }

        static (ProjectedPoint Heartline, ProjectedPoint Spine, ProjectedPoint Left, ProjectedPoint Right)
            CrossSection(ProjectedWireframe frame)
        {
            ProjectedPoint StartOf(string id) => frame.Lines.First(line => line.Selection.Id == id).Start;
            return (StartOf("Heartline"), StartOf("Spine"), StartOf("left"), StartOf("right"));
        }
    }

    [TestMethod]
    public void PerspectiveChangesTheApparentHeartlineToSpineSeparationWithDepth()
    {
        TrackWireframe scene = LevelRegion();
        Vector3d[] points = scene.Paths.SelectMany(path => path.Vertices.Select(vertex => vertex.Position)).ToArray();
        WireframeVertex[] heartline = TrackWireframeTests.Vertices(scene, "Heartline");
        WireframeVertex[] spine = TrackWireframeTests.Vertices(scene, "Spine");
        var camera = new TrackCamera();

        camera.SetView(TrackView.Isometric);
        camera.Frame(points, 640, 480);
        CameraProjection isometric = camera.ProjectTo(640, 480);
        Assert.AreEqual(Gap(isometric, 0), Gap(isometric, heartline.Length - 1), 1e-10);

        camera.SetView(TrackView.Perspective);
        camera.Frame(points, 640, 480);
        CameraProjection perspective = camera.ProjectTo(640, 480);
        Assert.IsGreaterThan(Gap(perspective, 0), Gap(perspective, heartline.Length - 1));

        double Gap(CameraProjection projection, int index)
        {
            ProjectedPoint heart = Project(projection, heartline[index].Position);
            ProjectedPoint structure = Project(projection, spine[index].Position);
            return double.Hypot(heart.X - structure.X, heart.Y - structure.Y);
        }
    }

    [TestMethod]
    public void FixedViewsUseApprovedAxesAndFrameReferenceGeometry()
    {
        TrackWireframe scene = ReferenceTrack.Create().SampleWireframe(24);
        Vector3d[] points = scene.Paths.SelectMany(path => path.Vertices.Select(vertex => vertex.Position)).ToArray();
        Vector3d center = points[12];
        (TrackView View, Vector3d TowardScene, Vector3d Up, Vector3d Right)[] fixedViews =
        [
            (TrackView.Front, Vector3d.UnitY, Vector3d.UnitZ, Vector3d.UnitX),
            (TrackView.Back, -Vector3d.UnitY, Vector3d.UnitZ, -Vector3d.UnitX),
            (TrackView.Left, Vector3d.UnitX, Vector3d.UnitZ, -Vector3d.UnitY),
            (TrackView.Right, -Vector3d.UnitX, Vector3d.UnitZ, Vector3d.UnitY),
            (TrackView.Top, -Vector3d.UnitZ, Vector3d.UnitY, Vector3d.UnitX),
            (TrackView.Bottom, Vector3d.UnitZ, Vector3d.UnitY, -Vector3d.UnitX)
        ];
        var camera = new TrackCamera();
        foreach (var (view, towardScene, up, right) in fixedViews)
        {
            camera.SetView(view);
            camera.Frame(points, 640, 480);
            var projection = camera.ProjectTo(640, 480);
            Assert.IsTrue(points.All(point => projection.TryProject(point, out _)), view.ToString());
            ProjectedPoint p = Project(projection, center);
            ProjectedPoint ahead = Project(projection, center + towardScene * .1);
            Assert.AreEqual(p.X, ahead.X, 1e-10);
            Assert.AreEqual(p.Y, ahead.Y, 1e-10);
            Assert.AreEqual(.1, ahead.Depth - p.Depth, 1e-10);
            Assert.IsTrue(Project(projection, center + right * .1).X > p.X, view.ToString());
            Assert.IsTrue(Project(projection, center + up * .1).Y < p.Y, view.ToString());
        }
    }

    [TestMethod]
    public void ProjectionClipsReferenceRailAndPickingUsesVisibleSegments()
    {
        TrackWireframe scene = ReferenceTrack.Create().SampleWireframe(48);
        Vector3d[] points = scene.Paths.SelectMany(path => path.Vertices.Select(vertex => vertex.Position)).ToArray();
        var camera = new TrackCamera();
        camera.SetView(TrackView.Top);
        camera.Frame(points, 640, 480);
        var projection = camera.ProjectTo(640, 480);
        var frame = new ProjectedWireframe(scene, projection, 0);
        ProjectedLine rail = frame.Lines.Where(line => line.Selection.Id == "inner").ElementAt(24);
        double x = (rail.Start.X + rail.End.X) / 2;
        double y = (rail.Start.Y + rail.End.Y) / 2;
        Assert.AreEqual("inner", frame.Pick(x, y, 5)?.Id);
        Assert.IsNull(frame.Pick(0, 0, 5));
        ProjectedMarker control = frame.Markers.Single(marker => marker.Selection.Label == "Control point 2");
        Assert.AreEqual(WireframeElement.ControlPoint, frame.Pick(control.Point.X, control.Point.Y, 5)?.Element);

        camera.Zoom(.55);
        projection = camera.ProjectTo(640, 480);
        var clipped = new ProjectedWireframe(scene, projection, 0);
        Assert.IsTrue(clipped.Lines.Count < frame.Lines.Count);
        Assert.IsTrue(clipped.Lines.Any(line => line.Selection.Id == "inner" &&
            new[] { line.Start, line.End }.Any(point => point.X < 1e-8 || point.X > 640 - 1e-8 ||
                point.Y < 1e-8 || point.Y > 480 - 1e-8)));
        Assert.IsTrue(clipped.Lines.SelectMany(line => new[] { line.Start, line.End }).All(point =>
            point.X is >= -1e-8 and <= 640.00000001 && point.Y is >= -1e-8 and <= 480.00000001));
    }

    [TestMethod]
    public void PerspectiveNearClipsSegmentsAndOrdersOverlappingPicksByDepth()
    {
        TrackWireframe scene = ReferenceTrack.Create().SampleWireframe(24);
        Vector3d origin = TrackWireframeTests.Vertices(scene, "inner")[5].Position;
        var projection = new CameraProjection(origin, Vector3d.UnitX, Vector3d.UnitZ, Vector3d.UnitY,
            true, 1, .5, 400, 300);
        Assert.IsFalse(projection.TryProject(origin - Vector3d.UnitY * 3, out _));
        Assert.IsFalse(projection.TryProjectSegment(origin - Vector3d.UnitY * 4, origin - Vector3d.UnitY * 3, out _, out _));
        Assert.IsTrue(projection.TryProjectSegment(origin - Vector3d.UnitY * 3, origin, out var clipped, out var inside));
        Assert.IsTrue(clipped.Depth > 0 && clipped.Depth < inside.Depth);
        Assert.AreEqual(200, clipped.X, 1e-10);
        Assert.AreEqual(150, clipped.Y, 1e-10);

        // Two paths intersect the same screen pixel at different camera depths.
        var far = new WireframePath(WireframePathKind.Rail, "far",
            [new(origin - Vector3d.UnitX + Vector3d.UnitY, 0), new(origin + Vector3d.UnitX + Vector3d.UnitY, 1)]);
        var near = new WireframePath(WireframePathKind.Rail, "near",
            [new(origin - Vector3d.UnitZ, 0), new(origin + Vector3d.UnitZ, 1)]);
        var frame = new ProjectedWireframe(new([far, near], [], [], []), projection, 0);
        Assert.AreEqual("near", frame.Pick(200, 150, 5)?.Id);
        Assert.AreEqual(1.5, projection.DepthAt(new(0, 0, 1), new(4, 0, 3), .5), 1e-12);
    }

    private static ProjectedPoint Project(CameraProjection projection, Vector3d point)
    {
        Assert.IsTrue(projection.TryProject(point, out ProjectedPoint result));
        return result;
    }

    private static TrackWireframe LevelRegion()
    {
        TrackDocument document = TrackDocument.CreateAuthored("Level", RigidPose3d.Identity, 10);
        Assert.IsNull(document.Timeline.AddRegion(prepend: false, length: 12, initialStation: 0));
        return document.SampleWireframe(RegionTrackGeometry.DisplayIntervals);
    }
}
