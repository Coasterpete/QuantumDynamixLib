using System;
using System.Collections.Generic;
using System.Linq;
using QuantumDynamix.Math.Numerics;
using QuantumDynamix.Math.Timelines;

namespace QuantumDynamix.FVD;

/// <summary>Integrates ground heading independently of tangent, so heading remains defined through vertical.</summary>
internal sealed class HeartlineGeometricHeading
{
    internal const double RadiansPerDegree = System.Math.PI / 180;
    private const double FullTurn = 2 * System.Math.PI;
    private readonly ScalarTimeline yaw;
    private readonly double[] stations;
    private readonly double[] headings;
    private readonly QuadratureSettings settings;
    private readonly bool fromEnd;

    internal HeartlineGeometricHeading(IEnumerable<ScalarTimeline> profiles, double initialHeading,
        bool fromEnd, ReconstructionWork work)
    {
        yaw = new(ProfileDomain.Distance, profiles.SelectMany(profile => profile.Segments));
        stations = [yaw.Minimum, .. yaw.Breakpoints, yaw.Maximum];
        headings = new double[stations.Length];
        // Use the existing adaptive quadrature on smooth pieces, with an absolute angular budget in radians.
        settings = new(absoluteTolerance: 1e-12 / (stations.Length - 1), relativeTolerance: 0);
        this.fromEnd = fromEnd;
        int direction = fromEnd ? -1 : 1;
        int start = fromEnd ? stations.Length - 1 : 0;
        headings[start] = initialHeading;
        for (int next = start + direction; next >= 0 && next < stations.Length; next += direction)
        {
            int previous = next - direction;
            headings[next] = AddHeading(headings[previous], Integrate(stations[previous], stations[next], work));
        }
    }

    internal double At(double station, ReconstructionWork work)
    {
        int index = Array.BinarySearch(stations, station);
        if (index >= 0) { return headings[index]; }
        int origin = fromEnd ? ~index : ~index - 1;
        return AddHeading(headings[origin], Integrate(stations[origin], station, work));
    }

    private static double AddHeading(double heading, double change) =>
        System.Math.IEEERemainder(heading + System.Math.IEEERemainder(change, FullTurn), FullTurn);

    private double Integrate(double start, double end, ReconstructionWork work) =>
        AdaptiveGaussKronrod.Integrate(station =>
        {
            work.Charge();
            return yaw.Evaluate(station).Value * RadiansPerDegree;
        }, start, end, settings).Value;
}
