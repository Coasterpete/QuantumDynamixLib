using System;

namespace QuantumDynamix.FVD;

public static class HeartlineReconstructor
{
    /// <summary>Integrates a forward numerical heartline using the supplied transverse coordinate accelerations.</summary>
    public static ReconstructedHeartline Reconstruct(HeartlineReconstructionRequest request,
        HeartlineReconstructionSettings settings)
    {
        ArgumentNullException.ThrowIfNull(request);
        ArgumentNullException.ThrowIfNull(settings);
        settings.Validate();
        var work = new ReconstructionWork(settings.MaximumWork);
        var inputs = new HeartlineReconstructionInputs(request, settings, work);
        return new HeartlineReconstructionIntegrator(inputs, settings, work).Integrate(request.InitialUnbankedPose);
    }

    /// <summary>Fits a degree-five spline of the numerical heartline and forward-verifies it at sampled stations.</summary>
    public static HeartlineFitResult FitToTrack(ReconstructedHeartline source, HeartlineFitSettings settings)
    {
        ArgumentNullException.ThrowIfNull(source);
        ArgumentNullException.ThrowIfNull(settings);
        settings.Validate();
        return new HeartlineSplineConversion(source, settings).Fit();
    }
}
