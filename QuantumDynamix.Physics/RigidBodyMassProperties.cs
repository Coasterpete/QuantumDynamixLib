using System;
using QuantumDynamix.Math;

namespace QuantumDynamix.Physics;

/// <summary>Constant mass (kg) and symmetric positive-definite body-frame inertia about the center of mass (kg*m^2).</summary>
/// <remarks>The caller supplies the mass distribution's tensor; no geometry-based inertia approximation is performed.</remarks>
public sealed class RigidBodyMassProperties
{
    private readonly Cholesky3x3 inertiaFactor;

    public double Mass { get; }
    public Matrix3x3d BodyInertia { get; }

    public RigidBodyMassProperties(double mass, Matrix3x3d bodyInertia)
    {
        if (!double.IsFinite(mass) || mass <= 0)
        {
            throw new ArgumentOutOfRangeException(nameof(mass), "Mass must be finite and positive.");
        }

        inertiaFactor = new Cholesky3x3(bodyInertia);
        Mass = mass;
        BodyInertia = bodyInertia;
    }

    /// <summary>Solves I_body*x=rightHandSide using the cached factorization.</summary>
    public Vector3d SolveBodyInertia(Vector3d rightHandSide) => inertiaFactor.Solve(rightHandSide);
}
