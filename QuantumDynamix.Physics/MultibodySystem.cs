using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using QuantumDynamix.Math;

namespace QuantumDynamix.Physics;

/// <summary>Three bilateral equations joining two body-local points, in metres from their centers of mass.</summary>
/// <remarks>The ideal joint permits all relative rotations and transmits no independent couple.</remarks>
public readonly record struct SphericalJoint(int BodyA, int BodyB, Vector3d LocalAnchorA, Vector3d LocalAnchorB);

/// <summary>Immutable ordered definitions for finite-mass bodies and spherical joints.</summary>
public sealed class MultibodySystem
{
    public ReadOnlyCollection<RigidBodyMassProperties> Bodies { get; }
    public ReadOnlyCollection<SphericalJoint> Joints { get; }

    public MultibodySystem(IEnumerable<RigidBodyMassProperties> bodies, IEnumerable<SphericalJoint> joints)
    {
        ArgumentNullException.ThrowIfNull(bodies);
        ArgumentNullException.ThrowIfNull(joints);
        RigidBodyMassProperties[] bodyArray = new List<RigidBodyMassProperties>(bodies).ToArray();
        if (bodyArray.Length == 0)
        {
            throw new ArgumentException("At least one body is required.", nameof(bodies));
        }

        foreach (RigidBodyMassProperties body in bodyArray)
        {
            ArgumentNullException.ThrowIfNull(body);
        }

        SphericalJoint[] jointArray = new List<SphericalJoint>(joints).ToArray();
        foreach (SphericalJoint joint in jointArray)
        {
            if ((uint)joint.BodyA >= (uint)bodyArray.Length || (uint)joint.BodyB >= (uint)bodyArray.Length
                || joint.BodyA == joint.BodyB)
            {
                throw new ArgumentException("Each joint must reference two distinct existing bodies.", nameof(joints));
            }
        }

        Bodies = Array.AsReadOnly(bodyArray);
        Joints = Array.AsReadOnly(jointArray);
    }

    internal void ValidateState(MultibodyState state)
    {
        ArgumentNullException.ThrowIfNull(state);
        if (state.Bodies.Count != Bodies.Count)
        {
            throw new ArgumentException("The state must contain one entry per system body.", nameof(state));
        }
    }
}

/// <summary>An immutable, copied snapshot in the same body order as its system definition.</summary>
public sealed class MultibodyState
{
    public ReadOnlyCollection<RigidBodyState> Bodies { get; }

    public MultibodyState(IEnumerable<RigidBodyState> bodies)
    {
        ArgumentNullException.ThrowIfNull(bodies);
        RigidBodyState[] copy = new List<RigidBodyState>(bodies).ToArray();
        if (copy.Length == 0)
        {
            throw new ArgumentException("At least one body state is required.", nameof(bodies));
        }

        foreach (RigidBodyState body in copy)
        {
            body.Validate(nameof(bodies));
        }

        Bodies = Array.AsReadOnly(copy);
    }
}
