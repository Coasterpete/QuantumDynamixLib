using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.Linq;
using QuantumDynamix.Math;

namespace QuantumDynamix.Physics;

/// <summary>A complete owner-supplied normal-law assignment to existing geometry bindings.
/// The first formulation permits one bound patch per wheel and stationary rigid rails.</summary>
public sealed class CompliantWheelRailContactModel
{
    public WheelRailContactSystem Geometry { get; }
    public ReadOnlyDictionary<WheelRailPair, PowerLawNormalContact> Laws { get; }

    public CompliantWheelRailContactModel(WheelRailContactSystem geometry,
        IEnumerable<KeyValuePair<WheelRailPair, PowerLawNormalContact>> laws)
    {
        ArgumentNullException.ThrowIfNull(geometry);
        ArgumentNullException.ThrowIfNull(laws);
        var copy = new Dictionary<WheelRailPair, PowerLawNormalContact>();
        foreach (var entry in laws)
        {
            if (entry.Key is null || entry.Value is null || !copy.TryAdd(entry.Key, entry.Value))
            { throw new ArgumentException("Supply non-null, unique pair-to-law assignments.", nameof(laws)); }
        }
        if (copy.Count != geometry.Pairs.Count || geometry.Pairs.Any(p => !copy.ContainsKey(p)))
        { throw new ArgumentException("Assign exactly one explicit law to every bound pair.", nameof(laws)); }
        if (geometry.Pairs.Select(p => p.WheelId).Distinct(StringComparer.Ordinal).Count() != geometry.Pairs.Count)
        { throw new ArgumentException("Independent local tread compliance permits one bound patch per wheel.", nameof(geometry)); }
        Geometry = geometry;
        Laws = new(geometry.Pairs.ToDictionary(p => p, p => copy[p]));
    }
}

/// <summary>Instantaneous contact force (N) and world COM torque (N*m), never an impulse or step average.</summary>
public readonly record struct WheelRailContactForce(WheelRailContactSample Geometry,
    NormalContactResponse Response, Vector3d WorldForce, Vector3d WorldTorque);

public sealed class CompliantContactEvaluation
{
    public ReadOnlyCollection<WheelRailContactForce> Contacts { get; }
    public ReadOnlyCollection<RigidBodyLoads> BodyLoads { get; }
    public double ElasticEnergy { get; }
    public double DissipationPower { get; }

    internal CompliantContactEvaluation(WheelRailContactForce[] contacts, RigidBodyLoads[] loads)
    {
        Contacts = Array.AsReadOnly(contacts);
        BodyLoads = Array.AsReadOnly(loads);
        ElasticEnergy = contacts.Sum(c => c.Response.ElasticEnergy);
        DissipationPower = contacts.Sum(c => c.Response.DissipationPower);
        if (!double.IsFinite(ElasticEnergy) || !double.IsFinite(DissipationPower))
        { throw new ArithmeticException("Contact energy/power sum exceeds finite range."); }
    }
}

/// <summary>Pure signed-gap-to-force evaluation. Negative geometric gaps represent physical compression;
/// no rigid nonpenetration projection, contact impulse, friction, or mounting motion is imposed.</summary>
public static class CompliantWheelRailContact
{
    public static CompliantContactEvaluation Evaluate(CompliantWheelRailContactModel model, MultibodyState state,
        ContactGeometrySettings? settings = null)
    {
        ArgumentNullException.ThrowIfNull(model);
        model.Geometry.Dynamics.ValidateState(state);
        ContactFormulationScope.Check(model.Geometry, rigid: false);
        var samples = WheelRailContactGeometry.Query(model.Geometry, state, settings).Contacts;
        var forces = new WheelRailContactForce[samples.Count];
        var loads = new RigidBodyLoads[state.Bodies.Count];
        for (int i = 0; i < samples.Count; i++)
        {
            WheelRailContactSample sample = samples[i];
            PowerLawNormalContact law = model.Laws[model.Geometry.Pairs[i]];
            double compression = System.Math.Max(0, -sample.Gap);
            if (compression > law.MaximumCompression)
            {
                throw new ContactSolveException(ContactSolveFailure.CompressionLimit, "Compliant contact",
                    "Compression exceeds the owner-supplied physical validity limit (m).",
                    sample.WheelId, sample.PatchId, residual: compression, involvedPairs: model.Geometry.Pairs);
            }
            try
            {
                NormalContactResponse response = law.Evaluate(compression, -sample.NormalVelocity);
                Vector3d force = sample.Normal * response.NormalForce;
                RigidBodyLoads load = RigidBodyLoads.FromForceAtPoint(force, sample.WheelPoint,
                    state.Bodies[sample.BodyIndex].Pose.Position);
                loads[sample.BodyIndex] += load;
                forces[i] = new(sample, response, force, load.Torque);
            }
            catch (Exception error) when (error is ArithmeticException or ArgumentOutOfRangeException)
            {
                throw new ContactSolveException(ContactSolveFailure.NumericalFailure, "Compliant force law",
                    "Force, energy, stiffness, dissipation, or resultant load is not numerically resolved.",
                    sample.WheelId, sample.PatchId, inner: error, involvedPairs: model.Geometry.Pairs);
            }
        }
        try { return new(forces, loads); }
        catch (ArithmeticException error)
        {
            throw new ContactSolveException(ContactSolveFailure.NumericalFailure, "Compliant totals",
                "Energy/power summation exceeded supported arithmetic.", inner: error, involvedPairs: model.Geometry.Pairs);
        }
    }
}
