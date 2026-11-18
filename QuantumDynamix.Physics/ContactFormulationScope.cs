using System;
using System.Linq;
using System.Threading;

namespace QuantumDynamix.Physics;

// Tracks typed contact operations inside synchronous/pure load callbacks. Raw caller-authored
// force vectors carry no provenance; callers must not precompute and disguise duplicate reactions.
internal sealed class ContactFormulationScope : IDisposable
{
    private static readonly AsyncLocal<ContactFormulationScope?> Current = new();
    private readonly ContactFormulationScope? previous;
    private readonly WheelRailContactSystem system;
    private readonly bool rigid;

    internal ContactFormulationScope(WheelRailContactSystem system, bool rigid)
    {
        Check(system, rigid);
        this.system = system;
        this.rigid = rigid;
        previous = Current.Value;
        Current.Value = this;
    }

    internal static void Check(WheelRailContactSystem system, bool rigid)
    {
        for (ContactFormulationScope? scope = Current.Value; scope is not null; scope = scope.previous)
        {
            if (scope.rigid == rigid || !ReferenceEquals(scope.system.Dynamics, system.Dynamics)
                || !ReferenceEquals(scope.system.Rails, system.Rails)) { continue; }
            WheelRailPair? overlap = system.Pairs.FirstOrDefault(p => scope.system.Pairs.Contains(p));
            if (overlap is not null)
            {
                throw new ContactSolveException(ContactSolveFailure.MixedFormulations, "Contact formulations",
                    "The same binding cannot receive rigid enforcement and compliant forces in one operation.",
                    overlap.WheelId, overlap.PatchId, involvedPairs: system.Pairs);
            }
        }
    }

    public void Dispose() => Current.Value = previous;
}
