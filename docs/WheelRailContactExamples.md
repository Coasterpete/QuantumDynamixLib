# Practical wheel/rail contact examples

These examples use the implemented public geometry, rigid-contact, and
compliant-contact APIs. They illustrate normal contact with a stationary planar
rail patch. They do not model a complete vehicle, tangential friction, wheel
spin, suspension, or revolute joints.

All dimensions, masses, stiffnesses, timesteps, and tolerances are synthetic
teaching inputs. They are neither library defaults nor manufacturer properties.
The examples use metres (m), seconds (s), kilograms (kg), newtons (N), newton
seconds (N*s), and joules (J). The fixture chooses +Z as its contact normal; the
library does not impose that axis or an automatic gravity load.

## Running the examples

Use the README's [console-project setup](../README.md#quick-start), which references
the production `QuantumDynamixLib.csproj`. Copy the shared block below into a file
named `ContactExampleSetup.cs` beside the console project's `Program.cs`. Then
copy any one numbered program below, or a contact program from the README, into
`Program.cs`. Run it with:

```powershell
dotnet run --project ../QuantumDynamixDemo/QuantumDynamixDemo.csproj
```

Run each program separately. Their variables and states do not depend on running
an earlier example. Printed numbers below are rounded; tiny numerical residuals
can print as `-0.000` for a nominally zero component. A failed query or step throws
rather than silently reporting separation or a partially advanced state.

## Shared setup

`ContactExampleSetup` is a helper defined entirely in this documentation, not a
library API. It supplies a 2 kg body, a 10 m straight reference line, a 0.2 m
wide open planar rail boundary, and crowned wheels of radius 0.2 m. It contains
no additional contact algorithm. Existing APIs construct and evaluate everything.

<!-- example: contact-example-setup -->
```csharp
using System;
using System.Linq;
using QuantumDynamix.Geometry;
using QuantumDynamix.Math;
using QuantumDynamix.Math.Functions;
using QuantumDynamix.Physics;
using QuantumDynamix.Splines.Curves;
using QuantumDynamix.Track;

public static class ContactExampleSetup
{
    public static WheelDefinition CreateWheel(string id, double offsetX)
    {
        // The tread's local +X axle is rotated onto body +Y.
        var localPose = new RigidPose3d(new Vector3d(offsetX, 0, 0),
            Quaterniond.FromAxisAngle(Vector3d.UnitZ, System.Math.PI / 2));
        return new WheelDefinition(id, bodyIndex: 0, localPose,
            new CrownedWheelTread(radius: 0.2, halfWidth: 0.08, crownRadius: 0.4));
    }

    public static WheelRailContactSystem CreateSystem(params WheelDefinition[] wheels)
    {
        var line = SplineCurve3d.Clamped(new[] {
            Vector3d.Zero, new Vector3d(10, 0, 0) }, degree: 1);
        var track = new HeartlineTrack(line, Vector3d.UnitZ,
            TrackOffsetProfile.Constant(Vector3d.Zero, 10));
        var boundary = SplineCurve3d.Clamped(new[] {
            new Vector3d(0, -0.1, 0), new Vector3d(0, 0.1, 0) }, degree: 1);
        var section = new RailCrossSection(new[] { boundary });
        var roll = new CustomFunction(0, 10, (_, _) => new ScalarJet(0));
        var rail = new RailDefinition("rail",
            TrackOffsetProfile.Constant(Vector3d.Zero, 10), section, roll);
        var rails = new RailGeometry(track, new RailArrangement(new[] { rail }),
            new RailValidationSettings(1e-8, 1e-7, 1e-7, 1e-7, 1e-10, 1e-8));

        var bodies = new MultibodySystem(new[] {
            new RigidBodyMassProperties(2, Matrix3x3d.Diagonal(2, 3, 4)) },
            Array.Empty<SphericalJoint>());
        var patch = new RailContactPatch("top", "rail", boundaryIndex: 0,
            stationMinimum: 0, stationMaximum: 10,
            profileMinimum: 0, profileMaximum: 1,
            RailMaterialSide.PositiveAreaVector);
        if (wheels.Length == 0)
        {
            wheels = new[] { CreateWheel("wheel", offsetX: 0) };
        }
        var pairs = wheels.Select(wheel => new WheelRailPair(wheel.Id, patch.Id));
        return new WheelRailContactSystem(bodies, rails, wheels, new[] { patch }, pairs);
    }

    public static MultibodyState CreateState(double gap, double normalVelocity = 0)
    {
        // Only for this horizontal fixture: wheel center height = radius + gap.
        var pose = new RigidPose3d(new Vector3d(5, 0, 0.2 + gap), Quaterniond.Identity);
        return new MultibodyState(new[] {
            new RigidBodyState(pose, new Vector3d(0, 0, normalVelocity), Vector3d.Zero) });
    }
}
```

The body position is its center of mass; wheel `LocalPose` values are relative
to that frame. The mass is 2 kg and the diagonal inertia entries are 2, 3, and
4 kg*m^2. They are supplied properties, not inferred from wheel dimensions.
`params` lets the helper accept zero, one, or several explicit wheel definitions.
`Select` creates one explicit wheel/patch pair for each wheel; no contact is
inferred from a wheel role or naming convention.

`PositiveAreaVector` chooses +Z for this boundary's parameter orientation.
The boundary is an open surface, not an inferred closed rail solid. Surface
normals and material sides for other geometries require their own declarations.
The rail validation values reproduce existing numerical fixtures. See
[physical rail geometry](PhysicalRailGeometryFoundation.md) for their units,
surface conventions, and inspection limits.

## 1. Detection and signed gaps

Query three separate snapshots: clearance, touching, and nominal overlap.

<!-- example: guide-contact-detection -->
```csharp
using System;
using QuantumDynamix.Physics;

var contacts = ContactExampleSetup.CreateSystem();
foreach (double gap in new[] { 0.01, 0.0, -0.01 })
{
    var state = ContactExampleSetup.CreateState(gap);
    WheelRailContactSample hit = WheelRailContactGeometry.Query(contacts, state).Contacts[0];
    Console.WriteLine($"Gap: {hit.Gap:F6} m");
    Console.WriteLine($"Normal: ({hit.Normal.X:F1}, {hit.Normal.Y:F1}, {hit.Normal.Z:F1})");
    Console.WriteLine($"Wheel point: ({hit.WheelPoint.X:F3}, {hit.WheelPoint.Y:F3}, {hit.WheelPoint.Z:F3}) m");
    Console.WriteLine($"Rail point: ({hit.RailPoint.X:F3}, {hit.RailPoint.Y:F3}, {hit.RailPoint.Z:F3}) m");
}
```

The gaps are 0.01, 0, and -0.01 m. Normals are unit vectors `(0, 0, 1)` and have
no physical length unit. Rail witnesses are `(5, 0, 0)` m. Wheel witnesses are
`(5, 0, 0.01)`, `(5, 0, 0)`, and `(5, 0, -0.01)` m. The wheel center instead
lies at Z=0.21, 0.20, and 0.19 m.

The witnesses satisfy `WheelPoint - RailPoint = Gap * Normal` to search accuracy.
Consequently a negative gap places the nominal wheel surface inside the declared
rail material side. In the compliant formulation that overlap represents local
compression. The rigid integrator requires initially nonpenetrating contact
within its numerical tolerance and would reject this -0.01 m initial overlap.
Geometry queries alone apply neither formulation and do not move the body.

The contact samples are ordered immutable snapshots. `[0]` selects the only
sample here; it is not an automatic selection of the most heavily loaded wheel.

## 2. Rigid impulses, separation, and average force

The following compares two timesteps from the same initially touching incoming
state, followed by an independent separating case.

<!-- example: guide-rigid-contact -->
```csharp
using System;
using QuantumDynamix.Physics;

var contacts = ContactExampleSetup.CreateSystem();
var controls = new ContactSolverSettings(
    maximumPointTravel: 0.05, maximumAngularStep: 0.05,
    maximumPositionCorrection: 0.001, maximumOrientationCorrection: 0.01);
var incoming = ContactExampleSetup.CreateState(gap: 0, normalVelocity: -1);
foreach (double step in new[] { 0.01, 0.005 })
{
    ContactStepResult impact = ContactMultibodyIntegrator.Step(contacts, incoming,
        time: 0, step: step, evaluateLoads: (_, _) => new RigidBodyLoads[1], settings: controls);
    WheelRailContactImpulse contact = impact.ContactImpulses[0];
    Console.WriteLine($"Step: {step:F3} s; impulse: {contact.NormalImpulse:F3} N*s; average: {contact.AverageForce.Z:F3} N");
    Console.WriteLine($"Accepted normal velocity: {impact.AcceptedContacts[0].NormalVelocity:F3} m/s");
}

var leaving = ContactExampleSetup.CreateState(gap: 0, normalVelocity: 0.5);
ContactStepResult separation = ContactMultibodyIntegrator.Step(contacts, leaving,
    time: 0, step: 0.01, evaluateLoads: (_, _) => new RigidBodyLoads[1], settings: controls);
Console.WriteLine($"Separating impulse: {separation.ContactImpulses[0].NormalImpulse:F3} N*s");
Console.WriteLine($"Separated gap: {separation.AcceptedContacts[0].Gap:F6} m");
```

Expected numerical results are:

| Case | Normal impulse | Step-average force along +Z | Accepted normal velocity/gap |
| --- | --- | --- | --- |
| Incoming, h=0.01 s | 2 N*s | 200 N | Velocity 0 m/s |
| Incoming, h=0.005 s | 2 N*s | 400 N | Velocity 0 m/s |
| Leaving, h=0.01 s | 0 N*s | 0 N | Gap 0.005 m |

For the central impact, `impulse = mass * change_in_normal_velocity` gives
`2 kg * (0 - (-1)) m/s = 2 N*s`. Impulse is force integrated over time. The rigid
model changes velocity without resolving the real deformation history or impact
duration. `AverageForce = WorldImpulse / step` spreads that impulse over the
chosen numerical step. Halving the step therefore doubles the average in this
case, without establishing a physical peak force.

Positive normal velocity is separating. A 0.5 m/s departure over 0.01 s opens
0.005 m clearance with no attractive impulse. Each case starts from its own
immutable snapshot; the loop does not advance one impact twice.

`(_, _) => new RigidBodyLoads[1]` is a callback that ignores its time and state
arguments and returns one zero-valued force/torque for this one-body fixture.
The four solver bounds are numerical travel (m), rotation (rad), position
correction (m), and orientation correction (rad). They are not material properties.
Physical impulses use start-configuration witnesses; `AcceptedContacts` uses
the accepted endpoint. Numerical corrections are separately available in
`NumericalCorrections` and do not contribute to the reported physical impulse.

The current rigid method is first order and uses gap/step to approximate impact
timing. Initially positive clearance can produce a predictive impulse, and a
newly closed contact can remain closing until the next step. The exact stopping
calculation above applies to the stated initially touching central case. See
[the rigid foundation](WheelRailContactFoundation.md) for those limits.

## 3. Compliant force and elastic energy

Supply a linear stiffness, assemble a compressed state, and balance its force
with an explicit external load. Then query an independent state with clearance.

<!-- example: guide-compliant-contact -->
```csharp
using System;
using System.Collections.Generic;
using QuantumDynamix.Math;
using QuantumDynamix.Physics;

var contacts = ContactExampleSetup.CreateSystem();
var law = new PowerLawNormalContact(
    coefficient: 2000, exponent: 1, dissipation: 0, maximumCompression: 0.02);
var laws = new Dictionary<WheelRailPair, PowerLawNormalContact>
{
    [contacts.Pairs[0]] = law
};
var model = new CompliantWheelRailContactModel(contacts, laws);
var state = ContactExampleSetup.CreateState(gap: -0.01);
WheelRailContactSample hit = WheelRailContactGeometry.Query(contacts, state).Contacts[0];
double compression = System.Math.Max(0.0, -hit.Gap);
CompliantContactEvaluation initial = CompliantWheelRailContact.Evaluate(model, state);
Console.WriteLine($"Compression from gap: {compression:F6} m");
Console.WriteLine($"Initial force: {initial.Contacts[0].Response.NormalForce:F3} N");

var controls = new CompliantContactStepSettings(0.05, 0.05, 0.001, 0.01);
CompliantContactStepResult result = CompliantContactMultibodyIntegrator.Step(model, state,
    time: 0, step: 0.001,
    evaluateLoads: (_, _) => new[] {
        new RigidBodyLoads(new Vector3d(0, 0, -20), Vector3d.Zero) }, settings: controls);
NormalContactResponse response = result.AcceptedContacts.Contacts[0].Response;
Console.WriteLine($"Accepted compression: {response.Compression:F6} m");
Console.WriteLine($"Accepted force: {response.NormalForce:F3} N");
Console.WriteLine($"Stored energy: {response.ElasticEnergy:F6} J");

var clearState = ContactExampleSetup.CreateState(gap: 0.005);
NormalContactResponse clear = CompliantWheelRailContact.Evaluate(model, clearState).Contacts[0].Response;
Console.WriteLine($"Force with clearance: {clear.NormalForce:F3} N");
```

`System.Math.Max(0, -gap)` turns -0.01 m overlap into +0.01 m compression and
returns zero at positive clearance. It does not make negative force possible.
For this linear, zero-dissipation law:

```text
F = k*delta = 2000 N/m * 0.01 m = 20 N
U = 0.5*k*delta^2 = 0.1 J
```

The output retains 0.01 m compression, 20 N accepted normal force, and 0.1 J
stored energy. The clear state gives zero force. `Evaluate` samples the model
without stepping; `Step` updates the coupled system. The dictionary associates
a law with the explicit geometry pair, rather than assigning an implicit wheel
material. Its key must match a bound `WheelRailPair`.

The general implemented law is `Fn = K*delta^p*max(0, 1 + alpha*deltaDot)` for
positive compression, and zero force otherwise. K has units N/m^p, exponent p is
dimensionless and at least one, alpha has units s/m, and `deltaDot=-NormalVelocity`
is positive during compression. The `max` clips unloading force at zero; contact
never pulls the wheel toward the rail. Stored elastic energy is
`K*delta^(p+1)/(p+1)` J. The examples explicitly choose p=1 and alpha=0 to make
the arithmetic easy; those values do not specify a real wheel material.

`MaximumCompression` is the physical law's allowed range. Exceeding it throws;
it does not cap force or change stiffness. Contact compression is retained while
joint projection can still change energy numerically. The wrapper's `Energy`
diagnostics distinguish physical dissipation estimates from projection-induced
kinetic/elastic changes. Timestep and geometry refinement remain necessary.
See [the compliant foundation](CompliantWheelRailContactFoundation.md) for the
full numerical and material limitations, including the limits of Hertz theory.

## 4. Three-support load sharing

This is the documented analytical three-support case: one body, three collinear
wheels at X offsets -0.5, 0, and +0.5 m, equal undeformed clearances, and a
central downward load of 4000 N. Equal outside stiffnesses and symmetry let the
body settle without tilting. Force balance gives:

```text
Fleft + Fmiddle + Fright = W
Fi = ki*delta
delta = W/(kleft + kmiddle + kright)
```

There is no generic static-equilibrium assembler. The example supplies the
analytically assembled initial state, then verifies it by stepping the production
dynamics under the balanced load. Each iteration constructs an independent law
assignment and equilibrium snapshot; it does not change a material during motion.

<!-- example: guide-three-supports -->
```csharp
using System;
using System.Collections.Generic;
using QuantumDynamix.Math;
using QuantumDynamix.Physics;

var contacts = ContactExampleSetup.CreateSystem(
    ContactExampleSetup.CreateWheel("a", offsetX: -0.5),
    ContactExampleSetup.CreateWheel("b", offsetX: 0),
    ContactExampleSetup.CreateWheel("c", offsetX: 0.5));
double load = 4000; // N, applied downward through the COM
var controls = new CompliantContactStepSettings(0.05, 0.05, 0.001, 0.01);

foreach (double middleStiffness in new[] { 1e6, 2e6 })
{
    double[] stiffnesses = { 1e6, middleStiffness, 1e6 }; // N/m
    double compression = load / (stiffnesses[0] + stiffnesses[1] + stiffnesses[2]);
    var laws = new Dictionary<WheelRailPair, PowerLawNormalContact>();
    for (int i = 0; i < contacts.Pairs.Count; i++)
    {
        laws.Add(contacts.Pairs[i], new PowerLawNormalContact(
            stiffnesses[i], exponent: 1, dissipation: 0, maximumCompression: 0.02));
    }
    var model = new CompliantWheelRailContactModel(contacts, laws);
    var state = ContactExampleSetup.CreateState(gap: -compression);
    CompliantContactStepResult result = CompliantContactMultibodyIntegrator.Step(model, state,
        time: 0, step: 0.0001,
        evaluateLoads: (_, _) => new[] {
            new RigidBodyLoads(new Vector3d(0, 0, -load), Vector3d.Zero) }, settings: controls);
    Console.WriteLine($"Middle stiffness: {middleStiffness:F0} N/m; compression: {compression:F9} m");
    foreach (WheelRailContactForce wheel in result.AcceptedContacts.Contacts)
    {
        Console.WriteLine($"{wheel.Geometry.WheelId}: {wheel.Response.NormalForce:F3} N");
    }
    Console.WriteLine($"Resultant support: {result.AcceptedContacts.BodyLoads[0].Force.Z:F3} N");
}
```

Pairs are ordered by wheel ID, so `a`, `b`, and `c` correspond to left, middle,
and right in this explicitly named example. A general application should map
laws by its own explicit pair identities.

| Stiffnesses (N/m) | Compression (m) | Loads a/b/c (N) |
| --- | --- | --- |
| (1, 1, 1)*10^6 | About 0.001333333 | About 1333.333 / 1333.333 / 1333.333 |
| (1, 2, 1)*10^6 | 0.001 | 1000 / 2000 / 1000 |

Both resultants are 4000 N. Doubling the middle stiffness makes it carry twice
the load of either outside wheel at the new equilibrium. It also reduces the
common compression. Keeping the old compressed state instead would create an
unbalanced transient; equilibrium would not be preserved automatically.

This equal-compression equation depends on the stated geometry and moment
balance. Different gaps, unequal outside stiffnesses, tilted normals, applied
moments, or articulated attachments require their own compatibility/dynamics.
The rigid three-support equivalent can raise `IndeterminateReactions`: rigid
equilibrium alone does not choose the individual load split.

## 5. Rigid versus compliant output quantities

The following starts both formulations independently from the same touching,
incoming geometry. It illustrates units and sampling, not equivalence of their
physical impact assumptions.

<!-- example: guide-contact-comparison -->
```csharp
using System;
using System.Collections.Generic;
using QuantumDynamix.Physics;

var contacts = ContactExampleSetup.CreateSystem();
var incoming = ContactExampleSetup.CreateState(gap: 0, normalVelocity: -1);
double step = 0.01;
var rigidControls = new ContactSolverSettings(0.05, 0.05, 0.001, 0.01);
ContactStepResult rigid = ContactMultibodyIntegrator.Step(contacts, incoming, 0, step,
    (_, _) => new RigidBodyLoads[1], rigidControls);

var law = new PowerLawNormalContact(2000, 1, 0, 0.02);
var model = new CompliantWheelRailContactModel(contacts,
    new Dictionary<WheelRailPair, PowerLawNormalContact> { [contacts.Pairs[0]] = law });
var compliantControls = new CompliantContactStepSettings(0.05, 0.05, 0.001, 0.01);
CompliantContactStepResult compliant = CompliantContactMultibodyIntegrator.Step(model, incoming, 0, step,
    (_, _) => new RigidBodyLoads[1], compliantControls);

Console.WriteLine($"Rigid impulse: {rigid.ContactImpulses[0].NormalImpulse:F3} N*s");
Console.WriteLine($"Rigid average force: {rigid.ContactImpulses[0].AverageForce.Z:F3} N");
Console.WriteLine($"Compliant initial force: {compliant.InitialContacts.Contacts[0].Response.NormalForce:F3} N");
Console.WriteLine($"Compliant endpoint force: {compliant.AcceptedContacts.Contacts[0].Response.NormalForce:F3} N");
Console.WriteLine($"Compliant endpoint compression: {compliant.AcceptedContacts.Contacts[0].Response.Compression:F6} m");
```

The rigid result is 2 N*s and a 200 N step average. The compliant initial force
is zero at first touch; after this one step its endpoint force is about 19.512 N
at about 0.009756 m compression. That endpoint force is neither a step average
nor an impact peak. The compliant wheel is still decelerating while storing
elastic energy. With alpha=0 it can later return that energy; the rigid impact
is inelastic. A different stiffness, damping law, or timestep changes this
comparison. Neither set of fixture values is a calibrated real wheel.

Use the rigid formulation when negligible deformation and its discrete inelastic
impact approximation fit the intended question and reaction loads are resolved.
Use compliant contact when deformation/load sharing is part of the intended
model and owner-supplied coefficients have a physical basis. Both retain the
existing articulated-body architecture; neither prescribes spline-following
motion. Local compliance does not guarantee a unique stable equilibrium in every
assembly.

The program runs two separate alternatives. Never feed compliant contact forces
into rigid enforcement of the same physical contact. External load callbacks must
also exclude contact loads already supplied by the selected stepping path.

## Verification

The new README programs and all five numbered programs in this guide are
extracted from their Markdown code blocks, compiled as separate .NET 10 console
programs with the shared setup, and run against the production library. Numerical
output is checked against the stated gaps, impulses, forces, energy, and support
loads. Existing complete README C# examples are also compiled and run.
The engineering foundation documents retain the broader regression evidence.

This documentation update verified 16 independent console programs in Debug:
four new README programs, five numbered programs here, the five existing README
programs, and the complete rigid/compliant foundation examples. There were 52
checks of printed numerical results, treating a rounded negative zero as zero.
The shared setup was compiled with every program that uses it. No production or
test source changes were needed for this verification.

| Final check | Result |
| --- | --- |
| Complete Debug test suite | 272 passed, 0 failed, 0 skipped |
| Complete Release test suite | 272 passed, 0 failed, 0 skipped |
| `dotnet format QuantumDynamixLib.sln --verify-no-changes --no-restore` | Passed |
| Final Markdown code versus compiled examples | All 16 program blocks matched |
| Local documentation links and heading anchors | 24 checked |

The full test commands were `dotnet test QuantumDynamixLib.sln --configuration Debug --no-restore`
and the corresponding `--configuration Release` command. These built and tested
the production library and its MSTest project. Console-program verification used
temporary projects outside the repository; no example project or helper API was
added to the library or solution.
