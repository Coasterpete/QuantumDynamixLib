# Practical revolute-joint and wheel-spin examples

These programs use the implemented public APIs. Start with the README's
[body-local attachment frames](../README.md#body-local-revolute-joint-frames) and
[independent wheel body](../README.md#independent-wheel-spin). This guide expands
them into complete programs for angle history, paired shaft torques, bearing
reactions, normal contact, and timestep refinement. The
[engineering foundation](RevoluteJointFoundation.md) gives the analytic joint
equations and regression evidence.

All masses, inertia tensors, dimensions, loads, and numerical controls are
synthetic teaching inputs. They are not manufacturer dimensions, real-wheel
calibration, or engineering acceptance criteria. The examples use metres,
kilograms, seconds, radians, N, N*m, and kg*m^2 consistently. Friction, traction,
braking mechanisms, suspension, and automatic no-slip rolling are not implemented.

## Running the programs

Use the README's [console-project setup](../README.md#quick-start), which references
`QuantumDynamixLib.csproj`. Save the shared block below as `RevoluteExampleSetup.cs`
beside your demo's `Program.cs`, then copy any one numbered program into
`Program.cs`. Run each program independently from the repository root:

```powershell
dotnet run --project ../QuantumDynamixDemo/QuantumDynamixDemo.csproj
```

The helper belongs to this documentation, not the production library. It only
constructs definitions/states and supplies explicit loads using public APIs.
Each program starts from fresh definitions and snapshots. A failed solve throws;
accept a state only after `Step` returns successfully.

## Shared setup

Body 0 is the 2 kg carrier and body 1 the separate 0.5 kg wheel. Their centers of mass (COMs) are
coincident in this fixture to keep bearing-load arithmetic simple. The README
also demonstrates nonzero attachment offsets. These are finite bodies; neither
is fixed to the world. Carrier mass and inertia exclude the wheel.

<!-- example: revolute-example-setup -->
```csharp
using System;
using System.Collections.Generic;
using QuantumDynamix.Geometry;
using QuantumDynamix.Math;
using QuantumDynamix.Math.Functions;
using QuantumDynamix.Physics;
using QuantumDynamix.Splines.Curves;
using QuantumDynamix.Track;

public static class RevoluteExampleSetup
{
    public static Quaterniond AxleAlongY =>
        Quaterniond.FromAxisAngle(Vector3d.UnitZ, System.Math.PI / 2);

    public static MultibodySystem CreateSystem() => new(
        new[]
        {
            new RigidBodyMassProperties(2, Matrix3x3d.Diagonal(2, 2, 2)),
            new RigidBodyMassProperties(0.5, Matrix3x3d.Diagonal(0.02, 0.03, 0.03))
        },
        Array.Empty<SphericalJoint>(),
        new[]
        {
            new RevoluteJoint(0, 1,
                new RigidPose3d(Vector3d.Zero, AxleAlongY), RigidPose3d.Identity)
        });

    public static MultibodyState CreateState(double spin = 5, double speed = 10,
        double phase = 0, double gap = 0)
    {
        var center = new Vector3d(5.1, 0, 0.2 + gap);
        var velocity = new Vector3d(speed, 0, 0);
        var wheelOrientation = AxleAlongY * Quaterniond.FromAxisAngle(Vector3d.UnitX, phase);
        return new MultibodyState(new[]
        {
            new RigidBodyState(new RigidPose3d(center, Quaterniond.Identity), velocity, Vector3d.Zero),
            new RigidBodyState(new RigidPose3d(center, wheelOrientation), velocity, new Vector3d(spin, 0, 0))
        });
    }

    public static IReadOnlyList<RigidBodyLoads> NoLoads(double time, MultibodyState state) =>
        new RigidBodyLoads[state.Bodies.Count];

    public static IReadOnlyList<RigidBodyLoads> DownwardLoads(double time, MultibodyState state) =>
        new[]
        {
            new RigidBodyLoads(new Vector3d(0, 0, -20), Vector3d.Zero),
            new RigidBodyLoads(new Vector3d(0, 0, -5), Vector3d.Zero)
        };

    public static WheelRailContactSystem CreateContacts(MultibodySystem dynamics)
    {
        var line = SplineCurve3d.Clamped(new[]
        {
            Vector3d.Zero, new Vector3d(10, 0, 0)
        }, degree: 1);
        var track = new HeartlineTrack(line, Vector3d.UnitZ,
            TrackOffsetProfile.Constant(Vector3d.Zero, 10));
        var boundary = SplineCurve3d.Clamped(new[]
        {
            new Vector3d(0, -0.1, 0), new Vector3d(0, 0.1, 0)
        }, degree: 1);
        var section = new RailCrossSection(new[] { boundary });
        var roll = new CustomFunction(0, 10, (_, _) => new ScalarJet(0));
        var rail = new RailDefinition("rail",
            TrackOffsetProfile.Constant(Vector3d.Zero, 10), section, roll);
        var rails = new RailGeometry(track, new RailArrangement(new[] { rail }),
            new RailValidationSettings(1e-8, 1e-7, 1e-7, 1e-7, 1e-10, 1e-8));
        var wheel = new WheelDefinition("wheel", 1, RigidPose3d.Identity,
            new CrownedWheelTread(0.2, 0.08, 0.4));
        var patch = new RailContactPatch("top", "rail", 0, 0, 10, 0, 1,
            RailMaterialSide.PositiveAreaVector);
        return new WheelRailContactSystem(dynamics, rails, new[] { wheel },
            new[] { patch }, new[] { new WheelRailPair("wheel", "top") });
    }
}
```

The carrier's attachment +X points along carrier +Y. The wheel's attachment frame
is identity, so its axle is wheel-body +X. Rotating the wheel body into the world
aligns both axles with world +Y. Right-multiplying by the local +X phase rotation
then spins the wheel around its own axle. Both attachment origins and their
velocities agree, satisfying the initial joint constraints.

`Matrix3x3d.Diagonal` supplies a full tensor with zero off-diagonal entries for
these principal-axis examples. The wheel's axial COM inertia is 0.02 kg*m^2;
its other two entries are 0.03 kg*m^2. Inertia measures resistance to angular
acceleration and depends on mass distribution. It is supplied explicitly rather
than calculated from the tread radius. General symmetric positive-definite tensors
are supported. Splitting an existing combined body also requires recalculating
the carrier's COM and inertia consistently; the library cannot detect physical
double-counting in caller-supplied properties.

`CreateContacts` adds a single crowned tread on the wheel body and a stationary
planar rail patch with +Z normal. The material-side declaration follows that
boundary's parameter orientation. This local open surface does not define a
closed rail solid or complete vehicle. See the existing
[contact guide](WheelRailContactExamples.md) for its geometry/search restrictions.
The position `radius + gap` is a convenience of this horizontal fixture only.

## 1. Wrapped angle and accumulated revolutions

One full revolution is 2*pi radians. Angular velocity is the rate of angle change,
in rad/s. The wheel below starts with a 0.3 rad phase and spins at 5 rad/s relative
to its stationary-in-rotation carrier. Linear translation does not set that spin.

<!-- example: guide-revolute-history -->
```csharp
using System;
using QuantumDynamix.Physics;

var system = RevoluteExampleSetup.CreateSystem();
var state = RevoluteExampleSetup.CreateState(spin: 5, speed: 10, phase: 0.3);
RevoluteJointSample initial = RevoluteJointKinematics.Evaluate(system, state, 0);
double initialAngle = initial.Angle;
double accumulatedAngle = initialAngle;
double h = 0.005;

for (int i = 0; i < 400; i++)
{
    ConstrainedStepResult result = ConstrainedMultibodyIntegrator.Step(
        system, state, i * h, h, RevoluteExampleSetup.NoLoads);
    state = result.State;
    RevoluteJointSample current = RevoluteJointKinematics.Evaluate(system, state, 0);
    accumulatedAngle = RevoluteJointKinematics.UnwrapAngle(accumulatedAngle, current.Angle);
}

RevoluteJointSample final = RevoluteJointKinematics.Evaluate(system, state, 0);
double revolutions = (accumulatedAngle - initialAngle) / (2 * System.Math.PI);
Console.WriteLine($"Wrapped angle: {final.Angle:F6} rad");
Console.WriteLine($"Accumulated change: {accumulatedAngle - initialAngle:F6} rad");
Console.WriteLine($"Revolutions since start: {revolutions:F6}");
Console.WriteLine($"Relative angular velocity: {final.AngularRate:F6} rad/s");
Console.WriteLine($"World axle Y: {final.WorldAxis.Y:F6}");
```

After 2 s, the accumulated change is about 10 rad, or 1.59153 revolutions.
The wrapped angle is about -2.26650 rad, while the rate remains 5 rad/s. The
initial 0.3 rad phase is subtracted when counting revolutions *since start*.

`Angle` is in (-pi,pi]; an orientation snapshot cannot tell how many complete
turns preceded it. `UnwrapAngle` chooses the closest branch to the previous
accumulated value. The application owns and initializes that history. Only
accepted states update it, and the true relative phase change between samples
must be strictly below pi. Endpoints cannot reveal missed full turns. The helper
rejects an exactly ambiguous half turn; very large accumulated doubles also lose
phase precision. Longer-range counters may need a separate integer turn count.

The load callback is pure and contains no angle accumulation. Newton can call it
many times at one time with different trial states; accumulating there would
count solver trials as motion. History records the accepted numerical phase,
which still has timestep error. The rate is
`dot(worldAxle, omegaB_world - omegaA_world)`, not a subtraction of different
bodies' local angular components.

## 2. Applying an axle torque

An internal shaft action applies +T to the wheel and -T to the carrier about the
same world axle. A finite carrier responds to that torque. This program supplies
constant torques through the existing load callback; it is not a motor or brake
mechanism implementation.

<!-- example: guide-revolute-torque -->
```csharp
using System;
using System.Collections.Generic;
using QuantumDynamix.Math;
using QuantumDynamix.Physics;

var system = RevoluteExampleSetup.CreateSystem();
var state = RevoluteExampleSetup.CreateState(spin: 0, speed: 0);
double torque = 0.1; // N*m on the wheel
double h = 0.001;

IReadOnlyList<RigidBodyLoads> Loads(double time, MultibodyState snapshot)
{
    Vector3d axis = RevoluteJointKinematics.Evaluate(system, snapshot, 0).WorldAxis;
    return new[]
    {
        new RigidBodyLoads(Vector3d.Zero, -torque * axis),
        new RigidBodyLoads(Vector3d.Zero, torque * axis)
    };
}

for (int i = 0; i < 100; i++)
{
    state = ConstrainedMultibodyIntegrator.Step(system, state, i * h, h, Loads).State;
}

RevoluteJointSample sample = RevoluteJointKinematics.Evaluate(system, state, 0);
double carrierRate = Vector3d.Dot(sample.WorldAxis, state.Bodies[0].WorldAngularVelocity);
double wheelRate = Vector3d.Dot(sample.WorldAxis, state.Bodies[1].WorldAngularVelocity);
Console.WriteLine($"Carrier absolute axle rate: {carrierRate:F6} rad/s");
Console.WriteLine($"Wheel absolute axle rate: {wheelRate:F6} rad/s");
Console.WriteLine($"Relative axle rate: {sample.AngularRate:F6} rad/s");
```

For these centered principal-axis motions, `alpha = torque / inertia` gives:

| Quantity after 0.1 s from rest | Value |
| --- | --- |
| Carrier absolute axle rate | -0.005 rad/s, from -0.1 / 2 rad/s^2 |
| Wheel absolute axle rate | +0.5 rad/s, from +0.1 / 0.02 rad/s^2 |
| Relative axle rate | 0.5 - (-0.005) = 0.505 rad/s |

The two world torques sum to zero. Angular momentum transfers between the bodies;
the prescribed action can still supply energy at power `T * AngularRate` in W.
The ideal bearing contributes no independent resisting couple along the free
axle. Off-COM attachments, general inertia, and changing axle directions require
the complete coupled equations; the simple scalar acceleration equation is not
a replacement for that solver.

## 3. Bearing forces and gyroscopic couples

This instantaneous example adds a 0.7 rad/s world +Z turning motion to both bodies
while the wheel spins at 80 rad/s about +Y relative to the carrier. The wheel's
world angular velocity is therefore `(0, 80, 0.7)` rad/s. Convert it into its
body frame before constructing `RigidBodyState`.

<!-- example: guide-revolute-reactions -->
```csharp
using System;
using QuantumDynamix.Math;
using QuantumDynamix.Physics;

var system = RevoluteExampleSetup.CreateSystem();
var initial = RevoluteExampleSetup.CreateState(spin: 80, speed: 0);
RigidBodyState carrier = initial.Bodies[0];
RigidBodyState wheel = initial.Bodies[1];
var turning = new Vector3d(0, 0, 0.7);
var wheelWorldOmega = new Vector3d(0, 80, 0.7);
var state = new MultibodyState(new[]
{
    new RigidBodyState(carrier.Pose, carrier.LinearVelocity,
        carrier.Pose.InverseTransformDirection(turning)),
    new RigidBodyState(wheel.Pose, wheel.LinearVelocity,
        wheel.Pose.InverseTransformDirection(wheelWorldOmega))
});
var loads = new[]
{
    new RigidBodyLoads(new Vector3d(0, 0, -10), Vector3d.Zero),
    new RigidBodyLoads(Vector3d.Zero, Vector3d.Zero)
};

ConstrainedDynamicsResult evaluation = ConstrainedMultibodyDynamics.Evaluate(system, state, loads);
RevoluteJointReaction bearing = evaluation.RevoluteJointReactions[0];
Console.WriteLine($"Force on A, Z: {bearing.ForceOnA.Z:F6} N");
Console.WriteLine($"Force on B, Z: {bearing.ForceOnB.Z:F6} N");
Console.WriteLine($"Couple on A, X: {bearing.CoupleOnA.X:F6} N*m");
Console.WriteLine($"Couple on B, X: {bearing.CoupleOnB.X:F6} N*m");
Console.WriteLine($"COM torque on A, X: {bearing.TorqueOnA.X:F6} N*m");
Console.WriteLine($"COM torque on B, X: {bearing.TorqueOnB.X:F6} N*m");
```

The -10 N load gives common vertical acceleration -10/2.5 = -4 m/s^2.
The bearing exerts +2 N on A and -2 N on B along Z, so each body has the proper
resultant for its mass. Turning the spinning wheel also produces about
+1.103448 N*m on A and -1.103448 N*m on B along X. These are finite-carrier
gyroscopic reactions, not an extra torque supplied by the program.

| Property | Units and meaning |
| --- | --- |
| `ForceOnA`, `ForceOnB` | World bearing forces, N; opposite signs |
| `CoupleOnA`, `CoupleOnB` | Independent world bearing couples, N*m; opposite signs |
| `TorqueOnA`, `TorqueOnB` | World total torque about each body's own COM, N*m |

For an attachment arm r from a body's COM, `COM torque = r cross force + couple`.
The centered fixture has r=0, so its total torque equals its independent couple.
For offset attachments those quantities can differ, and the two COM torques
need not be negatives of each other. The ideal independent couple is transverse
to the common axle; an offset force can still have an axial COM moment.

`Evaluate` reports instantaneous reactions and derivatives at the supplied state;
it does not advance time. `ConstrainedStepResult.AcceptedDynamics` similarly
reports instantaneous endpoint reactions under endpoint loads. Existing
`JointReactions` remains spherical-only; `RevoluteJointReactions` uses the
revolute definition order.

Spin angular momentum about the wheel's COM is `H = R * I_body * omega_body`. Changing its direction requires
torque, so a turning carrier and spinning wheel affect one another. The existing
Newton-Euler equation `I*omegaDot = torque - omega cross (I*omega)` accounts for
this in body coordinates, with torque transformed from world coordinates. Adding
another hand-written gyroscopic torque would count the same effect twice.

## 4. Rigid and compliant normal contact

Use the same independently spinning wheel body in either contact formulation.
The rigid case starts just touching. The compliant case starts at the explicitly
assembled compression 0.01 m, balancing its chosen 2500 N/m linear stiffness
against 25 N total downward load. They are independent demonstrations, with
different physical contact assumptions.

<!-- example: guide-revolute-contact -->
```csharp
using System;
using System.Collections.Generic;
using QuantumDynamix.Math;
using QuantumDynamix.Physics;

var system = RevoluteExampleSetup.CreateSystem();
var contacts = RevoluteExampleSetup.CreateContacts(system);
double h = 0.001;
var rigidControls = new ContactSolverSettings(0.05, 0.05, 0.001, 0.01);
ContactStepResult rigid = ContactMultibodyIntegrator.Step(contacts,
    RevoluteExampleSetup.CreateState(spin: 5, speed: 10, gap: 0),
    0, h, RevoluteExampleSetup.DownwardLoads, rigidControls);

RevoluteJointImpulse rigidBearing = rigid.RevoluteJointImpulses[0];
Console.WriteLine($"Rigid normal impulse: {rigid.ContactImpulses[0].NormalImpulse:F6} N*s");
Console.WriteLine($"Rigid average support: {rigid.ContactImpulses[0].AverageForce.Z:F6} N");
Console.WriteLine($"Rigid bearing force on A, average Z: {rigidBearing.ImpulseOnA.Z / h:F6} N");
Console.WriteLine($"Rigid bearing couple on A, average length: {rigidBearing.CoupleImpulseOnA.Length / h:F6} N*m");

var law = new PowerLawNormalContact(2500, 1, 0, maximumCompression: 0.04);
var model = new CompliantWheelRailContactModel(contacts,
    new Dictionary<WheelRailPair, PowerLawNormalContact> { [contacts.Pairs[0]] = law });
var compliantControls = new CompliantContactStepSettings(0.05, 0.05, 0.001, 0.01);
CompliantContactStepResult compliant = CompliantContactMultibodyIntegrator.Step(model,
    RevoluteExampleSetup.CreateState(spin: 5, speed: 10, gap: -0.01),
    0, h, RevoluteExampleSetup.DownwardLoads, compliantControls);

Console.WriteLine($"Compliant instantaneous support: {compliant.AcceptedContacts.Contacts[0].Response.NormalForce:F6} N");
Console.WriteLine($"Compliant bearing force on A, endpoint Z: {compliant.JointStep.AcceptedDynamics.RevoluteJointReactions[0].ForceOnA.Z:F6} N");
Console.WriteLine($"Compliant compression: {compliant.AcceptedContacts.Contacts[0].Response.Compression:F6} m");
Console.WriteLine($"Compliant stored energy: {compliant.AcceptedContacts.ElasticEnergy:F6} J");

RevoluteJointSample rigidSpin = RevoluteJointKinematics.Evaluate(system, rigid.State, 0);
RevoluteJointSample compliantSpin = RevoluteJointKinematics.Evaluate(system, compliant.State, 0);
WheelRailContactSample point = rigid.AcceptedContacts[0];
Vector3d slip = point.RelativeVelocity - point.Normal * Vector3d.Dot(point.Normal, point.RelativeVelocity);
Console.WriteLine($"Rigid relative spin: {rigidSpin.AngularRate:F6} rad/s");
Console.WriteLine($"Compliant relative spin: {compliantSpin.AngularRate:F6} rad/s");
Console.WriteLine($"Rigid tangential slip X: {slip.X:F6} m/s");
```

Expected results, to the displayed precision apart from possible signed zeros:

| Quantity | Value |
| --- | --- |
| Rigid normal impulse / step-average support | 0.025 N*s / 25 N |
| Rigid bearing force on carrier, step average | +20 N along Z |
| Independent rigid bearing couple magnitude, step average | Approximately 0 N*m |
| Compliant instantaneous support / endpoint bearing force on carrier | 25 N / +20 N along Z |
| Compliant compression / stored elastic energy | 0.01 m / 0.125 J |
| Relative spin in either formulation | 5 rad/s |
| Tangential contact-point velocity along X | 9 m/s |

The carrier receives its 20 N through the bearing. The wheel receives 25 N from
the rail and -20 N from the bearing, balancing its own 5 N downward load. Body
mass is counted once in `CreateSystem`; geometry adds no mass. The compliant
overlap represents physical deformation and remains in the accepted state.

For the stationary rail, `v_point = v_COM + omega_world cross r`, with r from
wheel COM to the wheel witness. Here `10 - 0.2 * 5 = 9 m/s` is tangential slip.
Only zero slip would imply `omega = v/r = 50 rad/s` for this simple upright
circular configuration. The normal-only laws supply no tangential force to spin
the wheel up to that value. Other axle orientations and crowned contact locations
require the vector equation and the wheel's complete absolute angular velocity.

The rigid solver couples normal and joint impulses and reports them at the start
configuration. `ImpulseOnA/B` is in N*s; `CoupleImpulseOnA/B` and COM
`AngularImpulseOnA/B` are in N*m*s. Division by h gives step-average N and N*m,
not instantaneous or peak impact loads. The compliant wrapper supplies normal
forces to the joint solver and reports endpoint bearing reactions through
`JointStep.AcceptedDynamics`. Numerical corrections and projection energy changes
remain separate from physical forces and dissipation.

The program selects each formulation in a separate operation. Never add
compliant forces to rigid enforcement of the same contact, and exclude contact
loads already supplied by the wrapper from external callbacks. Declared smooth
patches, bounded travel, and resolved geometry are still required. Rotating an
axisymmetric tread's phase with equivalent world configuration and velocities
does not change this geometry or contact-point velocity field; it changes the
material-point identity at that location.

## 5. Refining the timestep for fast spin

Fast wheel spin can accumulate angular phase error even while the joint's
position and angular-velocity residuals remain small. The following compares
three steps over the same 0.08 s with the exact constant-spin change
`500 * 0.08 = 40 rad`. It starts fresh for every refinement.

<!-- example: guide-revolute-refinement -->
```csharp
using System;
using QuantumDynamix.Physics;

var system = RevoluteExampleSetup.CreateSystem();
double duration = 0.08;
double spin = 500;
foreach (int count in new[] { 100, 200, 400 })
{
    var state = RevoluteExampleSetup.CreateState(spin: spin, speed: 0, phase: 0.3);
    double initialAngle = RevoluteJointKinematics.Evaluate(system, state, 0).Angle;
    double angle = initialAngle;
    double h = duration / count;
    double maximumAlignment = 0;
    for (int i = 0; i < count; i++)
    {
        ConstrainedStepResult result = ConstrainedMultibodyIntegrator.Step(
            system, state, i * h, h, RevoluteExampleSetup.NoLoads);
        state = result.State;
        angle = RevoluteJointKinematics.UnwrapAngle(angle,
            RevoluteJointKinematics.Evaluate(system, state, 0).Angle);
        maximumAlignment = System.Math.Max(maximumAlignment,
            result.AcceptedDynamics.Diagnostics.MaximumAngularPositionError);
    }
    double error = System.Math.Abs(angle - initialAngle - spin * duration);
    Console.WriteLine($"h={h:F4} s; phase error={error:E6} rad; max alignment={maximumAlignment:E6} rad");
}
```

The phase errors are approximately 0.132539, 0.0332834, and 0.00833021 rad: about
fourfold reduction when h is halved. The constant-spin midpoint update advances
`4 * atan(h * omega / 4)` radians per step, approaching `h * omega` as h decreases.
This explains why a stable, well-constrained step can still have appreciable phase
error. Tightening angular projection tolerances alone does not remove it.

For general motion, compare h, h/2, and h/4 at a common duration, examining wheel
phase, angular/linear velocities, individual bearing/contact loads, and energy.
Also resolve contact search and compression timescales where applicable. These
midpoint examples are second order on smooth resolved branches; the rigid
contact integrator remains first order. Projection can change energy/momentum.
Both contact wrappers count axle spin in their angular/point travel guards, and
the supplied bounds do not perform automatic step refinement. The history
helper's less-than-pi phase requirement is a separate branch-resolution condition,
not an accuracy criterion. Numerical defaults and the timesteps in these examples
are not validated operating limits for a coaster.

## Verification

The verification extracts the Markdown C# blocks, compiles complete programs
with their declared shared helper against the production project, and checks
their output against the equations and values above. Existing README/contact
examples are checked alongside the new programs. Temporary console projects
stay outside the repository; no production API, test source, package, or assembly
configuration is changed by this documentation update.

All 23 complete programs were compiled and executed independently in **both
Debug and Release**, with 151 numeric output checks per configuration (302 total).
That includes the 11 README programs, five existing contact-guide programs, five
numbered programs here, and the complete rigid/compliant foundation examples.
Two existing README programs update state without printing; their compilation
and successful execution were checked. The 21 programs that print results were
also checked numerically, accepting rounded signed zero where appropriate.
Both shared helper blocks were compiled with every program that uses them.

| Check | Result |
| --- | --- |
| `dotnet test QuantumDynamixLib.sln --configuration Debug --no-restore` | 331 passed, 0 failed, 0 skipped |
| Corresponding `--configuration Release` test command | 331 passed, 0 failed, 0 skipped |
| `dotnet format QuantumDynamixLib.sln --verify-no-changes --no-restore` | Passed |
| Documentation example builds and execution | 23 programs in each configuration |
| Numeric output comparisons | 302 passed |
| Local documentation links and heading anchors | 36 checked |
| `git diff --check` | Passed |

The nine pre-existing README C# programs and all 24 original Markdown links were
preserved. Final Markdown code is checked against the compiled program/helper
files. Only the README and this new guide are changed in this documentation pass;
the preceding physics implementation and its tests are left intact. No commit or
push was performed.
