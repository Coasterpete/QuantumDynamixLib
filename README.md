# QuantumDynamixLib

**QuantumDynamixLib** is the official C#/.NET mathematics, geometry, and physics
library supporting **QUANTUM DYNAMIX**, a human-directed roller coaster engineering
and simulation application.

Licensed under the [MIT License](LICENSE).

## Quick start

Install a stable .NET 10 SDK in the **10.0.4xx** feature band. The repository's
`global.json` starts at **10.0.400**, selects the latest installed patch within
that band, and excludes prerelease SDKs. Both projects target `net10.0`.
Run `dotnet --version` from the repository root to check the selected SDK.
[.NET SDK selection](https://learn.microsoft.com/en-us/dotnet/core/tools/global-json)

From the repository root, build the solution and run the MSTest suite in Debug
and Release. The build restores the required test packages.

```powershell
dotnet build QuantumDynamixLib.sln --configuration Debug
dotnet test QuantumDynamixLib.sln --configuration Debug
dotnet build QuantumDynamixLib.sln --configuration Release
dotnet test QuantumDynamixLib.sln --configuration Release
```

To try the library, create a sibling console project and add a reference to the
core project. Use the same project reference from an existing .NET 10 project.

```powershell
dotnet new console --name QuantumDynamixDemo --output ../QuantumDynamixDemo --framework net10.0
dotnet add ../QuantumDynamixDemo/QuantumDynamixDemo.csproj reference ./QuantumDynamixLib.csproj
```

Replace the generated `Program.cs` with one of the [usage examples](#usage-examples),
following its setup instructions, then run this command from the repository root.

```powershell
dotnet run --project ../QuantumDynamixDemo/QuantumDynamixDemo.csproj
```

## Current capabilities and source layout

[QuantumDynamixLib.sln](QuantumDynamixLib.sln) contains the single core class
library [QuantumDynamixLib.csproj](QuantumDynamixLib.csproj) and the separate
MSTest project [QuantumDynamix.Tests](QuantumDynamix.Tests/QuantumDynamix.Tests.csproj).
The subsystem folders below organize source within the core assembly, which
has no external package or application framework dependencies.

| Source area | Implemented on `main` |
| --- | --- |
| `QuantumDynamix.Math` | Double-precision vectors, 3×3 matrices, and Hamilton quaternions. Easing functions, time and distance timelines, feature-relative anchors, and multichannel regions with sided evaluation and join diagnostics. |
| `QuantumDynamix.Splines`, `QuantumDynamix.Geometry` | Scalar and spatial B-splines and NURBS with analytic derivatives, constrained fitting, and nondestructive smoothing. Rigid poses, curve curvature, arc-length maps, and rotation-minimizing frames. |
| `QuantumDynamix.Track` | Banked heartlines, offset spines, and configurable rail arrangements and cross-sections. Rail surface queries, independent spine and rail distance maps, gauge measurements, and geometry diagnostics. Region-based longitudinal acceleration profiles integrate to speed and station for time-authored motion, or to speed and elapsed time for distance-authored motion. Time-authored motion can stop or reverse. Distance-authored motion requires verified positive forward speed. |
| `QuantumDynamix.Physics` | Newton–Euler rigid-body dynamics with RK4 and spherical or revolute multibody constraints with projected implicit-midpoint integration. Revolute joints allow independent wheel spin and report bearing reactions. Wheel/rail contact provides signed-gap geometry on declared smooth patches, first-order rigid normal impulses, and a separate compliant normal-contact solver using caller-supplied power-law coefficients. |
| `QuantumDynamix.FVD` | Forward force-vector design (FVD) evaluation for an ideal point with prescribed heartline motion. It calculates world coordinate acceleration and specific force in the banked track frame, using caller-supplied gravity. Numerical 3D heartline reconstruction uses prescribed longitudinal motion, transverse coordinate acceleration, and banking. Quintic track spline fitting includes sampled forward verification. |
| `QuantumDynamix.Supports` | Straight support-member geometry between a named rail-section datum and a supplied foundation point. |
| `QuantumDynamix.Operations` | Block occupancy for one supplied train extent on an open or closed route, with an unknown state for missing or invalid localization. |
| `QuantumDynamix.Telemetry` | Per-body center-of-mass position, speed, and instantaneous linear acceleration sampled at accepted constrained-step endpoints. |

The project also reserves source areas for `QuantumDynamix.Units`,
`QuantumDynamix.IO`, and `QuantumDynamix.Interop`. Full vehicle simulation,
tangential wheel/rail friction, traction and braking mechanisms, structural
support analysis, rendering, and UI are outside the current library scope.

## M3.x development status

The table reflects `main` at commit `da1a5ae`.

| Milestone | Status on `main` | Functionality |
| --- | --- | --- |
| M3 | Integrated | Reconstructs a numerical 3D heartline from prescribed longitudinal motion, transverse acceleration, and banking profiles. Regression cases include straights, circular arcs, a helix, and time- and distance-authored inputs. |
| M3.1 | Integrated | Fits the numerical heartline to a quintic spline for `HeartlineTrack`. Independent samples check geometry, banking, and acceleration through the forward FVD evaluator. |
| M3.2 | Integrated | Stabilizes frame transport when authored knots and generated sample parameters collide at floating-point resolution. Adds regression coverage and preserves numerical context for fit, arc-length, and frame-transport failures. |
| M3.3 Phase 1 | Integrated | Reduces allocations in adaptive quadrature and spline-basis evaluation. |
| M3.3 Phase 2, Step 1 | Integrated | Replaces a capturing segment-search predicate in `ScalarTimeline.Evaluate` with an indexed scan. |
| M3.3 Phase 2, Step 2 | Pending | |

### M3.3 Phase 2, Step 1

Regression tests exercise exact joins, adjacent floating-point coordinates,
sided evaluation, and boundary policies. [Step 1 change and benchmark record](https://github.com/Coasterpete/QuantumDynamixLib/pull/1)

An isolated Release probe measured **88 → 0 bytes per segment lookup**.
In the **600 m distance-authored reconstruction** benchmark on the development
tree, the same fixture and tolerances produced **40.3% fewer total allocated
bytes** and **11.0% lower median runtime** (8.158 s → 7.263 s across three warmed
Release runs per version). Numerical results and work accounting were
unchanged, including reconstruction and fit diagnostics, 74,802,135 quadrature
evaluations, and reconstruction `WorkUsed` of 4,320,117.

## Engineering guides

- Spline definitions, arc length, easing, timelines, anchors, and constrained
  smoothing. [Spline and timeline foundation](docs/SplineTimelineFoundation.md)
- Rail arrangements, analytic offsets, distance maps, gauge, and continuity
  diagnostics. [Physical rail geometry foundation](docs/PhysicalRailGeometryFoundation.md)
- Revolute constraint equations, reaction units, independent spin, and C#
  examples for angle history, axle torque, and gyroscopic coupling.
  [Revolute-joint foundation](docs/RevoluteJointFoundation.md)
  · [Revolute-joint examples](docs/RevoluteJointExamples.md)
- Signed-gap geometry, rigid impulses, compliant normal forces, and solver
  limits. [Rigid wheel/rail contact foundation](docs/WheelRailContactFoundation.md)
  · [Compliant normal-contact foundation](docs/CompliantWheelRailContactFoundation.md)
  · [Wheel/rail contact examples](docs/WheelRailContactExamples.md)

## Usage examples

The vector, spline, physical-rail, and introductory revolute blocks are independent
console programs.
The later contact programs use the explicitly provided shared example setup.
Geometry inputs below are illustrative caller-supplied values in metres, not
library defaults or manufacturer dimensions. `Vector3d` stores numbers without attaching units;
callers must use consistent units.

### Vector mathematics

Subtract two positions to find their displacement, measure its length, and
normalize it to obtain a unit direction. A dot product with a unit axis gives
the displacement along that axis. These operations help measure track-point
separation and resolve directions in engineering calculations.

```csharp
using System;
using QuantumDynamix.Math;

var start = new Vector3d(1, 2, 0);
var end = new Vector3d(4, 6, 0);
Vector3d displacement = end - start;
double distance = displacement.Length;
Vector3d direction = displacement.Normalized();
double alongX = Vector3d.Dot(displacement, Vector3d.UnitX);
Vector3d midpoint = start + displacement * 0.5;

Console.WriteLine($"Straight-line distance: {distance:F1} m");
Console.WriteLine($"Unit direction: ({direction.X:F1}, {direction.Y:F1}, {direction.Z:F1})");
Console.WriteLine($"Displacement along X: {alongX:F1} m");
Console.WriteLine($"Midpoint: {midpoint}");
```

Here the displacement is `(3, 4, 0)` m, its length is 5 m, and the unit direction
is `(0.6, 0.8, 0)`. This is a straight-line distance, not distance along a curved
track. Normalizing a zero vector throws because it has no direction.

### Spline and heartline geometry

Create a cubic curve with four control points, measure its heartline length,
and evaluate a derived spine halfway along that length. This separates the
authored heartline from an offset track spine, which is useful when reasoning
about banking and heartline-relative geometry.

```csharp
using System;
using QuantumDynamix.Geometry;
using QuantumDynamix.Math;
using QuantumDynamix.Splines.Curves;
using QuantumDynamix.Track;

var heartline = SplineCurve3d.Clamped(new[]
{
    new Vector3d(0, 0, 0), new Vector3d(5, 0, 0),
    new Vector3d(10, 5, 0), new Vector3d(10, 10, 0)
}, degree: 3);

var stations = new ArcLengthMap(heartline);
var offsets = TrackOffsetProfile.Constant(new Vector3d(0, 0, -1), stations.Length);
var track = new HeartlineTrack(heartline, Vector3d.UnitZ, offsets);
HeartlineTrackSample sample = track.Evaluate(stations.Length / 2);

Console.WriteLine($"Curve at u=0.5: {heartline.Evaluate(0.5).Position}");
Console.WriteLine($"Heartline length: {stations.Length:F3} m");
Console.WriteLine($"Heartline at half length: {sample.HeartlinePosition}");
Console.WriteLine($"Spine at half heartline length: {sample.SpinePosition}");
```

`degree: 3` explicitly selects a cubic; the spatial factory otherwise defaults
to degree five. Control points shape the curve and generally are not points it
passes through. The clamped parameter `u` ranges from 0 to 1 and is not a distance;
`HeartlineTrack.Evaluate` instead accepts heartline station in metres.

This example explicitly chooses initial up as +Z and an offset of -1 m along the
banked up axis. Omitted banking means zero bank. The spine position is a geometric
offset, not a vehicle trajectory. See [Heartline and track spine](docs/SplineTimelineFoundation.md#heartline-and-track-spine)
for varying offsets, banking, frames, and numerical limits.

### Custom physical rails and independent distances

Define two rails around a quarter-circle heartline and query a rail's position
and curvature. Each rail gets its own distance map, so the inside and outside
paths can have different lengths. This is useful for measuring rail geometry
without assuming that heartline, spine, and rail stations are interchangeable.

```csharp
using System;
using QuantumDynamix.Geometry;
using QuantumDynamix.Math;
using QuantumDynamix.Math.Functions;
using QuantumDynamix.Splines.Curves;
using QuantumDynamix.Track;

// Rational quadratic: a quarter circle of radius 10 m.
var heartline = SplineCurve3d.Clamped(new[]
{
    new Vector3d(10, 0, 0), new Vector3d(10, 10, 0), new Vector3d(0, 10, 0)
}, degree: 2, weights: new[] { 1, System.Math.Sqrt(0.5), 1 });
double length = new ArcLengthMap(heartline).Length;
var track = new HeartlineTrack(heartline, Vector3d.UnitZ,
    TrackOffsetProfile.Constant(new Vector3d(0, 1, 0), length));

// Minimal open section boundary in local X=0, not a manufactured rail profile.
var boundary = SplineCurve3d.Clamped(new[]
{
    new Vector3d(0, -0.1, 0), new Vector3d(0, 0.1, 0)
}, degree: 1);
var section = new RailCrossSection(new[] { boundary });
var zeroRoll = new CustomFunction(0, length, (_, _) => new ScalarJet(0));
var arrangement = new RailArrangement(new[]
{
    new RailDefinition("inner",
        TrackOffsetProfile.Constant(new Vector3d(0, 0.5, 0), length), section, zeroRoll),
    new RailDefinition("outer",
        TrackOffsetProfile.Constant(new Vector3d(0, -0.5, 0), length), section, zeroRoll)
});

// Numerical controls from RailTestFixtures, not engineering acceptance criteria.
var settings = new RailValidationSettings(
    positionTolerance: 1e-8, angularTolerance: 1e-7, curvatureTolerance: 1e-7,
    stationDerivativeTolerance: 1e-7, minimumSpeedRatio: 1e-10, minimumFrameProjection: 1e-8);
var rails = new RailGeometry(track, arrangement, settings);
double heartlineStation = length / 2;
RailSample sample = rails.Evaluate("inner", heartlineStation);
TrackDistanceMap spine = rails.CreateSpineDistanceMap();
TrackDistanceMap inner = rails.CreateRailDistanceMap("inner");
TrackDistanceMap outer = rails.CreateRailDistanceMap("outer");

Console.WriteLine($"Inner rail position: {sample.Position}");
Console.WriteLine($"Inner rail curvature: {sample.Curvature:F6} 1/m");
Console.WriteLine($"Spine / inner / outer lengths: {spine.Length:F3} / {inner.Length:F3} / {outer.Length:F3} m");
Console.WriteLine($"Inner rail station: {inner.StationAtHeartlineStation(heartlineStation):F3} m");
```

The weights represent the circular arc exactly; its distance map is evaluated
numerically. Offsets use `(forward, left, up)` components in the banked heartline
frame. Rail offsets are relative to the spine: its 1 m inward offset and the
rails' +0.5 m and -0.5 m offsets produce radii of 8.5 m and 9.5 m. Their lengths
are approximately 13.352 m and 14.923 m, while the spine length is 14.137 m.

`zeroRoll` explicitly supplies a constant section roll of zero radians; the
callback returns a scalar value with zero derivatives. All profiles cover the
complete heartline distance range. The settings above reproduce existing
[regression-fixture controls](QuantumDynamix.Tests/RailTestFixtures.cs), not
recommended coaster tolerances. Constructing `RailGeometry` does not run its
separate `Validate()` inspection. See [Physical rail geometry](docs/PhysicalRailGeometryFoundation.md)
for validation units and limits, surface queries, gauge datums, distance inversion,
and caller-defined profiles. This example defines geometry only; it does not
model wheel/rail contact or guide a vehicle.

### Body-local revolute joint frames

A revolute joint keeps two attachment points together and aligns their axles,
leaving one relative rotation free. Each attachment is a `RigidPose3d` in its
body's center-of-mass frame. Here both attachment +X axes point along body +Y,
and the two local offsets meet halfway between the body centers.

<!-- example: readme-revolute-frames -->
```csharp
using System;
using QuantumDynamix.Geometry;
using QuantumDynamix.Math;
using QuantumDynamix.Physics;

Quaterniond axleAlongY = Quaterniond.FromAxisAngle(Vector3d.UnitZ, System.Math.PI / 2);
var joint = new RevoluteJoint(0, 1,
    new RigidPose3d(new Vector3d(0, 0.5, 0), axleAlongY),
    new RigidPose3d(new Vector3d(0, -0.5, 0), axleAlongY));
var system = new MultibodySystem(new[]
{
    new RigidBodyMassProperties(2, Matrix3x3d.Diagonal(2, 3, 4)),
    new RigidBodyMassProperties(3, Matrix3x3d.Diagonal(3, 4, 5))
}, Array.Empty<SphericalJoint>(), new[] { joint });
var state = new MultibodyState(new[]
{
    new RigidBodyState(RigidPose3d.Identity, Vector3d.Zero, Vector3d.Zero),
    new RigidBodyState(new RigidPose3d(new Vector3d(0, 1, 0),
        Quaterniond.FromAxisAngle(Vector3d.UnitY, 0.3)), Vector3d.Zero, new Vector3d(0, 2, 0))
});

RevoluteJointSample sample = RevoluteJointKinematics.Evaluate(system, state, jointIndex: 0);
Console.WriteLine($"Wrapped angle: {sample.Angle:F3} rad");
Console.WriteLine($"Relative angular velocity: {sample.AngularRate:F3} rad/s");
Console.WriteLine($"World axle: ({sample.WorldAxis.X:F3}, {sample.WorldAxis.Y:F3}, {sample.WorldAxis.Z:F3})");
```

The angle is 0.300 rad, relative angular velocity is 2.000 rad/s, and world axle
is `(0, 1, 0)` to roundoff. A radian measures angle: pi radians is 180 degrees
and 2*pi radians is one revolution. Positive angle turns body B relative to A
by the right-hand rule about the directed axle. `jointIndex` indexes
`RevoluteJoints`; the existing `Joints` collection remains spherical-only.
The empty spherical array leaves only the revolute joint, and body indices 0/1
match the mass/state array order. These are finite bodies; the joint does not
fix either one to the world.

### Independent wheel spin

Give the wheel its own rigid body and full inertia tensor, then attach its tread
geometry to that body. This free-space example supplies a 2 kg carrier and a
separate 0.5 kg wheel. The wheel body's local +X is its axle; its world pose
rotates that direction onto +Y. The carrier's local attachment lies 1 m along +Y.

<!-- example: readme-independent-wheel -->
```csharp
using System;
using QuantumDynamix.Geometry;
using QuantumDynamix.Math;
using QuantumDynamix.Physics;

Quaterniond axleAlongY = Quaterniond.FromAxisAngle(Vector3d.UnitZ, System.Math.PI / 2);
var system = new MultibodySystem(new[]
{
    new RigidBodyMassProperties(2, Matrix3x3d.Diagonal(2, 2, 2)),
    new RigidBodyMassProperties(0.5, Matrix3x3d.Diagonal(0.02, 0.03, 0.03))
}, Array.Empty<SphericalJoint>(), new[]
{
    new RevoluteJoint(0, 1,
        new RigidPose3d(Vector3d.UnitY, axleAlongY), RigidPose3d.Identity)
});
var wheel = new WheelDefinition("wheel", bodyIndex: 1, RigidPose3d.Identity,
    new CrownedWheelTread(radius: 0.2, halfWidth: 0.08, crownRadius: 0.4));
var velocity = new Vector3d(10, 0, 0); // world m/s
var state = new MultibodyState(new[]
{
    new RigidBodyState(RigidPose3d.Identity, velocity, Vector3d.Zero),
    new RigidBodyState(new RigidPose3d(Vector3d.UnitY, axleAlongY),
        velocity, new Vector3d(5, 0, 0)) // wheel-body rad/s; world rotation is about +Y
});

state = ConstrainedMultibodyIntegrator.Step(system, state, 0, 0.001,
    (_, _) => new RigidBodyLoads[2]).State;
RevoluteJointSample sample = RevoluteJointKinematics.Evaluate(system, state, 0);
Console.WriteLine($"Carrier speed: {state.Bodies[0].LinearVelocity.X:F3} m/s");
Console.WriteLine($"Independent wheel spin: {sample.AngularRate:F3} rad/s");
Console.WriteLine($"No-slip comparison only: {state.Bodies[0].LinearVelocity.X / wheel.Tread.Radius:F3} rad/s");
```

The wheel keeps 5.000 rad/s while the carrier moves at 10.000 m/s. The printed
50.000 rad/s comparison is `v/r`; it is never assigned to the wheel. A free
bearing permits spin and does not impose no-slip rolling. The
[normal-contact program](docs/RevoluteJointExamples.md#4-rigid-and-compliant-normal-contact)
demonstrates this distinction while the rail actually supports the wheel.

`BodyAngularVelocity` uses body coordinates; linear velocity uses world coordinates.
The query calculates `dot(worldAxle, omegaB_world - omegaA_world)`, so it remains
correct when the bodies have different orientations. `WheelDefinition` supplies
geometry, while `RigidBodyMassProperties` supplies mass and COM inertia in kg*m^2.
The 0.02 entry resists rotation about wheel-body +X; the full tensor also governs
turning and tilting. The carrier properties exclude this wheel's mass and inertia.
All values here are supplied examples, not geometry-derived inertia or presets.

### Wheel/rail contact examples

Copy the [shared example setup](docs/WheelRailContactExamples.md#shared-setup)
into `ContactExampleSetup.cs` beside your demo's `Program.cs`. Each contact program
below can then replace `Program.cs` and run independently. The helper is example
code that builds explicit definitions using public library APIs; it is not an
additional library API. All dimensions, masses, stiffnesses, and numerical bounds
are synthetic examples. [The full guide](docs/WheelRailContactExamples.md) includes
the complete setup, longer examples, expected results, and verification details.

#### Detect a wheel/rail gap and contact position

<!-- example: readme-contact-detection -->
```csharp
using System;
using QuantumDynamix.Physics;

var contacts = ContactExampleSetup.CreateSystem();
var state = ContactExampleSetup.CreateState(gap: 0.01);
WheelRailContactSample hit = WheelRailContactGeometry.Query(contacts, state).Contacts[0];
Console.WriteLine($"Gap: {hit.Gap:F6} m");
Console.WriteLine($"Normal: ({hit.Normal.X:F1}, {hit.Normal.Y:F1}, {hit.Normal.Z:F1})");
Console.WriteLine($"Wheel point: ({hit.WheelPoint.X:F3}, {hit.WheelPoint.Y:F3}, {hit.WheelPoint.Z:F3}) m");
```

This gives a 0.01 m clearance, normal `(0, 0, 1)`, and wheel-surface witness
`(5, 0, 0.01)` m above the rail at Z=0. Tiny near-zero components may print as
`-0.000`. The witness is a point on the undeformed
tread, not the wheel center. A positive gap means clearance, zero means touching,
and a negative gap represents nominal overlap. A query reports geometry without
applying force or advancing time; unresolved geometry throws rather than implying
separation. See [detection and signed gaps](docs/WheelRailContactExamples.md#1-detection-and-signed-gaps).

#### Rigid contact impulses and separation

<!-- example: readme-rigid-contact -->
```csharp
using System;
using QuantumDynamix.Physics;

var contacts = ContactExampleSetup.CreateSystem();
var controls = new ContactSolverSettings(0.05, 0.05, 0.001, 0.01);
var incoming = ContactExampleSetup.CreateState(gap: 0, normalVelocity: -1);
ContactStepResult impact = ContactMultibodyIntegrator.Step(contacts, incoming, 0, 0.01,
    (_, _) => new RigidBodyLoads[1], controls);
Console.WriteLine($"Impulse: {impact.ContactImpulses[0].NormalImpulse:F3} N*s");
Console.WriteLine($"Average force: {impact.ContactImpulses[0].AverageForce.Z:F3} N");

var leaving = ContactExampleSetup.CreateState(gap: 0, normalVelocity: 0.5);
ContactStepResult separation = ContactMultibodyIntegrator.Step(contacts, leaving, 0, 0.01,
    (_, _) => new RigidBodyLoads[1], controls);
Console.WriteLine($"Separating impulse: {separation.ContactImpulses[0].NormalImpulse:F3} N*s");
Console.WriteLine($"Separated gap: {separation.AcceptedContacts[0].Gap:F6} m");
```

The 2 kg body closing at 1 m/s receives 2 N*s, stopping its normal motion in this
central, initially touching case. Impulse is the change in momentum; dividing it
by the 0.01 s step gives a **step-average** force of 200 N. This is not a measured
impact peak. The separate leaving case receives zero impulse and opens a 0.005 m
gap: contact can push, but cannot pull the wheel back. The load callbacks return
one zero force/torque value, so no gravity or other external load is applied.
[The longer rigid example](docs/WheelRailContactExamples.md#2-rigid-impulses-separation-and-average-force)
also shows why halving the step doubles this impact's reported average force.

#### Compliant compression, normal force, and stored energy

<!-- example: readme-compliant-contact -->
```csharp
using System;
using System.Collections.Generic;
using QuantumDynamix.Math;
using QuantumDynamix.Physics;

var contacts = ContactExampleSetup.CreateSystem();
var law = new PowerLawNormalContact(
    coefficient: 2000, exponent: 1, dissipation: 0, maximumCompression: 0.02);
var model = new CompliantWheelRailContactModel(contacts,
    new Dictionary<WheelRailPair, PowerLawNormalContact> { [contacts.Pairs[0]] = law });
var state = ContactExampleSetup.CreateState(gap: -0.01);
var controls = new CompliantContactStepSettings(0.05, 0.05, 0.001, 0.01);
CompliantContactStepResult result = CompliantContactMultibodyIntegrator.Step(model, state, 0, 0.001,
    (_, _) => new[] { new RigidBodyLoads(new Vector3d(0, 0, -20), Vector3d.Zero) }, controls);
NormalContactResponse response = result.AcceptedContacts.Contacts[0].Response;
Console.WriteLine($"Compression: {response.Compression:F6} m");
Console.WriteLine($"Normal force: {response.NormalForce:F3} N");
Console.WriteLine($"Stored energy: {response.ElasticEnergy:F6} J");
```

Compression is `delta = max(0, -gap)`: negating a negative gap gives a positive
deformation, while positive clearance gives zero compression. With exponent 1
and zero dissipation, `force = k * delta` and `energy = 0.5 * k * delta * delta`.
Here k=2000 N/m and delta=0.01 m give 20 N and 0.1 J. The supplied -20 N load
balances that force, so compression remains in the accepted state. The 0.02 m
physical validity limit is separate from numerical movement/correction bounds.
[The compliant example](docs/WheelRailContactExamples.md#3-compliant-force-and-elastic-energy)
explains the general power law and non-attractive damping.

#### Multiple-wheel load sharing

For three collinear supports with equal clearances, equal outside stiffnesses,
and no overturning moment, equilibrium requires the common compression below.
This small program is an analytical check; the
[complete three-wheel library example](docs/WheelRailContactExamples.md#4-three-support-load-sharing)
constructs all contacts, advances the coupled body, and prints each wheel's load.

<!-- example: readme-three-support-check -->
```csharp
using System;

double load = 4000;             // N, applied downward through the center
double outsideStiffness = 1e6;  // N/m for each outside wheel
double middleStiffness = 2e6;   // N/m
double compression = load / (2 * outsideStiffness + middleStiffness);
Console.WriteLine($"Compression: {compression:F6} m");
Console.WriteLine($"Each outside wheel: {outsideStiffness * compression:F3} N");
Console.WriteLine($"Middle wheel: {middleStiffness * compression:F3} N");
```

The result is 0.001 m compression and loads `(1000, 2000, 1000)` N. If the middle
stiffness is also 1e6 N/m, compression becomes about 0.001333 m and each wheel
carries about 1333.333 N. The full example verifies both cases. Changing stiffness
changes the equilibrium deformation as well as the force split; clearance,
unequal outside stiffness, or applied moments require the full geometry/dynamics.

#### Choosing rigid or compliant contact

| Question | Rigid formulation | Compliant formulation |
| --- | --- | --- |
| Appropriate use | Negligible-deformation approximation with determinate reactions | Deformation and load sharing using justified owner-supplied coefficients |
| Main contact output | `NormalImpulse` in N*s; `AverageForce` in N over the step | `NormalForce` in N at the evaluated state; `ElasticEnergy` in J |
| Compression | Enforces numerical nonpenetration bounds | Retains physical compression within the supplied law's domain |
| Redundant rigid supports | Can raise `IndeterminateReactions` | Forces follow the supplied deformation laws; a stable equilibrium is still required |

[The side-by-side program](docs/WheelRailContactExamples.md#5-rigid-versus-compliant-output-quantities)
evaluates both formulations independently. The current rigid impacts are inelastic;
the zero-dissipation compliant example stores and returns elastic energy. Their
outputs describe different assumptions, and neither establishes a real impact
peak. Use one formulation per physical contact. Both paths retain bounded local
geometry and timestep limitations and implement normal interaction only.

## Development infrastructure

`Directory.Build.props` enables deterministic compilation, nullable reference
checking, and the SDK's built-in analyzers. The test project uses
`Microsoft.NET.Test.Sdk`, `MSTest.TestAdapter`, and `MSTest.TestFramework`.
Run `dotnet sln QuantumDynamixLib.sln list` to see the two projects.

Open `QuantumDynamixLib.sln` in a Visual Studio version that supports
`net10.0` and the selected SDK, or open the repository folder in VS Code.
The VS Code workspace recommends the Microsoft C# and EditorConfig extensions
and provides build, formatting check, and test tasks. **Ctrl+Shift+B** runs the
Debug build task. [SDK and Visual Studio compatibility](https://learn.microsoft.com/en-us/dotnet/core/porting/versioning-sdk-msbuild-vs)

## Unconstrained rigid-body foundation

The first implementation provides custom double-precision vectors, 3x3 matrices,
Hamilton quaternions, rigid poses, constant mass properties, Newton-Euler dynamics,
and fixed-step RK4 integration for one freely moving body. Coordinates are
right-handed. Position is the world center of mass (m), linear velocity is in world
coordinates (m/s), angular velocity is in body coordinates (rad/s), and orientation
maps body directions into world coordinates. Mass is in kg and body-frame inertia
about the center of mass is in kg*m^2. No up axis or gravity constant is imposed.

```csharp
using QuantumDynamix.Geometry;
using QuantumDynamix.Math;
using QuantumDynamix.Physics;

// Example supplied properties and load, not application defaults.
var properties = new RigidBodyMassProperties(2, Matrix3x3d.Diagonal(2, 3, 4));
var state = new RigidBodyState(RigidPose3d.Identity, Vector3d.Zero, Vector3d.Zero);
state = RigidBodyIntegrator.Step(properties, state, time: 0, step: 0.01,
    evaluateLoads: (time, stage) => new RigidBodyLoads(new Vector3d(4, 0, 0), Vector3d.Zero));
```

`RigidBodyDynamics.EvaluateDerivative` evaluates the continuous equations without
advancing time. `RigidBodyIntegrator.Step` evaluates the supplied load function at
all four RK4 stage times and states, including two distinct midpoint states. Loads
may depend on time, position, orientation, and velocities. They are world-space
forces (N) and world-space torques about the center of mass (N*m). Use
`RigidBodyLoads.FromForceAtPoint` to include an off-center force's moment. Load
functions should be deterministic and have no side effects.

Numerical constructors reject non-finite components. Mass must be positive;
inertia must be exactly symmetric and numerically positive definite. A scaled
Cholesky factorization is cached for inertia solves. Pose construction normalizes
nonzero quaternions; zero quaternions and default poses/states are invalid.
Quaternion derivatives are algebraic values and are never normalized. Raw RK4
quaternion stage sums are retained while normalized stage views are passed to
dynamics and loads; the accepted pose is normalized. Non-finite or unresolvable
stage times and non-finite arithmetic results are rejected.

RK4 is fourth order for smooth loads, but is not symplectic or exactly
energy-conserving. Large timesteps, stiff loads, ill-conditioned inertia, and
floating-point range limitations require care. Symmetric positive-definite
validation does not establish that a supplied tensor matches the intended mass
distribution. Test tolerances are numerical regression checks, not application
engineering acceptance criteria. Repeatability is tested on the same runtime and
platform; cross-platform bitwise identity is not promised. This unconstrained path
excludes constraints, contacts, splines, and articulation.

### Owner decisions

The owner approved the initial single-core-library and MSTest arrangement,
double-precision SI mathematics, center-of-mass frames, full inertia tensors,
Newton-Euler dynamics, and fixed-step RK4. Architecture, new APIs and algorithms,
engineering requirements, and future changes remain human-directed; see
`AGENTS.md`.

## Constrained multibody foundation

The owner approved full rigid-body coordinates, Lagrange-multiplier bilateral
constraints, body-local spherical-joint attachments, coupled dense Cholesky
solving, and separate projected implicit-midpoint integration. The unconstrained
and constrained paths use the same assembly, mathematics, units, and frame
conventions.

### Definitions, state, and reactions

`MultibodySystem` copies ordered mass properties and joint definitions;
`MultibodyState` copies the corresponding `RigidBodyState` values. Collections
are read-only, and the contained properties and values are immutable. Body
indices refer to this fixed order. All bodies have finite positive mass and
numerically positive-definite inertia; no fixed-world body representation is
provided. A system can have disconnected bodies and any number of independent
spherical/revolute constraint rows. Redundant rows are explicitly rejected.

For a spherical joint, `x` is each body's world center of mass, `R` maps body
directions to world directions, and `s` is its local attachment offset in metres. The joint imposes
`C = x_A + R_A*s_A - x_B - R_B*s_B = 0`. This is three scalar constraints,
leaving all relative rotations free. Stacking world linear and body angular
velocities into `u` gives:

```text
J_A = [identity, -R_A*[s_A]x]     J_B = [-identity, R_B*[s_B]x]
C_dot = J*u
C_ddot = J*u_dot + b
b = R_A*(omega_A x (omega_A x s_A)) - R_B*(omega_B x (omega_B x s_B))

M = block_diagonal(m_i*identity, I_body_i)
W = J*M^-1*J^T
W*lambda = -(J*a_free + b)
u_dot = a_free + M^-1*J^T*lambda
```

`[s]x*v` means `s x v`. `a_free` includes the existing Euler gyroscopic term.
Inverse-mass applications use division by mass and cached body-inertia solves;
no matrix inverse is explicitly formed. All joint rows are solved together,
including coupling through shared bodies. `ConstraintAssembly` holds the row
construction internally. Revolute joints use the same dynamics and mass operations,
adding two axle-alignment equations to the three point equations; see the
[revolute foundation](docs/RevoluteJointFoundation.md#five-constraints-and-their-derivatives).

`ConstrainedMultibodyDynamics.Evaluate` returns derivatives, resultant constraint
loads per body, reactions per joint, and residual diagnostics. It evaluates the
acceleration-level extension at the supplied snapshot, including off-manifold
snapshots; inspect its reported position and velocity errors. It does not
assemble initial data or advance time.

Each spherical joint's `ForceOnA` is `lambda` in world newtons, and `ForceOnB`
is its negative. COM torques are `(R_A*s_A) x lambda` and `(R_B*s_B) x (-lambda)` in
world N*m. The spherical joint has no independent rotational couple.

### Integration and quaternion treatment

`ConstrainedMultibodyIntegrator.Step` solves the coupled implicit-midpoint
equations using damped Newton iterations. Its unknowns are scaled endpoint
velocity increments, six per body. Midpoint positions follow from midpoint
linear velocity. For each body's midpoint angular velocity, define
`a = h*omega_mid/4` and `H = normalize(Quaterniond(1, a.X, a.Y, a.Z))`:

```text
q_mid_physical = q_start * H
q_end         = q_start * H * H
```

This algebraically solves the raw quaternion midpoint equation
`q_end - q_start = (h/4)*(q_start + q_end)*(0,omega_mid)`.
The physical stage orientation is the normalized arithmetic midpoint; the
quaternion derivative is never treated as a rotation. The Cayley rotation update
preserves unit norm to roundoff and avoids dividing by a zero spin axis. Pose
construction normalizes physical views. It is second order, with phase error
when angular displacement per step is too large.

After the midpoint solve, mass/inertia-weighted Newton position projection
reduces attachment gaps and revolute axle misalignment, using right-multiplied
axis-angle quaternion corrections.
Velocity projection then reduces `J*u` at the corrected poses. These numerical
operations do not add stiffness or compliance. Body angular velocity components
are held during position projection and subsequently velocity-projected;
projection can therefore change energy and angular momentum.

`ConstrainedStepResult.AcceptedDynamics` recomputes **instantaneous** reactions
at the accepted endpoint and endpoint external loads. These are not timestep
averages. `ProjectionCorrections` separately records numerical world position
changes, relative orientation quaternions, world velocity changes, and changes
of body angular velocity components. None is reported as a physical force or
torque. Step diagnostics include midpoint residual/iterations, projection
iterations, and pre-projection gap/slip; accepted errors are in
`AcceptedDynamics.Diagnostics`.

The external load callback receives time and the whole immutable state, and
returns one world force/COM torque per body. Newton's central finite differences
evaluate midpoint trial states repeatedly, which may be off the joint manifold.
Callbacks must be smooth, deterministic, and side-effect free; their derivatives
are not required. Initial and accepted-endpoint load evaluations also occur.

```csharp
using System.Collections.Generic;
using QuantumDynamix.Geometry;
using QuantumDynamix.Math;
using QuantumDynamix.Physics;

// Supplied demonstration properties and loads, not application defaults.
var system = new MultibodySystem(
    new[] {
        new RigidBodyMassProperties(2, Matrix3x3d.Diagonal(2, 3, 4)),
        new RigidBodyMassProperties(3, Matrix3x3d.Diagonal(3, 4, 5))
    },
    new[] { new SphericalJoint(0, 1, Vector3d.UnitY, -Vector3d.UnitY) });
var state = new MultibodyState(new[] {
    new RigidBodyState(new RigidPose3d(new Vector3d(0, -1, 0), Quaterniond.Identity), Vector3d.Zero, Vector3d.Zero),
    new RigidBodyState(new RigidPose3d(new Vector3d(0, 1, 0), Quaterniond.Identity), Vector3d.Zero, Vector3d.Zero)
});
static IReadOnlyList<RigidBodyLoads> Loads(double time, MultibodyState snapshot) =>
    new[] { new RigidBodyLoads(new Vector3d(10, 0, 0), Vector3d.Zero), default };

ConstrainedStepResult result = ConstrainedMultibodyIntegrator.Step(system, state, 0, 0.01, Loads);
state = result.State;
Vector3d instantaneousForceOnA = result.AcceptedDynamics.JointReactions[0].ForceOnA;
```

### Provisional numerical controls and failure behavior

`ConstrainedSolverSettings` is immutable. Its optional constructor arguments
allow independent control of the following provisional defaults:

| Control | Default | Units and rationale |
| --- | --- | --- |
| `PositionTolerance` | `1e-10` | Maximum Euclidean joint gap, m; tight baseline for metre-scale fixtures |
| `VelocityTolerance` | `1e-10` | Maximum Euclidean attachment slip, m/s |
| `NonlinearTolerance` | `1e-11` | Infinity norm of dimensionless midpoint velocity residual |
| `LinearSolveTolerance` | `1e-11` | Scaled normwise linear backward error; leaves room for accumulated roundoff |
| `MinimumScaledPivot` | `1e-12` | Reject very small Cholesky/relative LU pivots; a rank/conditioning guard, not a condition-number estimate |
| `LinearVelocityScale` | `1` | Per-body scale floor, m/s |
| `AngularVelocityScale` | `1` | Per-body scale floor, rad/s |
| `FiniteDifferenceStep` | `6e-6` | Relative dimensionless central-difference perturbation, approximately cube-root double machine precision |
| `MaxNonlinearIterations` | `20` | Maximum Newton corrections per step |
| `MaxProjectionIterations` | `12` | Maximum corrections for each projection |
| `MaxLineSearchIterations` | `12` | Full step followed by up to 11 halvings |

Revolute joints also use `AngularConstraintSettings`: provisional defaults are
1e-10 rad axle misalignment and 1e-10 1/s alignment-row rate (forbidden rad/s at
alignment). A `ConstrainedSolverSettings` overload accepts these settings through
its `angularConstraints` argument. Angular diagnostics and tolerances are separate
from the metre-valued attachment errors above; see
[angular numerical controls](docs/RevoluteJointFoundation.md#numerical-controls-and-limits).

Each velocity scale is the maximum of its floor, initial vector speed, and
`h*initial_acceleration_length`; it stays fixed throughout a step. The Newton
unknown is `z = (u_end-u_start)/scale`, with residual
`z - h*acceleration_mid/scale`. Central differences perturb each component by
`6e-6*max(1,abs(z_component))`. Both line searches require either meeting the
relevant tolerance or reducing the residual by a factor below
`1 - 1e-4*step_fraction`.

Cholesky factors `D^-1*W*D^-1` with `D_ii=sqrt(W_ii)`. It rejects scaled
diagonal pivot remainders at or below `1e-12`. Newton linear systems use row
scaling and partial-pivoted LU, with the same relative pivot floor. Both solvers
check `||A*x-b||_inf/(||A||_inf*||x||_inf+||b||_inf)` in their scaled systems.
An adequate backward error does not guarantee a small forward error in an
ill-conditioned system. Unscaled acceleration residuals are also reported in m/s^2.

Initial gap and slip must meet the configured tolerances. Otherwise the step
fails before load evaluation; initial assembly is the caller's responsibility.
`ConstrainedSolveException` identifies the failure, stage, completed iteration
count, and residual. Messages identify residual units; singular constraints
report a scaled pivot. Singular constraints, failed linear/nonlinear solves,
failed projection, and unsupported arithmetic never return an advanced state.
Inputs remain unchanged. Invalid definitions, control values, load counts, and
unresolvable times produce argument exceptions. There is no automatic retry,
pseudoinverse, hidden regularization, or fallback force model.

These defaults are provisional numerical choices, not permanent engineering
requirements. A successful step establishes constraint and solver residual
bounds; it does not bound trajectory or reaction-force error. Timestep refinement
is still required. Absolute gap/slip limits can become unattainable at large
world coordinates or extreme velocities because of double-precision resolution.
Dense factorization and finite-difference Newton are intended for small systems;
they have substantial allocation and cubic solve costs as systems grow.
Ordinary endpoint projection is not symplectic or exactly conservative.

### Regression evidence and limits

The new tests retain all original regression cases and check analytic force
sharing, off-center torque, centripetal reactions, rotated full tensors, ideal
joint power, total force/moment balance, multiple coupled joints, disconnected
bodies, immutable snapshots, endpoint reaction semantics, and failure behavior.
New integration tests cover analytical spin, analytical coupled rotation,
nonlinear spin damping, general tumbling, and timestep/mass-ratio sensitivity.

In the initial Debug run on this machine:

| Fixture | Measurement |
| --- | --- |
| Analytical spin, halved steps | Error ratios `3.982`, `3.996` |
| Coupled analytical rotation, halved steps | Error ratios `3.969`, `3.985` |
| Nonlinear spin damping, halved steps | Error ratios `3.9994`, `3.9999` |
| Rotating pair, 10 s, h=0.01 s | Max gap `1.8e-15` m; slip `3.4e-16` m/s |
| Same rotating pair | Max relative energy error `1.4e-7`; relative total angular momentum error `7.1e-8` |
| Same rotating pair | Max total linear momentum error `3.8e-15` kg*m/s |
| General tumbling, 1 s, h=0.01 s | Endpoint relative energy error `1.5e-7`; angular momentum error `2.5e-6` kg*m^2/s |

Total angular momentum includes both COM spin and orbital `x x (m*v)`.
Fixture regression limits allow relative energy error below `1e-6` and relative
angular momentum error below `5e-7` in the 10 s pair, linear momentum error below
`1e-11` kg*m/s, and gap/slip within their configured tolerances. The general
tumble permits endpoint relative energy error below `1e-6` and angular momentum
error below `2e-5` kg*m^2/s. These provide margins around measured errors while
detecting degradation; they are not coaster acceptance criteria.

Sensitivity fixtures use mass ratios 1, 1000, and 1,000,000 with speeds 0.2, 2,
and 10 rad/s, respectively, and `h*speed=0.02`. They check constraints, total
momentum/energy, and the light body's analytical motion and reaction separately
so a heavy body's spin energy cannot conceal light-body errors. Motion and
relative reaction error must be below `1e-3`; relative energy error below
`5e-6`, and momentum error below `1e-10` kg*m/s. These are selected cases, not a
validated operating envelope. Convergence checks expect approximately 4x error
reduction when halving h, with bands allowing higher-order terms and finite
reference error. Unit-scale analytic dynamics comparisons use roughly
`1e-15` to `3e-14` component tolerances for floating-point roundoff.

The RK4 and bilateral midpoint entry points above do not themselves impose
wheel/rail contact. The compliant wrapper supplies contact forces to the existing
midpoint infrastructure through its load callback. The separate contact foundations
are documented below; complete trains, graphics,
and application-specific gravity/friction models remain outside these solvers.
Spline and rail geometry are implemented separately as described above.
Useful mathematical references are
[MIT's bilateral constraint equations](https://underactuated.mit.edu/multibody.html)
and [Hairer's projection analysis](https://www.unige.ch/~hairer/preprints/symproj.pdf).

## Revolute joints and independent wheel spin

The [introductory programs](#body-local-revolute-joint-frames) use the implemented
five-row joint: three equations keep attachment origins together and two align
their directed +X axles. Relative axial rotation is free. Initial attachment
positions, axle directions, and constrained velocities must be consistent;
antiparallel/singular configurations and unresolved solves fail explicitly.

`RevoluteJointKinematics.Evaluate` returns `Angle` in (-pi,pi] radians,
`AngularRate` in relative rad/s, and `WorldAxis` as a world unit direction. A
rotating carrier contributes to the wheel's absolute angular velocity; relative
spin measures only B's motion along the axle relative to A.

An orientation snapshot does not count complete turns. Keep an application-owned
angle history and update it with `RevoluteJointKinematics.UnwrapAngle` only after
accepting a step. Changes must be resolved to less than pi radians between samples;
the helper cannot detect missed full turns. Divide accumulated angle change by
`2 * System.Math.PI` for revolutions since the initial sample. The
[complete history program](docs/RevoluteJointExamples.md#1-wrapped-angle-and-accumulated-revolutions)
keeps that history outside the pure load callback.

An applied axle torque T on the wheel needs -T on its carrier when representing
an internal shaft action. `RigidBodyLoads.Torque` is a world COM torque in N*m.
The [paired-torque program](docs/RevoluteJointExamples.md#2-applying-an-axle-torque)
shows the wheel accelerate and the finite carrier respond in the opposite
direction. This demonstrates explicit supplied loads, not a built-in motor or
braking mechanism.

`ConstrainedMultibodyDynamics.Evaluate(...).RevoluteJointReactions` reports
instantaneous `ForceOnA/B` in N, independent `CoupleOnA/B` in N*m, and
`TorqueOnA/B` about the respective COMs in N*m. Force and couple have opposite
signs on the two bodies; the total COM torque includes the attachment force's
lever-arm moment: `torque = arm cross force + couple`. The ideal bearing couple
has no axial component on an assembled hinge. See the
[bearing-force and gyroscopic-couple program](docs/RevoluteJointExamples.md#3-bearing-forces-and-gyroscopic-couples).

A wheel's spin angular momentum about its COM is `H = R * I_body * omega_body`.
Turning its axle changes H's direction, requiring torque and producing a bearing
reaction on the carrier. The existing full Newton-Euler equations supply this
gyroscopic coupling. Do not add another gyroscopic torque to the callback.

Both normal-contact paths use the wheel body's complete angular motion. The
rigid result exposes `RevoluteJointImpulses`: force impulses are N*s, couple/COM
angular impulses are N*m*s, and division by the step gives an average load.
The compliant result exposes instantaneous endpoint bearing reactions through
`JointStep.AcceptedDynamics.RevoluteJointReactions` while retaining physical
compression. Numerical projection corrections remain separate from physical loads.
Neither normal-contact path implements tangential friction or enforces `v/r`.

Fast spin needs a timestep-refinement study. Halve h and compare accumulated
phase, velocities, individual reactions, and energy for the same initial state
and duration. Small constraint residuals alone do not establish accuracy. The
[fast-spin program](docs/RevoluteJointExamples.md#5-refining-the-timestep-for-fast-spin)
shows phase error decrease under refinement; tightening angular tolerances alone
does not remove that integration error. The midpoint and smooth compliant paths
are second order under their documented conditions; rigid contact remains first
order. Contact travel limits also count axle spin. See the
[engineering foundation](docs/RevoluteJointFoundation.md) for the mathematics,
measured regressions, geometry limits, and failure behavior.

## Wheel/rail contact foundation

`WheelRailContactSystem` binds existing bodies and stationary `RailGeometry` to
explicit wheel/patch pairs. `WheelRailContactGeometry.Query` returns local signed
gaps, normals, witnesses, and relative material-point velocities.
`ContactMultibodyIntegrator.Step` advances the existing `MultibodyState` using a
coupled bilateral/unilateral impulse solve and bounded numerical corrections.

This is a first-order rigid inelastic approximation for small, determinate systems
on declared regular smooth patches. It is not continuous collision detection or a
complete vehicle model. Unresolved searches, unsupported features, dependent tight
support rows, and exhausted solver budgets fail explicitly without advancing input
snapshots. See [the engineering document](docs/WheelRailContactFoundation.md) for
the exact supported domain, failure policy, units, and numerical limitations.

`CompliantWheelRailContactModel` assigns a `PowerLawNormalContact` to each binding.
`CompliantWheelRailContact.Evaluate` reports instantaneous normal forces and stored
elastic energy; `CompliantContactMultibodyIntegrator.Step` couples those forces to
the existing joint solver. Physical compression is retained and joint-projection
energy changes are reported separately. See the
[compliant engineering document](docs/CompliantWheelRailContactFoundation.md) and
[practical C# contact guide](docs/WheelRailContactExamples.md) for explicit physical
parameters, working examples, numerical controls, and limitations.
