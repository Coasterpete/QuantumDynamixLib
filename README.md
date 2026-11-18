# QuantumDynamixLib
Official C# mathematics, geometry, and physics simulation library for QUANTUM DYNAMIX.

Licensed under the [MIT License](LICENSE).

## Current capabilities

Implemented in the current library:

- **Double-precision mathematics:** vectors, 3x3 matrices, Hamilton quaternions,
  and rigid poses in right-handed coordinates.
- **Splines and heartline geometry:** scalar and spatial B-splines/NURBS,
  analytic derivatives, curvature, arc-length mapping, rotation-minimizing frames,
  banking, and derived track-spine offsets.
- **Profiles and fitting:** easing functions, time/distance timelines,
  feature-relative anchors, constrained fitting, and nondestructive smoothing.
- **Physical rail geometry:** custom rail arrangements and cross-sections,
  section roll, surface queries, explicit spacing/gauge measurements, independent
  spine/rail distance maps, and geometric diagnostics.
- **Dynamics foundations:** unconstrained Newton-Euler rigid-body dynamics with
  RK4 integration, plus spherical-joint multibody dynamics and projected
  implicit-midpoint integration.

Future application functionality, including FVD reconstruction, wheel/rail contact
and guidance, complete coaster simulation, train animation, rendering, and UI,
is not implemented here. Rail geometry queries do not advance vehicles or apply
contact forces.

The owner-approved spline, easing, timeline, heartline, and constrained smoothing
foundation is documented in [Spline and timeline mathematics](docs/SplineTimelineFoundation.md),
including C# examples, numerical conventions, diagnostics, and implementation limits.

The owner-approved [physical rail geometry foundation](docs/PhysicalRailGeometryFoundation.md)
adds configurable rail arrangements and cross-sections, analytic offset geometry,
independent spine/rail distances, explicit spacing/gauge measurements, and continuity
diagnostics. Physical motion remains the responsibility of the existing dynamics solvers.

## Quick start

Install a stable .NET 10 SDK compatible with `global.json` (currently selecting
10.0.400 in the 10.0.4xx feature band; see [SDK and editor compatibility](#sdk-and-editor-compatibility)).
From the repository root, build the library and run the MSTest suite:

```powershell
dotnet build QuantumDynamixLib.sln --configuration Debug
dotnet test QuantumDynamixLib.sln --configuration Debug
```

To try the library from another .NET 10 project, these commands create a sibling
console project and reference the core library directly:

```powershell
dotnet new console --name QuantumDynamixDemo --output ../QuantumDynamixDemo --framework net10.0
dotnet add ../QuantumDynamixDemo/QuantumDynamixDemo.csproj reference ./QuantumDynamixLib.csproj
```

Replace the generated `Program.cs` with any one complete C# example below, then run
it from the repository root:

```powershell
dotnet run --project ../QuantumDynamixDemo/QuantumDynamixDemo.csproj
```

For an existing project, use its `.csproj` path in the `dotnet add ... reference`
command. Only `QuantumDynamixLib.csproj` is needed; the subsystem folders are part
of that single assembly. See [Development infrastructure](#development-infrastructure)
for Release builds, formatting checks, and editor setup.

## Usage examples

Each block is an independent console program. Geometry inputs below are
illustrative caller-supplied values in metres, not library defaults or
manufacturer dimensions. `Vector3d` stores numbers without attaching units;
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

## Development infrastructure

`QuantumDynamixLib.sln` contains one `net10.0` core class library and one MSTest
project. The core project explicitly includes C# sources from the existing
subsystem directories and excludes tests and generated files. Existing folders,
including `QuantumDynamix.Splines/Curves` and `QuantumDynamix.Tests/Application`,
retain their owner-defined organization. The core library has no external package
dependencies or application-framework dependencies; test packages are confined to
`QuantumDynamix.Tests`.

### SDK and editor compatibility

`global.json` selects the installed stable .NET SDK **10.0.400**, accepts newer
installed servicing patches in the **10.0.4xx** feature band, and excludes
prerelease SDKs. This selects the development toolchain; it does not choose a
library target framework. See Microsoft's [SDK selection documentation](https://learn.microsoft.com/en-us/dotnet/core/tools/global-json).

The setup machine has the following tools installed:

| Tool | Installed version(s) |
| --- | --- |
| .NET SDK | 8.0.303, 8.0.422, 9.0.313, 9.0.315, **10.0.400** |
| SDK MSBuild | 18.9.6 |
| Visual Studio Community 2026 | 18.9.1 |
| Visual Studio Community 2022 | 17.14.31 |
| Visual Studio Code | 1.145.2 |
| Git for Windows | 2.55.0.windows.3 |

Visual Studio 2026 18.9.1 is compatible with the selected SDK. Visual Studio 2022
17.14 cannot build projects with this selected SDK: the 10.0.400 SDK requires
Visual Studio 18.0 or newer. See Microsoft's [SDK and Visual Studio compatibility table](https://learn.microsoft.com/en-us/dotnet/core/porting/versioning-sdk-msbuild-vs).

In Visual Studio 2026, open `QuantumDynamixLib.sln`. In VS Code, open the repository
folder (`code .`). The workspace recommends the Microsoft C# and EditorConfig
extensions, both already installed on the setup machine. C# Dev Kit is also
installed locally and is optional. C# files use the Microsoft C# formatter on
save. Build, formatting verification, and test commands are available through
**Tasks: Run Task**; **Ctrl+Shift+B** selects the Debug build task. Build and test
tasks build and test the production library and its MSTest project.

### Shared conventions

- `.editorconfig` specifies UTF-8, LF line endings, four-space C# indentation,
  braces on new lines, and System imports first. JSON and MSBuild XML use two
  spaces. Solution files retain conventional BOM, CRLF, and tab formatting.
  C# style preferences are editor suggestions.
- `.gitattributes` keeps text line endings consistent across Git checkouts,
  including when a developer has `core.autocrlf` enabled.
- `Directory.Build.props` enables deterministic compilation and, for C#
  projects, nullable reference checking and the SDK's built-in analyzers with
  the latest stable rule version and the standard `Default` rule set.
- Compiler warning levels and Debug/Release behavior use SDK defaults. Warnings
  are not promoted to errors, and code-style rules are not enforced on build.
  No analyzer packages or other dependencies are added.
- The existing Visual Studio `.gitignore` covers build outputs, IDE caches,
  per-user settings, test results, and common tooling artifacts. It already
  permits shared VS Code settings, extension recommendations, and tasks.

These shared properties apply to C# projects beneath the repository root.
Individual project files can override them. Both current projects target
`net10.0`; the test project references the single core library. The SDK chooses
the default C# language version. Test execution uses MSTest through VSTest.

### Commands

Run these from the repository root to inspect the toolchain and solution:

```powershell
dotnet --info
dotnet --list-sdks
dotnet --version
dotnet sln QuantumDynamixLib.sln list
```

Build and verify the implementation with:

```powershell
# Build (also restores required packages).
dotnet build QuantumDynamixLib.sln --configuration Debug
dotnet build QuantumDynamixLib.sln --configuration Release

# Check formatting after restore/build; apply formatting with the second command.
dotnet format QuantumDynamixLib.sln --verify-no-changes --no-restore
dotnet format QuantumDynamixLib.sln --no-restore

# Run the MSTest suite.
dotnet test QuantumDynamixLib.sln --configuration Debug
dotnet test QuantumDynamixLib.sln --configuration Release
```

Check that test execution reports discovered and executed tests; a successful
command alone is not evidence of coverage or engineering validation.

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
solving, and separate projected implicit-midpoint integration. The existing
unconstrained dynamics, RK4 implementation, and original 48 regression cases are
unchanged. Both paths use the same assembly, mathematics, units, and frame
conventions. No package dependency was added.

### Definitions, state, and reactions

`MultibodySystem` copies ordered mass properties and spherical joints;
`MultibodyState` copies the corresponding `RigidBodyState` values. Collections
are read-only, and the contained properties and values are immutable. Body
indices refer to this fixed order. All bodies have finite positive mass and
numerically positive-definite inertia; no fixed-world body representation is
provided. A system can have disconnected bodies and any number of independent
spherical-joint rows. Redundant rows are explicitly rejected.

For each body, `x` is its world center of mass, `R` maps body directions to world
directions, and `s` is its local attachment offset in metres. The joint imposes
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
construction internally so additional joint equations can use the same dynamics
and mass operations when separately authorized. Current public joints are
spherical joints only.

`ConstrainedMultibodyDynamics.Evaluate` returns derivatives, resultant constraint
loads per body, reactions per joint, and residual diagnostics. It evaluates the
acceleration-level extension at the supplied snapshot, including off-manifold
snapshots; inspect its reported position and velocity errors. It does not
assemble initial data or advance time.

Each joint's `ForceOnA` is `lambda` in world newtons, and `ForceOnB` is its
negative. COM torques are `(R_A*s_A) x lambda` and `(R_B*s_B) x (-lambda)` in
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
reduces attachment gaps, using right-multiplied axis-angle quaternion corrections.
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

The dynamics foundations do not implement wheel/rail contact or guidance,
complete trains, graphics, or application-specific gravity/friction models.
Spline and rail geometry are implemented separately as described above.
Useful mathematical references are
[MIT's bilateral constraint equations](https://underactuated.mit.edu/multibody.html)
and [Hairer's projection analysis](https://www.unige.ch/~hairer/preprints/symproj.pdf).
