# Wheel/rail contact foundation

This implements the owner's approved first-stage contact architecture in the
existing core assembly. No package, project, or assembly dependency was added.
The existing rigid-body, spherical-joint, spline, and rail implementations retain
their entry points and semantics. The new contact path consumes their existing
mathematics and geometry; it does not prescribe a vehicle trajectory.

This document describes the rigid impulse formulation. The optional
[compliant normal-contact foundation](CompliantWheelRailContactFoundation.md)
uses the same signed-gap geometry and existing joint dynamics through a separate
force/integration path; its physical compression is not rigid penetration correction.

## Supported physical model

- Stationary rails and explicit owner-authored wheel/rail pair bindings.
- Finite-width circularly crowned treads rigidly mounted to existing body COM frames.
- Frictionless rigid unilateral normal contact, with inelastic discrete impacts.
- Spherical and revolute joints, solved together with contact impulses.
- Arbitrary definition counts and body assignments, including single, dual, and
  asymmetric rail arrangements, subject to determinate, numerically resolved rows.

Running, side-friction, and upstop wheels use the same equations. Orientation and
opposing surface determine the force direction. A side wheel supplies normal
lateral guidance without tangential friction. There are no manufacturer presets,
gauge assumptions, wheel-count templates, or automatic bogie construction.

Independent wheel spin is available through explicit wheel bodies and the
[revolute-joint foundation](RevoluteJointFoundation.md). Existing mounted-wheel
definitions retain their meaning. Friction, suspension, elastic contact,
preload distribution, derailment rules, animation, and rendering are absent from
this rigid path. The separate compliant path supplies local normal deformation
and owner-parameterized load sharing.
Owners supply the body's mass and inertia, including whatever mounted mass their
model represents. Axisymmetric tread geometry needs no spin coordinate, but this
does not by itself model an independently spinning wheel's inertia or gyroscopic
effects; those belong to the explicit wheel body and its bearing constraints.

## Definitions, units, and immutability

`WheelDefinition` contains a stable ID, existing body index, `LocalPose`, and
`CrownedWheelTread`. Local +X is the axle; the pose maps wheel coordinates into
the body's COM coordinates. The body pose then maps them into world coordinates.
No body frame is inferred from the track. Dimensions are metres, mass kg, time s,
and angles radians, following the existing library conventions.

`RailContactPatch` declares a rail ID, boundary index, heartline-station interval,
profile-parameter interval, and `RailMaterialSide`. Bounds must lie inside the
existing rail and boundary domains. The first implementation conservatively
rejects interior known station/profile breakpoints, even if a particular join is
smooth. Crossing corners, seams, and patch edges is not implemented.

The caller explicitly declares each bound patch regular, smooth, and free of
intersections. Sampled Jacobian and coincidence checks are additional numerical
guards, not a continuous-domain certificate. Arbitrary callbacks do not expose
global derivative bounds. Open boundaries are not interpreted as closed solids,
and no interior, end cap, material winding, or connectivity is invented.

`WheelRailContactSystem` references the existing `MultibodySystem` and
`RailGeometry`, copies the input definition collections, and exposes read-only
collections. IDs are unique and ordinal. Wheels, patches, and pairs are sorted
by ordinal IDs for deterministic arithmetic; body order remains the existing
system's fixed order. Duplicate pair bindings and unknown references are rejected.
Existing scalar callbacks must remain pure, deterministic, and unchanged.

Only the frictionless rigid inelastic law is available. There are no accepted
but ignored restitution, stiffness, damping, or friction parameters.

## Tread and signed-gap geometry

For axial coordinate a, running radius R, crown radius C, and half-width b:

```text
rho(a) = R - C + sqrt(C*C - a*a),  |a| <= b
wheel(a,theta) = (a, rho(a)*cos(theta), rho(a)*sin(theta))
```

All dimensions are owner supplied. Construction requires positive finite
dimensions, b<C, and positive radius throughout the width. Only the smooth tread
interior is a supported contact feature. Sidewalls and tread edges are absent.

For a rail point transformed into wheel coordinates `(a,y,z)`, let
`r=sqrt(y*y+z*z)` and `L=sqrt(a*a+(r-(R-C))^2)`. The outer-crown distance chart is

```text
d = L - C
a_w = C*a/L
radial_normal = (r-(R-C))/L
rho_w = R-C+C*radial_normal
```

It requires r>0, r-(R-C)>0, L>0, and an accepted witness strictly inside
`abs(a_w)<b`. This is a local signed normal-distance chart, not a signed distance
to an inferred closed wheel/rail solid. The meridian's analytic extension can be
used for search seeds outside the trimmed width; only valid interior witnesses
are published. Rationalized forms of `L-C` and `rho_w` avoid cancellation for a
large crown radius and a small clearance.

The search minimizes this chart over the declared rail patch, using the existing
surface position and analytic first partials. It uses normalized patch coordinates,
a deterministic seed grid, finite differences of the search gradient for a 2x2
search Hessian, bounded Newton/descent directions, and damped line searches.
Finite differences are used only for the geometric search, not to manufacture
missing rail derivatives or contact acceleration/curvature terms.

Search-gradient stencils can shrink up to eight times when an existing numerical
surface/frame evaluation cannot resolve a trial; the same total evaluation budget
still applies. Convergence checks tangency and estimated spatial Newton correction. Distinct
resolved minima are ambiguous; a nonisolated minimum is unsupported. All seed
searches must resolve. A failed numerical trial may be backtracked within the
same budget; exhausting the search still fails. A surface query failure is never
replaced by zero force or a positive gap. Existing rail/frame/inversion failures
remain available through the exception chain.

The material normal is

```text
n = material_side * normalize(X_s cross X_v)
```

It must oppose the wheel's outward tread normal. The accepted witnesses satisfy
`p_w-p_r = gap*n` to the search accuracy. Positive gap means local clearance;
negative gap means local penetration. Wrong material sides, degenerate surfaces,
sampled intersections, coincident bindings for one wheel, patch-boundary minima,
tread-edge witnesses, and unresolved/ambiguous searches fail explicitly.

Each wheel searches independently. Heartline station is a contact coordinate,
not the body's trajectory and not rail distance. No nearest-station projection
is used to set a body pose. Stationary rail material has zero velocity:
`SectionAngularRate` describes spatial frame change per heartline metre and is
not a rail material angular velocity.

For world COM x, wheel witness p_w, world angular velocity omega, and normal n:

```text
r = p_w-x
v_point = v_COM + omega cross r
v_normal = dot(n,v_point)             // positive means separating
force = normal_force*n
COM_torque = r cross force
```

`WheelRailContactSample` stores these geometric quantities, coordinates, residual,
and evaluation count. Samples are `readonly record struct` values: immutable
snapshots, independent of subsequent simulation states.

## First-order coupled impulse method

Continuous rigid contact expresses "can push, cannot pull":
`gap>=0`, `normal_force>=0`, and `gap*normal_force=0`. Impacts additionally need
impulses to change velocity discontinuously. The initial integrator uses a
first-order discrete impulse formulation rather than an instantaneous
acceleration-contact evaluator requiring surface Hessians.

The external load callback is called once at the start time/state and must return
one world force/COM torque per body. The free velocity includes the existing
Newton-Euler gyroscopic term. For step h and stacked world-linear/body-angular
velocities u, the coupled solve is

```text
u_free = u_start + h*a_free
M*(u_end-u_free) = J_b^T*p_b + N^T*p_n
J_b*u_end = -C_b/h
w = N*u_end + gap/h
p_n >= 0, w >= 0, p_n*w = 0           // componentwise
```

`p_b` are unrestricted joint impulses and `p_n` nonnegative normal impulses.
Spherical and hinge point impulses have units N*s; hinge couple impulses have
units N*m*s. The original spherical target shown above is unchanged. New hinge
rows use their analytic acceleration bias in `J_h*u_end=-C_h/h-h*b_h`; see the
revolute foundation for this first-order extension and angular tolerances.
The contact row, using body-local arm r_local and body-to-world rotation R, is
`[n^T, (r_local cross R^T*n)^T]`. It correctly couples world translation and body
angular velocity, and its transpose supplies forces and moments in those frames.

With geometry frozen at the start configuration, this is the convex problem
`min 0.5*(u-u_free)^T*M*(u-u_free)` subject to the stated equality/inequality rows.
The internal dense active-set solver adds violated inequalities and releases
rows requiring tension. All joint/contact cross-coupling enters `A*M^-1*A^T`.
Mass/inertia applications use existing divisions and cached inertia solves;
there is no explicit matrix inverse, hidden compliance, diagonal softness, or
arbitrary distribution of wheel loads.

Cholesky uses the existing scaled-pivot and backward-error checks. Every tight
potential support row is checked for independence before publishing physical
impulses. Dependence or poor resolution raises `IndeterminateReactions`. This is
a conservative rejection policy: even unloaded redundant tight rows may fail.
It is not proof that no motion exists or that every rejected multiplier vector
is nonunique. The first stage refuses to choose a basis-dependent load split.
General preloaded/redundant assemblies need additional owner-approved physics.

Positions advance with the new linear velocities; orientations use right-multiplied
axis-angle rotations from body angular velocity. This path is first order. It
does not inherit RK4 or implicit midpoint accuracy or conservation properties.

## Collision timing, sampling, and correction limits

`gap/h` enforces linearized endpoint geometry, not exact impact time. A predictive
impulse can occur while the initial gap is still positive. A newly closed contact
can retain closing velocity, which the next step resolves. For example, 2 kg
closing at 1 m/s from 0.01 m clearance with h=0.02 s receives a 1 N*s impulse and
ends at zero gap with -0.5 m/s normal velocity. The next unloaded step supplies
the remaining 1 N*s. This is documented discrete behavior, not exact continuous
collision detection or an instantaneous impact law evaluated at the true event.

All explicitly bound pairs are queried; there is no implicit global collision
system or silently ignored distant/unsupported pair. `MaximumPointTravel` bounds
COM translation plus a wheel-extent/angular-travel estimate. `MaximumAngularStep`
bounds rotation. `TravelSamples` checks the proposed discrete path, including
its endpoint. Sampled penetration beyond the supplied correction limit fails.
There is no automatic retry or substep refinement. Callers choose/refine h.

These bounded checks can miss thin features or complete crossings between
samples. They do not certify the absence of intermediate penetration. Callers
must restrict patches and refine timesteps/search resolution for the intended
case. Failure to resolve geometry is an error, never separation. Leaving a patch
is an unsupported search-domain event, not a detected derailment or automatic seam
transition. No circuit wrapping is inferred.

Coupled mass-weighted endpoint position corrections enforce joint equalities and
contact nonpenetration inequalities, permitting separation. They requery geometry
at corrected poses, use bounded line searches, and enforce caller-supplied maximum
position/orientation correction magnitudes. Endpoint velocity correction restores
joint consistency without worsening any contact's post-impulse normal closing
velocity; it does not impose an extra physical impact on a newly closed branch.

`NumericalCorrections` records changes from the unprojected discrete endpoint.
They can change energy/momentum and are never added to reported physical loads.
Small residuals establish numerical constraints, not trajectory or force accuracy.
Large origins, extreme scales/conditioning, inaccurate frame transport, and
unresolvable parameter increments can defeat otherwise reasonable tolerances.

## Outputs, diagnostics, and failure behavior

`ContactImpulses` contains physical discrete normal/world/angular impulses at the
**start-configuration witnesses**. `AverageForce` and `AverageTorque` divide them
by h; they are step-average quantities, never instantaneous impact values.
`JointImpulses` uses world impulses on A and opposite impulses on B, with each
COM angular impulse. `AverageConstraintLoads` sums physical joint/contact
impulses per body and divides by h. `AcceptedContacts` instead describes the
accepted endpoint geometry. No instantaneous `AcceptedDynamics` contact evaluator
is supplied, and the bilateral-only endpoint evaluator is not used to invent one.

For a 2 kg central wheel initially touching and closing at 1 m/s, the stopping
impulse is 2 N*s. h=0.01 s reports an average 200 N; h=0.005 s reports 400 N. A
rigid impulse model cannot establish a physical peak impact force or duration.

Diagnostics report active-set/projection iterations, accepted minimum gap,
joint gap/slip, discrete velocity feasibility, negative impulse, scaled
complementarity, linear backward error, momentum backward error, and minimum
scaled pivot. Complementarity scales each impulse/slack by its maximum absolute
size with the corresponding tolerance as a floor. Momentum error divides the
impulse-balance residual by solved/free velocity momentum magnitudes plus absolute
row contributions, avoiding false relative errors from cancelling torques.
These backward errors are not forward-error or condition-number estimates.

`ContactSolveException` records failure, stage, iterations, residual, geometry IDs
where applicable, and the inner numerical cause. Integrator failures include
ordered `InvolvedPairs`: contact row k follows `Dynamics.BilateralConstraintCount` bilateral rows
and corresponds to pair k. Invalid input definitions/counts/controls use argument
exceptions. Solver/query failures never return a partially advanced state; input
snapshots remain unchanged. No fallback contact law or fabricated zero reaction is
used. Sorting and pure callbacks support same-runtime/platform repeatability;
cross-platform bitwise identity is not promised.

## Numerical controls

`ContactSolverSettings` requires explicit maximum point travel (m), angular step
(rad), position correction (m), and orientation correction (rad). These are
caller-selected numerical limits, not library engineering clearances.

Provisional optional defaults are:

| Control | Default | Meaning |
| --- | --- | --- |
| Geometry position tolerance | 1e-9 m | Estimated search-location resolution and coincidence checks |
| Geometry tangency tolerance | 1e-8 | Dimensionless tangent/normal and surface-regularity controls |
| Boundary fraction | 1e-7 | Excluded fraction at normalized patch boundaries |
| Search difference step | 1e-4 | Normalized-coordinate gradient-difference step |
| Seeds per axis | 3 | Deterministic grid, including bounds |
| Search iterations/evaluations | 40 / 20000 | Per pair; includes all seeds and search derivatives |
| Contact position tolerance | 1e-8 m | Accepted penetration limit; must cover geometry tolerance |
| Discrete velocity tolerance | 1e-8 m/s | Linearized constraint feasibility |
| Impulse tolerance | 1e-10 N*s | Physical multiplier checks/scaling |
| Active-set iterations | 128 | Per physical or numerical correction solve |
| Position projection iterations | 12 | Coupled position-correction budget |
| Travel samples | 4 | Equally spaced checks of the proposed discrete path |

`Joints` retains existing independent joint tolerances, Cholesky controls, and
line-search budget; their definitions and defaults are unchanged. Position
correction multipliers are numerical mass-weighted displacement quantities, not
physical impulses. All defaults are numerical regression baselines, not coaster
acceptance criteria. Dense allocation/factorization and repeated rail searches
target small systems, without a real-time performance claim.

## C# example

This complete console-style example supplies demonstration geometry, properties,
loads, and numerical controls. None is a manufacturer dimension or engineering
acceptance recommendation. The single supported wheel balances an explicitly
supplied central -20 N load, giving a 0.2 N*s impulse over 0.01 s.

```csharp
using System;
using QuantumDynamix.Geometry;
using QuantumDynamix.Math;
using QuantumDynamix.Math.Functions;
using QuantumDynamix.Physics;
using QuantumDynamix.Splines.Curves;
using QuantumDynamix.Track;

var line = SplineCurve3d.Clamped(new[] {
    Vector3d.Zero, new Vector3d(10, 0, 0) }, degree: 1);
var track = new HeartlineTrack(line, Vector3d.UnitZ,
    TrackOffsetProfile.Constant(Vector3d.Zero, 10));
var section = new RailCrossSection(new[] {
    SplineCurve3d.Clamped(new[] {
        new Vector3d(0, -0.1, 0), new Vector3d(0, 0.1, 0) }, degree: 1) });
var roll = new CustomFunction(0, 10, (_, _) => new ScalarJet(0));
var rail = new RailDefinition("rail", TrackOffsetProfile.Constant(Vector3d.Zero, 10), section, roll);
var rails = new RailGeometry(track, new RailArrangement(new[] { rail }),
    new RailValidationSettings(1e-8, 1e-7, 1e-7, 1e-7, 1e-10, 1e-8));

var bodies = new MultibodySystem(
    new[] { new RigidBodyMassProperties(2, Matrix3x3d.Diagonal(2, 3, 4)) },
    Array.Empty<SphericalJoint>());
var wheel = new WheelDefinition("wheel", bodyIndex: 0,
    new RigidPose3d(Vector3d.Zero,
        Quaterniond.FromAxisAngle(Vector3d.UnitZ, System.Math.PI / 2)),
    new CrownedWheelTread(radius: 0.2, halfWidth: 0.08, crownRadius: 0.4));
var patch = new RailContactPatch("top", "rail", boundaryIndex: 0,
    stationMinimum: 0, stationMaximum: 10, profileMinimum: 0, profileMaximum: 1,
    RailMaterialSide.PositiveAreaVector);
var contacts = new WheelRailContactSystem(bodies, rails,
    new[] { wheel }, new[] { patch }, new[] { new WheelRailPair("wheel", "top") });
var state = new MultibodyState(new[] {
    new RigidBodyState(new RigidPose3d(new Vector3d(5, 0, 0.2), Quaterniond.Identity),
        Vector3d.Zero, Vector3d.Zero) });
var controls = new ContactSolverSettings(maximumPointTravel: 0.05,
    maximumAngularStep: 0.05, maximumPositionCorrection: 0.001,
    maximumOrientationCorrection: 0.01);

WheelRailGeometryResult geometry = WheelRailContactGeometry.Query(contacts, state);
ContactStepResult result = ContactMultibodyIntegrator.Step(contacts, state,
    time: 0, step: 0.01,
    evaluateLoads: (_, _) => new[] { new RigidBodyLoads(new Vector3d(0, 0, -20), Vector3d.Zero) },
    settings: controls);
state = result.State;
Console.WriteLine($"Initial gap: {geometry.Contacts[0].Gap:G6} m");
Console.WriteLine($"Normal impulse: {result.ContactImpulses[0].NormalImpulse:G6} N*s");
Console.WriteLine($"Step-average force: {result.ContactImpulses[0].AverageForce.Z:G6} N");
```

The arrays supply explicit definitions and snapshots; constructors copy them.
The load lambda `(_, _) => ...` ignores time/state because this demonstration
load is constant. In an application, that callback supplies the owner's loads.
`state = result.State` explicitly accepts a successful immutable result.

## Regression coverage and references

`WheelRailContactTests` starts with analytic plane and rational circular-section
gaps, tilted crowned support, and material-point velocities. It extends to support,
separation, inelastic recontact, off-center effective mass, spherical-joint coupling,
supported articulated bodies with numerical projection, simultaneous independent
contacts, wheel unloading, asymmetric rails, world transforms, mass scaling,
large-crown cancellation, first-order motion and impact-timing refinement,
same-platform repeatability, immutable definitions, indeterminate supports, and
bounded geometry/solver/arithmetic failures. Existing suites remain intact.
These selected fixtures are not a validated vehicle operating envelope.

Verification on the development machine after implementation:

| Command | Result |
| --- | --- |
| `dotnet test QuantumDynamixLib.sln --configuration Debug --no-restore` | 240 passed, 0 failed, 0 skipped |
| `dotnet test QuantumDynamixLib.sln --configuration Release --no-restore` | 240 passed, 0 failed, 0 skipped |
| `dotnet format QuantumDynamixLib.sln --verify-no-changes --no-restore` | Passed |

The complete suites include 34 new contact regression cases. These commands built
the production library and test assembly in each configuration; no empty-solution
success is being counted as test coverage. Existing production source files were
not changed. No commit or push was performed.

The impulse/complementarity family is described by
[Anitescu, A Fixed Time-Step Approach for Multibody Dynamics with Contact and Friction](https://www.mcs.anl.gov/papers/P1034.pdf).
The rigid approximation's indeterminate-force limitation is discussed in
[Drake's contact modeling documentation](https://drake.mit.edu/doxygen_cxx/group__drake__contacts.html).
This implementation supplies its own frictionless, bounded first-stage adaptation;
it does not claim every capability or guarantee of those broader models.
