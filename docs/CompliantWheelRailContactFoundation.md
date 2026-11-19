# Compliant wheel/rail normal-contact foundation

This implements the owner-approved optional normal-compliance formulation in the
existing core assembly. The rigid impulse solver remains a separate formulation.
No dependencies, assemblies, suspension mechanisms, friction, material presets,
or vehicle-guidance rules were added by the normal law. Explicit independently
spinning wheel bodies and general-purpose revolute joints are supported through
the separate [revolute foundation](RevoluteJointFoundation.md).

## Physical scope

`CompliantWheelRailContactModel` references the existing immutable
`WheelRailContactSystem` and assigns an explicit `PowerLawNormalContact` to every
bound pair. Collections are copied and exposed read-only. Evaluation follows the
geometry system's ordinal pair order, independent of input assignment order.

The first model permits one bound patch per wheel. Multiple wheels may attach to
one body or to articulated bodies. Independent springs on multiple contacts of
one deforming tread could double-count shared deformation and are not supported.
Rails remain stationary and rigid; wheel geometry remains rigidly mounted to its
existing COM frame. Owner-supplied body mass and full inertia retain their current
meaning. There are no additional deformation coordinates. Independent wheel
spin resides in the wheel body's existing orientation/angular velocity; the
normal law adds no separate spin coordinate or inertia.

All existing geometric restrictions still apply: isolated smooth tread-interior
witnesses, explicit rail material sides, restricted smooth patches, bounded
searches, and explicit failure at unresolved branches, edges, corners, seams, or
unsupported chart locations. There is no inferred closed rail solid or global
collision detector. See [the rigid contact foundation](WheelRailContactFoundation.md)
for the shared signed-gap geometry.

## Force law, signs, and units

The unchanged geometry gives signed gap `g` in metres: positive means clearance,
zero means touching, and negative means nominal overlap. The normal `n` points
out of rail material toward the wheel. For a stationary rail:

```text
r = wheel_point - body_COM
v_point = v_COM + omega_world cross r
v_n = dot(n, v_point)                    // m/s; positive is separating
delta = max(0, -g)                       // m; compression
deltaDot = -v_n                          // m/s; approach rate
```

`deltaDot` is the compression derivative on the compressed branch. At clearance
the response still records the signed approach rate; it cannot generate force.
Overlap represents local deformation omitted from the rigid geometry. It is
never automatically projected back to zero as rigid penetration.

The approved compression-only power law is:

```text
Fe = K * delta^p
Fn = Fe * max(0, 1 + alpha*deltaDot)       // delta>0; otherwise Fn=0
U  = K * delta^(p+1) / (p+1)
kt = p*K*delta^(p-1)                      // compressed elastic tangent
force = Fn*n
COM_torque = r cross force
```

| Parameter/output | Units | Meaning |
| --- | --- | --- |
| `Coefficient` K | N/m^p | Positive owner-supplied force coefficient |
| `Exponent` p | Dimensionless | Finite exponent >=1 |
| `Dissipation` alpha | s/m | Finite nonnegative damping parameter |
| `MaximumCompression` | m | Positive physical validity limit, not a solver tolerance |
| `NormalForce`, `ElasticForce` | N | Instantaneous compressive forces |
| `ElasticEnergy` U | J | Recoverable energy of the elastic component |
| `ElasticTangentStiffness` kt | N/m | Elastic slope, excluding velocity-dependent damping |
| `DissipationPower` | W | Nonnegative loss rate in the selected model |

There are no default physical coefficients. K is not Young's modulus except
through a separately justified derivation. The implementation does not infer K
or alpha from timestep, mass, desired penetration, hardness, a settling target,
or numerical tolerances. A maximum-compression limit rejects evaluation beyond
the declared domain; it does not cap force or soften the law.

At delta=0, force, energy, and physical dissipation are exactly zero for every
finite input rate. The compressed-side elastic slope is K when p=1 and zero
when p>1. Exponents below one would produce singular elastic slope at first
touch and are rejected. A linear unilateral law still has a slope discontinuity
across first touch; clipping adds another piecewise-smooth boundary.

Construction checks that force, energy, and elastic tangent resolve at the
maximum compression. Evaluation rejects non-finite rates and out-of-domain
compression. Positive force/energy/stiffness/loss quantities that overflow or
round to zero fail rather than silently removing support. A logarithmic fallback
handles a power or scaling intermediate that underflows/overflows when the final
positive quantity can still resolve. Extreme scales, subnormal arithmetic,
finite-difference resolution, and poorly conditioned geometry still limit
accuracy; finite values are not a condition-number or engineering-accuracy
certificate. These are numerical checks, not an approved operating envelope.

## Unloading, separation, and dissipation

Compression can only push. Loading increases the resisting force. Unloading
reduces it, and rapid unloading clips it to zero instead of allowing attraction.
Zero force while still compressed is a possible clipped unloading state, not
proof of geometric separation. At positive clearance force is zero even for a
large closing velocity. Recontact develops finite force as compression develops;
there is no simultaneous rigid stopping impulse or separate restitution rule.

The stored energy satisfies dU/ddelta=Fe. With stationary rails, contact power
on the rigid bodies is `-Fn*deltaDot`, giving the continuous identity:

```text
d(kinetic + U)/dt = external_power - physical_loss
physical_loss = (Fn - Fe)*deltaDot
              = alpha*Fe*deltaDot^2       // unclipped branch
              = -Fe*deltaDot              // clipped unloading; deltaDot<0
```

Both loss branches are nonnegative. Clipped unloading dissipates stored elastic
energy while exerting no resultant force; that is an explicit feature of this
reduced model, not a resolved viscoelastic recovery process. Frequent clipping
or use outside a calibrated rate range warrants reassessing the physical model.
Alpha is not a Kelvin-Voigt damping coefficient in N*s/m and does not independently
specify a constant coefficient of restitution.

Hunt-Crossley-style compression-weighted damping and non-attractive clipping are
described in [Drake's compliant contact documentation](https://drake.mit.edu/doxygen_cxx/group__compliant__contact.html).
The library uses its own explicit coefficients; it does not adopt heuristic
penetration-based parameter selection.

## Load distribution and preload

Every force follows its local compression and approach rate. Geometry, rigid-body
translation/rotation, applied moments, and joint coupling determine these values
together. The implementation never divides a total bogie load by wheel count.

For three symmetric collinear supports with equal undeformed clearances, equal
outside stiffnesses, and no applied overturning moment, a common settlement gives:

```text
delta = W/(k1+k2+k3)
F1=k1*delta, F2=k2*delta, F3=k3*delta
```

The synthetic regression uses W=4000 N and stiffnesses (1,2,1)*10^6 N/m, giving
delta=0.001 m and forces (1000,2000,1000) N. The rigid equivalent continues to
raise `IndeterminateReactions`. Other regressions verify that clearance unloads
the middle wheel, a pitch/moment equilibrium leaves one support unloaded, and
opposing running/upstop contacts retain compressive preload from installed
geometry. No additive force at positive clearance creates preload.

Compliance does not ensure a unique stable equilibrium for arbitrary assemblies.
Small clearance or stiffness differences can significantly change individual
loads. Redundant bilateral joints remain unsupported. Initial assembly and
equilibrium are caller responsibilities; no static assembler or automatic
settling procedure is supplied.

## Coupling and integration

`CompliantWheelRailContact.Evaluate` is a pure geometry-to-force evaluator. It
returns ordered immutable force snapshots and resultant `RigidBodyLoads` per
body, using the existing off-center force/COM-torque operation. It does not
advance time or solve an independent load-distribution problem.

`CompliantContactMultibodyIntegrator.Step` combines these loads with the caller's
external loads inside the existing `ConstrainedMultibodyIntegrator.Step`:

```text
M*uDot = external + gyroscopic_terms + N^T*Fn + J^T*lambda
J*uDot + joint_bias = 0
```

N maps generalized velocity to contact normal velocity; J remains the existing
bilateral Jacobian, including spherical and revolute rows. World linear velocity and body angular velocity,
right-handed rotations, full body inertia, normalized physical quaternion views,
and SI double-precision calculations retain their existing meanings. Contact
loads affect all joint reactions through the existing coupled solve. No compliant
contact row is added to rigid equality/inequality enforcement.

The existing implicit midpoint/Newton solve reevaluates geometry and forces at
trial states. Callbacks must be deterministic and side-effect free and may be
called repeatedly at the same time and different joint-off-manifold states.
The external callback must exclude the contact loads already supplied by this
wrapper. A domain or geometry failure in a trial rejects the operation; no
extrapolation, retry, timestep subdivision, or force-law fallback is hidden.

Only bilateral joints receive endpoint position/velocity projection. Their
numerical corrections can change wheel compression and stored energy. The
existing joint step result now retains its unprojected endpoint internally so
the wrapper can inspect that state directly; its existing public API and
integration equations are unchanged.

The wrapper checks caller-selected point travel, angular travel, and joint
position/orientation correction bounds. Angular bounds must be below pi to
resolve the sampled rotation branch. It samples a linear-COM/shortest-axis
rotation interpolation between the initial and unprojected endpoints, with
linearly interpolated velocities; its midpoint matches the solver's physical
midpoint to roundoff. It queries the unprojected and accepted endpoints as well.
These checks can miss events between samples and do not certify a continuous
trajectory, intermediate constraint satisfaction, or absence of tunneling.

`CompliantContactStepSettings` contains only numerical controls. `Geometry` and
`Joints` retain the existing settings types and defaults. Four movement/correction
bounds must be supplied explicitly in m/rad; `TravelSamples` defaults to four.
There is no compliant penetration tolerance, automatic stiffness adjustment,
default timestep, or numerical damping masquerading as physical dissipation.

## Outputs and energy diagnostics

`InitialContacts`, `MidpointContacts`, `UnprojectedContacts`, and `AcceptedContacts`
identify the states at which forces were evaluated. All forces are instantaneous
model values, not impulses or timestep averages. `JointStep.AcceptedDynamics`
contains instantaneous accepted-endpoint joint reactions/derivatives under the
combined loads; its `ConstraintLoads` still contains bilateral joint loads only.

The energy diagnostics use kinetic plus contact elastic energy. External
potential energy is not included; supplied conservative loads, such as gravity,
contribute through external work. The diagnostics report:

```text
E0   = initial kinetic + initial contact U
Eraw = unprojected kinetic + unprojected contact U
E1   = accepted kinetic + accepted contact U
Wmid = h*external_power_at_midpoint
Dmid = h*physical_loss_at_midpoint
integration_residual = Eraw - E0 - Wmid + Dmid
projection_kinetic_change = Taccepted - Traw
projection_elastic_change = Uaccepted - Uraw
```

Wmid and Dmid are midpoint quadrature estimates, not exact integrals. The
integration residual can include discretization, nonlinear/linear solve,
geometric, and roundoff effects. Joint projection is reported separately and
never converted to physical loads or damping. No energy residual is silently
corrected. Without active contact the rail supplies no force; with contact the
stationary rail exchanges momentum with the bodies, so total body momentum
alone is not generally conserved.

## Timestep and material limitations

The local frequency scale is approximately sqrt(kt/meffective) rad/s. Effective
mass includes rotational and joint response; multiple contacts create coupled
modes. Stiffer contacts require shorter resolved timescales. Implicit midpoint
does not make large-step force peaks or transition timing accurate and does not
automatically suppress unresolved high-frequency oscillations. Its ordinary
projection is not symplectic or exactly energy conserving.

Finite-difference Newton also reevaluates geometric searches for every perturbed
velocity unknown. Dense solves, repeated searches, and allocation target small
systems. There is no real-time claim. Query-location error changes force roughly
by kt times gap error; existing geometry tolerances are not force tolerances.
Refine timestep, geometric resolution, and numerical controls for the intended
case, checking individual loads and energy rather than constraint residuals alone.

Linear p=1 can represent a calibrated force-compression approximation. p=1.5 is
compatible with suitable localized Hertz elastic contact, but selecting that
exponent does not validate Hertz assumptions. A spherical specialization gives
K=(4/3)*Eeffective*sqrt(Reffective), with elastic moduli in Pa and radii in m.
Crowned-wheel/tubular-rail contact generally needs both principal curvatures and
their orientation. Existing rail first partials and geometric-search Hessians
do not establish an automatic Hertz coefficient. See
[Simbody's contact-law explanation](https://simbody.github.io/simbody-master-doxygen/api/classSimTK_1_1HuntCrossleyForce.html).

Polymer treads may be nonlinear, temperature/rate dependent, viscoelastic finite
layers bonded to stiff cores. This stateless resultant law resolves no contact
patch pressure, tread stress, thermal field, wear, shared rail flexibility, or
material history. Steady rolling at constant total compression has zero loss in
this law, although material cycles through a real contact patch. Rolling
hysteresis resistance is therefore absent. These limitations matter for
[polyurethane wheel self-heating and damping](https://repository.tudelft.nl/record/uuid:b34750ac-ea71-47e9-95cd-c86bc4c80771).

## Formulation exclusivity and failure behavior

Each stepping entry point selects a single formulation for its bindings. A scoped
guard rejects typed compliant evaluation inside rigid enforcement of the same
binding and rejects nested rigid enforcement during compliant stepping. It keys
bindings by the same dynamics/rail instances and wheel/patch IDs, including
separate contact-system wrappers around those instances. Scope cleanup allows
independent rigid/compliant comparisons on the same definitions afterward.

Arbitrary external `RigidBodyLoads` values have no provenance. The guard cannot
identify manually encoded or precomputed duplicate contact forces, renamed
aliases of the same physical contact, or impure callbacks that deliberately
bypass the model. Callers must not add those loads or simultaneously enforce the
same physical contact elsewhere. No accepted API combines rigid enforcement and
compliant forces for a pair.

`ContactSolveFailure.CompressionLimit` identifies excess physical compression,
with wheel/patch IDs and compression in metres. `MixedFormulations` identifies
conflicting typed operations. Existing geometry failures remain explicit.
Unrepresentable law quantities use `NumericalFailure`; existing joint singular,
Newton, linear solve, and projection failures retain `ConstrainedSolveException`
and their original diagnostics. Invalid definitions/counts/settings/times use
argument exceptions. No failed query or step returns fabricated separation, zero
reaction as a substitute for failure, or a partially advanced input snapshot.

## Complete C# example

This example is exercised by `DocumentedSingleSupportExampleBalancesExplicitLoad`.
All dimensions, properties, loads, and numerical bounds are synthetic fixtures,
not manufacturer data or engineering acceptance recommendations. A 2000 N/m
linear contact compressed 0.01 m balances the explicitly supplied 20 N load and
stores 0.1 J. No gravity constant or up axis is imposed by the library.

```csharp
using System;
using System.Collections.Generic;
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
var section = new RailCrossSection(new[] { SplineCurve3d.Clamped(new[] {
    new Vector3d(0, -0.1, 0), new Vector3d(0, 0.1, 0) }, degree: 1) });
var roll = new CustomFunction(0, 10, (_, _) => new ScalarJet(0));
var rails = new RailGeometry(track, new(new[] { new RailDefinition("rail",
    TrackOffsetProfile.Constant(Vector3d.Zero, 10), section, roll) }),
    new(1e-8, 1e-7, 1e-7, 1e-7, 1e-10, 1e-8));
var bodies = new MultibodySystem(new[] {
    new RigidBodyMassProperties(2, Matrix3x3d.Diagonal(2, 3, 4)) },
    Array.Empty<SphericalJoint>());
var wheel = new WheelDefinition("wheel", 0,
    new(Vector3d.Zero, Quaterniond.FromAxisAngle(Vector3d.UnitZ, System.Math.PI / 2)),
    new(0.2, 0.08, 0.4));
var patch = new RailContactPatch("top", "rail", 0, 0, 10, 0, 1,
    RailMaterialSide.PositiveAreaVector);
var pair = new WheelRailPair("wheel", "top");
var geometry = new WheelRailContactSystem(bodies, rails,
    new[] { wheel }, new[] { patch }, new[] { pair });
var model = new CompliantWheelRailContactModel(geometry, new[] {
    KeyValuePair.Create(pair, new PowerLawNormalContact(
        coefficient: 2000, exponent: 1, dissipation: 0, maximumCompression: 0.02)) });
var state = new MultibodyState(new[] { new RigidBodyState(
    new(new Vector3d(5, 0, 0.19), Quaterniond.Identity),
    Vector3d.Zero, Vector3d.Zero) });
var controls = new CompliantContactStepSettings(0.05, 0.05, 0.001, 0.01);

CompliantContactEvaluation sample = CompliantWheelRailContact.Evaluate(model, state);
CompliantContactStepResult result = CompliantContactMultibodyIntegrator.Step(
    model, state, time: 0, step: 0.001,
    evaluateLoads: (_, _) => new[] {
        new RigidBodyLoads(new(0, 0, -20), Vector3d.Zero) }, settings: controls);
state = result.State;
Console.WriteLine($"Normal force: {result.AcceptedContacts.Contacts[0].Response.NormalForce:G6} N");
Console.WriteLine($"Gap: {result.AcceptedContacts.Contacts[0].Geometry.Gap:G6} m");
Console.WriteLine($"Stored energy: {result.AcceptedContacts.ElasticEnergy:G6} J");
```

The dictionary-like assignment binds an immutable law to an explicit geometry
pair. `KeyValuePair.Create` preserves that association without inferring wheel
roles. `(_, _) => ...` is a load callback ignoring its time/state arguments in
this constant-load example. `readonly record struct` force/response values are
immutable snapshots; new step results replace the caller's state reference.
Only pure callbacks are suitable for repeated Newton evaluations.

## Regression evidence

`CompliantWheelRailContactTests` contains analytic law/energy/tangent checks,
non-attractive loading/unloading/clipping, finite zero limits, extreme arithmetic,
three-support sharing, clearance and moment unloading, geometric preload,
articulated reactions, torque, world-transform behavior, immutable definitions,
formulation exclusion, failure atomicity, and the complete example above.

The linear central-contact oscillator checks independently derived position and
velocity and approximately fourfold error reduction when halving h on its smooth
branch. Linear and p=1.5 elastic recontact cases compare sampled peak compression
with energy conservation:

```text
delta_peak = ((p+1)*m*v_in^2/(2*K))^(1/(p+1))
```

Those cases also check unloading, separation, rebound speed, and energy error.
They do not establish peak-force accuracy outside the tested synthetic cases.
A nonlinear dissipative case checks positive physical loss, decreasing mechanical
energy, and refinement of the accumulated midpoint energy balance. An articulated
case measures projection-induced kinetic and elastic changes separately.
Regression tolerances are numerical checks, not wheel-design acceptance criteria.

Measured Debug fixture results on this machine:

| Check | Measurement |
| --- | --- |
| Smooth oscillator, h=0.006/0.003/0.0015 s | Scaled motion errors 5.936980e-3 / 1.490184e-3 / 3.729190e-4 |
| Linear elastic recontact, h=0.001 s | Sampled peak 0.003162112813 m; analytical peak 0.003162277660 m |
| Same linear recontact | Maximum energy error 1.254038e-6 J against initial 0.01 J |
| p=1.5 elastic recontact, h=0.001 s | Sampled peak 0.004352861426 m; analytical peak 0.004352752816 m |
| Same nonlinear recontact | Maximum energy error 6.542462e-7 J against initial 0.01 J |
| Nonlinear damping, h=0.004/0.002 s | Accumulated midpoint energy-balance error 9.913859e-7 / 2.496898e-7 J |
| Articulated projection, h=0.02 s | Elastic-energy change -8.013458e-10 J; kinetic change near roundoff |

The motion error sums gap and velocity errors divided by the fixture's explicit
0.01 m and 0.1 m/s scales. These observations characterize these cases only;
they do not validate actual wheel stiffness, damping, impact peaks, or runtime
performance. The complete suite includes the existing rigid-contact regressions.

Verification completed for this implementation:

| Command | Result |
| --- | --- |
| `dotnet test QuantumDynamixLib.sln --configuration Debug --no-restore` | 272 passed, 0 failed, 0 skipped |
| `dotnet test QuantumDynamixLib.sln --configuration Release --no-restore` | 272 passed, 0 failed, 0 skipped |
| `dotnet format QuantumDynamixLib.sln --verify-no-changes --no-restore` | Passed |

The full suites built the production and test assemblies and include 32 new
compliant-contact cases. No empty-solution success is counted as verification.
