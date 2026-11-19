# Revolute joints and independent wheel spin

This implements the owner-approved general-purpose revolute foundation in the
existing core assembly. Each independently spinning wheel is an explicit finite-mass
rigid body with owner-supplied full inertia. A revolute joint connects it to its
carrier. The existing Newton-Euler equations provide its gyroscopic dynamics.
No package, assembly, friction, traction, braking model, suspension, manufacturer
geometry, rendering, or UI is added. The README and its examples are unchanged.

## Definitions and frames

`RevoluteJoint(BodyA, BodyB, LocalFrameA, LocalFrameB)` stores two existing body
indices and two `RigidPose3d` attachment frames. Each position is an offset from
that body's center of mass in metres. Each orientation maps attachment-frame
directions into that body's coordinates. Local +X is the directed axle; local Y/Z
establish zero angle. Positive relative rotation is B turning about +X by the
right-hand rule. Attachment origins coincide and the two directed axles align;
their transverse axes may rotate freely relative to one another.

The origins may be anywhere on the intended bearing axis, including away from a
body's COM. Joint frames need not coincide with body principal-inertia frames.
There is no inferred track-relative frame, world-up direction, wheel count,
wheelbase, gauge, or axle spacing. These joints can connect bodies other than wheels.
All existing bodies still require positive finite mass and positive-definite full
COM inertia. A fixed-world body is not introduced.

`MultibodySystem(bodies, joints)` retains its exact public signature and creates
an empty revolute collection. The additional `(bodies, joints, revoluteJoints)`
constructor copies both collections. `Joints` remains the spherical collection;
`RevoluteJoints` is a separate read-only collection in caller order. State body
indices retain their existing meaning. Definitions reject invalid body indices,
self-joints, and invalid/default attachment poses. Default structs bypass C#
constructors, so frame validation also occurs when a system copies definitions.
The public `readonly record struct` joint/sample/reaction types are immutable value
snapshots with generated value equality. New accepted states and result objects
replace caller references; they do not mutate the supplied state collections.

`BilateralConstraintCount` counts scalar equations: three per spherical joint,
followed by five per revolute joint. Normal contact rows follow these bilateral
rows in the rigid contact solve. There is no public arbitrary-matrix constraint API.

## Five constraints and their derivatives

Let x be world COM position, R a body-to-world rotation, and s a local attachment
origin. Let a_A/a_B be the world attachment +X axes and y_A/z_A be A's transverse
axes. The three position equations and two dimensionless alignment equations are:

```text
C_point = x_A + R_A*s_A - x_B - R_B*s_B = 0       // m, three rows
C_y     = dot(y_A, a_B) = 0                      // two dimensionless rows
C_z     = dot(z_A, a_B) = 0
dot(a_A,a_B) > 0                                // directed branch
```

A free pair has six relative degrees of freedom. Coincident attachment points
remove three translations; aligned axles remove two tilts. Axial rotation remains.
The dot equations alone also admit antiparallel axles. Explicit branch validation
rejects those configurations rather than silently flipping a frame or reporting
zero alignment error. The local chart becomes singular at perpendicular axles.
A positive squared axle dot at or below double machine epsilon is rejected as
numerically singular; scaled Cholesky can reject conditioning earlier. This is a
numerical chart-resolution guard, not an engineering angular clearance.

The generalized velocity u stacks world linear velocity and body angular
velocity, six components per body. For either transverse vector t=y_A or z_A,
set k=t cross a_B. All omega values in the following formulas are world vectors:

```text
C_t_dot = dot(k, omega_A - omega_B)
J_t_A_angular = transpose(R_A)*k
J_t_B_angular = -transpose(R_B)*k

b_t = dot(omega_A cross (omega_A cross t), a_B)
    + 2*dot(omega_A cross t, omega_B cross a_B)
    + dot(t, omega_B cross (omega_B cross a_B))

C_ddot = J*uDot + b
```

The first and last terms differentiate each rotating direction twice; the middle
term differentiates both directions once. They are the rotational counterpart of
the existing attachment centripetal terms. Production rows and biases are analytic.
Independent regressions difference geometry-only constraint values under world
translations, right-multiplied body rotations, and constant-body-angular-velocity
flows. They check all Jacobian columns and the second derivative, including
slightly off-manifold states and a 500 rad/s relative spin.

All joint rows enter the existing mass response and Schur system:

```text
M = block_diagonal(m_i*identity, I_body_i)
W = J*M^-1*transpose(J)
W*lambda = -(J*a_free + b)
uDot = a_free + M^-1*transpose(J)*lambda
```

The first three hinge multipliers are force components in N; the last two are
couple coefficients in N*m. Diagonal-scaled Cholesky and cached inertia solves
remain in use. Mixed units are not combined in a raw residual norm. Redundant
bilateral constraints continue to fail; there is no regularization or chosen
basis-dependent reaction split.

## Wheel state, inertia, and mass accounting

The wheel's existing `RigidBodyState` owns its independent pose and angular
velocity. Its existing `RigidBodyMassProperties` owns mass and full COM inertia.
`WheelDefinition.BodyIndex` references that wheel body, and `LocalPose` continues
to map tread coordinates into its COM frame. For a coaxial installation, the
tread centerline and hinge axle must describe the same physical axis. Definitions
do not infer that installation or silently move geometry into alignment.

The owner must partition mass consistently. When a combined carrier/wheel body
is split, the carrier properties must exclude the separately represented wheel.
If the combined COM changes, recalculate COM offsets and inertia with the proper
parallel-axis terms; simply subtracting unrelated COM tensors is insufficient.
Geometry contains no mass. The library can validate positivity and symmetry but
cannot infer whether supplied carrier properties already include a wheel, or
whether a tensor matches a real component. Contact regressions explicitly use
2 kg carrier + 0.5 kg wheel = 2.5 kg and apply their 20 N + 5 N loads separately.

Axial COM inertia is dot(a_body, I_body*a_body). A scalar relation I*spinDot=T is
only a suitable reduction for appropriate fixed-axis/principal-axis conditions.
An off-COM axle adds orbital inertia, and a moving finite carrier is dynamically
coupled. The production solver uses the full equations in all cases.

## Angle, rate, and phase

`RevoluteJointKinematics.Evaluate(system, state, jointIndex)` uses the revolute
collection's index. For an assembled joint it returns:

```text
WorldAxis   = a_A
AngularRate = dot(a_A, omega_B_world - omega_A_world)     // rad/s
Angle       = atan2(dot(z_A,y_B), dot(y_A,y_B))           // (-pi,pi] rad
```

Never subtract angular-velocity components from different rotating body frames.
The query transforms through each body pose first. At a slightly misaligned
trial state the projected rate is a diagnostic extension; it is not exactly the
derivative of the off-manifold angle chart. Invalid axle-chart branches fail.
Equivalent quaternion signs yield the same angle and rate.

Orientation is the authoritative physical state. Cumulative turns cannot be
recovered from one orientation snapshot. `UnwrapAngle(previousUnwrappedAngle,
currentWrappedAngle)` chooses the closest 2*pi branch. It requires caller-maintained
history updated only after accepted steps and true relative phase changes
strictly below pi. It rejects an exactly ambiguous half turn, non-finite input,
out-of-range wrapped input, or an unrepresentable accumulated change. Endpoints
alone cannot detect missed complete turns. Very large accumulated doubles lose
phase resolution; applications needing that range should retain a turn counter.
Do not update history inside load callbacks: Newton evaluates repeated and
off-manifold trial states. Wrapped-angle-dependent loads must also respect their
branch cut and the solver's smooth-callback requirement.

Changing the axle phase of an axisymmetric tread while retaining the same world
center, axle, COM velocity, and world angular velocity preserves the surface and
the contact-point velocity field. It changes which material point occupies a
location. Phase regressions recompute body angular components from the fixed
world vector rather than accidentally changing physical angular velocity.

## Bearing reactions and gyroscopic coupling

For world force F on A and angular multipliers lambda_y/lambda_z:

```text
mu_A = lambda_y*(y_A cross a_B) + lambda_z*(z_A cross a_B)
F_B = -F_A
mu_B = -mu_A
COM_torque_A = (R_A*s_A) cross F_A + mu_A
COM_torque_B = (R_B*s_B) cross F_B + mu_B
```

`RevoluteJointReaction` reports `ForceOnA/B` in N, independent `CoupleOnA/B`
in N*m, and `TorqueOnA/B` about the respective COMs in N*m. Only the independent
couple is perpendicular to the common axle on a consistent joint. A force's
lever-arm moment can have an axial component. Ideal joint power is zero when
attachment velocities agree and relative angular velocity is axial.

`ConstrainedDynamicsResult.RevoluteJointReactions` is instantaneous at the
evaluated snapshot. `ConstraintLoads` sums all bilateral reactions per body;
`JointReactions` retains its spherical-only meaning. Midpoint accepted reactions
are recomputed at the accepted endpoint under endpoint loads.

The existing Euler equation is unchanged:

```text
I_body*omegaDot_body = torque_body - omega_body cross (I_body*omega_body)
```

Tilting a spinning wheel changes its angular momentum direction. The hinge
supplies transverse torque and the carrier receives the opposite couple. No
additional artificial gyroscopic torque is applied. An exact instantaneous
regression uses centered bodies, carrier inertia K*identity, wheel inertia
diag(Iax,J,J), omega_A=Omega*e_z and omega_B=s*e_x+Omega*e_z. With no external loads:

```text
alpha_A_y = -Iax*s*Omega/(K+J)
mu_B_y    = K*Iax*s*Omega/(K+J)
```

Both bodies have finite supplied properties. Reversing spin reverses the gyro
reaction. General full-tensor regressions check force/moment balance and ideal
joint power. Total angular momentum includes COM spin and x cross (m*v).

## Integration and normal contact

The bilateral and compliant paths retain projected implicit midpoint, analytic
quaternion midpoint kinematics, finite-difference Newton, and mass-weighted
position/velocity projection. Both point and angular hinge blocks are projected.
The position merit function compares actual axle angle to its angular tolerance.
Angular row rates have units 1/s and equal forbidden angular speed at alignment.
Projection corrections retain their numerical meaning and may change energy or
momentum. Compliant energy diagnostics already include every body's kinetic
energy and separately report projection-induced kinetic and elastic changes.

The rigid path retains its first-order normal impulse formulation and original
spherical-joint discrete equations. For a new hinge, the known analytic bias
enters the first-order bilateral velocity target:

```text
J_hinge*u_end = -C_hinge/h - h*b_hinge
```

At consistent initial data this imposes J*delta_u/h+b=0 to first order. Including
the bias ensures the physical hinge impulse contains the turning/gyroscopic
reaction instead of shifting its leading contribution into numerical endpoint
velocity correction. No rail acceleration/Hessian or new normal contact law is
inferred. The rigid path still uses gap/h for approximate contact timing.

`ContactStepResult.RevoluteJointImpulses` reports world `ImpulseOnA/B` in N*s,
independent `CoupleImpulseOnA/B` in N*m*s, and COM `AngularImpulseOnA/B` in N*m*s,
all at the start configuration. `AverageConstraintLoads` sums physical spherical,
revolute, and normal impulses divided by h. Endpoint numerical corrections are
excluded. These quantities are not instantaneous or peak impact forces.

Contact active-set feasibility uses separate linear and angular tolerances.
`MaximumVelocityViolation` remains an m/s quantity. New diagnostics report
`MaximumAngularVelocityViolation` (1/s), `ScaledVelocityViolation` (dimensionless),
and accepted `MaximumJointAngularGap` (rad) / `MaximumJointAngularSlip` (1/s).
Angular couple impulses are excluded from force-impulse complementarity scales.
Numerical position-projection dual floors use displacement tolerance divided by
the relevant inverse effective mass, giving kg*m rather than reusing an N*s
impulse tolerance. These numerical multipliers are never physical loads.

Rigid contact still requires nonpenetrating initial wheels. Compliant contact
retains physical compression and its explicit power law, one patch per wheel,
normal dissipation, and maximum-compression failure. Only joint errors are
projected in the compliant path. Typed mixed-formulation exclusion remains active.

## Why a free wheel does not automatically roll without slip

For a stationary rail the unchanged contact evaluator uses:

```text
r = wheel_point - wheel_COM
v_relative = v_COM + omega_world cross r
v_tangent = v_relative - normal*dot(normal,v_relative)
```

No-slip rolling additionally requires v_tangent=0. In a simple upright circular
wheel case this reduces to v-r*omega=0. A free bearing permits spin but does not
enforce that condition. For example, radius 0.2 m and speed 10 m/s would require
50 rad/s to roll without slip; a frictionless wheel initialized at zero spin can
continue sliding at zero spin. Suitable matching initial conditions can also
continue rolling freely. No tangential force is invented to establish either case.

A coaxial axisymmetric tread's normal force has zero axle moment. It may still
exert other moments and participate in full carrier/wheel gyroscopic coupling.
Normal damping is not tangential friction or rolling resistance. The current
compliant law has no steady rolling loss at constant compression. Existing load
callbacks can accept explicit equal-and-opposite shaft torques, but this change
adds no actuator/brake/bearing-loss model. Future traction/slip laws require a
separate owner-approved tangential force or impulse formulation.

## Numerical controls and limits

`AngularConstraintSettings` supplies provisional defaults of 1e-10 rad axle
alignment tolerance and 1e-10 1/s angular-row rate tolerance. Both must be finite
and positive, with orientation tolerance below pi/2. The original
`ConstrainedSolverSettings` constructor is preserved exactly; a new overload
accepts these angular controls followed by the existing optional controls.
Its `AngularConstraints` property exposes the immutable settings.

Existing diagnostic positional constructors retain their signatures and linear
units. `ConstraintDiagnostics` adds maximum angular position, velocity, and
acceleration errors. `ConstrainedStepDiagnostics` adds pre-projection angular
position and velocity errors. Acceleration residuals use the two dimensionless
alignment rows and have units 1/s^2. No angular acceleration residual is fabricated
for the rigid impulse path.

Initial data must satisfy both point and angular tolerances. Unsupported directed
branches use `UnsupportedJointConfiguration`; singular charts/rows retain
`SingularConstraints`. Failed linear/Newton/projection solves and unsupported
arithmetic return no advanced state. Mixed projection failure residuals are
tolerance-scaled; spherical-only projection failures retain metre/metre-per-second
residuals. No automatic assembly, step retry, subdivision, stiffness, or damping
is added.
The rigid contact entry point retains its existing wrapping of bilateral failures
as `ContactSolveException(LinearSolve)`, with the located `ConstrainedSolveException`
available as the inner cause. The compliant wrapper retains bilateral exception
types directly, as before.

Small constraint errors do not establish trajectory or reaction accuracy.
For constant principal-axis spin, the existing midpoint quaternion increment is
4*atan(h*omega/4), approaching h*omega under refinement. Fast spin therefore still
requires short steps. Contact travel guards count full body angular travel,
including phase rotation of an axisymmetric tread. The first-order rigid path's
explicit free dynamics can require more restrictive steps for gyroscopic motion.
Geometry witness errors also affect velocity by roughly |omega| times position
error, and rigid step-average force error can amplify velocity error by mass/h.

The existing bounded rail/contact searches retain their numerical limitations,
including unresolved frame evaluations next to cached parameters at finite
world-coordinate resolution. Searches do not become continuous collision
detection. The new fixtures keep isolated witnesses away from such cached-node
resolution failures. No geometry failure is replaced with separation or zero force.
Dense solves and repeated contact searches remain intended for small systems.

## Regression evidence

The new suites cover analytic rows/bias, off-manifold perturbations, full tensors,
five-row freedom, signed phase/rate, angle history, finite-carrier gyroscopes,
shaft torques, mass/inertia ratios, power/momentum, mixed joints, fast-spin
refinement, rigid/compliant bearing loads, material-point velocity, axisymmetric
phase invariance, normal oscillation, projection energy, formulation exclusion,
immutable definitions/results, and explicit failures. Fixtures are synthetic;
numerical thresholds are regression margins, not coaster acceptance criteria.

Measured on the development machine in both Debug and Release:

| Fixture | Result |
| --- | --- |
| Independent geometry perturbations, all tested Jacobian entries | Maximum absolute numeric entry difference 6.86e-11 |
| Analytic bias versus geometric second differences, including 500 rad/s | Maximum normalized difference 2.86e-9 |
| 500 rad/s, 0.08 s, h=0.0008/0.0004/0.0002 s | Accumulated phase errors 0.1325390 / 0.03328342 / 0.008330210 rad |
| Same fast spin, default angular tolerances | Maximum alignment 1.47e-14 rad; maximum row-rate residual 2.01e-12 1/s |
| Same fast spin, tolerances tightened to 1e-12 | Same displayed phase errors; maximum row-rate residual 9.70e-13 1/s |
| Full-tensor coupled pair, h=0.02/0.01/0.005 s to 0.4 s | Scaled state errors 7.808586e-4 / 1.934616e-4 / 4.612545e-5 |
| Same pair, 1 s at h=0.005 s | Maximum relative energy drift 8.12e-7; angular-momentum drift 8.84e-5 kg*m^2/s |
| Spinning wheel/carrier normal oscillator, halved steps | Scaled motion errors 1.490184e-3 / 3.729190e-4 / 9.325308e-5 |
| Rigid hinge refinement against fine midpoint reference | Scaled errors 0.06776821 / 0.03354826 / 0.01668898, approximately first order |
| 500 rad/s, rigid 25 N support, h=0.0002 s | Maximum normal step-average load error 5.00e-7 N in ten steps |
| Turning compliant wheel, h=0.001 s | Projection kinetic change -7.11e-12 J; elastic change -1.44e-14 J |

Jacobian comparisons use each entry's native fixture units. Bias comparisons
normalize each native row by `1 + abs(bias) + speed^2` using explicit 1 s and
1 m point-row fixture scales (unit alignment for angular rows). Full-tensor
state comparisons use 1 m, 1 m/s, and 1 rad/s scales; angular orientation is
compared through unit directions. The compliant oscillator uses 0.01 m and
0.1 m/s scales. The rigid refinement uses 1 m, 1 m/s, and 1 rad/s scales.
The reference integrations are numerical comparisons, not analytical solutions.

The fast-spin comparison justifies keeping the angular defaults provisional:
the tested constraints are comfortably below 1e-10 while halving h controls
the much larger physical phase error. Tightening projection tolerance alone
does not improve that phase accuracy. The rapid-spin rigid support regression
allows a 5e-6 N average-load margin around its measured witness-sensitive error;
lower-spin fixtures use 2e-7 N. Neither is a universal force-error guarantee.
Full-tensor drift thresholds are 1e-4 relative energy and 1e-4 kg*m^2/s angular
momentum, with actual observations shown above. These measurements characterize
the selected cases, not real-wheel material behavior or an operating envelope.

Final verification built and tested the production library and MSTest assembly:

| Command | Result |
| --- | --- |
| `dotnet test QuantumDynamixLib.sln --configuration Debug --no-restore` | 331 passed, 0 failed, 0 skipped |
| `dotnet test QuantumDynamixLib.sln --configuration Release --no-restore` | 331 passed, 0 failed, 0 skipped |
| `dotnet format QuantumDynamixLib.sln --verify-no-changes --no-restore` | Passed |
| `git diff --check` | Passed |

The test runs used detailed console logging to retain fixture measurements.
The total includes all 272 existing cases and 59 new revolute/wheel-spin cases.
The README was not changed, and no commit or push was performed.
