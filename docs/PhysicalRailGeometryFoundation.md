# Physical rail geometry foundation

This implements the owner's approved rail-geometry architecture in the existing
core assembly. It adds no packages or rendering dependencies. Rail geometry is
stationary spatial data queried by callers; the rigid-body and constrained
multibody solvers remain responsible for physical motion and reactions.

## Definitions and coordinates

`RailArrangement` accepts any nonempty collection of uniquely identified
`RailDefinition` objects. One rail, two rails, asymmetric arrangements, and more
rails use the same evaluator. There are no manufacturer, gauge, profile, or
spine-structure presets.

Each rail definition holds spine-relative `TrackOffsetProfile` translations,
a `RailCrossSection`, and an explicit `IScalarFunction` section-roll profile.
Translations are metres in the existing banked **heartline** forward/left/up
axes. Section roll is in radians. All profile arguments are heartline distance
in metres, including forward translations. Distance timelines, custom analytic
functions, and scalar B-splines/NURBS can supply profiles. Time timelines are
rejected by the rail binding. Profiles must cover the complete heartline range.

The heartline remains the authored B-spline/NURBS; its existing degree-five
spatial default is unchanged. The spine and rails are directly evaluated offsets,
not fitted replacement splines. For heartline station s, banked frame R(s),
heartline-to-spine offset d(s), and spine-to-rail offset a_i(s):

```text
P(s)   = H(s) + R(s) d(s)
r_i(s) = H(s) + R(s) [d(s) + a_i(s)]
```

Definitions copy collection inputs and expose read-only views. Existing scalar
function objects are retained. Custom callbacks must remain pure, deterministic,
and unchanged for the life of geometry and distance maps; captured mutable state
cannot be made immutable by copying a containing collection.

## Analytic centerline derivatives

`TrackOffsetGeometry` shares the existing offset mathematics with
`HeartlineTrack.Evaluate`. With q=d+a_i, D=Rq, T=dH/ds, K=dT/ds, bank beta, and
angular change omega=T cross K + beta_s T:

```text
r_s  = T + omega cross D + R q_s
r_ss = K + omega_s cross D + omega cross (omega cross D)
         + 2 omega cross (R q_s) + R q_ss
omega_s = T cross K_s + beta_s K + beta_ss T
```

The cross product is `Vector3d.Cross`; `frame.Transform(local)` multiplies the
local forward/left/up components by the corresponding world axes. The factor two
in the second derivative follows the product rule for the two changing factors.
Banking and changing rail/spine offsets all contribute their supplied derivatives.
Production evaluation does not substitute finite differences.

`RailSample.Centerline` is a `CurveJet3d` with available derivative order 0, 1,
or 2. Primes here are with respect to heartline distance, **not rail distance**.
The rail's own tangent and curvature vector use v=|r_s|:

```text
T_rail = r_s / v
K_rail = [r_ss - T_rail dot(T_rail,r_ss)] / v^2
```

The vector has units 1/m; its length is curvature. Accessing unavailable geometry
throws a located `RailGeometryException`, rather than returning an invented zero.
Second rail derivatives require third heartline derivatives and second bank/offset
derivatives. A regular C3 heartline with C2 profiles is a sufficient continuity
condition. At repeated knots and profile joins, `EvaluationSide.Automatic` retains
the guaranteed input derivative availability; explicit Left/Right requests expose
one-sided limits. Custom functions must truthfully declare their availability and
all breakpoints. Rail third derivatives, torsion, and surface Hessians are not
part of this second-order foundation.

## Cross-section frame convention

`RailCrossSection` contains one or more supplied spatial B-spline/NURBS boundaries
whose controls lie exactly in local X=0. Their Y/Z coordinates are section-left/up
metres. Named datums are also local X=0 points. Open boundaries and sharp corners
are permitted. Winding, material sides, boundary correspondence, and whether a
datum lies on a physical surface belong to the owner.

For each rail sample:

1. Forward is the rail's own unit tangent t.
2. Left is normalized projection of banked heartline-left L perpendicular to t.
   The equivalent vector triple product `(t cross L) cross t` is used for numerical
   perpendicularity near parallel directions.
3. Up is forward cross left.
4. The explicit section roll rotates left toward up about forward, using the
   existing positive right-hand banking convention.

The projected direction must exceed the caller's `MinimumFrameProjection`.
There is no world-up fallback or axis switch. A failed section frame does not
invalidate otherwise regular centerline position, curvature, or distance queries.
The frame convention is implemented by the evaluator, not encoded into the rail
definitions, leaving those definitions usable by a future explicitly approved
frame convention.

`SectionAngularRate` is analytically differentiated world angular change per
**heartline metre**, including section-roll change. It is not angular velocity.
For a fixed section boundary c(v), `EvaluateSurface` supplies:

```text
X(s,v) = r(s) + Q(s)c(v)
X_s    = r_s + omega_section cross [Q c(v)]
X_v    = Q c_v
```

`RailSurfaceSample.AreaVector` is X_s cross X_v; it has no inferred outward
material-side meaning. Angular rates and surface station partials require second
centerline derivatives and a first section-roll derivative.

## Independent distances and measurements

`CreateSpineDistanceMap()` and `CreateRailDistanceMap(id)` construct separate
`TrackDistanceMap` objects integrating their own speed |r_s|. The existing spline
`ArcLengthMap` API remains unchanged. Derived maps split at spline knots, profile
breakpoints, and authored features; use adaptive 7/15 Gauss-Kronrod quadrature;
and invert with bracketed Newton steps and bisection fallback.

`StationAtHeartlineStation` returns derived metres. `Invert` returns a
`TrackDistanceInversion` containing **HeartlineStation**, station residual,
estimated quadrature error, and iteration count. `ResolveFeatures` returns those
same feature IDs measured along the selected spine or rail. Queries reject
out-of-range inputs; neither wrapping nor a lap identity is inferred.

Quadrature defaults remain the established 1e-8 m absolute and 1e-10 relative
controls. Cached and partial integrals reserve budget for inversion. Returned
quadrature errors exclude heartline inversion and frame approximation errors.
`ReferenceStationErrorBudget` reports the configured reference inversion budget;
`EstimatedFrameAngularError` reports the cached RMF transport estimate. Neither is
a certified bound on derived rail position or length. Quadrature limits apply per
integration call, consistent with the existing numerical facility.

`RailSpacingDefinition` explicitly selects two rails, an expected metre-valued
profile, and a measurement:

- `Euclidean`: nonnegative point separation at corresponding heartline stations.
- `BankedLateral`: signed projection of B-A onto banked heartline-left.

Omitting datums measures center spacing. Supplying two named section datums
measures the explicitly declared gauge, including each rail's section roll.
These are corresponding-station measurements, not shortest distances between
curves or solids. A zero datum gap is permitted; a negative Euclidean expectation
is invalid. No minimum physical clearance or manufacturer gauge is assumed.

## Validation and numerical limits

`RailValidationSettings` requires caller-supplied position tolerance (m), angular
tolerance (rad), curvature tolerance (1/m), station-derivative tolerance
(m per heartline m), minimum speed ratio (rail m per heartline m), and minimum
projected-left length (dimensionless). Curvature tolerance also compares section
angular-rate changes in rad/m, with radians dimensionless. Regression fixture
values are numerical tests, not coaster engineering acceptance criteria.

`Validate` reports located `RailDiagnostic` records for known join discontinuities,
singular/poorly resolved centerlines, projected-left singularities, degenerate
profiles, local swept-section Jacobian failures, coincident rail centers, spacing
mismatches, unavailable derivatives, arithmetic failures, and sampling exhaustion.
Validation never moves, smooths, regularizes, or repairs a definition.

It inspects both sides of known joins and samples each boundary knot span. Across
heartline spans it adaptively subdivides for nonlinear speed variation, tangent
and section-axis changes, and analytic section angular rate. Angular rate helps
avoid aliasing complete section turns from endpoint orientations alone.
`SamplesPerSpan`, `MaximumDepth`, `MaximumEvaluations` (rail-station samples), and
sampling tolerances bound that inspection. Profile inspection uses the same
per-span sample count; it is not included in the rail-station evaluation count.
Known-join checks are separate from that adaptive count.

An empty diagnostic list establishes only that these checks passed at their
inspection resolution. It does not certify absence of all interior singularities,
cross-section self-intersections, inter-rail surface overlap, global rail
self-intersections, or clearance violations. Local Jacobian checks are sampled.

`Validate(closedCircuit: true)` additionally compares the actual rail positions,
station derivatives, tangents, curvature vectors, section axes, and section angular
rates across the circuit seam. Both first and second heartline-station derivatives
are compared, including tangential metric changes that curvature alone would miss.
Periodic heartline controls alone are insufficient:
RMF holonomy, authored banking, offsets, and section roll must combine consistently.
Unwrapped bank/roll differences of complete turns are valid when the evaluated
geometry agrees. RMF `ClosureTwist` remains reported without automatic correction.

`ValidateJoin(next)` checks independently authored sections by stable rail ID.
Boundary correspondence uses collection index and normalized profile parameter;
it does not infer equivalent reparameterizations or stitch geometry. Its diagnostic
tolerances are those of the receiving/current geometry. Missing rail IDs and
incompatible profile boundaries are reported explicitly.

Relative offset differences are retained when measuring gauge and checking joins,
so a large world origin does not conceal a small authored gap. Returned absolute
positions still have ordinary double-precision world-coordinate resolution.

## Production API usage

Given the owner's authored track, rail offsets, sections, roll, expected center
spacing, and validation settings:

```csharp
using QuantumDynamix.Track;

var arrangement = new RailArrangement(
    new[] {
        new RailDefinition("rail-a", offsetsA, sectionA, rollA),
        new RailDefinition("rail-b", offsetsB, sectionB, rollB)
    },
    new[] {
        new RailSpacingDefinition("centers", "rail-a", "rail-b",
            RailSpacingMeasurement.Euclidean, expectedCenterSpacing)
    });
var rails = new RailGeometry(track, arrangement, validationSettings);
RailSample sample = rails.Evaluate("rail-a", heartlineStation);
TrackDistanceMap railDistances = rails.CreateRailDistanceMap("rail-a");
double railStation = railDistances.StationAtHeartlineStation(heartlineStation);
TrackDistanceInversion inverse = railDistances.Invert(railStation);
IReadOnlyList<RailDiagnostic> diagnostics = rails.Validate(closedCircuit: true);
```

These inputs intentionally have no library geometry defaults. The array initializer
creates the supplied collection; the classes copy it. A sample's `readonly record
struct` stores a small value snapshot. Geometry queries never write a body pose,
velocity, load, or constraint. No contact, suspension, friction, train animation,
or vehicle dynamics is implemented by this layer.
