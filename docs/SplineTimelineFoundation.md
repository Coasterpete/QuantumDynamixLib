# Spline, timeline, heartline, and smoothing foundation

This implementation follows the owner's approved definitions. It lives in the
existing core assembly, adds no package dependencies, and leaves both existing
physics implementations intact. All examples below use library APIs, not a
separate tutorial project.

## Spline definitions and derivatives

`SplineDefinition` owns copied, read-only degree, knots, and positive finite
weights. `ScalarSpline` and `SplineCurve3d` share its local Cox–de Boor basis and
rational derivative evaluation. There is no separate Bézier authoring API.

New spatial curves default to degree five; scalar profiles default to degree
three. Clamped factories generate simple, uniformly spaced interior knots on
`[0,1]`. Supplied nonuniform knot vectors retain their domain. A degree-p curve
requires at least p+1 controls. Control points generally are not interpolation
points; clamped endpoint controls are interpolated.

```csharp
using QuantumDynamix.Math;
using QuantumDynamix.Math.Functions;
using QuantumDynamix.Splines.Curves;

var heartline = SplineCurve3d.Clamped(new Vector3d[] {
    new(0, 0, 0), new(5, 0, 0), new(10, 0, 0), new(15, 0, 1),
    new(20, 0, 2), new(25, 0, 3), new(30, 0, 4), new(35, 0, 3),
    new(40, 0, 4), new(45, 0, 4), new(50, 0, 4), new(55, 0, 4)
});
CurveJet3d sample = heartline.Evaluate(0.4);
Vector3d derivativeWithRespectToU = sample.FirstDerivative;
```

The parameter u has no inherent metre or second interpretation. `ScalarJet`
and `CurveJet3d` carry the available derivative order. Reading an unavailable
derivative throws instead of manufacturing a value. Automatic evaluation at an
interior spline knot uses its guaranteed continuity, `p - multiplicity`.
Special control configurations may actually be smoother. Use `EvaluationSide`
for explicit left/right derivatives. A value-discontinuous knot selects the
right value automatically; final endpoints use the left interior limit.

Analytic rational derivatives use the quotient recurrence. If C=A/W,
`C'=(A'-C*W')/W`; differentiating again gives
`C''=(A''-2*C'*W'-C*W'')/W`. Rational derivatives above the polynomial degree
can be nonzero. Scalar and spatial samples expose derivatives through order
three. The basis evaluator also supports the higher orders used for periodic
seam validation.

`Periodic` factories construct uniform cyclic curves and extend the unique
controls and weights by their first p entries. The exposed controls include
that extension. Explicit supplied periodic definitions must match values and
derivatives through order p-1; constructors validate this numerically. Periodic
curves still reject outside-domain evaluations. Call `WrapParameter` explicitly
or use a repeating timeline. Coincident endpoints do not imply periodicity.

Finite inputs can still lead to unresolvable arithmetic. Extreme weight ratios,
unresolved knot spans, overflow, and invalid rational denominators fail explicitly.

## Geometry and arc length

`CurveGeometry` computes the unit tangent, curvature vector `K=dT/ds`, curvature
magnitude, curvature-vector station derivative when available, and torsion when
defined. Straight lines have zero curvature and undefined torsion; stationary
parameters have no defined tangent and are rejected by differential geometry.

`ArcLengthMap` integrates `|C'(u)|` separately across knot spans with adaptive
7/15 Gauss–Kronrod quadrature. The default global numerical budget is
`1e-8 m + 1e-10 * length`; internal integrations reserve part of that budget for
inversion. The cumulative map belongs to an immutable curve snapshot.

```csharp
using QuantumDynamix.Geometry;

var stations = new ArcLengthMap(heartline);
ArcLengthInversion inverse = stations.Invert(stations.Length / 2);
double u = inverse.Parameter;
CurveGeometrySample geometry = CurveGeometry.Evaluate(heartline, u);
```

Inversion brackets a cached interval and uses Newton steps with bisection fallback.
Its result reports station residual, estimated length error, and iterations.
Zero-length spans and definitions allowing position jumps are rejected. A zero
speed at an isolated endpoint can be handled by inversion, although its tangent
remains undefined. Error estimates are not certified geometric bounds. Resource
limits and numerical tolerances are configurable and failures carry diagnostics.

## Easings and profile derivatives

`EasingKind` preserves separately selectable Linear, Quadratic, Cubic, Quartic,
Quintic, Sine, Cosine, Plateau, and Custom presets. Sine and Cosine retain their
identity while sharing the approved trigonometric mathematics:

- In: `1-cos(pi*x/2)`.
- Out: `sin(pi*x/2)`.
- InOut: `(1-cos(pi*x))/2`.

Power bases are `x^n`, n=2..5. Out and InOut use the approved reflected/scaled
composition. InOut power curves generally have a second-derivative jump at x=0.5;
automatic samples expose only first derivatives there. Quintic `x^5` does not
mean endpoint-flat interpolation.

Plateau defaults to hold start 0.25, hold end 0.75, and hold value 0.5. Its
shoulders use `Q(x)=35x^4-84x^5+70x^6-20x^7`, with the first three derivatives
zero at either end. Out reflects the base plateau. InOut composes two scaled
plateau transitions, consequently producing holds at two intermediate values.
Use In (the default) for one intermediate hold.

`RiseHoldFallEnvelope` is separate: its endpoints are zero and its hold value is
one. Both shapes can be scaled by `FunctionSegment`. A transition requires an
easing with endpoints zero and one; an envelope is supplied as a general segment.

`CustomFunction` and `EasingFunction.Custom` accept mathematical callbacks and
explicit derivative availability. Callbacks must be pure and deterministic;
captured mutable state is the caller's responsibility. Custom breakpoints and
sided derivative behavior must be supplied. No expression language or executable
serialization format is imposed, and no numerical differentiation is substituted.

## Shared timelines and anchors

`ScalarTimeline` has an explicit Time (seconds) or Distance (metres) domain.
Output units belong to the property it controls. Segments must be ordered and
contiguous. Keyframes use linear transitions unless an easing is supplied.

```csharp
using QuantumDynamix.Math.Timelines;

var envelope = new RiseHoldFallEnvelope();
var command = new ScalarTimeline(ProfileDomain.Time,
    new[] { new FunctionSegment(0, 4, envelope, scale: 2) },
    BoundaryPolicy.Zero);
ScalarJet atOneSecond = command.EvaluateAtTime(101, originTime: 100);
```

`FunctionSegment` applies the chain rule: its r-th derivative scales by
`amplitude / duration^r` (or distance-span^r). `ProfileKinematics.ToTimeDerivatives`
converts q(s) using caller-supplied signed station speed, acceleration, and jerk:

```text
q_t   = q_s * v
q_tt  = q_ss * v^2 + q_s * a
q_ttt = q_sss * v^3 + 3*q_ss*v*a + q_s*j
```

This conversion does not integrate or prescribe physical motion.

Boundary policy defaults to Reject. Hold, Zero, and Repeat are explicit options.
An intentional value jump requires `allowDiscontinuities: true`. Hold/Zero/Repeat
joins participate in derivative validation. `ValidateContinuity` inspects segment
joins and reported internal breakpoints; -1 denotes a value jump. Its numerical
comparison tolerance is configurable. Automatic samples at joins retain the
right value and only the derivatives available across that join.

`SpatialAnchor.BetweenFeatures` stores two stable feature IDs and a fractional
distance. `SpatialAnchor.AtDistance` explicitly stores a fixed metre station.
`AnchoredTimeline.Resolve` produces a fresh distance timeline from `FeatureStations`;
it does not mutate authored keys. Missing, deleted, or reversed features fail
explicitly. `TrackFeature` associates a stable ID with an authored curve parameter;
the caller must preserve or update that association when changing parameterization.
Loop wrapping does not infer lap identity. Time origins are explicit scalar
arguments, and evaluating a profile does not advance a clock.

## Heartline and track spine

`HeartlineTrack` uses the authored heartline as canonical geometry. Its
`RotationMinimizingFrames` transport starts from an explicitly supplied up vector,
projected perpendicular to the starting tangent. Axes are forward/left/up;
positive bank is right-hand rotation about forward. Banking is in unwrapped
radians and offsets are explicitly supplied in metres.

Frames use adaptively subdivided double-reflection transport with an angular
error estimate. The estimate is not a rigorous bound. Closed-loop twist is
reported through `ClosureTwist`; no correction is distributed automatically.
Frame transport requires a regular, at least C1 curve at interior knots.

Build an `ArcLengthMap` first to obtain the exact station-domain endpoints used
by the numerical snapshot, then supply banking and offset functions covering
that domain. `TrackOffsetProfile.Constant` supports explicitly supplied constant
offsets; three scalar functions support varying forward, left, and up offsets.

```csharp
using QuantumDynamix.Track;

var offsets = TrackOffsetProfile.Constant(new Vector3d(0, 0, -1), stations.Length);
// This offset is an illustrative caller input, not a library heartline-height default.
var track = new HeartlineTrack(heartline, Vector3d.UnitZ, offsets);
HeartlineTrackSample trackSample = track.Evaluate(stations.Length / 2);
```

The derived spine is `C=H+R*d`. Its first and second derivatives are with respect
to heartline station, including bank and varying-offset derivatives. The RMF's
angular change per metre is `T x K`, with banking adding `beta_s*T`.
`IntegrateSpineLength` integrates the resulting spine speed rather than equating
spine length to heartline length. Its quadrature error excludes frame and station
inversion approximation errors. The derived spine is not refitted to a NURBS.
No rail geometry, gauge, vehicle trajectory, or contact behavior is inferred by
`HeartlineTrack`. The separately configured, owner-approved rail layer is described
in [Physical rail geometry](PhysicalRailGeometryFoundation.md).

## Fitting and nondestructive smoothing

`ConstrainedSplineFitter` fits scalar or spatial samples with fixed knots and
weights. Sample weights multiply squared residuals. Constraints impose value or
parameter-derivative equalities, including feature preservation. Consistent
redundant constraints are supported; inconsistent and rank-deficient systems are
distinguished. Equality elimination and reduced least squares use pivoted
Householder QR, without forming normal equations. The implementation is dense
and intended for moderate fitting problems.

`SplineSmoothing` returns the original, a separate candidate on success,
diagnostics, roughness estimates, and sampled comparisons. It never modifies
source controls or silently relaxes constraints. Candidates retain the same
degree, knots, weights, and periodic definition.

```csharp
using QuantumDynamix.Splines.Fitting;

var result = SplineSmoothing.SmoothCurve(heartline,
    new SmoothingSettings(strength: 1e-6),
    lockedControlPoints: new[] { 0, heartline.ControlPoints.Count - 1 });
if (result.Diagnostics.Status == FitStatus.Success)
{
    SplineCurve3d candidate = result.Candidate!;
    // The owner/application chooses whether to accept this separate candidate.
}
```

Strength is nonnegative and dimensionless after normalization. Zero returns the
original exactly after feasibility checks. Default roughness order is two for
scalar profiles and three for spatial curves; an explicit order overrides it.
`ValueScale` is a caller-specified output scale. Geometry uses derivatives with
respect to normalized *original* arc length, keeping the fitting problem quadratic;
it does not optimize derivatives against the candidate's changing arc length.

Region endpoints use the source spline argument u (or the scalar profile argument).
Convert a station selection using `ArcLengthMap.Invert` before smoothing geometry.
Controls whose support reaches outside a selected region are frozen. Boundary
values and derivatives through order three are locked by default. A narrow region
can have no adjustable coefficients; the frozen count and fit ranks expose this.
Existing low-continuity features are preserved rather than automatically repaired.
Explicit control locks preserve their coefficient values exactly.

The fidelity and roughness integrals use 15-point Kronrod collocation on subdivided
knot spans. `QuadratureSubdivisions` controls resolution; this fixed quadrature has
no adaptive error certificate. Increase it and compare candidates when rational
weights or parameterization vary sharply. Custom nonnegative roughness weights
receive the source argument. Roughness and maximum sampled deviation are estimates,
not continuous-domain guarantees. Constraint tolerances are relative numerical
residual controls, not physical feature-deviation acceptance criteria.

The initial system supports linear equality constraints, locks, and custom
quadratic roughness weighting. It does not implement nonlinear constraints, SQP,
automatic knot refinement, or simulation-driven G-force/jerk optimization.
Curvature comparisons diagnose changes; derivative fairing alone does not establish
dynamic smoothness. Acceleration/G-force targets can be scalar profiles, while
physical acceleration and reactions remain outputs of the existing simulation.

## Verification and scope

Tests include polynomial reproduction, rational derivatives, exact circular arcs,
nonuniform/repeated/periodic knots, analytic lengths, safeguarded inversion,
frame convergence, banked-offset geometry, easing joins, anchor re-resolution,
boundary policies, constrained fitting, smoothing locks and regions, quadrature
refinement, and profile loads through both existing physics integrators.

No FVD reconstruction, launch/brake actuators, wheel/rail models, train animation,
GUI, or rendering dependencies are introduced.

Mathematical references:

- [Cox–de Boor basis definition](https://pages.mtu.edu/~shene/COURSES/cs3621/NOTES/spline/B-spline/bspline-basis.html).
- [NURBS homogeneous formulation](https://pages.mtu.edu/~shene/COURSES/cs3621/NOTES/surface/NURBS-def.html).
- [Gauss–Kronrod quadrature](https://www.boost.org/doc/libs/1_86_0/libs/math/doc/html/math_toolkit/gauss_kronrod.html).
- [Double-reflection rotation-minimizing frames](https://www.microsoft.com/en-us/research/publication/computation-rotation-minimizing-frames/).
- [Weighted spline smoothing](https://ftp.cs.wisc.edu/Approx/smooth.pdf).
