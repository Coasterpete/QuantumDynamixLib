# Force-based Region authoring

Create a track with a prescribed forward speed. In the Region List, choose **Force-based** in **Type**, then choose **Append Region** or **Prepend Region**. The new Region appears immediately in the Region List, Transition Editor, and 3D view. **Adjoining** uses the adjoining Region’s mode, and the first Region then uses coordinate acceleration. The first Region needs a length and start station. Later Regions use the adjoining Region’s length when **Length** is blank. Prepend uses at most the heartline before the first Region. The **Region** inspector beside the Transition Editor shows the selected Region’s mode and edits its name and heartline length.

Select **Normal force (g)**, **Lateral force (g)**, or **Banking (rad)** in the Transition Editor. Use **Keyframes** to add or drag a key or **Transitions** to edit the mathematical spans between keys. Dragging previews the heartline, spine, and rails live. Release commits the edit and rebuilds them. [Direct graph editing](MathematicalTransitionAuthoring.md#direct-graph-editing) The readout gives calculated normal and lateral specific force at the selected keyframe, at the midpoint of a selected Transition, or at the Region start when neither is selected. The plotted profiles are authored targets. [Mathematical Transition authoring](MathematicalTransitionAuthoring.md) · [FVD++ documentation](https://lucasbosch.de/nolimits-tools/FVD%2B%2B/older_versions/fvd%2B%2B_0.5_documentation.pdf)

## Coordinates and units

- **World** uses a right-handed frame with +Z up. Zero heading points along +X and positive heading turns toward +Y. World gravity is constant at `(0, 0, -g₀)`, with `g₀ = 9.80665 m/s²`. [NIST standard gravity conversion](https://www.nist.gov/pml/special-publication-811/nist-guide-si-appendix-b-conversion-factors/nist-guide-si-appendix-b9)
- **Unbanked track** uses forward (the unit heartline tangent), left, and up, stored as local X, Y, and Z. The orientation is transported with zero twist about the tangent. Its up axis travels with the track and can point away from world up.
- **Rider and banked track** share the same axes at the heartline in this model. Positive bank rotates left toward unbanked up about forward. Positive lateral specific force points along banked left and positive normal specific force points along banked up. Bank is an unwrapped angle in radians.
- Stations and Region lengths are heartline arc length in metres. The coordinate-acceleration profiles remain in m/s² and the force profiles express specific force in multiples of `g₀`.

## Reconstruction and continuity

Specific force is `f = a − gravity`, the non-gravitational force per unit mass. At each integration stage the solver projects gravity onto the current banked left and up axes. It multiplies each authored force value by `g₀` and adds the corresponding gravity component to obtain transverse coordinate acceleration. With prescribed constant speed `v`, heartline curvature is `(aLeft * bankedLeft + aUp * bankedUp) / v²`. The existing quaternion Runge–Kutta integration advances the unbanked frame over station, and position quadrature integrates the unit tangent. Geometry-derived readings evaluate the reconstructed curvature, multiply by `v²`, subtract world gravity, and project the result back into the banked frame. For a straight, level, unbanked track this gives 1 g normal and 0 g lateral. A straight incline starts with the gravity projection appropriate to its pitch.

Append inherits the adjoining bank and the transverse values expressed in the new Region’s mode. At a mixed-mode join, the downstream opening keys are linked to the incoming transverse acceleration and are recalculated after upstream edits. The rest of the downstream keys retain their authored values. Prepend integrates toward decreasing station from the existing adjoining world pose, keeping the rider axes oriented toward increasing station and gravity fixed in world space. Position, tangent, bank, and curvature are continuous at accepted joins within the reconstruction tolerances. Higher derivative continuity depends on the profiles. Undo and Redo restore the resolved profiles and corresponding geometry together, and a failed rebuild keeps the previous committed state.

This model calculates the kinematics of an ideal point at the heartline with prescribed speed. Keeping speed constant on an incline requires a longitudinal specific force of `−dot(gravity, forward)`. The authoring model imposes that speed and does not model its physical source. Vehicle motion, wheel–rail loads, energy changes, and ride validation require separate models.

## Generic upright wireframe

Authored tracks use a generic two-rail upright cross-section for the preview. The dimensions describe centerlines relative to the evaluated heartline, in metres along the banked track axes.

| Centerline | Forward | Left | Up |
| --- | ---: | ---: | ---: |
| Heartline | 0 | 0 | 0 |
| Left rail | 0 | +0.50 | −0.75 |
| Right rail | 0 | −0.50 | −0.75 |
| Spine | 0 | 0 | −1.00 |

The rail midpoint is 0.75 m below the rider heartline along local up, and the spine sits another 0.25 m below that midpoint. The rail centerlines are 1.00 m apart along local left. These illustrative dimensions apply only to the authored wireframe. The quarter-circle development fixture uses its own radial offsets.

For a straight, level Region at zero heading and bank, **Top** projects the spine onto the heartline with a rail on each side. **Front** is the side view and projects both rails at the same height between the heartline and spine. **Left** looks along the track and projects each longitudinal line to its cross-section position. In **Perspective**, the same separation can look larger at the near end because the display scale varies with camera depth.
