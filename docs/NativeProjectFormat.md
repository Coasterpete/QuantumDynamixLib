# Native QUANTUM DYNAMIX projects

Use **File → New Track** or the existing **New Track** button to create a project. **Save Project** prompts for a location the first time and then writes to that location. **Save Project As** chooses a new destination. **Empty Workspace** and the File menu examples leave the active project available through **Return to Active Track**.

An asterisk in the title bar marks unsaved changes. Undo and Redo recognize the saved revision. Opening or creating another project, or closing the editor with changes pending, offers **Save**, **Discard**, or **Cancel**. If a save is cancelled or fails, the pending replacement or close is cancelled. An unsuccessful Open leaves the current track and its editing history available.

## Version 1

A `.qdproject` file is a UTF-8 JSON object with `format` set to `"quantum-dynamix-project"`, integer `version` set to `1`, and one `track` object. The reader accepts an optional UTF-8 byte order mark. All fields listed below are required, and names and identifiers are case-sensitive.

| Location | Fields and meaning |
| --- | --- |
| `track` | `name`, `reconstructionSpeedMetresPerSecond`, `startingFrame`, `reconstructionAnchor`, and the ordered `regions` array. |
| Each frame | `positionMetres`, `forward`, `left`, and `up`, each an object containing `x`, `y`, and `z`. The three directions are a right-handed orthonormal basis in world coordinates. |
| `reconstructionAnchor` | `stationMetres` and the unbanked world `frame` used for the saved reconstruction. The station is the first Region's start or, immediately after a prepend, the first join. A track with no Regions stores `null`. |
| Each Region | Nonempty GUID `id`, `name`, `startMetres`, `endMetres`, `mode` (`"coordinateAcceleration"` or `"force"`), and the ordered `profiles` array. |
| Each profile | `channel`, `interpolation` (`"linear"`), and ordered `keyframes`. |
| Each keyframe | `stationMetres` and `value`. The first and last keys must match the Region endpoints exactly. Interior stations strictly increase. |

`startingFrame` preserves the starting position and the orientation defined by the New Track heading and pitch. An unbanked frame carries that orientation even at vertical pitch. The reconstruction anchor preserves the world pose the editor held fixed during the last reconstruction. It can differ from the starting frame after prepending or removing Regions. Both are inputs to future authoring. Region length is `endMetres - startMetres`. Storing the actual endpoints preserves the exact stations and shared boundaries after a resize.

The required banking channel is `Banking (rad)` and stores unwrapped radians. Coordinate-acceleration Regions use `Left accel. (m/s²)` and `Up accel. (m/s²)` in metres per second squared. Force-based Regions use `Lateral force (g)` and `Normal force (g)` in multiples of standard gravity. Custom channels retain their editor names and values. Each linear span uses the existing scalar timeline interpolation between its two keys. See [Force-based Region authoring](ForceRegionAuthoring.md) for the frames, signs, and units.

The application reads and writes finite double-precision numbers with round-trip precision. It rebuilds timelines, derives the linked opening transverse keys at mixed-mode joins, and reconstructs heartline, spine, rails, and calculated forces through the same pipeline used during editing. Opening selects the first Region when present and starts a fresh Undo/Redo history. Viewport camera settings, selection, wireframe samples, and calculated force readouts are session state.

The current application reads version 1 with the original linear interpolation. It also writes version 1 for projects whose profiles and Region modes are fully representable in that format. The development examples are separate workspaces.

## Version 2

Version 2 keeps the same envelope, frames, Regions, and keyframes. The writer uses it when the project contains a Geometric Region or a profile with a function other than the original Linear easing. The following fields extend the version 1 schema.

| Location | Version 2 addition |
| --- | --- |
| Region `mode` | `"geometric"` requires `Pitch Change (°/m)`, `Yaw Change (°/m)`, and `Banking (rad)`. |
| `reconstructionAnchor` | Optional `geometricHeadingRadians` in [−π, π]. Preserves the carried heading when re-anchoring a Geometric profile. If omitted the anchor frame supplies it. |
| Profile `interpolation` | `"linear"` keeps version 1 behavior. `"piecewise"` requires an ordered `transitions` array with exactly one object per consecutive pair of keyframes. The keys give each span's exact stations and values. |
| Transition | `type` is `"constant"`, `"linear"`, `"sine"`, `"cosine"`, `"quadratic"`, `"cubic"`, `"quartic"`, `"quintic"`, or `"plateau"`. For easings, `direction` is `"in"`, `"out"`, or `"inOut"`. A Constant takes only `type` and has equal adjacent key values. |
| Plateau | A Plateau transition also requires a `plateau` object with `holdStart`, `holdEnd`, and `holdValue` fractions. |

The project reader rejects unknown or duplicate fields, unsupported types and versions, and input that fails timeline or reconstruction validation. Opening restores the authored functions and rebuilds geometry and geometric readouts at the saved reconstruction speed. [Mathematical Transition and Geometric Region conventions](MathematicalTransitionAuthoring.md)

## Version 3

Version 3 extends version 2 with `track.operationalSegments`. The writer uses it for projects with at least one operational segment. Track-only projects continue to use the earliest version that represents their Regions and reconstruction anchor.

| Location | Version 3 addition |
| --- | --- |
| `operationalSegments` | Array of disjoint intervals, written in increasing station order. |
| Each segment | Unique nonempty GUID `id`, `startMetres` and `endMetres` on the authored heartline, and `type` (`"normalTrack"` or `"transport"`). |
| `device` | Required on a Transport segment. Contains `type` (`"idealizedPoweredTransport"`), `entrySpeedMetresPerSecond`, optional `speedLimitMetresPerSecond`, and `accelerationRegions`. |
| Each Acceleration Region | `keyframes` with `offsetMetres` from the segment start and `accelerationMetresPerSecondSquared`. An ordered `transitions` array uses the version 2 Transition schema. The first and last keys define the interval. |

The loader validates device and interval settings along with the track. It restores the authored command timelines. Longitudinal motion is calculated on request in Track Segment Settings. [Operational track segment and device conventions](OperationsAuthoring.md)

Save writes and flushes a new file in the destination directory before renaming it over the destination. This keeps the existing file in place if preparing the replacement fails. The desktop picker requires a filesystem path so the replacement can take place in the same directory.
