# Wave + Pins

## Purpose and scope

Wave + Pins is an opt-in native FDM overhang experiment for supported-anchor geometry,
including curved and perforated surfaces. It replaces eligible
overhanging model paths with ordered, laterally growing Wave paths and reduces
the ordinary support contact area underneath those paths to discrete footprints.
The footprints are built by the existing Snug support generator. They are not
separate free-standing pin meshes or a new tree-support generator.

There is no contact-interface segmentation mode and no Python plugin. The
implementation lives in `src/libslic3r/WavePins/`; its callers provide model paths,
layer masks and native support contacts. The user-visible version is
`2.5.0-dev-wave-pins.0.2`.

The integration baseline is OrcaSlicer main commit
`baff031626ae0035d1761d4f690bd7d5e406303e`. The Wave algorithm and clipping approach
are adapted from Dennis Klappe's OrcaSlicer-WaveOverhangs commit
`0e44f2bcef5e38c84607f3c955a03629342cdc32`, with attribution to Janis A. Andersons
and Steven McCulloch retained in the source under AGPL-3.0-or-later.

## Settings and eligibility

The process settings have a dedicated **Wave + Pins (experimental)** page, after
the upstream pages. All eleven experimental settings are on that page, grouped
into the master switch, Wave paths, support contacts and upper-layer protection.
The ordinary Support page retains its upstream contents. The master switch
is visible in Simple mode and defaults to disabled. Detailed controls appear
when it is enabled, subject to the usual Advanced/Expert visibility settings.
The settings are object configuration options and can be saved in process presets
and projects. Enabling the experiment does not silently change ordinary support
settings.

| Configuration key | Default | Meaning |
| --- | --- | --- |
| `wave_pins_enabled` | `false` | Master switch. |
| `wave_pins_filter_supported` | `true` | Keep native paths on shallow Wave candidates. |
| `wave_pins_max_span` | 12 mm | Maximum unsupported reach from the model below. |
| `wave_pins_speed` | 2 mm/s | Upper speed limit for Wave paths. |
| `wave_pins_spacing` | 0 | Automatic spacing, 85% of local Wave extrusion width. |
| `wave_pins_outer_walls` | 1 | Ordinary outer wall loops preserved at each boundary. |
| `wave_pins_pin_spacing` | 8 mm | Target spacing along the inset contact boundaries. |
| `wave_pins_pin_diameter` | 2 mm | Requested retained contact footprint diameter. |
| `wave_pins_fan_speed` | -1 | Inherit filament cooling, including native overhang/bridge control; 0-100 explicitly overrides the part fan on Wave paths. |
| `wave_pins_floor_layers` | 3 | Solid, speed-limited model layers above Wave footprints. |
| `wave_pins_floor_speed` | 20 mm/s | Upper speed limit in those protected layers. |

Validation requires enabled Normal automatic support with Snug style, a positive
Top Z distance, and at least one top support interface layer. This version accepts
one nozzle and one filament, with raft, spiral vase and belt printing disabled.
Overhang wall detection must be enabled, the wall count must accommodate the
preserved outer walls, and Z anti-aliasing must be disabled. Pin spacing must
exceed pin diameter; diameter must be at least the nozzle diameter. All experiment
settings have bounded ranges. A nonzero Wave spacing must not exceed the resolved
local Wave extrusion width; invalid geometry parameters raise an error instead
of creating a disconnected Wave sheet.

These defaults are initial experiment settings, not a calibrated material
profile. Wave speed and upper-layer speed are caps: existing lower native speed
or volumetric-flow limits still apply. Fan override affects the part cooling fan,
not the auxiliary fan. The feature introduces no nozzle temperature override.

## Model path generation

The shallow-overhang filter operates on the candidate inside the preserved outer
walls. It excludes protrusions no wider than half the local Wave extrusion width
relative to the previous model layer. Classification is local: a steep section
does not cause an entire connected shallow rim to become Wave. The retained
regions extend back to the actual lower-layer boundary and include the nearby
supported anchor band; expanding the lower-layer mask for classification never
creates fictitious anchors. Native paths and support remain outside emitted
coverage. This is a geometric heuristic, not a calibrated material overhang limit.
Disabling the filter restores the original candidate construction.

`PerimeterGenerator::apply_extra_perimeters` calls `WavePins::apply_to_island`
after the ordinary Classic or Arachne wall generation for an island. It supplies
the actual local bridge flow, lower-layer geometry, preserved wall count,
resolution and cancellation callback. With the switch disabled, the upstream
extra-perimeter branch runs unchanged.

The generator intersects candidate geometry with the layer below to find
supported seeds. Wavefronts grow laterally from these seeds, subject to both
distance and iteration limits. Islands without a suitable anchor or beyond the
reach limit keep their native paths. Holes remain excluded from the candidate
geometry. Splits used internally to traverse narrow geometry are algorithmic
wavefront operations, not segmented support contact sheets.

Only emitted path footprints count as coverage. A path's swept extrusion width
is intersected with the eligible model area; an unfilled candidate region never
authorizes support removal. The module clips overlapping inner model paths and
fill surfaces, while retaining the requested outer wall loops. Replacements
which cannot preserve affected nonplanar path information are declined as a
whole. The original island remains intact when no usable Wave paths are produced.

Wave paths precede the remaining walls within an ordered island collection.
Each Wave path forbids reversal. G-code perimeter dispatch flattens such
collections in their stored order while retaining the normal deferred pass for
unsupported wall loops. Physical extrusion roles remain in use, so the Wave
paths still receive native overhang flow and process handling. A separate preview
identity presents actual Wave extrusion as **Wave + Pins** in the line-type legend,
with vivid cyan (`#00E5FF`) and its own visibility filter. Existing role colors
and their indices are preserved. Ordinary overhanging walls and support
contacts keep their usual types. The upper protected layers are not Wave paths
and keep their ordinary types as well.

Scoped G-code comments carry the preview identity through export and reopening
in this fork. The identity is attached only to extrusion moves inside the scope;
travel, retraction and later ordinary extrusion are not painted as Wave. Preview
identity does not add an extrusion role to the physical slicing engine. A file
exported by an older build without these markers cannot acquire the identity
retroactively; re-slice it with this version.

## Support contacts

The normal support generator receives actual unsupported Wave footprints, even
where ordinary bridge detection would have omitted them. Support blockers and
build-plate-only exclusions are honored. Explicit support enforcers are excluded
from contact reduction.

`WavePins::plan_pin_contacts` operates on the intersection of actual Wave coverage
and native contact polygons. It offsets each connected contact component inward
by the requested contact radius, then samples its boundaries to retain small
polygonal footprints. It returns a removal mask rather than replacing the whole
support layer. Every component from which contact is removed retains contacts;
components too narrow to fit a contact retain their original support. The number
of pins is bounded, with whole-operation fallback to ordinary contacts if that
bound is exceeded.

The removal mask is applied before native contact generation and again after
Snug contact expansion. Support below these footprints continues through the
existing interface and base generator, so it may widen or merge farther down.
The normal Top Z distance, support flow and support layer-height rules remain in
effect. Uncovered areas, narrow fallback regions and explicit enforcers can still
have substantial ordinary support.

The retained contacts are intended to reduce droop and help stabilize the Wave
sheet. They cannot clamp the model across an air gap: restraint against upward
curl requires some adhesion. Removing all contact marks is therefore not a
guaranteed outcome. Contact diameter describes the requested geometry, not a
guaranteed printed point size after native line generation and material spreading.
The 2 mm default gives a larger contact footprint than a 1.2 mm contact, at the
cost of potentially larger removal marks. It is not a calibrated P2S/PETG setting.
Narrow components which cannot fit the requested diameter retain ordinary support.

## Upper layers and process overrides

Each layer records actual coverage in `wave_pins_covered` and `wave_pins_floor`.
After ordinary infill preparation, the latter masks remove any fill regenerated
over current Wave paths and make the configured number of subsequent model
layers solid within their projected footprints. Existing top surfaces remain
top surfaces; other protected surfaces become internal solid fill. The protection
stays within model geometry and does not close ventilation holes.

After normal fill generation, model paths intersecting the protected footprint
are tagged for the upper-layer speed cap. A crossing path is capped as a whole;
wall loops are not split merely to change speed. Setting the protected layer
count to zero disables both the added solid layers and their speed cap.

G-code applies Wave and upper-layer speed caps after native first-layer,
resonance and overhang-speed calculations, including segmented per-point speeds.
Scoped cooling markers pass the requested Wave fan speed through CoolingBuffer.
They take priority over ordinary role fan requests during the Wave path and
restore the normal cooling decision at its end. Markers are consumed by the
cooling stage rather than left as printer-side commands requiring a plugin.

## PETG cooling policy

New presets inherit the existing filament cooling policy by default
(`wave_pins_fan_speed = -1`). This is a software default chosen for material-aware
behavior, not a validated PETG Wave temperature or airflow prescription. Native
overhang/bridge fan control may still request 100% while inheriting. Saved
profiles with an explicit percentage retain that value. An explicit value from 0 to 100
affects only Wave part-cooling requests; it does not change auxiliary cooling or
force the upper protected layers to use the same fan percentage. Leaving Wave
returns to the native cooling decision, which need not be a lower fan speed.

There is evidence for both the usefulness and the limits of maximum Wave cooling:

- [The upstream Wave settings guide](https://waveoverhangs.com/docs/settings)
  recommends a local 100% fan for PLA/PETG, while its
  [limitations guide](https://waveoverhangs.com/docs/limitations) describes PETG
  warping/delamination and thermal stresses in subsequent layers. These are
  experimental project recommendations, not a calibration for a particular model.
- [PETG report 229](https://waveoverhangs.com/gallery/229) records a stock Prusa
  MK3S+, 0.4 mm nozzle, 0.2 mm layers, 240 degrees C, 2 mm/s Wave paths and 100%
  Wave fan. The author rates it Good with some sag on the side away from the fan.
  Its ordinary bridge fan setting was 50%, illustrating the separate scopes.
- [PETG report 162](https://waveoverhangs.com/gallery/162) records an Elegoo
  Centauri Carbon at 245 degrees C, 2 mm/s and 100% Wave fan, with about 1 mm
  upward deformation on a 10 mm overhang. It uses a different older Wave
  algorithm and flow, so it is not a controlled comparison with report 229.
- [Prusa's material guidance](https://help.prusa3d.com/article/petg_2059) describes
  the ordinary PETG tradeoff between cooling for detail and warmth for layer
  fusion. It does not establish an optimum fan percentage for Wave paths.

Neither a percentage nor a good single-layer photograph establishes the strength
or dimensional stability of an actual printed part. A printer's fan, duct, material,
temperature and geometry change the result. At slow Wave speeds, even a locally
scoped override can expose neighboring shell regions to airflow for a long time.
Compare a full-thickness curved sample with a hole edge at inherited cooling and
an explicit 100% Wave setting while holding temperature, speed, flow, orientation
and support contacts fixed. Inspect the actual fan-speed preview first: inherited
cooling may already reach 100%, in which case those two nominal settings do not
provide a distinct experiment. Intermediate percentages are experiment settings,
not universal PETG recommendations.

## Disabled behavior and caching

Projects and presets missing `wave_pins_filter_supported` use its enabled default;
an explicitly saved value is retained. The master switch remains disabled by
default. Existing explicit pin diameters, including 1.2 mm, are preserved; only
configurations without a diameter use the 2 mm default. The new filter key is part
of the existing cache signature, so older enabled caches require regeneration.

All geometry-changing hooks test the master switch. New path flags default to
false. Changing any `wave_pins_` option invalidates slicing from the slice step,
including an On-to-Off transition; perimeter regeneration clears the layer masks.
While enabled, any perimeter invalidation also invalidates support, and a bridge
line-width change invalidates Wave perimeters. Direct support preview establishes
the perimeter prerequisite before reading Wave coverage.
Thus disabling the feature rebuilds through native model and support generation.
Version text, additional available settings and preset metadata still identify
the fork, so byte-identical complete G-code files are not an appropriate equality
test. Motion, extrusion and support geometry are the relevant comparison.

Experimental cache files carry a format version, all WavePins settings, and native
object/print configuration hashes. Layer masks and path process flags also
survive serialization. The loader rejects an ordinary cache for an enabled
experiment, an experimental cache for a disabled experiment, or an experimental
cache whose settings differ. Ordinary disabled caches omit the additional fields.

## Verification boundaries

`tests/libslic3r/test_wave_pins.cpp` exercises real polygon geometry, emitted
coverage, holes, anchor and reach fallback, cancellation, contact preservation and
model-path replacement. Shallow-candidate tests cover local extrusion widths,
mixed shallow and steep sections, and preservation of native walls and fill.
The tests use the same native geometry implementation as
the slicer. `tests/slic3rutils/test_wave_pins_preview.cpp` checks renderer color and
filter compatibility. The FFF suite's `test_wave_pins_preview.cpp` covers preview
scope boundaries, arcs, reopening and unchanged physical move properties;
`test_wave_pins.cpp` covers the slicing switch, filter toggles and support prerequisites.

Geometric tests do not establish peel force, curl resistance, dimensional error,
or surface quality on a printer. Evaluate a representative full-thickness sample with
a challenging overhang or hole edge before a full-size part. Compare the disabled fork to
the same upstream commit with matching settings, and compare enabled versus
disabled on the same sample, material and orientation. Inspect that the Wave
starts on existing model material, that the intended contacts survive native
support toolpath generation, and that the protective layers are present.
