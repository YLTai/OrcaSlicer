// Wavefront algorithm by Janis A. Andersons, based on Steven McCulloch's
// arc-overhang work and Dennis Klappe's Orca port. AGPL-3.0-or-later.
#pragma once

#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/ExtrusionEntity.hpp"
#include "libslic3r/Flow.hpp"
#include "libslic3r/Polygon.hpp"
#include "libslic3r/Polyline.hpp"

#include <functional>
#include <vector>

namespace Slic3r::WavePins {

struct Params {
    Flow flow;
    double line_spacing = 0.0; // mm; zero derives spacing from the local flow.
    double max_span = 12.0;    // maximum distance from underlying model, in mm.
    double scaled_resolution = 1.0;
    int perimeter_count = 1;  // controls the supported anchor band only.
    bool filter_supported = true; // omit shallow overhangs within half a local bead width.
    std::function<void()> throw_if_canceled;
};

struct Result {
    // Paths are ordered from a supported seed toward the free edge. Neither
    // their ordering nor their direction may be changed by the G-code planner.
    std::vector<ExtrusionPaths> paths;
    // Actual swept extrusion footprint, clipped to the input area. A candidate
    // polygon or a wavefront that emitted no paths never counts as coverage.
    Polygons covered;
};

Result generate(const ExPolygons &area, const Polygons &lower_slices, const Params &params);

namespace detail {
// Deterministic chaining of adjacent wavefront fragments.
Polylines reconnect_polylines(const Polylines &polylines, double limit_distance);
}

} // namespace Slic3r::WavePins
