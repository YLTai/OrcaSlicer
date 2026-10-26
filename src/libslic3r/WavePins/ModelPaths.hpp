#pragma once

#include "Wave.hpp"
#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/Polygon.hpp"

namespace Slic3r {
class ExtrusionEntityCollection;
class SurfaceCollection;
}

namespace Slic3r::WavePins {

// Replace an island's inner paths only where a Wave was actually emitted.
// Input collections are unchanged if the candidate is empty or cannot be
// represented without discarding path attributes. Returns actual coverage.
Polygons apply_to_island(const ExPolygon &island, const Polygons &lower,
                        const Params &params, double inset_scaled, int preserve_outer,
                        ExtrusionEntityCollection &perimeters,
                        ExtrusionEntityCollection &gap_fill, SurfaceCollection &fill_surfaces);

} // namespace Slic3r::WavePins
