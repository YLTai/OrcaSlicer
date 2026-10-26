// Native model-path integration for the Wave + Pins experiment.
// Path clipping is adapted from Dennis Klappe's Orca Wave port
// (0e44f2bcef5e38c84607f3c955a03629342cdc32), AGPL-3.0-or-later.
#include "ModelPaths.hpp"

#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/ExtrusionEntity.hpp"
#include "libslic3r/ExtrusionEntityCollection.hpp"
#include "libslic3r/Point.hpp"
#include "libslic3r/Polyline.hpp"
#include "libslic3r/Surface.hpp"
#include "libslic3r/SurfaceCollection.hpp"
#include "libslic3r/libslic3r.h"

#include <algorithm>
#include <cstddef>
#include <utility>

namespace Slic3r::WavePins {
namespace {

void restore_inset_indices(const ExtrusionEntity &source, ExtrusionEntity &copy)
{
    // Native path/collection copy constructors do not copy the base inset
    // field. Keep this correction local to the experimental replacement.
    copy.inset_idx = source.inset_idx;
    if (const auto *collection = dynamic_cast<const ExtrusionEntityCollection *>(&source)) {
        auto &copied = static_cast<ExtrusionEntityCollection &>(copy);
        for (size_t i = 0; i < collection->entities.size(); ++i)
            restore_inset_indices(*collection->entities[i], *copied.entities[i]);
    } else if (const auto *loop = dynamic_cast<const ExtrusionLoop *>(&source)) {
        auto &copied = static_cast<ExtrusionLoop &>(copy);
        for (size_t i = 0; i < loop->paths.size(); ++i)
            copied.paths[i].inset_idx = loop->paths[i].inset_idx;
    } else if (const auto *multi = dynamic_cast<const ExtrusionMultiPath *>(&source)) {
        auto &copied = static_cast<ExtrusionMultiPath &>(copy);
        for (size_t i = 0; i < multi->paths.size(); ++i)
            copied.paths[i].inset_idx = multi->paths[i].inset_idx;
    }
}

void append_preserved(ExtrusionEntityCollection &output, const ExtrusionEntity &entity)
{
    output.append(entity);
    restore_inset_indices(entity, *output.entities.back());
}

bool clip_paths(const ExtrusionPaths &input, const Polygons &clip, ExtrusionPaths &output, bool &changed)
{
    for (const ExtrusionPath &path : input) {
        const Polyline original = path.polyline.to_polyline();
        const Polylines kept = diff_pl(Polylines{original}, clip);
        if (kept.size() == 1 && kept.front().points == original.points) {
            output.emplace_back(path);
            continue;
        }
        // A planar replacement cannot preserve a scarf/Z-contoured path's
        // pointwise heights. Decline the whole candidate rather than flatten it.
        if (path.z_contoured || std::any_of(path.polyline.points.begin(), path.polyline.points.end(),
                                          [](const Point3 &p) { return p.z() != 0; }))
            return false;
        changed = true;
        if (original.points.size() < 2)
            continue;
        const double half_width = scale_(0.5 * path.width);
        for (Polyline line : kept) {
            if (line.points.size() < 2)
                continue;
            const auto original_end = [&original](const Point &p) {
                return p == original.first_point() || p == original.last_point();
            };
            const bool trim_start = !original_end(line.first_point());
            const bool trim_end = !original_end(line.last_point());
            if (line.length() <= (int(trim_start) + int(trim_end)) * half_width + SCALED_EPSILON)
                continue;
            if (trim_start)
                line.clip_start(half_width);
            if (trim_end && line.points.size() >= 2)
                line.clip_end(half_width);
            if (line.points.size() >= 2) {
                ExtrusionPath replacement(path);
                replacement.polyline = Polyline3(line);
                output.emplace_back(std::move(replacement));
            }
        }
    }
    return true;
}

bool clip_collection(const ExtrusionEntityCollection &input, const Polygons &clip, int preserve_outer,
                     ExtrusionEntityCollection &output)
{
    output.no_sort = input.no_sort;
    output.inset_idx = input.inset_idx;
    if (!input.can_reverse())
        output.set_reverse();
    const BoundingBox clip_bounds = get_extents(clip).inflated(SCALED_EPSILON);
    for (const ExtrusionEntity *entity : input.entities) {
        // These entities carry additional height/flow state which plain path
        // clipping cannot preserve, including slopes with zero pointwise Z.
        if (dynamic_cast<const ExtrusionPathSloped *>(entity) ||
            dynamic_cast<const ExtrusionPathContoured *>(entity) ||
            dynamic_cast<const ExtrusionLoopSloped *>(entity))
            return false;
        if (entity->inset_idx >= 0 && entity->inset_idx < preserve_outer) {
            append_preserved(output, *entity);
            continue;
        }
        Points points;
        entity->collect_points(points);
        if (points.empty() || !clip_bounds.overlap(BoundingBox(points))) {
            append_preserved(output, *entity);
            continue;
        }
        if (const auto *collection = dynamic_cast<const ExtrusionEntityCollection *>(entity)) {
            ExtrusionEntityCollection clipped;
            if (!clip_collection(*collection, clip, preserve_outer, clipped))
                return false;
            if (!clipped.empty()) {
                const int inset = clipped.inset_idx;
                output.append(std::move(clipped));
                output.entities.back()->inset_idx = inset;
            }
            continue;
        }
        ExtrusionPaths source;
        if (const auto *loop = dynamic_cast<const ExtrusionLoop *>(entity))
            source = loop->paths;
        else if (const auto *multi = dynamic_cast<const ExtrusionMultiPath *>(entity))
            source = multi->paths;
        else if (const auto *path = dynamic_cast<const ExtrusionPath *>(entity))
            source.emplace_back(*path);
        else
            return false;
        ExtrusionPaths kept;
        bool changed = false;
        if (!clip_paths(source, clip, kept, changed))
            return false;
        if (!changed)
            append_preserved(output, *entity);
        else {
            // Clipped pieces can be disconnected; emitting each as an ordinary
            // path lets Orca insert a travel/retraction between them.
            for (ExtrusionPath &path : kept) {
                output.append(std::move(path));
                output.entities.back()->inset_idx = entity->inset_idx;
            }
        }
    }
    return true;
}

} // namespace

Polygons apply_to_island(const ExPolygon &island, const Polygons &lower,
                        const Params &params, double inset_scaled, int preserve_outer,
                        ExtrusionEntityCollection &perimeters,
                        ExtrusionEntityCollection &gap_fill, SurfaceCollection &fill_surfaces)
{
    if (preserve_outer < 1 || perimeters.empty())
        return {};
    const ExPolygons area = offset_ex(island, -float(inset_scaled));
    if (area.empty())
        return {};
    Result wave = generate(area, lower, params);
    if (wave.paths.empty() || wave.covered.empty())
        return {};

    ExtrusionEntityCollection clipped_perimeters;
    ExtrusionEntityCollection clipped_gaps;
    if (!clip_collection(perimeters, wave.covered, preserve_outer, clipped_perimeters) ||
        !clip_collection(gap_fill, wave.covered, 0, clipped_gaps))
        return {};

    SurfaceCollection clipped_surfaces;
    for (const Surface &surface : fill_surfaces.surfaces)
        clipped_surfaces.append(diff_ex(ExPolygons{surface.expolygon}, wave.covered), surface);

    ExtrusionEntityCollection replacement;
    replacement.no_sort = true;
    replacement.set_reverse();
    // Waves must reach the free edge from their supported seeds before the
    // preserved overhanging walls are printed. Keep this order across export.
    for (ExtrusionPaths &region : wave.paths)
        replacement.append(region);
    replacement.append(std::move(clipped_perimeters.entities));
    if (params.throw_if_canceled)
        params.throw_if_canceled();
    perimeters.swap(replacement);
    perimeters.set_reverse();
    gap_fill.swap(clipped_gaps);
    fill_surfaces.surfaces.swap(clipped_surfaces.surfaces);
    return std::move(wave.covered);
}

} // namespace Slic3r::WavePins
