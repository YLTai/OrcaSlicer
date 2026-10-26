#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/ExtrusionEntity.hpp"
#include "libslic3r/ExtrusionEntityCollection.hpp"
#include "libslic3r/Flow.hpp"
#include "libslic3r/WavePins/ModelPaths.hpp"
#include "libslic3r/Point.hpp"
#include "libslic3r/Polygon.hpp"
#include "libslic3r/Polyline.hpp"
#include "libslic3r/Surface.hpp"
#include "libslic3r/SurfaceCollection.hpp"

#include <cstddef>
#include <utility>
#include <vector>

using namespace Slic3r;
using Catch::Matchers::WithinAbs;

namespace {

Polygon rectangle(double x0, double y0, double x1, double y1)
{
    return Polygon(Points{Point::new_scale(x0, y0), Point::new_scale(x1, y0),
                          Point::new_scale(x1, y1), Point::new_scale(x0, y1)});
}

WavePins::Params parameters()
{
    WavePins::Params params;
    params.flow = Flow(0.45f, 0.20f, 0.40f);
    params.line_spacing = 0.35;
    params.max_span = 12.0;
    params.scaled_resolution = scale_(0.0125);
    params.filter_supported = true;
    return params;
}

struct IslandPaths {
    ExPolygon island{rectangle(0, 0, 12, 16)};
    Polygons lower{rectangle(0, 0, 4, 16)};
    WavePins::Params params{parameters()};
    ExtrusionEntityCollection perimeters;
    ExtrusionEntityCollection gaps;
    SurfaceCollection fills;

    IslandPaths()
    {
        for (int inset = 0; inset < 3; ++inset) {
            const double margin = 0.225 + 0.4 * inset;
            ExtrusionPath path(inset == 0 ? erExternalPerimeter : erPerimeter,
                               params.flow.mm3_per_mm(), params.flow.width(), params.flow.height());
            path.polyline = Polyline3(rectangle(margin, margin, 12 - margin, 16 - margin)
                                         .split_at_first_point());
            path.overhang_degree = 0.25 + 0.1 * inset;
            path.curve_degree = 4 + inset;
            path.smooth_speed = 17.25 + inset;
            path.set_reverse();
            ExtrusionLoop loop(std::move(path), inset == 0 ? elrDefault : elrInternal);
            loop.inset_idx = inset;
            loop.print_after_infill = inset == 0;
            perimeters.append(std::move(loop));
            auto &stored = static_cast<ExtrusionLoop &>(*perimeters.entities.back());
            stored.paths.front().inset_idx = inset;
        }
        Surface fill(stInternal, ExPolygon(rectangle(1.3, 1.3, 10.7, 14.7)));
        fill.thickness = 0.2;
        fill.thickness_layers = 1;
        fills.surfaces.push_back(std::move(fill));
    }

    Polygons apply(int preserve = 2)
    {
        return WavePins::apply_to_island(island, lower, params,
                                       scale_(params.flow.spacing() * preserve), preserve,
                                       perimeters, gaps, fills);
    }
};

void require_same_path(const ExtrusionPath &actual, const ExtrusionPath &expected)
{
    REQUIRE(actual.polyline.points == expected.polyline.points);
    REQUIRE(actual.role() == expected.role());
    REQUIRE(actual.inset_idx == expected.inset_idx);
    REQUIRE_THAT(actual.width, WithinAbs(expected.width, 1e-7));
    REQUIRE_THAT(actual.height, WithinAbs(expected.height, 1e-7));
    REQUIRE_THAT(actual.mm3_per_mm, WithinAbs(expected.mm3_per_mm, 1e-10));
    REQUIRE_THAT(actual.overhang_degree, WithinAbs(expected.overhang_degree, 1e-10));
    REQUIRE(actual.curve_degree == expected.curve_degree);
    REQUIRE_THAT(actual.smooth_speed, WithinAbs(expected.smooth_speed, 1e-10));
    REQUIRE(actual.z_contoured == expected.z_contoured);
    REQUIRE(actual.wave_pins == expected.wave_pins);
    REQUIRE(actual.wave_pins_floor == expected.wave_pins_floor);
    REQUIRE(actual.can_reverse() == expected.can_reverse());
    REQUIRE(actual.is_force_no_extrusion() == expected.is_force_no_extrusion());
}

} // namespace

TEST_CASE("Wave replacement keeps every protected wall and its process properties", "[WavePinsModelPaths]")
{
    IslandPaths sample;
    const int preserved = 2;
    std::vector<ExtrusionLoop> original;
    original.reserve(size_t(preserved));
    for (int i = 0; i < preserved; ++i) {
        const auto &loop = static_cast<const ExtrusionLoop &>(*sample.perimeters.entities[i]);
        original.push_back(loop);
        // Native path copying does not copy this inherited field.
        original.back().paths.front().inset_idx = loop.paths.front().inset_idx;
    }

    REQUIRE_FALSE(sample.apply(preserved).empty());
    REQUIRE_FALSE(sample.perimeters.can_sort());
    REQUIRE_FALSE(sample.perimeters.can_reverse());
    size_t found = 0;
    for (const ExtrusionEntity *entity : sample.perimeters.entities) {
        const auto *loop = dynamic_cast<const ExtrusionLoop *>(entity);
        if (loop == nullptr || loop->inset_idx < 0 || loop->inset_idx >= preserved)
            continue;
        const ExtrusionLoop &expected = original[size_t(loop->inset_idx)];
        REQUIRE(loop->paths.size() == expected.paths.size());
        REQUIRE(loop->loop_role() == expected.loop_role());
        REQUIRE(loop->print_after_infill == expected.print_after_infill);
        for (size_t i = 0; i < loop->paths.size(); ++i)
            require_same_path(loop->paths[i], expected.paths[i]);
        ++found;
    }
    REQUIRE(found == size_t(preserved));
}

TEST_CASE("Wave fronts precede retained model paths after collection flattening", "[WavePinsModelPaths]")
{
    IslandPaths sample;
    REQUIRE_FALSE(sample.apply().empty());

    // G-code emission must use this exact child order for an ordered island;
    // it must not pass the collection itself to the ordinary leaf dispatcher.
    const ExtrusionEntityCollection flat = sample.perimeters.flatten();
    REQUIRE(flat.entities.size() == sample.perimeters.entities.size());
    bool seen_native = false;
    size_t waves = 0;
    for (size_t i = 0; i < flat.entities.size(); ++i) {
        const ExtrusionEntity &entity = *flat.entities[i];
        REQUIRE_FALSE(entity.is_collection());
        REQUIRE(entity.as_polyline().points == sample.perimeters.entities[i]->as_polyline().points);
        const auto *path = dynamic_cast<const ExtrusionPath *>(&entity);
        if (path != nullptr && path->wave_pins) {
            REQUIRE_FALSE(seen_native);
            REQUIRE_FALSE(path->can_reverse());
            ++waves;
        } else {
            seen_native = true;
        }
    }
    REQUIRE(waves > 0);
    REQUIRE(seen_native);
}

TEST_CASE("Wave replacement removes infill only from its emitted footprint", "[WavePinsModelPaths]")
{
    IslandPaths sample;
    const ExPolygon original = sample.fills.surfaces.front().expolygon;
    const Polygons covered = sample.apply();
    REQUIRE_FALSE(covered.empty());
    Polygons remaining;
    for (const Surface &surface : sample.fills.surfaces) {
        REQUIRE(surface.surface_type == stInternal);
        REQUIRE_THAT(surface.thickness, WithinAbs(0.2, 1e-10));
        REQUIRE(surface.thickness_layers == 1);
        polygons_append(remaining, to_polygons(surface.expolygon));
    }
    const Polygons expected = diff(to_polygons(original), covered);
    REQUIRE(diff(expected, remaining).empty());
    REQUIRE(diff(remaining, expected).empty());
    REQUIRE_FALSE(remaining.empty());
}

TEST_CASE("A rejected unanchored Wave candidate leaves its native objects untouched", "[WavePinsModelPaths]")
{
    IslandPaths sample;
    sample.lower.clear();
    const ExtrusionEntitiesPtr original = sample.perimeters.entities;
    const Points fill_contour = sample.fills.surfaces.front().expolygon.contour.points;
    REQUIRE(sample.apply().empty());
    REQUIRE(sample.perimeters.entities == original);
    REQUIRE(sample.perimeters.can_sort());
    REQUIRE(sample.perimeters.can_reverse());
    REQUIRE(sample.fills.surfaces.front().expolygon.contour.points == fill_contour);
}

TEST_CASE("Filtering a shallow Wave candidate preserves native walls and fill", "[WavePinsModelPaths]")
{
    IslandPaths sample;
    sample.params.filter_supported = true;
    const double inset = sample.params.flow.spacing();
    sample.lower = Polygons{rectangle(0, 0, 12 - inset - 0.1, 16)};
    const ExtrusionEntitiesPtr original = sample.perimeters.entities;
    const Points fill_contour = sample.fills.surfaces.front().expolygon.contour.points;
    REQUIRE(sample.apply(1).empty());
    REQUIRE(sample.perimeters.entities == original);
    REQUIRE(sample.perimeters.can_sort());
    REQUIRE(sample.fills.surfaces.front().expolygon.contour.points == fill_contour);

    sample.params.filter_supported = false;
    REQUIRE_FALSE(sample.apply(1).empty());
}

TEST_CASE("Wave replacement declines a path with nonplanar extrusion state", "[WavePinsModelPaths]")
{
    IslandPaths sample;
    ExtrusionPath path(erGapFill, sample.params.flow.mm3_per_mm(),
                       sample.params.flow.width(), sample.params.flow.height());
    path.polyline = Polyline3(Polyline(Points{Point::new_scale(3, 8), Point::new_scale(10, 8)}));
    path.z_contoured = true;
    sample.gaps.append(std::move(path));
    const ExtrusionEntitiesPtr original = sample.perimeters.entities;
    const ExtrusionEntity *original_gap = sample.gaps.entities.front();
    REQUIRE(sample.apply().empty());
    REQUIRE(sample.perimeters.entities == original);
    REQUIRE(sample.gaps.entities.front() == original_gap);
}
