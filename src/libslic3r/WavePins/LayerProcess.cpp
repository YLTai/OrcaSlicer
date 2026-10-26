#include "LayerProcess.hpp"

#include <algorithm>
#include <cstddef>
#include <utility>

#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/ExtrusionEntity.hpp"
#include "libslic3r/ExtrusionEntityCollection.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/Polygon.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintBase.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/Surface.hpp"

namespace Slic3r::WavePins {
namespace {

Polygons floor_footprint(const PrintObject &object, size_t layer_index)
{
    Polygons footprint;
    const size_t depth = size_t(std::max(0, object.config().wave_pins_floor_layers.value));
    const size_t begin = layer_index > depth ? layer_index - depth : 0;
    for (size_t i = begin; i < layer_index; ++i) {
        const Polygons &wave = object.layers()[i]->wave_pins_floor;
        footprint.insert(footprint.end(), wave.begin(), wave.end());
    }
    return union_(footprint);
}

void tag_path(ExtrusionPath &path, const Polygons &footprint)
{
    // Keep loop identities and their order intact. A path crossing the protected
    // footprint receives the speed cap as a whole; no exterior wall is cut here.
    path.wave_pins_floor = !path.wave_pins && !footprint.empty() &&
        !intersection_pl(Polylines{path.as_polyline()}, footprint).empty();
}

void tag_entity(ExtrusionEntity &entity, const Polygons &footprint)
{
    if (auto *path = dynamic_cast<ExtrusionPath *>(&entity)) {
        tag_path(*path, footprint);
    } else if (auto *loop = dynamic_cast<ExtrusionLoop *>(&entity)) {
        for (ExtrusionPath &path : loop->paths)
            tag_path(path, footprint);
    } else if (auto *multi = dynamic_cast<ExtrusionMultiPath *>(&entity)) {
        for (ExtrusionPath &path : multi->paths)
            tag_path(path, footprint);
    } else if (auto *collection = dynamic_cast<ExtrusionEntityCollection *>(&entity)) {
        for (ExtrusionEntity *child : collection->entities)
            tag_entity(*child, footprint);
    }
}

} // namespace

void prepare_floor_surfaces(PrintObject &object)
{
    if (!object.config().wave_pins_enabled.value)
        return;
    for (size_t i = 0; i < object.layers().size(); ++i) {
        if (object.print()->canceled())
            throw CanceledException();
        Layer &layer = *object.layers()[i];
        const Polygons footprint = floor_footprint(object, i);
        const Polygons &own_wave = layer.wave_pins_floor;
        if (footprint.empty() && own_wave.empty())
            continue;
        for (LayerRegion *region : layer.regions()) {
            Surfaces output;
            for (const Surface &surface : region->fill_surfaces.surfaces) {
                // Later shell/bridge classification must not fill the area
                // already occupied by the current layer's ordered wave paths.
                const ExPolygons remaining = diff_ex(surface.expolygon, own_wave);
                for (ExPolygon &part : diff_ex(remaining, footprint))
                    output.emplace_back(surface, std::move(part));
                for (ExPolygon &part : intersection_ex(remaining, footprint)) {
                    Surface floor(surface.is_top() ? stTop : stInternalSolid, std::move(part));
                    floor.thickness = layer.height;
                    floor.thickness_layers = 1;
                    output.emplace_back(std::move(floor));
                }
            }
            region->fill_surfaces.surfaces = std::move(output);
        }
    }
}

void tag_floor_paths(PrintObject &object)
{
    if (!object.config().wave_pins_enabled.value)
        return;
    for (size_t i = 0; i < object.layers().size(); ++i) {
        if (object.print()->canceled())
            throw CanceledException();
        const Polygons footprint = floor_footprint(object, i);
        for (LayerRegion *region : object.layers()[i]->regions()) {
            tag_entity(region->perimeters, footprint);
            tag_entity(region->fills, footprint);
            tag_entity(region->thin_fills, footprint);
        }
    }
}

} // namespace Slic3r::WavePins
