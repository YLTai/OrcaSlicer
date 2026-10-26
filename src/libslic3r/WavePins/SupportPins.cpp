#include "SupportPins.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <utility>

#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/Point.hpp"
#include "libslic3r/libslic3r.h"

namespace Slic3r::WavePins {
namespace {

Polygon pin_disk(const Point &center, coord_t radius)
{
    // Inscribed polygon: rounding must not make a pin invade a neighbouring hole.
    constexpr int sides = 24;
    Points vertices;
    vertices.reserve(sides);
    const double pi = std::acos(-1.0);
    for (int i = 0; i < sides; ++i) {
        const double angle = 2.0 * pi * i / sides;
        vertices.emplace_back(center.x() + coord_t(std::llround(radius * std::cos(angle))),
                              center.y() + coord_t(std::llround(radius * std::sin(angle))));
    }
    return Polygon(std::move(vertices));
}

} // namespace

PinContacts plan_pin_contacts(const Polygons &contacts, const Polygons &wave_covered,
                              const Polygons &enforcers, double pitch_mm, double diameter_mm)
{
    PinContacts result;
    if (contacts.empty() || wave_covered.empty() || !std::isfinite(pitch_mm) ||
        !std::isfinite(diameter_mm) || diameter_mm <= 0.0 || pitch_mm <= diameter_mm)
        return result;

    const coord_t radius = scale_(0.5 * diameter_mm);
    if (radius <= 0)
        return result;
    const double pitch = scale_(pitch_mm);
    const double min_distance_squared = pitch * pitch * 0.25;
    const ExPolygons eligible = diff_ex(intersection(contacts, wave_covered), enforcers);

    // Bound work for adversarial geometry. Reaching the limit retains the
    // original contacts; it never silently omits pins from a released area.
    constexpr size_t max_pins = 4096;
    size_t total_pins = 0;
    for (const ExPolygon &island : eligible) {
        const ExPolygons centers = offset_ex(island, -double(radius + SCALED_EPSILON));
        if (centers.empty())
            continue;

        Points chosen;
        Polygons pins;
        auto add_contour = [&](const Polygon &contour) {
            for (const Point &p : contour.equally_spaced_points(pitch)) {
                if (std::any_of(chosen.begin(), chosen.end(), [&](const Point &q) {
                        return (p.cast<double>() - q.cast<double>()).squaredNorm() < min_distance_squared;
                    }))
                    continue;
                chosen.push_back(p);
                pins.push_back(pin_disk(p, radius));
                if (++total_pins > max_pins)
                    return false;
            }
            return true;
        };
        for (const ExPolygon &center_area : centers) {
            if (!add_contour(center_area.contour))
                return {};
            for (const Polygon &hole : center_area.holes)
                if (!add_contour(hole))
                    return {};
        }
        if (pins.empty())
            continue;

        pins = intersection(pins, Polygons(to_polygons(island)));
        if (pins.empty())
            continue;
        const Polygons released = diff(to_polygons(island), pins);
        result.removable.insert(result.removable.end(), released.begin(), released.end());
        result.retained.insert(result.retained.end(), pins.begin(), pins.end());
    }
    result.removable = union_(result.removable);
    result.retained = union_(result.retained);
    return result;
}

} // namespace Slic3r::WavePins
