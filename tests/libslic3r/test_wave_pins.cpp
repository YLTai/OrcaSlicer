#include <catch2/catch_test_macros.hpp>

#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/Flow.hpp"
#include "libslic3r/WavePins/Wave.hpp"
#include "libslic3r/WavePins/SupportPins.hpp"
#include "libslic3r/Point.hpp"
#include "libslic3r/Polygon.hpp"
#include "libslic3r/Polyline.hpp"

#include <stdexcept>
#include <vector>

using namespace Slic3r;

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
    params.perimeter_count = 1;
    return params;
}

Polylines generated_lines(const WavePins::Result &result)
{
    Polylines lines;
    for (const ExtrusionPaths &region : result.paths)
        for (const ExtrusionPath &path : region)
            lines.push_back(path.polyline.to_polyline());
    return lines;
}

} // namespace

TEST_CASE("Wave regions without an underlying anchor produce no paths", "[WavePins]")
{
    const ExPolygons area{ExPolygon(rectangle(0, 0, 12, 12))};
    const WavePins::Result result = WavePins::generate(area, {}, parameters());
    REQUIRE(result.paths.empty());
    REQUIRE(result.covered.empty());
}

TEST_CASE("Fully supported regions do not acquire Wave paths", "[WavePins]")
{
    const Polygon rectangle_model = rectangle(0, 0, 12, 12);
    const WavePins::Result result = WavePins::generate(
        {ExPolygon(rectangle_model)}, {rectangle_model}, parameters());
    REQUIRE(result.paths.empty());
    REQUIRE(result.covered.empty());
}

TEST_CASE("An anchored overhang produces only physically swept coverage", "[WavePins]")
{
    const ExPolygons area{ExPolygon(rectangle(0, 0, 12, 16))};
    const Polygons lower{rectangle(0, 0, 4, 16)};
    const WavePins::Params params = parameters();
    const WavePins::Result result = WavePins::generate(area, lower, params);
    REQUIRE_FALSE(result.paths.empty());
    REQUIRE_FALSE(result.covered.empty());
    REQUIRE(diff(result.covered, to_polygons(area)).empty());

    Polygons swept;
    for (const ExtrusionPaths &region : result.paths) {
        REQUIRE_FALSE(region.empty());
        for (const ExtrusionPath &path : region) {
            REQUIRE(path.polyline.points.size() >= 2);
            REQUIRE(path.wave_pins);
            REQUIRE_FALSE(path.can_reverse());
            REQUIRE(path.mm3_per_mm > 0.0);
            REQUIRE(path.width > 0.0);
            REQUIRE(path.height > 0.0);
            polygons_append(swept, offset(path.polyline, 0.5f * float(scale_(path.width)),
                                         jtRound, 0.0, etOpenRound));
        }
    }
    // A proposed region must not be mistaken for actual emitted coverage.
    REQUIRE(diff(result.covered, union_(swept)).empty());
}

TEST_CASE("Wave propagation keeps ventilation holes open", "[WavePins]")
{
    ExPolygon shell(rectangle(0, 0, 14, 24));
    Polygon hole = rectangle(7, 8, 10, 16);
    hole.reverse();
    shell.holes.push_back(hole);
    const WavePins::Result result = WavePins::generate(
        {shell}, {rectangle(0, 0, 4, 24)}, parameters());
    REQUIRE_FALSE(result.paths.empty());
    REQUIRE(intersection(result.covered, Polygons{rectangle(7, 8, 10, 16)}).empty());
    // Check centerlines as well as the bookkeeping coverage mask.
    const Polylines inside_hole = intersection_pl(
        generated_lines(result), Polygons{rectangle(7.01, 8.01, 9.99, 15.99)});
    REQUIRE(inside_hole.empty());
}

TEST_CASE("Wave coverage obeys the configured distance from its anchor", "[WavePins]")
{
    const ExPolygons area{ExPolygon(rectangle(0, 0, 24, 16))};
    const Polygons lower{rectangle(0, 0, 4, 16)};
    WavePins::Params params = parameters();
    params.max_span = 3.0;
    const WavePins::Result result = WavePins::generate(area, lower, params);
    const Polygons permitted = offset(lower, float(scale_(params.max_span)));
    REQUIRE(result.paths.empty());
    REQUIRE(result.covered.empty());
    REQUIRE(diff(result.covered, permitted).empty());
}

TEST_CASE("Wave generation reports cancellation without returning a partial plan", "[WavePins]")
{
    WavePins::Params params = parameters();
    params.throw_if_canceled = [] { throw std::runtime_error("cancelled"); };
    REQUIRE_THROWS_AS(WavePins::generate({ExPolygon(rectangle(0, 0, 12, 16))},
                                      {rectangle(0, 0, 4, 16)}, params),
                      std::runtime_error);
}

TEST_CASE("Pin planning releases only covered contacts outside enforced regions", "[WavePins]")
{
    const Polygons contacts{rectangle(0, 0, 30, 20)};
    const Polygons covered{rectangle(0, 0, 12, 20)};
    const Polygons enforced{rectangle(2, 2, 5, 5)};
    const WavePins::PinContacts plan = WavePins::plan_pin_contacts(
        contacts, covered, enforced, 6.0, 1.2);
    REQUIRE_FALSE(plan.removable.empty());
    REQUIRE_FALSE(plan.retained.empty());
    REQUIRE(diff(plan.removable, contacts).empty());
    REQUIRE(diff(plan.removable, covered).empty());
    REQUIRE(intersection(plan.removable, enforced).empty());
    REQUIRE(intersection(plan.removable, plan.retained).empty());
    REQUIRE(intersection(plan.removable, Polygons{rectangle(12, 0, 30, 20)}).empty());
}

TEST_CASE("Every released contact island keeps printable pins", "[WavePins]")
{
    const Polygons contacts{rectangle(0, 0, 20, 20), rectangle(40, 0, 60, 20)};
    const WavePins::PinContacts plan = WavePins::plan_pin_contacts(
        contacts, contacts, {}, 6.0, 1.2);
    for (const Polygon &island : contacts) {
        REQUIRE_FALSE(intersection(plan.removable, Polygons{island}).empty());
        REQUIRE_FALSE(intersection(plan.retained, Polygons{island}).empty());
    }
}

TEST_CASE("Contacts too narrow for a pin retain their original support", "[WavePins]")
{
    const Polygons contacts{rectangle(0, 0, 0.5, 20)};
    const WavePins::PinContacts plan = WavePins::plan_pin_contacts(
        contacts, contacts, {}, 6.0, 1.2);
    REQUIRE(plan.removable.empty());
    REQUIRE(plan.retained.empty());
}

TEST_CASE("Invalid pin spacing leaves original support intact", "[WavePins]")
{
    const Polygons contacts{rectangle(0, 0, 20, 20)};
    const WavePins::PinContacts plan = WavePins::plan_pin_contacts(
        contacts, contacts, {}, 1.0, 1.2);
    REQUIRE(plan.removable.empty());
    REQUIRE(plan.retained.empty());
}
