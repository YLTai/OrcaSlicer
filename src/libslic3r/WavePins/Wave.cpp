// Wavefront algorithm by Janis A. Andersons, based on Steven McCulloch's
// arc-overhang work. Adapted from Dennis Klappe's Orca port at
// 0e44f2bcef5e38c84607f3c955a03629342cdc32 (AGPL-3.0-or-later).
// This adapter retains wavefront ordering, bounds propagation, and derives
// support removal from emitted extrusion footprints rather than candidate area.
#include "Wave.hpp"

#include "libslic3r/Algorithm/RegionExpansion.hpp"
#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/Point.hpp"
#include "libslic3r/libslic3r.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <utility>

namespace Slic3r::WavePins {
namespace detail {
Polylines reconnect_polylines(const Polylines &polylines, double limit_distance)
{
    std::vector<Polyline> chains(polylines.begin(), polylines.end());
    std::vector<bool>     alive(chains.size());
    for (size_t i = 0; i < chains.size(); ++i)
        alive[i] = ! chains[i].empty();

    const double limit_sq = limit_distance * limit_distance;
    auto close = [limit_sq](const Point &p1, const Point &p2) {
        return (p1 - p2).cast<double>().squaredNorm() < limit_sq;
    };

    // Repeat until nothing joins any more: a merge changes the end points, so earlier
    // rejected pieces may match now.
    for (bool changed = true; changed;) {
        changed = false;
        for (size_t a = 0; a < chains.size(); ++a) {
            if (! alive[a])
                continue;
            Polyline &base = chains[a];
            for (size_t b = a + 1; b < chains.size(); ++b) {
                if (! alive[b])
                    continue;
                Polyline &next = chains[b];
                if (close(base.last_point(), next.first_point())) {
                    base.append(std::move(next));
                } else if (close(base.last_point(), next.last_point())) {
                    base.points.insert(base.points.end(), next.points.rbegin(), next.points.rend());
                } else if (close(base.first_point(), next.last_point())) {
                    next.append(std::move(base));
                    base = std::move(next);
                    base.reverse();
                } else if (close(base.first_point(), next.first_point())) {
                    base.reverse();
                    base.append(std::move(next));
                    base.reverse();
                } else
                    continue;
                alive[b] = false;
                changed  = true;
            }
        }
    }

    Polylines result;
    for (size_t i = 0; i < chains.size(); ++i)
        if (alive[i])
            result.push_back(std::move(chains[i]));
    return result;
}


} // namespace detail
namespace {
template <class Fn>
void for_each_boundary_point(const ExPolygon &expoly, Fn &&fn)
{
    for (const Point &pt : expoly.contour.points)
        fn(pt);
    for (const Polygon &hole : expoly.holes)
        for (const Point &pt : hole.points)
            fn(pt);
}

struct ClosestBoundaryPair {
    Point  a;
    Point  b;
    double distance_sq{ std::numeric_limits<double>::infinity() };
    bool   valid{ false };
};

struct NarrowSplitCandidate {
    Point   a;
    Point   b;
    Point   midpoint;
    double  distance_sq{ std::numeric_limits<double>::infinity() };
    Polygon slit;
};

ClosestBoundaryPair find_closest_boundary_pair(const ExPolygon &a, const ExPolygon &b, const ExPolygon &container)
{
    ClosestBoundaryPair best;

    auto try_pair = [&](const Point &src, const ExPolygon &other, bool src_is_a) {
        Point projected = other.point_projection(src);
        const double distance_sq = (projected - src).cast<double>().squaredNorm();
        if (distance_sq >= best.distance_sq)
            return;

        const Point midpoint = (0.5 * (src.cast<double>() + projected.cast<double>())).cast<coord_t>();
        if (! container.contains(midpoint))
            return;

        best.distance_sq = distance_sq;
        best.valid = true;
        if (src_is_a) {
            best.a = src;
            best.b = projected;
        } else {
            best.a = projected;
            best.b = src;
        }
    };

    for_each_boundary_point(a, [&](const Point &pt) { try_pair(pt, b, true); });
    for_each_boundary_point(b, [&](const Point &pt) { try_pair(pt, a, false); });

    return best;
}

Polygon make_split_slit(const Point &a, const Point &b, coord_t extension, coord_t half_width)
{
    const Vec2d start = a.cast<double>();
    const Vec2d end   = b.cast<double>();
    const Vec2d delta = end - start;
    const double length = delta.norm();
    if (length <= 0.)
        return {};

    const Vec2d dir = delta / length;
    const Vec2d normal(-dir.y(), dir.x());
    const Vec2d extended_start = start - dir * double(extension);
    const Vec2d extended_end   = end + dir * double(extension);
    const Vec2d offset         = normal * double(std::max<coord_t>(1, half_width));

    Polygon slit;
    slit.points = {
        Point((extended_start + offset).cast<coord_t>()),
        Point((extended_end   + offset).cast<coord_t>()),
        Point((extended_end   - offset).cast<coord_t>()),
        Point((extended_start - offset).cast<coord_t>())
    };
    return slit;
}

size_t total_hole_count(const ExPolygons &expolygons)
{
    size_t count = 0;
    for (const ExPolygon &expolygon : expolygons)
        count += expolygon.holes.size();
    return count;
}

bool slit_changes_topology(const ExPolygon &wave_cover, const Polygon &slit)
{
    if (! slit.is_valid())
        return false;

    const ExPolygons split_result = union_ex(diff_ex(ExPolygons{ wave_cover }, Polygons{ slit }));
    return split_result.size() != 1 || total_hole_count(split_result) != wave_cover.holes.size();
}

// The slit starts about one wave spacing wide, so it is never thinner than a wave line.
Polygon make_effective_split_slit(const ExPolygon &wave_cover, const Point &a, const Point &b, coord_t extension, coord_t wave_spacing)
{
    coord_t half_width = wave_spacing / 2 + 1;
    for (int attempt = 0; attempt < 6; ++attempt) {
        Polygon slit = make_split_slit(a, b, extension, half_width);
        if (slit_changes_topology(wave_cover, slit))
            return slit;

        half_width = std::max<coord_t>(half_width + 1, half_width * 2);
    }

    return {};
}

Polygons generate_narrow_split_slits(const ExPolygon &wave_cover, coord_t wave_spacing, coord_t minimum_wave_width)
{
    const coord_t effective_minimum_width = std::max<coord_t>(0, minimum_wave_width);
    if (effective_minimum_width <= 0)
        return {};

    const double max_gap_sq = std::pow(double(effective_minimum_width), 2);
    const coord_t slit_half_width = std::max<coord_t>(1, wave_spacing / 20);
    const coord_t slit_extension  = std::max<coord_t>(slit_half_width, effective_minimum_width);
    const std::array<double, 4> inset_fractions{{ 0.25, 0.5, 0.75, 1.0 }};
    const double duplicate_radius_sq = std::pow(0.5 * double(wave_spacing), 2);
    const size_t original_hole_count = wave_cover.holes.size();

    std::vector<NarrowSplitCandidate> candidates;
    auto append_candidate = [&](const ClosestBoundaryPair &pair) {
        if (! pair.valid || pair.distance_sq > max_gap_sq)
            return;

        NarrowSplitCandidate candidate;
        candidate.a = pair.a;
        candidate.b = pair.b;
        candidate.distance_sq = pair.distance_sq;
        candidate.midpoint = (0.5 * (pair.a.cast<double>() + pair.b.cast<double>())).cast<coord_t>();
        candidate.slit = make_effective_split_slit(wave_cover, pair.a, pair.b, wave_spacing + slit_extension, wave_spacing);
        if (! candidate.slit.is_valid())
            return;

        candidates.push_back(std::move(candidate));
    };

    for (double inset_fraction : inset_fractions) {
        const coord_t inset_depth = std::max<coord_t>(1, coord_t(std::round(inset_fraction * double(wave_spacing))));
        const ExPolygons inset_components = offset_ex(wave_cover, -float(inset_depth), jtRound, 0.);
        const bool component_count_changed = inset_components.size() > 1;
        const bool hole_count_changed = total_hole_count(inset_components) != original_hole_count;
        if (! component_count_changed && ! hole_count_changed)
            continue;

        if (component_count_changed) {
            for (size_t i = 0; i < inset_components.size(); ++i) {
                for (size_t j = i + 1; j < inset_components.size(); ++j) {
                    ClosestBoundaryPair pair = find_closest_boundary_pair(inset_components[i], inset_components[j], wave_cover);
                    append_candidate(pair);
                }
            }
        }

        if (hole_count_changed && ! wave_cover.holes.empty()) {
            ExPolygon outer_boundary;
            outer_boundary.contour = wave_cover.contour;

            for (size_t hole_idx = 0; hole_idx < wave_cover.holes.size(); ++hole_idx) {
                ExPolygon hole_boundary;
                hole_boundary.contour = wave_cover.holes[hole_idx];
                append_candidate(find_closest_boundary_pair(outer_boundary, hole_boundary, wave_cover));

                for (size_t other_hole_idx = hole_idx + 1; other_hole_idx < wave_cover.holes.size(); ++other_hole_idx) {
                    ExPolygon other_hole_boundary;
                    other_hole_boundary.contour = wave_cover.holes[other_hole_idx];
                    append_candidate(find_closest_boundary_pair(hole_boundary, other_hole_boundary, wave_cover));
                }
            }
        }
    }

    if (candidates.empty())
        return {};

    std::sort(candidates.begin(), candidates.end(), [](const NarrowSplitCandidate &lhs, const NarrowSplitCandidate &rhs) {
        return lhs.distance_sq < rhs.distance_sq;
    });

    Polygons slits;
    std::vector<Point> kept_midpoints;
    for (NarrowSplitCandidate &candidate : candidates) {
        bool duplicate = false;
        for (const Point &kept_midpoint : kept_midpoints) {
            if ((candidate.midpoint - kept_midpoint).cast<double>().squaredNorm() <= duplicate_radius_sq) {
                duplicate = true;
                break;
            }
        }

        if (duplicate)
            continue;

        kept_midpoints.push_back(candidate.midpoint);
        slits.push_back(std::move(candidate.slit));
    }

    return slits.empty() ? Polygons{} : union_(slits);
}

Polylines generate_wave_overhang_seeds(const ExPolygon &boundary, const Polygons &anchoring, const coord_t seed_expansion)
{
    if (anchoring.empty())
        return {};

    Polylines seeds;
    for (const Algorithm::WaveSeed &seed : Algorithm::wave_seeds(union_ex(anchoring), ExPolygons{ boundary }, float(seed_expansion), true)) {
        if (seed.boundary == 0 && seed.path.size() >= 2)
            seeds.emplace_back(seed.path);
    }

    if (seeds.empty())
        seeds = intersection_pl(to_polylines(boundary), offset(anchoring, float(seed_expansion), jtRound, 0.));

    return seeds;
}

static ExtrusionPath make_wave_path(const Polyline &polyline, const Flow &flow)
{
    ExtrusionPath path(erOverhangPerimeter, flow.mm3_per_mm(), flow.width(), flow.height());
    path.polyline = Polyline3(polyline);
    path.wave_pins = true;
    path.set_reverse();
    return path;
}

void append_wave_fronts(ExtrusionPaths &overhang_region,
                        const Polylines &fronts,
                        const Flow      &wave_flow,
                        coord_t          connector_limit,
                        const std::function<void()> &cancel)
{
    if (fronts.empty())
        return;

    auto point_at_distance = [](const Polyline &line, double distance) {
        if (line.points.empty())
            return Point{};
        if (distance <= 0. || line.points.size() == 1)
            return line.first_point();

        double walked = 0.;
        for (size_t i = 1; i < line.points.size(); ++i) {
            const Vec2d a = line.points[i - 1].cast<double>();
            const Vec2d b = line.points[i].cast<double>();
            const Vec2d segment = b - a;
            const double segment_length = segment.norm();
            if (segment_length <= 0.)
                continue;
            if (walked + segment_length >= distance) {
                const double t = (distance - walked) / segment_length;
                return Point((a + t * segment).cast<coord_t>());
            }
            walked += segment_length;
        }
        return line.last_point();
    };

    auto support_score = [&point_at_distance](const Polyline &candidate, const Polylines &support_paths, coord_t support_reach, coord_t prefix_length) {
        if (support_paths.empty() || candidate.points.size() < 2)
            return -1.;

        const double candidate_length = candidate.length();
        if (candidate_length <= 0.)
            return -1.;

        const double sample_length = std::min(candidate_length, double(std::max<coord_t>(1, prefix_length)));
        const std::array<std::pair<double, double>, 3> samples = {{
            { 0.0,                 3.0 },
            { 0.5 * sample_length, 2.0 },
            { sample_length,       1.0 }
        }};

        double best_score = -1.;
        for (auto it = support_paths.rbegin(); it != support_paths.rend(); ++it) {
            if (it->points.size() < 2)
                continue;

            double score = 0.;
            for (const auto &[distance_along, weight] : samples) {
                Point sample = point_at_distance(candidate, distance_along);
                std::pair<int, Point> foot = foot_pt(it->points, sample);
                int seg_idx = foot.first;
                if (seg_idx < 0 || size_t(seg_idx + 1) >= it->points.size())
                    continue;

                const Point &a = it->points[size_t(seg_idx)];
                const Point &b = it->points[size_t(seg_idx + 1)];
                const bool interior_projection = foot.second != a && foot.second != b;
                const double distance_to_support = (sample - foot.second).cast<double>().norm();
                const double normalized_support = std::max(0.0, 1.0 - distance_to_support / double(std::max<coord_t>(1, support_reach)));

                score += weight * (3.0 * normalized_support + (interior_projection ? 1.5 : 0.2));
            }

            best_score = std::max(best_score, score);
        }

        return best_score;
    };

    Polylines support_paths;
    support_paths.reserve(overhang_region.size() + fronts.size());
    for (const ExtrusionPath &path : overhang_region)
        support_paths.emplace_back(path.polyline.to_polyline());
    const coord_t support_reach = std::max<coord_t>(wave_flow.scaled_width(), connector_limit);
    const coord_t prefix_length = std::max<coord_t>(wave_flow.scaled_width(), connector_limit / 2);

    for (const Polyline &source_front : fronts) {
        if (cancel)
            cancel();
        Polyline front = source_front;
        if (front.points.size() < 2)
            continue;

        Polyline reversed = front;
        reversed.reverse();
        const double forward_score = support_score(front, support_paths, support_reach, prefix_length);
        const double reverse_score = support_score(reversed, support_paths, support_reach, prefix_length);
        if (reverse_score > forward_score)
            front.reverse();

        overhang_region.push_back(make_wave_path(front, wave_flow));
        support_paths.push_back(std::move(front));
    }
}


} // namespace

Result generate(const ExPolygons &infill_area, const Polygons &lower_slices, const Params &params)
{
    const auto cancel = [&params] { if (params.throw_if_canceled) params.throw_if_canceled(); };
    cancel();
    if (infill_area.empty() || lower_slices.empty())
        return {};
    const Flow &flow = params.flow;
    const double spacing_mm = params.line_spacing == 0.0 ? 0.85 * flow.width() : params.line_spacing;
    if (!std::isfinite(spacing_mm) || spacing_mm <= 0.0 || spacing_mm > flow.width() ||
        !std::isfinite(params.max_span) || params.max_span <= 0.0 || params.max_span > 1000.0 ||
        !std::isfinite(flow.mm3_per_mm()) || flow.mm3_per_mm() <= 0.0 ||
        !std::isfinite(params.scaled_resolution) || params.scaled_resolution <= 0.0)
        throw std::invalid_argument("Invalid Wave + Pins spacing, span, flow or resolution.");

    const coord_t spacing = std::max<coord_t>(1, scaled<coord_t>(spacing_mm));
    const coord_t base_spacing = flow.scaled_spacing();
    const coord_t half_width = std::max<coord_t>(1, flow.scaled_width() / 2);
    const coord_t anchor_depth = std::min(scaled<coord_t>(3.0), base_spacing * (std::max(1, params.perimeter_count) + 1));
    const coord_t seed_expansion = std::max<coord_t>(1, base_spacing / 10);
    const coord_t min_wave_width = std::max<coord_t>(spacing, flow.scaled_width());
    // Bound both footprint reach and iteration count. The latter also bounds a
    // wave winding around a hole whose Euclidean distance to a seed is small.
    const int max_iterations = int(std::min(4092.0, std::ceil((params.max_span + unscale<double>(anchor_depth)) / spacing_mm))) + 4;
    const double min_growth = 0.02 * double(spacing) * double(spacing);
    const BoundingBox bounds = get_extents(infill_area).inflated(SCALED_EPSILON);
    const Polygons lower = ClipperUtils::clip_clipper_polygons_with_subject_bbox(lower_slices, bounds);
    Polygons overhangs = diff(infill_area, lower);
    if (params.filter_supported && !overhangs.empty()) {
        // Classify locally, not per connected island: one steep section of a
        // curved shell must not turn its entire shallow rim into slow Wave paths.
        // Expand back to the real lower-layer boundary so retained Wave regions
        // keep their anchors; the expanded lower layer is never an actual seed.
        const Polygons steep = diff(overhangs, offset(lower, float(half_width), jtRound, 0.));
        overhangs = intersection(overhangs, offset(steep, float(half_width), jtRound, 0.));
    }
    if (overhangs.empty())
        return {};
    const Polygons anchors = intersection(infill_area, lower);
    if (anchors.empty())
        return {};
    const Polygons inset_anchors = diff(anchors, expand(overhangs, anchor_depth + 0.1 * flow.scaled_width(), jtSquare, 0.));
    const Polygons candidate = params.filter_supported
        ? union_(overhangs, diff(anchors, inset_anchors))
        : diff(infill_area, inset_anchors);
    const Polygons reach = offset(lower, float(scale_(params.max_span)), jtRound, 0.);
    Result result;
    Polylines emitted;

    for (const ExPolygon &region : union_ex(candidate)) {
        cancel();
        const Polygons region_polys = to_polygons(region);
        const Polygons unsupported = intersection(region_polys, overhangs);
        if (unsupported.empty() || !diff(unsupported, reach).empty())
            continue; // Retain the native paths and supports for this component.
        ExPolygons split_regions{region};
        if (const Polygons slits = generate_narrow_split_slits(region, spacing, min_wave_width); !slits.empty())
            split_regions = diff_ex(split_regions, slits);
        Polygons anchoring = intersection(expand(region_polys, 1.1 * base_spacing, jtRound, 0.), inset_anchors);
        if (anchoring.empty())
            anchoring = intersection(expand(region_polys, 1.1 * base_spacing, jtRound, 0.), anchors);
        const Polylines base_seeds = generate_wave_overhang_seeds(region, anchoring, seed_expansion);
        if (base_seeds.empty())
            continue;
        ExtrusionPaths region_paths;
        for (const ExPolygon &piece : split_regions) {
            cancel();
            const Polygons domain = to_polygons(piece);
            const Polylines seeds = intersection_pl(base_seeds, domain);
            if (seeds.empty())
                continue;
            const Polygons trim = shrink(domain, half_width, jtRound, 0.);
            if (trim.empty())
                continue; // Less than a bead width; keep the original thin wall.
            Polygons accumulated = intersection(offset(seeds, float(seed_expansion), jtRound, 0., etOpenRound), domain);
            if (accumulated.empty())
                continue;
            double previous_area = area(accumulated);
            Polylines fronts;
            for (int iteration = 0; iteration < max_iterations; ++iteration) {
                cancel();
                Polygons next = intersection(offset(accumulated, float(spacing), jtRound, 0.), domain);
                const double next_area = area(next);
                if (next.empty() || next_area <= previous_area + min_growth)
                    break;
                Polylines level = intersection_pl(to_polylines(next), trim);
                for (Polyline &line : level)
                    line.simplify(std::min(0.05 * double(spacing), params.scaled_resolution));
                level.erase(std::remove_if(level.begin(), level.end(), [](const Polyline &line) {
                    return line.points.size() < 2;
                }), level.end());
                level = detail::reconnect_polylines(level, spacing);
                // Reconnecting tiny fragments must not create a chord across a
                // hole. Clip the resulting centerlines back to the safe domain.
                level = intersection_pl(level, trim);
                append(fronts, std::move(level));
                accumulated = std::move(next);
                previous_area = next_area;
            }
            append_wave_fronts(region_paths, fronts, flow, std::max(spacing, flow.scaled_width()), params.throw_if_canceled);
        }
        if (!region_paths.empty()) {
            for (const ExtrusionPath &path : region_paths)
                emitted.emplace_back(path.polyline.to_polyline());
            result.paths.emplace_back(std::move(region_paths));
        }
    }
    cancel();
    if (!emitted.empty())
        result.covered = intersection(offset(emitted, float(half_width), jtRound, 0., etOpenRound), to_polygons(infill_area));
    return result;
}

} // namespace Slic3r::WavePins
