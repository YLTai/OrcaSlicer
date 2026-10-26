#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "libslic3r/ExtrusionEntity.hpp"
#include "libslic3r/GCode/GCodeProcessor.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "test_utils.hpp"

#include <algorithm>
#include <cstddef>
#include <fstream>
#include <string>

using namespace Slic3r;
using Catch::Matchers::WithinAbs;

namespace {

struct CompatibleTags {
    bool previous = GCodeProcessor::s_IsBBLPrinter;
    CompatibleTags() { GCodeProcessor::s_IsBBLPrinter = false; }
    ~CompatibleTags() { GCodeProcessor::s_IsBBLPrinter = previous; }
};

std::string preview_gcode(bool mark_wave)
{
    const std::string begin = mark_wave ? ";_WAVE_PINS_START\n" : "";
    const std::string end = mark_wave ? ";_WAVE_PINS_END\n" : "";
    return "G90\nG21\nM83\n;TYPE:Overhang wall\n;WIDTH:0.45\n;HEIGHT:0.2\n"
           "G1 X0 Y0 Z0.2 F1200\nG1 X10 Y0 E0.4\n" + begin +
           "G1 X20 Y0 E0.4\nG1 E-0.6 F1800\nG0 X20 Y2 F6000\nG1 E0.6 F1800\n"
           "G2 X30 Y2 I5 J0 E0.6 F600\n" + end +
           "G1 X40 Y2 E0.4 F1200\n;TYPE:Outer wall\nG1 X50 Y2 E0.4\n";
}

void process(GCodeProcessor &processor, const std::string &gcode)
{
    processor.apply_config(FullPrintConfig{});
    processor.initialize_result_moves();
    processor.process_buffer(gcode);
    processor.finalize(false);
}

std::size_t wave_move_count(const GCodeProcessorResult &result)
{
    return std::count_if(result.moves.begin(), result.moves.end(),
                         [](const auto &move) { return move.wave_pins; });
}

void require_scoped_preview(const GCodeProcessorResult &result)
{
    std::size_t arc_wave_moves = 0;
    std::size_t ordinary_overhangs = 0;
    std::size_t ordinary_walls = 0;
    std::size_t travels_or_retractions = 0;
    for (const auto &move : result.moves) {
        if (move.wave_pins) {
            REQUIRE(move.type == EMoveType::Extrude);
            REQUIRE(move.extrusion_role == erOverhangPerimeter);
            if (move.position.x() > 20.01f)
                ++arc_wave_moves;
        }
        if (move.type != EMoveType::Extrude) {
            REQUIRE_FALSE(move.wave_pins);
            if (move.type == EMoveType::Travel || move.type == EMoveType::Retract)
                ++travels_or_retractions;
        } else if (move.extrusion_role == erExternalPerimeter) {
            REQUIRE_FALSE(move.wave_pins);
            ++ordinary_walls;
        } else if (move.position.x() > 30.01f) {
            REQUIRE_FALSE(move.wave_pins);
            ++ordinary_overhangs;
        }
    }
    REQUIRE(arc_wave_moves > 1);
    REQUIRE(ordinary_overhangs > 0);
    REQUIRE(ordinary_walls > 0);
    REQUIRE(travels_or_retractions > 0);
}

} // namespace

TEST_CASE("Wave preview scopes include arcs and exclude following paths and travel", "[WavePinsPreview]")
{
    CompatibleTags tags;
    GCodeProcessor marked;
    process(marked, preview_gcode(true));
    const auto &result = marked.get_result();
    REQUIRE(wave_move_count(result) > 0);
    require_scoped_preview(result);

    GCodeProcessor native;
    process(native, preview_gcode(false));
    const auto &baseline = native.get_result();
    REQUIRE(wave_move_count(baseline) == 0);
    REQUIRE(baseline.print_statistics.used_filaments_wave_pins_per_role.empty());
    REQUIRE(result.moves.size() == baseline.moves.size());
    for (std::size_t i = 0; i < result.moves.size(); ++i) {
        const auto &actual = result.moves[i];
        const auto &expected = baseline.moves[i];
        REQUIRE(actual.type == expected.type);
        REQUIRE(actual.extrusion_role == expected.extrusion_role);
        REQUIRE(actual.position.isApprox(expected.position, 1e-6f));
        REQUIRE_THAT(actual.width, WithinAbs(expected.width, 1e-6));
        REQUIRE_THAT(actual.height, WithinAbs(expected.height, 1e-6));
        REQUIRE_THAT(actual.mm3_per_mm, WithinAbs(expected.mm3_per_mm, 1e-6));
        REQUIRE_THAT(actual.feedrate, WithinAbs(expected.feedrate, 1e-6));
    }
    const auto &wave_usage = result.print_statistics.used_filaments_wave_pins_per_role;
    REQUIRE(wave_usage.count(erOverhangPerimeter) == 1);
    const auto &total = result.print_statistics.used_filaments_per_role.at(erOverhangPerimeter);
    const auto &ordinary_total = baseline.print_statistics.used_filaments_per_role.at(erOverhangPerimeter);
    REQUIRE(wave_usage.at(erOverhangPerimeter).first > 0.0);
    REQUIRE(wave_usage.at(erOverhangPerimeter).first < total.first);
    REQUIRE_THAT(total.first, WithinAbs(ordinary_total.first, 1e-8));
    REQUIRE_THAT(total.second, WithinAbs(ordinary_total.second, 1e-8));
}

TEST_CASE("Reopening exported Wave comments restores the independent preview identity", "[WavePinsPreview]")
{
    CompatibleTags tags;
    ScopedTemporaryFile file(".gcode");
    {
        std::ofstream stream(file.string());
        REQUIRE(stream.is_open());
        stream << preview_gcode(true);
        REQUIRE(stream.good());
    }
    GCodeProcessor processor;
    processor.apply_config(FullPrintConfig{});
    processor.process_file(file.string());
    require_scoped_preview(processor.get_result());
}

TEST_CASE("A new physical role and processor reset clear unfinished Wave preview scopes", "[WavePinsPreview]")
{
    CompatibleTags tags;
    GCodeProcessor processor;
    process(processor,
            "G90\nG21\nM83\nG1 X0 Y0 Z0.2 F1200\n;TYPE:Overhang wall\n"
            ";_WAVE_PINS_START\nG1 X10 Y0 E0.4\n"
            ";TYPE:Outer wall\nG1 X20 Y0 E0.4\n");
    REQUIRE(wave_move_count(processor.get_result()) > 0);
    for (const auto &move : processor.get_result().moves)
        if (move.extrusion_role == erExternalPerimeter)
            REQUIRE_FALSE(move.wave_pins);

    // Leave a fresh scope open immediately before reset.
    processor.process_buffer(";_WAVE_PINS_START\n");
    processor.reset();
    process(processor, preview_gcode(false));
    REQUIRE(wave_move_count(processor.get_result()) == 0);
    REQUIRE(processor.get_result().print_statistics.used_filaments_wave_pins_per_role.empty());
}
