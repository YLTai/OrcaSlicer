#include <catch2/catch_test_macros.hpp>

#include "test_helpers.hpp"

#include "libslic3r/Config.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/TriangleMesh.hpp"

#include <cstddef>
#include <sstream>
#include <string>

using namespace Slic3r;

namespace {

TriangleMesh anchored_shelf(double overhang = 8.)
{
    TriangleMesh stem = make_cube(12. - overhang, 20., 6.);
    TriangleMesh shelf = make_cube(12., 20., 1.2);
    // The solids overlap to avoid an ambiguous coplanar join at the first
    // overhanging layer.
    shelf.translate(0.f, 0.f, 5.8f);
    stem.merge(shelf);
    return stem;
}

DynamicPrintConfig wave_print_config()
{
    return Test::multifilament_config(1, {
        {"layer_height", 0.2},
        {"initial_layer_print_height", 0.2},
        {"layer_change_gcode", "G92 E0\n"},
        {"wall_generator", "classic"},
        {"wall_loops", 2},
        {"detect_overhang_wall", true},
        {"enable_support", true},
        {"support_type", "normal(auto)"},
        {"support_style", "snug"},
        {"support_top_z_distance", 0.2},
        {"support_interface_top_layers", 2},
        {"raft_layers", 0},
        {"wave_pins_max_span", 12.},
        {"wave_pins_filter_supported", true},
        {"wave_pins_outer_walls", 1},
        {"wave_pins_floor_layers", 2},
        {"wave_pins_pin_spacing", 6.},
        {"wave_pins_pin_diameter", 1.2},
    });
}

std::string machine_commands(const std::string &gcode)
{
    // Version strings, timestamps and serialized settings are comments. Keep
    // every executable command, including speeds, fan control and extrusion,
    // in its emitted order so a supposedly disabled feature cannot hide here.
    std::istringstream input(gcode);
    std::string output;
    std::string line;
    while (std::getline(input, line)) {
        const auto comment = line.find(';');
        if (comment != std::string::npos)
            line.erase(comment);
        const auto last = line.find_last_not_of(" \t\r");
        if (last == std::string::npos)
            continue;
        line.erase(last + 1);
        output += line + '\n';
    }
    return output;
}

bool has_wave_coverage(const Print &print)
{
    for (const PrintObject *object : print.objects())
        for (const Layer *layer : object->layers())
            if (!layer->wave_pins_covered.empty())
                return true;
    return false;
}

void require_no_wave_state(const Print &print)
{
    for (const PrintObject *object : print.objects())
        for (const Layer *layer : object->layers()) {
            REQUIRE(layer->wave_pins_covered.empty());
            REQUIRE(layer->wave_pins_floor.empty());
        }
}

} // namespace

TEST_CASE("Disabled Wave settings leave native machine commands unchanged", "[WavePins]")
{
    DynamicPrintConfig config = wave_print_config();
    REQUIRE_FALSE(config.opt_bool("wave_pins_enabled"));
    Print print;
    Model model;
    Test::init_print({anchored_shelf()}, print, model, config);
    const std::string native = machine_commands(Test::gcode(print));
    REQUIRE_FALSE(native.empty());
    require_no_wave_state(print);

    config.set_deserialize_strict({
        {"wave_pins_enabled", false},
        {"wave_pins_filter_supported", false},
        {"wave_pins_max_span", 3.},
        {"wave_pins_speed", 1.},
        {"wave_pins_spacing", 0.8},
        {"wave_pins_pin_spacing", 20.},
        {"wave_pins_pin_diameter", 2.},
        {"wave_pins_fan_speed", 100},
        {"wave_pins_floor_layers", 0},
        {"wave_pins_floor_speed", 1.},
    });
    print.apply(model, config);
    const std::string retuned = machine_commands(Test::gcode(print));
    require_no_wave_state(print);
    REQUIRE(retuned == native);
}

TEST_CASE("Filtering shallow Wave overhangs rebuilds native model and support paths", "[WavePins][Regression]")
{
    DynamicPrintConfig config = wave_print_config();
    config.set_deserialize_strict({
        {"wave_pins_enabled", true},
        {"wave_pins_filter_supported", false},
        {"inner_wall_line_width", 0.45},
        {"outer_wall_line_width", 0.45},
        {"bridge_line_width", "100%"},
    });
    Print print;
    Model model;
    // After preserving one wall, this 0.5 mm ledge leaves a shallow candidate
    // extending less than half a Wave bead beyond the previous layer.
    Test::init_print({anchored_shelf(0.5)}, print, model, config);
    const std::string unfiltered = Test::gcode(print);
    REQUIRE(has_wave_coverage(print));
    REQUIRE(unfiltered.find(";_WAVE_PINS_START\n") != std::string::npos);

    config.set_deserialize_strict({{"wave_pins_filter_supported", true}});
    print.apply(model, config);
    const PrintObject *object = print.get_object(std::size_t(0));
    REQUIRE_FALSE(object->is_step_done(posPerimeters));
    REQUIRE_FALSE(object->is_step_done(posSupportMaterial));
    const std::string filtered = Test::gcode(print);
    require_no_wave_state(print);
    REQUIRE(filtered.find(";_WAVE_PINS_START\n") == std::string::npos);

    config.set_deserialize_strict({{"wave_pins_filter_supported", false}});
    print.apply(model, config);
    REQUIRE(machine_commands(Test::gcode(print)) == machine_commands(unfiltered));
}

TEST_CASE("Switching Wave off after slicing restores native output", "[WavePins][Regression]")
{
    DynamicPrintConfig config = wave_print_config();
    config.set_deserialize_strict({{"wave_pins_enabled", false}, {"gcode_comments", false}});
    Print print;
    Model model;
    Test::init_print({anchored_shelf()}, print, model, config);
    const std::string native = machine_commands(Test::gcode(print));
    REQUIRE_FALSE(native.empty());

    config.set_deserialize_strict({{"wave_pins_enabled", true}});
    print.apply(model, config);
    const std::string experimental_gcode = Test::gcode(print);
    const std::string experimental = machine_commands(experimental_gcode);
    // Preview identity must survive export even with descriptive comments off.
    REQUIRE(experimental_gcode.find(";_WAVE_PINS_START\n") != std::string::npos);
    REQUIRE(experimental_gcode.find(";_WAVE_PINS_END\n") != std::string::npos);
    // A no-op enable path would make the off/on/off comparison meaningless.
    REQUIRE(has_wave_coverage(print));
    REQUIRE(experimental != native);

    config.set_deserialize_strict({{"wave_pins_enabled", false}});
    print.apply(model, config);
    const std::string restored = machine_commands(Test::gcode(print));
    require_no_wave_state(print);
    REQUIRE(restored == native);
}

TEST_CASE("Wave support preview follows perimeter prerequisites and invalidation", "[WavePins][Regression]")
{
    DynamicPrintConfig config = wave_print_config();
    config.set_deserialize_strict({
        {"wave_pins_enabled", true},
        {"bridge_line_width", "100%"},
    });
    Print print;
    Model model;
    Test::init_print({anchored_shelf()}, print, model, config);
    PrintObject *object = print.get_object(std::size_t(0));
    object->slice();
    REQUIRE_FALSE(object->is_step_done(posPerimeters));

    // The public GUI preview enters generate_support_material() without a
    // complete Print::process(). Pins must still see generated Wave paths.
    object->generate_support_preview();
    REQUIRE(object->is_step_done(posPerimeters));
    REQUIRE(object->is_step_done(posSupportMaterial));
    REQUIRE(has_wave_coverage(print));

    config.set_deserialize_strict({{"wall_loops", 3}});
    print.apply(model, config);
    object = print.get_object(std::size_t(0));
    REQUIRE_FALSE(object->is_step_done(posPerimeters));
    REQUIRE_FALSE(object->is_step_done(posSupportMaterial));
    object->generate_support_preview();
    REQUIRE(object->is_step_done(posPerimeters));
    REQUIRE(object->is_step_done(posSupportMaterial));

    // Wave flow uses bridge width while generating perimeters, so changing
    // this otherwise infill-related setting must rebuild contact placement.
    config.set_deserialize_strict({{"bridge_line_width", "90%"}});
    print.apply(model, config);
    object = print.get_object(std::size_t(0));
    REQUIRE_FALSE(object->is_step_done(posPerimeters));
    REQUIRE_FALSE(object->is_step_done(posSupportMaterial));

    object->generate_support_preview();
    config.set_deserialize_strict({{"wave_pins_filter_supported", false}});
    print.apply(model, config);
    object = print.get_object(std::size_t(0));
    REQUIRE_FALSE(object->is_step_done(posPerimeters));
    REQUIRE_FALSE(object->is_step_done(posSupportMaterial));
}
