#include <catch2/catch_test_macros.hpp>

#include "libvgcode/include/Types.hpp"
#include "libvgcode/include/Viewer.hpp"
#include "libslic3r/ExtrusionEntity.hpp"
#include "libslic3r/GCode/GCodeProcessor.hpp"
#include "slic3r/GUI/LibVGCode/LibVGCodeWrapper.hpp"

#include <array>
#include <cstddef>
#include <initializer_list>

TEST_CASE("Wave has a distinct preview color while existing role colors stay unchanged", "[WavePinsPreview]")
{
    using Role = libvgcode::EGCodeExtrusionRole;
    // Append-only compatibility: these existing role indices and colors also
    // serve other preview modes and must not shift when Wave is introduced.
    REQUIRE(static_cast<std::size_t>(Role::Mixed) == 19);
    REQUIRE(static_cast<std::size_t>(Role::WavePins) == 20);
    const std::array<libvgcode::Color, 20> existing{{
        {230, 179, 179}, {255, 230, 77}, {255, 125, 56}, {31, 31, 255},
        {176, 48, 41}, {150, 84, 204}, {240, 64, 64}, {255, 140, 105},
        {77, 128, 186}, {255, 255, 255}, {0, 135, 110}, {0, 255, 0},
        {0, 128, 0}, {179, 227, 171}, {94, 209, 148}, {102, 92, 199},
        {77, 128, 186}, {0, 59, 110}, {0, 64, 0}, {128, 128, 128}
    }};
    libvgcode::Viewer viewer;
    const libvgcode::Color wave = viewer.get_extrusion_role_color(Role::WavePins);
    REQUIRE(wave == libvgcode::Color{0, 229, 255});
    for (std::size_t i = 0; i < existing.size(); ++i) {
        REQUIRE(viewer.get_extrusion_role_color(static_cast<Role>(i)) == existing[i]);
        int distance_squared = 0;
        for (std::size_t channel = 0; channel < 3; ++channel) {
            const int difference = int(wave[channel]) - int(existing[i][channel]);
            distance_squared += difference * difference;
        }
        // More than 100 RGB units from every previous role, not just unequal.
        REQUIRE(distance_squared > 100 * 100);
    }
}

TEST_CASE("The Wave preview filter is visible by default and independent of ordinary overhangs", "[WavePinsPreview]")
{
    using Role = libvgcode::EGCodeExtrusionRole;
    libvgcode::Viewer viewer;
    REQUIRE(viewer.is_extrusion_role_visible(Role::WavePins));
    REQUIRE(viewer.is_extrusion_role_visible(Role::OverhangPerimeter));
    viewer.toggle_extrusion_role_visibility(Role::WavePins);
    REQUIRE_FALSE(viewer.is_extrusion_role_visible(Role::WavePins));
    REQUIRE(viewer.is_extrusion_role_visible(Role::OverhangPerimeter));
    viewer.toggle_extrusion_role_visibility(Role::WavePins);
    REQUIRE(viewer.is_extrusion_role_visible(Role::WavePins));
}

TEST_CASE("Only marked extrusion moves acquire the separate Wave rendering role", "[WavePinsPreview]")
{
    using Role = libvgcode::EGCodeExtrusionRole;
    Slic3r::GCodeProcessorResult::MoveVertex move;
    move.type = Slic3r::EMoveType::Extrude;
    move.extrusion_role = Slic3r::erOverhangPerimeter;
    REQUIRE(libvgcode::convert(move) == Role::OverhangPerimeter);
    move.wave_pins = true;
    REQUIRE(libvgcode::convert(move) == Role::WavePins);
    REQUIRE(move.extrusion_role == Slic3r::erOverhangPerimeter);
    for (Slic3r::EMoveType type : {Slic3r::EMoveType::Travel, Slic3r::EMoveType::Retract,
                                 Slic3r::EMoveType::Unretract, Slic3r::EMoveType::Wipe}) {
        move.type = type;
        REQUIRE(libvgcode::convert(move) != Role::WavePins);
    }
    move.type = Slic3r::EMoveType::Extrude;
    move.wave_pins = false;
    REQUIRE(libvgcode::convert(move) == Role::OverhangPerimeter);
}
