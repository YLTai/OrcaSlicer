#pragma once

#include "libslic3r/Polygon.hpp"

namespace Slic3r::WavePins {

// A removal mask, never a replacement for the native support generator. Every
// disconnected contact component keeps pins, or keeps its original support if
// it is too narrow to fit a printable pin. Explicit enforcers always win.
struct PinContacts {
    Polygons removable;
    Polygons retained;
};

PinContacts plan_pin_contacts(const Polygons &contacts, const Polygons &wave_covered,
                              const Polygons &enforcers, double pitch_mm, double diameter_mm);

} // namespace Slic3r::WavePins
