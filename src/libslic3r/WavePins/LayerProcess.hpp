#pragma once

namespace Slic3r {
class PrintObject;
namespace WavePins {

// Called at the existing prepare-infill and infill boundaries. Both are inert
// when the object's master switch is off.
void prepare_floor_surfaces(PrintObject &object);
void tag_floor_paths(PrintObject &object);

} // namespace WavePins
} // namespace Slic3r
