#pragma once

namespace esphome {
namespace esp32ppc {

/**
 * The orientation of the sensor's two pads in relation to the entryway being tracked.
 * The advised orientation is parallel as it maximizes the placement of the two ROIs/zones.
 */
enum Orientation { Parallel, Perpendicular };

}  // namespace esp32ppc
}  // namespace esphome
