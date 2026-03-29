/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */

#include "../inc/MarlinConfig.h"

#if ENABLED(M970_M979_GCODE)

#include "input_shaper_runtime.h"

namespace {

  constexpr float IS_RUNTIME_MIN_FREQ_HZ = 10.0f;
  constexpr float IS_RUNTIME_MAX_FREQ_HZ = 120.0f;
  constexpr float IS_RUNTIME_DEFAULT_DAMPING = 0.10f;
  constexpr float IS_RUNTIME_DEFAULT_SMOOTHING = 0.00f;

  float clampf(const float value, const float low, const float high) {
    if (value < low) return low;
    if (value > high) return high;
    return value;
  }

  bool axis_includes_x(const uint8_t axis_mask) { return axis_mask & 0x01; }
  bool axis_includes_y(const uint8_t axis_mask) { return axis_mask & 0x02; }

}

input_shaper_runtime_t input_shaper_runtime = {
  false,
  3,
  0.0f,
  0.0f,
  IS_RUNTIME_DEFAULT_DAMPING,
  IS_RUNTIME_DEFAULT_SMOOTHING
};

void input_shaper_runtime_reset() {
  input_shaper_runtime.enabled = false;
  input_shaper_runtime.axis_mask = 3;
  input_shaper_runtime.x_hz = 0.0f;
  input_shaper_runtime.y_hz = 0.0f;
  input_shaper_runtime.damping = IS_RUNTIME_DEFAULT_DAMPING;
  input_shaper_runtime.smoothing = IS_RUNTIME_DEFAULT_SMOOTHING;
}

void input_shaper_runtime_apply(const uint8_t axis_mask, const float x_hz, const float y_hz, const float damping, const float smoothing) {
  const uint8_t normalized_mask = axis_mask & 0x03;
  const uint8_t effective_mask = normalized_mask ? normalized_mask : 0x03;

  if (axis_includes_x(effective_mask))
    input_shaper_runtime.x_hz = x_hz > 0.0f ? clampf(x_hz, IS_RUNTIME_MIN_FREQ_HZ, IS_RUNTIME_MAX_FREQ_HZ) : 0.0f;

  if (axis_includes_y(effective_mask))
    input_shaper_runtime.y_hz = y_hz > 0.0f ? clampf(y_hz, IS_RUNTIME_MIN_FREQ_HZ, IS_RUNTIME_MAX_FREQ_HZ) : 0.0f;

  input_shaper_runtime.damping = clampf(damping, 0.03f, 0.35f);
  input_shaper_runtime.smoothing = clampf(smoothing, 0.0f, 0.20f);

  uint8_t active_axes = 0;
  if (input_shaper_runtime.x_hz > 0.0f) active_axes |= 0x01;
  if (input_shaper_runtime.y_hz > 0.0f) active_axes |= 0x02;

  input_shaper_runtime.enabled = active_axes;
  input_shaper_runtime.axis_mask = active_axes ? active_axes : effective_mask;
}

#endif // M970_M979_GCODE