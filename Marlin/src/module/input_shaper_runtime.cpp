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

float input_shaper_runtime_axis_accel_limit(const uint8_t axis_index, const float axis_max_accel) {
  if (!input_shaper_runtime.enabled || axis_max_accel <= 0.0f)
    return axis_max_accel;

  if (axis_index != X_AXIS && axis_index != Y_AXIS)
    return axis_max_accel;

  const bool enabled_for_axis = axis_index == X_AXIS ? axis_includes_x(input_shaper_runtime.axis_mask) : axis_includes_y(input_shaper_runtime.axis_mask);
  if (!enabled_for_axis)
    return axis_max_accel;

  const float axis_hz = axis_index == X_AXIS ? input_shaper_runtime.x_hz : input_shaper_runtime.y_hz;
  if (axis_hz <= 0.0f)
    return axis_max_accel;

  const float freq_scale = clampf(axis_hz / 60.0f, 0.45f, 1.20f);
  const float damping_scale = clampf(1.0f - input_shaper_runtime.damping * 0.40f, 0.50f, 1.00f);
  const float smoothing_scale = clampf(1.0f - input_shaper_runtime.smoothing * 0.85f, 0.20f, 1.00f);

  const float shaped = axis_max_accel * freq_scale * damping_scale * smoothing_scale;
  const float min_limited = axis_max_accel * 0.20f;
  return clampf(shaped, min_limited, axis_max_accel);
}

#endif // M970_M979_GCODE