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
#pragma once

#include "../inc/MarlinConfigPre.h"

#if ENABLED(M970_M979_GCODE)

struct input_shaper_runtime_t {
  bool enabled;
  uint8_t axis_mask;
  float x_hz;
  float y_hz;
  float damping;
  float smoothing;
};

extern input_shaper_runtime_t input_shaper_runtime;

void input_shaper_runtime_reset();
void input_shaper_runtime_apply(const uint8_t axis_mask, const float x_hz, const float y_hz, const float damping, const float smoothing);
float input_shaper_runtime_axis_accel_limit(const uint8_t axis_index, const float axis_max_accel);

#endif // M970_M979_GCODE