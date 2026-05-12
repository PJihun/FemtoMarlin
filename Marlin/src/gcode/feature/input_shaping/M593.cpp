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

#include "../../../inc/MarlinConfig.h"

#if HAS_ZV_SHAPING

#include "../../gcode.h"
#include "../../../module/stepper.h"

void GcodeSuite::M593_report(const bool forReplay/*=true*/) {
  report_heading(forReplay, F("Input Shaping"));

  #if ENABLED(INPUT_SHAPING_X)
    report_echo_start(forReplay);
    SERIAL_ECHOLNPGM("  M593 X"
      " F", stepper.get_shaping_frequency(X_AXIS),
      " D", stepper.get_shaping_damping_ratio(X_AXIS)
    );
  #endif
  #if ENABLED(INPUT_SHAPING_Y)
    report_echo_start(forReplay);
    SERIAL_ECHOLNPGM("  M593 Y"
      " F", stepper.get_shaping_frequency(Y_AXIS),
      " D", stepper.get_shaping_damping_ratio(Y_AXIS)
    );
  #endif
}

/**
 * M593: Get or Set Input Shaping Parameters
 *  D<factor>    Set damping factor (0..0.99).
 *  F<frequency> Set frequency in Hz. Use 0 to disable shaping for the axis.
 *  X            Apply to X axis.
 *  Y            Apply to Y axis.
 */
void GcodeSuite::M593() {
  if (!parser.seen_any()) return M593_report();

  const bool seen_X = TERN0(INPUT_SHAPING_X, parser.seen_test('X')),
             seen_Y = TERN0(INPUT_SHAPING_Y, parser.seen_test('Y')),
             all_axes = !seen_X && !seen_Y,
             for_X = TERN0(INPUT_SHAPING_X, seen_X || all_axes),
             for_Y = TERN0(INPUT_SHAPING_Y, seen_Y || all_axes);

  if (parser.seen('D')) {
    const float zeta = parser.value_float();
    if (!WITHIN(zeta, 0.0f, 0.99f))
      SERIAL_ECHO_MSG("?Zeta (D) value out of range (0-0.99)");
    else {
      if (for_X) stepper.set_shaping_damping_ratio(X_AXIS, zeta);
      if (for_Y) stepper.set_shaping_damping_ratio(Y_AXIS, zeta);
    }
  }

  if (parser.seen('F')) {
    const float freq = parser.value_float();
    if (freq < 0)
      SERIAL_ECHO_MSG("?Frequency (F) must be >= 0");
    else {
      if (for_X) stepper.set_shaping_frequency(X_AXIS, freq);
      if (for_Y) stepper.set_shaping_frequency(Y_AXIS, freq);
    }
  }
}

#endif
