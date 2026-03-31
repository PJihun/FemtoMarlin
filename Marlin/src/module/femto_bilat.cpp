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

/**
 * femto_bilat.cpp
 */

#include "../inc/MarlinConfig.h"

#if ENABLED(FEMTO_BILAT)

#include "femto_bilat.h"
#include "motion.h"

float segments_per_second; // Initialized by settings.load()

xy_pos_t femto_bilat_anchor_a,
         femto_bilat_anchor_b;
bool femto_bilat_solution_high;

static float anchor_dx, anchor_dy, anchor_dist, anchor_dist2;

void recalc_femto_bilat_settings() {
  anchor_dx = femto_bilat_anchor_b.x - femto_bilat_anchor_a.x;
  anchor_dy = femto_bilat_anchor_b.y - femto_bilat_anchor_a.y;
  anchor_dist2 = sq(anchor_dx) + sq(anchor_dy);
  anchor_dist = SQRT(anchor_dist2);
}

void inverse_kinematics(const xyz_pos_t &raw) {
  const float dax = raw.x - femto_bilat_anchor_a.x,
              day = raw.y - femto_bilat_anchor_a.y,
              dbx = raw.x - femto_bilat_anchor_b.x,
              dby = raw.y - femto_bilat_anchor_b.y;

  delta.set(HYPOT(dax, day), HYPOT(dbx, dby), raw.z);
}


void forward_kinematics(const_float_t len_a, const_float_t len_b) {
  if (anchor_dist <= 0.0f) {
    cartes.x = femto_bilat_anchor_a.x;
    cartes.y = femto_bilat_anchor_a.y;
    return;
  }

  const float a = (sq(len_a) - sq(len_b) + anchor_dist2) / (2.0f * anchor_dist);
  const float h2 = sq(len_a) - sq(a);
  const float h = SQRT(_MAX(h2, 0.0f));

  const float ux = anchor_dx / anchor_dist,
              uy = anchor_dy / anchor_dist;

  const float p0x = femto_bilat_anchor_a.x + a * ux,
              p0y = femto_bilat_anchor_a.y + a * uy;

  const float px = -uy,
              py = ux;

  const float sign = femto_bilat_solution_high ? 1.0f : -1.0f;
  cartes.x = p0x + sign * h * px;
  cartes.y = p0y + sign * h * py;
}

#endif // FEMTO_BILAT
