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

/**
 * femto_bilat.h - FEMTO bilateration kinematics
 */

#include "../core/types.h"
#include "../core/macros.h"

extern float segments_per_second;

extern xy_pos_t femto_bilat_anchor_a,
                femto_bilat_anchor_b;
extern bool femto_bilat_solution_high;

void recalc_femto_bilat_settings();

void inverse_kinematics(const xyz_pos_t &raw);
void forward_kinematics(const_float_t len_a, const_float_t len_b);
