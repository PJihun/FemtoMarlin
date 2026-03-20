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


#include "env_validate.h"

#if EXTRUDERS > 2 || E_STEPPERS > 2
#error "MKS ESP Nano only supports two E Steppers. Comment out this line to continue."
#elif HOTENDS > 2
#error "MKS ESP Nano only supports two hotend / E-stepper. Comment out this line to continue."
#endif

#define BOARD_INFO_NAME      "MKS TinyBee V2"
#define BOARD_WEBSITE_URL    "https://github.com/makerbase-mks"
#define DEFAULT_MACHINE_NAME BOARD_INFO_NAME

// #define MKS_TEST            43

//
// Servos
//
//#define SERVO0_PIN                            47

//#define NEOPIXEL_PIN                          48  //different than LCD_D5 ? => 2 Neopixels : one on LCD one on board ?

//
// Limit Switches
//
#define X_STOP_PIN                            16
#define Y_STOP_PIN                            15
#define Z_STOP_PIN                            7
// #define FIL_RUNOUT_PIN                        3

#define X_DIAG_PIN                          X_STOP_PIN
#define Y_DIAG_PIN                          X_STOP_PIN
#define Z_DIAG_PIN                          X_STOP_PIN
#define E0_DIAG_PIN                         3
//
// Enable I2S stepper stream
//
#undef I2S_STEPPER_STREAM
#define I2S_STEPPER_STREAM
#define I2S_WS                                19
#define I2S_BCK                               8
#define I2S_DATA                              20
#undef LIN_ADVANCE                                // Currently, I2S stream does not work with linear advance

//
// Steppers
//
#define X_STEP_PIN                           129
#define X_DIR_PIN                            130
#define X_ENABLE_PIN                         128

#define Y_STEP_PIN                           132
#define Y_DIR_PIN                            133
#define Y_ENABLE_PIN                         131

#define Z_STEP_PIN                           135
#define Z_DIR_PIN                            136
#define Z_ENABLE_PIN                         134

#define E0_STEP_PIN                          138
#define E0_DIR_PIN                           139
#define E0_ENABLE_PIN                        137

//#define E1_STEP_PIN                          141
//#define E1_DIR_PIN                           142
//#define E1_ENABLE_PIN                        140

//#define Z2_STEP_PIN                          141
//#define Z2_DIR_PIN                           142
//#define Z2_ENABLE_PIN                        140
#undef X2_DIR_PIN

#if HAS_TMC_UART
//
// Software serial
// No Hardware serial for steppers
//
#define X_SERIAL_TX_PIN                   14
#define X_SERIAL_RX_PIN        X_SERIAL_TX_PIN

#define Y_SERIAL_TX_PIN                   21
#define Y_SERIAL_RX_PIN        Y_SERIAL_TX_PIN

#define Z_SERIAL_TX_PIN                   10
#define Z_SERIAL_RX_PIN        Z_SERIAL_TX_PIN

#define E0_SERIAL_TX_PIN                  9
#define E0_SERIAL_RX_PIN      E0_SERIAL_TX_PIN

//#define E1_SERIAL_TX_PIN                  46
//#define E1_SERIAL_RX_PIN      E1_SERIAL_TX_PIN

//#define Z2_SERIAL_TX_PIN                  46
//#define Z2_SERIAL_RX_PIN      Z2_SERIAL_TX_PIN

// Reduce baud rate to improve software serial reliability
#define TMC_BAUD_RATE                    19200
#endif
//
// Temperature Sensors
//
#define TEMP_0_PIN                            4  // Analog Input
#define TEMP_1_PIN                            5  // Analog Input, you need set R6=0Ω and R7=NC
#define TEMP_BED_PIN                          6  // Analog Input

//
// Heaters / Fans
//
#define HEATER_0_PIN                         145
#ifndef MKS_TEST
#define HEATER_1_PIN                         146
#define FAN_PIN                              147
#define FAN1_PIN                             148
#endif
#define HEATER_BED_PIN                       144

//#define CONTROLLER_FAN_PIN                 148
//#define E0_AUTO_FAN_PIN                    148  // need to update Configuration_adv.h @section extruder
//#define E1_AUTO_FAN_PIN                    149  // need to update Configuration_adv.h @section extruder
#ifdef MKS_TEST
#define HEATER_1_PIN_T                         146
#define FAN_PIN_T                              147
#define FAN1_PIN_T                             148
#define HEATER_BED_PIN_T                       144
#endif

//
// ADC Reference Voltage
//
//#define ADC_REFERENCE_VOLTAGE                2.6  // 2.6V reference VDDA


//
// MicroSD card
//
#define SD_MOSI_PIN                           35
#define SD_MISO_PIN                           37
#define SD_SCK_PIN                            36
#define SDSS                                  38
#define SD_DETECT_PIN                         39 // IO34 default is SD_DET signal(Jump to SDDET)
#define USES_SHARED_SPI                       // SPI is shared by SD card with TMC SPI drivers

/**
 *                _____                                             _____
 * (BEEPER)IO149 |10 9 | IO11(BTN_ENC)             (SPI MISO) IO37 |10 9 | IO36 (SPI SCK)
 *  (LCD_EN)IO41 | 8 7 | IO40(LCD_RS)               (BTN_EN1) IO12 | 8 7 | IO38 (SPI CS)
 *  (LCD_D4)IO0  | 6 5   IO42(LCD_D5)               (BTN_EN2) IO13 | 6 5   IO35 (SPI MOSI)
 *  (LCD_D6)IO45 | 4 3 | IO01(LCD_D7)               (SPI_DET) IO39 | 4 3 | RESET
 *           GND | 2 1 | 5V                                    GND | 2 1 | 3.3V
 *                ￣￣￣                                            ￣￣￣
 *                EXP1                                               EXP2
 */

#define EXP1_03_PIN                         1
#define EXP1_04_PIN                         45
#define EXP1_05_PIN                         42
#define EXP1_06_PIN                         0
#define EXP1_07_PIN                         40
#define EXP1_08_PIN                         41
#define EXP1_09_PIN                         11
#define EXP1_10_PIN                         149

#define EXP2_03_PIN                         -1    // RESET
#define EXP2_04_PIN                         39
#define EXP2_05_PIN                         35
#define EXP2_06_PIN                         13
#define EXP2_07_PIN                         38
#define EXP2_08_PIN                         12
#define EXP2_09_PIN                         36
#define EXP2_10_PIN                         37

#if HAS_WIRED_LCD

#define BEEPER_PIN                          149
#define BTN_ENC                             11
#define LCD_PINS_ENABLE                     41
#define LCD_PINS_RS                         40
#define BTN_EN1                             12
#define BTN_EN2                             13
#define LCD_BACKLIGHT_PIN                   -1

// MKS MINI12864 and MKS LCD12864B; If using MKS LCD12864A (Need to remove RPK2 resistor)
#if ENABLED(MKS_MINI_12864)
//#define LCD_BACKLIGHT_PIN             -1
//#define LCD_RESET_PIN                 -1
#define DOGLCD_A0                       45
#define DOGLCD_CS                       42
//#define DOGLCD_SCK                    37
//#define DOGLCD_MOSI                   35

// Required for MKS_MINI_12864 with this board
//#define MKS_LCD12864B

#elif ENABLED(MKS_MINI_12864_V3)
#define DOGLCD_CS                       EXP1_08_PIN
#define DOGLCD_A0                       EXP1_07_PIN
#define LCD_PINS_DC                     DOGLCD_A0
#define LCD_BACKLIGHT_PIN               -1
#define LCD_RESET_PIN                   EXP1_06_PIN
#ifndef NEOPIXEL_PIN
#define NEOPIXEL_PIN                    EXP1_05_PIN // NeoPixel on EXP1_05_PIN
#endif // NEOPIXEL_PIN
#define DOGLCD_MOSI                     EXP2_05_PIN
#define DOGLCD_SCK                      EXP2_09_PIN
#if SD_CONNECTION_IS(ONBOARD)
#define FORCE_SOFT_SPI
#endif
#else // !MKS_MINI_12864

#define LCD_PINS_D4                     0
#if ENABLED(REPRAP_DISCOUNT_SMART_CONTROLLER)
#define LCD_PINS_D5                   42
#define LCD_PINS_D6                   45
#define LCD_PINS_D7                   1
#endif

#define ST7920_DELAY_1         DELAY_NS(96)
#define ST7920_DELAY_2         DELAY_NS(48)
#define ST7920_DELAY_3         DELAY_NS(600)

// #define BOARD_ST7920_DELAY_1    DELAY_NS(96)
// #define BOARD_ST7920_DELAY_2    DELAY_NS(48)
// #define BOARD_ST7920_DELAY_3    DELAY_NS(600)

#endif // !MKS_MINI_12864

#endif // HAS_WIRED_LCD


