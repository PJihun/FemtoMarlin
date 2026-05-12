#pragma once

/**
 * Fixed-Time Motion (FTM) / Input Shaping Data Types & Settings
 * 
 * Defines core structures, limits, and runtime settings for the FTM 
 * trajectory engine. The FTM engine replaces standard stepping and uses
 * a fixed time-step model, making it mathematically ideal for advanced
 * velocity profiles and input shaping algorithms (like ZV, EI).
 */

#include <stdint.h>
#include "../core/macros.h"
#include "../core/types.h"

/**
 * Buffer Sizes
 * FTM_STEPPERCMD_BUFF_SIZE : Size of the ring buffer holding finalized stepper commands
 * FTM_WINDOW_SIZE          : Moving window queue size for trajectory planning
 * FTM_BATCH_SIZE           : Standard batch size for trajectory point calculation
 * FTM_ZMAX                 : Maximum step delay window allowed for Input Shaping
 *                            (sized for ZVDDD/3HEI at FTM_MIN_SHAPE_FREQ with zeta <= 0.99)
 */
#define FTM_STEPPERCMD_BUFF_SIZE 1024
#define FTM_WINDOW_SIZE 64
#define FTM_BATCH_SIZE  32
#define FTM_ZMAX 3072

// Stepper / Execution Rates
#define FTM_STEPS_PER_UNIT_TIME 5
#define FTM_STEPPER_FS 50000.0f

// Trajectory generation frequency
#define FTM_FS 1000.0f
#define FTM_TS (1.0f / (FTM_FS))
#define FTM_MIN_SHAPE_FREQ 5.0f

#ifndef ENABLED
  #define ENABLED(b) (!!(b))
#endif

// Configuration default fallbacks
#define FTM_LINEAR_ADV_DEFAULT_ENA 0
#define FTM_LINEAR_ADV_DEFAULT_K 0.0f
#define FTM_DEFAULT_DYNFREQ_MODE dynFreqMode_DISABLED
#define FTM_IS_DEFAULT_MOTION 1

typedef uint32_t ft_command_t;
typedef xyze_bool_t AxisBits;

enum FT_BIT {
  FT_BIT_STEP_X = 0, FT_BIT_DIR_X, FT_BIT_STEP_x = 0, FT_BIT_DIR_x = 1,
  FT_BIT_STEP_Y=2, FT_BIT_DIR_Y=3, FT_BIT_STEP_y = 2, FT_BIT_DIR_y = 3,
  FT_BIT_STEP_Z=4, FT_BIT_DIR_Z=5, FT_BIT_STEP_z = 4, FT_BIT_DIR_z = 5,
  FT_BIT_STEP_I, FT_BIT_DIR_I,
  FT_BIT_STEP_J, FT_BIT_DIR_J,
  FT_BIT_STEP_K, FT_BIT_DIR_K,
  FT_BIT_STEP_U, FT_BIT_DIR_U,
  FT_BIT_STEP_V, FT_BIT_DIR_V,
  FT_BIT_STEP_W, FT_BIT_DIR_W,
  FT_BIT_SYNC = 29, FT_BIT_STEP_e = 30, FT_BIT_DIR_e = 31,
  FT_BIT_STEP_E = 30, FT_BIT_DIR_E = 31
};

typedef enum FTMotionShaper {
  ftMotionShaper_NONE,
  ftMotionShaper_ZV,
  ftMotionShaper_ZVD,
  ftMotionShaper_ZVDD,
  ftMotionShaper_ZVDDD,
  ftMotionShaper_EI,
  ftMotionShaper_2HEI,
  ftMotionShaper_3HEI,
  ftMotionShaper_MZV
} ftMotionShaper_t;

enum dynFreqMode_t : uint8_t {
  dynFreqMode_DISABLED,
  dynFreqMode_Z_BASED,
  dynFreqMode_MASS_BASED
};

struct ft_shaped_shaper_t {
  ftMotionShaper_t x;
  ftMotionShaper_t y;
  ftMotionShaper_t& operator[](const int i) { return i == 0 ? x : y; }
  const ftMotionShaper_t& operator[](const int i) const { return i == 0 ? x : y; }
};

struct ft_shaped_float_t {
  float x;
  float y;
  float& operator[](const int idx) { return idx == 0 ? x : y; }
  const float& operator[](const int idx) const { return idx == 0 ? x : y; }
};

#define SHAPED_ELEM(X, Y) X, Y

typedef struct xyze_trajectory {
  float x[FTM_WINDOW_SIZE];
  float y[FTM_WINDOW_SIZE];
  float z[FTM_WINDOW_SIZE];
  float i[FTM_WINDOW_SIZE];
  float j[FTM_WINDOW_SIZE];
  float k[FTM_WINDOW_SIZE];
  float u[FTM_WINDOW_SIZE];
  float v[FTM_WINDOW_SIZE];
  float w[FTM_WINDOW_SIZE];
  float e[FTM_WINDOW_SIZE];
  void reset() {
    for (int idx = 0; idx < FTM_WINDOW_SIZE; ++idx) {
      x[idx] = y[idx] = z[idx] = e[idx] = 0.0f;
      i[idx] = j[idx] = k[idx] = 0.0f;
      u[idx] = v[idx] = w[idx] = 0.0f;
    }
  }
} xyze_trajectory_t;

typedef xyze_trajectory_t xyze_trajectoryMod_t;
#ifndef HAS_X_AXIS
  #define HAS_X_AXIS 1
#endif
#ifndef HAS_Y_AXIS
  #define HAS_Y_AXIS 1
#endif
#ifndef HAS_Z_AXIS
  #define HAS_Z_AXIS 1
#endif
#ifndef AXIS_HAS_SHAPER
  #define AXIS_HAS_SHAPER(A) (ftMotion.cfg.shaper[_AXIS(A)] != ftMotionShaper_NONE)
#endif
#ifndef AXIS_HAS_EISHAPER
  #define AXIS_HAS_EISHAPER(A) (ftMotion.cfg.shaper[_AXIS(A)] == ftMotionShaper_EI || ftMotion.cfg.shaper[_AXIS(A)] == ftMotionShaper_2HEI || ftMotion.cfg.shaper[_AXIS(A)] == ftMotionShaper_3HEI)
#endif
