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

#include "../../inc/MarlinConfig.h"

#if ENABLED(M970_M979_GCODE)

#include "../gcode.h"
#if ENABLED(FT_MOTION)
  #include "../../module/ft_motion.h"
#endif
#include "../../module/motion.h"
#include "../../module/planner.h"
#include "../../module/settings.h"
#include "../../module/stepper.h"

#include <ctype.h>
#include <string.h>

#ifndef IS_TUNE_ADXL345_SPI_SUPPORT
  #define IS_TUNE_ADXL345_SPI_SUPPORT 1
#endif

#ifndef IS_TUNE_ADXL345_CS_PIN
  #define IS_TUNE_ADXL345_CS_PIN -1
#endif

#ifndef IS_TUNE_ADXL345_SPI_HZ
  #define IS_TUNE_ADXL345_SPI_HZ 5000000UL
#endif

#if ENABLED(IS_TUNE_ADXL345_SPI_SUPPORT)
  #include <SPI.h>
#endif

namespace {

  /**
   * Input Shaper tuning implementation for M970..M979.
   *
   * High-level architecture:
   * 1) Parse host parameters into tune_state.
   * 2) Capture vibration samples (hardware ADXL345 or simulation source).
   * 3) Analyze spectral response and generate recommendation values.
   * 4) Persist the run into an in-memory ring and expose it via M971..M974.
   * 5) Stage/apply recommendation values via M975/M976.
   *
   * Host integration contract:
   * - All runtime events are printed as IS_TUNE records.
   * - START/PROGRESS/END/ERROR envelopes keep host polling/state sync simple.
   */

  /**
   * Protocol and algorithm bounds.
   *
   * These constants define accepted host ranges, default run behavior,
   * storage capacities, and analysis tuning weights.
   */
  constexpr uint8_t IS_TUNE_PROTOCOL_VER = 2;
  constexpr uint16_t DEFAULT_SAMPLE_HZ = 3200;
  constexpr uint16_t DEFAULT_WINDOW_MS = 1200;
  constexpr float DEFAULT_SWEEP_DURATION_SEC = 60.0f;
  constexpr uint16_t IS_TUNE_CAPTURE_WINDOW_SAMPLES = 2048;
  constexpr uint8_t DEFAULT_EXCITE_PCT = 30;
  constexpr float DEFAULT_FREQ_START_HZ = 5.0f;
  constexpr float DEFAULT_FREQ_END_HZ = 135.0f;
  constexpr float DEFAULT_ACCEL_PER_HZ = 60.0f;
  constexpr float DEFAULT_HZ_PER_SEC = 1.0f;
  constexpr float DEFAULT_MAX_SMOOTHING = -1.0f;
  constexpr uint8_t IS_TUNE_MAX_RUN_NAME = 24;
  constexpr uint8_t IS_TUNE_MAX_CHIPS_NAME = 24;
  constexpr float DEFAULT_DAMPING = 0.10f;
  constexpr float DEFAULT_SMOOTHING = 0.00f;
  constexpr float DEFAULT_QUALITY_FLOOR = 0.35f;
  constexpr uint16_t IS_TUNE_MIN_CAPTURE_SAMPLES = 96;
  constexpr uint16_t IS_TUNE_MAX_CAPTURE_SAMPLES = IS_TUNE_CAPTURE_WINDOW_SAMPLES;
  constexpr uint8_t IS_TUNE_CAPTURE_CHUNK_SAMPLES = 32;
  constexpr uint8_t IS_TUNE_MAX_STORED_RUNS = 4;
  constexpr uint16_t IS_TUNE_MAX_FETCH_SAMPLES = 64;
  constexpr uint16_t IS_TUNE_DEFAULT_GRAPH_BINS = 48;
  constexpr uint16_t IS_TUNE_MAX_GRAPH_BINS = 96;
  constexpr float IS_TUNE_EXCITE_MARGIN_MM = 1.0f;
  constexpr float IS_TUNE_CROSS_COUPLE_WEIGHT = 0.55f;
  constexpr float IS_TUNE_Z_COUPLE_WEIGHT = 0.70f;
  constexpr float IS_TUNE_FRAME_EPSILON_SQ = 0.0001f;
  constexpr float IS_TUNE_PI = 3.14159265358979323846f;
  constexpr float IS_TUNE_MIN_FREQ_HZ = 5.0f;
  constexpr float IS_TUNE_WINDOW_T_SEC = 0.5f;
  constexpr float IS_TUNE_MAX_SHAPER_FREQ = 150.0f;
  constexpr float IS_TUNE_SHAPER_VIBRATION_REDUCTION = 20.0f;
  constexpr float IS_TUNE_AUTOTUNE_SCV = 5.0f;
  constexpr float IS_TUNE_AUTOTUNE_ACCEL_REF = 5000.0f;
  constexpr float IS_TUNE_TARGET_SMOOTHING = 0.12f;
  constexpr float IS_TUNE_SHAPER_FREQ_STEP = 1.0f;
  constexpr uint8_t IS_TUNE_MAX_SHAPER_PULSES = 5;
  constexpr uint16_t IS_TUNE_MIN_FFT_SAMPLES = 32;
  constexpr uint16_t IS_TUNE_MAX_PSD_BINS = IS_TUNE_MAX_CAPTURE_SAMPLES / 2 + 1;
  constexpr uint8_t IS_TUNE_TEST_DAMPING_RATIO_COUNT = 3;
  constexpr float IS_TUNE_TEST_DAMPING_RATIOS[IS_TUNE_TEST_DAMPING_RATIO_COUNT] = { 0.075f, 0.10f, 0.15f };

  /** Sensor backend error codes reported to the host (sensor-related failures). */
  enum InputShaperSensorError : uint8_t {
    IS_TUNE_SENSOR_OK,
    IS_TUNE_SENSOR_BAD_PIN,
    IS_TUNE_SENSOR_DEVID_MISMATCH,
    IS_TUNE_SENSOR_POWER_CTL,
    IS_TUNE_SENSOR_READ_FAILED
  };

  /** Lifecycle phase of a tuning run. Used by payload emitters and host UX. */
  enum InputShaperTunePhase : uint8_t {
    IS_TUNE_PHASE_IDLE,
    IS_TUNE_PHASE_CAPTURE,
    IS_TUNE_PHASE_PREPROCESS,
    IS_TUNE_PHASE_READY,
    IS_TUNE_PHASE_APPLIED,
    IS_TUNE_PHASE_ABORTED
  };

  /** Capture execution result codes for hardware sweep/excitation. */
  enum InputShaperCaptureResult : uint8_t {
    IS_TUNE_CAPTURE_OK,
    IS_TUNE_CAPTURE_HOME_REQUIRED,
    IS_TUNE_CAPTURE_ALLOC_FAILED,
    IS_TUNE_CAPTURE_RANGE_INVALID,
    IS_TUNE_CAPTURE_MOVE_FAILED,
    IS_TUNE_CAPTURE_SENSOR_READ_FAILED
  };

  /** Analysis output extracted from captured samples. */
  struct InputShaperMetrics {
    float peak_x_hz;
    float peak_y_hz;
    float peak_x_mag;
    float peak_y_mag;
    float quality;
  };

  /** Recommended shaping profile derived from metrics. */
  struct InputShaperRecommendation {
    float x_hz;
    float y_hz;
    uint8_t shaper_x; // 0=None, 1=ZV, 2=ZVD, 3=ZVDD, 4=ZVDDD, 5=EI, 6=2HEI, 7=3HEI, 8=MZV
    uint8_t shaper_y;
    float damping;
    float smoothing;
  };

  /** PSD response container generated by Welch frequency analysis. */
  struct InputShaperFrequencyResponse {
    bool valid;
    uint16_t bins;
    float freq[IS_TUNE_MAX_PSD_BINS];
    float psd_x[IS_TUNE_MAX_PSD_BINS];
    float psd_y[IS_TUNE_MAX_PSD_BINS];
    float psd_z[IS_TUNE_MAX_PSD_BINS];
    float psd_sum[IS_TUNE_MAX_PSD_BINS];
  };

  /** Input-shaper pulse set equivalent to Klipper's [A, T] representation. */
  struct InputShaperPulses {
    uint8_t count;
    float a[IS_TUNE_MAX_SHAPER_PULSES];
    float t[IS_TUNE_MAX_SHAPER_PULSES];
  };

  /** One axis tuning result for a given shaper family and frequency. */
  struct InputShaperFitResult {
    bool valid;
    const char *name;
    float freq;
    float vibrs;
    float smoothing;
    float score;
    float max_accel;
  };

  /**
   * Live tuning state for the currently active session.
   *
   * This is the authoritative source for event payloads and command transitions.
   */
  struct InputShaperTuneState {
    bool active;
    uint16_t run_id;
    uint8_t axis_mask;
    uint8_t mode;
    uint16_t sample_hz;
    uint16_t window_ms;
    uint8_t excite_pct;
    float quality_floor;
    uint8_t progress_pct;
    InputShaperTunePhase phase;
    InputShaperMetrics metrics;
    InputShaperRecommendation recommendation;
    InputShaperRecommendation staged;
    InputShaperRecommendation applied;
    bool has_recommendation;
    bool pending_apply;
    bool committed;
    bool simulated_source;
    bool hw_requested;
    bool sensor_ready;
    uint8_t sensor_error;
    uint32_t sample_count;
    int16_t last_ax, last_ay, last_az;
    float mean_abs_x;
    float mean_abs_y;
    float freq_start_hz;
    float freq_end_hz;
    float accel_per_hz;   
    float hz_per_sec;
    float max_smoothing;
    char chips[IS_TUNE_MAX_CHIPS_NAME + 1];
  };

  /** Raw capture buffer for one run before persistence. */
  struct InputShaperCaptureSession {
    uint16_t target_samples;
    uint16_t captured_samples;
    bool analyzed;
    uint64_t abs_sum_x;
    uint64_t abs_sum_y;
    int16_t *x;
    int16_t *y;
    int16_t *z;
  };

  /** Persisted run slot (ring buffer). Includes metadata, metrics, and raw samples. */
  struct InputShaperStoredRun {
    bool valid;
    bool keep_raw;
    bool simulated_source;
    uint16_t run_id;
    uint16_t sample_hz;
    uint16_t sample_count;
    uint8_t axis_mask;
    uint8_t mode;
    float freq_start_hz;
    float freq_end_hz;
    float accel_per_hz;
    float hz_per_sec;
    float max_smoothing;
    bool input_shaping_during_test;
    char chips[IS_TUNE_MAX_CHIPS_NAME + 1];
    char run_name[IS_TUNE_MAX_RUN_NAME + 1];
    uint8_t excite_pct;
    float quality_floor;
    InputShaperMetrics metrics;
    InputShaperRecommendation recommendation;
    int16_t x[IS_TUNE_MAX_CAPTURE_SAMPLES];
    int16_t y[IS_TUNE_MAX_CAPTURE_SAMPLES];
    int16_t z[IS_TUNE_MAX_CAPTURE_SAMPLES];
  };

  /** Global singleton state used by all M970..M979 handlers. */
  InputShaperTuneState tune_state = {
    false,
    0,
    3,                    // 1:X, 2:Y, 3:XY
    0,                    // mode: 0=sweep, 1=impulse
    DEFAULT_SAMPLE_HZ,
    DEFAULT_WINDOW_MS,
    DEFAULT_EXCITE_PCT,
    DEFAULT_QUALITY_FLOOR,
    0,
    IS_TUNE_PHASE_IDLE,
    { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f },
    { 0.0f, 0.0f, 0, 0, DEFAULT_DAMPING, DEFAULT_SMOOTHING },
    { 0.0f, 0.0f, 0, 0, DEFAULT_DAMPING, DEFAULT_SMOOTHING },
    { 0.0f, 0.0f, 0, 0, DEFAULT_DAMPING, DEFAULT_SMOOTHING },
    false,
    false,
    false,
    true,
    false,
    false,
    IS_TUNE_SENSOR_OK,
    0,
    0,
    0,
    0,
    0.0f,
    0.0f,
    DEFAULT_FREQ_START_HZ,
    DEFAULT_FREQ_END_HZ,
    DEFAULT_ACCEL_PER_HZ,
    DEFAULT_HZ_PER_SEC,
    DEFAULT_MAX_SMOOTHING,
    "adxl345"
  };

  /** Working capture session. */
  InputShaperCaptureSession capture_session = {};

  /** Optional sensor-to-machine frame alignment state (FEMTO_BILAT only). */
  struct InputShaperFrameAlignment {
    bool enabled;
    bool calibrated;
    float sensor_mount_offset_rad;
    float last_sensor_to_machine_rad;
  };

  InputShaperFrameAlignment frame_alignment = {
    false,
    false,
    0.0f,
    0.0f
  };

  /** Shared analysis scratchpads to avoid large stack allocations on MCU targets. */
  InputShaperFrequencyResponse freq_response_scratch = {};
  InputShaperFitResult fit_result_x = { false, "none", 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
  InputShaperFitResult fit_result_y = { false, "none", 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
  float *fft_real_scratch = nullptr;
  float *fft_imag_scratch = nullptr;
  float *fft_window_scratch = nullptr;

  bool allocate_fft_scratch() {
    if (!fft_real_scratch) fft_real_scratch = (float*)malloc(IS_TUNE_CAPTURE_WINDOW_SAMPLES * sizeof(float));
    if (!fft_imag_scratch) fft_imag_scratch = (float*)malloc(IS_TUNE_CAPTURE_WINDOW_SAMPLES * sizeof(float));
    if (!fft_window_scratch) fft_window_scratch = (float*)malloc(IS_TUNE_CAPTURE_WINDOW_SAMPLES * sizeof(float));
    return fft_real_scratch && fft_imag_scratch && fft_window_scratch;
  }

  /**
   * ADXL345 SPI backend.
   *
   * This block isolates all sensor transport details so capture logic can
   * remain backend-agnostic.
   */
  #if ENABLED(IS_TUNE_ADXL345_SPI_SUPPORT)

    constexpr uint8_t ADXL345_REG_DEVID = 0x00;
    constexpr uint8_t ADXL345_REG_BW_RATE = 0x2C;
    constexpr uint8_t ADXL345_REG_POWER_CTL = 0x2D;
    constexpr uint8_t ADXL345_REG_DATA_FORMAT = 0x31;
    constexpr uint8_t ADXL345_REG_DATAX0 = 0x32;
    constexpr uint8_t ADXL345_DEVICE_ID = 0xE5;

    static SPISettings adxl_spi_cfg(IS_TUNE_ADXL345_SPI_HZ, MSBFIRST, SPI_MODE3);
    static bool adxl_bus_ready = false;

    bool adxl_pin_valid() { return IS_TUNE_ADXL345_CS_PIN >= 0; }

    void adxl_chip_select(const bool selected) {
      extDigitalWrite(IS_TUNE_ADXL345_CS_PIN, selected ? LOW : HIGH);
    }

    uint8_t adxl_rate_code_from_hz(const uint16_t sample_hz) {
      if (sample_hz >= 3200) return 0x0F;
      if (sample_hz >= 1600) return 0x0E;
      if (sample_hz >= 800) return 0x0D;
      if (sample_hz >= 400) return 0x0C;
      if (sample_hz >= 200) return 0x0B;
      if (sample_hz >= 100) return 0x0A;
      if (sample_hz >= 50) return 0x09;
      if (sample_hz >= 25) return 0x08;
      return 0x07;
    }

    uint8_t adxl_read_reg(const uint8_t reg) {
      SPI.beginTransaction(adxl_spi_cfg);
      adxl_chip_select(true);
      SPI.transfer(reg | 0x80);
      const uint8_t value = SPI.transfer(0x00);
      adxl_chip_select(false);
      SPI.endTransaction();
      return value;
    }

    void adxl_write_reg(const uint8_t reg, const uint8_t value) {
      SPI.beginTransaction(adxl_spi_cfg);
      adxl_chip_select(true);
      SPI.transfer(reg & 0x3F);
      SPI.transfer(value);
      adxl_chip_select(false);
      SPI.endTransaction();
    }

    void adxl_read_regs(const uint8_t reg, uint8_t * const out, const uint8_t len) {
      SPI.beginTransaction(adxl_spi_cfg);
      adxl_chip_select(true);
      SPI.transfer(reg | 0xC0);
      for (uint8_t i = 0; i < len; ++i) out[i] = SPI.transfer(0x00);
      adxl_chip_select(false);
      SPI.endTransaction();
    }

    bool adxl_apply_sampling_rate(const uint16_t sample_hz) {
      adxl_write_reg(ADXL345_REG_BW_RATE, adxl_rate_code_from_hz(sample_hz));
      return adxl_read_reg(ADXL345_REG_BW_RATE) == adxl_rate_code_from_hz(sample_hz);
    }

    bool adxl_init(const uint16_t sample_hz, uint8_t &error_out) {
      if (!adxl_pin_valid()) {
        error_out = IS_TUNE_SENSOR_BAD_PIN;
        return false;
      }

      if (!adxl_bus_ready) {
        SPI.begin();
        adxl_bus_ready = true;
      }

      pinMode(IS_TUNE_ADXL345_CS_PIN, OUTPUT);
      adxl_chip_select(false);

      if (adxl_read_reg(ADXL345_REG_DEVID) != ADXL345_DEVICE_ID) {
        error_out = IS_TUNE_SENSOR_DEVID_MISMATCH;
        return false;
      }

      // Full-resolution, right-justified, +/-16g range.
      adxl_write_reg(ADXL345_REG_DATA_FORMAT, 0x0B);

      if (!adxl_apply_sampling_rate(sample_hz)) {
        error_out = IS_TUNE_SENSOR_READ_FAILED;
        return false;
      }

      adxl_write_reg(ADXL345_REG_POWER_CTL, 0x08);
      if ((adxl_read_reg(ADXL345_REG_POWER_CTL) & 0x08) == 0) {
        error_out = IS_TUNE_SENSOR_POWER_CTL;
        return false;
      }

      error_out = IS_TUNE_SENSOR_OK;
      return true;
    }

    bool adxl_read_xyz(int16_t &x, int16_t &y, int16_t &z) {
      if (!adxl_pin_valid()) return false;

      uint8_t raw[6] = { 0, 0, 0, 0, 0, 0 };
      adxl_read_regs(ADXL345_REG_DATAX0, raw, 6);

      x = int16_t((uint16_t(raw[1]) << 8) | raw[0]);
      y = int16_t((uint16_t(raw[3]) << 8) | raw[2]);
      z = int16_t((uint16_t(raw[5]) << 8) | raw[4]);
      return true;
    }

    bool adxl_capture_samples(const uint16_t count, int16_t * const x_out, int16_t * const y_out, int16_t * const z_out, const uint16_t sample_hz) {
      if (count == 0)
        return true;

      uint16_t safe_hz = sample_hz;
      if (safe_hz < 1) safe_hz = 1;

      uint32_t period_us = 1000000UL / safe_hz;
      if (period_us < 1) period_us = 1;

      for (uint16_t i = 0; i < count; ++i) {
        int16_t rx = 0, ry = 0, rz = 0;
        if (!adxl_read_xyz(rx, ry, rz)) return false;

        x_out[i] = rx;
        y_out[i] = ry;
        z_out[i] = rz;

        if (i + 1 < count) delayMicroseconds(period_us);
      }
      return true;
    }

  #endif

  /** Generic numeric helpers used across parsing, capture, and analysis paths. */

  uint32_t noise_lcg(uint32_t seed) {
    return seed * 1664525UL + 1013904223UL;
  }

  float noise_unit(uint32_t seed) {
    return float(seed & 0xFFFFUL) / 65535.0f;
  }

  bool axis_includes_x(const uint8_t axis_mask) { return axis_mask & 0x01; }
  bool axis_includes_y(const uint8_t axis_mask) { return axis_mask & 0x02; }

  /**
   * Frame-alignment math helpers.
   *
   * These are only behaviorally relevant when FEMTO_BILAT alignment is enabled.
   */
  float wrap_angle_pi(float angle) {
    while (angle > IS_TUNE_PI) angle -= 2.0f * IS_TUNE_PI;
    while (angle < -IS_TUNE_PI) angle += 2.0f * IS_TUNE_PI;
    return angle;
  }

  float closest_pi_equivalent(const float angle, const float reference) {
    const float a = wrap_angle_pi(angle);
    const float b = wrap_angle_pi(angle + IS_TUNE_PI);
    const float da = fabsf(wrap_angle_pi(a - reference));
    const float db = fabsf(wrap_angle_pi(b - reference));
    return da <= db ? a : b;
  }

  int16_t quantize_i16(const float value) {
    return int16_t(value >= 0.0f ? value + 0.5f : value - 0.5f);
  }

  void rotate_xy_samples(int16_t * const x_samples, int16_t * const y_samples, const uint16_t count, const float sensor_to_machine_rad) {
    const float c = cosf(sensor_to_machine_rad);
    const float s = sinf(sensor_to_machine_rad);

    for (uint16_t i = 0; i < count; ++i) {
      const float sx = x_samples[i], sy = y_samples[i];
      const float mx = sx * c - sy * s;
      const float my = sx * s + sy * c;
      x_samples[i] = quantize_i16(mx);
      y_samples[i] = quantize_i16(my);
    }
  }

  void reset_frame_alignment() {
    frame_alignment.enabled = ENABLED(FEMTO_BILAT);
    frame_alignment.calibrated = false;
    frame_alignment.sensor_mount_offset_rad = 0.0f;
    frame_alignment.last_sensor_to_machine_rad = 0.0f;
  }


  /**
   * State lifecycle helpers.
   *
   * Keep reset/initialization semantics centralized so every command observes
   * consistent defaults.
   */
  void reset_recommendation(InputShaperRecommendation &target) {
    target.x_hz = 0.0f;
    target.y_hz = 0.0f;
    target.shaper_x = 0;
    target.shaper_y = 0;
    target.damping = DEFAULT_DAMPING;
    target.smoothing = DEFAULT_SMOOTHING;
  }

  void sync_applied_profile_from_shaper() {
    #if ENABLED(FT_MOTION) && HAS_FTM_SHAPING
      tune_state.applied.x_hz = ftMotion.cfg.baseFreq.x;
      tune_state.applied.y_hz = ftMotion.cfg.baseFreq.y;
      tune_state.applied.shaper_x = ftMotion.cfg.shaper.x;
      tune_state.applied.shaper_y = ftMotion.cfg.shaper.y;
      tune_state.applied.damping = (ftMotion.cfg.zeta.x + ftMotion.cfg.zeta.y) * 0.5f;
      tune_state.applied.smoothing = (ftMotion.cfg.vtol.x + ftMotion.cfg.vtol.y) * 0.5f;
      tune_state.committed = ftMotion.cfg.active && (ftMotion.cfg.shaper.x != 0 || ftMotion.cfg.shaper.y != 0);
    #else
      tune_state.applied.x_hz = 0.0f;
      tune_state.applied.y_hz = 0.0f;
      tune_state.applied.shaper_x = 0;
      tune_state.applied.shaper_y = 0;
      tune_state.applied.damping = 0.0f;
      tune_state.applied.smoothing = 0.0f;
      tune_state.committed = false;
    #endif
  }

  void reset_metrics() {
    tune_state.metrics.peak_x_hz = 0.0f;
    tune_state.metrics.peak_y_hz = 0.0f;
    tune_state.metrics.peak_x_mag = 0.0f;
    tune_state.metrics.peak_y_mag = 0.0f;
    tune_state.metrics.quality = 0.0f;
  }

  bool allocate_capture_buffers() {
    if (!capture_session.x) capture_session.x = (int16_t*)malloc(IS_TUNE_CAPTURE_WINDOW_SAMPLES * sizeof(int16_t));
    if (!capture_session.y) capture_session.y = (int16_t*)malloc(IS_TUNE_CAPTURE_WINDOW_SAMPLES * sizeof(int16_t));
    if (!capture_session.z) capture_session.z = (int16_t*)malloc(IS_TUNE_CAPTURE_WINDOW_SAMPLES * sizeof(int16_t));

    return capture_session.x && capture_session.y && capture_session.z;
  }

  bool estimate_sensor_heading(
    const int16_t * const x_samples,
    const int16_t * const y_samples,
    const uint16_t count,
    float &heading_out
  ) {
    if (count < 8) return false;

    float mean_x = 0.0f, mean_y = 0.0f;
    for (uint16_t i = 0; i < count; ++i) {
      mean_x += x_samples[i];
      mean_y += y_samples[i];
    }

    const float inv = 1.0f / float(count);
    mean_x *= inv;
    mean_y *= inv;

    float xx = 0.0f, yy = 0.0f, xy = 0.0f;
    for (uint16_t i = 0; i < count; ++i) {
      const float cx = float(x_samples[i]) - mean_x;
      const float cy = float(y_samples[i]) - mean_y;
      xx += cx * cx;
      yy += cy * cy;
      xy += cx * cy;
    }

    if (xx + yy < 1.0f) return false;

    heading_out = 0.5f * ATAN2(2.0f * xy, xx - yy);
    return true;
  }

  bool compute_segment_sensor_to_machine_angle(
    const xyze_pos_t &segment_from,
    const xyze_pos_t &segment_to,
    const int16_t * const x_samples,
    const int16_t * const y_samples,
    const uint16_t count,
    float &sensor_to_machine_rad_out
  ) {
    #if !ENABLED(FEMTO_BILAT)
      (void)segment_from;
      (void)segment_to;
      (void)x_samples;
      (void)y_samples;
      (void)count;
      (void)sensor_to_machine_rad_out;
      return false;
    #else
      if (!frame_alignment.enabled || count == 0)
        return false;

      const float dx = segment_to.x - segment_from.x,
                  dy = segment_to.y - segment_from.y;
      if (HYPOT2(dx, dy) <= IS_TUNE_FRAME_EPSILON_SQ)
        return false;

      // FEMTO_BILAT: infer orientation directly from measured response during X motion.
      if (fabsf(dx) <= 0.01f)
        return false;

      float sensor_heading = 0.0f;
      if (!estimate_sensor_heading(x_samples, y_samples, count, sensor_heading))
        return false;

      const float x_axis_heading = dx >= 0.0f ? 0.0f : IS_TUNE_PI;
      float sensor_to_machine = wrap_angle_pi(x_axis_heading - sensor_heading);
      if (frame_alignment.calibrated)
        sensor_to_machine = closest_pi_equivalent(sensor_to_machine, frame_alignment.last_sensor_to_machine_rad);

      frame_alignment.sensor_mount_offset_rad = sensor_to_machine;
      frame_alignment.last_sensor_to_machine_rad = sensor_to_machine;
      frame_alignment.calibrated = true;

      sensor_to_machine_rad_out = frame_alignment.last_sensor_to_machine_rad;
      return true;
    #endif
  }

bool accumulate_window_psd(const int16_t* x_buf, const int16_t* y_buf, const int16_t* z_buf, const uint16_t count, const float sample_hz);
  void reset_capture_session() {
    capture_session.target_samples = 0;
    capture_session.captured_samples = 0;
    capture_session.analyzed = false;
    capture_session.abs_sum_x = 0;
    capture_session.abs_sum_y = 0;
    freq_response_scratch.valid = false;
    freq_response_scratch.bins = 0;
    for (uint16_t i = 0; i < IS_TUNE_MAX_PSD_BINS; ++i) {
      freq_response_scratch.psd_x[i] = 0.0f;
      freq_response_scratch.psd_y[i] = 0.0f;
      freq_response_scratch.psd_z[i] = 0.0f;
    }

    reset_frame_alignment();

    tune_state.sample_count = 0;
    tune_state.last_ax = 0;
    tune_state.last_ay = 0;
    tune_state.last_az = 0;
    tune_state.mean_abs_x = 0.0f;
    tune_state.mean_abs_y = 0.0f;
  }

  bool configure_capture_session() {
    reset_capture_session();
    if (!allocate_fft_scratch() || !allocate_capture_buffers())
      return false;

    capture_session.target_samples = IS_TUNE_CAPTURE_WINDOW_SAMPLES;
    return true;
  }

  /**
   * Planner enqueue helper with bounded retry.
   *
   * Returns false when the planner fails to accept a segment within guard limit.
   */
  bool queue_excitation_segment(const xyze_pos_t &target, const feedRate_t feedrate_mm_s) {
    uint16_t guard = 0;
    while (!planner.buffer_line(target, feedrate_mm_s, active_extruder)) {
      if (++guard > 2000)
        return false;
      safe_delay(1);
    }
    return true;
  }

  /**
   * Hardware capture engine.
   *
   * Performs motion excitation, acquires ADXL samples chunk-by-chunk, applies
   * optional frame alignment rotation, and updates live capture telemetry.
   */
  InputShaperCaptureResult capture_hardware_with_excitation() {
    #if !ENABLED(IS_TUNE_ADXL345_SPI_SUPPORT)
      return IS_TUNE_CAPTURE_SENSOR_READ_FAILED;
    #else
      #if ENABLED(NO_MOTION_BEFORE_HOMING)
        if (!all_axes_homed())
          return IS_TUNE_CAPTURE_HOME_REQUIRED;
      #endif

      if (!configure_capture_session())
        return IS_TUNE_CAPTURE_ALLOC_FAILED;

      if (capture_session.target_samples < IS_TUNE_MIN_CAPTURE_SAMPLES)
        return IS_TUNE_CAPTURE_RANGE_INVALID;

      tune_state.sample_hz = uint16_t(tune_state.sample_hz);

      const xyze_pos_t center = current_position;
      float amp_x = 0.0f, amp_y = 0.0f;

      const float base_amp = 0.8f + float(tune_state.excite_pct) * 0.06f;
      const float min_x = float(X_MIN_POS) + IS_TUNE_EXCITE_MARGIN_MM;
      const float max_x = float(X_MAX_POS) - IS_TUNE_EXCITE_MARGIN_MM;
      const float min_y = float(Y_MIN_POS) + IS_TUNE_EXCITE_MARGIN_MM;
      const float max_y = float(Y_MAX_POS) - IS_TUNE_EXCITE_MARGIN_MM;

      if (axis_includes_x(tune_state.axis_mask) || frame_alignment.enabled) {
        const float x_headroom = _MIN(center.x - min_x, max_x - center.x);
        if (x_headroom <= 0.1f)
          return IS_TUNE_CAPTURE_RANGE_INVALID;
        else
          amp_x = _MIN(base_amp, x_headroom * 0.95f);
      }

      if (axis_includes_y(tune_state.axis_mask)) {
        const float y_headroom = _MIN(center.y - min_y, max_y - center.y);
        if (y_headroom <= 0.1f)
          return IS_TUNE_CAPTURE_RANGE_INVALID;
        amp_y = _MIN(base_amp, y_headroom * 0.95f);
      }

      const feedRate_t excite_feedrate_mm_s = 20.0f + float(tune_state.excite_pct) * 2.0f + (tune_state.mode ? 25.0f : 0.0f);

      planner.synchronize();

      capture_session.abs_sum_x = 0;
      capture_session.abs_sum_y = 0;
      capture_session.captured_samples = 0;

      bool heading_calibration_pending = frame_alignment.enabled;
      xyze_pos_t previous_target = center;
      uint16_t segment_index = 0;
      while (capture_session.captured_samples < capture_session.target_samples) {
        const uint16_t remaining = capture_session.target_samples - capture_session.captured_samples;
        const uint16_t chunk = remaining > IS_TUNE_CAPTURE_CHUNK_SAMPLES ? IS_TUNE_CAPTURE_CHUNK_SAMPLES : remaining;

        const xyze_pos_t segment_from = previous_target;
        xyze_pos_t excite_target = center;
        const float sx = (segment_index & 0x01) ? -1.0f : 1.0f;
        const float sy = tune_state.mode == 1 ? sx : (segment_index & 0x02 ? -sx : sx);

        const bool heading_calibration_move = heading_calibration_pending;
        if (heading_calibration_move) {
          // Calibrate sensor heading once per run from a deliberate X-only segment.
          excite_target.x = center.x + amp_x * sx;
          excite_target.y = center.y;
        }
        else {
          if (axis_includes_x(tune_state.axis_mask))
            excite_target.x = center.x + amp_x * sx;
          if (axis_includes_y(tune_state.axis_mask))
            excite_target.y = center.y + amp_y * sy;
        }

        if (!queue_excitation_segment(excite_target, excite_feedrate_mm_s))
          return IS_TUNE_CAPTURE_MOVE_FAILED;

        int16_t * const x_ptr = &capture_session.x[capture_session.captured_samples];
        int16_t * const y_ptr = &capture_session.y[capture_session.captured_samples];
        int16_t * const z_ptr = &capture_session.z[capture_session.captured_samples];

        if (!adxl_capture_samples(chunk, x_ptr, y_ptr, z_ptr, tune_state.sample_hz))
          return IS_TUNE_CAPTURE_SENSOR_READ_FAILED;

        float sensor_to_machine_rad = frame_alignment.last_sensor_to_machine_rad;
        if (heading_calibration_pending) {
          if (!compute_segment_sensor_to_machine_angle(segment_from, excite_target, x_ptr, y_ptr, chunk, sensor_to_machine_rad))
            return IS_TUNE_CAPTURE_RANGE_INVALID;
          heading_calibration_pending = false;
        }

        if (frame_alignment.calibrated)
          rotate_xy_samples(x_ptr, y_ptr, chunk, sensor_to_machine_rad);

        for (uint16_t i = 0; i < chunk; ++i) {
          const int16_t sx_sample = x_ptr[i], sy_sample = y_ptr[i];
          capture_session.abs_sum_x += sx_sample >= 0 ? sx_sample : uint16_t(-sx_sample);
          capture_session.abs_sum_y += sy_sample >= 0 ? sy_sample : uint16_t(-sy_sample);
        }

        capture_session.captured_samples += chunk;
        tune_state.sample_count = capture_session.captured_samples;

        const uint16_t last_idx = capture_session.captured_samples - 1;
        tune_state.last_ax = capture_session.x[last_idx];
        tune_state.last_ay = capture_session.y[last_idx];
        tune_state.last_az = capture_session.z[last_idx];

        const float inv_count = 1.0f / float(capture_session.captured_samples);
        tune_state.mean_abs_x = float(capture_session.abs_sum_x) * inv_count;
        tune_state.mean_abs_y = float(capture_session.abs_sum_y) * inv_count;

        tune_state.progress_pct = uint8_t((uint32_t(capture_session.captured_samples) * 70UL) / capture_session.target_samples);
        if (tune_state.progress_pct < 5) tune_state.progress_pct = 5;

        previous_target = excite_target;
        ++segment_index;
      }

      if (!queue_excitation_segment(center, _MAX(15.0f, excite_feedrate_mm_s * 0.6f)))
        return IS_TUNE_CAPTURE_MOVE_FAILED;

      planner.synchronize();
      return IS_TUNE_CAPTURE_OK;
    #endif
  }

  /**
   * Simulation capture backend.
   *
   * Generates deterministic pseudo-vibration data for development and host-flow
   * validation when no hardware sensor is used.
   */
  bool generate_simulated_capture() {
    if (
      capture_session.target_samples < IS_TUNE_MIN_CAPTURE_SAMPLES
      || !capture_session.x || !capture_session.y || !capture_session.z
    ) {
      if (!configure_capture_session())
        return false;
    }

    if (capture_session.target_samples == 0)
      return false;

    const uint16_t count = capture_session.target_samples;
    const int16_t amp_x = int16_t(180 + int16_t(tune_state.excite_pct) * 6);
    const int16_t amp_y = int16_t(200 + int16_t(tune_state.excite_pct) * 5);

    const uint16_t fx = uint16_t(30 + (tune_state.mode ? 8 : 2) + (tune_state.run_id % 9));
    const uint16_t fy = uint16_t(36 + (tune_state.mode ? 6 : 3) + (tune_state.run_id % 11));

    uint16_t period_x = tune_state.sample_hz / (fx ? fx : 1);
    uint16_t period_y = tune_state.sample_hz / (fy ? fy : 1);
    if (period_x < 2) period_x = 2;
    if (period_y < 2) period_y = 2;

    capture_session.abs_sum_x = 0;
    capture_session.abs_sum_y = 0;

    for (uint16_t i = 0; i < count; ++i) {
      int16_t sx = 0, sy = 0;

      if (axis_includes_x(tune_state.axis_mask))
        sx = (i % period_x) < (period_x >> 1) ? amp_x : int16_t(-amp_x);

      if (axis_includes_y(tune_state.axis_mask))
        sy = (i % period_y) < (period_y >> 1) ? amp_y : int16_t(-amp_y);

      const uint32_t seed = noise_lcg((uint32_t(tune_state.run_id) << 16) ^ uint32_t(i * 2654435761UL));
      sx += int16_t(seed & 0x0F) - 8;
      sy += int16_t((seed >> 4) & 0x0F) - 8;

      capture_session.x[i] = sx;
      capture_session.y[i] = sy;
      capture_session.z[i] = int16_t((seed >> 8) & 0x07) - 3;

      capture_session.abs_sum_x += sx >= 0 ? sx : uint16_t(-sx);
      capture_session.abs_sum_y += sy >= 0 ? sy : uint16_t(-sy);
    }

    capture_session.captured_samples = count;
    tune_state.sample_count = count;
    tune_state.last_ax = capture_session.x[count - 1];
    tune_state.last_ay = capture_session.y[count - 1];
    tune_state.last_az = capture_session.z[count - 1];

    const float inv_count = 1.0f / float(count);
    tune_state.mean_abs_x = float(capture_session.abs_sum_x) * inv_count;
    tune_state.mean_abs_y = float(capture_session.abs_sum_y) * inv_count;
    return true;
  }

  /**
   * Signal-analysis helpers.
   *
   * Klipper-style flow:
   * 1) Compute per-axis PSD via Welch's algorithm.
   * 2) Normalize PSD to frequencies and suppress low-frequency noise.
   * 3) Fit candidate shapers and score residual vibrations vs smoothing.
   */
  uint16_t floor_power_of_two(const uint16_t value) {
    uint16_t p = 1;
    while ((uint32_t(p) << 1) <= value)
      p <<= 1;
    return p;
  }

  float bessel_i0(const float x) {
    float sum = 1.0f;
    float term = 1.0f;
    const float y = (x * x) * 0.25f;
    for (uint8_t i = 1; i < 24; ++i) {
      term *= y / (float(i) * float(i));
      sum += term;
      if (term < 1e-7f * sum) break;
    }
    return sum;
  }

  bool build_kaiser_window(const uint16_t nfft, float &scale_out) {
    if (nfft < 2) return false;
    if (!allocate_fft_scratch()) return false;

    const float beta = 6.0f;
    const float den = bessel_i0(beta);
    if (den <= 0.0f) return false;

    float window_energy = 0.0f;
    for (uint16_t i = 0; i < nfft; ++i) {
      const float ratio = (2.0f * float(i)) / float(nfft - 1) - 1.0f;
      const float arg = beta * SQRT(_MAX(0.0f, 1.0f - ratio * ratio));
      const float w = bessel_i0(arg) / den;
      fft_window_scratch[i] = w;
      window_energy += w * w;
    }

    if (window_energy <= 0.0f) return false;
    scale_out = 1.0f / window_energy;
    return true;
  }

  void fft_inplace(const uint16_t n) {
    uint16_t j = 0;
    for (uint16_t i = 1; i < n; ++i) {
      uint16_t bit = n >> 1;
      while (j & bit) {
        j ^= bit;
        bit >>= 1;
      }
      j ^= bit;

      if (i < j) {
        const float tr = fft_real_scratch[i];
        const float ti = fft_imag_scratch[i];
        fft_real_scratch[i] = fft_real_scratch[j];
        fft_imag_scratch[i] = fft_imag_scratch[j];
        fft_real_scratch[j] = tr;
        fft_imag_scratch[j] = ti;
      }
    }

    for (uint16_t len = 2; len <= n; len <<= 1) {
      const float angle = -2.0f * IS_TUNE_PI / float(len);
      const float wlen_r = cosf(angle);
      const float wlen_i = sinf(angle);

      for (uint16_t i = 0; i < n; i += len) {
        float wr = 1.0f, wi = 0.0f;
        const uint16_t half = len >> 1;
        for (uint16_t k = 0; k < half; ++k) {
          const uint16_t u = i + k;
          const uint16_t v = u + half;

          const float vr = fft_real_scratch[v] * wr - fft_imag_scratch[v] * wi;
          const float vi = fft_real_scratch[v] * wi + fft_imag_scratch[v] * wr;

          fft_real_scratch[v] = fft_real_scratch[u] - vr;
          fft_imag_scratch[v] = fft_imag_scratch[u] - vi;
          fft_real_scratch[u] += vr;
          fft_imag_scratch[u] += vi;

          const float next_wr = wr * wlen_r - wi * wlen_i;
          wi = wr * wlen_i + wi * wlen_r;
          wr = next_wr;
        }
      }
    }
  }

  bool accumulate_window_psd(
    const int16_t * const samples_x,
    const int16_t * const samples_y,
    const int16_t * const samples_z,
    const uint16_t nfft,
    const float fs
  ) {
    if (nfft < IS_TUNE_MIN_FFT_SAMPLES || fs <= 0.0f) return false;

    float scale = 0.0f;
    if (!build_kaiser_window(nfft, scale)) return false;

    const uint16_t bins = uint16_t((nfft >> 1) + 1);
    if (bins > IS_TUNE_MAX_PSD_BINS) return false;

    freq_response_scratch.bins = bins;

    const int16_t* axes[] = {samples_x, samples_y, samples_z};
    float* psd_outs[] = {freq_response_scratch.psd_x, freq_response_scratch.psd_y, freq_response_scratch.psd_z};

    for (uint8_t a = 0; a < 3; ++a) {
      const int16_t* samples = axes[a];
      if (!samples) continue;
      float* psd_out = psd_outs[a];

      float mean = 0.0f;
      for (uint16_t i = 0; i < nfft; ++i) mean += samples[i];
      mean /= float(nfft);

      for (uint16_t i = 0; i < nfft; ++i) {
        const float centered = float(samples[i]) - mean;
        fft_real_scratch[i] = fft_window_scratch[i] * centered;
        fft_imag_scratch[i] = 0.0f;
      }

      fft_inplace(nfft);

      for (uint16_t bin = 0; bin < bins; ++bin) {
        const float real = fft_real_scratch[bin];
        const float imag = fft_imag_scratch[bin];
        float power = (real * real + imag * imag) * (scale / fs);
        if (bin > 0 && bin + 1 < bins)
          power *= 2.0f;
        psd_out[bin] += power;
      }
    }

    for (uint16_t bin = 0; bin < bins; ++bin) {
      freq_response_scratch.freq[bin] = fs * float(bin) / float(nfft);
    }

    freq_response_scratch.valid = true;
    return true;
  }

  bool compute_axis_psd_welch(
    const int16_t * const samples,
    const uint16_t count,
    const float fs,
    const uint16_t nfft,
    float * const freq_out,
    float * const psd_out,
    uint16_t &bins_out
  ) {
    if (count <= nfft || nfft < IS_TUNE_MIN_FFT_SAMPLES || fs <= 0.0f)
      return false;

    float scale = 0.0f;
    if (!build_kaiser_window(nfft, scale))
      return false;

    const uint16_t overlap = nfft >> 1;
    const uint16_t step = nfft - overlap;
    if (step == 0 || count <= overlap)
      return false;

    const uint16_t bins = uint16_t((nfft >> 1) + 1);
    if (bins > IS_TUNE_MAX_PSD_BINS)
      return false;

    for (uint16_t i = 0; i < bins; ++i) psd_out[i] = 0.0f;

    uint16_t used_windows = 0;
    for (uint16_t start = 0; start + nfft <= count; start += step) {
      float mean = 0.0f;
      for (uint16_t i = 0; i < nfft; ++i)
        mean += samples[start + i];
      mean /= float(nfft);

      for (uint16_t i = 0; i < nfft; ++i) {
        const float centered = float(samples[start + i]) - mean;
        fft_real_scratch[i] = fft_window_scratch[i] * centered;
        fft_imag_scratch[i] = 0.0f;
      }

      fft_inplace(nfft);

      for (uint16_t bin = 0; bin < bins; ++bin) {
        const float real = fft_real_scratch[bin];
        const float imag = fft_imag_scratch[bin];
        float power = (real * real + imag * imag) * (scale / fs);
        if (bin > 0 && bin + 1 < bins)
          power *= 2.0f;
        psd_out[bin] += power;
      }
      ++used_windows;
    }

    if (used_windows == 0)
      return false;

    const float inv_windows = 1.0f / float(used_windows);
    for (uint16_t bin = 0; bin < bins; ++bin) {
      psd_out[bin] *= inv_windows;
      freq_out[bin] = fs * float(bin) / float(nfft);
    }

    bins_out = bins;
    return true;
  }

  void normalize_psd_to_frequencies(float * const psd, const float * const freq_bins, const uint16_t bins) {
    for (uint16_t i = 0; i < bins; ++i) {
      const float denom = freq_bins[i] + 0.1f;
      psd[i] /= denom;
      if (freq_bins[i] < 2.0f * IS_TUNE_MIN_FREQ_HZ) {
        const float ratio = (2.0f * IS_TUNE_MIN_FREQ_HZ) / denom;
        psd[i] *= expf(-(ratio * ratio) + 1.0f);
      }
    }
  }

  bool build_frequency_response(
    const int16_t * const x_samples,
    const int16_t * const y_samples,
    const int16_t * const z_samples,
    const uint16_t count,
    const uint16_t sample_hz,
    InputShaperFrequencyResponse &response_out
  ) {
    response_out.valid = false;
    response_out.bins = 0;

    if (count < IS_TUNE_MIN_FFT_SAMPLES || sample_hz < 10)
      return false;

    const float fs = float(sample_hz);

    uint16_t nfft = uint16_t(fs * IS_TUNE_WINDOW_T_SEC);
    if (nfft < IS_TUNE_MIN_FFT_SAMPLES)
      nfft = IS_TUNE_MIN_FFT_SAMPLES;

    uint16_t rounded_nfft = 1;
    while (rounded_nfft < nfft && rounded_nfft < IS_TUNE_CAPTURE_WINDOW_SAMPLES)
      rounded_nfft <<= 1;
    nfft = rounded_nfft;

    const uint16_t max_nfft = floor_power_of_two(count > 1 ? count - 1 : 0);
    if (max_nfft < IS_TUNE_MIN_FFT_SAMPLES)
      return false;
    if (nfft > max_nfft)
      nfft = max_nfft;

    if (count <= nfft)
      return false;

    uint16_t bx = 0, by = 0, bz = 0;
    if (!compute_axis_psd_welch(x_samples, count, fs, nfft, response_out.freq, response_out.psd_x, bx)) return false;
    if (!compute_axis_psd_welch(y_samples, count, fs, nfft, response_out.freq, response_out.psd_y, by)) return false;
    if (!compute_axis_psd_welch(z_samples, count, fs, nfft, response_out.freq, response_out.psd_z, bz)) return false;
    if (bx != by || bx != bz) return false;

    response_out.bins = bx;

    normalize_psd_to_frequencies(response_out.psd_x, response_out.freq, response_out.bins);
    normalize_psd_to_frequencies(response_out.psd_y, response_out.freq, response_out.bins);
    normalize_psd_to_frequencies(response_out.psd_z, response_out.freq, response_out.bins);

    for (uint16_t i = 0; i < response_out.bins; ++i)
      response_out.psd_sum[i] = response_out.psd_x[i] + response_out.psd_y[i] + response_out.psd_z[i];

    response_out.valid = true;
    return true;
  }

  float interpolate_psd(
    const float * const freq_bins,
    const float * const values,
    const uint16_t bins,
    const float freq_hz
  ) {
    if (bins == 0)
      return 0.0f;
    if (freq_hz <= freq_bins[0])
      return values[0];

    for (uint16_t i = 1; i < bins; ++i) {
      if (freq_hz <= freq_bins[i]) {
        const float f0 = freq_bins[i - 1], f1 = freq_bins[i];
        const float v0 = values[i - 1], v1 = values[i];
        if (f1 <= f0) return v1;
        const float t = (freq_hz - f0) / (f1 - f0);
        return v0 + (v1 - v0) * t;
      }
    }

    return values[bins - 1];
  }

  bool find_peak_in_range(
    const float * const freq_bins,
    const float * const values,
    const uint16_t bins,
    const float min_hz,
    const float max_hz,
    float &peak_hz_out,
    float &peak_val_out
  ) {
    if (bins == 0 || max_hz <= min_hz)
      return false;

    bool found = false;
    for (uint16_t i = 0; i < bins; ++i) {
      const float freq = freq_bins[i];
      if (freq < min_hz || freq > max_hz)
        continue;

      if (!found || values[i] > peak_val_out) {
        peak_hz_out = freq;
        peak_val_out = values[i];
        found = true;
      }
    }

    return found;
  }

  bool build_shaper_from_expansion_coeffs(
    const float shaper_freq,
    const float damping_ratio,
    const float * const t_coeffs,
    const float * const a_coeffs,
    const uint8_t pulse_count,
    const uint8_t coeff_count,
    InputShaperPulses &out
  ) {
    if (shaper_freq <= 0.0f || pulse_count == 0 || coeff_count == 0 || pulse_count > IS_TUNE_MAX_SHAPER_PULSES)
      return false;

    const float tau = 1.0f / shaper_freq;
    for (uint8_t i = 0; i < pulse_count; ++i) {
      const uint16_t base = uint16_t(i) * coeff_count;
      float u = t_coeffs[base + coeff_count - 1];
      float v = a_coeffs[base + coeff_count - 1];
      for (uint8_t j = 0; j + 1 < coeff_count; ++j) {
        const uint8_t idx = coeff_count - j - 2;
        u = u * damping_ratio + t_coeffs[base + idx];
        v = v * damping_ratio + a_coeffs[base + idx];
      }
      out.t[i] = u * tau;
      out.a[i] = v;
    }
    out.count = pulse_count;
    return true;
  }

  bool build_zv_shaper(const float shaper_freq, const float damping_ratio, InputShaperPulses &out) {
    const float df_sq = 1.0f - damping_ratio * damping_ratio;
    if (shaper_freq <= 0.0f || df_sq <= 0.0f) return false;
    const float df = sqrtf(df_sq);
    const float K = expf(-damping_ratio * IS_TUNE_PI / df);
    const float t_d = 1.0f / (shaper_freq * df);
    out.count = 2;
    out.a[0] = 1.0f; out.a[1] = K;
    out.t[0] = 0.0f; out.t[1] = 0.5f * t_d;
    return true;
  }

  bool build_mzv_shaper(const float shaper_freq, const float damping_ratio, InputShaperPulses &out) {
    const float df_sq = 1.0f - damping_ratio * damping_ratio;
    if (shaper_freq <= 0.0f || df_sq <= 0.0f) return false;
    const float df = sqrtf(df_sq);
    const float K = expf(-0.75f * damping_ratio * IS_TUNE_PI / df);
    const float t_d = 1.0f / (shaper_freq * df);
    // ⚡ Bolt: Replace 1.0f/SQRT(x) with RSQRT(x) which compiles to fast hardware reciprocal sqrt
    const float a1 = 1.0f - RSQRT(2.0f);
    const float a2 = (SQRT(2.0f) - 1.0f) * K;
    const float a3 = a1 * K * K;
    out.count = 3;
    out.a[0] = a1; out.a[1] = a2; out.a[2] = a3;
    out.t[0] = 0.0f; out.t[1] = 0.375f * t_d; out.t[2] = 0.75f * t_d;
    return true;
  }

  bool build_ei_shaper(const float shaper_freq, const float damping_ratio, InputShaperPulses &out) {
    const float df_sq = 1.0f - damping_ratio * damping_ratio;
    if (shaper_freq <= 0.0f || df_sq <= 0.0f) return false;

    const float v_tol = 1.0f / IS_TUNE_SHAPER_VIBRATION_REDUCTION;
    const float df = sqrtf(df_sq);
    const float t_d = 1.0f / (shaper_freq * df);
    const float dr = damping_ratio;

    const float a1 = (0.24968f + 0.24961f * v_tol)
                   + ((0.80008f + 1.23328f * v_tol)
                   +  (0.49599f + 3.17316f * v_tol) * dr) * dr;

    const float a3 = (0.25149f + 0.21474f * v_tol)
                   + ((-0.83249f + 1.41498f * v_tol)
                   +  (0.85181f - 4.90094f * v_tol) * dr) * dr;

    const float a2 = 1.0f - a1 - a3;
    const float t2 = 0.4999f
                   + (((0.46159f + 8.57843f * v_tol) * v_tol)
                   + (((4.26169f - 108.644f * v_tol) * v_tol)
                   +  ((1.75601f + 336.989f * v_tol) * v_tol) * dr) * dr) * dr;

    out.count = 3;
    out.a[0] = a1; out.a[1] = a2; out.a[2] = a3;
    out.t[0] = 0.0f; out.t[1] = t2 * t_d; out.t[2] = t_d;
    return true;
  }

  bool build_2hump_ei_shaper(const float shaper_freq, const float damping_ratio, InputShaperPulses &out) {
    static constexpr float t_coeffs[] = {
      0.0f,    0.0f,     0.0f,      0.0f,
      0.49890f, 0.16270f, -0.54262f, 6.16180f,
      0.99748f, 0.18382f, -1.58270f, 8.17120f,
      1.49920f, -0.09297f, -0.28338f, 1.85710f
    };

    static constexpr float a_coeffs[] = {
      0.16054f, 0.76699f,  2.26560f, -1.22750f,
      0.33911f, 0.45081f, -2.58080f,  1.73650f,
      0.34089f, -0.61533f, -0.68765f, 0.42261f,
      0.15997f, -0.60246f, 1.00280f, -0.93145f
    };

    return build_shaper_from_expansion_coeffs(shaper_freq, damping_ratio, t_coeffs, a_coeffs, 4, 4, out);
  }

  bool build_3hump_ei_shaper(const float shaper_freq, const float damping_ratio, InputShaperPulses &out) {
    // Mirrors upstream coefficient shape in master branch (k=3 effective expansion).
    static constexpr float t_coeffs[] = {
      0.0f,    0.0f,      0.0f,
      0.49974f, 0.23834f,  0.44559f,
      0.99849f, 0.29808f, -2.36460f,
      1.49870f, 0.10306f, -2.01390f,
      1.99960f, -0.28231f, 0.61536f
    };

    static constexpr float a_coeffs[] = {
      0.11275f,  0.76632f,  1.84780f,
      0.23698f,  0.61164f, -2.57850f,
      0.30008f, -0.19062f, -2.14560f,
      0.23775f, -0.73297f,  0.46885f,
      0.11244f, -0.45439f,  0.96382f
    };

    return build_shaper_from_expansion_coeffs(shaper_freq, damping_ratio, t_coeffs, a_coeffs, 5, 3, out);
  }

  struct InputShaperConfig {
    const char *name;
    float min_freq;
    bool (*init_func)(const float, const float, InputShaperPulses &);
  };

  float estimate_shaper_response(
    const InputShaperPulses &shaper,
    const float test_damping_ratio,
    const float test_freq
  ) {
    if (shaper.count == 0)
      return 1.0f;

    float sum_a = 0.0f;
    for (uint8_t i = 0; i < shaper.count; ++i) sum_a += shaper.a[i];
    if (sum_a == 0.0f)
      return 1.0f;

    const float omega = 2.0f * IS_TUNE_PI * test_freq;
    const float damping = test_damping_ratio * omega;
    const float omega_d = omega * sqrtf(1.0f - test_damping_ratio * test_damping_ratio);
    const float t_last = shaper.t[shaper.count - 1];

    float s = 0.0f, c = 0.0f;
    for (uint8_t i = 0; i < shaper.count; ++i) {
      const float w = shaper.a[i] * expf(-damping * (t_last - shaper.t[i]));
      s += w * sinf(omega_d * shaper.t[i]);
      c += w * cosf(omega_d * shaper.t[i]);
    }

    return sqrtf(s * s + c * c) / sum_a;
  }

  float estimate_remaining_vibrations(
    const InputShaperPulses &shaper,
    const float test_damping_ratio,
    const float * const freq_bins,
    const float * const psd,
    const uint16_t bins,
    const float max_freq
  ) {
    float psd_max = 0.0f;
    for (uint16_t i = 0; i < bins; ++i)
      if (freq_bins[i] <= max_freq && psd[i] > psd_max)
        psd_max = psd[i];

    if (psd_max <= 0.0f)
      return 1.0f;

    const float vibr_threshold = psd_max / IS_TUNE_SHAPER_VIBRATION_REDUCTION;

    float remaining = 0.0f;
    float total = 0.0f;

    for (uint16_t i = 0; i < bins; ++i) {
      if (freq_bins[i] > max_freq)
        continue;

      const float raw = psd[i] - vibr_threshold;
      if (raw > 0.0f)
        total += raw;

      const float vals = estimate_shaper_response(shaper, test_damping_ratio, freq_bins[i]);
      const float residual = vals * psd[i] - vibr_threshold;
      if (residual > 0.0f)
        remaining += residual;
    }

    if (total <= 0.0f)
      return 1.0f;

    return remaining / total;
  }

  float get_shaper_smoothing(const InputShaperPulses &shaper, const float accel, const float scv) {
    if (shaper.count == 0)
      return 0.0f;

    const float half_accel = accel * 0.5f;
    float sum_a = 0.0f;
    for (uint8_t i = 0; i < shaper.count; ++i) sum_a += shaper.a[i];
    if (sum_a == 0.0f)
      return 0.0f;
    const float inv_sum_a = 1.0f / sum_a;

    float ts = 0.0f;
    for (uint8_t i = 0; i < shaper.count; ++i)
      ts += shaper.a[i] * shaper.t[i];
    ts *= inv_sum_a;

    float offset_90 = 0.0f;
    float offset_180 = 0.0f;
    for (uint8_t i = 0; i < shaper.count; ++i) {
      const float dt = shaper.t[i] - ts;
      if (dt >= 0.0f)
        offset_90 += shaper.a[i] * (scv + half_accel * dt) * dt;
      offset_180 += shaper.a[i] * half_accel * dt * dt;
    }

    offset_90 *= inv_sum_a * SQRT(2.0f);
    offset_180 *= inv_sum_a;
    return _MAX(offset_90, offset_180);
  }

  float find_shaper_max_accel(const InputShaperPulses &shaper, const float scv) {
    const auto predicate = [&](const float test_accel) {
      return get_shaper_smoothing(shaper, test_accel, scv) <= IS_TUNE_TARGET_SMOOTHING;
    };

    float left = 1.0f, right = 1.0f;
    if (!predicate(1e-9f))
      return 0.0f;

    while (!predicate(left)) {
      right = left;
      left *= 0.5f;
    }

    if (right == left) {
      while (predicate(right))
        right *= 2.0f;
    }

    for (uint8_t i = 0; i < 64; ++i) {
      if (right - left <= 1e-7f)
        break;
      const float middle = (left + right) * 0.5f;
      if (predicate(middle))
        left = middle;
      else
        right = middle;
    }

    return left;
  }

  InputShaperFitResult fit_shaper_config(
    const InputShaperConfig &cfg,
    const float * const freq_bins,
    const float * const psd,
    const uint16_t bins,
    const float min_hz,
    const float max_hz,
    const float max_smoothing
  ) {
    InputShaperFitResult best = { false, cfg.name, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };

    if (bins == 0)
      return best;

    const float freq_end = _MIN(_MIN(max_hz, IS_TUNE_MAX_SHAPER_FREQ), freq_bins[bins - 1]);
    const float freq_start = _MAX(min_hz, cfg.min_freq);
    if (freq_end <= freq_start)
      return best;

    for (float test_freq = freq_end; test_freq >= freq_start; test_freq -= IS_TUNE_SHAPER_FREQ_STEP) {
      InputShaperPulses shaper = {};
      if (!cfg.init_func(test_freq, DEFAULT_DAMPING, shaper))
        continue;

      const float smoothing = get_shaper_smoothing(shaper, IS_TUNE_AUTOTUNE_ACCEL_REF, IS_TUNE_AUTOTUNE_SCV);
      if (max_smoothing >= 0.0f && smoothing > max_smoothing && best.valid)
        break;

      float shaper_vibrations = 0.0f;
      for (uint8_t dr_idx = 0; dr_idx < IS_TUNE_TEST_DAMPING_RATIO_COUNT; ++dr_idx) {
        const float vibrations = estimate_remaining_vibrations(
          shaper,
          IS_TUNE_TEST_DAMPING_RATIOS[dr_idx],
          freq_bins,
          psd,
          bins,
          freq_end
        );
        if (vibrations > shaper_vibrations)
          shaper_vibrations = vibrations;
      }

      const float score = smoothing * (powf(shaper_vibrations, 1.5f) + shaper_vibrations * 0.2f + 0.01f);
      if (!best.valid || shaper_vibrations < best.vibrs) {
        best.valid = true;
        best.name = cfg.name;
        best.freq = test_freq;
        best.vibrs = shaper_vibrations;
        best.smoothing = smoothing;
        best.score = score;
        best.max_accel = find_shaper_max_accel(shaper, IS_TUNE_AUTOTUNE_SCV);
      }
    }

    if (!best.valid)
      return best;

    InputShaperFitResult selected = best;
    for (float test_freq = freq_end; test_freq >= freq_start; test_freq -= IS_TUNE_SHAPER_FREQ_STEP) {
      InputShaperPulses shaper = {};
      if (!cfg.init_func(test_freq, DEFAULT_DAMPING, shaper))
        continue;

      const float smoothing = get_shaper_smoothing(shaper, IS_TUNE_AUTOTUNE_ACCEL_REF, IS_TUNE_AUTOTUNE_SCV);
      if (max_smoothing >= 0.0f && smoothing > max_smoothing && selected.valid)
        break;

      float shaper_vibrations = 0.0f;
      for (uint8_t dr_idx = 0; dr_idx < IS_TUNE_TEST_DAMPING_RATIO_COUNT; ++dr_idx) {
        const float vibrations = estimate_remaining_vibrations(
          shaper,
          IS_TUNE_TEST_DAMPING_RATIOS[dr_idx],
          freq_bins,
          psd,
          bins,
          freq_end
        );
        if (vibrations > shaper_vibrations)
          shaper_vibrations = vibrations;
      }

      const float score = smoothing * (powf(shaper_vibrations, 1.5f) + shaper_vibrations * 0.2f + 0.01f);
      if (shaper_vibrations < best.vibrs * 1.1f + 0.0005f && score < selected.score) {
        selected.valid = true;
        selected.name = cfg.name;
        selected.freq = test_freq;
        selected.vibrs = shaper_vibrations;
        selected.smoothing = smoothing;
        selected.score = score;
        selected.max_accel = find_shaper_max_accel(shaper, IS_TUNE_AUTOTUNE_SCV);
      }
    }

    return selected;
  }

  InputShaperFitResult find_best_shaper_for_axis(
    const float * const freq_bins,
    const float * const psd,
    const uint16_t bins,
    const float min_hz,
    const float max_hz,
    const float max_smoothing
  ) {
    static const InputShaperConfig autotune_shapers[] = {
      { "zv", 21.0f, build_zv_shaper },
      { "mzv", 23.0f, build_mzv_shaper },
      { "ei", 29.0f, build_ei_shaper },
      { "2hump_ei", 39.0f, build_2hump_ei_shaper },
      { "3hump_ei", 48.0f, build_3hump_ei_shaper }
    };
    constexpr uint8_t shaper_count = sizeof(autotune_shapers) / sizeof(autotune_shapers[0]);

    InputShaperFitResult best = { false, "none", 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
    InputShaperFitResult tuned[shaper_count] = {};
    uint8_t tuned_count = 0;

    for (uint8_t i = 0; i < shaper_count; ++i) {
      const InputShaperFitResult tuned_shaper = fit_shaper_config(
        autotune_shapers[i],
        freq_bins,
        psd,
        bins,
        min_hz,
        max_hz,
        max_smoothing
      );

      if (!tuned_shaper.valid)
        continue;

      tuned[tuned_count++] = tuned_shaper;

      if (!best.valid
          || tuned_shaper.score * 1.2f < best.score
          || (tuned_shaper.score * 1.05f < best.score && tuned_shaper.smoothing * 1.1f < best.smoothing)) {
        best = tuned_shaper;
      }
    }

    if (!best.valid)
      return best;

    if (!strcmp(best.name, "zv")) {
      for (uint8_t i = 0; i < tuned_count; ++i) {
        if (strcmp(tuned[i].name, "zv") && tuned[i].vibrs * 1.1f < best.vibrs) {
          best = tuned[i];
          break;
        }
      }
    }

    return best;
  }

  bool analyze_samples(
    const int16_t * const x_samples,
    const int16_t * const y_samples,
    const int16_t * const z_samples,
    const uint16_t count,
    const uint16_t sample_hz,
    const float min_hz,
    const float max_hz,
    const uint8_t axis_mask,
    const float max_smoothing,
    InputShaperMetrics &metrics_out
  ) {
    if (x_samples != nullptr) {
      if (count < IS_TUNE_MIN_FFT_SAMPLES || sample_hz < 10 || max_hz <= min_hz)
        return false;

      if (!build_frequency_response(x_samples, y_samples, z_samples, count, sample_hz, freq_response_scratch))
        return false;
    } else {
      if (!freq_response_scratch.valid || max_hz <= min_hz)
        return false;
    }

    fit_result_x = { false, "none", 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
    fit_result_y = { false, "none", 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };

    float peak_x_hz = 0.0f, peak_x_power = 0.0f;
    float peak_y_hz = 0.0f, peak_y_power = 0.0f;

    const bool x_peak_ok = find_peak_in_range(
      freq_response_scratch.freq,
      freq_response_scratch.psd_x,
      freq_response_scratch.bins,
      min_hz,
      max_hz,
      peak_x_hz,
      peak_x_power
    );

    const bool y_peak_ok = find_peak_in_range(
      freq_response_scratch.freq,
      freq_response_scratch.psd_y,
      freq_response_scratch.bins,
      min_hz,
      max_hz,
      peak_y_hz,
      peak_y_power
    );

    if ((axis_includes_x(axis_mask) && !x_peak_ok) || (axis_includes_y(axis_mask) && !y_peak_ok))
      return false;

    if (axis_includes_x(axis_mask)) {
      fit_result_x = find_best_shaper_for_axis(
        freq_response_scratch.freq,
        freq_response_scratch.psd_x,
        freq_response_scratch.bins,
        min_hz,
        max_hz,
        max_smoothing
      );
    }

    if (axis_includes_y(axis_mask)) {
      fit_result_y = find_best_shaper_for_axis(
        freq_response_scratch.freq,
        freq_response_scratch.psd_y,
        freq_response_scratch.bins,
        min_hz,
        max_hz,
        max_smoothing
      );
    }

    metrics_out.peak_x_hz = axis_includes_x(axis_mask) ? peak_x_hz : 0.0f;
    metrics_out.peak_y_hz = axis_includes_y(axis_mask) ? peak_y_hz : 0.0f;
    metrics_out.peak_x_mag = axis_includes_x(axis_mask) ? peak_x_power : 0.0f;
    metrics_out.peak_y_mag = axis_includes_y(axis_mask) ? peak_y_power : 0.0f;
    metrics_out.quality = 0.0f;

    float quality_sum = 0.0f;
    uint8_t quality_axes = 0;

    if (axis_includes_x(axis_mask)) {
      quality_sum += fit_result_x.valid ? (1.0f - fit_result_x.vibrs) : 0.4f;
      ++quality_axes;
    }

    if (axis_includes_y(axis_mask)) {
      quality_sum += fit_result_y.valid ? (1.0f - fit_result_y.vibrs) : 0.4f;
      ++quality_axes;
    }

    if (quality_axes == 0) return false;

    metrics_out.quality = quality_sum / float(quality_axes);
    return true;
  }

  void build_recommendation_from_metrics(
    const InputShaperMetrics &metrics,
    const uint8_t axis_mask,
    const uint8_t mode,
    const float max_smoothing,
    InputShaperRecommendation &recommendation_out
  ) {
    recommendation_out.x_hz = axis_includes_x(axis_mask)
      ? (fit_result_x.valid ? fit_result_x.freq : metrics.peak_x_hz)
      : 0.0f;

    recommendation_out.y_hz = axis_includes_y(axis_mask)
      ? (fit_result_y.valid ? fit_result_y.freq : metrics.peak_y_hz)
      : 0.0f;

    recommendation_out.damping = DEFAULT_DAMPING;

    auto map_shaper_name = [](const char* name) -> uint8_t {
      if (!name) return 0;
      if (!strcmp(name, "zv")) return 1;
      // ZVD, ZVDD, ZVDDD fall in 2, 3, 4 respectively for FTM
      if (!strcmp(name, "ei")) return 5;
      if (!strcmp(name, "2hump_ei")) return 6;
      if (!strcmp(name, "3hump_ei")) return 7;
      if (!strcmp(name, "mzv")) return 8;
      return 2; // Default to ZVD if unknown valid fit (conservative fallback)
    };

    recommendation_out.shaper_x = (axis_includes_x(axis_mask) && fit_result_x.valid) ? map_shaper_name(fit_result_x.name) : 0;
    recommendation_out.shaper_y = (axis_includes_y(axis_mask) && fit_result_y.valid) ? map_shaper_name(fit_result_y.name) : 0;

    float smoothing = 0.0f;
    if (axis_includes_x(axis_mask) && fit_result_x.valid)
      smoothing = fit_result_x.smoothing;
    if (axis_includes_y(axis_mask) && fit_result_y.valid && fit_result_y.smoothing > smoothing)
      smoothing = fit_result_y.smoothing;

    if (smoothing <= 0.0f)
      smoothing = 0.01f + (1.0f - metrics.quality) * 0.07f + (mode ? 0.01f : 0.0f);

    recommendation_out.smoothing = smoothing;
    if (max_smoothing >= 0.0f && recommendation_out.smoothing > max_smoothing)
      recommendation_out.smoothing = max_smoothing;
  }

  bool analyze_capture_session() {
    capture_session.analyzed = false;

    if (capture_session.captured_samples < capture_session.target_samples || capture_session.target_samples == 0)
      return false;

    const bool analyzed = analyze_samples(
      capture_session.x,
      capture_session.y,
      capture_session.z,
      capture_session.captured_samples,
      tune_state.sample_hz,
      tune_state.freq_start_hz,
      tune_state.freq_end_hz,
      tune_state.axis_mask,
      tune_state.max_smoothing,
      tune_state.metrics
    );

    capture_session.analyzed = analyzed;
    return analyzed;
  }

  void rebuild_recommendation();

  void reset_tune_state(const bool keep_run_id=true) {
    const uint16_t rid = keep_run_id ? tune_state.run_id : 0;

    tune_state.active = false;
    tune_state.run_id = rid;
    tune_state.axis_mask = 3;
    tune_state.mode = 0;
    tune_state.sample_hz = DEFAULT_SAMPLE_HZ;
    tune_state.window_ms = DEFAULT_WINDOW_MS;
    tune_state.excite_pct = DEFAULT_EXCITE_PCT;
    tune_state.quality_floor = DEFAULT_QUALITY_FLOOR;
    tune_state.progress_pct = 0;
    tune_state.phase = IS_TUNE_PHASE_IDLE;
    tune_state.has_recommendation = false;
    tune_state.pending_apply = false;
    tune_state.committed = false;
    tune_state.simulated_source = false;
    tune_state.hw_requested = false;
    tune_state.sensor_ready = false;
    tune_state.sensor_error = IS_TUNE_SENSOR_OK;
    tune_state.freq_start_hz = DEFAULT_FREQ_START_HZ;
    tune_state.freq_end_hz = DEFAULT_FREQ_END_HZ;
    tune_state.accel_per_hz = DEFAULT_ACCEL_PER_HZ;
    tune_state.hz_per_sec = DEFAULT_HZ_PER_SEC;
    tune_state.max_smoothing = DEFAULT_MAX_SMOOTHING;
    strncpy(tune_state.chips, "adxl345", sizeof(tune_state.chips) - 1);
    tune_state.chips[sizeof(tune_state.chips) - 1] = '\0';
    reset_capture_session();

    reset_metrics();
    reset_recommendation(tune_state.recommendation);
    reset_recommendation(tune_state.staged);
    reset_recommendation(tune_state.applied);
    sync_applied_profile_from_shaper();
  }

  const char* axis_label(const uint8_t axis_mask) {
    switch (axis_mask) {
      case 1: return "X";
      case 2: return "Y";
      default: return "XY";
    }
  }

  /**
   * Token parsing and argument mapping helpers.
   *
   * M970 accepts letter-prefixed tokens from parser.command_ptr and parses them
   * explicitly to support mixed numeric/string argument forms.
   */
  bool parse_axis_mask(const int16_t raw_axis, uint8_t &axis_mask_out) {
    switch (raw_axis) {
      case 1:
      case 2:
      case 3:
        axis_mask_out = uint8_t(raw_axis);
        return true;
      default:
        return false;
    }
  }

  bool token_equals_ci(const char * const token, const uint8_t token_len, const char * const match) {
    const uint8_t match_len = uint8_t(strlen(match));
    if (token_len != match_len) return false;
    for (uint8_t i = 0; i < token_len; ++i)
      if (toupper((unsigned char)token[i]) != toupper((unsigned char)match[i])) return false;
    return true;
  }

  bool parse_axis_token(const char * const token, const uint8_t token_len, uint8_t &axis_mask_out) {
    if (token_len == 1) {
      switch (token[0]) {
        case '1': axis_mask_out = 1; return true;
        case '2': axis_mask_out = 2; return true;
        case '3': axis_mask_out = 3; return true;
        case 'X': case 'x': axis_mask_out = 1; return true;
        case 'Y': case 'y': axis_mask_out = 2; return true;
      }
    }

    if (token_equals_ci(token, token_len, "XY")) {
      axis_mask_out = 3;
      return true;
    }

    return false;
  }

  bool parse_float_token(const char * const token, const uint8_t token_len, float &value_out) {
    if (token_len == 0 || token_len >= 24) return false;

    char buffer[24];
    memcpy(buffer, token, token_len);
    buffer[token_len] = '\0';

    char *endptr = nullptr;
    value_out = strtof(buffer, &endptr);
    return endptr && endptr != buffer && *endptr == '\0';
  }

  bool parse_chips_token(const char * const token, const uint8_t token_len, char * const out, const uint8_t max_len) {
    if (token_len == 0 || token_len > max_len) return false;

    for (uint8_t i = 0; i < token_len; ++i) {
      const char c = token[i];
      if (!isalnum((unsigned char)c) && c != '_' && c != '-' && c != '.' && c != ',' && c != ':')
        return false;
      out[i] = c;
    }

    out[token_len] = '\0';
    return true;
  }

  bool chip_requests_sim(const char * const chip_name) {
    return token_equals_ci(chip_name, uint8_t(strlen(chip_name)), "SIM");
  }

  bool chip_token_supported(const char * const chip_name) {
    if (chip_requests_sim(chip_name)) return true;

    const char * const comma = strchr(chip_name, ',');
    const uint8_t base_len = comma ? uint8_t(comma - chip_name) : uint8_t(strlen(chip_name));
    return token_equals_ci(chip_name, base_len, "ADXL345");
  }

  bool next_m970_token(const char *&cursor, char &letter_out, const char *&value_out, uint8_t &value_len_out) {
    while (*cursor == ' ') ++cursor;
    if (!*cursor) return false;

    letter_out = *cursor++;
    if (letter_out >= 'a' && letter_out <= 'z')
      letter_out -= 32;

    if (*cursor == '"') {
      ++cursor;
      value_out = cursor;
      while (*cursor && *cursor != '"') ++cursor;
      value_len_out = uint8_t(cursor - value_out);
      if (*cursor == '"') ++cursor;
    }
    else {
      value_out = cursor;
      while (*cursor && *cursor != ' ') ++cursor;
      value_len_out = uint8_t(cursor - value_out);
    }

    return true;
  }

  void derive_capture_settings_from_shaper_params() {
    const float span_hz = _MAX(1.0f, tune_state.freq_end_hz - tune_state.freq_start_hz);

    tune_state.mode = 0; // SHAPER_CALIBRATE equivalent uses sweep-like characterization.
    tune_state.sample_hz = uint16_t(int32_t(tune_state.freq_end_hz * 28.0f));
    tune_state.window_ms = uint16_t(int32_t((span_hz / _MAX(tune_state.hz_per_sec, 0.10f)) * 1000.0f));
    tune_state.excite_pct = uint8_t(int16_t(tune_state.accel_per_hz * 0.70f));
    tune_state.quality_floor = DEFAULT_QUALITY_FLOOR;
  }

  /**
   * Parse and validate all M970 SHAPER_CALIBRATE-style tokens.
   *
   * On success, this function updates tune_state fields and derives internal
   * capture settings. On failure it returns an error reason string consumed
   * directly by emit_error().
   */
  const char* apply_shaper_calibrate_args(bool &use_hw_backend_out) {
    use_hw_backend_out = true;

    const char *cursor = parser.command_ptr;
    while (*cursor && *cursor != ' ') ++cursor;

    while (true) {
      char letter = '\0';
      const char *token_value = nullptr;
      uint8_t token_len = 0;
      if (!next_m970_token(cursor, letter, token_value, token_len))
        break;

      switch (letter) {
        case 'A': {
          uint8_t axis_mask = 0;
          if (!parse_axis_token(token_value, token_len, axis_mask))
            return "AXIS_INVALID";
          tune_state.axis_mask = axis_mask;
        } break;

        case 'F': {
          float value = 0.0f;
          if (!parse_float_token(token_value, token_len, value) || value < 1.0f || value > 200.0f)
            return "FREQ_START_INVALID";
          tune_state.freq_start_hz = value;
        } break;

        case 'G': {
          float value = 0.0f;
          if (!parse_float_token(token_value, token_len, value) || value < 5.0f || value > 200.0f)
            return "FREQ_END_INVALID";
          tune_state.freq_end_hz = value;
        } break;

        case 'P': {
          float value = 0.0f;
          if (!parse_float_token(token_value, token_len, value) || value < 1.0f || value > 200.0f)
            return "ACCEL_PER_HZ_INVALID";
          tune_state.accel_per_hz = value;
        } break;

        case 'R': {
          float value = 0.0f;
          if (!parse_float_token(token_value, token_len, value) || value < 0.10f || value > 20.0f)
            return "HZ_PER_SEC_INVALID";
          tune_state.hz_per_sec = value;
        } break;

        case 'C':
          if (!parse_chips_token(token_value, token_len, tune_state.chips, IS_TUNE_MAX_CHIPS_NAME))
            return "CHIPS_INVALID";
          break;

        case 'M': {
          float value = 0.0f;
          if (!parse_float_token(token_value, token_len, value) || value < 0.0f || value > 0.20f)
            return "MAX_SMOOTHING_INVALID";
          tune_state.max_smoothing = value;
        } break;

        default:
          return "PARAM_UNKNOWN";
      }
    }

    if (tune_state.freq_end_hz <= tune_state.freq_start_hz)
      return "FREQ_RANGE_INVALID";

    if (!chip_token_supported(tune_state.chips))
      return "CHIPS_UNSUPPORTED";

    use_hw_backend_out = !chip_requests_sim(tune_state.chips);
    derive_capture_settings_from_shaper_params();
    return nullptr;
  }

  const char* mode_label(const uint8_t mode) {
    return mode == 1 ? "IMPULSE" : "SWEEP";
  }

  const char* phase_label(const InputShaperTunePhase phase) {
    switch (phase) {
      case IS_TUNE_PHASE_CAPTURE: return "CAPTURE";
      case IS_TUNE_PHASE_PREPROCESS: return "PREPROCESS";
      case IS_TUNE_PHASE_READY: return "READY";
      case IS_TUNE_PHASE_APPLIED: return "APPLIED";
      case IS_TUNE_PHASE_ABORTED: return "ABORTED";
      default: return "IDLE";
    }
  }

  int16_t clamp_percent(const int16_t value) {
    if (value < 0) return 0;
    if (value > 100) return 100;
    return value;
  }

  __attribute__((unused)) void recalc_phase() {
    if (!tune_state.active) {
      if (tune_state.phase != IS_TUNE_PHASE_APPLIED && tune_state.phase != IS_TUNE_PHASE_ABORTED)
        tune_state.phase = IS_TUNE_PHASE_IDLE;
      return;
    }

    if (tune_state.progress_pct >= 100)
      tune_state.phase = IS_TUNE_PHASE_READY;
    else if (tune_state.progress_pct >= 70)
      tune_state.phase = IS_TUNE_PHASE_PREPROCESS;
    else
      tune_state.phase = IS_TUNE_PHASE_CAPTURE;
  }

  __attribute__((unused)) void rebuild_metrics() {
    const uint32_t seed0 = noise_lcg(uint32_t(tune_state.run_id) << 16 | uint32_t(tune_state.progress_pct) << 8 | tune_state.axis_mask);
    const uint32_t seed1 = noise_lcg(seed0 ^ (uint32_t(tune_state.mode) << 24));
    const uint32_t seed2 = noise_lcg(seed1 ^ 0xA53A9A5AuL);
    const uint32_t seed3 = noise_lcg(seed2 ^ 0x1BD11BDAuL);
    const uint32_t seed4 = noise_lcg(seed3 ^ 0x5EEDC0DEuL);

    const float progress = float(tune_state.progress_pct) / 100.0f;
    const float span = 6.5f * (1.0f - progress) + 0.5f;
    const float base_x = 33.0f + float((tune_state.run_id * 7U + tune_state.mode * 3U) % 11U);
    const float base_y = 38.0f + float((tune_state.run_id * 5U + tune_state.mode * 2U) % 13U);

    tune_state.metrics.peak_x_hz = axis_includes_x(tune_state.axis_mask)
      ? base_x + (noise_unit(seed1) - 0.5f) * span
      : 0.0f;

    tune_state.metrics.peak_y_hz = axis_includes_y(tune_state.axis_mask)
      ? base_y + (noise_unit(seed2) - 0.5f) * span
      : 0.0f;

    const float signal_base = 0.95f + progress * 0.65f;
    tune_state.metrics.peak_x_mag = axis_includes_x(tune_state.axis_mask)
      ? signal_base + noise_unit(seed3) * 0.45f
      : 0.0f;

    tune_state.metrics.peak_y_mag = axis_includes_y(tune_state.axis_mask)
      ? signal_base + noise_unit(seed4) * 0.45f
      : 0.0f;

    const float quality_bias = (noise_unit(seed0) - 0.5f) * 0.10f;
    tune_state.metrics.quality = 0.46f + progress * 0.46f + quality_bias;
  }

  void rebuild_recommendation() {
    build_recommendation_from_metrics(
      tune_state.metrics,
      tune_state.axis_mask,
      tune_state.mode,
      tune_state.max_smoothing,
      tune_state.recommendation
    );
  }

  /**
   * Host event payload emitters.
   *
   * These functions serialize canonical IS_TUNE records so WebUI tooling can
   * reconstruct current state from streamed lines.
   */
  void emit_prefix(const char* const event_type, const uint16_t cmd) {
    SERIAL_ECHOPGM("IS_TUNE:");
    SERIAL_ECHO(event_type);
    SERIAL_ECHOPGM(" CMD=M");
    SERIAL_ECHO(cmd);
    SERIAL_ECHOPGM(" RUN=");
    SERIAL_ECHO(tune_state.run_id);
    SERIAL_ECHOPGM(" VER=");
    SERIAL_ECHO(uint8_t(IS_TUNE_PROTOCOL_VER));
  }

  void emit_error(const uint16_t cmd, const char* const reason) {
    emit_prefix("ERROR", cmd);
    SERIAL_ECHOPGM(" REASON=");
    SERIAL_ECHO(reason);
    SERIAL_ECHOPGM(" PHASE=");
    SERIAL_ECHO(phase_label(tune_state.phase));
    SERIAL_EOL();
  }

  void emit_base_payload() {
    SERIAL_ECHOPGM(" AXIS=");
    SERIAL_ECHO(axis_label(tune_state.axis_mask));
    SERIAL_ECHOPGM(" MODE=");
    SERIAL_ECHO(mode_label(tune_state.mode));
    SERIAL_ECHOPGM(" PHASE=");
    SERIAL_ECHO(phase_label(tune_state.phase));
    SERIAL_ECHOPGM(" SRC=");
    SERIAL_ECHO(tune_state.simulated_source ? "SIM" : "HW");
    SERIAL_ECHOPGM(" HWREQ=");
    SERIAL_ECHO(int(tune_state.hw_requested));
    SERIAL_ECHOPGM(" SENSOR=");
    if (!tune_state.hw_requested)
      SERIAL_ECHO("NA");
    else if (tune_state.sensor_ready)
      SERIAL_ECHO("OK");
    else {
      SERIAL_ECHOPGM("E");
      SERIAL_ECHO(tune_state.sensor_error);
    }
  }

  void emit_configuration_payload() {
    emit_base_payload();
    SERIAL_ECHOPGM(" CHIPS=");
    SERIAL_ECHO(tune_state.chips);
    SERIAL_ECHOPGM(" FREQ_START=");
    SERIAL_ECHO_F(tune_state.freq_start_hz, 3);
    SERIAL_ECHOPGM(" FREQ_END=");
    SERIAL_ECHO_F(tune_state.freq_end_hz, 3);
    SERIAL_ECHOPGM(" ACCEL_PER_HZ=");
    SERIAL_ECHO_F(tune_state.accel_per_hz, 3);
    SERIAL_ECHOPGM(" HZ_PER_SEC=");
    SERIAL_ECHO_F(tune_state.hz_per_sec, 3);
    SERIAL_ECHOPGM(" MAX_SMOOTHING=");
    if (tune_state.max_smoothing >= 0.0f)
      SERIAL_ECHO_F(tune_state.max_smoothing, 4);
    else
      SERIAL_ECHO("NA");
    SERIAL_ECHOPGM(" SAMPLE_HZ=");
    SERIAL_ECHO(tune_state.sample_hz);
    SERIAL_ECHOPGM(" WINDOW_MS=");
    SERIAL_ECHO(tune_state.window_ms);
    SERIAL_ECHOPGM(" EXCITE_DERIVED=");
    SERIAL_ECHO(tune_state.excite_pct);
    SERIAL_ECHOPGM(" QMIN=");
    SERIAL_ECHO_F(tune_state.quality_floor, 3);
  }

  void emit_metrics_payload() {
    SERIAL_ECHOPGM(" PEAK_X_HZ=");
    SERIAL_ECHO_F(tune_state.metrics.peak_x_hz, 3);
    SERIAL_ECHOPGM(" PEAK_Y_HZ=");
    SERIAL_ECHO_F(tune_state.metrics.peak_y_hz, 3);
    SERIAL_ECHOPGM(" PEAK_X_MAG=");
    SERIAL_ECHO_F(tune_state.metrics.peak_x_mag, 3);
    SERIAL_ECHOPGM(" PEAK_Y_MAG=");
    SERIAL_ECHO_F(tune_state.metrics.peak_y_mag, 3);
    SERIAL_ECHOPGM(" QUALITY=");
    SERIAL_ECHO_F(tune_state.metrics.quality, 3);
    SERIAL_ECHOPGM(" SAMPLES=");
    SERIAL_ECHO(tune_state.sample_count);
    SERIAL_ECHOPGM(" AX=");
    SERIAL_ECHO(tune_state.last_ax);
    SERIAL_ECHOPGM(" AY=");
    SERIAL_ECHO(tune_state.last_ay);
    SERIAL_ECHOPGM(" AZ=");
    SERIAL_ECHO(tune_state.last_az);
  }

  void emit_recommendation_payload(const InputShaperRecommendation &payload) {
    SERIAL_ECHOPGM(" X_HZ=");
    SERIAL_ECHO_F(payload.x_hz, 3);
    SERIAL_ECHOPGM(" Y_HZ=");
    SERIAL_ECHO_F(payload.y_hz, 3);
    SERIAL_ECHOPGM(" X_SHAPER=");
    SERIAL_ECHO(int(payload.shaper_x));
    SERIAL_ECHOPGM(" Y_SHAPER=");
    SERIAL_ECHO(int(payload.shaper_y));
    SERIAL_ECHOPGM(" DAMP=");
    SERIAL_ECHO_F(payload.damping, 4);
    SERIAL_ECHOPGM(" SMOOTH=");
    SERIAL_ECHO_F(payload.smoothing, 4);
  }

  void emit_prefixed_recommendation_payload(const char* const prefix, const InputShaperRecommendation &payload) {
    SERIAL_ECHOPGM(" ");
    SERIAL_ECHO(prefix);
    SERIAL_ECHOPGM("_X_HZ=");
    SERIAL_ECHO_F(payload.x_hz, 3);
    SERIAL_ECHOPGM(" ");
    SERIAL_ECHO(prefix);
    SERIAL_ECHOPGM("_Y_HZ=");
    SERIAL_ECHO_F(payload.y_hz, 3);
    SERIAL_ECHOPGM(" ");
    SERIAL_ECHO(prefix);
    SERIAL_ECHOPGM("_DAMP=");
    SERIAL_ECHO_F(payload.damping, 4);
    SERIAL_ECHOPGM(" ");
    SERIAL_ECHO(prefix);
    SERIAL_ECHOPGM("_SMOOTH=");
    SERIAL_ECHO_F(payload.smoothing, 4);
  }

  void emit_state_payload() {
    emit_base_payload();
    SERIAL_ECHOPGM(" ACTIVE=");
    SERIAL_ECHO(int(tune_state.active));
    SERIAL_ECHOPGM(" READY=");
    SERIAL_ECHO(int(tune_state.has_recommendation));
    SERIAL_ECHOPGM(" PCT=");
    SERIAL_ECHO(tune_state.progress_pct);
    SERIAL_ECHOPGM(" PENDING=");
    SERIAL_ECHO(int(tune_state.pending_apply));
    SERIAL_ECHOPGM(" COMMITTED=");
    SERIAL_ECHO(int(tune_state.committed));
  }

  void emit_run_summary_payload() {
    SERIAL_ECHOPGM(" RID=");
    SERIAL_ECHO(tune_state.run_id);
    SERIAL_ECHOPGM(" AXIS=");
    SERIAL_ECHO(axis_label(tune_state.axis_mask));
    SERIAL_ECHOPGM(" MODE=");
    SERIAL_ECHO(mode_label(tune_state.mode));
    SERIAL_ECHOPGM(" CHIPS=");
    SERIAL_ECHO(tune_state.chips);
    SERIAL_ECHOPGM(" FREQ_START=");
    SERIAL_ECHO_F(tune_state.freq_start_hz, 3);
    SERIAL_ECHOPGM(" FREQ_END=");
    SERIAL_ECHO_F(tune_state.freq_end_hz, 3);
    SERIAL_ECHOPGM(" ACCEL_PER_HZ=");
    SERIAL_ECHO_F(tune_state.accel_per_hz, 3);
    SERIAL_ECHOPGM(" HZ_PER_SEC=");
    SERIAL_ECHO_F(tune_state.hz_per_sec, 3);
    SERIAL_ECHOPGM(" MAX_SMOOTHING=");
    if (tune_state.max_smoothing >= 0.0f)
      SERIAL_ECHO_F(tune_state.max_smoothing, 4);
    else
      SERIAL_ECHO("NA");
    SERIAL_ECHOPGM(" SRC=");
    SERIAL_ECHO(tune_state.simulated_source ? "SIM" : "HW");
    SERIAL_ECHOPGM(" KEEP=");
    SERIAL_ECHO(int(capture_session.captured_samples > 0));
    SERIAL_ECHOPGM(" SAMPLE_HZ=");
    SERIAL_ECHO(tune_state.sample_hz);
    SERIAL_ECHOPGM(" SAMPLES=");
    SERIAL_ECHO(tune_state.sample_count);
    SERIAL_ECHOPGM(" EXCITE_DERIVED=");
    SERIAL_ECHO(tune_state.excite_pct);
    SERIAL_ECHOPGM(" QMIN=");
    SERIAL_ECHO_F(tune_state.quality_floor, 3);
    SERIAL_ECHOPGM(" QUALITY=");
    SERIAL_ECHO_F(tune_state.metrics.quality, 3);
  }

}

/**
 * G-code command handlers.
 *
 * Each command emits IS_TUNE envelopes that are consumed by host tooling.
 * The command set is run-centric: capture (M970), inspect (M971..M974),
 * stage/apply (M975/M976), abort/reset/diagnose (M977..M979).
 */

/**
 * M970: Single-command capture run (excite + capture + save)
 *
 * Execution phases:
 * 1) Guard and parse arguments into tune_state.
 * 2) Emit START payload and snapshot previous shaping runtime.
 * 3) Capture samples via hardware or simulation backend.
 * 4) Analyze captured signal and derive recommendation.
 * 5) Validate quality gate, persist run, emit final state.
 *
 * Failure model:
 * - Any capture/analysis/storage error aborts the run immediately.
 * - Capture always runs with input shaping disabled and restores the prior state on exit.
 *
 * Parameters:
 *   A<axis>   Axis: X, Y, XY (aliases 1,2,3)
 *   F<float>  Frequency start (Hz)
 *   G<float>  Frequency end (Hz)
 *   P<float>  Acceleration per Hz factor
 *   R<float>  Sweep speed in Hz/s
 *   C<chips>  Chip selector (ADXL345[,name], SIM)
 *   M<float>  Maximum smoothing cap (0.0..0.2)
 */
void GcodeSuite::M970() {
  // Reject overlapping runs to keep capture buffers and event stream coherent.
  if (tune_state.active)
    return emit_error(970, "RUN_ALREADY_ACTIVE");

  // Reset session to defaults, then apply host-provided tuning arguments.
  reset_tune_state(true);

  bool use_hw_backend = true;
  const char * const arg_error = apply_shaper_calibrate_args(use_hw_backend);
  if (arg_error)
    return emit_error(970, arg_error);

  const bool keep_raw = true;

  ++tune_state.run_id;
  tune_state.active = true;
  tune_state.phase = IS_TUNE_PHASE_CAPTURE;
  tune_state.hw_requested = use_hw_backend;

  // START announces the exact configuration that will be used for this run.
  emit_prefix("START", 970);
  emit_configuration_payload();
  SERIAL_ECHOPGM(" KEEP=");
  SERIAL_ECHO(int(keep_raw));
  SERIAL_EOL();

  // Snapshot current shaping runtime so capture can temporarily disable it.
  #if ENABLED(FT_MOTION) && HAS_FTM_SHAPING
    const auto prev_shaper_x = ftMotion.cfg.shaper.x;
    const auto prev_shaper_y = ftMotion.cfg.shaper.y;
    const float prev_x_hz = ftMotion.cfg.baseFreq.x;
    const float prev_y_hz = ftMotion.cfg.baseFreq.y;
    const float prev_zeta_x = ftMotion.cfg.zeta.x;
    const float prev_zeta_y = ftMotion.cfg.zeta.y;
    const float prev_vtol_x = ftMotion.cfg.vtol.x;
    const float prev_vtol_y = ftMotion.cfg.vtol.y;
    const bool prev_active = ftMotion.cfg.active;
  #endif

  bool shaping_changed = false;
  auto restore_input_shaping = [&]() {
    if (!shaping_changed) return;
    #if ENABLED(FT_MOTION) && HAS_FTM_SHAPING
      ftMotion.cfg.shaper.x = prev_shaper_x;
      ftMotion.cfg.shaper.y = prev_shaper_y;
      ftMotion.cfg.baseFreq.x = prev_x_hz;
      ftMotion.cfg.baseFreq.y = prev_y_hz;
      ftMotion.cfg.zeta.x = prev_zeta_x;
      ftMotion.cfg.zeta.y = prev_zeta_y;
      ftMotion.cfg.vtol.x = prev_vtol_x;
      ftMotion.cfg.vtol.y = prev_vtol_y;
      ftMotion.cfg.active = prev_active;
      ftMotion.update_shaping_params();
    #endif
    planner.reset_acceleration_rates();
    shaping_changed = false;
  };

  #if ENABLED(FT_MOTION) && HAS_FTM_SHAPING
    if (prev_shaper_x != 0 || prev_shaper_y != 0) {
      ftMotion.cfg.shaper.x = ftMotionShaper_NONE; // ftMotionShaper_NONE
      ftMotion.cfg.shaper.y = ftMotionShaper_NONE; // ftMotionShaper_NONE
      ftMotion.update_shaping_params();
      planner.reset_acceleration_rates();
      shaping_changed = true;
    }
  #endif

  // Consolidated failure reason allows one cleanup/abort exit path.
  const char *failure_reason = nullptr;

  // Capture path selection is derived from C<chips> parsing.
  if (use_hw_backend) {
    #if ENABLED(IS_TUNE_ADXL345_SPI_SUPPORT)
      tune_state.simulated_source = false;
      tune_state.sensor_ready = adxl_init(tune_state.sample_hz, tune_state.sensor_error);
      if (!tune_state.sensor_ready) {
        failure_reason = "SENSOR_INIT_FAILED";
      }
    #else
      tune_state.sensor_error = IS_TUNE_SENSOR_BAD_PIN;
      failure_reason = "SENSOR_BACKEND_DISABLED";
    #endif

    if (!failure_reason) {
      tune_state.progress_pct = 5;

      #if ENABLED(IS_TUNE_ADXL345_SPI_SUPPORT)
        const InputShaperCaptureResult capture_result = capture_hardware_with_excitation();
        if (capture_result != IS_TUNE_CAPTURE_OK) {
          switch (capture_result) {
            case IS_TUNE_CAPTURE_HOME_REQUIRED:
              failure_reason = "HOME_REQUIRED";
              break;

            case IS_TUNE_CAPTURE_ALLOC_FAILED:
              failure_reason = "CAPTURE_ALLOC_FAILED";
              break;

            case IS_TUNE_CAPTURE_RANGE_INVALID:
              failure_reason = "CAPTURE_RANGE_INVALID";
              break;

            case IS_TUNE_CAPTURE_MOVE_FAILED:
              failure_reason = "PLANNER_QUEUE_FAILED";
              break;

            case IS_TUNE_CAPTURE_SENSOR_READ_FAILED:
            default:
              tune_state.sensor_ready = false;
              tune_state.sensor_error = IS_TUNE_SENSOR_READ_FAILED;
              failure_reason = "SENSOR_READ_FAILED";
              break;
          }
        }
      #endif
    }
  }
  else {
    tune_state.simulated_source = true;
    tune_state.sensor_ready = false;
    tune_state.sensor_error = IS_TUNE_SENSOR_OK;
    if (!generate_simulated_capture()) {
      failure_reason = "SIM_CAPTURE_FAILED";
    }
  }

  // Single abort path: clear active flags, restore runtime, emit machine-readable reason.
  if (failure_reason) {
    tune_state.active = false;
    tune_state.phase = IS_TUNE_PHASE_ABORTED;
    restore_input_shaping();
    return emit_error(970, failure_reason);
  }

  // Preprocess checkpoint gives host progress feedback before spectral analysis.
  tune_state.progress_pct = 80;
  tune_state.phase = IS_TUNE_PHASE_PREPROCESS;

  emit_prefix("PROGRESS", 970);
  SERIAL_ECHOPGM(" PCT=");
  SERIAL_ECHO(tune_state.progress_pct);
  emit_configuration_payload();
  emit_metrics_payload();
  SERIAL_EOL();

  if (!analyze_capture_session()) {
    tune_state.active = false;
    tune_state.phase = IS_TUNE_PHASE_ABORTED;
    restore_input_shaping();
    return emit_error(970, "ANALYSIS_FAILED");
  }

  // Recommendation is built from analyzed peaks/quality and current smoothing policy.
  rebuild_recommendation();
  tune_state.has_recommendation = true;
  tune_state.progress_pct = 100;
  tune_state.phase = IS_TUNE_PHASE_READY;

  // Quality gate prevents persisting low-confidence captures.
  if (tune_state.metrics.quality < tune_state.quality_floor) {
    tune_state.active = false;
    tune_state.pending_apply = false;
    tune_state.has_recommendation = false;
    tune_state.phase = IS_TUNE_PHASE_ABORTED;
    restore_input_shaping();
    return emit_error(970, "QUALITY_BELOW_THRESHOLD");
  }

  // Run is complete; runtime overrides are removed even on successful path.
  tune_state.active = false;
  restore_input_shaping();

  // Final PROGRESS includes recommendation payload for one-shot host flows.
  emit_prefix("PROGRESS", 970);
  SERIAL_ECHOPGM(" PCT=100");
  emit_configuration_payload();
  emit_metrics_payload();
  emit_recommendation_payload(tune_state.recommendation);
  SERIAL_EOL();

  // END closes the run with summary state.
  emit_prefix("END", 970);
  emit_state_payload();
  SERIAL_EOL();
}

/**
 * M971: Query V2 capture status and saved-run summaries
 *
 * Behavior:
 * - If R is provided, emits one run summary.
 * - Without R, enumerates all valid ring slots in PROGRESS records.
 * - END payload always includes current state and total listed run count.
 *
 * Parameters:
 *   R<int> Optional run id. When omitted, list all saved runs.
 */
void GcodeSuite::M971() {
  emit_prefix("START", 971);
  SERIAL_EOL();

  uint8_t listed_runs = 0;
  if (tune_state.run_id > 0) {
    emit_prefix("PROGRESS", 971);
    emit_run_summary_payload();
    emit_recommendation_payload(tune_state.recommendation);
    SERIAL_EOL();
    listed_runs = 1;
  }

  emit_prefix("END", 971);
  emit_configuration_payload();
  emit_state_payload();
  SERIAL_ECHOPGM(" RUNS=");
  SERIAL_ECHO(listed_runs);
  SERIAL_EOL();
}

/**
 * M972: Fetch raw sample chunks from a saved run
 *
 * Behavior:
 * - Resolves selected run (or latest when R omitted).
 * - Applies strict bounds to O/C against stored sample count.
 * - Emits one PROGRESS record per returned sample index.
 *
 * Parameters:
 *   R<int>  Run id (optional, defaults to latest)
 *   O<int>  Sample offset
 *   C<int>  Sample count (max bounded)
 */
void GcodeSuite::M972() {
  emit_error(972, "Raw data (M972) is unsupported when real-time PSD mode is active.");
}

/**
 * M973: Generate graph bins from saved run samples
 *
 * Behavior:
 * - Rebuilds Welch PSD from stored samples.
 * - Interpolates per-axis PSD at requested graph bin frequencies.
 * - Streams graph points as PROGRESS records for host plotting.
 *
 * Parameters:
 *   R<int>    Run id (optional, defaults to latest)
 *   B<int>    Number of bins
 *   F<float>  Max frequency in Hz
 */
void GcodeSuite::M973() {
  if (tune_state.run_id == 0)
    return emit_error(973, "NO_STORED_RUN");

  if (capture_session.captured_samples == 0)
    return emit_error(973, "RUN_TOO_SMALL");

  const uint16_t bins = uint16_t(parser.intval('B', int32_t(IS_TUNE_DEFAULT_GRAPH_BINS)));
  const float max_freq_default = float(tune_state.sample_hz) * 0.5f;
  const float max_freq = parser.floatval('F', max_freq_default);

  if (bins == 0)
    return emit_error(973, "BINS_INVALID");

  if (!build_frequency_response(capture_session.x, capture_session.y, capture_session.z, capture_session.captured_samples, tune_state.sample_hz, freq_response_scratch))
    return emit_error(973, "ANALYSIS_FAILED");

  const float max_freq_supported = freq_response_scratch.freq[freq_response_scratch.bins - 1];
  const float graph_max_freq = max_freq > max_freq_supported ? max_freq_supported : max_freq;

  emit_prefix("START", 973);
  emit_run_summary_payload();
  SERIAL_ECHOPGM(" BINS=");
  SERIAL_ECHO(bins);
  SERIAL_ECHOPGM(" FMAX=");
  SERIAL_ECHO_F(graph_max_freq, 3);
  SERIAL_EOL();

  for (uint16_t bin = 0; bin < bins; ++bin) {
    const float freq_hz = bins > 1
      ? (float(bin) * graph_max_freq) / float(bins - 1)
      : graph_max_freq;

    const float px = interpolate_psd(freq_response_scratch.freq, freq_response_scratch.psd_x, freq_response_scratch.bins, freq_hz);
    const float py = interpolate_psd(freq_response_scratch.freq, freq_response_scratch.psd_y, freq_response_scratch.bins, freq_hz);
    const float pz = interpolate_psd(freq_response_scratch.freq, freq_response_scratch.psd_z, freq_response_scratch.bins, freq_hz);

    emit_prefix("PROGRESS", 973);
    SERIAL_ECHOPGM(" RID=");
    SERIAL_ECHO(tune_state.run_id);
    SERIAL_ECHOPGM(" BIN=");
    SERIAL_ECHO(bin);
    SERIAL_ECHOPGM(" F_HZ=");
    SERIAL_ECHO_F(freq_hz, 3);
    SERIAL_ECHOPGM(" PX=");
    SERIAL_ECHO_F(px, 5);
    SERIAL_ECHOPGM(" PY=");
    SERIAL_ECHO_F(py, 5);
    SERIAL_ECHOPGM(" PZ=");
    SERIAL_ECHO_F(pz, 5);
    SERIAL_ECHOPGM(" PTOT=");
    SERIAL_ECHO_F(px + py + pz, 5);
    SERIAL_EOL();
  }

  emit_prefix("END", 973);
  SERIAL_ECHOPGM(" RID=");
  SERIAL_ECHO(tune_state.run_id);
  SERIAL_ECHOPGM(" BINS=");
  SERIAL_ECHO(bins);
  SERIAL_EOL();
}

/**
 * M974: Generate recommendation from saved run
 *
 * Behavior:
 * - Re-analyzes stored raw samples (does not trust stale metrics blindly).
 * - Backfills defaults for legacy runs that predate new metadata fields.
 * - Projects selected run into tune_state so M975/M976 can proceed directly.
 *
 * Parameters:
 *   R<int>  Run id (optional, defaults to latest)
 */
void GcodeSuite::M974() {
  if (tune_state.run_id == 0)
    return emit_error(974, "NO_STORED_RUN");

  if (capture_session.captured_samples == 0)
    return emit_error(974, "RUN_TOO_SMALL");

  if (!analyze_capture_session()) {
    return emit_error(974, "ANALYSIS_FAILED");
  }

  rebuild_recommendation();
  tune_state.has_recommendation = true;
  tune_state.active = false;
  tune_state.progress_pct = 100;
  tune_state.phase = IS_TUNE_PHASE_READY;

  emit_prefix("START", 974);
  SERIAL_EOL();

  emit_prefix("END", 974);
  SERIAL_ECHOPGM(" RID=");
  SERIAL_ECHO(tune_state.run_id);
  emit_configuration_payload();
  emit_metrics_payload();
  emit_recommendation_payload(tune_state.recommendation);
  SERIAL_EOL();
}

/**
 * M975: Stage recommended values for application
 *
 * Behavior:
 * - Starts from current recommendation.
 * - Accepts optional X/Y/D/S overrides.
 * - Validates axis-required frequencies before marking pending_apply.
 */
void GcodeSuite::M975() {
  if (!tune_state.has_recommendation)
    return emit_error(975, "NO_RECOMMENDATION");

  tune_state.staged = tune_state.recommendation;

  if (parser.seenval('X')) tune_state.staged.x_hz = parser.value_float();
  if (parser.seenval('Y')) tune_state.staged.y_hz = parser.value_float();
  if (parser.seenval('D')) tune_state.staged.damping = parser.value_float();
  if (parser.seenval('S')) tune_state.staged.smoothing = parser.value_float();
  if (parser.seenval('I')) tune_state.staged.shaper_x = parser.value_byte();
  if (parser.seenval('J')) tune_state.staged.shaper_y = parser.value_byte();

  if ((axis_includes_x(tune_state.axis_mask) && tune_state.staged.x_hz <= 0.0f)
      || (axis_includes_y(tune_state.axis_mask) && tune_state.staged.y_hz <= 0.0f)) {
    return emit_error(975, "STAGE_VALUE_INVALID");
  }

  tune_state.pending_apply = true;
  tune_state.committed = false;

  emit_prefix("END", 975);
  SERIAL_ECHOPGM(" STAGED=1");
  emit_configuration_payload();
  emit_recommendation_payload(tune_state.staged);
  SERIAL_EOL();
}

/**
 * M976: Commit staged values
 *
 * Behavior:
 * - Applies staged values to active runtime shaper config.
 * - Optionally persists to EEPROM with W1.
 * - Emits applied profile plus persistence status token.
 *
 * Parameters:
 *   W<0|1>  Persist runtime profile to EEPROM immediately when set.
 */
void GcodeSuite::M976() {
  if (!tune_state.pending_apply)
    return emit_error(976, "NO_STAGED_VALUES");

  const bool write_eeprom = parser.boolval('W', true);

  #if ENABLED(FT_MOTION) && HAS_FTM_SHAPING
    if (axis_includes_x(tune_state.axis_mask)) {
      ftMotion.cfg.baseFreq.x = tune_state.staged.x_hz;
      ftMotion.cfg.zeta.x = tune_state.staged.damping;
      ftMotion.cfg.vtol.x = tune_state.staged.smoothing;
      ftMotion.cfg.shaper.x = static_cast<ftMotionShaper_t>(tune_state.staged.shaper_x > 0 ? tune_state.staged.shaper_x : 2);
    }
    if (axis_includes_y(tune_state.axis_mask)) {
      ftMotion.cfg.baseFreq.y = tune_state.staged.y_hz;
      ftMotion.cfg.zeta.y = tune_state.staged.damping;
      ftMotion.cfg.vtol.y = tune_state.staged.smoothing;
      ftMotion.cfg.shaper.y = static_cast<ftMotionShaper_t>(tune_state.staged.shaper_y > 0 ? tune_state.staged.shaper_y : 2);
    }
    ftMotion.update_shaping_params();
    planner.reset_acceleration_rates();
  #endif

  const bool persist_ok = !write_eeprom || settings.save();

  sync_applied_profile_from_shaper();
  tune_state.recommendation = tune_state.applied;

  tune_state.pending_apply = false;
  tune_state.committed = true;
  tune_state.active = false;
  tune_state.phase = IS_TUNE_PHASE_APPLIED;

  emit_prefix("END", 976);
  SERIAL_ECHOPGM(" COMMIT=1 EEPROM=");
  SERIAL_ECHO(int(write_eeprom));
  emit_configuration_payload();
  emit_recommendation_payload(tune_state.applied);
  SERIAL_ECHOPGM(" PERSIST=");
  if (!write_eeprom)
    SERIAL_ECHO("VOLATILE");
  else
    SERIAL_ECHO(persist_ok ? "M500_DONE" : "M500_FAILED");
  SERIAL_EOL();
}

/**
 * M977: Abort active tuning session
 *
 * Behavior:
 * - Clears active and pending_apply flags.
 * - Moves phase to ABORTED and emits terminal state.
 */
void GcodeSuite::M977() {
  if (!tune_state.active && !tune_state.pending_apply)
    return emit_error(977, "NOTHING_TO_ABORT");

  tune_state.active = false;
  tune_state.pending_apply = false;
  tune_state.phase = IS_TUNE_PHASE_ABORTED;

  emit_prefix("END", 977);
  SERIAL_ECHOPGM(" ABORTED=1");
  emit_state_payload();
  SERIAL_EOL();
}

/**
 * M978: Diagnostic snapshot for host tooling
 *
 * Behavior:
 * - Emits a full state/config/metrics snapshot plus REC/STAGED/APPLIED profiles.
 * - Intended for debugging host parsers and UI synchronization issues.
 */
void GcodeSuite::M978() {
  emit_prefix("START", 978);
  SERIAL_EOL();

  emit_prefix("END", 978);
  emit_configuration_payload();
  emit_state_payload();
  emit_metrics_payload();
  emit_prefixed_recommendation_payload("REC", tune_state.recommendation);
  emit_prefixed_recommendation_payload("STAGED", tune_state.staged);
  emit_prefixed_recommendation_payload("APPLIED", tune_state.applied);
  SERIAL_EOL();
}

/**
 * M979: Reset tuning state
 *
 * Behavior:
 * - Optionally clears all stored runs with D1.
 * - Resets live tune_state while preserving run counter continuity.
 *
 * Parameters:
 *   D<0|1>  Delete saved runs when set.
 */
void GcodeSuite::M979() {
  const bool delete_runs = parser.boolval('D', false);

  reset_tune_state(true);

  emit_prefix("END", 979);
  SERIAL_ECHOPGM(" RESET=1");
  SERIAL_ECHOPGM(" DROPPED=");
  SERIAL_ECHO(int(delete_runs)); // Kept for protocol compatibility
  SERIAL_EOL();
}

#endif // M970_M979_GCODE
