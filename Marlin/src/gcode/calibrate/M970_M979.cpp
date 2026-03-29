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
#include "../../module/input_shaper_runtime.h"
#include "../../module/planner.h"
#include "../../module/settings.h"
#include "../../module/stepper.h"

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

  constexpr uint8_t IS_TUNE_PROTOCOL_VER = 1;
  constexpr uint16_t DEFAULT_SAMPLE_HZ = 3200;
  constexpr uint16_t DEFAULT_WINDOW_MS = 1200;
  constexpr uint8_t DEFAULT_EXCITE_PCT = 30;
  constexpr float DEFAULT_DAMPING = 0.10f;
  constexpr float DEFAULT_SMOOTHING = 0.00f;
  constexpr float DEFAULT_QUALITY_FLOOR = 0.35f;
  constexpr uint16_t IS_TUNE_MIN_CAPTURE_SAMPLES = 96;
  constexpr uint16_t IS_TUNE_MAX_CAPTURE_SAMPLES = 512;
  constexpr uint8_t IS_TUNE_CAPTURE_CHUNK_SAMPLES = 32;

  enum InputShaperSensorError : uint8_t {
    IS_TUNE_SENSOR_OK,
    IS_TUNE_SENSOR_BAD_PIN,
    IS_TUNE_SENSOR_DEVID_MISMATCH,
    IS_TUNE_SENSOR_POWER_CTL,
    IS_TUNE_SENSOR_READ_FAILED
  };

  enum InputShaperTunePhase : uint8_t {
    IS_TUNE_PHASE_IDLE,
    IS_TUNE_PHASE_CAPTURE,
    IS_TUNE_PHASE_PREPROCESS,
    IS_TUNE_PHASE_READY,
    IS_TUNE_PHASE_APPLIED,
    IS_TUNE_PHASE_ABORTED
  };

  struct InputShaperMetrics {
    float peak_x_hz;
    float peak_y_hz;
    float peak_x_mag;
    float peak_y_mag;
    float quality;
  };

  struct InputShaperRecommendation {
    float x_hz;
    float y_hz;
    float damping;
    float smoothing;
  };

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
  };

  struct InputShaperCaptureSession {
    uint16_t target_samples;
    uint16_t captured_samples;
    bool analyzed;
    uint64_t abs_sum_x;
    uint64_t abs_sum_y;
    int16_t x[IS_TUNE_MAX_CAPTURE_SAMPLES];
    int16_t y[IS_TUNE_MAX_CAPTURE_SAMPLES];
    int16_t z[IS_TUNE_MAX_CAPTURE_SAMPLES];
  };

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
    { 0.0f, 0.0f, DEFAULT_DAMPING, DEFAULT_SMOOTHING },
    { 0.0f, 0.0f, DEFAULT_DAMPING, DEFAULT_SMOOTHING },
    { 0.0f, 0.0f, DEFAULT_DAMPING, DEFAULT_SMOOTHING },
    false,
    false,
    false,
    true
    , false,
    false,
    IS_TUNE_SENSOR_OK,
    0,
    0,
    0,
    0,
    0.0f,
    0.0f
  };

  InputShaperCaptureSession capture_session = {};

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

  float clampf(const float value, const float low, const float high) {
    if (value < low) return low;
    if (value > high) return high;
    return value;
  }

  uint16_t clampu16(const int32_t value, const uint16_t low, const uint16_t high) {
    if (value < low) return low;
    if (value > high) return high;
    return uint16_t(value);
  }

  uint8_t clampu8(const int16_t value, const uint8_t low, const uint8_t high) {
    if (value < low) return low;
    if (value > high) return high;
    return uint8_t(value);
  }

  uint32_t noise_lcg(uint32_t seed) {
    return seed * 1664525UL + 1013904223UL;
  }

  float noise_unit(uint32_t seed) {
    return float(seed & 0xFFFFUL) / 65535.0f;
  }

  bool axis_includes_x(const uint8_t axis_mask) { return axis_mask & 0x01; }
  bool axis_includes_y(const uint8_t axis_mask) { return axis_mask & 0x02; }

  void reset_recommendation(InputShaperRecommendation &target) {
    target.x_hz = 0.0f;
    target.y_hz = 0.0f;
    target.damping = DEFAULT_DAMPING;
    target.smoothing = DEFAULT_SMOOTHING;
  }

  void sync_applied_profile_from_shaper() {
    tune_state.applied.x_hz = TERN0(INPUT_SHAPING_X, stepper.get_shaping_frequency(X_AXIS));
    tune_state.applied.y_hz = TERN0(INPUT_SHAPING_Y, stepper.get_shaping_frequency(Y_AXIS));
    tune_state.applied.damping = stepper.get_shaping_damping_ratio(X_AXIS);
    tune_state.applied.smoothing = input_shaper_runtime.smoothing;
    tune_state.committed = input_shaper_runtime.enabled;
  }

  void reset_metrics() {
    tune_state.metrics.peak_x_hz = 0.0f;
    tune_state.metrics.peak_y_hz = 0.0f;
    tune_state.metrics.peak_x_mag = 0.0f;
    tune_state.metrics.peak_y_mag = 0.0f;
    tune_state.metrics.quality = 0.0f;
  }

  void reset_capture_session() {
    capture_session.target_samples = 0;
    capture_session.captured_samples = 0;
    capture_session.analyzed = false;
    capture_session.abs_sum_x = 0;
    capture_session.abs_sum_y = 0;

    tune_state.sample_count = 0;
    tune_state.last_ax = 0;
    tune_state.last_ay = 0;
    tune_state.last_az = 0;
    tune_state.mean_abs_x = 0.0f;
    tune_state.mean_abs_y = 0.0f;
  }

  void configure_capture_session() {
    reset_capture_session();

    uint32_t target = uint32_t(tune_state.sample_hz) * uint32_t(tune_state.window_ms) / 1000UL;
    if (target < IS_TUNE_MIN_CAPTURE_SAMPLES) target = IS_TUNE_MIN_CAPTURE_SAMPLES;
    if (target > IS_TUNE_MAX_CAPTURE_SAMPLES) target = IS_TUNE_MAX_CAPTURE_SAMPLES;

    capture_session.target_samples = uint16_t(target);
  }

  bool estimate_axis_resonance(const int16_t * const samples, const uint16_t count, float &peak_hz, float &peak_mag, float &axis_quality) {
    if (count < 8 || tune_state.sample_hz < 10) return false;

    float mean = 0.0f;
    for (uint16_t i = 0; i < count; ++i) mean += samples[i];
    mean /= float(count);

    float energy = 0.0f;
    for (uint16_t i = 0; i < count; ++i) {
      const float centered = float(samples[i]) - mean;
      energy += centered * centered;
    }
    if (energy < 1.0f) return false;

    peak_mag = clampf(SQRT(energy / float(count)) / 256.0f, 0.0f, 16.0f);

    uint16_t min_lag = tune_state.sample_hz / 120U;
    if (min_lag < 1) min_lag = 1;

    uint16_t max_lag = tune_state.sample_hz / 10U;
    if (count > 2 && max_lag > count - 2) max_lag = count - 2;
    if (max_lag <= min_lag) return false;

    float best_norm = -1.0f;
    uint16_t best_lag = 0;
    for (uint16_t lag = min_lag; lag <= max_lag; ++lag) {
      float corr = 0.0f;
      const uint16_t limit = count - lag;
      for (uint16_t i = 0; i < limit; ++i) {
        const float a = float(samples[i]) - mean;
        const float b = float(samples[i + lag]) - mean;
        corr += a * b;
      }

      const float norm = corr / energy;
      if (norm > best_norm) {
        best_norm = norm;
        best_lag = lag;
      }
    }

    if (best_lag == 0) return false;

    peak_hz = clampf(float(tune_state.sample_hz) / float(best_lag), 10.0f, 120.0f);
    axis_quality = clampf(best_norm, 0.0f, 1.0f);
    return true;
  }

  bool analyze_capture_session() {
    if (capture_session.captured_samples < capture_session.target_samples || capture_session.target_samples == 0)
      return false;

    float quality_sum = 0.0f;
    uint8_t quality_axes = 0;

    tune_state.metrics.peak_x_hz = 0.0f;
    tune_state.metrics.peak_y_hz = 0.0f;

    if (axis_includes_x(tune_state.axis_mask)) {
      float axis_quality = 0.0f, axis_mag = 0.0f, axis_hz = 0.0f;
      if (estimate_axis_resonance(capture_session.x, capture_session.captured_samples, axis_hz, axis_mag, axis_quality)) {
        tune_state.metrics.peak_x_hz = axis_hz;
        tune_state.metrics.peak_x_mag = axis_mag;
        quality_sum += axis_quality;
        ++quality_axes;
      }
    }
    else {
      tune_state.metrics.peak_x_mag = 0.0f;
    }

    if (axis_includes_y(tune_state.axis_mask)) {
      float axis_quality = 0.0f, axis_mag = 0.0f, axis_hz = 0.0f;
      if (estimate_axis_resonance(capture_session.y, capture_session.captured_samples, axis_hz, axis_mag, axis_quality)) {
        tune_state.metrics.peak_y_hz = axis_hz;
        tune_state.metrics.peak_y_mag = axis_mag;
        quality_sum += axis_quality;
        ++quality_axes;
      }
    }
    else {
      tune_state.metrics.peak_y_mag = 0.0f;
    }

    if (quality_axes == 0) return false;

    tune_state.metrics.quality = clampf(quality_sum / float(quality_axes), 0.0f, 1.0f);
    return true;
  }

  void rebuild_recommendation();

  bool advance_hardware_capture() {
    #if ENABLED(IS_TUNE_ADXL345_SPI_SUPPORT)
      if (capture_session.target_samples < IS_TUNE_MIN_CAPTURE_SAMPLES)
        configure_capture_session();

      if (capture_session.captured_samples < capture_session.target_samples) {
        const uint16_t remaining = capture_session.target_samples - capture_session.captured_samples;
        const uint16_t chunk = remaining > IS_TUNE_CAPTURE_CHUNK_SAMPLES ? IS_TUNE_CAPTURE_CHUNK_SAMPLES : remaining;

        int16_t * const x_ptr = &capture_session.x[capture_session.captured_samples];
        int16_t * const y_ptr = &capture_session.y[capture_session.captured_samples];
        int16_t * const z_ptr = &capture_session.z[capture_session.captured_samples];

        if (!adxl_capture_samples(chunk, x_ptr, y_ptr, z_ptr, tune_state.sample_hz))
          return false;

        for (uint16_t i = 0; i < chunk; ++i) {
          const int16_t sx = x_ptr[i], sy = y_ptr[i];
          capture_session.abs_sum_x += sx >= 0 ? sx : uint16_t(-sx);
          capture_session.abs_sum_y += sy >= 0 ? sy : uint16_t(-sy);
        }

        capture_session.captured_samples += chunk;
        tune_state.sample_count = capture_session.captured_samples;

        const uint16_t last_idx = capture_session.captured_samples - 1;
        tune_state.last_ax = capture_session.x[last_idx];
        tune_state.last_ay = capture_session.y[last_idx];
        tune_state.last_az = capture_session.z[last_idx];

        if (capture_session.captured_samples > 0) {
          const float inv_count = 1.0f / float(capture_session.captured_samples);
          tune_state.mean_abs_x = float(capture_session.abs_sum_x) * inv_count;
          tune_state.mean_abs_y = float(capture_session.abs_sum_y) * inv_count;
        }

        tune_state.progress_pct = uint8_t((uint32_t(capture_session.captured_samples) * 70UL) / capture_session.target_samples);
        if (tune_state.progress_pct > 69) tune_state.progress_pct = 69;
        tune_state.phase = IS_TUNE_PHASE_CAPTURE;

        tune_state.metrics.peak_x_mag = axis_includes_x(tune_state.axis_mask) ? clampf(tune_state.mean_abs_x / 128.0f, 0.0f, 16.0f) : 0.0f;
        tune_state.metrics.peak_y_mag = axis_includes_y(tune_state.axis_mask) ? clampf(tune_state.mean_abs_y / 128.0f, 0.0f, 16.0f) : 0.0f;
        tune_state.metrics.quality = clampf(0.20f + (float(tune_state.progress_pct) / 100.0f) * 0.45f, 0.0f, 1.0f);
      }

      if (capture_session.captured_samples >= capture_session.target_samples) {
        tune_state.phase = IS_TUNE_PHASE_PREPROCESS;

        if (!capture_session.analyzed) {
          if (!analyze_capture_session()) return false;
          rebuild_recommendation();
          capture_session.analyzed = true;
        }

        tune_state.progress_pct = 100;
        tune_state.phase = IS_TUNE_PHASE_READY;
        tune_state.has_recommendation = true;
      }

      return true;
    #else
      return false;
    #endif
  }

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
    tune_state.simulated_source = true;
    tune_state.hw_requested = false;
    tune_state.sensor_ready = false;
    tune_state.sensor_error = IS_TUNE_SENSOR_OK;
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

  bool parse_mode(const int16_t raw_mode, uint8_t &mode_out) {
    switch (raw_mode) {
      case 0:
      case 1:
        mode_out = uint8_t(raw_mode);
        return true;
      default:
        return false;
    }
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

  void recalc_phase() {
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

  void rebuild_metrics() {
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
      ? clampf(base_x + (noise_unit(seed1) - 0.5f) * span, 10.0f, 120.0f)
      : 0.0f;

    tune_state.metrics.peak_y_hz = axis_includes_y(tune_state.axis_mask)
      ? clampf(base_y + (noise_unit(seed2) - 0.5f) * span, 10.0f, 120.0f)
      : 0.0f;

    const float signal_base = 0.95f + progress * 0.65f;
    tune_state.metrics.peak_x_mag = axis_includes_x(tune_state.axis_mask)
      ? signal_base + noise_unit(seed3) * 0.45f
      : 0.0f;

    tune_state.metrics.peak_y_mag = axis_includes_y(tune_state.axis_mask)
      ? signal_base + noise_unit(seed4) * 0.45f
      : 0.0f;

    const float quality_bias = (noise_unit(seed0) - 0.5f) * 0.10f;
    tune_state.metrics.quality = clampf(0.46f + progress * 0.46f + quality_bias, 0.0f, 1.0f);
  }

  void rebuild_recommendation() {
    tune_state.recommendation.x_hz = axis_includes_x(tune_state.axis_mask)
      ? clampf(tune_state.metrics.peak_x_hz * 0.985f, 10.0f, 120.0f)
      : 0.0f;

    tune_state.recommendation.y_hz = axis_includes_y(tune_state.axis_mask)
      ? clampf(tune_state.metrics.peak_y_hz * 0.985f, 10.0f, 120.0f)
      : 0.0f;

    tune_state.recommendation.damping = clampf(
      0.06f + (1.0f - tune_state.metrics.quality) * 0.14f,
      0.03f,
      0.35f
    );

    tune_state.recommendation.smoothing = clampf(
      0.01f + (1.0f - tune_state.metrics.quality) * 0.07f + (tune_state.mode ? 0.01f : 0.0f),
      0.0f,
      0.20f
    );
  }

  void apply_configuration_args() {
    if (parser.seenval('A')) {
      uint8_t parsed_axis_mask = 3;
      if (parse_axis_mask(parser.value_int(), parsed_axis_mask))
        tune_state.axis_mask = parsed_axis_mask;
    }

    if (parser.seenval('O')) {
      uint8_t parsed_mode = 0;
      if (parse_mode(parser.value_int(), parsed_mode))
        tune_state.mode = parsed_mode;
    }

    if (parser.seenval('H')) tune_state.sample_hz = clampu16(parser.value_int(), 200, 8000);
    if (parser.seenval('T')) tune_state.window_ms = clampu16(parser.value_int(), 100, 10000);
    if (parser.seenval('E')) tune_state.excite_pct = clampu8(parser.value_int(), 5, 80);
    if (parser.seenval('Q')) tune_state.quality_floor = clampf(parser.value_float(), 0.10f, 0.95f);
  }

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
    SERIAL_ECHOPGM(" SAMPLE_HZ=");
    SERIAL_ECHO(tune_state.sample_hz);
    SERIAL_ECHOPGM(" WINDOW_MS=");
    SERIAL_ECHO(tune_state.window_ms);
    SERIAL_ECHOPGM(" EXCITE=");
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

}

/**
 * M970: Start tuning run
 *
 * Parameters:
 *   A<1|2|3>  Axis mask: 1=X, 2=Y, 3=XY
 *   B<0|1>    Backend: 0=SIM, 1=ADXL345 SPI
 *   O<0|1>    Mode: 0=SWEEP, 1=IMPULSE
 *   H<int>    Effective sample rate in Hz
 *   T<int>    Measurement window in ms
 *   E<int>    Excitation profile intensity (5..80)
 *   Q<float>  Minimum quality threshold (0.10..0.95)
 *   F<0|1>    Force restart when a run is active
 *   X<float>  Optional seed recommendation for X shaper frequency
 *   Y<float>  Optional seed recommendation for Y shaper frequency
 *   D<float>  Optional seed damping ratio
 *   S<float>  Optional seed smoothing
 */
void GcodeSuite::M970() {
  const bool force_restart = parser.boolval('F', false);
  const bool use_hw_backend = parser.boolval('B', false);

  if (tune_state.active && !force_restart)
    return emit_error(970, "RUN_ALREADY_ACTIVE");

  reset_tune_state(true);

  if (parser.seenval('A')) {
    uint8_t parsed_axis_mask = 3;
    if (!parse_axis_mask(parser.value_int(), parsed_axis_mask))
      return emit_error(970, "AXIS_MASK_INVALID");
    tune_state.axis_mask = parsed_axis_mask;
  }

  if (parser.seenval('O')) {
    uint8_t parsed_mode = 0;
    if (!parse_mode(parser.value_int(), parsed_mode))
      return emit_error(970, "MODE_INVALID");
    tune_state.mode = parsed_mode;
  }

  apply_configuration_args();

  ++tune_state.run_id;
  tune_state.active = true;
  tune_state.phase = IS_TUNE_PHASE_CAPTURE;
  tune_state.hw_requested = use_hw_backend;

  if (use_hw_backend) {
    #if ENABLED(IS_TUNE_ADXL345_SPI_SUPPORT)
      tune_state.simulated_source = false;
      tune_state.sensor_ready = adxl_init(tune_state.sample_hz, tune_state.sensor_error);
      if (!tune_state.sensor_ready) {
        tune_state.active = false;
        tune_state.phase = IS_TUNE_PHASE_ABORTED;
        return emit_error(970, "SENSOR_INIT_FAILED");
      }
    #else
      tune_state.active = false;
      tune_state.phase = IS_TUNE_PHASE_ABORTED;
      tune_state.sensor_error = IS_TUNE_SENSOR_BAD_PIN;
      return emit_error(970, "SENSOR_BACKEND_DISABLED");
    #endif

    configure_capture_session();
    tune_state.progress_pct = 0;
  }
  else {
    tune_state.simulated_source = true;
    tune_state.sensor_ready = false;
    tune_state.sensor_error = IS_TUNE_SENSOR_OK;
  }

  tune_state.recommendation.x_hz = parser.floatval('X', 0.0f);
  tune_state.recommendation.y_hz = parser.floatval('Y', 0.0f);
  tune_state.recommendation.damping = clampf(parser.floatval('D', DEFAULT_DAMPING), 0.03f, 0.35f);
  tune_state.recommendation.smoothing = clampf(parser.floatval('S', DEFAULT_SMOOTHING), 0.0f, 0.20f);

  if (tune_state.simulated_source)
    rebuild_metrics();
  else
    reset_metrics();

  emit_prefix("START", 970);
  emit_configuration_payload();
  SERIAL_EOL();

  emit_prefix("PROGRESS", 970);
  SERIAL_ECHOPGM(" PCT=0");
  emit_configuration_payload();
  emit_metrics_payload();
  SERIAL_EOL();

  emit_prefix("END", 970);
  emit_state_payload();
  SERIAL_EOL();
}

/**
 * M971: Query tuning status
 */
void GcodeSuite::M971() {
  emit_prefix("START", 971);
  SERIAL_EOL();

  emit_prefix("END", 971);
  emit_configuration_payload();
  emit_state_payload();
  emit_metrics_payload();
  SERIAL_EOL();
}

/**
 * M972: Update axis / mode / run parameters for current run
 */
void GcodeSuite::M972() {
  if (!tune_state.active)
    return emit_error(972, "NO_ACTIVE_RUN");

  const bool capture_param_changed = parser.seenval('H') || parser.seenval('T');

  if (parser.seenval('A')) {
    uint8_t parsed_axis_mask = 3;
    if (!parse_axis_mask(parser.value_int(), parsed_axis_mask))
      return emit_error(972, "AXIS_MASK_INVALID");
  }

  if (parser.seenval('O')) {
    uint8_t parsed_mode = 0;
    if (!parse_mode(parser.value_int(), parsed_mode))
      return emit_error(972, "MODE_INVALID");
  }

  apply_configuration_args();

  #if ENABLED(IS_TUNE_ADXL345_SPI_SUPPORT)
    if (!tune_state.simulated_source && tune_state.sensor_ready && parser.seenval('H')) {
      if (!adxl_apply_sampling_rate(tune_state.sample_hz)) {
        tune_state.active = false;
        tune_state.phase = IS_TUNE_PHASE_ABORTED;
        tune_state.sensor_ready = false;
        tune_state.sensor_error = IS_TUNE_SENSOR_READ_FAILED;
        return emit_error(972, "SENSOR_RATE_SET_FAILED");
      }
    }
  #endif

  if (!tune_state.simulated_source && tune_state.sensor_ready && capture_param_changed) {
    configure_capture_session();
    tune_state.progress_pct = 0;
    tune_state.has_recommendation = false;
    tune_state.pending_apply = false;
    tune_state.phase = IS_TUNE_PHASE_CAPTURE;
  }

  if (parser.seenval('X')) tune_state.recommendation.x_hz = parser.value_float();
  if (parser.seenval('Y')) tune_state.recommendation.y_hz = parser.value_float();
  if (parser.seenval('D')) tune_state.recommendation.damping = clampf(parser.value_float(), 0.03f, 0.35f);
  if (parser.seenval('S')) tune_state.recommendation.smoothing = clampf(parser.value_float(), 0.0f, 0.20f);

  if (tune_state.simulated_source)
    rebuild_metrics();

  emit_prefix("END", 972);
  emit_configuration_payload();
  emit_state_payload();
  emit_metrics_payload();
  SERIAL_EOL();
}

/**
 * M973: Advance/report progress and generate transient preprocessing metrics
 *
 * Parameters:
 *   P<int>  Optional absolute progress override (0..100)
 *   I<int>  Optional auto-increment for polling path (default 20)
 */
void GcodeSuite::M973() {
  if (!tune_state.active)
    return emit_error(973, "NO_ACTIVE_RUN");

  const bool hardware_capture_mode = !tune_state.simulated_source && tune_state.sensor_ready;

  if (hardware_capture_mode) {
    if (!advance_hardware_capture()) {
      tune_state.active = false;
      tune_state.pending_apply = false;
      tune_state.has_recommendation = false;
      tune_state.phase = IS_TUNE_PHASE_ABORTED;
      tune_state.sensor_ready = false;
      tune_state.sensor_error = IS_TUNE_SENSOR_READ_FAILED;
      return emit_error(973, "SENSOR_READ_FAILED");
    }
  }
  else {
    if (parser.seenval('P')) {
      tune_state.progress_pct = uint8_t(clamp_percent(parser.value_int()));
    }
    else {
      const uint8_t increment = clampu8(parser.intval('I', 20), 1, 100);
      tune_state.progress_pct = uint8_t(clamp_percent(tune_state.progress_pct + increment));
    }

    recalc_phase();
    rebuild_metrics();

    if (tune_state.progress_pct >= 100) {
      rebuild_recommendation();
      tune_state.has_recommendation = true;
      tune_state.phase = IS_TUNE_PHASE_READY;
    }
  }

  if (tune_state.progress_pct >= 100 && tune_state.metrics.quality < tune_state.quality_floor) {
    tune_state.active = false;
    tune_state.pending_apply = false;
    tune_state.has_recommendation = false;
    tune_state.phase = IS_TUNE_PHASE_ABORTED;
    return emit_error(973, "QUALITY_BELOW_THRESHOLD");
  }

  if (parser.seenval('X')) tune_state.recommendation.x_hz = parser.value_float();
  if (parser.seenval('Y')) tune_state.recommendation.y_hz = parser.value_float();
  if (parser.seenval('D')) tune_state.recommendation.damping = clampf(parser.value_float(), 0.03f, 0.35f);
  if (parser.seenval('S')) tune_state.recommendation.smoothing = clampf(parser.value_float(), 0.0f, 0.20f);

  emit_prefix("PROGRESS", 973);
  SERIAL_ECHOPGM(" PCT=");
  SERIAL_ECHO(tune_state.progress_pct);
  emit_configuration_payload();
  emit_metrics_payload();
  if (tune_state.has_recommendation) emit_recommendation_payload(tune_state.recommendation);
  SERIAL_EOL();

  emit_prefix("END", 973);
  emit_state_payload();
  SERIAL_EOL();
}

/**
 * M974: Report recommended shaper values
 */
void GcodeSuite::M974() {
  if (!tune_state.has_recommendation)
    return emit_error(974, "RECOMMENDATION_NOT_READY");

  emit_prefix("START", 974);
  SERIAL_EOL();

  emit_prefix("END", 974);
  emit_configuration_payload();
  emit_metrics_payload();
  emit_recommendation_payload(tune_state.recommendation);
  SERIAL_EOL();
}

/**
 * M975: Stage recommended values for application
 */
void GcodeSuite::M975() {
  if (!tune_state.has_recommendation)
    return emit_error(975, "NO_RECOMMENDATION");

  tune_state.staged = tune_state.recommendation;

  if (parser.seenval('X')) tune_state.staged.x_hz = parser.value_float();
  if (parser.seenval('Y')) tune_state.staged.y_hz = parser.value_float();
  if (parser.seenval('D')) tune_state.staged.damping = clampf(parser.value_float(), 0.03f, 0.35f);
  if (parser.seenval('S')) tune_state.staged.smoothing = clampf(parser.value_float(), 0.0f, 0.20f);

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
 * Parameters:
 *   W<0|1>  Persist runtime profile to EEPROM immediately when set.
 */
void GcodeSuite::M976() {
  if (!tune_state.pending_apply)
    return emit_error(976, "NO_STAGED_VALUES");

  const bool write_eeprom = parser.boolval('W', true);

  if (axis_includes_x(tune_state.axis_mask))
    stepper.set_shaping_frequency(X_AXIS, tune_state.staged.x_hz);
  if (axis_includes_y(tune_state.axis_mask))
    stepper.set_shaping_frequency(Y_AXIS, tune_state.staged.y_hz);

  stepper.set_shaping_damping_ratio(X_AXIS, tune_state.staged.damping);

  if (input_shaper_runtime.smoothing != tune_state.staged.smoothing) {
    input_shaper_runtime_apply(
      input_shaper_runtime.axis_mask,
      stepper.get_shaping_frequency(X_AXIS),
      stepper.get_shaping_frequency(Y_AXIS),
      stepper.get_shaping_damping_ratio(X_AXIS),
      tune_state.staged.smoothing
    );
    planner.reset_acceleration_rates();
  }

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
 */
void GcodeSuite::M979() {
  reset_tune_state(true);

  emit_prefix("END", 979);
  SERIAL_ECHOPGM(" RESET=1");
  SERIAL_EOL();
}

#endif // M970_M979_GCODE