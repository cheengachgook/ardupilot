/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
 */

//	Initial Code by Jon Challinger
//  Modified by Paul Riseborough

#include <AP_HAL/AP_HAL.h>
#include "AP_PitchController.h"
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Scheduler/AP_Scheduler.h>
#include <AP_Airspeed/AP_Airspeed.h>
#include <GCS_MAVLink/GCS.h>
#include <SRV_Channel/SRV_Channel.h>

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_PitchController::var_info[] = {
    AP_GROUPINFO("2SRV_TCONST",      0, AP_PitchController, gains.tau,       0.5f),
    AP_GROUPINFO("2SRV_RMAX_UP",     4, AP_PitchController, gains.rmax_pos,   0.0f),
    AP_GROUPINFO("2SRV_RMAX_DN",     5, AP_PitchController, gains.rmax_neg,   0.0f),
    AP_GROUPINFO("2SRV_RLL",         6, AP_PitchController, _roll_ff,        1.0f),
    AP_SUBGROUPINFO(rate_pid, "_RATE_", 11, AP_PitchController, AC_PID),
    AP_GROUPINFO("2SRV_ACCEL",      12, AP_PitchController, accel_limit, 500),
    AP_GROUPINFO("_ANGLE_P",        13, AP_PitchController, angle_p, 0.0),

#if AP_ESNCONTROLLER_ENABLED
    AP_SUBGROUPINFO(_esn_P, "_ESN_", 14, AP_PitchController, ESNController),
    AP_GROUPINFO("_ESN_RESET", 15, AP_PitchController, _esn_reset_param, 0),
    AP_SUBGROUPINFO(_esn_D, "_ESND_", 16, AP_PitchController, ESNController),
#endif

    AP_GROUPEND
};

AP_PitchController::AP_PitchController(const AP_FixedWing &parms)
    : AP_FW_Controller(parms,
      AC_PID::Defaults{
        .p         = 0.04,
        .i         = 0.15,
        .d         = 0.0,
        .ff        = 0.345,
        .imax      = 0.666,
        .filt_T_hz = 3.0,
        .filt_E_hz = 0.0,
        .filt_D_hz = 12.0,
        .srmax     = 150.0,
        .srtau     = 1.0
    },
    AP_AutoTune::ATType::AUTOTUNE_PITCH)
{
    AP_Param::setup_object_defaults(this, var_info);
}

#if AP_ESNCONTROLLER_ENABLED
float AP_PitchController::esn_update(float rate_err, float desired_rate,
                                     float measured_rate, float scaler)
{
    // ── Статики ──
    static uint32_t last_osc_ms          = 0;
    static bool     has_oscillated       = false;
    static bool     prev_was_osc         = false;
    static float    raw_err_filt         = 0.0f;
    static int8_t   prev_des_sign        = 0;
    static uint32_t des_flip_times[5]    = {0};
    static uint8_t  des_flip_idx         = 0;
    static uint16_t small_err_count      = 0;
    static float    slow_err             = 0.0f;
    static float    lp_gyro              = 0.0f;
    static uint16_t train_div            = 0;
    static uint32_t last_creep_ms        = 0;
    static uint32_t last_forced_train_ms = 0;
    static uint32_t soft_pio_since_ms    = 0;
    static uint32_t esn_dbg_ms           = 0;
    static float    lfo_envelope         = 0.0f;
    static uint32_t last_sign_change_ms  = 0;
    static float    zcr_frequency        = 0.0f;
    static uint32_t deadlock_since_ms    = 0;

    // in_deadlock объявлен здесь, чтобы быть в scope для printf в конце
    bool in_deadlock = false;

    // ── Константы ──
    constexpr float    SCALE_FLOOR_NORMAL       = 0.25f;
    constexpr float    SCALE_FLOOR_DEADLOCK     = 0.50f;
    constexpr float    SCALE_FLOOR_SOFT         = 0.50f;
    constexpr float    PIO_VIB_THRESH           = 0.25f;
    constexpr float    HARD_ALT_M               = 10.0f;
    constexpr float    HARD_SPEED_FACTOR        = 1.2f;
    constexpr float    SOFT_ALT_M               = 50.0f;
    constexpr float    SOFT_SPEED_FACTOR        = 1.5f;
    constexpr uint32_t FORCE_TRAIN_INTERVAL_MS  = 25;
    constexpr float    ERR_OSC                  = 5.0f;
    constexpr float    CREEP_STEP               = 0.001f;
    constexpr uint32_t COOLDOWN_MS              = 4000;
    constexpr uint16_t SMALL_ERR_COUNT_THRESHOLD = 200;
    constexpr uint32_t SOFT_PIO_CONFIRM_MS      = 500;
    constexpr float    SOFT_RATE_LIMIT_START    = 40.0f;
    constexpr float    SOFT_RATE_LIMIT_FLOOR    = 20.0f;
    constexpr uint32_t SOFT_RATE_REDUCE_MS      = 2000;
    constexpr uint32_t SOFT_SCALE_CREEP_MS      = 200;
    constexpr uint32_t SOFT_SEVERE_AFTER_MS     = 3000;
    constexpr uint32_t DEADLOCK_CONFIRM_MS      = 500;
    constexpr float    DEADLOCK_RELEASE_STEP    = 10.0f;
    constexpr float    DEADLOCK_RATE_MAX        = 60.0f;

    // ─── RESET ───
    if (_esn_reset_param.get() != 0) {
        _esn_reset_param.set(0);
        _esn_initialized = false;
        _filt_abs_err_pitch = 0.0f;
        _prev_raw_err = 0.0f;
        _zcr = 0;
        _safe_scale_P = 1.0f; _safe_scale_D = 1.0f;
        _esn_scale_P  = 1.0f; _esn_scale_D  = 1.0f;
        _sluggish_mult = 1.0f;
        _adaptive_rate_limit = 0.0f;
        _adaptive_rate_limit_hold = 0.0f;
        _last_rate_reduction_ms = 0;
        _maneuver_in_progress = false;
        _desired_dir = 0;
        _desired_dir_since_ms = 0;
        _filt_vib = 0.0f;
        _pio_detected = false;
        last_osc_ms = 0; has_oscillated = false; prev_was_osc = false;
        raw_err_filt = 0.0f; prev_des_sign = 0;
        memset(des_flip_times, 0, sizeof(des_flip_times)); des_flip_idx = 0;
        small_err_count = 0; slow_err = 0.0f; lp_gyro = 0.0f;
        train_div = 0; last_creep_ms = 0; last_forced_train_ms = 0;
        soft_pio_since_ms = 0;
        lfo_envelope = 0.0f; last_sign_change_ms = 0; zcr_frequency = 0.0f;
        deadlock_since_ms = 0;
        in_deadlock = false;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ESN Pitch: reset requested");
    }

    // ─── INIT ───
    if (!_esn_initialized) {
        _esn_P.init(12, 70);
        _esn_D.init(12, 70);
        _base_kP = rate_pid.kP();
        _base_kD = rate_pid.kD();
        _base_kI = rate_pid.kI();

        float* pretrain_data = nullptr;
        const char* pretrain_file = "/home/station106/ardupilot/esn_pretrain.csv";
        uint16_t n_samples = _esn_P.load_pretrain_data(pretrain_file, &pretrain_data);
        if (n_samples > 0) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ESN Pitch: Pretraining on %d samples", n_samples);
            _esn_P.pretrain(pretrain_data, n_samples, 0.001f, 0.0f, 1.0f, 5);
            _esn_D.pretrain(pretrain_data, n_samples, 0.001f, 0.0f, 1.0f, 5);
            delete[] pretrain_data;
        }
        _esn_initialized = true;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ESN Pitch: init done, base_kP=%.4f", (double)_base_kP);
    }

    // ─── GUARD-УРОВНИ ───
    const float eas_now = get_airspeed();
    float alt_now = 0.0f;
    AP::ahrs().get_relative_position_D_home(alt_now);

    bool hard_guard = (-alt_now < HARD_ALT_M) ||
                      (eas_now < (aparm.airspeed_min * HARD_SPEED_FACTOR));
    bool soft_guard = (-alt_now < SOFT_ALT_M) ||
                      (eas_now < (aparm.airspeed_min * SOFT_SPEED_FACTOR));

    // ─── HARD GUARD ───
    if (hard_guard) {
        _safe_scale_P = 1.0f; _safe_scale_D = 1.0f;
        _esn_scale_P  = 1.0f; _esn_scale_D  = 1.0f;
        _sluggish_mult = 1.0f;
        _adaptive_rate_limit = 0.0f;
        _adaptive_rate_limit_hold = 0.0f;
        _last_rate_reduction_ms = 0;
        _maneuver_in_progress = false;
        _desired_dir = 0;
        _desired_dir_since_ms = 0;
        _zcr = 0;
        _filt_vib = 0.0f;
        _pio_detected = false;
        _prev_raw_err = 0.0f;
        last_osc_ms = 0; has_oscillated = false; prev_was_osc = false;
        raw_err_filt = 0.0f; prev_des_sign = 0;
        memset(des_flip_times, 0, sizeof(des_flip_times)); des_flip_idx = 0;
        small_err_count = 0; slow_err = 0.0f; lp_gyro = 0.0f;
        train_div = 0; last_creep_ms = 0; last_forced_train_ms = 0;
        soft_pio_since_ms = 0;
        lfo_envelope = 0.0f; last_sign_change_ms = 0; zcr_frequency = 0.0f;
        deadlock_since_ms = 0;

        rate_pid.kP().set(_base_kP);
        rate_pid.kD().set(_base_kD);
        rate_pid.kI().set(_base_kI);
        return 1.0f;
    }

    // ─── ОБЩИЕ ДЕТЕКТОРЫ ───
    const Vector3f gyro = AP::ahrs().get_gyro();
    const float safe_rate_err = constrain_float(rate_err, -3.0f, 3.0f);

    lp_gyro = 0.95f * lp_gyro + 0.05f * gyro.y;
    float hp_gyro = gyro.y - lp_gyro;
    if (fabsf(hp_gyro) > 1.0f) hp_gyro = 0.0f;
    _filt_vib = 0.98f * _filt_vib + 0.02f * fabsf(hp_gyro);

    // Огибающая ошибки для LFO
    float abs_err_now = fabsf(rate_err);
    if (abs_err_now > lfo_envelope) {
        lfo_envelope = 0.8f * lfo_envelope + 0.2f * abs_err_now;
    } else {
        lfo_envelope = 0.99f * lfo_envelope + 0.01f * abs_err_now;
    }

    uint32_t now_ms = AP_HAL::millis();
    if (last_sign_change_ms == 0) last_sign_change_ms = now_ms;

    // ZCR и частота смены знака
    if (fabsf(safe_rate_err) > 0.5f) {
        if (safe_rate_err * _prev_raw_err < 0.0f) {
            _zcr++;
            uint32_t dt = now_ms - last_sign_change_ms;
            last_sign_change_ms = now_ms;

            if (dt > 50 && dt < 500) {
                zcr_frequency = 0.9f * zcr_frequency + 0.1f * (1000.0f / dt);
            } else {
                zcr_frequency *= 0.9f;
            }
        }
    }
    _prev_raw_err = safe_rate_err;

    // sustained_ms для детекторов
    int8_t curr_dir = (_last_unclamped_desired_rate > 10.0f) ? 1
                    : ((_last_unclamped_desired_rate < -10.0f) ? -1 : 0);
    if (curr_dir != 0 && curr_dir == _desired_dir) {
        // same
    } else if (curr_dir != 0) {
        _desired_dir = curr_dir;
        _desired_dir_since_ms = AP_HAL::millis();
    } else {
        _desired_dir = 0;
        _desired_dir_since_ms = 0;
    }
    uint32_t sustained_ms = (_desired_dir_since_ms > 0)
                            ? (AP_HAL::millis() - _desired_dir_since_ms) : 0;

    raw_err_filt = 0.995f * raw_err_filt + 0.005f * fabsf(rate_err);

    int8_t des_sign = (_last_unclamped_desired_rate > 15.0f) ? 1
                     : ((_last_unclamped_desired_rate < -15.0f) ? -1 : 0);
    if (des_sign != 0 && prev_des_sign != 0 && des_sign != prev_des_sign) {
        des_flip_times[des_flip_idx] = AP_HAL::millis();
        des_flip_idx = (des_flip_idx + 1) % 5;
    }
    if (des_sign != 0) prev_des_sign = des_sign;

    uint32_t oldest_flip = des_flip_times[des_flip_idx];
    uint32_t newest_flip = des_flip_times[(des_flip_idx + 4) % 5];
    bool rapid_flip = (oldest_flip != 0 && newest_flip != 0 &&
                       (newest_flip - oldest_flip) < 2000);

    // ─── ДЕТЕКТОР RATE LIMIT DEADLOCK ───
    bool rate_limit_active = (_adaptive_rate_limit > 0.0f);
    bool rate_limit_dominating = rate_limit_active &&
                                  (fabsf(_last_unclamped_desired_rate) > _adaptive_rate_limit * 1.5f) &&
                                  (sustained_ms > 500) &&
                                  (_zcr < 5) &&
                                  !rapid_flip;

    if (rate_limit_dominating) {
        if (deadlock_since_ms == 0) deadlock_since_ms = now_ms;
    } else {
        deadlock_since_ms = 0;
    }
    in_deadlock = (deadlock_since_ms != 0) &&
                  (now_ms - deadlock_since_ms > DEADLOCK_CONFIRM_MS);

    // При deadlock — отпускаем rate limit и восстанавливаем safe_scale
    if (in_deadlock) {
        _adaptive_rate_limit = MIN(_adaptive_rate_limit + DEADLOCK_RELEASE_STEP, DEADLOCK_RATE_MAX);
        if (_adaptive_rate_limit >= DEADLOCK_RATE_MAX) _adaptive_rate_limit = 0.0f;
        _maneuver_in_progress = true;
        _safe_scale_P = MIN(_safe_scale_P + 0.02f, 1.0f);
        _safe_scale_D = MIN(_safe_scale_D + 0.02f, 1.0f);
        _pio_detected = false;
        if (AP_HAL::millis() - esn_dbg_ms >= 1000) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING,
                          "ESN Pitch: DEADLOCK, releasing rate limit %.0f",
                          _adaptive_rate_limit);
        }
    }

    // ─── LFO / OSC ───
    bool lfo_is_rate_limit = rate_limit_active && (lfo_envelope < _adaptive_rate_limit * 1.5f);
    bool is_lfo_osc = (lfo_envelope > 5.0f) &&
                      (zcr_frequency > 1.0f) &&
                      (zcr_frequency < 6.0f) &&
                      !lfo_is_rate_limit &&
                      !in_deadlock;
    bool osc_detected = ((_zcr > 5) || (_filt_vib > PIO_VIB_THRESH) || rapid_flip || is_lfo_osc)
                        && !in_deadlock;
    bool sustained_one_direction = (sustained_ms > 1000) &&
                                    (fabsf(_last_unclamped_desired_rate) > 15.0f) &&
                                    (_zcr < 3) &&
                                    !is_lfo_osc;

    // ─── ESN INFERENCE ───
    const float groundspeed = AP::ahrs().groundspeed();
    const float angle_err_local = isfinite(angle_err_deg)
                                    ? constrain_float(angle_err_deg, -180.0f, 180.0f)
                                    : 0.0f;
    const float throttle_norm = constrain_float(
        SRV_Channels::get_output_norm(SRV_Channel::k_throttle), 0.0f, 1.0f);

    // В Pitch measured_rate = degrees(gyro.y), поэтому gyro.y был бы дублем.
    // Используем gyro.x как cross-axis (крен при рулении тангажом).
    float features_pitch[12] = {
        rate_err, angle_err_local, desired_rate, measured_rate,
        throttle_norm, gyro.x, gyro.z,
        groundspeed, alt_now, scaler,
        _esn_scale_P, _base_kP
    };

    float corr_P = _esn_P.update(features_pitch);

    // D-сеть получает свой контекст: scale_D и base_kD
    features_pitch[10] = _esn_scale_D;
    features_pitch[11] = _base_kD;
    float corr_D = _esn_D.update(features_pitch);

    corr_P = constrain_float(corr_P, -2.0f, 0.3f);
    corr_D = constrain_float(corr_D, -2.0f, 0.3f);

    // ─── SOFT GUARD ───
    if (soft_guard) {
        bool soft_pio_candidate = osc_detected &&
                                  !sustained_one_direction &&
                                  (raw_err_filt > 5.0f) &&
                                  !in_deadlock;
        if (soft_pio_candidate) {
            if (soft_pio_since_ms == 0) soft_pio_since_ms = AP_HAL::millis();
        } else {
            soft_pio_since_ms = 0;
        }
        bool real_pio = (soft_pio_since_ms != 0) &&
                        (AP_HAL::millis() - soft_pio_since_ms > SOFT_PIO_CONFIRM_MS);

        if (real_pio && !in_deadlock) {
            if (_adaptive_rate_limit <= 0.0f) {
                _adaptive_rate_limit = SOFT_RATE_LIMIT_START;
                _last_rate_reduction_ms = AP_HAL::millis();
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING,
                              "ESN Pitch(soft): real PIO in takeoff, rate=%.0f",
                              _adaptive_rate_limit);
            } else if (AP_HAL::millis() - _last_rate_reduction_ms > SOFT_RATE_REDUCE_MS) {
                if (_adaptive_rate_limit > SOFT_RATE_LIMIT_FLOOR) {
                    _adaptive_rate_limit = MAX(_adaptive_rate_limit - 5.0f, SOFT_RATE_LIMIT_FLOOR);
                    _last_rate_reduction_ms = AP_HAL::millis();
                    GCS_SEND_TEXT(MAV_SEVERITY_WARNING,
                                  "ESN Pitch(soft): rate limit -> %.0f", _adaptive_rate_limit);
                }
            }
            _adaptive_rate_limit_hold = _adaptive_rate_limit;
        }

        bool very_severe = real_pio && !in_deadlock &&
                           (AP_HAL::millis() - soft_pio_since_ms > SOFT_SEVERE_AFTER_MS) &&
                           (_adaptive_rate_limit <= SOFT_RATE_LIMIT_FLOOR);
        if (very_severe && (AP_HAL::millis() - last_creep_ms >= SOFT_SCALE_CREEP_MS)) {
            last_creep_ms = AP_HAL::millis();
            if (_safe_scale_P > SCALE_FLOOR_SOFT) {
                _safe_scale_P = MAX(_safe_scale_P - 0.01f, SCALE_FLOOR_SOFT);
                _safe_scale_D = MAX(_safe_scale_D - 0.01f, SCALE_FLOOR_SOFT);
            }
        }

        float effective_P = _safe_scale_P;
        float effective_D = _safe_scale_D;

        _esn_scale_P = effective_P;
        _esn_scale_D = effective_D;
        rate_pid.kP().set(_base_kP * effective_P);
        rate_pid.kD().set(MAX(_base_kD * effective_D, 0.003f));
        rate_pid.kI().set(_base_kI * effective_P);

        if (AP_HAL::millis() - esn_dbg_ms >= 1000) {
            esn_dbg_ms = AP_HAL::millis();
            hal.console->printf("ESN Pitch(soft): alt=%.1f eas=%.1f vib=%.2f zcr=%d lfo=%d dl=%d real_pio=%d scP=%.2f safeP=%.2f\n",
                                (double)(-alt_now), (double)eas_now, (double)_filt_vib,
                                (int)_zcr, (int)is_lfo_osc, (int)in_deadlock, (int)real_pio,
                                (double)effective_P, (double)_safe_scale_P);
        }
        return 1.0f;
    }

    // ─── NORMAL ───
    bool pio_now = (raw_err_filt > 5.0f) && osc_detected && !in_deadlock;
    if (sustained_one_direction) pio_now = false;

    if (pio_now && !_pio_detected) {
        _pio_detected = true;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ESN Pitch: PIO (avg=%.1f zcr=%d vib=%.2f lfo=%d)",
                      (double)raw_err_filt, (int)_zcr, (double)_filt_vib, (int)is_lfo_osc);
    } else if (!pio_now && _pio_detected && raw_err_filt < 3.0f && !rapid_flip) {
        _pio_detected = false;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ESN Pitch: PIO cleared");
    }

    if (fabsf(rate_err) < 15.0f) {
        if (small_err_count < SMALL_ERR_COUNT_THRESHOLD) small_err_count++;
    } else {
        if (small_err_count > 0) small_err_count--;
    }
    bool sustained_small_err = (small_err_count >= SMALL_ERR_COUNT_THRESHOLD);

    slow_err = 0.995f * slow_err + 0.005f * fabsf(safe_rate_err);

    // Выбор пола: при deadlock — выше (0.50), иначе — обычный (0.25)
    float floor_effective = in_deadlock ? SCALE_FLOOR_DEADLOCK : SCALE_FLOOR_NORMAL;
    float upper_P = MAX(_safe_scale_P, floor_effective);
    float upper_D = MAX(_safe_scale_D, floor_effective);

    float new_scale_P = expf(corr_P);
    float new_scale_D = expf(corr_D);

    new_scale_P = MIN(new_scale_P, upper_P);
    new_scale_P = MAX(new_scale_P, floor_effective);
    new_scale_D = MIN(new_scale_D, upper_D);
    new_scale_D = MAX(new_scale_D, floor_effective);

    const float max_delta = 0.05f;
    new_scale_P = constrain_float(new_scale_P, _esn_scale_P - max_delta, _esn_scale_P + max_delta);
    new_scale_D = constrain_float(new_scale_D, _esn_scale_D - max_delta, _esn_scale_D + max_delta);
    new_scale_P = MAX(new_scale_P, floor_effective);
    new_scale_D = MAX(new_scale_D, floor_effective);

    _esn_scale_P = new_scale_P;
    _esn_scale_D = new_scale_D;
    rate_pid.kP().set(_base_kP * new_scale_P);
    rate_pid.kD().set(MAX(_base_kD * new_scale_D, 0.003f));
    rate_pid.kI().set(_base_kI * new_scale_P);

    bool sustained_maneuver = (sustained_ms > 500) &&
                              (fabsf(_last_unclamped_desired_rate) > 30.0f) &&
                              (sustained_small_err || (_zcr < 3) || in_deadlock) &&
                              (!_pio_detected || in_deadlock) &&
                              (_filt_vib < PIO_VIB_THRESH) &&
                              (!is_lfo_osc || in_deadlock);
    bool is_maneuver = sustained_maneuver;

    bool force_train = (_pio_detected || (_filt_vib > PIO_VIB_THRESH) || is_lfo_osc) &&
                       (AP_HAL::millis() - last_forced_train_ms >= FORCE_TRAIN_INTERVAL_MS);
    if (force_train) last_forced_train_ms = AP_HAL::millis();
    bool do_train = force_train || (++train_div >= 8);
    if (do_train) train_div = 0;

    if (do_train) {
        bool is_osc = (osc_detected || _pio_detected) && !is_maneuver && !in_deadlock;
        float target_P, target_D;

        if (is_maneuver) {
            target_P = 0.0f; target_D = 0.0f;
            if (_safe_scale_P < 1.0f) {
                _safe_scale_P = MIN(_safe_scale_P + 0.02f, 1.0f);
                _safe_scale_D = MIN(_safe_scale_D + 0.02f, 1.0f);
            }
            prev_was_osc = false;
        } else if (is_osc) {
            if (!prev_was_osc) {
                _safe_scale_P *= 0.8f;
                _safe_scale_D *= 0.8f;
            }
            prev_was_osc = true;
            has_oscillated = true;
            last_osc_ms = AP_HAL::millis();
            target_P = -1.0f; target_D = -1.0f;
        } else {
            prev_was_osc = false;
            bool cooldown_active = has_oscillated && (AP_HAL::millis() - last_osc_ms < COOLDOWN_MS);
            if (cooldown_active) {
                target_P = logf(_esn_scale_P);
                target_D = logf(_esn_scale_D);
            } else {
                target_P = 0.0f; target_D = 0.0f;
            }
        }
        _safe_scale_P = constrain_float(_safe_scale_P, 0.05f, 1.0f);
        _safe_scale_D = constrain_float(_safe_scale_D, 0.05f, 1.0f);
        _esn_P.train(constrain_float(target_P, -1.0f, 0.3f));
        _esn_D.train(constrain_float(target_D, -1.0f, 0.3f));
    }

    if (AP_HAL::millis() - last_creep_ms >= 100) {
        last_creep_ms = AP_HAL::millis();
        if (slow_err < (ERR_OSC * 0.2f)) _zcr = 0;
        _zcr = (_zcr > 0) ? (_zcr - 1) : 0;

        // При deadlock — восстанавливаем safe_scale быстрее и без условий
        if (in_deadlock) {
            _safe_scale_P = MIN(_safe_scale_P + 0.03f, 1.0f);
            _safe_scale_D = MIN(_safe_scale_D + 0.03f, 1.0f);
            _sluggish_mult = 1.0f;
        } else {
            bool is_osc_10hz = (_zcr > 5) || (_filt_vib > PIO_VIB_THRESH) || _pio_detected || is_lfo_osc;
            if (is_osc_10hz || is_maneuver) {
                _sluggish_mult = 1.0f;
            } else if (slow_err > (ERR_OSC * 0.5f) && _safe_scale_P < 1.0f) {
                _sluggish_mult += 0.3f;
                _sluggish_mult = MIN(_sluggish_mult, 3.0f);
                _safe_scale_P += CREEP_STEP * _sluggish_mult;
                _safe_scale_D += CREEP_STEP * _sluggish_mult;
            } else {
                _sluggish_mult = 1.0f;
            }
        }
    }

    constexpr float    RATE_LIMIT_START    = 60.0f;
    constexpr float    RATE_LIMIT_FLOOR    = 15.0f;
    constexpr float    RATE_LIMIT_STEP     = 5.0f;
    constexpr uint32_t RATE_LIMIT_REDUCE_INTERVAL_MS = 3000;

    bool low_freq_osc = _pio_detected && !is_maneuver && !in_deadlock;
    if (low_freq_osc) {
        if (_adaptive_rate_limit <= 0.0f) {
            _adaptive_rate_limit = RATE_LIMIT_START;
            _last_rate_reduction_ms = now_ms;
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ESN Pitch: rate limit START at %.0f", _adaptive_rate_limit);
        } else if (now_ms - _last_rate_reduction_ms > RATE_LIMIT_REDUCE_INTERVAL_MS) {
            if (_adaptive_rate_limit > RATE_LIMIT_FLOOR) {
                _adaptive_rate_limit = MAX(_adaptive_rate_limit - RATE_LIMIT_STEP, RATE_LIMIT_FLOOR);
                _last_rate_reduction_ms = now_ms;
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ESN Pitch: rate limit -> %.0f", _adaptive_rate_limit);
            }
        }
        _adaptive_rate_limit_hold = _adaptive_rate_limit;
    }

    if (is_maneuver || in_deadlock) {
        _maneuver_in_progress = true;
    } else if (sustained_ms < 200 && fabsf(_last_unclamped_desired_rate) < _adaptive_rate_limit * 0.7f) {
        _maneuver_in_progress = false;
    }

    if (AP_HAL::millis() - esn_dbg_ms >= 1000) {
        esn_dbg_ms = AP_HAL::millis();
        hal.console->printf("ESN Pitch: raw=%.1f ae=%.1f thr=%.2f zcr=%d f=%.1f env=%.1f pio=%d osc=%d lfo=%d sod=%d m=%d dl=%d scP=%.2f safeP=%.2f rl=%.0f\n",
                            (double)raw_err_filt,
                            (double)angle_err_local,
                            (double)throttle_norm,
                            (int)_zcr,
                            (double)zcr_frequency,
                            (double)lfo_envelope,
                            (int)_pio_detected,
                            (int)osc_detected,
                            (int)is_lfo_osc,
                            (int)sustained_one_direction,
                            (int)is_maneuver,
                            (int)in_deadlock,
                            (double)_esn_scale_P,
                            (double)_safe_scale_P,
                            (double)_adaptive_rate_limit);
    }

    log_esn_data(1, new_scale_P, new_scale_D, _base_kP, _base_kD, rate_err, 0, 0, corr_P, 0, false);
    return 1.0f;
}
#endif

float AP_PitchController::get_measured_angle_deg() const
{
    return AP::ahrs().get_pitch_deg();
}

float AP_PitchController::get_measured_rate_rads() const
{
    return AP::ahrs().get_gyro().y;
}

bool AP_PitchController::is_underspeed() const
{
    return get_airspeed() <= 0.5*float(aparm.airspeed_min);
}

bool AP_PitchController::is_inverted() const
{
    return fabsf(AP::ahrs().get_roll_deg()) >= 90.0;
}

float AP_PitchController::get_positive_rate_limit_degs() const
{
    float base_limit = MAX(gains.rmax_pos.get(), 0.0f);

    #if AP_ESNCONTROLLER_ENABLED
        if (_esn_initialized && _adaptive_rate_limit > 0.0f) {
            if (_maneuver_in_progress) {
                return (base_limit > 0.0f) ? base_limit : _adaptive_rate_limit * 2.0f;
            }
            if (base_limit > 0.0f) {
                return MIN(base_limit, _adaptive_rate_limit);
            }
            return _adaptive_rate_limit;
        }
    #endif

    return base_limit;
}

float AP_PitchController::get_negative_rate_limit_degs() const
{
    float base_limit = MAX(gains.rmax_neg.get(), 0.0f);

    #if AP_ESNCONTROLLER_ENABLED
        if (_esn_initialized && _adaptive_rate_limit > 0.0f) {
            if (_maneuver_in_progress) {
                return (base_limit > 0.0f) ? base_limit : _adaptive_rate_limit * 2.0f;
            }
            if (base_limit > 0.0f) {
                return MIN(base_limit, _adaptive_rate_limit);
            }
            return _adaptive_rate_limit;
        }
    #endif

    return base_limit;
}

bool AP_PitchController::should_apply_rate_limits() const
{
    return !is_inverted();
}

float AP_PitchController::get_rate_target_offset_degs() const
{
    const AP_AHRS &_ahrs = AP::ahrs();
    float bank_angle = _ahrs.get_roll_rad();

    if (!is_inverted()) {
        bank_angle = constrain_float(bank_angle, -radians(80), radians(80));
    } else {
        bank_angle = constrain_float(wrap_2PI(bank_angle), radians(100), radians(260));
    }
    if (abs(_ahrs.pitch_sensor) > 7000) {
        return 0.0;
    }

    const float true_airspeed = MAX((get_airspeed() * _ahrs.get_EAS2TAS()), MAX(aparm.airspeed_min, 1));
    const float lateral_accel = tanf(bank_angle) * GRAVITY_MSS * cosf(_ahrs.get_pitch_rad());
    const float turn_rate = (lateral_accel / true_airspeed) * sinf(bank_angle);
    float rate_offset = fabsf(degrees(turn_rate)) * _roll_ff;

    return rate_offset;
}

float AP_PitchController::run_axis_rate_control(float desired_rate_degs, float scaler, bool disable_integrator, bool ground_mode)
{
    if (is_inverted()) {
        desired_rate_degs *= -1.0;
    }

    const AP_AHRS &_ahrs = AP::ahrs();
    float roll_wrapped = labs(_ahrs.roll_sensor);
    if (roll_wrapped > 9000) {
        roll_wrapped = 18000 - roll_wrapped;
    }
    const float roll_limit_margin = MIN(aparm.roll_limit*100 + 500.0, 8500.0);
    if (roll_wrapped > roll_limit_margin && labs(_ahrs.pitch_sensor) < 7000) {
        float roll_prop = (roll_wrapped - roll_limit_margin) / (float)(9000 - roll_limit_margin);
        desired_rate_degs *= (1 - roll_prop);
    }

    return run_rate_control(desired_rate_degs, scaler, disable_integrator, ground_mode);
}