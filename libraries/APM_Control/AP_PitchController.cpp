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
    // Сеть для P: префикс _ESN_
    AP_SUBGROUPINFO(_esn_P, "_ESN_", 14, AP_PitchController, ESNController),
    AP_GROUPINFO("_ESN_RESET", 15, AP_PitchController, _esn_reset_param, 0),
    // Сеть для D: префикс _ESND_
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
    static uint32_t last_osc_ms = 0;
    static bool has_oscillated = false;

    if (_esn_reset_param.get() != 0) {
        _esn_reset_param.set(0);
        _esn_initialized = false;
        _filt_abs_err_pitch = 0.0f;
        _prev_raw_err = 0.0f;
        _zcr = 0;
        _safe_scale_P = 1.0f;
        _safe_scale_D = 1.0f;
        _sluggish_mult = 1.0f;
        last_osc_ms = 0;
        has_oscillated = false;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ESN Pitch: reset requested");
    }

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

    const Vector3f gyro = AP::ahrs().get_gyro();
    const float safe_rate_err = constrain_float(rate_err, -3.0f, 3.0f);

    // ─── ДЕТЕКТОРЫ (Pitch = gyro.y) ───
    static float lp_gyro = 0.0f;
    lp_gyro = 0.95f * lp_gyro + 0.05f * gyro.y;
    float hp_gyro = gyro.y - lp_gyro;
    if (fabsf(hp_gyro) > 1.0f) hp_gyro = 0.0f;
    if (fabsf(desired_rate) > 25.0f || fabsf(measured_rate) > 25.0f) {
        hp_gyro = 0.0f; 
    }
    _filt_vib = 0.98f * _filt_vib + 0.02f * fabsf(hp_gyro);

    if (fabsf(safe_rate_err) > 0.3f) {  // Было 0.5
        if (safe_rate_err * _prev_raw_err < 0.0f) _zcr++;
    }
    _prev_raw_err = safe_rate_err;

    const float PIO_THRESHOLD = 40.0f;
    const uint32_t PIO_TIME_MS = 500;
    static uint32_t pio_timer = 0;
    static int8_t pio_sign_changes = 0;
    static int8_t pio_prev_sign = 0;

    if (fabsf(rate_err) > PIO_THRESHOLD) {
        if (pio_timer == 0) { pio_timer = AP_HAL::millis(); pio_sign_changes = 0; }
        int8_t sign = (rate_err > 0) ? 1 : -1;
        if (pio_prev_sign != 0 && sign != pio_prev_sign) pio_sign_changes++;
        pio_prev_sign = sign;
        if (AP_HAL::millis() - pio_timer > PIO_TIME_MS && pio_sign_changes >= 2) _pio_detected = true;
    } else if (AP_HAL::millis() - pio_timer > 3000) {
        pio_timer = 0; pio_sign_changes = 0; pio_prev_sign = 0; _pio_detected = false;
    }

    static int8_t prev_desired_sign = 0;
    static int16_t desired_zcr = 0;
    int8_t curr_desired_sign = (desired_rate > 5.0f) ? 1 : ((desired_rate < -5.0f) ? -1 : 0);
    if (curr_desired_sign != 0 && prev_desired_sign != 0 && curr_desired_sign != prev_desired_sign) desired_zcr++;
    if (curr_desired_sign != 0) prev_desired_sign = curr_desired_sign;

    static float slow_err = 0.0f;
    slow_err = 0.995f * slow_err + 0.005f * fabsf(safe_rate_err);

    float airspeed = AP::ahrs().groundspeed();
    float alt = 0.0f;
    AP::ahrs().get_relative_position_D_home(alt);

    float features_pitch[12] = {
        rate_err, 0, desired_rate, measured_rate,
        gyro.x, gyro.y, gyro.z,
        airspeed, alt, scaler,
        _esn_scale_P, _base_kP
    };

    float corr_P = _esn_P.update(features_pitch);
    float corr_D = _esn_D.update(features_pitch);
    corr_P = constrain_float(corr_P, -3.0f, 0.3f);
    corr_D = constrain_float(corr_D, -3.0f, 0.3f);

    float new_scale_P = expf(corr_P);
    float new_scale_D = expf(corr_D);

    new_scale_P = constrain_float(new_scale_P, 0.02f, _safe_scale_P);
    new_scale_D = constrain_float(new_scale_D, 0.02f, _safe_scale_D);

    const float max_delta = 0.05f;
    new_scale_P = constrain_float(new_scale_P, _esn_scale_P - max_delta, _esn_scale_P + max_delta);
    new_scale_D = constrain_float(new_scale_D, _esn_scale_D - max_delta, _esn_scale_D + max_delta);

    _esn_scale_P = new_scale_P;
    _esn_scale_D = new_scale_D;

    rate_pid.kP().set(_base_kP * new_scale_P);
    rate_pid.kD().set(MAX(_base_kD * new_scale_D, 0.003f));
    rate_pid.kI().set(_base_kI * new_scale_P);

    constexpr float ERR_OSC = 5.0f;
    constexpr float CREEP_STEP = 0.001f;
    constexpr uint32_t COOLDOWN_MS = 4000;

    bool is_maneuver = (fabsf(desired_rate) > 25.0f) || (fabsf(measured_rate) > 25.0f);

    static uint16_t train_div = 0;
    bool do_train = (++train_div >= 8);
    if (do_train) train_div = 0;
    // Учимся КАЖДЫЙ такт при вибрации, PIO или большой ошибке
    if ((fabsf(safe_rate_err) > 2.0f) || _pio_detected || (_filt_vib > 0.15f) || (_zcr > 5)) {
        do_train = true;
    }

    if (do_train) {
        bool is_low_alt = (-alt < 20.0f) || (get_airspeed() < (aparm.airspeed_min * 1.2f));
        
        // is_osc — только по PIO, ZCR или вибрации. is_maneuver НЕ блокирует срезание!
        bool is_osc = ((_zcr > 5) || (_filt_vib > 0.15f) || _pio_detected) && !is_low_alt;
        
        // "Форсаж": если PIO длится > 0.5 сек, срезаем принудительно до минимума
        static uint32_t osc_start_ms = 0;
        if (is_osc) {
            if (osc_start_ms == 0) osc_start_ms = AP_HAL::millis();
        } else {
            osc_start_ms = 0;
        }
        bool osc_long = (osc_start_ms > 0) && (AP_HAL::millis() - osc_start_ms > 500);

        float target_P, target_D;

        if (is_low_alt) {
            // Взлет: полные пиды
            target_P = 0.0f; target_D = 0.0f;
            _safe_scale_P = MAX(_safe_scale_P, 1.0f);
            _safe_scale_D = MAX(_safe_scale_D, 1.0f);
        } else if (is_osc) {
            // Раскачка: ЖЕСТКО режем (даже если это выглядит как маневр)
            has_oscillated = true;
            last_osc_ms = AP_HAL::millis();
            
            if (osc_long) {
                // Затяжной PIO: срезаем до минимума
                target_P = -5.0f; target_D = -5.0f;
                _safe_scale_P = 0.05f;
                _safe_scale_D = 0.05f;
            } else {
                // Обычный PIO: быстро режем
                target_P = -3.0f; target_D = -3.0f;
                _safe_scale_P *= 0.7f;
                _safe_scale_D *= 0.7f;
            }
        } else {
            bool cooldown_active = has_oscillated && (AP_HAL::millis() - last_osc_ms < COOLDOWN_MS);
            if (is_maneuver || cooldown_active) {
                // Заморозка скейла (не режем, не растим)
                target_P = logf(_esn_scale_P);
                target_D = logf(_esn_scale_D);
            } else {
                // Расслабление: медленно возвращаем к 1.0
                target_P = 0.0f; target_D = 0.0f;
            }
        }

        _safe_scale_P = constrain_float(_safe_scale_P, 0.05f, 1.0f);
        _safe_scale_D = constrain_float(_safe_scale_D, 0.05f, 1.0f);
        _esn_P.train(constrain_float(target_P, -5.0f, 0.3f));
        _esn_D.train(constrain_float(target_D, -5.0f, 0.3f));
    }

    static uint32_t last_creep_ms = 0;
    if (AP_HAL::millis() - last_creep_ms >= 100) {
        last_creep_ms = AP_HAL::millis();

        if (slow_err < (ERR_OSC * 0.2f)) _zcr = 0;
        _zcr = (_zcr > 2) ? (_zcr - 2) : 0;
        if (desired_zcr > 0) desired_zcr--;

        bool is_low_alt = (-alt < 20.0f) || (get_airspeed() < (aparm.airspeed_min * 1.2f));
        bool is_osc_10hz = (!is_low_alt) && ((_zcr > 5) || (_filt_vib > 0.15f) || _pio_detected);
        bool cooldown_active = has_oscillated && (AP_HAL::millis() - last_osc_ms < COOLDOWN_MS);

        if (is_osc_10hz || is_low_alt || is_maneuver || cooldown_active) {
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

    static uint32_t esn_dbg_ms = 0;
    if (AP_HAL::millis() - esn_dbg_ms >= 1000) {
        esn_dbg_ms = AP_HAL::millis();
        bool cd = has_oscillated && (AP_HAL::millis() - last_osc_ms < 4000);
        hal.console->printf("ESN Pitch: err=%.2f zcr=%d pio=%d m=%d cd=%d scP=%.2f safeP=%.2f\n",
                            (double)slow_err, (int)_zcr, (int)_pio_detected,
                            (int)is_maneuver, (int)cd,
                            (double)_esn_scale_P, (double)_safe_scale_P);
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
    return MAX(gains.rmax_pos.get(), 0.0);
}

float AP_PitchController::get_negative_rate_limit_degs() const
{
    return MAX(gains.rmax_neg.get(), 0.0);
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