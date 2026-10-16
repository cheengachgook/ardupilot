/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
 */

//	Code by Jon Challinger
//  Modified by Paul Riseborough
//

#include <AP_HAL/AP_HAL.h>
#include "AP_RollController.h"
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Scheduler/AP_Scheduler.h>
#include <AP_Airspeed/AP_Airspeed.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_RollController::var_info[] = {
    AP_GROUPINFO("2SRV_TCONST",      0, AP_RollController, gains.tau,       0.5f),
    AP_GROUPINFO("2SRV_RMAX",        4, AP_RollController, gains.rmax_pos,  0),
    AP_SUBGROUPINFO(rate_pid, "_RATE_", 9, AP_RollController, AC_PID),
    AP_GROUPINFO("2SRV_ACCEL",      10, AP_RollController, accel_limit, 500),
    AP_GROUPINFO("_ANGLE_P",        11, AP_RollController, angle_p, 0.0),

#if AP_ESNCONTROLLER_ENABLED
    // Сеть для P: префикс _ESN_
    AP_SUBGROUPINFO(_esn_P, "_ESN_", 12, AP_RollController, ESNController),
    AP_GROUPINFO("_ESN_RESET", 13, AP_RollController, _esn_reset_param, 0),
    // Сеть для D: префикс _ESND_
    AP_SUBGROUPINFO(_esn_D, "_ESND_", 14, AP_RollController, ESNController),
#endif

    AP_GROUPEND
};

// constructor
AP_RollController::AP_RollController(const AP_FixedWing &parms)
    : AP_FW_Controller(parms,
      AC_PID::Defaults{
        .p         = 0.08,
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
    AP_AutoTune::ATType::AUTOTUNE_ROLL)
{
    AP_Param::setup_object_defaults(this, var_info);
}

#if AP_ESNCONTROLLER_ENABLED
float AP_RollController::esn_update(float rate_err, float desired_rate,
                                    float measured_rate, float scaler)
{
    // ─── 1. Триггер сброса ───
    if (_esn_reset_param.get() != 0) {
        _esn_reset_param.set(0);
        _esn_initialized = false;
        _prev_eff_err = 0.0f;
        _filtered_indicator_P = 0.0f;
        _filtered_indicator_D = 0.0f;
        _filtered_indicator_gyro = 0.0f;
        _lp_rate_err = 0.0f;
        _lp_gyro_fast = 0.0f;
        _lp_gyro_slow = 0.0f;
        _prev_hp_rate = 0.0f;
        _prev_hp_gyro = 0.0f;
        _prev_sign = 0;
        _sign_changes = 0;
        _last_check_ms = 0;
        _osc_freq = 0.0f;
        _esn_train_divider = 0;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ESN Roll: reset requested");
    }

    // ─── 2. Ленивая инициализация ───
    if (!_esn_initialized) {
        _esn_P.init(12, 70);
        _esn_D.init(12, 70);
        _base_kP = rate_pid.kP();
        _base_kD = rate_pid.kD();
        _esn_scale_P = 1.0f;
        _esn_scale_D = 1.0f;
        _esn_initialized = true;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO,
                      "ESN Roll: init done, base_kP=%.4f base_kD=%.5f",
                      (double)_base_kP, (double)_base_kD);
    }

    const float dt = AP::scheduler().get_loop_period_s();
    const bool do_train = (++_esn_train_divider >= _ESN_TRAIN_DIVIDER);
    if (do_train) {
        _esn_train_divider = 0;
    }

    // ─── 3. Общие признаки ───
    const Vector3f gyro = AP::ahrs().get_gyro();
    const float gyro_x_deg = degrees(gyro.x);
    const float gyro_y_deg = degrees(gyro.y);
    const float gyro_z_deg = degrees(gyro.z);
    const float gyro_axis_deg = gyro_x_deg;   // активная ось для roll

    float airspeed = 0.0f;
    const AP_Airspeed &aspeed = AP::airspeed();
    if (aspeed.healthy()) {
        airspeed = aspeed.get_airspeed();
    } else {
        airspeed = AP::ahrs().groundspeed();
    }

    float alt = 0.0f;
    AP::ahrs().get_relative_position_D_home(alt);

    const float rate_err_deriv = (rate_err - _prev_rate_err) / dt;
    _prev_rate_err = rate_err;

    // ─── 4. Feature-вектор (всё в °/с) ───
    float features[12] = {
        rate_err, rate_err_deriv, desired_rate, measured_rate,
        gyro_x_deg, gyro_y_deg, gyro_z_deg,
        airspeed, alt, scaler,
        _esn_scale_P, _base_kP
    };

    // ══════════════════════════════════════════════════════════════════
    // ЧАСТЬ 1. СЕТЬ ДЛЯ P
    // ══════════════════════════════════════════════════════════════════

    float correction_P = _esn_P.update(features);
    correction_P = constrain_float(correction_P, -3.0f, 0.3f);
    float new_scale_P = expf(correction_P);
    new_scale_P = constrain_float(new_scale_P, 0.15f, 1.5f);

    // ─── HP-фильтр rate_err ───
    const float lpf_alpha_slow = 0.99f;
    _lp_rate_err = lpf_alpha_slow * _lp_rate_err + (1.0f - lpf_alpha_slow) * rate_err;
    const float hp_rate_err = rate_err - _lp_rate_err;

    const float hp_deriv = constrain_float((hp_rate_err - _prev_hp_rate) / dt, -10.0f, 10.0f);
    _prev_hp_rate = hp_rate_err;

    const float indicator_P = fabsf(hp_rate_err) + 0.2f * fabsf(hp_deriv);
    _filtered_indicator_P = 0.92f * _filtered_indicator_P + 0.08f * indicator_P;

    // ─── Детектор знакопеременности ───
    const int8_t curr_sign = (hp_rate_err >= 0.0f) ? 1 : -1;
    if (curr_sign != _prev_sign && fabsf(hp_rate_err) > 0.3f) {
        _sign_changes++;
    }
    _prev_sign = curr_sign;

    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - _last_check_ms >= 200) {
        _osc_freq = _sign_changes / 0.2f;
        _sign_changes = 0;
        _last_check_ms = now_ms;
    }

    // ─── Детектор раскачки по гироскопу (ось X) ───
    const float lpf_alpha_gyro = 0.98f;
    _lp_gyro_slow = lpf_alpha_gyro * _lp_gyro_slow + (1.0f - lpf_alpha_gyro) * gyro_axis_deg;
    const float hp_gyro_slow = gyro_axis_deg - _lp_gyro_slow;

    const float gyro_deriv = constrain_float((hp_gyro_slow - _prev_hp_gyro) / dt, -500.0f, 500.0f);
    _prev_hp_gyro = hp_gyro_slow;

    const float indicator_gyro = fabsf(hp_gyro_slow) + 0.2f * fabsf(gyro_deriv);
    _filtered_indicator_gyro = 0.92f * _filtered_indicator_gyro + 0.08f * indicator_gyro;

    // ─── Обучение P-сети ───
    const bool is_maneuver_P = fabsf(desired_rate) > 20.0f;

    const bool oscillating_P = ((_filtered_indicator_P > 3.0f) ||
                                (_filtered_indicator_gyro > 5.0f)) &&
                               !is_maneuver_P;

    if (do_train) {
        if (oscillating_P) {
            const float best_ind = MAX(_filtered_indicator_P, _filtered_indicator_gyro);
            float target_P = -0.3f * best_ind;
            target_P = constrain_float(target_P, -2.0f, 0.3f);
            _esn_P.train(target_P);
        } else {
            _esn_P.train(0.0f);
        }
    }

    const float max_delta_P = 0.03f;
    new_scale_P = constrain_float(new_scale_P,
                                  _esn_scale_P - max_delta_P,
                                  _esn_scale_P + max_delta_P);

    // ══════════════════════════════════════════════════════════════════
    // ЧАСТЬ 2. СЕТЬ ДЛЯ D — двунаправленная
    // ══════════════════════════════════════════════════════════════════

    float correction_D = _esn_D.update(features);
    correction_D = constrain_float(correction_D, -3.0f, 0.8f);
    float new_scale_D = expf(correction_D);
    new_scale_D = constrain_float(new_scale_D, 0.15f, 4.0f);

    const float lpf_alpha_fast = 0.95f;
    _lp_gyro_fast = lpf_alpha_fast * _lp_gyro_fast + (1.0f - lpf_alpha_fast) * gyro_axis_deg;
    const float hp_gyro_fast = gyro_axis_deg - _lp_gyro_fast;

    const float hf_energy = hp_gyro_fast * hp_gyro_fast;
    _filtered_indicator_D = 0.92f * _filtered_indicator_D + 0.08f * hf_energy;

    const bool is_maneuver_D = fabsf(desired_rate) > 30.0f;

    const bool need_less_damping = (_filtered_indicator_D > 0.5f);
    const bool need_more_damping = (_filtered_indicator_D < 0.05f) &&
                                   (_filtered_indicator_gyro > 1.5f) &&
                                   !is_maneuver_D;

    if (do_train) {
        float target_D = 0.0f;
        if (need_less_damping) {
            target_D = -0.3f * _filtered_indicator_D;
        } else if (need_more_damping) {
            target_D = +0.10f * _filtered_indicator_gyro;
        }
        target_D = constrain_float(target_D, -1.0f, 0.8f);
        _esn_D.train(target_D);
    }

    const float max_delta_D = 0.02f;
    new_scale_D = constrain_float(new_scale_D,
                                  _esn_scale_D - max_delta_D,
                                  _esn_scale_D + max_delta_D);

    // ══════════════════════════════════════════════════════════════════
    // ЧАСТЬ 3. ПРИМЕНЕНИЕ
    // ══════════════════════════════════════════════════════════════════

    const float new_kP = _base_kP * new_scale_P;
    const float new_kD = _base_kD * new_scale_D;

    rate_pid.kP().set(new_kP);
    rate_pid.kD().set(new_kD);

    static uint32_t last_dbg_ms = 0;
    if (AP_HAL::millis() - last_dbg_ms >= 1000) {
        last_dbg_ms = AP_HAL::millis();
        hal.console->printf("ESN Roll: ind_P=%.2f ind_gyro=%.2f ind_D=%.3f scale_P=%.3f scale_D=%.3f kP=%.4f kD=%.5f\n",
                            (double)_filtered_indicator_P,
                            (double)_filtered_indicator_gyro,
                            (double)_filtered_indicator_D,
                            (double)new_scale_P,
                            (double)new_scale_D,
                            (double)rate_pid.kP(),
                            (double)rate_pid.kD());

        GCS_SEND_TEXT(MAV_SEVERITY_INFO,
                      "ESN Roll: kP=%.4f kD=%.5f scale_P=%.3f scale_D=%.3f",
                      (double)rate_pid.kP(),
                      (double)rate_pid.kD(),
                      (double)new_scale_P,
                      (double)new_scale_D);
    }

    _esn_scale_P = new_scale_P;
    _esn_scale_D = new_scale_D;

    log_esn_data(
        0, new_scale_P, new_scale_D, _base_kP, _base_kD,
        rate_err, hp_rate_err, hp_gyro_fast, correction_P, _osc_freq, oscillating_P
    );

    return 1.0f;
}
#endif

float AP_RollController::get_measured_angle_deg() const
{
    return AP::ahrs().get_roll_deg();
}

float AP_RollController::get_measured_rate_rads() const
{
    return AP::ahrs().get_gyro().x;
}

bool AP_RollController::is_underspeed() const
{
    return get_airspeed() <= float(aparm.airspeed_min);
}

float AP_RollController::get_positive_rate_limit_degs() const
{
    return MAX(gains.rmax_pos.get(), 0.0);
}

float AP_RollController::get_negative_rate_limit_degs() const
{
    return get_positive_rate_limit_degs();
}

bool AP_RollController::should_apply_rate_limits() const
{
    return !in_recovery;
}

float AP_RollController::run_axis_rate_control(float desired_rate_degs, float scaler, bool disable_integrator, bool ground_mode)
{
    const float indecision_threshold_deg = 160;
    const float last_desired_rate = _pid_info.target;
    const float abs_angle_err_deg = fabsf(angle_err_deg);
    if (abs_angle_err_deg > indecision_threshold_deg &&
        angle_err_deg <= 180) {
        if (desired_rate_degs * last_desired_rate < 0) {
            desired_rate_degs = -desired_rate_degs;
            const float new_angle_err_deg = abs_angle_err_deg + (180 - abs_angle_err_deg)*2;
            desired_rate_degs *= new_angle_err_deg / abs_angle_err_deg;
        }
    }

    in_recovery = false;

    return run_rate_control(desired_rate_degs, scaler, disable_integrator, ground_mode);
}