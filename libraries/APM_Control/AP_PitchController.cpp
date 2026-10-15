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
    AP_SUBGROUPINFO(_esn, "_ESN_", 14, AP_PitchController, ESNController),
    AP_GROUPINFO("_ESN_RESET",  15, AP_PitchController, _esn_reset_param, 0),
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
    // 1. Триггер сброса (с обнулением состояния)
    if (_esn_reset_param.get() != 0) {
        _esn_reset_param.set(0);
        _esn_initialized = false;
        _prev_eff_err = 0.0f;
        _filtered_indicator = 0.0f;
        _lp_rate_err = 0.0f;
        _prev_hp = 0.0f;
        _sign_changes = 0;
        _prev_sign = 0;
        _last_check_ms = 0;
        _osc_freq = 0.0f;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ESN Pitch: reset requested");
    }

    // 2. Ленивая инициализация
    if (!_esn_initialized) {
        _esn.init(12, 70);
        _base_kP = rate_pid.kP();
        _base_kD = rate_pid.kD();
        _esn_scale = 1.0f;
        _esn_initialized = true;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO,
                      "ESN Pitch: init done, base_kP=%.4f",
                      (double)_base_kP);
    }

    const float dt = AP::scheduler().get_loop_period_s();

    // 3. Признаки для сети
    const Vector3f gyro = AP::ahrs().get_gyro();

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

    float features[12] = {
        rate_err, rate_err_deriv, desired_rate, measured_rate,
        gyro.x, gyro.y, gyro.z,
        airspeed, alt, scaler,
        _esn_scale, _base_kP
    };

    // 4. Прямой проход
    float correction = _esn.update(features);
    correction = constrain_float(correction, -4.0f, 0.5f);

    float new_scale = expf(correction);
    new_scale = constrain_float(new_scale, 0.1f, 1.5f);

    // 5. HP-фильтр для выделения колебаний
    const float lpf_alpha = 0.98f;
    _lp_rate_err = lpf_alpha * _lp_rate_err + (1.0f - lpf_alpha) * rate_err;
    const float hp_rate_err = rate_err - _lp_rate_err;

    const float hp_deriv = constrain_float((hp_rate_err - _prev_hp) / dt, -10.0f, 10.0f);
    _prev_hp = hp_rate_err;

    const float indicator = fabsf(hp_rate_err) + 0.2f * fabsf(hp_deriv);
    _filtered_indicator = 0.92f * _filtered_indicator + 0.08f * indicator;

    // 6. Детектор смены знака (реальная раскачка)
    const int8_t curr_sign = (hp_rate_err >= 0.0f) ? 1 : -1;
    if (curr_sign != _prev_sign && fabsf(hp_rate_err) > 1.0f) {
        _sign_changes++;
    }
    _prev_sign = curr_sign;

    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - _last_check_ms >= 200) {
        _osc_freq = _sign_changes / 0.2f;
        _sign_changes = 0;
        _last_check_ms = now_ms;
    }

    const bool oscillating = (_osc_freq > 2.0f);

    // 7. Обучение только при реальной раскачке
    if (oscillating && _filtered_indicator > 5.0f) {
        float target_correction = -0.3f * _filtered_indicator;
        target_correction = constrain_float(target_correction, -2.0f, 0.3f);
        _esn.train(target_correction);
    } else {
        _esn.train(0.0f);
    }

    log_esn_data(
        1,                  // axis: roll
        new_scale,          // текущий масштаб
        _base_kP,           // базовый kP
        rate_err,           // ошибка скорости
        hp_rate_err,        // HP-компонента
        correction,         // выход сети
        _osc_freq,          // частота колебаний
        oscillating         // признак раскачки
    );

    return new_scale;
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