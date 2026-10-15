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
    AP_SUBGROUPINFO(_esn, "_ESN_", 12, AP_RollController, ESNController),
    AP_GROUPINFO("_ESN_RESET",  13, AP_RollController, _esn_reset_param, 0),
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
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ESN Roll: reset requested");
    }

    // 2. Ленивая инициализация
    if (!_esn_initialized) {
        _esn.init(12, 70);
        _base_kP = rate_pid.kP();
        _base_kD = rate_pid.kD();
        _esn_scale = 1.0f;
        _esn_initialized = true;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO,
                      "ESN Roll: init done, base_kP=%.4f",
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
        _osc_freq = _sign_changes / 0.2f;    // смен знака в секунду
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
        // В норме учим сеть НЕ вмешиваться
        _esn.train(0.0f);
    }

    log_esn_data(
        0,                  // axis: roll
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