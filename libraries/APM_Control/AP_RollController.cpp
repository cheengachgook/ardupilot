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
    static uint32_t last_osc_ms = 0;
    static bool has_oscillated = false;
    
    if (_esn_reset_param.get() != 0) {
        _esn_reset_param.set(0);
        _esn_initialized = false;
        _filt_abs_err_roll = 0.0f;
        _prev_raw_err = 0.0f;
        _zcr = 0;
        _safe_scale_P = 1.0f;
        _safe_scale_D = 1.0f;
        _sluggish_mult = 1.0f;
        last_osc_ms = 0;
        has_oscillated = false;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ESN Roll: reset requested");
    }

    // ─── 2. Инициализация и предобучение ───
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
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ESN Roll: Pretraining on %d samples", n_samples);
            _esn_P.pretrain(pretrain_data, n_samples, 0.001f, 0.0f, 1.0f, 5);
            _esn_D.pretrain(pretrain_data, n_samples, 0.001f, 0.0f, 1.0f, 5);
            delete[] pretrain_data;
        }
        _esn_initialized = true;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ESN Roll: init done, base_kP=%.4f", (double)_base_kP);
    }

    const Vector3f gyro = AP::ahrs().get_gyro();
    const float safe_rate_err = constrain_float(rate_err, -3.0f, 3.0f);

    // ─── 3. ДЕТЕКТОРЫ ───
    static float lp_gyro = 0.0f;
    lp_gyro = 0.95f * lp_gyro + 0.05f * gyro.x;
    float hp_gyro = gyro.x - lp_gyro;
    if (fabsf(hp_gyro) > 1.0f) hp_gyro = 0.0f;
    if (fabsf(desired_rate) > 25.0f || fabsf(measured_rate) > 25.0f) {
        hp_gyro = 0.0f; 
    }
    _filt_vib = 0.98f * _filt_vib + 0.02f * fabsf(hp_gyro);

    if (fabsf(safe_rate_err) > 0.3f) {  // Было 0.5
        if (safe_rate_err * _prev_raw_err < 0.0f) _zcr++;
    }
    _prev_raw_err = safe_rate_err;

    // PIO детектор
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

    // Отслеживаем колебания желаемой скорости
    static int8_t prev_desired_sign = 0;
    static int16_t desired_zcr = 0;
    int8_t curr_desired_sign = (desired_rate > 5.0f) ? 1 : ((desired_rate < -5.0f) ? -1 : 0);
    if (curr_desired_sign != 0 && prev_desired_sign != 0 && curr_desired_sign != prev_desired_sign) desired_zcr++;
    if (curr_desired_sign != 0) prev_desired_sign = curr_desired_sign;

    // ─── 4. ФИЛЬТРЫ ───
    static float slow_err = 0.0f;
    slow_err = 0.995f * slow_err + 0.005f * fabsf(safe_rate_err);

    // ─── 5. ФИЧИ ───
    float airspeed = AP::ahrs().groundspeed();
    float alt = 0.0f;
    AP::ahrs().get_relative_position_D_home(alt);

    float features_roll[12] = {
        rate_err, 0, desired_rate, measured_rate,
        gyro.x, gyro.y, gyro.z,
        airspeed, alt, scaler,
        _esn_scale_P, _base_kP
    };

    // ─── 6. UPDATE ───
    float corr_P = _esn_P.update(features_roll);
    float corr_D = _esn_D.update(features_roll);
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

    // ─── 7. ПРИМЕНЕНИЕ ───
    rate_pid.kP().set(_base_kP * new_scale_P);
    rate_pid.kD().set(MAX(_base_kD * new_scale_D, 0.003f));
    rate_pid.kI().set(_base_kI * new_scale_P);

    // ─── 8. TRAIN ───
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

    // ─── 9. Поиск потолка (10 Гц) ───
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
            // Никакого роста во время раскачки, взлета, маневра или остывания
            _sluggish_mult = 1.0f;
        } else if (slow_err > (ERR_OSC * 0.5f) && _safe_scale_P < 1.0f) {
            // Медленный рост: 0.3 за такт (не 1.0, как было)
            _sluggish_mult += 0.3f;
            _sluggish_mult = MIN(_sluggish_mult, 3.0f);
            _safe_scale_P += CREEP_STEP * _sluggish_mult;
            _safe_scale_D += CREEP_STEP * _sluggish_mult;
        } else {
            _sluggish_mult = 1.0f;
        }
    }

    // ─── 10. Диагностика ───
    static uint32_t esn_dbg_ms = 0;
    if (AP_HAL::millis() - esn_dbg_ms >= 1000) {
        esn_dbg_ms = AP_HAL::millis();
        bool cd = has_oscillated && (AP_HAL::millis() - last_osc_ms < 4000);
        hal.console->printf("ESN Roll: err=%.2f zcr=%d pio=%d m=%d cd=%d scP=%.2f safeP=%.2f\n",
                            (double)slow_err, (int)_zcr, (int)_pio_detected,
                            (int)is_maneuver, (int)cd,
                            (double)_esn_scale_P, (double)_safe_scale_P);
    }

    log_esn_data(0, new_scale_P, new_scale_D, _base_kP, _base_kD, rate_err, 0, 0, corr_P, 0, false);
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