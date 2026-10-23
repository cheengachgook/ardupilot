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
        _filt_abs_err_roll = 0.0f;
        _filtered_indicator_gyro = 0.0f;
        _prev_raw_err = 0.0f;
        _zcr = 0;
        _safe_scale_P = 1.00f;
        _safe_scale_D = 1.00f;
        _sluggish_mult = 1.0f;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ESN Roll: reset requested");
    }

    // ─── 2. Ленивая инициализация ───
    if (!_esn_initialized) {
        _esn_P.init(12, 70);
        _esn_D.init(12, 70);
        _base_kP = rate_pid.kP();
        _base_kD = rate_pid.kD();
        _esn_initialized = true;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ESN Roll: init done, base_kP=%.4f base_kD=%.5f", (double)_base_kP, (double)_base_kD);
    }

    //const float dt = AP::scheduler().get_loop_period_s();
    const Vector3f gyro = AP::ahrs().get_gyro();
    //const float gyro_axis_deg = degrees(gyro.x); // Ось X для Roll

    // Ограничиваем ошибку для защиты нейронов от насыщения
    const float safe_rate_err = constrain_float(rate_err, -3.0f, 3.0f);

    // ─── 3. ДЕТЕКТОР ВИБРАЦИИ (С фильтрацией) ───
    static float lp_gyro = 0.0f;
    lp_gyro = 0.95f * lp_gyro + 0.05f * gyro.x; // Для Pitch: gyro.y
    float hp_gyro = gyro.x - lp_gyro;
    float vib_energy = fabsf(hp_gyro); 

    // ДОБАВЛЯЕМ ФИЛЬТР: Сглаживаем вибрацию, чтобы одиночные кочки не вызывали панику
    _filt_vib = 0.98f * _filt_vib + 0.02f * vib_energy;

    static float prev_rate_err = 0.0f;
    float rate_err_deriv = safe_rate_err - prev_rate_err;
    prev_rate_err = safe_rate_err;

    if (fabsf(safe_rate_err) > 0.5f) {
        if (safe_rate_err * _prev_raw_err < 0.0f) _zcr++;
    }
    static float prev_deriv = 0.0f;
    if (fabsf(rate_err_deriv) > 0.1f) {
        if (rate_err_deriv * prev_deriv < 0.0f) _zcr++;
    }
    prev_deriv = rate_err_deriv;
    _prev_raw_err = safe_rate_err;

    // ─── 4. Фильтры ошибки ───
    const float lpf_alpha = 0.92f;
    _filt_abs_err_roll = lpf_alpha * _filt_abs_err_roll + (1.0f - lpf_alpha) * fabsf(safe_rate_err);
    
    // Медленный фильтр для оценки стабильности
    static float slow_err = 0.0f;
    slow_err = 0.995f * slow_err + 0.005f * fabsf(safe_rate_err);

    // ─── 5. Формирование фичей ───
    float airspeed = AP::ahrs().groundspeed();
    float alt = 0.0f;
    AP::ahrs().get_relative_position_D_home(alt);
    
    float features_roll[12] = {
        rate_err, 0, desired_rate, measured_rate,
        gyro.x, gyro.y, gyro.z,
        airspeed, alt, scaler,
        _esn_scale_P, _base_kP
    };

    // ─── 6. UPDATE (Прямой проход) ───
    float corr_P = _esn_P.update(features_roll);
    float corr_D = _esn_D.update(features_roll);

    corr_P = constrain_float(corr_P, -3.0f, 0.3f);
    corr_D = constrain_float(corr_D, -3.0f, 0.3f);

    float new_scale_P = expf(corr_P);
    float new_scale_D = expf(corr_D);

    // Жесткие рамки масштаба
    new_scale_P = constrain_float(new_scale_P, 0.02f, _safe_scale_P);
    new_scale_D = constrain_float(new_scale_D, 0.02f, _safe_scale_D);

    // Rate limiter (не более 3% за такт)
    const float max_delta = 0.03f;
    new_scale_P = constrain_float(new_scale_P, _esn_scale_P - max_delta, _esn_scale_P + max_delta);
    new_scale_D = constrain_float(new_scale_D, _esn_scale_D - max_delta, _esn_scale_D + max_delta);

    _esn_scale_P = new_scale_P;
    _esn_scale_D = new_scale_D;

    // ─── 7. Применение к PID (Защита D как на коптере) ───
    // D не может упасть ниже 0.003 (иначе будет вибрация)
    float p_out = _base_kP * new_scale_P;
    float d_out = MAX(_base_kD * new_scale_D, 0.003f); 

    rate_pid.kP().set(p_out);
    rate_pid.kD().set(d_out);

    // ─── 8. TRAIN (Умное обучение) ───
    // ВЫНОСИМ КОНСТАНТЫ НАВЕРХ, ЧТОБЫ ОНИ БЫЛИ ВИДНЫ В БЛОКЕ 9
    constexpr float ERR_OSC = 5.0f;     
    constexpr float CREEP_STEP = 0.001f; 
    //constexpr float VIB_THRESHOLD = 0.05f; 

    static uint16_t train_div = 0;
    bool do_train = (++train_div >= 8);
    if (do_train) train_div = 0;
    
    // Учимся чаще, если ошибка огромная
    if (fabsf(safe_rate_err) > 2.0f) do_train = true;

       if (do_train) {
        // Защита на взлете (скорость меньше 1.2 от минимальной)
        bool is_low_speed = get_airspeed() < (aparm.airspeed_min * 1.2f);
        
        // Раскачка детектится только если мы УЖЕ в воздухе
        bool is_osc = ((_zcr > 3) || (_filt_vib > 0.1f)) && 
                      (slow_err > 2.0f) && 
                      !is_low_speed;

        float target_P, target_D;

        if (is_low_speed) {
            // НА ВЗЛЕТЕ: Запрещаем срезать пиды!
            // Вместо этого плавно тянем масштаб к потолку (safe_scale).
            // Если базовые пиды низкие, safe_scale уже начал расти в блоке 9.
            target_P = 0.1f * (_safe_scale_P - _esn_scale_P);
            target_D = 0.1f * (_safe_scale_D - _esn_scale_D);
        } else if (is_osc) {
            // В ВОЗДУХЕ И ТРЯСЕТ: Жестко режем
            target_P = -3.0f; 
            _safe_scale_P *= 0.7f; 
            
            target_D = -3.0f; 
            _safe_scale_D *= 0.7f; 
        } else {
            // В ВОЗДУХЕ И НЕ ТРЯСЕТ: Тянемся к потолку (возвращаем пиды)
            target_P = 0.1f * (_safe_scale_P - _esn_scale_P);
            target_D = 0.1f * (_safe_scale_D - _esn_scale_D);
        }
        
        _safe_scale_P = constrain_float(_safe_scale_P, 0.05f, 5.0f);
        _safe_scale_D = constrain_float(_safe_scale_D, 0.05f, 5.0f);
        _esn_P.train(constrain_float(target_P, -3.0f, 0.3f));
        _esn_D.train(constrain_float(target_D, -3.0f, 0.3f));
    }

    // ─── 9. Поиск потолка (10 Гц) ───
    static uint32_t last_creep_ms = 0;
    if (AP_HAL::millis() - last_creep_ms >= 100) { 
        last_creep_ms = AP_HAL::millis();
        
        if (slow_err < (ERR_OSC * 0.2f)) _zcr = 0; 
        _zcr = (_zcr > 2) ? (_zcr - 2) : 0;

        static float lp_gyro_10hz = 0.0f;
        lp_gyro_10hz = 0.95f * lp_gyro_10hz + 0.05f * gyro.x; // Для Pitch: gyro.y
        float vib_energy_10hz = fabsf(gyro.x - lp_gyro_10hz);

        constexpr float VIB_THRESHOLD_10HZ = 0.05f; 
        
        // На малой скорости (на земле) игнорируем вибрации от шасси
        bool is_low_speed = get_airspeed() < (aparm.airspeed_min * 1.2f);
        bool is_osc_10hz = (!is_low_speed) && ((_zcr > 2) || (vib_energy_10hz > VIB_THRESHOLD_10HZ));

        if (is_osc_10hz) {
            // РАСКАЧКА В ВОЗДУХЕ: Сбрасываем ускорение
            _sluggish_mult = 1.0f;
        } else if (slow_err > (ERR_OSC * 0.5f) && _safe_scale_P < 5.0f) {
            // ВЯЛОСТЬ (или ВЗЛЕТ с низкими пидами): Ускоряем рост потолка!
            _sluggish_mult += 0.5f; 
            _sluggish_mult = MIN(_sluggish_mult, 10.0f); 
            
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
        hal.console->printf("ESN Roll: err=%.2f zcr=%d scP=%.2f scD=%.2f safeP=%.2f safeD=%.2f\n",
                            (double)slow_err, (int)_zcr, (double)_esn_scale_P, (double)_esn_scale_D, (double)_safe_scale_P, (double)_safe_scale_D);
    }

    // Логирование
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