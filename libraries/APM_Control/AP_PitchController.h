#pragma once

#include "AP_FW_Controller.h"
#include <AP_ESNController/AP_ESNController_config.h>

#if AP_ESNCONTROLLER_ENABLED
    #include <AP_ESNController/ESNController.h>
#endif

class AP_PitchController : public AP_FW_Controller
{
public:
    AP_PitchController(const AP_FixedWing &parms);

    /* Do not allow copies */
    CLASS_NO_COPY(AP_PitchController);

    static const struct AP_Param::GroupInfo var_info[];

protected:
#if AP_ESNCONTROLLER_ENABLED
    // ── ESN для P ──
    ESNController _esn_P;
    float _prev_rate_err = 0.0f;
    float _base_kP = 0.0f;
    float _esn_scale_P = 1.0f;
    bool  _esn_initialized = false;

    float _prev_eff_err = 0.0f;
    float _filtered_indicator_P = 0.0f;
    float _lp_rate_err = 0.0f;
    float _prev_hp_rate = 0.0f;
    int8_t   _prev_sign = 0;
    int      _sign_changes = 0;
    uint32_t _last_check_ms = 0;
    float    _osc_freq = 0.0f;

    // ── ESN для D ──
    ESNController _esn_D;
    float _base_kD = 0.0f;
    float _esn_scale_D = 1.0f;
    float _lp_gyro_fast = 0.0f;          // LPF для HP-фильтра гироскопа
    float _filtered_indicator_D = 0.0f;  // накопленный индикатор HF-шума
    float _lp_gyro_slow = 0.0f;      // LPF для выделения медленной раскачки в гироскопе
    float _prev_hp_gyro = 0.0f;
    float _filtered_indicator_gyro = 0.0f;

    float _filt_abs_err_pitch = 0.0f;
    float _prev_raw_err = 0.0f;
    uint16_t _zcr = 0;
    float _safe_scale_P = 1.0f;
    float _safe_scale_D = 1.0f;
    float _sluggish_mult = 1.0f;
    float _filt_vib = 0.0f; 
    bool _pio_detected = false;
    float _base_kI = 0.0f;
    // ── Троттлинг обучения ──
    // train() вызывается раз в _ESN_TRAIN_DIVIDER тактов.
    // При 400 Гц и делителе 8 → 50 Гц обучения.
    uint16_t _esn_train_divider = 0;
    static constexpr uint16_t _ESN_TRAIN_DIVIDER = 400;

    AP_Int8  _esn_reset_param;

    float esn_update(float rate_err, float desired_rate,
                     float measured_rate, float scaler) override;
#endif

private:
    AP_Float _roll_ff;

    float run_axis_rate_control(float desired_rate_degs, float scaler, bool disable_integrator, bool ground_mode) override;

    bool is_underspeed() const override;
    float get_measured_angle_deg() const override;
    float get_measured_rate_rads() const override;
    bool should_apply_rate_limits() const override;
    float get_rate_target_offset_degs() const override;
    float get_positive_rate_limit_degs() const override;
    float get_negative_rate_limit_degs() const override;

    bool is_inverted() const;
};