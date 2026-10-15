#pragma once

#include "AP_FW_Controller.h"
#include <AP_ESNController/AP_ESNController_config.h>

#if AP_ESNCONTROLLER_ENABLED
    #include <AP_ESNController/ESNController.h>
#endif

class AP_RollController : public AP_FW_Controller
{
public:
    AP_RollController(const AP_FixedWing &parms);

    /* Do not allow copies */
    CLASS_NO_COPY(AP_RollController);

    static const struct AP_Param::GroupInfo var_info[];

    void set_in_recovery(void) {
        in_recovery = true;
    }

protected:
#if AP_ESNCONTROLLER_ENABLED
    ESNController _esn;
    float _prev_rate_err = 0.0f;
    float _base_kP = 0.0f;
    float _base_kD = 0.0f;
    float _esn_scale = 1.0f;
    bool  _esn_initialized = false;

    AP_Int8 _esn_reset_param;

    float _prev_eff_err = 0.0f;
    float _filtered_indicator = 0.0f;
    float _lp_rate_err = 0.0f;
    float _prev_hp = 0.0f;

    int      _sign_changes = 0;
    int8_t   _prev_sign = 0;
    uint32_t _last_check_ms = 0;
    float    _osc_freq = 0.0f;    

    float esn_update(float rate_err, float desired_rate,
                     float measured_rate, float scaler) override;
#endif

private:
    float run_axis_rate_control(float desired_rate_degs, float scaler, bool disable_integrator, bool ground_mode) override;

    bool is_underspeed() const override;
    float get_measured_angle_deg() const override;
    float get_measured_rate_rads() const override;
    bool should_apply_rate_limits() const override;
    float get_positive_rate_limit_degs() const override;
    float get_negative_rate_limit_degs() const override;

    bool in_recovery;
};