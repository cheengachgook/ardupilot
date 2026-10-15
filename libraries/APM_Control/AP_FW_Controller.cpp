/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
//	Code by Jon Challinger
//  Modified by Paul Riseborough
//

#include "AP_FW_Controller.h"
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Scheduler/AP_Scheduler.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Math/AP_Math.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Airspeed/AP_Airspeed.h>
#include <AP_Logger/AP_Logger.h>

extern const AP_HAL::HAL& hal;

AP_FW_Controller::AP_FW_Controller(const AP_FixedWing &parms, const AC_PID::Defaults &defaults, AP_AutoTune::ATType _autotune_type)
    : aparm(parms),
      rate_pid(defaults),
      autotune_type(_autotune_type)
{
    rate_pid.set_slew_limit_scale(45);
}

// Return true if input shaping should be used
bool AP_FW_Controller::should_apply_input_shaping() const
{
    if (!should_apply_rate_limits()) {
        return false;
    }
    if (!is_positive(accel_limit.get())) {
        return false;
    }
    if ((autotune != nullptr) && autotune->running) {
        return false;
    }
    return true;
}

// Run angle controller
float AP_FW_Controller::run_angle_control(int32_t desired_angle_cd, float scaler, bool disable_integrator, bool ground_mode)
{
    if (gains.tau < 0.05f) {
        gains.tau.set(0.05f);
    }
    const float desired_angle_deg = wrap_180(desired_angle_cd * 0.01);

    if (!should_apply_input_shaping()) {
        angle_err_deg = wrap_180(desired_angle_deg - get_measured_angle_deg());
        float desired_rate_degs = angle_err_deg / gains.tau;

        reset_input_shaping_deg(desired_angle_deg, desired_rate_degs);
        desired_rate_degs += get_rate_target_offset_degs();

        if (should_apply_rate_limits()) {
            desired_rate_degs = rate_limit_degs(desired_rate_degs);
        }

        return run_axis_rate_control(desired_rate_degs, scaler, disable_integrator, ground_mode);
    }

    const float dt = AP::scheduler().get_loop_period_s();
    const float accel_max = accel_limit.get();
    const float jerk_limit = accel_max / MAX(gains.tau.get(), 0.1);

    const float angle_error = wrap_180(desired_angle_deg - angle_target_deg);

    shape_pos_vel_accel(
        angle_error, 0.0, 0.0,
        0.0, rate_target_degs, accel_target_degss,
        -get_negative_rate_limit_degs(), get_positive_rate_limit_degs(),
        -accel_max, accel_max,
        jerk_limit,
        dt, true
    );

    angle_target_deg += rate_target_degs * dt + accel_target_degss * 0.5 * sq(dt);
    rate_target_degs += accel_target_degss * dt;
    angle_target_deg = wrap_180(angle_target_deg);

    angle_err_deg = wrap_180(angle_target_deg - get_measured_angle_deg());

    float angle_gain = 1.0 / gains.tau.get();
    if (is_positive(angle_p.get())) {
        angle_gain = angle_p.get();
    }

    float desired_rate_degs = sqrt_controller(angle_err_deg, angle_gain, accel_max * 0.5, dt);
    desired_rate_degs = rate_limit_degs(desired_rate_degs + rate_target_degs + get_rate_target_offset_degs());

    return run_axis_rate_control(desired_rate_degs, scaler, disable_integrator, ground_mode);
}

/*
  AC_PID based rate controller
*/
float AP_FW_Controller::run_rate_control(float desired_rate_degs, float scaler, bool disable_integrator, bool ground_mode)
{
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    const uint32_t ticks = AP::scheduler().ticks32();
    if (last_run_ticks != 0) {
        if (last_run_ticks == ticks) {
            AP_HAL::panic("FW rate control must be run only once per loop");
        }
        if ((last_run_ticks + 1) != ticks) {
            AP_HAL::panic("FW rate control must be run or reset every loop");
        }
    }
    last_run_ticks = ticks;
#endif

    const float dt = AP::scheduler().get_loop_period_s();
    const float eas2tas = AP::ahrs().get_EAS2TAS();
    bool limit_I = fabsf(_last_out) >= 45;
    const float rate_rads = get_measured_rate_rads();
    const float old_I = rate_pid.get_i();

    const bool underspeed = is_underspeed();
    if (underspeed) {
        limit_I = true;
    }

    rate_pid.update_all(radians(desired_rate_degs) * scaler * scaler, rate_rads * scaler * scaler, dt, limit_I);

    if (underspeed) {
        rate_pid.set_integrator(old_I);
    }

    
#if AP_ESNCONTROLLER_ENABLED
    float esn_scale = 1.0f;
    // Виртуальный вызов: для roll это AP_RollController::esn_update,
    // для pitch — AP_PitchController::esn_update
    const float rate_err_degs = desired_rate_degs - degrees(rate_rads);
    esn_scale = esn_update(rate_err_degs, desired_rate_degs, degrees(rate_rads), scaler);

    static uint16_t esn_dbg_cnt = 0;
    if (++esn_dbg_cnt >= 50) {
        esn_dbg_cnt = 0;
        hal.console->printf("ESN: scale=%.3f rate_err=%.2f desired=%.1f actual=%.1f\n",
                    esn_scale, rate_err_degs, desired_rate_degs, degrees(rate_rads));
    }
#endif

    const float ff = degrees(ff_scale * rate_pid.get_ff_component() / (scaler * eas2tas));
    const float dff = degrees(ff_scale * rate_pid.get_dff_component() / (scaler * eas2tas));
    ff_scale = 1.0;

    if (disable_integrator) {
        rate_pid.reset_I();
    }

    _pid_info = rate_pid.get_pid_info();
    auto &pinfo = _pid_info;

    const float deg_scale = degrees(1);
    pinfo.FF = ff;
    pinfo.P *= deg_scale;
    pinfo.I *= deg_scale;
    pinfo.D *= deg_scale;
    pinfo.DFF = dff;

#if AP_ESNCONTROLLER_ENABLED
    pinfo.P *= esn_scale;
    pinfo.D *= esn_scale;
#endif

    pinfo.target = desired_rate_degs;
    pinfo.actual = degrees(rate_rads);

    float out = pinfo.FF + pinfo.P + pinfo.I + pinfo.D + pinfo.DFF;
    if (ground_mode) {
        out -= pinfo.D + 0.5*pinfo.P;
    }

    _last_out = out;

    if (autotune != nullptr && autotune->running && get_airspeed() > aparm.airspeed_min) {
        autotune->update(pinfo, scaler, angle_err_deg);
    }

    return constrain_float(out * 100, -4500, 4500);
}

/*
 Function returns an equivalent control surface deflection in centi-degrees
*/
float AP_FW_Controller::run_rate_control(float desired_rate_degs, float scaler)
{
    angle_err_deg = 0.0;

    if (!should_apply_input_shaping()) {
        reset_input_shaping_deg(get_measured_angle_deg(), desired_rate_degs);
        return run_rate_control(desired_rate_degs, scaler, false, false);
    }

    const float dt = AP::scheduler().get_loop_period_s();
    angle_target_deg = get_measured_angle_deg();

    const float accel_max = accel_limit.get();
    const float jerk_limit = accel_max / MAX(gains.tau.get(), 0.1);

    shape_pos_vel_accel(
        0.0, desired_rate_degs, 0.0,
        0.0, rate_target_degs, accel_target_degss,
        -get_negative_rate_limit_degs(), get_positive_rate_limit_degs(),
        -accel_max, accel_max,
        jerk_limit,
        dt, true
    );

    rate_target_degs += accel_target_degss * dt;

    return run_rate_control(rate_target_degs, scaler, false, false);
}

// Reset I term
void AP_FW_Controller::reset_I()
{
    rate_pid.reset_I();
    _last_out = 0.0;
}

void AP_FW_Controller::decay_I()
{
    _pid_info.I *= 0.995f;
    rate_pid.set_integrator(rate_pid.get_i() * 0.995);
}

void AP_FW_Controller::autotune_restore(void)
{
    if (autotune != nullptr) {
        autotune->stop();
    }
}

void AP_FW_Controller::autotune_start(void)
{
    if (autotune == nullptr) {
        autotune = NEW_NOTHROW AP_AutoTune(gains, autotune_type, aparm, rate_pid);
        if (autotune == nullptr) {
            if (!failed_autotune_alloc) {
                GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "AutoTune: failed %s allocation", AP_AutoTune::axis_string(autotune_type));
            }
            failed_autotune_alloc = true;
        }
    }
    if (autotune != nullptr) {
        autotune->start();
    }
}

// Return the airspeed in m/s
float AP_FW_Controller::get_airspeed() const
{
    float aspeed;
    if (!AP::ahrs().airspeed_EAS(aspeed)) {
        aspeed = 0.5f*(float(aparm.airspeed_min) + float(aparm.airspeed_max));
    }
    return aspeed;
}

// Reset controller
void AP_FW_Controller::reset()
{
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    last_run_ticks = 0;
#endif
    rate_pid.reset_I();
    rate_pid.reset_filter();
    _last_out = 0.0;
    reset_input_shaping_deg(get_measured_angle_deg(), degrees(get_measured_rate_rads()));
}

float AP_FW_Controller::rate_limit_degs(float rate_degs) const
{
    const float pos_rate_limit = get_positive_rate_limit_degs();
    if (is_positive(pos_rate_limit)) {
        rate_degs = MIN(rate_degs, pos_rate_limit);
    }
    const float neg_rate_limit = get_negative_rate_limit_degs();
    if (is_positive(neg_rate_limit)) {
        rate_degs = MAX(rate_degs, -neg_rate_limit);
    }
    return rate_degs;
}

void AP_FW_Controller::reset_input_shaping_deg(const float angle_deg, const float rate_degs)
{
    angle_target_deg = angle_deg;
    rate_target_degs = rate_limit_degs(rate_degs);
    accel_target_degss = 0.0;
}

void AP_FW_Controller::get_input_shaping(float &angle_deg, float &rate_degs, float &accel_degss) const
{
    angle_deg = angle_target_deg;
    rate_degs = rate_target_degs;
    accel_degss = accel_target_degss;
}

void AP_FW_Controller::ahrs_reset()
{
    angle_target_deg = wrap_180(get_measured_angle_deg() + angle_err_deg);
}

float AP_FW_Controller::get_angle_p() const
{
    if (should_apply_input_shaping() && is_positive(angle_p.get())) {
        return angle_p.get();
    }
    return 1.0 / gains.tau.get();
}

#if AP_ESNCONTROLLER_ENABLED
void AP_FW_Controller::log_esn_data(uint8_t axis, float scale, float base_kP,
                                    float rate_err, float hp_rate_err,
                                    float correction, float osc_freq, bool oscillating)
{
#if HAL_LOGGING_ENABLED
    // Ограничиваем частоту логирования до 25 Гц на каждую ось,
    // чтобы не забивать лог
    static uint32_t last_log_ms[2] = {0, 0};
    if (axis > 1) {
        return;
    }

    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - last_log_ms[axis] < 40) {   // 40 мс = 25 Гц
        return;
    }
    last_log_ms[axis] = now_ms;

    // Логируем всегда, когда логирование активно.
    // Ограничение по частоте уже сделано выше (25 Гц).
    if (!AP::logger().logging_started()) {
        return;
    }
    
    const uint8_t flags = (oscillating ? 0x01 : 0x00);
    const float kP_eff = base_kP * scale;

    const struct log_ESN pkt {
        LOG_PACKET_HEADER_INIT(LOG_ESN_MSG),
        time_us     : AP_HAL::micros64(),
        axis        : axis,
        flags       : flags,
        scale       : scale,
        base_kP     : base_kP,
        kP_eff      : kP_eff,
        rate_err    : rate_err,
        hp_rate_err : hp_rate_err,
        correction  : correction,
        osc_freq    : osc_freq
    };
    AP::logger().WriteBlock(&pkt, sizeof(pkt));
#endif
}
#endif