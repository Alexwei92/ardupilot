#include "Copter.h"

/*
 * Init and run calls for system id flight mode
 */

// sysid_init - initialise sysid controller
bool Copter::ModeSystemID::init(bool ignore_checks)
{
    // if landed and the mode we're switching from does not have manual throttle and the throttle stick is too high
    if (motors->armed() && ap.land_complete && !copter.flightmode->has_manual_throttle() &&
            (get_pilot_desired_throttle(channel_throttle->get_control_in()) > get_non_takeoff_throttle())) {
        return false;
    }
    return true;
}

// sysid_run - runs the main sysid controller
// should be called at 100hz or more
void Copter::ModeSystemID::run()
{
    float target_roll, target_pitch;
    float target_yaw_rate;
    float pilot_throttle_scaled;

    // if not armed set throttle to zero and exit immediately
    if (!motors->armed() || ap.throttle_zero || !motors->get_interlock()) {
        zero_throttle_and_relax_ac();
        return;
    }

    // clear landing flag
    set_land_complete(false);

    motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

    // apply SIMPLE mode transform to pilot inputs
    update_simple_mode();

    AP_Vehicle::MultiCopter &aparm = copter.aparm;

    // convert pilot input to lean angles
    get_pilot_desired_lean_angles(target_roll, target_pitch, aparm.angle_max, aparm.angle_max);

    // get pilot's desired yaw rate
    target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());

    // get pilot's desired throttle
    pilot_throttle_scaled = get_pilot_desired_throttle(channel_throttle->get_control_in());

    // inject the frequency sweep signal
    if (check_status()) {
        switch (copter.sweep.axis) {
            case ROLL:
                if(copter.sweep.type == SWEEP) {
                    run_frequency_sweep(copter.sweep_roll);
                } else {
                    run_doublet(copter.doublet_roll);
                }
                target_roll += (float)copter.sweep.signal*aparm.angle_max;
                target_roll = constrain_float(target_roll, -aparm.angle_max, aparm.angle_max);
                //copter.tmp3 = target_roll;
                break;
            case PITCH:
                if(copter.sweep.type == SWEEP) {
                    run_frequency_sweep(copter.sweep_pitch);
                } else {
                    run_doublet(copter.doublet_pitch);
                }
                target_pitch += (float)copter.sweep.signal*aparm.angle_max;
                target_pitch = constrain_float(target_pitch, -aparm.angle_max, aparm.angle_max);
                //copter.tmp3 = target_pitch;
                break;
            case YAW:
                if(copter.sweep.type == SWEEP) {
                    run_frequency_sweep(copter.sweep_yaw);
                } else {
                    run_doublet(copter.doublet_yaw);
                }
                target_yaw_rate += (float)copter.sweep.signal*aparm.angle_max*g.acro_yaw_p;
                target_yaw_rate = constrain_float(target_yaw_rate, -aparm.angle_max*g.acro_yaw_p, aparm.angle_max*g.acro_yaw_p);
                //copter.tmp3 = target_yaw_rate;
                break;
            case THROTTLE:
                if(copter.sweep.type == SWEEP) {
                    run_frequency_sweep(copter.sweep_throttle);
                } else {
                    run_doublet(copter.doublet_throttle);
                }
                pilot_throttle_scaled += (float)copter.sweep.signal;
                pilot_throttle_scaled = constrain_float(pilot_throttle_scaled, 0.0f, 1.0f);
                //copter.tmp3 = pilot_throttle_scaled;
                break;
        }
    }

    // call attitude controller
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);

    // body-frame rate controller is run directly from 100hz loop

    // output pilot's throttle
    attitude_control->set_throttle_out(pilot_throttle_scaled, true, g.throttle_filt);
}

// check if the frequency sweep is enabled
bool Copter::ModeSystemID::check_status()
{
    if (copter.sweep.status == START) {
        return true;
    } else {
        return false;
    }
}

// create the sinusoidal signal
void Copter::ModeSystemID::run_frequency_sweep(const freq_setting my_settings)
{
    uint32_t my_current_time = millis();
    uint32_t my_dt = my_current_time - copter.my_last_time;
    uint32_t my_period = my_current_time - copter.my_start_time;
    
    float my_delta = 0.0f;
    float my_K = 0.0f;

    // assign and pre-calculate the parameters
    uint8_t _T_trimin = my_settings.T_trimin;
    uint8_t _T_trimout = my_settings.T_trimout;
    uint8_t _T_fadein = my_settings.T_fadein;
    uint8_t _T_fadeout = my_settings.T_fadeout;
    uint8_t _T_total = my_settings.T_total;
    uint8_t _T_N = my_settings.T_N;
    float _T_wmin = (2.0f*MY_PI*my_settings.F_min);
    float _T_wmax = (2.0f*MY_PI*my_settings.F_max);
    float _T_tmin = (1.0f/my_settings.F_min);
    float _T_active = _T_total - _T_tmin*_T_N;
    
    copter.my_last_time = my_current_time;  // remember time when last run called
    float _A = my_settings.A_min + copter.sweep.amplitude*(my_settings.A_max-my_settings.A_min);

    // if time > (t_trimin+t_total+t_trimout), return 0 and set sweep.status to 3
    if ((my_period > (_T_trimin+_T_total+_T_trimout)*S_2_MILLIS)) {
        my_delta = 0.0f;
        copter.sweep.signal = (float)my_delta;
        if (copter.sweep.status == START) {
            gcs().send_text(MAV_SEVERITY_INFO, "Frequency sweep end");
            copter.sweep.status = STOP;
            copter.my_theta = 0.0f;
        };
        return;
    }

    // if time < t_trimin or time > (t_trimin+t+t_total), return 0
    if ((my_period < _T_trimin*S_2_MILLIS) || (my_period > (_T_trimin+_T_total)*S_2_MILLIS)) {
        my_delta = 0.0f;
        copter.sweep.signal = (float)my_delta;
        return;
    }

    // else do the frequency sweep
    if (my_period < (_T_trimin+_T_tmin*_T_N)*S_2_MILLIS) {
        copter.my_theta += _T_wmin*(my_dt*1e-3f);
    } else {
        my_K = MY_C2*(expf(MY_C1*(my_period*1e-3f-_T_tmin*_T_N-_T_trimin)/_T_active)-1.0f);
        copter.my_theta += (_T_wmin + my_K*(_T_wmax-_T_wmin))*(my_dt*1e-3f);
    }
    my_delta = _A*sinf(copter.my_theta);

    // Apply fadein and fadeout
    if (my_period < (_T_trimin+_T_fadein)*S_2_MILLIS) {
        my_delta = my_delta*(my_period*1e-3f-_T_trimin)/_T_fadein;
    } else if (my_period > (_T_trimin+_T_total-_T_fadeout)*S_2_MILLIS) {
        my_delta = my_delta*(_T_trimin+_T_total-my_period*1e-3f)/_T_fadeout;
    }
    copter.sweep.signal = (float)my_delta;
    return;
}

// create the doublet signal
void Copter::ModeSystemID::run_doublet(const doub_setting my_settings)
{
    uint32_t my_current_time = millis();
    uint32_t my_period = my_current_time - copter.my_start_time;

    uint8_t _T_trimin = my_settings.T_trimin;
    uint8_t _T_trimout = my_settings.T_trimout;
    uint8_t _T_pulse = my_settings.T_pulse;

    float my_delta = 0.0f;

    if (my_period > (_T_trimin+2*_T_pulse+_T_trimout)*S_2_MILLIS) {
        my_delta = 0.0f;
        copter.sweep.signal = (float)my_delta;
        if (copter.sweep.status == START) {
            gcs().send_text(MAV_SEVERITY_INFO, "Doublet end");
            copter.sweep.status = STOP;
        }
        return;
    }

    if ((my_period < _T_trimin*S_2_MILLIS) || (my_period > (_T_trimin+2*_T_pulse)*S_2_MILLIS)) {
        my_delta = 0.0f;
        copter.sweep.signal = (float)my_delta;
        return;
    }

    if (my_period < (_T_trimin+_T_pulse)*S_2_MILLIS) {
        my_delta = copter.sweep.amplitude*my_settings.A_max;
    } else {
        my_delta = -copter.sweep.amplitude*my_settings.A_max;
    }
    copter.sweep.signal = (float)my_delta;
    return;
}