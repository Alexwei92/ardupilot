#include "Copter.h"


/*
 * Init and run calls for althold, flight mode
 */

// althold_init - initialise althold controller
bool Copter::ModeAltHold::init(bool ignore_checks)
{
    // initialise position and desired velocity
    if (!pos_control->is_active_z()) {
        pos_control->set_alt_target_to_current_alt();
        pos_control->set_desired_velocity_z(inertial_nav.get_velocity_z());
    }

    return true;
}

// althold_run - runs the althold controller
// should be called at 100hz or more
void Copter::ModeAltHold::run()
{
    AltHoldModeState althold_state;
    float takeoff_climb_rate = 0.0f;

    // initialize vertical speeds and acceleration
    pos_control->set_speed_z(-get_pilot_speed_dn(), g.pilot_speed_up);
    pos_control->set_accel_z(g.pilot_accel_z);

    // apply SIMPLE mode transform to pilot inputs
    update_simple_mode();

    // get pilot desired lean angles
    float target_roll, target_pitch;
    get_pilot_desired_lean_angles(target_roll, target_pitch, copter.aparm.angle_max, attitude_control->get_althold_lean_angle_max());

    // get pilot's desired yaw rate
    float target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());

    // get pilot desired climb rate
    float target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
    target_climb_rate = constrain_float(target_climb_rate, -get_pilot_speed_dn(), g.pilot_speed_up);

    // Alt Hold State Machine Determination
    if (!motors->armed() || !motors->get_interlock()) {
        althold_state = AltHold_MotorStopped;
    } else if (takeoff.running() || takeoff.triggered(target_climb_rate)) {
        althold_state = AltHold_Takeoff;
    } else if (!ap.auto_armed || ap.land_complete) {
        althold_state = AltHold_Landed;
    } else {
        althold_state = AltHold_Flying;
    }

    // Alt Hold State Machine
    switch (althold_state) {

    case AltHold_MotorStopped:

        motors->set_desired_spool_state(AP_Motors::DESIRED_SHUT_DOWN);
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);
        attitude_control->reset_rate_controller_I_terms();
        attitude_control->set_yaw_target_to_current_heading();
#if FRAME_CONFIG == HELI_FRAME    
        // force descent rate and call position controller
        pos_control->set_alt_target_from_climb_rate(-abs(g.land_speed), G_Dt, false);
        heli_flags.init_targets_on_arming=true;
        if (ap.land_complete_maybe) {
            pos_control->relax_alt_hold_controllers(0.0f);
        }
#else
        pos_control->relax_alt_hold_controllers(0.0f);   // forces throttle output to go to zero
#endif
        pos_control->update_z_controller();
        break;

    case AltHold_Takeoff:
#if FRAME_CONFIG == HELI_FRAME    
        if (heli_flags.init_targets_on_arming) {
            heli_flags.init_targets_on_arming=false;
        }
#endif
        // set motors to full range
        motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

        // initiate take-off
        if (!takeoff.running()) {
            takeoff.start(constrain_float(g.pilot_takeoff_alt,0.0f,1000.0f));
            // indicate we are taking off
            set_land_complete(false);
            // clear i terms
            set_throttle_takeoff();
        }

        // get take-off adjusted pilot and takeoff climb rates
        takeoff.get_climb_rates(target_climb_rate, takeoff_climb_rate);

        // get avoidance adjusted climb rate
        target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

        // call attitude controller
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);

        // call position controller
        pos_control->set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
        pos_control->add_takeoff_climb_rate(takeoff_climb_rate, G_Dt);
        pos_control->update_z_controller();
        break;

    case AltHold_Landed:
        // set motors to spin-when-armed if throttle below deadzone, otherwise full range (but motors will only spin at min throttle)
        if (target_climb_rate < 0.0f) {
            motors->set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
        } else {
            motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);
        }

#if FRAME_CONFIG == HELI_FRAME    
        if (heli_flags.init_targets_on_arming) {
            attitude_control->reset_rate_controller_I_terms();
            attitude_control->set_yaw_target_to_current_heading();
            if (motors->get_interlock()) {
                heli_flags.init_targets_on_arming=false;
            }
        }
#else
        attitude_control->reset_rate_controller_I_terms();
        attitude_control->set_yaw_target_to_current_heading();
#endif
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);
        pos_control->relax_alt_hold_controllers(0.0f);   // forces throttle output to go to zero
        pos_control->update_z_controller();
        break;

    case AltHold_Flying:
        motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

#if AC_AVOID_ENABLED == ENABLED
        // apply avoidance
        copter.avoid.adjust_roll_pitch(target_roll, target_pitch, copter.aparm.angle_max);
#endif
        // inject the frequency sweep signal
        if (check_status()) {
            switch (copter.sweep.axis) {
                case ROLL:
                    if(copter.sweep.type == SWEEP) {
                        run_frequency_sweep(copter.sweep_roll);
                    } else {
                        run_doublet(copter.doublet_roll);
                    }
                    target_roll += (float)copter.sweep.signal*copter.aparm.angle_max;
                    target_roll = constrain_float(target_roll, -copter.aparm.angle_max, copter.aparm.angle_max);
                    //copter.tmp3 = target_roll;
                    break;
                case PITCH:
                    if(copter.sweep.type == SWEEP) {
                        run_frequency_sweep(copter.sweep_pitch);
                    } else {
                        run_doublet(copter.doublet_pitch);
                    }
                    target_pitch += (float)copter.sweep.signal*copter.aparm.angle_max;
                    target_pitch = constrain_float(target_pitch, -copter.aparm.angle_max, copter.aparm.angle_max);
                    //copter.tmp3 = target_pitch;
                    break;
                case YAW:
                    if(copter.sweep.type == SWEEP) {
                        run_frequency_sweep(copter.sweep_yaw);
                    } else {
                        run_doublet(copter.doublet_yaw);
                    }
                    target_yaw_rate += (float)copter.sweep.signal*copter.aparm.angle_max*g.acro_yaw_p;
                    target_yaw_rate = constrain_float(target_yaw_rate, -copter.aparm.angle_max*g.acro_yaw_p, copter.aparm.angle_max*g.acro_yaw_p);
                    //copter.tmp3 = target_yaw_rate;
                    break;
                case THROTTLE:
                    break;
            }
        }

        // call attitude controller
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);

        // adjust climb rate using rangefinder
        target_climb_rate = get_surface_tracking_climb_rate(target_climb_rate, pos_control->get_alt_target(), G_Dt);

        // get avoidance adjusted climb rate
        target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

        // inject throttle frequency input
        if (check_status() && copter.sweep.axis == THROTTLE) {
            if(copter.sweep.type == SWEEP) {
                run_frequency_sweep(copter.sweep_throttle);
            } else {
                run_doublet(copter.doublet_throttle);
            }
            target_climb_rate += (float)copter.sweep.signal*g.pilot_speed_up;
            target_climb_rate = constrain_float(target_climb_rate, -g.pilot_speed_up, g.pilot_speed_up);
            //copter.tmp3 = target_climb_rate;
        }

        // call position controller
        pos_control->set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
        pos_control->update_z_controller();
        break;
    }
}


// check if the frequency sweep is enabled
bool Copter::ModeAltHold::check_status()
{
    if (copter.sweep.status == START) {
        return true;
    } else {
        return false;
    }
}

// create the sinusoidal signal
void Copter::ModeAltHold::run_frequency_sweep(const freq_setting my_settings)
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
    float _T_wmin = (2.0f*MY_PI*my_settings.F_min);
    float _T_wmax = (2.0f*MY_PI*my_settings.F_max);
    float _T_tmin = (1.0f/my_settings.F_min);
    float _T_active = _T_total - _T_tmin;
    
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
    if (my_period < (_T_trimin+_T_tmin)*S_2_MILLIS) {
        copter.my_theta += _T_wmin*(my_dt*1e-3f);
    } else {
        my_K = MY_C2*(expf(MY_C1*(my_period*1e-3f-_T_tmin-_T_trimin)/_T_active)-1.0f);
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
void Copter::ModeAltHold::run_doublet(const doub_setting my_settings)
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