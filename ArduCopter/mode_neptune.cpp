#include "Copter.h"

#define MAX_SPEED 1000.0f    // maximum forward speed in cm/s

// modeneptune init
bool ModeNeptune::init(bool ignore_checks)
{
    // if landed and the mode we're switching from does not have manual throttle
    if (motors->armed() && copter.ap.land_complete && !copter.flightmode->has_manual_throttle()) {
        return false;
    }

    // radio tunning check
    if (copter.g.radio_tuning != TUNING_FORWARD_SPEED) {
        gcs().send_text(MAV_SEVERITY_INFO, "CH6 not Activated"); // If the CH6 knob is not activated
        return false;
    } else if (!is_zero(forward_speed)) {
        gcs().send_text(MAV_SEVERITY_INFO, "CH6 not at Zero"); // If the forward speed from CH6 is not starting at 0
        //return false;
    }

    // send message to GCS
    gcs().send_text(MAV_SEVERITY_INFO, "Custom Mode Start");

    // initialize vertical speeds and acceleration
    pos_control->set_max_speed_z(-get_pilot_speed_dn(), g.pilot_speed_up);
    pos_control->set_max_accel_z(g.pilot_accel_z);

    // initialise position and desired velocity
    if (!pos_control->is_active_z()) {
        pos_control->set_alt_target_to_current_alt();
        pos_control->set_desired_velocity_z(inertial_nav.get_velocity_z());
    }

    return true;
}

// run the custom controller
// should be called at 100hz or more
void ModeNeptune::run()
{
    // if not armed set throttle to zero and exit immediately
    if (is_disarmed_or_landed()) {
        make_safe_spool_down();
        return;
    }

    // get pilot desired climb rate
    // float target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
    float target_climb_rate = 0.0f;
    target_climb_rate = constrain_float(target_climb_rate, -get_pilot_speed_dn(), g.pilot_speed_up);
    
    // adjust climb rate using rangefinder
    if (copter.rangefinder_alt_ok()) {
        // if rangefinder is ok, use surface tracking
        target_climb_rate = copter.surface_tracking.adjust_climb_rate(target_climb_rate);
    }

    pos_control->set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
    
    // set forward speed from CH6 reading
    forward_speed = constrain_float(forward_speed, 0.0f, MAX_SPEED);

    float latitude_speed  = forward_speed;
    float longitude_speed = 0.0f; 

    copter.rotate_body_frame_to_NE(latitude_speed, longitude_speed);
    pos_control->set_desired_velocity_xy(latitude_speed, longitude_speed);

    // run position controller
    pos_control->update_xy_controller();
    pos_control->update_z_controller();

    // get pilot's desired yaw rate
    float target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());

    // call attitude controller
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(pos_control->get_roll(), pos_control->get_pitch(), target_yaw_rate);
}

//TODO: data logging
// void ModeNeptune::log_data()
// {
// }