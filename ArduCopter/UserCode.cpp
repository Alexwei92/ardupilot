#include "Copter.h"

#ifdef USERHOOK_INIT
void Copter::userhook_init()
{
    sweep.signal = 0.0f;
    sweep.amplitude = 0.0f;
    sweep.axis = ROLL;
    sweep.status = STANDBY;
    sweep.type = SWEEP;

    onr_rpm.rpm1 = 0;
    onr_rpm.rpm2 = 0;
    onr_rpm.rpm3 = 0;
    onr_rpm.rpm4 = 0;
    onr_rpm.rpm5 = 0;
    onr_rpm.rpm6 = 0;
    onr_rpm.rpm7 = 0;
    onr_rpm.rpm8 = 0;

    onr_power.none1 = 0;
    onr_power.battery_temperature = 0;
    onr_power.battery_voltage = 0;
    onr_power.battery_current = 0;
    onr_power.bec_voltage = 0;
    onr_power.bec_current = 0;
    onr_power.none2 = 0;
    onr_power.none3 = 0;
}
#endif

#ifdef USERHOOK_FASTLOOP
void Copter::userhook_FastLoop()
{
    if (control_mode != SYS_ID && control_mode != ALT_HOLD && sweep.status == START) {
        gcs().send_text(MAV_SEVERITY_WARNING,"Must be in SYS_ID/ALT_HOLD flight mode");
        reset_frequency_sweep();
        sweep.status = STANDBY;
    } else if(sweep.status != START) {
        reset_frequency_sweep();
    }
        
 #if MIXERIN_DATAFLASH == ENABLED
    Log_Write_Mixerin();
 #endif
 #if STATES_DATAFLASH == ENABLED
    Log_Write_States();
    Log_Write_Accel();
 #endif
 #if RC_DATAFLASH == ENABLED
    DataFlash.Log_Write_RCIN();
    DataFlash.Log_Write_RCOUT();
 #endif
 #if SWEEP_DATAFLASH == ENABLED
    Log_Write_Sweep((float)sweep.signal,(uint8_t)sweep.axis,(uint8_t)sweep.status,(uint8_t)sweep.type);
 #endif
}
#endif

#ifdef USERHOOK_50HZLOOP
void Copter::userhook_50Hz()
{
    // put your 50Hz code here
}
#endif

#ifdef USERHOOK_MEDIUMLOOP
void Copter::userhook_MediumLoop()
{
 #if ONR_DATAFLASH == ENABLED
    Log_Write_ONRRPM();
    Log_Write_ONRPOWER();
 #endif
}
#endif

#ifdef USERHOOK_SLOWLOOP
void Copter::userhook_SlowLoop()
{
    RC_Channel *rc6 = RC_Channels::rc_channel(CH_6);

    // exit if tuning channel has been used, or radio failsafe
    if ((g.radio_tuning > 0) || failsafe.radio || failsafe.radio_counter != 0 || rc6->get_radio_in() == 0) {
        sweep.amplitude = 0.0f;
        sweep.status = STANDBY;
        return;
    }

    uint16_t radio_in = rc6->get_radio_in();
    float control_in = constrain_float((radio_in - rc6->get_radio_min()) / float(rc6->get_radio_max() - rc6->get_radio_min()), 0, 1); 
    //sweep.amplitude = control_in;
    if (control_in > 0.9f){
        if (sweep.status != START) {
            sweep.type = DOUBLET;
            sweep.amplitude = 0.5f;
        }
    } else {
        if (sweep.status != START || sweep.type == SWEEP){
            sweep.type = SWEEP;
            sweep.amplitude = control_in;
        }
    }
}
#endif

#ifdef USERHOOK_SUPERSLOWLOOP
void Copter::userhook_SuperSlowLoop()
{
    // 1Hz
}
#endif

#ifdef USERHOOK_AUXSWITCH
void Copter::userhook_auxSwitch1(uint8_t ch_flag)
{
    // put your aux switch #1 handler here (CHx_OPT = 47)
    // switch among roll, pitch, yaw axes
    if (sweep.status == START) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Change axis failed");
        return;
    }

    switch (ch_flag) {
        case AUX_SWITCH_LOW:
            sweep.axis = ROLL;
            break;
        case AUX_SWITCH_MIDDLE:
            sweep.axis = PITCH;
            break;
        case AUX_SWITCH_HIGH:
            sweep.axis = YAW;
            break;
    }
}

void Copter::userhook_auxSwitch2(uint8_t ch_flag)
{
    // put your aux switch #2 handler here (CHx_OPT = 48)
    // switch among the standby, start, stop status
    if (control_mode != SYS_ID && control_mode != ALT_HOLD) {
        return;
    }

    switch (ch_flag) {
        case AUX_SWITCH_LOW:
        case AUX_SWITCH_MIDDLE:
            sweep.status = STANDBY;
            break;
        case AUX_SWITCH_HIGH:
            if (sweep.status == STANDBY && sweep.status != STOP) {
                if (sweep.type == SWEEP) {
                    gcs().send_text(MAV_SEVERITY_INFO,"Frequency sweep begin");
                } else {
                    gcs().send_text(MAV_SEVERITY_INFO,"Doublet begin");
                }
                sweep.status = START;
                my_start_time = my_last_time = millis();
            }
            break;
    }
}

void Copter::userhook_auxSwitch3(uint8_t ch_flag)
{
    // put your aux switch #3 handler here (CHx_OPT = 49)
    // enable throttle axis
    switch (ch_flag) {
        case AUX_SWITCH_LOW:
        case AUX_SWITCH_MIDDLE:
            break;
        case AUX_SWITCH_HIGH:
            if (sweep.status == START) {
                gcs().send_text(MAV_SEVERITY_WARNING, "Change axis failed");
                return;
            } else {
                sweep.axis = THROTTLE;
            }
            break;
    }
}
#endif

void Copter::reset_frequency_sweep()
{
    sweep.signal = 0.0f;
    my_theta = 0.0f;
}

void Copter::handle_onr_rpm_msg(const mavlink_message_t* msg)
{
    __mavlink_onr_rpm_sensor_t packet;
    mavlink_msg_onr_rpm_sensor_decode(msg, &packet);
    onr_rpm.rpm1 = packet.rpm1;
    onr_rpm.rpm2 = packet.rpm2;
    onr_rpm.rpm3 = packet.rpm3;
    onr_rpm.rpm4 = packet.rpm4;
    onr_rpm.rpm5 = packet.rpm5;
    onr_rpm.rpm6 = packet.rpm6;
    onr_rpm.rpm7 = packet.rpm7;
    onr_rpm.rpm8 = packet.rpm8;
}

void Copter::handle_onr_power_msg(const mavlink_message_t* msg)
{
    __mavlink_onr_power_sensor_t packet;
    mavlink_msg_onr_power_sensor_decode(msg, &packet);
    onr_power.none1               = packet.none1;
    onr_power.battery_temperature = packet.battery_temperature;
    onr_power.battery_voltage     = packet.battery_voltage;
    onr_power.battery_current     = packet.battery_current;
    onr_power.bec_voltage         = packet.bec_voltage;
    onr_power.bec_current         = packet.bec_current;
    onr_power.none2               = packet.none2;
    onr_power.none3               = packet.none3;
}