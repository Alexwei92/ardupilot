#include "Copter.h"

#ifdef USERHOOK_INIT
void Copter::userhook_init()
{
    sweep.roll = 0.0f;
    sweep.pitch = 0.0f;
    sweep.yaw = 0.0f;
    sweep.throttle = 0.0f;
    sweep.amplitude = 0.0f;
    sweep.axis = ROLL;
    sweep.status = STANDBY;
   
    my_init_time = millis();
}
#endif

#ifdef USERHOOK_FASTLOOP
void Copter::userhook_FastLoop()
{
    // start the sweep 20 seconds after boot
    if (sweep.status == START) {
        uint32_t my_current_time = millis();
        start_frequency_sweep(my_current_time);
    } else {
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
    Log_Write_Sweep(sweep.roll,(uint8_t)sweep.axis,(uint8_t)sweep.status);
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
    Log_Write_ONR();
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
    float v = constrain_float((radio_in - rc6->get_radio_min()) / float(rc6->get_radio_max() - rc6->get_radio_min()), 0, 1); 
    float control_in = 0.0f + v * (0.4f - 0.0f);
    sweep.amplitude = constrain_float(control_in, 0.0f, 0.4f);
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
    switch (ch_flag) {
        case AUX_SWITCH_LOW:
        case AUX_SWITCH_MIDDLE:
            sweep.status = STANDBY;
            break;
        case AUX_SWITCH_HIGH:
            if (sweep.status == STANDBY && sweep.status != STOP) {
                gcs().send_text(MAV_SEVERITY_INFO,"Frequency sweep begin");
                sweep.status = START;
                my_start_time = my_last_time = millis();
            }
            break;
    }
}

void Copter::userhook_auxSwitch3(uint8_t ch_flag)
{
    // put your aux switch #3 handler here (CHx_OPT = 49)
}
#endif

void Copter::start_frequency_sweep(uint32_t my_current_time)
{
    my_dt = my_current_time - my_last_time;
    my_period = my_current_time - my_start_time;
    my_last_time = my_current_time;
    double my_delta = 0.0;
    double my_K = 0.0;

    // if time > (t_trimin+t_total+t_trimout), return 0 and set sweep.status to 3
    if ((my_period > (MY_TRIMIN+MY_TOTAL+MY_TRIMOUT)*S_2_MILLIS)) {
        my_delta = 0.0;
        sweep.roll = (float)my_delta;
        if (sweep.status == START) {
            gcs().send_text(MAV_SEVERITY_INFO, "Frequency sweep end");
            sweep.status = STOP;
        };
        return;
    }

    // if time < t_trimin or time > (t_trimin+t+t_total), return 0
    if ((my_period < MY_TRIMIN*S_2_MILLIS) || (my_period > (MY_TRIMIN+MY_TOTAL)*S_2_MILLIS)) {
        my_delta = 0.0;
        sweep.roll = (float)my_delta;
        return;
    }

    // else do the frequency sweep
    if (my_period < (MY_TRIMIN+MY_TMIN)*S_2_MILLIS) {
        my_theta += MY_WMIN*(my_dt*1e-3);      
    } else {
        my_K = MY_C2*(expf(MY_C1*(my_period*1e-3-MY_TMIN-MY_TRIMIN)/MY_TACTIVE)-1.0);
        my_theta += (MY_WMIN + my_K*(MY_WMAX-MY_WMIN))*(my_dt*1e-3);
    }
    my_delta = sweep.amplitude*sinf(my_theta);

    // Apply fadein and fadeout
    if (my_period < (MY_TRIMIN+MY_FADEIN)*S_2_MILLIS) {
        my_delta = my_delta*(my_period*1e-3-MY_TRIMIN)/MY_FADEIN;
    } else if (my_period > (MY_TRIMIN+MY_TOTAL-MY_FADEOUT)*S_2_MILLIS) {
        my_delta = my_delta*(MY_TRIMIN+MY_TOTAL-my_period*1e-3)/MY_FADEOUT;
    }
    sweep.roll = (float)my_delta;
    return;
}

void Copter::reset_frequency_sweep()
{
    sweep.roll = 0.0f;
    sweep.pitch = 0.0f;
    sweep.yaw = 0.0f;
    sweep.throttle = 0.0f;
}