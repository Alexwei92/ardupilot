#include "Copter.h"

#ifdef USERHOOK_INIT
void Copter::userhook_init()
{
    sweep.roll = 0.0f;
    sweep.pitch = 0.0f;
    sweep.yaw = 0.0f;
    sweep.throttle = 0.0f;
    sweep.amplitude = 0.0f;
    sweep.axis = 0;
    sweep.status = 0;
   
    my_init_time = millis();
}
#endif

#ifdef USERHOOK_FASTLOOP
void Copter::userhook_FastLoop()
{
    // start the sweep 20 seconds after boot
    if ((millis() - my_init_time) > 50*S_2_MILLIS) {
        if (sweep.status == 0) {
            my_start_time = millis();
            my_last_time = my_start_time;
            sweep.status = 1;
        }
        start_frequency_sweep();
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
    Log_Write_Sweep(sweep.roll,sweep.axis,sweep.status);
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
    // put your 3.3Hz code here
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
}

void Copter::userhook_auxSwitch2(uint8_t ch_flag)
{
    // put your aux switch #2 handler here (CHx_OPT = 48)
}

void Copter::userhook_auxSwitch3(uint8_t ch_flag)
{
    // put your aux switch #3 handler here (CHx_OPT = 49)
}
#endif

void Copter::start_frequency_sweep()
{
    my_current_time = millis();
    my_dt = my_current_time - my_last_time;
    my_period = my_current_time - my_start_time;
    my_last_time = my_current_time;

    // if time > (t_trimin+t_total+t_trimout), return 0 and set sweep.status to 3
    if ((my_period > (MY_TRIMIN+MY_TOTAL+MY_TRIMOUT)*S_2_MILLIS)) {
        my_delta = 0.0;
        sweep.roll = (float)my_delta;
        sweep.status = 2;
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
    my_delta = MY_A*sinf(my_theta);

    // Apply fadein and fadeout
    if (my_period < (MY_TRIMIN+MY_FADEIN)*S_2_MILLIS) {
        my_delta = my_delta*(my_period*1e-3-MY_TRIMIN)/MY_FADEIN;
    } else if (my_period > (MY_TRIMIN+MY_TOTAL-MY_FADEOUT)*S_2_MILLIS) {
        my_delta = my_delta*(MY_TRIMIN+MY_TOTAL-my_period*1e-3)/MY_FADEOUT;
    }
    sweep.roll = (float)my_delta;
    return;
}