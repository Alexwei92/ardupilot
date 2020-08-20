#include "Copter.h"

#ifdef USERHOOK_INIT
void Copter::userhook_init()
{
    // put your initialisation code here
    // this will be called once at start-up
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
    onr_power.battery_current = 0;
    onr_power.battery_voltage = 0;
    onr_power.bec_current = 0;
    onr_power.bec_voltage = 0;
    onr_power.none2 = 0;
    onr_power.none3 = 0;    
}
#endif

#ifdef USERHOOK_FASTLOOP
void Copter::userhook_FastLoop()
{
    // put your 100Hz code here
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
    // put your 10Hz code here
 #if ONR_DATAFLASH == ENABLED
    Log_Write_ONRRPM();
    Log_Write_ONRPOWER();
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
    // put your 1Hz code here
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
    onr_power.battery_current     = packet.battery_current;
    onr_power.battery_voltage     = packet.battery_voltage;
    onr_power.bec_current         = packet.bec_current;
    onr_power.bec_voltage         = packet.bec_voltage;
    onr_power.none2               = packet.none2;
    onr_power.none3               = packet.none3;
} 