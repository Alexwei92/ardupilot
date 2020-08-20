// user defined variables

// example variables used in Wii camera testing - replace with your own
// variables
#ifdef USERHOOK_VARIABLES

#if WII_CAMERA == 1
WiiCamera           ircam;
int                 WiiRange=0;
int                 WiiRotation=0;
int                 WiiDisplacementX=0;
int                 WiiDisplacementY=0;
#endif  // WII_CAMERA

// structure for ONR data logging
struct ONR_RPM {
    uint32_t rpm1;
    uint32_t rpm2;
    uint32_t rpm3;
    uint32_t rpm4;
    uint32_t rpm5;
    uint32_t rpm6;
    uint32_t rpm7;
    uint32_t rpm8;
};

struct ONR_POWER {
    uint32_t none1;
    uint32_t battery_temperature;
    uint32_t battery_current;
    uint32_t battery_voltage;
    uint32_t bec_current;
    uint32_t bec_voltage;
    uint32_t none2;
    uint32_t none3;
};

ONR_RPM onr_rpm;
ONR_POWER onr_power;

#endif  // USERHOOK_VARIABLES


