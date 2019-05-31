// user defined variables

// example variables used in Wii camera testing - replace with your own
// variables
#ifdef USERHOOK_VARIABLES

// useful paramters
#define MY_PI 3.141592653589793f
#define MY_C1 4.0f
#define MY_C2 0.0187f
#define S_2_MILLIS (uint32_t)1000
#define S_2_MICROS (uint32_t)1000000

enum Axis {
    ROLL = 0,
    PITCH = 1,
    YAW = 2,
    THROTTLE = 3,
};

enum Status {
    STANDBY = 0,
    START = 1,
    STOP = 2,
};

enum Type {
    SWEEP = 0,
    DOUBLET = 1,
};

struct sweep_monitor {
    float   signal;     // raw signal, range from -1 to 1
    float   amplitude;  // raw amplitude, rannge from 0 to 1
    Axis    axis;       // 0:roll, 1:pitch, 2:yaw, 3:throttle
    Status  status;     // 0:STANDBY, 1:START, 2:STOP
    Type    type;       // 0:frequency sweep, 1:doublet
};

struct freq_setting {
    uint8_t T_fadein;
    uint8_t T_fadeout;
    uint8_t T_trimin;
    uint8_t T_trimout;
    uint8_t T_total;
    float F_min;
    float F_max;
    float A_min;
    float A_max;
};

struct doub_setting {
    uint8_t T_trimin;
    uint8_t T_trimout;
    uint8_t T_pulse;
    float A_max;
};

sweep_monitor sweep;

// follow this format             = {T_fadein, T_fadeout, T_trimin, T_trimout, T_total,  F_min,  F_max,  A_min,  A_max}
const freq_setting sweep_roll     = {10,       2,         2,        2,         30,       0.1f,   10.0f,  0.0f,   0.4f}; //0.4
const freq_setting sweep_pitch    = {10,       2,         2,        2,         30,       0.1f,   10.0f,  0.0f,   0.4f}; //0.5
const freq_setting sweep_yaw      = {10,       2,         2,        2,         30,       0.1f,   10.0f,  0.0f,   0.3f};//0.38
const freq_setting sweep_throttle = {10,       2,         2,        2,         30,       0.1f,   10.0f,  0.0f,   0.4f}; //0.5

// follow this format               = {T_trimin, T_trimout, T_pulse, A_max}
const doub_setting doublet_roll     = {1,        2,         1,       0.4f};
const doub_setting doublet_pitch    = {1,        2,         1,       0.4f};
const doub_setting doublet_yaw      = {1,        2,         1,       0.3f};
const doub_setting doublet_throttle = {1,        2,         1,       0.4f};

// variables used to calculate frequency sweep
uint32_t my_start_time = 0;   // time when start the frequency sweep
uint32_t my_last_time = 0;    // to calculate dt
uint32_t my_period = 0;       // time elapsed after frequency sweep
float my_theta = 0.0f;
float tmp1 = 0.0f;
float tmp2 = 0.0f;
float tmp3 = 0.0f;


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


