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

struct sweep_monitor {
    float   signal;     // raw signal, range from -1 to 1
    float   amplitude;  // raw amplitude, rannge from 0 to 1
    Axis    axis;       // 0:roll, 1:pitch, 2:yaw, 3:throttle
    Status  status;     // 0:STANDBY, 1:START, 2:STOP
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

sweep_monitor sweep;

// follow this format             = {T_fadein, T_fadeout, T_trimin, T_trimout, T_total,  F_min,  F_max,  A_min,  A_max}
const freq_setting sweep_roll     = {10,       2,         2,        2,         30,       0.1f,   10.0f,  0.0f,   0.4f};
const freq_setting sweep_pitch    = {10,       2,         2,        2,         30,       0.1f,   10.0f,  0.0f,   0.4f};
const freq_setting sweep_yaw      = {10,       2,         2,        2,         30,       0.1f,   10.0f,  0.0f,   0.4f};
const freq_setting sweep_throttle = {10,       2,         2,        2,         30,       0.1f,   10.0f,  0.0f,   0.2f};

// variables used to calculate frequency sweep
uint32_t my_start_time = 0;   // time when start the frequency sweep
uint32_t my_last_time = 0;    // to calculate dt
uint32_t my_period = 0;       // time elapsed after frequency sweep
float my_theta = 0.0f;
float tmp1 = 0.0f;
float tmp2 = 0.0f;
float tmp3 = 0.0f;

#endif  // USERHOOK_VARIABLES


