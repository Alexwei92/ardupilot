// user defined variables

// example variables used in Wii camera testing - replace with your own
// variables
#ifdef USERHOOK_VARIABLES

#define MY_PI 3.141592653589793
#define MY_C1 4.0
#define MY_C2 0.0187
#define MY_A 1.0
#define MY_N 1
#define S_2_MILLIS 1000
#define S_2_MICROS 1000000

// in second
#define MY_FADEIN 10
#define MY_FADEOUT 2
#define MY_TRIMIN 2
#define MY_TRIMOUT 2
#define MY_TOTAL 30

#define MY_FMIN 0.1
#define MY_FMAX 10.0
#define MY_WMIN (2.0*MY_PI*MY_FMIN)
#define MY_WMAX (2.0*MY_PI*MY_FMAX)

#define MY_TMIN (1.0/MY_FMIN)*MY_N
#define MY_TACTIVE (MY_TOTAL-MY_TMIN)

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

struct freq_sweep{
    float   roll;     
    float   pitch;  
    float   yaw;    
    float   throttle;    
    float   amplitude;
    Axis    axis;   // 0:roll, 1:pitch, 2:yaw, 3:throttle
    Status  status; // 0:STANDBY, 1:START, 2:STOP
};

freq_sweep sweep;

uint32_t my_init_time = 0;
uint32_t my_start_time = 0;
//uint32_t my_current_time = 0;
uint32_t my_last_time = 0;
uint32_t my_period = 0;
uint32_t my_dt = 0;
double my_theta = 0.0;
//double my_delta = 0.0;
//double my_K = 0.0;

#endif  // USERHOOK_VARIABLES


