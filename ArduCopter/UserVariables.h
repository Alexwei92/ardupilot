// user defined variables

// example variables used in Wii camera testing - replace with your own
// variables
#ifdef USERHOOK_VARIABLES
struct freq_sweep{
    float   roll;     
    float   pitch;  
    float   yaw;    
    float   throttle;    
    float   amplitude;
    uint8_t   axis;        // 0:roll, 1:pitch, 2:yaw, 3:throttle
    uint8_t   status;      // 0:STANDBY, 1:START, 2:STOP
};

#endif  // USERHOOK_VARIABLES


