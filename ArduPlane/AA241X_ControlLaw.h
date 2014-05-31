#ifndef CONTROL_LAW_H
#define CONTROL_LAW_H

// NOTE: the parameter name has to have less than 14 characters! If not you get the error "initialize-string for array chars is too long"
// My parameter 1: Maximum error in waypoint capture
#define POSITION_ERROR    g.aa241x_1      // Don't change g.aa241x_1 ! Replace my_param_1 with whatever name you want to access the parameter with.  See the example in ControlLaw.ino   
#define AA241X_1_NAME     "AA241_Pos_Err" // Don't change AA241X_1_NAME ! Replace my_paramName1 with whateve name you want to see on the Mission Planner GCS 
#define AA241X_1_DEFAULT  10.0             // replace 10.0 with whatever default value you wante to this parameter to have, when reloading the code 
// My parameter 2: Maximum error in snapshot capture
#define SNAPSHOT_ERROR    g.aa241x_2
#define AA241X_2_NAME     "AA241_Snp_Err" 
#define AA241X_2_DEFAULT  5.0
// My parameter 3: Proportional gain on line tracking, "ROUTE_P = 1" means set heading to waypoint
#define ROUTE_P           g.aa241x_3
#define AA241X_3_NAME     "AA241_Rte_P" 
#define AA241X_3_DEFAULT  1.0
// My parameter 4
#define FLIGHT_MODE       g.aa241x_4
#define AA241X_4_NAME     "AA241_FltMode" 
#define AA241X_4_DEFAULT  1.0 
// My parameter 5
#define ALTITUDE_P        g.aa241x_5
#define AA241X_5_NAME     "AA241_Alt_P" 
#define AA241X_5_DEFAULT  0.1 
// My parameter 6
#define CLIMB_RATE_P      g.aa241x_6
#define AA241X_6_NAME     "AA241_Clmb_P" 
#define AA241X_6_DEFAULT  1.0 
// My parameter 7
#define AIRSPEED_P        g.aa241x_7
#define AA241X_7_NAME     "AA241_Spd_P" 
#define AA241X_7_DEFAULT  10.0 
// My parameter 8
#define MAX_CLIMB_PITCH   g.aa241x_8
#define AA241X_8_NAME     "AA241_MaxClmb" 
#define AA241X_8_DEFAULT  0.349
// My parameter 9
#define GLIDE_PITCH        g.aa241x_9
#define AA241X_9_NAME     "AA241_GldPtch" 
#define AA241X_9_DEFAULT   0.122
// My parameter 10
#define INIT_SPIRAL        g.aa241x_10
#define AA241X_10_NAME     "AA241_InitSpl" 
#define AA241X_10_DEFAULT  0.0
// My parameter 11
#define INIT_SIMPLE        g.aa241x_11
#define AA241X_11_NAME     "AA241_InitSim" 
#define AA241X_11_DEFAULT  0.0
// My parameter 12
#define my_param_12        g.aa241x_12
#define AA241X_12_NAME     "my_paramName12" 
#define AA241X_12_DEFAULT  -20.523 
// My parameter 13
#define my_param_13        g.aa241x_13
#define AA241X_13_NAME     "my_paramName13" 
#define AA241X_13_DEFAULT  10.0 
// My parameter 14
#define my_param_14        g.aa241x_14
#define AA241X_14_NAME     "my_paramName14" 
#define AA241X_14_DEFAULT  -20.523 
// My parameter 15
#define my_param_15        g.aa241x_15
#define AA241X_15_NAME     "my_paramName15" 
#define AA241X_15_DEFAULT  10.0 
// My parameter 16
#define my_param_16        g.aa241x_16
#define AA241X_16_NAME     "my_paramName16" 
#define AA241X_16_DEFAULT  -20.523 
// My parameter 17
#define my_param_17        g.aa241x_17
#define AA241X_17_NAME     "my_paramName17" 
#define AA241X_17_DEFAULT  10.0 
// My parameter 18
#define my_param_18        g.aa241x_18
#define AA241X_18_NAME     "my_paramName18" 
#define AA241X_18_DEFAULT  -20.523 
// My parameter 19
#define my_param_19        g.aa241x_19
#define AA241X_19_NAME     "my_paramName19" 
#define AA241X_19_DEFAULT  10.0 
// My parameter 20
#define my_param_20        g.aa241x_20
#define AA241X_20_NAME     "my_paramName20" 
#define AA241X_20_DEFAULT  -20.523 

#endif /* CONTROL_LAW_H */
