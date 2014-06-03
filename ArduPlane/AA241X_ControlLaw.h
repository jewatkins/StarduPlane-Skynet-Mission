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
#define TEST_AIRSPEED     g.aa241x_5
#define AA241X_5_NAME     "AA241_Airspd" 
#define AA241X_5_DEFAULT  11.0
// My parameter 6
#define TEST_PITCH        g.aa241x_6
#define AA241X_6_NAME     "AA241_Pitch" 
#define AA241X_6_DEFAULT  0.0 
// My parameter 7
#define TEST_ROLL         g.aa241x_7
#define AA241X_7_NAME     "AA241_Roll" 
#define AA241X_7_DEFAULT  0.0 
// My parameter 8
#define CLIMB_PITCH_TEST  g.aa241x_8
#define AA241X_8_NAME     "AA241_ClmbTst" 
#define AA241X_8_DEFAULT  0.349
// My parameter 9
#define PERSON_4_Y         g.aa241x_9
#define AA241X_9_NAME      "AA241_Prsn4Y" 
#define AA241X_9_DEFAULT   0.0
// My parameter 10
#define INIT_SPIRAL        g.aa241x_10
#define AA241X_10_NAME     "AA241_InitSpl" 
#define AA241X_10_DEFAULT  0.0
// My parameter 11
#define INIT_PHASE2        g.aa241x_11
#define AA241X_11_NAME     "AA241_InitPh2" 
#define AA241X_11_DEFAULT  0.0
// My parameter 12
#define INITIAL_ERROR      g.aa241x_12
#define AA241X_12_NAME     "AA241_InitErr" 
#define AA241X_12_DEFAULT  20.0
// My parameter 13
#define my_param_13        g.aa241x_13
#define AA241X_13_NAME     "AA241_Prsn4X" 
#define AA241X_13_DEFAULT  0.0 
// My parameter 14
#define PERSON_3_Y         g.aa241x_14
#define AA241X_14_NAME     "AA241_Prsn3Y" 
#define AA241X_14_DEFAULT  0.0 
// My parameter 15
#define PERSON_3_X         g.aa241x_15
#define AA241X_15_NAME     "AA241_Prsn3X" 
#define AA241X_15_DEFAULT  0.0 
// My parameter 16
#define PERSON_2_Y         g.aa241x_16
#define AA241X_16_NAME     "AA241_Prsn2Y" 
#define AA241X_16_DEFAULT  0.0 
// My parameter 17
#define PERSON_2_X         g.aa241x_17
#define AA241X_17_NAME     "AA241_Prsn2X" 
#define AA241X_17_DEFAULT  0.0 
// My parameter 18
#define PERSON_1_Y         g.aa241x_18
#define AA241X_18_NAME     "AA241_Prsn1Y" 
#define AA241X_18_DEFAULT  0.0 
// My parameter 19
#define PERSON_1_X         g.aa241x_19
#define AA241X_19_NAME     "AA241_Prsn1X" 
#define AA241X_19_DEFAULT  0.0 
// My parameter 20
#define T_SIGHT            g.aa241x_20
#define AA241X_20_NAME     "AA241_tSight" 
#define AA241X_20_DEFAULT  0.0

#endif /* CONTROL_LAW_H */
