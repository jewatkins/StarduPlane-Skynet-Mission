#ifndef CONTROLLER_FUNCTIONS_CONFIG_H
#define CONTROLLER_FUNCTIONS_CONFIG_H

//#include <avr/pgmspace.h>
#include <avr/pgmspace.h>

// This file defines all of the configuration parameters for the inner loop controllers

/*************************** Mechanical Limit Variables ***********************************************

Notes about the PWM signal commands, there doesn't seem to be a more appropriate place to put these
comments in anywhere but here at this point. The commanded signal to control the PWM signals going to
each of the servos is produced here in this loop. The command is given in values of percentage, which
are ultimately translated into the PWM duty cycle by the existing StarduPilot code. The chXout variables
given in the "Status" window of the Mission Planner are mapped linearly from the percentages commanded
in the AUTO_FastLoop function.

For each output, the 0% (0 command) is mapped to a corresponding value of 800 on the chXout menu. The
100% (100 command) is mapped to a corresponding value of 2200 on the chXmenu. The output of the menu
is cut of at 900 and 2100 for reasons unknown, but the middle value is 1500 and for each integer increase
in the value of the commanded percentage, a value of 14 is added to the chXout value.

Each control surface has mechanical limits that should not be exceeded by either the RC controller (pilot
input) or the automatic flight system. These mechanical limits are what is defined in this section.

--------------------------------------- Skynet Mechanical Limits ----------------------------------*/
#define pitchMax_DEF     100  // Elevator Down
#define pitchMin_DEF     0    // Elevator Up
#define rollMax_DEF      100  // Roll Left Aileron
#define rollMin_DEF      0    // Roll Right Aileron
#define rudderMax_DEF    100  // Left Rudder
#define rudderMin_DEF    0    // Right Rudder
#define throttleMax_DEF  95   // Throttle Up
#define throttleMin_DEF  5    // Throttle Down

/*------------------------------------ Controllers -------------------------------------------------*/
#define rollController_DEF         0
#define pitchController_DEF        1
#define rudderController_DEF       2
#define altitudeController_DEF     3
#define airspeedController_DEF     4
#define headingController_DEF      5
#define groundSpeedController_DEF  6
#define numControllers             7

/*------------------------------------ Inner Loop Limits ------------------------------------------*/
#define maximum_DEF 0
#define minimum_DEF 1
#define minMax      2

// These are the absolute values of the controller deviations (saturates the controller at a limit based on the units of the controller)
const float outputLimits[numControllers] = {
											25, // percent of aileron servo
											20, // percent of elevator servo
											20, // percent of rudder servo
											.08,  // altitude controller ~4.5 degrees 
											35, // throttle deviation maximum
											0.872, // heading angle deviation maximum ~50 degrees
											3   // airspeed maximum deviation
										   };

const float referenceLimits[numControllers][minMax] = {
                                   {0.52 /* 30 degrees max */, -0.52 /* -30 degrees min */ },  /* roll controller */
                                   {0.43 /* 24.7 degrees max */, -0.122 /* -7 degrees min */ },  /* pitch controller */
                                   {0.0, 0.0}, /* rudder controller */
                                   {120 /* 120 meters */, 0 /* 0 meters */ },  /* altitude controller */
                                   {14.0 /* 14 m/s max */, 7.0 /* 7.0 m/s min */}, /* airspeed controller */
                                   {6.30 /* 2 PI max */, -0.1 /* -0.1 min */}, /* heading controller */
								   {3.0 /* 3 m/s max */, -3.0 /* -3.0 m/s min */} /* ground speed controller */ 
                                  };

const float integralLimits[numControllers] PROGMEM = {
											  1, /* roll controller */
											  1, /* pitch controller */
											  1, /* rudder controller */
											  .05, /* altitude controller */
											  5, /* airspeed controller */
											  .05, /* heading controller */
											  .5 /* ground speed controller */
											 };

const float derivativeLimits[numControllers] PROGMEM = {
											   3, /* roll controller (% rc out) */
											   3, /* pitch controller (% rc out) */
											   3, /* rudder controller (% rc out) */
											   .05, /* altitude controller (meters) */
											   3, /* airspeed controller (meters/second) */
											   .05, /* heading controller (radians) */
											   .5 /* ground speed controller (meters/second) */
											  };

const float integralTermLimits[numControllers] PROGMEM = {
											   5, /* roll controller */
											   5, /* pitch controller */
											   5, /* rudder controller */
											   0.01, /* altitude controller */
											   5, /* airspeed controller */
											   .052, /* heading controller */
											   .5 /* ground speed controller */
											  };

const float derivativeTermLimits[numControllers] PROGMEM = {
											   5, /* roll controller */
											   5, /* pitch controller */
											   5, /* rudder controller */
											   0.01, /* altitude controller */
											   5, /* airspeed controller */
											   .052, /* heading controller */
											   .5 /* ground speed controller */
											  };

/*------------------------------------ Controller Gains --------------------------------------------*/
#define pGain 0
#define iGain 1
#define dGain 2
#define numGains 3

static float gains [numControllers][numGains] = {
                                          {25.0, .5, 0.0}, /* roll controller p, i, d */
                                          {40.0, 1.0, 0.0}, /* pitch controller p, i, d */
                                          {0.0, 0.0, 0.0},  /* rudder controller p, i, d */
                                          {0.1, 0.007, 0.0},  /* altitude controller p, i, d */                                        
                                          {12.0, 0.05, 0.0},  /* airspeed controller p, i, d */
										  {1.05, 0.007, 0.0},   /* heading controller p, i, d */ //{1.9, 0.002, 0.0},  for the quadratic
										  {1.0, 0.01, 0.0}    /* ground speed controller p, i, d */
                                         };

/*------------------------------------- Trim States ------------------------------------------------*/
/* Control state nominal outputs for a given commanded state:
 * airspeed
 * pitch angle
 * roll angle
 * States
 */

#define SEVEN_MPS_PITCH_DEF   0.1744f     // 7.5 degrees
#define TWELVE_MPS_PITCH_DEF  0.0f        // 0 degrees
#define PITCH_TRIM_SLOPE_DEF  -0.034f   // -2.0 degrees per mps

#define PITCH_TRIM_BANK_MAX_DEF   0.05233f    // 3 degrees added to pitch for a maximum bank angle

#define MAX_CLIMB_RATE_DEF   3.0f  // 3 m/s
#define MIN_CLIMB_RATE_DEF   -3.0f // -3 m/s

#define MAX_CLIMB_RATE_PITCH_DEF 0.2616f  // 15 degrees maximum
#define MIN_CLIMB_RATE_PITCH_DEF -0.1221f // -7 degrees minimum


#endif /* CONTROLLER_FUNCTIONS_CONFIG_H */
