#ifndef CONTROLLER_FUNCTIONS_CONFIG_H
#define CONTROLLER_FUNCTIONS_CONFIG_H

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
#define throttleMin_DEF  0    // Throttle Down

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
static const float PROGMEM outputLimits[] = {
											25.0f, // percent of aileron servo
											40.0f, // percent of elevator servo
											20.0f, // percent of rudder servo
											0.21f,  // altitude controller ~10 degrees 
											35.0f, // throttle deviation maximum
											0.872f, // heading angle deviation maximum ~50 degrees
											3.0f   // airspeed maximum deviation (for ground speed controller)
										    };


//const float referenceLimits[numControllers][minMax] = {
//                                   {0.52 /* 30 degrees max */, -0.52 /* -30 degrees min */ },  /* roll controller */
//                                   {0.43 /* 24.7 degrees max */, -0.24 /* -14 degrees min */ },  /* pitch controller */
//                                   {0.0, 0.0}, /* rudder controller */
//                                   {120 /* 120 meters */, 0 /* 0 meters */ },  /* altitude controller */
//                                   {13.5 /* 13.5 m/s max */, 8.5 /* 8.5 m/s min */}, /* airspeed controller */
//                                   {6.30 /* 2 PI max */, -0.1 /* -0.1 min */}, /* heading controller */
//								   {3.0 /* 3 m/s max */, -3.0 /* -3.0 m/s min */} /* ground speed controller */ 
//                                  };

static const float PROGMEM referenceLimits[numControllers][minMax] = {
                                   {0.52f /* 30 degrees max */, -0.52f /* -30 degrees min */ },  /* roll controller */
                                   {0.43f /* 24.7 degrees max */, -0.24f /* -14 degrees min */ },  /* pitch controller */
                                   {0.0f, 0.0f}, /* rudder controller */
                                   {120.0f /* 120 meters */, 0.0f /* 0 meters */ },  /* altitude controller */
                                   {13.5f /* 13.5 m/s max */, 8.5f /* 8.5 m/s min */}, /* airspeed controller */
                                   {6.30f /* 2 PI max */, -0.1f /* -0.1 min */}, /* heading controller */
								   {3.0f /* 3 m/s max */, -3.0f /* -3.0 m/s min */} /* ground speed controller */ 
                                  };

static const float PROGMEM integralLimits[] = {
											  1.0f, /* roll controller */
											  1.0f, /* pitch controller */
											  1.0f, /* rudder controller */
											  0.05f, /* altitude controller */
											  5.0f, /* airspeed controller  */
											  0.05f, /* heading controller  */
											  0.5f /* ground speed controller */
											  };

static const float PROGMEM derivativeLimits[] = {
											   3.0f, /* roll controller (% rc out) */
											   3.0f, /* pitch controller (% rc out) */
											   3.0f, /* rudder controller (% rc out) */
											   0.05f, /* altitude controller (radians - 2.86 degrees) */
											   3.0f, /* airspeed controller (meters/second) */
											   0.05f, /* heading controller (radians - 2.86 degrees) */
											   0.5f /* ground speed controller (meters/second) */
											    };

static const float PROGMEM integralTermLimits[] = {
											   5.0f, /* roll controller percent aileron */
											   5.0f, /* pitch controller percent elevator */
											   5.0f, /* rudder controller percent rudder */
											   0.01f, /* altitude controller pitch angle (.57 degrees) */
											   5.0f, /* airspeed controller percent throttle */
											   0.052f, /* heading controller (3 degrees)  */
											   0.5f /* ground speed controller (m/s) */
											      };

static const float PROGMEM derivativeTermLimits[] = {
											   5.0f, /* roll controller */
											   5.0f, /* pitch controller */
											   5.0f, /* rudder controller */
											   1.0f, /* altitude controller */
											   5.0f, /* airspeed controller */
											   2.0f, /* heading controller */
											   5.0f /* ground speed controller */
											  };

/*------------------------------------ Controller Gains --------------------------------------------*/
#define pGain 0
#define iGain 1
#define dGain 2
#define numGains 3

static const float PROGMEM gains [numControllers][numGains] = {
                                          {25.0f, .5f, 0.0f}, /* roll controller p, i, d */
                                          {40.0f, 1.0f, 0.0f}, /* pitch controller p, i, d */
                                          {0.0f, 0.0f, 0.0f},  /* rudder controller p, i, d */
                                          {0.15f, 0.009f, 0.0f},  /* altitude controller p, i, d */                                        
                                          {12.0f, 0.05f, 0.0f},  /* airspeed controller p, i, d */
										  {0.7f, 0.007f, 0.0f},   /* heading controller p, i, d */ //{1.9, 0.002, 0.0},  for the quadratic
										  {1.0f, 0.01f, 0.0f}    /* ground speed controller p, i, d */
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
