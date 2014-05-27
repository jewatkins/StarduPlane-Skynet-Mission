#ifndef CONTROLLER_FUNCTIONS_CONFIG_H
#define CONTROLLER_FUNCTIONS_CONFIG_H

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
#define throttleMin_DEF  20   // Throttle Down

/*------------------------------------ Controllers -------------------------------------------------*/
#define rollController_DEF         0
#define pitchController_DEF        1
#define rudderController_DEF       2
#define altitudeController_DEF     3
#define climbRateController_DEF    4
#define glideController_DEF        5
#define airspeedController_DEF     6
#define headingController_DEF      7
#define numControllers             8

/*------------------------------------ Inner Loop Limits ------------------------------------------*/
#define maximum_DEF 0
#define minimum_DEF 1
#define minMax      2

// These are the absolute values of the controller deviations (saturates the controller at a limit based on the units of the controller)
const float outputLimits[numControllers] = {
											25, // percent of aileron servo
											20, // percent of elevator servo
											20, // percent of rudder servo
											3,  // altitude controller m/s
											0.05, // pitch angle deviation maximum ~3 degrees (radians)
											0.174, // pitch angle deviation maximum (radians)
											25, // throttle deviation maximum
											25 // heading angle deviation maximum
										   };

const float referenceLimits[numControllers][minMax] = {
                                   {0.52 /* 30 degrees max */, -0.52 /* -30 degrees min */ },  /* roll controller */
                                   {0.35 /* 20 degrees max */, -0.122 /* -7 degrees min */ },  /* pitch controller */
                                   {0.0, 0.0}, /* rudder controller */
                                   {120 /* 120 meters */, 0 /* 0 meters */ },  /* altitude controller */
                                   {3.0 /* 3.0 m/s max */, -3.0 /* 0.0 m/s min */ }, /* climb rate controller */
                                   {0.122 /* 7 degrees max */, 0.0 /* 0.0 degrees min */ }, /* glide controller */
                                   {12.0 /* 12 m/s max */, 6.0 /* 6.0 m/s min */}, /* airspeed controller */
                                   {6.30 /* 2 PI max */, -0.1 /* -0.1 min */} /* heading controller */
                                  };

const float integralLimits[numControllers] = {
											  1, /* roll controller */
											  1, /* pitch controller */
											  1, /* rudder controller */
											  0.5, /* altitude controller */
											  .01, /* climb rate controller */
											  1, /* glide controller */
											  10, /* airspeed controller */
											  1 /* heading controller */
											 };

const float derivativeLimits[numControllers] = {
											   3, /* roll controller */
											   3, /* pitch controller */
											   3, /* rudder controller */
											   0.5, /* altitude controller */
											   .01, /* climb rate controller */
											   3, /* glide controller */
											   3, /* airspeed controller */
											   3 /* heading controller */
											  };

const float integralTermLimits[numControllers] = {
											   5, /* roll controller */
											   5, /* pitch controller */
											   5, /* rudder controller */
											   0.5, /* altitude controller */
											   .01, /* climb rate controller */
											   5, /* glide controller */
											   5, /* airspeed controller */
											   5 /* heading controller */
											  };

const float derivativeTermLimits[numControllers] = {
											   5, /* roll controller */
											   5, /* pitch controller */
											   5, /* rudder controller */
											   0.5, /* altitude controller */
											   .01, /* climb rate controller */
											   5, /* glide controller */
											   5, /* airspeed controller */
											   5 /* heading controller */
											  };

/*------------------------------------ Controller Gains --------------------------------------------*/
#define pGain 0
#define iGain 1
#define dGain 2
#define numGains 3

float gains [numControllers][numGains] = {
                                          {25.0, .5, 0.0}, /* roll controller p, i, d */
                                          {40.0, 1.0, 0.0}, /* pitch controller p, i, d */
                                          {0.0, 0.0, 0.0},  /* rudder controller p, i, d */
                                          {0.1, 0.0, 0.0},  /* altitude controller p, i, d */
                                          {0.5, 0.0, 0.0},  /* climb rate controller p, i, d */
                                          {1.0, 0.0, 0.0},  /* glide controller p, i, d */                                          
                                          {10.0, 0.1, 0.0},  /* airspeed controller p, i, d */
                                          {1.0, 0.01, 0.0}   /* heading controller p, i, d */
                                         };

/*------------------------------------- Trim States ------------------------------------------------*/
/* Control state nominal outputs for a given commanded state:
 * airspeed
 * pitch angle
 * roll angle
 * States
 */

#define SEVEN_MPS_PITCH_DEF   0.1308f     // 7.5 degrees
#define TWELVE_MPS_PITCH_DEF  0.0f        // 0 degrees
#define PITCH_TRIM_SLOPE_DEF  -0.02267f   // -1.3 degrees per mps

#define PITCH_TRIM_BANK_MAX_DEF   0.05233f    // 3 degrees added to pitch for a maximum bank angle

#define MAX_CLIMB_RATE_DEF   3.0f  // 3 m/s
#define MIN_CLIMB_RATE_DEF   -3.0f // -3 m/s

#define MAX_CLIMB_RATE_PITCH_DEF 0.2616f  // 15 degrees maximum
#define MIN_CLIMB_RATE_PITCH_DEF -0.1221f // -7 degrees minimum

/*
#define SEVEN_MPS_DEF                 0
#define EIGHT_MPS_DEF                 1
#define NINE_MPS_DEF                  2
#define TEN_MPS_DEF                   3
#define ELEVEN_MPS_DEF                4
#define TWELVE_MPS_DEF                5
#define MAX_CLIMB_DEF                 6
#define GLIDE_DEF                     7
#define MAX_BANK_RIGHT_SEVEN_MPS_DEF  8
#define MAX_BANK_LEFT_SEVEN_MPS_DEF   9
#define MAX_BANK_RIGHT_TWELVE_MPS_DEF 10
#define MAX_BANK_LEFT_TWELVE_MPS_DEF  11
#define MID_BANK_RIGHT_SEVEN_MPS_DEF  12
#define MID_BANK_LEFT_SEVEN_MPS_DEF   13
#define MID_BANK_RIGHT_TWELVE_MPS_DEF 14
#define MID_BANK_LEFT_TWELVE_MPS_DEF  15
#define numTrims                      16

#define airspeed_DEF 0
#define pitch_DEF    1
#define roll_DEF     2
#define numStates    3

const float trims [numTrims][numStates] = {
                              {7.0, (6.5/180)*(3.14), 0.0}, // Straight and Level 7 m/s flight
                              {8.0, (5.0/180)*(3.14), 0.0}, // Straight and Level 8 m/s flight 
                              {9.0, (3.5/180)*(3.14), 0.0}, // Straight and Level 9 m/s flight 
                              {10.0, (2.0/180)*(3.14), 0.0}, // Straight and Level 10 m/s flight 
                              {11.0, (0.5/180)*(3.14), 0.0}, // Straight and Level 11 m/s flight 
                              {12.0, 0.0, 0.0}, // Straight and Level 12 m/s flight 
                              {7.0, (20.0/180)*(3.14), 0.0}, // Max Climb Rate 
                              {7.0, (6.5/180)*(3.14), 0.0}, // Glide 
                              {7.0, (7.5/180)*(3.14), referenceLimits[rollController_DEF][maximum_DEF]}, // Max Right Bank 7 m/s 
                              {7.0, (7.5/180)*(3.14), referenceLimits[rollController_DEF][minimum_DEF]}, // Max Left Bank 7 m/s 
                              {12.0, (1.5/180)*(3.14), referenceLimits[rollController_DEF][maximum_DEF]}, // Max Right Bank 12 m/s 
                              {12.0, (1.5/180)*(3.14), referenceLimits[rollController_DEF][minimum_DEF]}, // Max Left Bank 12 m/s 
                              {7.0, (7.0/180)*(3.14), referenceLimits[rollController_DEF][maximum_DEF]/2.0}, // Mid Right Bank 7 m/s 
                              {7.0, (7.0/180)*(3.14), referenceLimits[rollController_DEF][minimum_DEF]/2.0}, // Mid Left Bank 7 m/s 
                              {12.0, (1.0/180)*(3.14), referenceLimits[rollController_DEF][maximum_DEF]/2.0}, // Mid Right Bank 12 m/s 
                              {12.0, (1.0/180)*(3.14), referenceLimits[rollController_DEF][minimum_DEF]/2.0} // Mid Left Bank 12 m/s 
                             };
							 */

/*----------------------------------------- Phase of Flight -------------------------------------------------*/
/* The phase of flight will help determine the trim schedule. There are a finite number of flight phases for this
 * mission.
 *
 */

#define	preMissionLoiter  0 // Hang out around 90 feet waiting for the mission start event
#define climb             1 // Climb to the maximum altitude to get the widest field of view with the camera
#define	tSight            2 // Cruise at highest altitude until all persons have been seen
#define refinement        3 // Algorithm based waypoint nav (still in cruise mode)
#define	glide             4 // Battery power is running low, lose altitude and refine as much as possible
#define	postMissionLoiter 5 // Hang out and wait for pilot to switch back into manual mode

#endif /* CONTROLLER_FUNCTIONS_CONFIG_H */