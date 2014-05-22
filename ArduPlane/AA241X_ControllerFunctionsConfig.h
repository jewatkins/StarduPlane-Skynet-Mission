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
#define throttleMax_DEF  100  // Throttle Up
#define throttleMin_DEF  0    // Throttle Down

/*------------------------------------ Inner Loop Limits ------------------------------------------*/
const float bankAngleMax       = .52;     // 30 degrees max
const float bankAngleMin       = -.52;    // -30 degrees min
const float pitchAngleMax      = .35;     // 20 degrees pitch
const float pitchAngleMin      = -0.122;  // -7 degrees pitch
const float airspeedCommandMax = 12;      // 12 meters / second
const float airspeedCommandMin = 5;       // 5 meters / second


/*------------------------------------ Controllers -------------------------------------------------*/
#define rollController_DEF     0
#define pitchController_DEF    1
#define rudderController_DEF   2
#define altitudeController_DEF 3
#define airspeedController_DEF 4
#define headingController_DEF  5
#define numControllers         6


/*------------------------------------ Controller Gains --------------------------------------------*/
#define pGain 0
#define iGain 1
#define dGain 2
#define numGains 3

float gains [numControllers][numGains] = {
                                          {25.0, 0.0, 0.0}, /* roll controller p, i, d */
                                          {40.0, 0.0, 0.0}, /* pitch controller p, i, d */
                                          {0.0, 0.0, 0.0},  /* rudder controller p, i, d */
                                          {1.0, 0.0, 0.0},  /* altitude controller p, i, d */
                                          {1.0, 0.0, 0.0},  /* airspeed controller p, i, d */
                                          {2.0, 0.0, 0.0}   /* heading controller p, i, d */
                                         };

/*------------------------------------- Trim States ------------------------------------------------*/

#endif /* CONTROLLER_FUNCTIONS_CONFIG_H */
