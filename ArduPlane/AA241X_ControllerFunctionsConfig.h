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


/*------------------------------------ Controller Gains --------------------------------------------*/
const float rollControllerP  = 25.0;
const float rollControllerI  = 0.0;
const float rollControllerD  = 0.0;

const float pitchControllerP = 40.0;
const float pitchControllerI = 0.0;
const float pitchControllerD = 0.0;

const float rudderControllerP = 0.0;
const float rudderControllerI = 0.0;
const float rudderControllerD = 0.0;

const float altitudeControllerP = 1.0;
const float altitudeControllerI = 0.0;
const float altitudeControllerD = 0.0;

const float airspeedControllerP = 1.0;
const float airspeedControllerI = 0.0;
const float airspeedControllerD = 0.0;

const float headingControllerP = 2.0;
const float headingControllerI = 0.0;
const float headingControllerD = 0.0;

/*------------------------------------- Trim States ------------------------------------------------*/

#endif /* CONTROLLER_FUNCTIONS_CONFIG_H */
