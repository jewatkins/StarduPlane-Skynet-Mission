#include <math.h>
#include <AP_Math.h>
#include "defines.h"
#include "AA241X_ControlLaw.h"
#include "AA241X_aux.h"
#include "AA241X_ControllerFunctions.h"
#include "AA241X_WaypointNavigation.h"

static uint32_t controlMode = MISSION;
static float headingCommand = 0.0;
static float altitudeCommand = 115.0;
static float airspeedCommand = 7.0;
static float delta_t = 0.0;
static char phaseOfFlight = preMissionLoiter; // Waiting to start mission

float variableOfInterest = 0.0;

// These functions are executed when control mode is in AUTO
// Please read AA241X_aux.h for all necessary definitions and interfaces

// *****   AA241X Fast Loop - @ ~50Hz   ***** //
static void AA241X_AUTO_FastLoop(void) 
{
  // Get the delta time between function calls
  delta_t = CPU_time_ms - Last_AUTO_stampTime_ms;

  // Checking if we've just switched to AUTO. If more than 100ms have gone past since last time in AUTO, then we are definitely just entering AUTO
  if (delta_t > 100)
  {
	// Set all references upon initialization of the autonomous mode
    //SetReference(rollController_DEF, 0.0);
    //SetReference(pitchController_DEF, 0.0);
    //SetReference(airspeedController_DEF, 11.0);
    //SetReference(altitudeHoldController_DEF, altitudeCommand);
    //SetReference(climbRateController_DEF, 3.0);
    //SetReference(glideController_DEF, 95.0);
    //SetReference(headingController_DEF, headingCommand);
  }

  // Check delta_t before sending it to any step functions for inner loop controls
  if (delta_t < 10)
  {
	  delta_t = 20;
  }

  // Set Reference for the heading
  SetReference(headingController_DEF, headingCommand);

  // Determine the roll command from the ground course error
  float rollCommand = StepController(headingController_DEF, ground_course, delta_t); 
  Limit(rollCommand, referenceLimits[rollController_DEF][maximum_DEF], referenceLimits[rollController_DEF][minimum_DEF]);

  /* Trim Scheduling */
  trimState_t trimSetting = ScheduleTrim(rollCommand, airspeedCommand, phaseOfFlight);

  // Step through each inner loop controller to get the RC output
  //float rollCommand = 0.0;
  SetReference(rollController_DEF, rollCommand);
  float rollControllerOut = StepController(rollController_DEF, roll, delta_t);

  float pitchCommand = trimSetting.pitch;
  SetReference(pitchController_DEF, pitchCommand);
  float pitchControllerOut = StepController(pitchController_DEF, pitch, delta_t);

  // Aileron Servo Command Out
  float rollOut    = RC_Roll_Trim + rollControllerOut;
  Limit(rollOut, rollMax_DEF, rollMin_DEF);
  Roll_servo       = rollOut;
  
  // Elevator Servo Command Out
  float pitchOut   = RC_Pitch_Trim + pitchControllerOut;
  Limit(pitchOut, pitchMax_DEF, pitchMin_DEF);
  Pitch_servo      = pitchOut;

  // Rudder Servo Command Out
  //float rudderOut  = RC_Rudder_Trim + rudderControllerOut;
  //Limit(rudderOut, rudderMax_DEF, rudderMin_DEF);
  //Rudder_servo     = 50; //rudderOut;
  
  // Throttle PWM Command Out
  //float throttleOut = RC_throttle + airspeedControllerOut;
  //Limit(throttleOut, throttleMin_DEF, throttleMax_DEF);
  Throttle_servo   = 0; // RC_throttle + airspeedControllerOut; //throttleOut;
  
};


// *****   AA241X Medium Loop - @ ~10Hz  *****  //
static void AA241X_AUTO_MediumLoop(void)
{
  // Time between function calls
  float delta_t = (CPU_time_ms - Last_AUTO_stampTime_ms); // Get delta time between AUTO_FastLoop calls  
  
  // Checking if we've just switched to AUTO. If more than 1000ms have gone past since last time in AUTO, then we are definitely just entering AUTO
  if (delta_t > 1000)
  {
    // Compute waypoint headings
    float dx = waypoints[0][0] - X_position;
    float dy = waypoints[0][1] - Y_position;
    Hwp[0] = WrapAngle(atan2f(dy,dx));
    uint32_t i;
    for (i=1; i<Nwp; i++) {
      dx = waypoints[i][0] - waypoints[i-1][0];
      dy = waypoints[i][1] - waypoints[i-1][1];
      Hwp[i] = WrapAngle(atan2f(dy,dx));
    }
    
    // Set waypoint iterator
    iwp = 0;
  }
  
  // Determine heading command based on specified route and current position
  if (controlMode == MISSION) {
    if (gpsOK == true)
    {
      // Check to see if waypoint is found
      float dx = waypoints[iwp][0] - X_position;
      float dy = waypoints[iwp][1] - Y_position;
      float pos_error = sqrtf(dx*dx + dy*dy);
      if (pos_error <= POSITION_ERROR) {
        // Take a snapshot
        snapshot mySnapShot = takeASnapshot();
        uint32_t i;
        for (i=0; i<Np; i++) {
          // Check if person found
          if (mySnapShot.personsInPicture[i] == 1) {
            persons_found[i] = 1;
            
            // Sum all persons found
            uint32_t ii;
            char sum = 0;
            for (ii=0; ii<Np; ii++) {
              sum += persons_found[ii];
            }
            
            // Set altitude command to 30 meters if all persons found
            if (sum == Np) {
              altitudeCommand = 30.0;
            }
          }
        }
        
        // Go to next waypoint
        iwp++;
      }
      
      // If all waypoints complete, restart route
      if (iwp == Nwp) {
        iwp = 0;
        dx = waypoints[0][0] - X_position;
        dy = waypoints[0][1] - Y_position;
        Hwp[0] = WrapAngle(atan2f(dy,dx));
      }
      
      // Compute heading (UAV to waypoint)
      dx = waypoints[iwp][0] - X_position;
      dy = waypoints[iwp][1] - Y_position;
      float Huav = WrapAngle(atan2f(dy,dx));
      
      // Compute heading error (rad)
      float Herr = (float)fabs(Huav - Hwp[iwp]);
      
      // Determine shortest angle and compute heading command
      if (Herr < (2*PI - Herr)) {
        headingCommand = WrapAngle(Hwp[iwp] + copysignf(1.0, Huav - Hwp[iwp])*ROUTE_P*Herr);
      }
      else {
        Herr = 2*PI - Herr;
        headingCommand = WrapAngle(Hwp[iwp] - copysignf(1.0, Huav - Hwp[iwp])*ROUTE_P*Herr);
      }
      
      // If Herr is too large, fly towards waypoint
      if (Herr > PI/4) {
        headingCommand = Huav;
      }
    }
  }
};





// *****   AA241X Slow Loop - @ ~1Hz  *****  //
static void AA241X_AUTO_SlowLoop(void){
  // YOUR CODE HERE
  
  /*
  float rollCommand = 0.0;
  
  SetReference(rollController_DEF, rollCommand);
  float rollControllerOut = StepController(rollController_DEF, roll, delta_t);

  // Aileron Servo Command Out
  float rollOut    = RC_Roll_Trim + rollControllerOut;
  Limit(rollOut, rollMax_DEF, rollMin_DEF);
  Roll_servo       = rollOut;
  */

  /*
  hal.console->printf_P(PSTR("Heading Command: %f \n"), headingCommand);
  hal.console->printf_P(PSTR("Altitude Command: %f \n"), altitudeCommand);
  hal.console->printf_P(PSTR("Airspeed Command: %f \n"), airspeedCommand);  
  hal.console->printf_P(PSTR("fabs(RC_roll - RC_Roll_Trim): %f \n"), fabs(RC_roll - RC_Roll_Trim) );
  hal.console->printf_P(PSTR("pitchCommand: %f \n"), pitchCommand);
  hal.console->printf_P(PSTR("rollCommand: %f \n"), rollCommand);
  */
  
};





