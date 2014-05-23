#include <math.h>
#include <AP_Math.h>
#include "defines.h"
#include "AA241X_ControlLaw.h"
#include "AA241X_aux.h"
#include "AA241X_ControllerFunctions.h"
#include "AA241X_WaypointNavigation.h"

static float headingCommand = 0.0;
static float altitudeCommand = 115.0;
static float airspeedCommand = 7.0;
static float delta_t = 0.0;
static char phaseOfFlight = preMissionLoiter; // Waiting to start mission

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
    SetReference(rollController_DEF, 0.0);
    SetReference(pitchController_DEF, 0.0);
    SetReference(airspeedController_DEF, 11.0);
    SetReference(altitudeHoldController_DEF, altitudeCommand);
    SetReference(climbRateController_DEF, 3.0);
    SetReference(glideController_DEF, 95.0);
    SetReference(headingController_DEF, headingCommand);
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
  Limit(rollCommand, limits[rollController_DEF][maximum_DEF], limits[rollController_DEF][minimum_DEF]);

  /* Trim Scheduling */
  // trimState_t trimSetting = ScheduleTrim(rollCommand, airspeedCommand, phaseOfFlight);

  SetReference(rollController_DEF, rollCommand);
  float rollControllerOut = StepController(rollController_DEF, roll, delta_t);
  Limit(rollControllerOut, rollMax_DEF, rollMin_DEF);
  
  // Step through each inner loop controller to get the RC output
  
  
};


// *****   AA241X Medium Loop - @ ~10Hz  *****  //
static void AA241X_AUTO_MediumLoop(void)
{  
  // Time between function calls
  float delta_t = (CPU_time_ms - Last_AUTO_stampTime_ms); // Get delta time between AUTO_FastLoop calls  
  
  // Checking if we've just switched to AUTO. If more than 100ms have gone past since last time in AUTO, then we are definitely just entering AUTO
  if (delta_t > 100)
  {
    // Compute waypoint headings
    float dx = waypoints[0][0] - X_position;
    float dy = waypoints[0][1] - Y_position;
    Hwp[0] = atan2(dy,dx) + PI;
    for (uint32_t i=1; i<Nwp; i++) {
      dx = waypoints[i][0] - waypoints[i-1][0];
      dy = waypoints[i][1] - waypoints[i-1][1];
      Hwp[i] = atan2(dy,dx) + PI;
    }
    
    // Set waypoint iterator
    iwp = 0;
  }
  
  // Determine heading command based on specified route and current position
  if (gpsOK == true)
  {
    // Compute heading (UAV to waypoint)
    float dx = waypoints[iwp][0] - X_position;
    float dy = waypoints[iwp][1] - Y_position;
    float Huav = atan2(dy,dx) + PI;
    
    // Check to see if waypoint is found
    float pos_error = sqrt(dx*dx + dy*dy);
    hal.console->printf_P(PSTR("Position Error: %f \n"), pos_error);
    if (pos_error <= POSITION_ERROR) {
      // Take a snapshot
      snapshot mySnapShot = takeASnapshot();
      for (uint32_t i=0; i<Np; i++) {
        // Check if person found
        if (mySnapShot.personsInPicture[i] == 1) {
          persons_found[i] = 1;
          
          // Sum all persons found
          char sum = 0;
          for (uint32_t ii=0; ii<Np; ii++) {
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
    if (iwp == Nwp+1) {
      iwp = 0;
      dx = waypoints[0][0] - X_position;
      dy = waypoints[0][1] - Y_position;
      Hwp[0] = atan2(dy,dx) + PI;
    }
        
    // Compute heading (UAV to waypoint)
    dx = waypoints[iwp][0] - X_position;
    dy = waypoints[iwp][1] - Y_position;
    Huav = atan2(dy,dx) + PI;
    
    // Compute heading error (rad)
    float Herr = fabs(Huav - Hwp[iwp]);
    
    // Determine shortest angle and compute heading command
    if (Herr < (2*PI - Herr)) {
      headingCommand = Hwp[iwp] + copysignf(1.0, Huav - Hwp[iwp])*ROUTE_P*Herr;
    }
    else {
      Herr = 2*PI - Herr;
      headingCommand = Hwp[iwp] - copysignf(1.0, Huav - Hwp[iwp])*ROUTE_P*Herr;
    }
    
    // Check radian range of heading command
    if(headingCommand > 2*PI) {
      headingCommand -= 2*PI;
    }
    else if(headingCommand < 0) {
      headingCommand += 2*PI;
    }
    hal.console->printf_P(PSTR("Heading Command: %f \n"), headingCommand);
  }
  
  /* Longitudinal Control */
  float altitudeError = altitudeCommand - (-Z_position_GPS); // Get altitude error to see if we need to climb
  
  /*
  if (fabs(altitudeError) > climbThreshold)
  {
    // Large discrepancy in altitude, use climb rate control
    longitudinalController = climbRateController_DEF;
  }
  else
  {
    // Not a big discrepancy in altitude, use altitude hold
    longitudinalController = altitudeHoldController_DEF; 
  }
  */
  
    
  
};





// *****   AA241X Slow Loop - @ ~1Hz  *****  //
static void AA241X_AUTO_SlowLoop(void){
  // YOUR CODE HERE
  
  
  
};





