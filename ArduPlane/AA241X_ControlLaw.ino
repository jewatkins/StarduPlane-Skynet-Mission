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
static char phaseOfFlight = preMissionLoiter; // Waiting to start mission
static float prevAltitude = 0.0;

// These functions are executed when control mode is in AUTO
// Please read AA241X_aux.h for all necessary definitions and interfaces

// *****   AA241X Fast Loop - @ ~50Hz   ***** //
static void AA241X_AUTO_FastLoop(void) 
{
  // Get the delta time between function calls
  float delta_t = CPU_time_ms - Last_AUTO_stampTime_ms;

  // Checking if we've just switched to AUTO. If more than 100ms have gone past since last time in AUTO, then we are definitely just entering AUTO
  if (delta_t > 100)
  {
	// Determine control mode from bits in parameter list
    if(FLIGHT_MODE > .5 && FLIGHT_MODE < 1.5){
      controlMode = ROLL_STABILIZE_MODE;
    }else if(FLIGHT_MODE > 1.5 && FLIGHT_MODE < 2.5){
      controlMode = STABILIZE_MODE;
    }else if(FLIGHT_MODE > 2.5 && FLIGHT_MODE < 3.5){
      controlMode = HEADING_HOLD_MODE;
    }else if(FLIGHT_MODE > 3.5 && FLIGHT_MODE < 4.5){
      controlMode = FBW_MODE;
      
      // Set altitude as current altitude
      altitudeCommand = -Z_position_GPS;
      altitudeController241X.SetReference(altitudeCommand);
      
      airspeedCommand = 11.0; // Phase 1 nominal speed
      
      // Mission Planner based pitch command rather than hard coded up top
      pitchCommand = (THETA_COMMAND/180.0)*PI;
    }else if(FLIGHT_MODE > 4.5 && FLIGHT_MODE < 5.5){
      controlMode = ATT_HOLD;
    }else if(FLIGHT_MODE > 5.5 && FLIGHT_MODE < 6.5){
      controlMode = WAYPOINT_NAV;
      
      airspeedCommand = 11.0;
      // Mission Planner based pitch command until trim settings determined
      pitchCommand = (THETA_COMMAND/180.0)*PI;      
    }

	// Set commands upon initialization of the autonomous mode
    SetReference(altitudeController_DEF, -Z_position_GPS); // Current Altitude

	airspeedCommand = 11.0; // 11.0 m/s
	SetReference(airspeedController_DEF, airspeedCommand);

	prevAltitude = -Z_position_GPS;
  }

  // Check delta_t before sending it to any step functions for inner loop controls
  if (delta_t < 10)
  {
	  delta_t = 20;
  }

  // Set reference for the heading
  SetReference(headingController_DEF, headingCommand);

  // Determine the roll command from the heading controller
  float rollCommand = StepController(headingController_DEF, ground_course, delta_t); 
  Limit(rollCommand, referenceLimits[rollController_DEF][maximum_DEF], referenceLimits[rollController_DEF][minimum_DEF]);

  // Determine climb rate command from the altitude controller
  float commandedClimbRate = StepController(altitudeController_DEF, -Z_position_GPS, delta_t);

  // Pitch trim scheduling
  float pitchTrim = SchedulePitchTrim(roll, Air_speed, commandedClimbRate);

  // Augment pitch angle through altitude controller
  if(fabs(commandedClimbRate) < 0.5)
  {
	  commandedClimbRate = 0.0;
  }
  SetReference(climbRateController_DEF, commandedClimbRate);
  
  // Get climb rate
  float climbRate = (-Z_position_GPS - prevAltitude) / delta_t;
  Limit(climbRate, MAX_CLIMB_RATE_DEF, MIN_CLIMB_RATE_DEF);

  // Find value to augment the pitch angle
  float pitchDeviation = StepController(climbRateController_DEF, climbRate, delta_t);

  // Step through each inner loop controller to get the RC output
  SetReference(rollController_DEF, rollCommand);
  float rollControllerOut = StepController(rollController_DEF, roll, delta_t);
  SetReference(pitchController_DEF, (pitchTrim + pitchDeviation));
  float pitchControllerOut = StepController(pitchController_DEF, pitch, delta_t);
  float airspeedControllerOut = StepController(airspeedController_DEF, Air_speed, delta_t);

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
  Rudder_servo     = RC_rudder;
  
  // Throttle PWM Command Out
  float throttleOut = RC_throttle + airspeedControllerOut;
  Limit(throttleOut, throttleMin_DEF, throttleMax_DEF);
  Throttle_servo   = throttleOut;
  
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

  /*
  gcs_send_text_P(SEVERITY_LOW, PSTR("Test Statement"));
  gcs_send_text_fmt(PSTR("Test Float = %f \n"), 25.5);
  */
};





