#include <math.h>
#include <AP_Math.h>
#include "defines.h"
#include "AA241X_ControlLaw.h"
#include "AA241X_aux.h"
#include "AA241X_ControllerFunctions.h"
#include "AA241X_WaypointNavigation.h"

/*----------------------------------------- Flight Modes ----------------------------------------------------*/
/* The different flight modes are good for characterizing and testing different components of the aircraft's performance.
 *
 */
#define ROLL_STABILIZE_MODE 1
#define STABILIZE_MODE      2
#define HEADING_HOLD_MODE   3
#define FBW_MODE            4
#define ATT_HOLD            5
#define WAYPOINT_NAV        6
#define MISSION             7  // controlMode for Mission
#define MAX_CLIMB           8
#define GLIDE               9
static uint16_t controlMode = ROLL_STABILIZE_MODE;

/*----------------------------------------- Phase of Flight -------------------------------------------------*/
/* The phase of flight will help determine the trim schedule. There are a finite number of flight phases for this
 * mission.
 *
 */

#define	LOITERING            0 // Hang out at current altitude, airspeed, and heading
#define CLIMBING             1 // Climb to the maximum altitude to get the widest field of view with the camera
#define	SIGHTING             2 // Cruise at highest altitude until all persons have been seen
#define REFINING             3 // Algorithm based waypoint nav (still in cruise mode)
#define	GLIDING              4 // Battery power is running low, lose altitude and refine as much as possible
#define	CELEBRATING          5 // Let's do some loops and fun crap here!
static char phaseOfFlight = LOITERING; // Waiting to start mission

/*----------------------------------------- Mission Planning Variables ----------------------------------------*/
#define SIGHTING_ALTITUDE 115.0f
#define MAX_CLIMB_AIRSPEED 7.5f

static bool initFastLoopPhase = true;

/*----------------------------------------- Outer Loop Control References -------------------------------------*/
/* These variables are the outer loop controllers' input to the inner loops to attain complete autonomous navigation.
 * Given a heading command, the inner loop will track that heading as best it can; likewise for airspeed and altitude.
 *
 */

static float headingCommand = 0.0;
static float altitudeCommand = 115.0;
static float airspeedCommand = 7.0;

// Debug variable
static float variableOfInterest1 = 0.0;
static float variableOfInterest2 = 0.0;

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
	// Adjust gains based on mission planner parameter input
	gains [altitudeController_DEF][pGain] = ALTITUDE_P;
	gains [airspeedController_DEF][pGain] = AIRSPEED_P;
	  
	// Determine control mode from bits in parameter list
    if(FLIGHT_MODE > .5 && FLIGHT_MODE < 1.5)
	{
      controlMode = ROLL_STABILIZE_MODE;
    }else if(FLIGHT_MODE > 1.5 && FLIGHT_MODE < 2.5)
	{
      controlMode = STABILIZE_MODE;
    }else if(FLIGHT_MODE > 2.5 && FLIGHT_MODE < 3.5)
	{
      controlMode = HEADING_HOLD_MODE;
    }else if(FLIGHT_MODE > 3.5 && FLIGHT_MODE < 4.5)
	{
		controlMode = FBW_MODE;
		// Set altitude as current altitude
		altitudeCommand = -Z_position_GPS;
		SetReference(altitudeController_DEF, altitudeCommand);
		airspeedCommand = 11.0; // Phase 1 nominal speed
    }else if(FLIGHT_MODE > 4.5 && FLIGHT_MODE < 5.5)
	{
		controlMode = ATT_HOLD;
    }else if(FLIGHT_MODE > 5.5 && FLIGHT_MODE < 6.5)
	{
		controlMode = WAYPOINT_NAV;
		airspeedCommand = 11.0;
    }else if(FLIGHT_MODE > 6.5 && FLIGHT_MODE < 7.5)
	{
		// This is probably the most important control mode, it is what will be used to fly the mission
		controlMode = MISSION;
		
		// Set commands upon initialization of the autonomous mode
		SetReference(altitudeController_DEF, 115.0); // Altitude needed for phase 1 of mission
		airspeedCommand = 15.0; // As fast as we can go
		SetReference(airspeedController_DEF, airspeedCommand);

		// Set the phase of flight
		phaseOfFlight = CLIMBING; // Climb as fast as possible to 115 meters
		initFastLoopPhase = true;

		// Medium Loop Initialization
		{
  	                // Start timer
			t_init = CPU_time_ms;
    
			// Set initial start position
			x_init = X_position;
			y_init = Y_position;

			// Set waypoint iterator
			iwp = 0;
    
			// Get first waypoint
			GetWaypoint();
    
			// Compute heading (waypoint tangent line)
			Hwp = WrapAngle(atan2f(ywp,xwp) + PI/2);
		}

	}else if(FLIGHT_MODE > 7.5 && FLIGHT_MODE < 8.5)
	{
      controlMode = MAX_CLIMB;
	  // Set pitch angle
	  float pitchCommand = MAX_CLIMB_PITCH;
	  SetReference(pitchController_DEF, pitchCommand);
    }else if(FLIGHT_MODE > 8.5 && FLIGHT_MODE < 9.5)
	{
      controlMode = GLIDE;
	  // Set pitch angle
	  float pitchCommand = GLIDE_PITCH;
	  SetReference(pitchController_DEF, pitchCommand);
    }
  }

  // Check delta_t before sending it to any step functions for inner loop controls
  if (delta_t < 10)
  {
	  delta_t = 20;
  }

  // Controller outputs needed for updating servos and throttle
  float rollControllerOut = 50.0;
  float pitchControllerOut = 50.0;
  float airspeedControllerOut = 75.0;
  float rudderControllerOut = 50.0;

  if(controlMode == ROLL_STABILIZE_MODE)
  {
	  // Keep Roll angle controlled based on RC pilot input (should be zero when stick is in center)
      float rollCommand = (RC_roll-RC_Roll_Trim)*0.01*PI;
      SetReference(rollController_DEF, rollCommand);
      rollControllerOut = StepController(rollController_DEF, roll, delta_t);
  }
  else if(controlMode == STABILIZE_MODE)
  {
	  // Keep Roll angle controlled based on RC pilot input (should be zero when stick is in center)
      float rollCommand = (RC_roll-RC_Roll_Trim)*0.01*PI;
      SetReference(rollController_DEF, rollCommand);
      rollControllerOut = StepController(rollController_DEF, roll, delta_t);

	  // Pitch Commands
      float pitchCommand = (RC_pitch - RC_Pitch_Trim)*0.01*PI/4.0 + (1.0/180.0)*PI;
      SetReference(pitchController_DEF, pitchCommand);
      pitchControllerOut = StepController(pitchController_DEF, pitch, delta_t);
  }
  else if (controlMode == HEADING_HOLD_MODE)
  {
      // Maintain Heading, RC pilot commands offset from heading that was saved
      // Heading Commands
      if (fabs(RC_roll - RC_Roll_Trim) > 5)
      {
        // Allow breakout room just in case RC_Roll_Trim is not DEAD on
        headingCommand += 0.01*(RC_roll - RC_Roll_Trim)/RC_Roll_Trim; // .0872 rad/s change rate based on 50 Hz
        
        // Check radian range of heading command
        if(headingCommand > 2*PI)
        {
          headingCommand -= 2*PI;
        }else if(headingCommand < 0)
        {
          headingCommand += 2*PI;
        }
        
        SetReference(headingController_DEF, headingCommand);
      }
      float headingControllerOut = StepController(headingController_DEF, ground_course, delta_t);
      Limit(headingControllerOut, referenceLimits[rollController_DEF][maximum_DEF], referenceLimits[rollController_DEF][minimum_DEF]);

      // Roll Commands
      SetReference(rollController_DEF, headingControllerOut);
      rollControllerOut = StepController(rollController_DEF, roll, delta_t);
  }
  else if (controlMode == FBW_MODE)
  {
	  // Maintain heading, altitude, and airspeed RC pilot commands offsets from saved initial conditions
	  // Heading Commands
      if ( fabs(RC_roll - RC_Roll_Trim) > 5 )
      {      
        // Allow breakout room just in case RC_Roll_Trim is not DEAD on
        headingCommand += 0.025*(RC_roll - RC_Roll_Trim)/RC_Roll_Trim; // .0872 rad/s change rate based on 50 Hz 0.00174
        
        // Check radian range of heading command
        if(headingCommand > 2*PI)
        {
          headingCommand -= 2*PI;
        }else if(headingCommand < 0)
        {
          headingCommand += 2*PI;
        }
        
        SetReference(headingController_DEF, headingCommand);
      }
      float rollCommand = StepController(headingController_DEF, ground_course, delta_t);
      Limit(rollCommand, referenceLimits[rollController_DEF][maximum_DEF], referenceLimits[rollController_DEF][minimum_DEF]);

      // Roll Control
      SetReference(rollController_DEF, rollCommand);
      rollControllerOut = StepController(rollController_DEF, roll, delta_t);
      
      // Altitude Control
      float altitude = 0.0;
	  if(gpsOK == true)
        altitude = -Z_position_GPS;
      else
        altitude = -Z_position_Baro;
      
      if(fabs(RC_pitch - RC_Pitch_Trim) > 5)
      {
        altitudeCommand += 0.04*(RC_pitch - RC_Pitch_Trim)/RC_Pitch_Trim; // 2 m/s change rate based on 50 Hz
        SetReference(altitudeController_DEF, altitudeCommand);
      }

	  // Pitch trim scheduling
	  float pitchTrim = SchedulePitchTrim(rollCommand, airspeedCommand, /*commandedClimbRate*/ 0.0 /* no contribution from climb rate */);

	  // Pitch Angle Control
	  float pitchDeviation = StepController(altitudeController_DEF, altitude, delta_t);
	  SetReference(pitchController_DEF, (pitchTrim + pitchDeviation));
	  pitchControllerOut = StepController(pitchController_DEF, pitch, delta_t);

	  // Airspeed Control
	  airspeedCommand = 7.0 + 5.0*RC_throttle*.01;
	  SetReference(airspeedController_DEF, airspeedCommand);
	  airspeedControllerOut = StepController(airspeedController_DEF, Air_speed, delta_t);
	  airspeedControllerOut += ScheduleThrottleTrim(airspeedCommand); // Add the trim depending on the desired airspeed

  }
  else if(controlMode == MISSION)
  {
	  // This mode is for flying the actual mission. It will have mission phase logic included soon enough.
	  
	  if(phaseOfFlight == CLIMBING)
	  {
			float altitude = -Z_position_GPS;
		    // Climb to 115 meters
			if(initFastLoopPhase == true)
			{
				initFastLoopPhase = false;
				SetReference(pitchController_DEF, MAX_CLIMB_PITCH);
				SetReference(rollController_DEF, 0.0);
				airspeedCommand = MAX_CLIMB_AIRSPEED;
				SetReference(airspeedController_DEF, airspeedCommand);
			}

			if(-Z_position_GPS >= altitude/*SIGHTING_ALTITUDE*/)
			{
				phaseOfFlight = SIGHTING;
				SetReference(altitudeController_DEF, SIGHTING_ALTITUDE);
				initFastLoopPhase = true;
				return;
			}

			// Step the PID controllers to keep max climb trim
			pitchControllerOut = StepController(pitchController_DEF, pitch, delta_t);
			rollControllerOut = StepController(rollController_DEF, roll, delta_t);
			airspeedControllerOut = ScheduleThrottleTrim(airspeedCommand) + StepController(airspeedController_DEF, Air_speed, delta_t); // Add the trim depending on the desired airspeed
	  }else if(phaseOfFlight == SIGHTING)
	  {		
			// Use waypoint navigation while in altitude hold, airspeed hold, and heading hold inner loop controllers
			// In this phase, inner loop controllers are assuming:
			// 1. headingCommand is given by the outer loop
			// 2. airspeedCommand is given by the outer loop
			// 3. altitudeCommand remains static at SIGHTING_ALTITUDE during the entire phase

		  	float altitude = 0.0;
			if(gpsOK == true)
				altitude = -Z_position_GPS;
			else
				altitude = -Z_position_Baro;
		  
		    if(initFastLoopPhase == true)
			{
				initFastLoopPhase = false;
				SetReference(altitudeController_DEF, altitude/*SIGHTING_ALTITUDE*/);
			}
			
			// Set reference for the heading
			SetReference(headingController_DEF, headingCommand);
			float headingControllerOut = StepController(headingController_DEF, ground_course, delta_t);
			Limit(headingControllerOut, referenceLimits[rollController_DEF][maximum_DEF], referenceLimits[rollController_DEF][minimum_DEF]);

			// Determine the roll command from the heading controller
			float rollCommand = StepController(headingController_DEF, ground_course, delta_t); 
			Limit(rollCommand, referenceLimits[rollController_DEF][maximum_DEF], referenceLimits[rollController_DEF][minimum_DEF]);
			
			// Roll Control
			SetReference(rollController_DEF, headingControllerOut);
			rollControllerOut = StepController(rollController_DEF, roll, delta_t);

			/*
			// Altitude Control
			float altitude = 0.0;
			if(gpsOK == true)
				altitude = -Z_position_GPS;
			else
				altitude = -Z_position_Baro;
			*/
			// Pitch trim scheduling
			float pitchTrim = SchedulePitchTrim(rollCommand, airspeedCommand, /*commandedClimbRate*/ 0.0 /* no contribution from climb rate */);

			// Pitch Angle Control
			float pitchDeviation = StepController(altitudeController_DEF, altitude, delta_t);
			SetReference(pitchController_DEF, (pitchTrim + pitchDeviation));
			pitchControllerOut = StepController(pitchController_DEF, pitch, delta_t);
	  
			// Airspeed Control
			SetReference(airspeedController_DEF, airspeedCommand);
			airspeedControllerOut = StepController(airspeedController_DEF, Air_speed, delta_t);
			airspeedControllerOut += ScheduleThrottleTrim(airspeedCommand); // Add the trim depending on the desired airspeed
	  }
	  /*
		case REFINING :
			// Use waypoint navigation while in altitude hold, airspeed hold, and heading hold inner loop controllers
			break;
		case GLIDING :
			// Out of power, need to glide to waypoints until lower limit of altitude is reached
			break;
		case CELEBRATING :
			// We have to do something fun here...
			break;
		case LOITERING :
			// Just hang out and circle, altitude hold, airspeed hold, roll angle hold
			break;
	  }*/
	  
  }else if(controlMode == MAX_CLIMB)
  {
	  // Set roll angle at zero degrees, keep wings level at max climb
	  float rollCommand = 0.0;
	  SetReference(rollController_DEF, rollCommand);
	  rollControllerOut = StepController(rollController_DEF, roll, delta_t);

	  // Set maximum throttle
	  airspeedControllerOut = 100.0;

	  // Keep pitch angle constant
	  pitchControllerOut = StepController(pitchController_DEF, pitch, delta_t);
  }else if(controlMode == GLIDE)
  {
	  // Set roll angle at zero degrees, keep wings level at max climb
	  float rollCommand = 0.0;
	  SetReference(rollController_DEF, rollCommand);
	  rollControllerOut = StepController(rollController_DEF, roll, delta_t);

	  // Set maximum throttle
	  airspeedControllerOut = 0.0;

	  // Keep pitch angle constant
	  pitchControllerOut = StepController(pitchController_DEF, pitch, delta_t);
  }

  // Aileron Servo Command Out
  if(controlMode == ROLL_STABILIZE_MODE 
	  || controlMode == STABILIZE_MODE
	  || controlMode == FBW_MODE
	  || controlMode == HEADING_HOLD_MODE
	  || controlMode == ATT_HOLD
	  || controlMode == WAYPOINT_NAV
	  || controlMode == MISSION
	  || controlMode == MAX_CLIMB
	  || controlMode == GLIDE)
  {
	float rollOut    = RC_Roll_Trim + rollControllerOut;
	Limit(rollOut, rollMax_DEF, rollMin_DEF);
	Roll_servo       = rollOut;
  }
  else
  {
    Roll_servo       = RC_roll;
  }
  
  // Elevator Servo Command Out
  if(controlMode == STABILIZE_MODE
	  || controlMode == FBW_MODE
	  || controlMode == ATT_HOLD
	  || controlMode == WAYPOINT_NAV
	  || controlMode == MISSION
	  || controlMode == MAX_CLIMB
	  || controlMode == GLIDE)
  {
	float pitchOut   = RC_Pitch_Trim + pitchControllerOut;
	Limit(pitchOut, pitchMax_DEF, pitchMin_DEF);
	Pitch_servo      = pitchOut;
  }
  else
  {
    Pitch_servo      = RC_pitch;
  }

  // Rudder Servo Command Out
  /*
  if(controlMode == STABILIZE_MODE || controlMode == HEADING_HOLD_MODE || controlMode == FBW_MODE || controlMode == ATT_HOLD)
  {
    float rudderOut  = RC_Rudder_Trim + rudderControllerOut;
    Limit(rudderOut, rudderMax_DEF, rudderMin_DEF);
    Rudder_servo     = rudderOut;
  } 
  else
  {*/
    Rudder_servo     = RC_rudder;
  //}
  
  // Throttle PWM Command Out
  if(controlMode == FBW_MODE
	  || controlMode == WAYPOINT_NAV
	  || controlMode == MISSION
	  || controlMode == MAX_CLIMB
	  || controlMode == GLIDE)
  {
	float throttleOut = airspeedControllerOut;
	Limit(throttleOut, throttleMax_DEF, throttleMin_DEF);
	Throttle_servo   = throttleOut;
  }
  else
  {
    Throttle_servo   = RC_throttle;
  }
  
};


// *****   AA241X Medium Loop - @ ~10Hz  *****  //
static void AA241X_AUTO_MediumLoop(void)
{
  // Determine heading command based on specified route and current position
  if (controlMode == MISSION) {
    if (gpsOK == true)
    {
      // Check to see if waypoint is found
      float dx = xwp - X_position;
      float dy = ywp - Y_position;
      float pos_error = sqrtf(dx*dx + dy*dy);
      if (pos_error <= SNAPSHOT_ERROR) {
		/*
        // Take a snapshot
        snapshot mySnapShot = takeASnapshot();
        
        // Post process results and go to next waypoint if snapshot is taken
        if (mySnapShot.pictureTaken == 1) {
          uint16_t i;
          for (i=0; i<Np; i++) {
            // Check if person found
            if (mySnapShot.personsInPicture[i] == 1) {
              persons_found[i] = 1;
              gcs_send_text_fmt(PSTR("Person %d found"),i+1);
              
              // Sum all persons found
              uint16_t ii;
              char sum = 0;
              for (ii=0; ii<Np; ii++) {
                sum += persons_found[ii];
              }
              
              // Display message if all persons found
              if (sum == Np) {
                gcs_send_text_P(SEVERITY_LOW, PSTR("All Persons found!"));
              }
            }
          }
		  */
          
		  // Output time of waypoint capture
		  float dt = (CPU_time_ms - t_init)/1000;
		  gcs_send_text_fmt(PSTR("Waypoint found: %f sec"),dt);

          // Go to next waypoint
          iwp++;
          
          // If all waypoints complete, restart route
          if (iwp == Nwp) {
            iwp = 0;
          }
          GetWaypoint();
          
          // Compute heading (waypoint tangent line)
          Hwp = WrapAngle(atan2f(ywp,xwp) + PI/2);
          
          // Start timer
          t_init = CPU_time_ms;
		  /*
        }
        else {
          gcs_send_text_P(SEVERITY_LOW, PSTR("Snapshot could not be taken!"));
        }
		*/
      }
      
      // Estimate needed airspeed to reach waypoint at correct time
      dx = xwp - X_position;
      dy = ywp - Y_position;
	  /*
      float ds = sqrtf(dx*dx + dy*dy);
      float dt = TIME_ESTIMATE - (CPU_time_ms - t_init)/1000;
      airspeedCommand = ds/dt;
      
      // Airspeed limiter
      if (airspeedCommand < 0.0 || airspeedCommand > 14.0) {
        airspeedCommand = 14.0;
      }
      else if (airspeedCommand < 7.0) {
        airspeedCommand = 7.0;
      }
	  */
	  airspeedCommand = 11.0;
      
      // Compute heading (UAV to waypoint)
      float Huav = WrapAngle(atan2f(dy,dx));
      
      // Compute heading error (rad)
      float Herr = (float)fabs(Huav - Hwp);
      
      // Determine shortest angle and compute heading command
      // Note: A line tracking gain of "ROUTE_P = 1" means set heading to waypoint.
      if (Herr < (2*PI - Herr)) {
        headingCommand = WrapAngle(Hwp + copysignf(1.0, Huav - Hwp)*ROUTE_P*Herr);
      }
      else {
        Herr = 2*PI - Herr;
        headingCommand = WrapAngle(Hwp - copysignf(1.0, Huav - Hwp)*ROUTE_P*Herr);
      }
      
      // If Herr is too large or if pos_error is small enough, set heading to waypoint.
      if (Herr >= PI/4 || pos_error <= POSITION_ERROR) {
        headingCommand = Huav;
      }
    }
  }
};





// *****   AA241X Slow Loop - @ ~1Hz  *****  //
static void AA241X_AUTO_SlowLoop(void)
{	

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

  //hal.console->printf_P(PSTR("\nx_init slow: %f \n"), x_init);
  //hal.console->printf_P(PSTR("y_init slow: %f \n"), y_init);
  //hal.console->printf_P(PSTR("xwp: %f \n"), xwp);
  //hal.console->printf_P(PSTR("ywp: %f \n"), ywp);
  /*
  gcs_send_text_P(SEVERITY_LOW, PSTR("Test Statement"));
  gcs_send_text_fmt(PSTR("Test Float = %f \n"), 25.5);
  */
};





