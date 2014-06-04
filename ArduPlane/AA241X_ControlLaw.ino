#include <math.h>
#include <AP_Math.h>
#include "defines.h"
#include "AA241X_ControlLaw.h"
#include "AA241X_aux.h"
#include "AA241X_ControllerFunctions.h"
#include "AA241X_MissionPlan.h"
#include "AA241X_WaypointNavigation.h"

/*----------------------------------------- Flight Modes ----------------------------------------------------*/
/* The different flight modes are good for characterizing and testing different components of the aircraft's performance.
 *
 */
#define ROLL_STABILIZE_MODE  1
#define STABILIZE_MODE       2
#define HEADING_HOLD_MODE    3
#define FBW_MODE             4
#define ATT_HOLD             5
#define WAYPOINT_NAV         6
#define MISSION              7  // controlMode for Mission
#define MAX_CLIMB            8
#define GLIDE                9
#define ALTITUDE_TEST        10
#define BANKED_ALTITUDE_TEST 11
static uint16_t controlMode = ROLL_STABILIZE_MODE;

/*----------------------------------------- Phase of Flight -------------------------------------------------*/
/* The phase of flight will help determine the trim schedule. There are a finite number of flight phases for this
 * mission.
 *
 */

#define	LOITERING				0 // Hang out at 90 feet, fly waypoints to get to CLIMBING initial position
#define CLIMBING				1 // Climb to the maximum altitude to get the widest field of view with the camera
#define	SIGHTING				2 // Cruise at highest altitude until all persons have been seen
#define REFINING				3 // Algorithm based waypoint nav (still in cruise mode)
#define	GLIDING					4 // Battery power is running low, lose altitude and refine as much as possible
#define	CELEBRATING				5 // Let's do some loops and fun crap here!
#define POST_MISSION_LOITERING	6 // Drop to a prescribed altitude and circle
static char phaseOfFlight = CLIMBING; // Waiting to start mission

/*----------------------------------------- Mission Planning Variables ----------------------------------------*/
#define LOITERING_ALTITUDE 10.0f
#define LOITERING_AIRSPEED 13.5f
#define SIGHTING_ALTITUDE 115.0f
#define MAX_CLIMB_AIRSPEED 15.0f
#define SIGHTING_GROUND_SPEED 10.5f
#define NOMINAL_AIRSPEED 10.0f
#define MAX_CLIMB_PITCH 0.38f  // 22 degrees
#define MAX_CLIMB_ROLL 0.1744f // 10 degrees
#define GLIDE_PITCH 0.122f

static bool initFastLoopPhase = true;

/*----------------------------------------- Outer Loop Control References -------------------------------------*/
/* These variables are the outer loop controllers' input to the inner loops to attain complete autonomous navigation.
 * Given a heading command, the inner loop will track that heading as best it can; likewise for airspeed and altitude.
 *
 */

static float headingCommand = 0.0;
static float altitudeCommand = 115.0;
static float airspeedCommand = 11.0;
static float pitchTrim = 0.0;

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
    if(FLIGHT_MODE > .5 && FLIGHT_MODE < 1.5)
    {
      controlMode = ROLL_STABILIZE_MODE;
    }
    else if(FLIGHT_MODE > 1.5 && FLIGHT_MODE < 2.5)
    {
      controlMode = STABILIZE_MODE;
    }
    else if(FLIGHT_MODE > 2.5 && FLIGHT_MODE < 3.5)
    {
      controlMode = HEADING_HOLD_MODE;
    }
    else if(FLIGHT_MODE > 3.5 && FLIGHT_MODE < 4.5)
    {
      controlMode = FBW_MODE;
      // Set altitude as current altitude
      altitudeCommand = -Z_position_GPS;
      SetReference(altitudeController_DEF, altitudeCommand);
      airspeedCommand = 11.0; // Phase 1 nominal speed
    }
    else if(FLIGHT_MODE > 4.5 && FLIGHT_MODE < 5.5)
    {
      controlMode = ATT_HOLD;
    }
    else if(FLIGHT_MODE > 5.5 && FLIGHT_MODE < 6.5)
    {
      controlMode = WAYPOINT_NAV;
      airspeedCommand = 11.0;
    }
    else if(FLIGHT_MODE > 6.5 && FLIGHT_MODE < 7.5)
    {
      // This is probably the most important control mode, it is what will be used to fly the mission
      controlMode = MISSION;

      switch(phaseOfFlight)
      {
      case LOITERING:
        initFastLoopPhase = true;
        SetReference(altitudeController_DEF, LOITERING_ALTITUDE);
        SetReference(airspeedController_DEF, LOITERING_AIRSPEED);
        break;
      case CLIMBING:
        initFastLoopPhase = true;
        SetReference(altitudeController_DEF, SIGHTING_ALTITUDE);
        SetReference(airspeedController_DEF, MAX_CLIMB_AIRSPEED);
        break;
      case SIGHTING:
		// Let the waypoint navigation command airspeed and heading
        initFastLoopPhase = true;
        break;
      case REFINING:
        initFastLoopPhase = true;
        break;
      case GLIDING:
        initFastLoopPhase = true;
        break;
      case CELEBRATING:
        initFastLoopPhase = true;
        break;
      case POST_MISSION_LOITERING:
        initFastLoopPhase = true;
        break;
      }

    }
    else if(FLIGHT_MODE > 7.5 && FLIGHT_MODE < 8.5)
    {
      controlMode = MAX_CLIMB;
      // Set pitch angle
      float pitchCommand = MAX_CLIMB_PITCH;
      SetReference(pitchController_DEF, pitchCommand);
      headingCommand = ground_course;
      SetReference(headingController_DEF, headingCommand);

    }
    else if(FLIGHT_MODE > 8.5 && FLIGHT_MODE < 9.5)
    {
      controlMode = GLIDE;
      // Set pitch angle
      float pitchCommand = GLIDE_PITCH;
      SetReference(pitchController_DEF, pitchCommand);
    }
    else if(FLIGHT_MODE > 9.5 && FLIGHT_MODE < 10.5)
    {
      controlMode = ALTITUDE_TEST;
      // Set pitch angle
      float pitchCommand = (TEST_PITCH/180)*PI;
      SetReference(pitchController_DEF, pitchCommand);

	  // Set airspeed
	  airspeedCommand = TEST_AIRSPEED;
	  SetReference(airspeedController_DEF, airspeedCommand);

	  // Keep wings perfectly level for this test
	  SetReference(rollController_DEF, 0.0);
    }
    else if(FLIGHT_MODE > 10.5 && FLIGHT_MODE < 11.5)
    {
      controlMode = BANKED_ALTITUDE_TEST;
      // Set pitch angle
      float pitchCommand = (TEST_PITCH/180.0)*PI;
      SetReference(pitchController_DEF, pitchCommand);

	  // Set airspeed
	  airspeedCommand = TEST_AIRSPEED;
	  SetReference(airspeedController_DEF, airspeedCommand);

	  // Keep wings at commanded bank angle
	  float rollCommand = (TEST_ROLL/180.0)*PI;
	  SetReference(rollController_DEF, rollCommand);
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
      }
      else if(headingCommand < 0)
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

    // Airspeed Control
    // airspeedCommand = 7.0 + 5.0*RC_throttle*.01;
    float groundSpeedCommand = 7.0 + 5.0*RC_throttle*.01;
    SetReference(groundSpeedController_DEF, groundSpeedCommand);
    airspeedCommand = NOMINAL_AIRSPEED + StepController(groundSpeedController_DEF, ground_speed, delta_t);
	DiscretizeAirspeedCommand(airspeedCommand);
    SetReference(airspeedController_DEF, airspeedCommand);
    airspeedControllerOut = StepController(airspeedController_DEF, Air_speed, delta_t);
    airspeedControllerOut += ScheduleThrottleTrim(airspeedCommand); // Add the trim depending on the desired airspeed

      // Heading Commands
    if ( fabs(RC_roll - RC_Roll_Trim) > 5 )
    {      
      // Allow breakout room just in case RC_Roll_Trim is not DEAD on
      headingCommand += 0.025*(RC_roll - RC_Roll_Trim)/RC_Roll_Trim; // .0872 rad/s change rate based on 50 Hz 0.00174

      // Check radian range of heading command
      if(headingCommand > 2*PI)
      {
        headingCommand -= 2*PI;
      }
      else if(headingCommand < 0)
      {
        headingCommand += 2*PI;
      }

      SetReference(headingController_DEF, headingCommand);
    }

    // Schedule the heading gains based on airspeed command
    //ScheduleHeadingGain(airspeedCommand);
    float rollCommand = StepController(headingController_DEF, ground_course, delta_t);
    Limit(rollCommand, referenceLimits[rollController_DEF][maximum_DEF], referenceLimits[rollController_DEF][minimum_DEF]);

    // Roll Control
    SetReference(rollController_DEF, rollCommand);
    rollControllerOut = StepController(rollController_DEF, roll, delta_t);

    // Altitude Control
    float altitude = -Z_position_Baro;

    if(fabs(RC_pitch - RC_Pitch_Trim) > 5)
    {
      altitudeCommand += 0.04*(RC_pitch - RC_Pitch_Trim)/RC_Pitch_Trim; // 2 m/s change rate based on 50 Hz
      SetReference(altitudeController_DEF, altitudeCommand);
    }

    // Pitch trim scheduling
    float pitchTrim = SchedulePitchTrim(rollCommand, airspeedCommand);

    // Pitch Angle Control
    float pitchDeviation = StepController(altitudeController_DEF, altitude, delta_t);
    SetReference(pitchController_DEF, (pitchTrim + pitchDeviation));
    pitchControllerOut = StepController(pitchController_DEF, pitch, delta_t);

  }
  else if(controlMode == MISSION)
  {
    // This mode is for flying the actual mission. It will have mission phase logic included soon enough.
    if(phaseOfFlight == LOITERING)
    {
      // Do pre-mission loitering circuit, then climb to altitude
    }
    else if(phaseOfFlight == CLIMBING)
    {
      float altitude = -Z_position_Baro;

      // Initialize t_sight
      if (init_t_sight_flag == 1 && altitude > 15.47) {
        init_t_sight_flag = 0;
        t_sight_start = CPU_time_ms;
      }

      // Set maximum throttle
      airspeedControllerOut = 100.0;

      // Climb to 115 meters
      if(initFastLoopPhase == true)
      {
        init_t_sight_flag = 1;
        finalize_t_sight_flag = 1;
        initFastLoopPhase = false;

        // Set pitch angle reference
        SetReference(pitchController_DEF, MAX_CLIMB_PITCH);

        // Set airspeed reference
        airspeedCommand = MAX_CLIMB_AIRSPEED;
        SetReference(airspeedController_DEF, airspeedCommand);
/*
        // Schedule the heading gains based on max climb conditions
        gains[headingController_DEF][pGain] = .5;
        gains[headingController_DEF][pGain] = .008;

        // Set heading angle
        headingCommand = ground_course;
        // Check radian range of heading command
        if(headingCommand > 2*PI)
        {
          headingCommand -= 2*PI;
        }
        else if(headingCommand < 0)
        {
          headingCommand += 2*PI;
        }
        SetReference(headingController_DEF, headingCommand);
*/
      }

      if(-Z_position_Baro >= SIGHTING_ALTITUDE)
      {
        phaseOfFlight = SIGHTING;
        SetReference(altitudeController_DEF, SIGHTING_ALTITUDE);
        initFastLoopPhase = true;
        return;
      }

      // Step the PID controllers to keep max climb trim
      pitchControllerOut = StepController(pitchController_DEF, pitch, delta_t);
      airspeedControllerOut = ScheduleThrottleTrim(airspeedCommand) + StepController(airspeedController_DEF, Air_speed, delta_t); // Add the trim depending on the desired airspeed

      //float rollCommand = StepController(headingController_DEF, ground_course, delta_t);
      //Limit(rollCommand, .175, -.175);

      // Roll Control
      //SetReference(rollController_DEF, rollCommand);
      rollControllerOut = StepController(rollController_DEF, roll, delta_t);
    }
    else if(phaseOfFlight == SIGHTING)
    {		
      // Use waypoint navigation while in altitude hold, airspeed hold, and heading hold inner loop controllers
      // In this phase, inner loop controllers are assuming:
      // 1. headingCommand is given by the outer loop
      // 2. airspeedCommand is given by the outer loop
      // 3. altitudeCommand remains static at SIGHTING_ALTITUDE during the entire phase

      float altitude = -Z_position_Baro;

      if(initFastLoopPhase == true)
      {
        initFastLoopPhase = false;

        // Reset Gains on heading controller
        gains[headingController_DEF][pGain] = 0.7;
        gains[headingController_DEF][iGain] = 0.007;

        SetReference(altitudeController_DEF, SIGHTING_ALTITUDE);
        // SetReference(groundSpeedController_DEF, SIGHTING_GROUND_SPEED);
      }

      // Roll Angle Control
      rollControllerOut = StepController(rollController_DEF, roll, delta_t);

      // Pitch Angle Control
      pitchControllerOut = StepController(pitchController_DEF, pitch, delta_t);

      // Airspeed Control
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

  }
  else if(controlMode == MAX_CLIMB)
  {        
    // Schedule the heading gains based on airspeed command
    gains[headingController_DEF][pGain] = .5;
    gains[headingController_DEF][pGain] = .008;
    float rollCommand = StepController(headingController_DEF,ground_course, delta_t);
    Limit(rollCommand, .175, -.175);
    SetReference(rollController_DEF, rollCommand);
    rollControllerOut = StepController(rollController_DEF, roll, delta_t);

    // Set maximum throttle
    airspeedControllerOut = 100.0;

    // Keep pitch angle constant
    pitchControllerOut = StepController(pitchController_DEF, pitch, delta_t);
  }
  else if(controlMode == GLIDE)
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
  else if(controlMode == ALTITUDE_TEST)
  {
	  // Step Airspeed Controller
	  airspeedControllerOut = StepController(airspeedController_DEF, Air_speed, delta_t);
	  //Limit(airspeedControllerOut, referenceLimits[airspeedController_DEF][maximum_DEF], referenceLimits[airspeedController_DEF][minimum_DEF]);
	  airspeedControllerOut += ScheduleThrottleTrim(airspeedCommand); // Add the trim depending on the desired airspeed

	  // Pitch Angle control
	  pitchControllerOut = StepController(pitchController_DEF, pitch, delta_t);
	  //Limit(pitchControllerOut, referenceLimits[pitchController_DEF][maximum_DEF], referenceLimits[pitchController_DEF][minimum_DEF]);

	  // Roll Angle control
	  rollControllerOut = StepController(rollController_DEF, roll, delta_t);
	  //Limit(rollControllerOut, referenceLimits[rollController_DEF][maximum_DEF], referenceLimits[rollController_DEF][minimum_DEF]);
  }
  else if(controlMode == BANKED_ALTITUDE_TEST)
  {
	  // Step Airspeed Controller
	  airspeedControllerOut = StepController(airspeedController_DEF, Air_speed, delta_t);
	  //Limit(airspeedControllerOut, referenceLimits[airspeedController_DEF][maximum_DEF], referenceLimits[airspeedController_DEF][minimum_DEF]);
	  airspeedControllerOut += ScheduleThrottleTrim(airspeedCommand); // Add the trim depending on the desired airspeed

	  // Pitch Angle control
	  pitchControllerOut = StepController(pitchController_DEF, pitch, delta_t);
	  //Limit(pitchControllerOut, referenceLimits[pitchController_DEF][maximum_DEF], referenceLimits[pitchController_DEF][minimum_DEF]);

	  // Roll Angle control
	  rollControllerOut = StepController(rollController_DEF, roll, delta_t);
	  //Limit(rollControllerOut, referenceLimits[rollController_DEF][maximum_DEF], referenceLimits[rollController_DEF][minimum_DEF]);
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
    || controlMode == GLIDE
	|| controlMode == ALTITUDE_TEST
	|| controlMode == BANKED_ALTITUDE_TEST)
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
    || controlMode == GLIDE
	|| controlMode == ALTITUDE_TEST
	|| controlMode == BANKED_ALTITUDE_TEST	)
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
    || controlMode == GLIDE
	|| controlMode == ALTITUDE_TEST
	|| controlMode == BANKED_ALTITUDE_TEST)
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
  // Get the delta time between function calls
  float delta_t = CPU_time_ms - Last_AUTO_stampTime_ms;
  if (delta_t < 100) {
    delta_t = 100;
  }

  // Determine heading command based on specified route and current position
  if (controlMode == MISSION && phaseOfFlight == SIGHTING) {

    // Initialize Phase-1 Spiral
    if (INIT_SPIRAL < .5 && phase_flag == 1) {
      InitPhase1Spiral();
    }

    // Phase-1 Logistics Loop
    if (gpsOK == true && phase_flag == 1) {
      Phase1();
    }

    
	// Initialize Phase-2 Simple
    if (INIT_PHASE2 < .5 && phase_flag == 2) {
      //InitPhase2Simple();
      InitPhase2();
    }

    // Phase-2 Logistics Loop
    if (gpsOK == true && phase_flag == 2) {
      Phase2();
    }

	// Phase-3 Logistics Loop
    if (gpsOK == true && phase_flag == 3) {
      Phase3();
	  altitudeCommand = 115.0;
	  SetReference(altitudeController_DEF, altitudeCommand);
    }

	/************** Set references for the fast loop based on medium loop updates ********************/

    // Navigation Loop
    headingCommand =  GetNavHeading();
	SetReference(headingController_DEF, headingCommand);
    float headingControllerOut = StepController(headingController_DEF, ground_course, delta_t);
    Limit(headingControllerOut, referenceLimits[rollController_DEF][maximum_DEF], referenceLimits[rollController_DEF][minimum_DEF]);

	// Settings to track ground speed
    //float groundSpeedCommand = GetNavAirspeed();
	//SetReference(groundSpeedController_DEF, groundSpeedCommand);
    //airspeedCommand = NOMINAL_AIRSPEED + StepController(groundSpeedController_DEF, ground_speed, delta_t);
	
	// Discretize the airspeedCommand to allow the controller to work
	//DiscretizeAirspeedCommand(airspeedCommand);
    //Limit(airspeedCommand, referenceLimits[airspeedController_DEF][maximum_DEF], referenceLimits[airspeedController_DEF][minimum_DEF]);
    //airspeedCommand = 9.0;
    //SetReference(airspeedController_DEF, airspeedCommand);

    // Determine the roll command from the heading controller
    float rollCommand = StepController(headingController_DEF, ground_course, delta_t); 
    Limit(rollCommand, referenceLimits[rollController_DEF][maximum_DEF], referenceLimits[rollController_DEF][minimum_DEF]);		
    SetReference(rollController_DEF, headingControllerOut);

    // Pitch trim scheduling
    float pitchTrim = SchedulePitchTrim(rollCommand, airspeedCommand);

    // Pitch Angle Control
	float altitude = -Z_position_Baro;
    float pitchDeviation = StepController(altitudeController_DEF, altitude, delta_t);
    SetReference(pitchController_DEF, (pitchTrim + pitchDeviation));
  }

  /***** References for the Fast Loop *******/
  if(controlMode == MISSION && phaseOfFlight == CLIMBING)
  {
    // Settings to track a heading
    //float rollCommand = StepController(headingController_DEF, ground_course, delta_t);
    //Limit(rollCommand, .175, -.175);
    SetReference(rollController_DEF, MAX_CLIMB_ROLL);
  }

};




// *****   AA241X Slow Loop - @ ~1Hz  *****  //
static void AA241X_AUTO_SlowLoop(void)
{

	// Estimate target locations (Phase-2 Simple)
	if (controlMode == MISSION && phaseOfFlight == SIGHTING) {

		// Phase-2 Logistics Loop
		if (gpsOK == true && phase_flag == 2) {

	   		// Check to see if snapshot is available
			float dt = (CPU_time_ms - t_init)/1000;
			if (dt >= 3.0) {
				// Take a snapshot
				snapshot mySnapShot = takeASnapshot();
      
			// Parse snapshots and continue to next target if all snapshots complete
			if (parseSnapshot(mySnapShot)) {
				//n_snaps[iTarget] = n_Inc;
				iorder++;
				SetTarget();
			}

			// Set our own global person estimates for MAV-link
			PERSON_1_X = X_person_estimate[0];
			PERSON_1_Y = Y_person_estimate[0];

			PERSON_2_X = X_person_estimate[1];
			PERSON_2_Y = Y_person_estimate[1];

			PERSON_3_X = X_person_estimate[2];
			PERSON_3_Y = Y_person_estimate[2];

			PERSON_4_X = X_person_estimate[3];
			PERSON_4_Y = Y_person_estimate[3];

			T_SIGHT = t_sight_end - t_sight_start;
		// 
			if (iorder >= Ntargets) {
				phase_flag = 3;
			}
			}
		}

    	// Set airspeed for mission
		airspeedCommand = GetNavAirspeed();
		SetReference(airspeedController_DEF, airspeedCommand);

		// Compute estimated target locations
		//EstimateTargetLocation();
  	}

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

	//hal.console->printf_P(PSTR("derivativeTermLimits[0]: "), pgm_read_int(&derivativeTermLimits[0]));
	//hal.console->printf_P(PSTR("derivativeTermLimits[0]: "), pgm_read_float_near(&derivativeTermLimits[0]));
	//hal.console->printf_P(PSTR("derivativeTermLimits[0]: "), pgm_read_float_far(&derivativeTermLimits[0]));

};







