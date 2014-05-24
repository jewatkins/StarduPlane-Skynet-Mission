#include "AA241X_ControllerFunctions.h"
#include "AA241X_ControllerFunctionsConfig.h"

float StepController(unsigned char controller, float measured, float &delta_t)
{
  float command = 0.0;
  float pTerm = 0.0;
  float iTerm = 0.0;
  float dTerm = 0.0;

  float error = references[controller] - measured;

  //hal.console->printf_P(PSTR("\n error: %f \n"), error);

  if(controller == headingController_DEF)
  {
	  if(error >= PI)
	  {
           error = error - 2*PI;
      }else if (error <= - PI)
      {
           error = error + 2*PI;
      } 
  }

  // Calculate Running Integral of Error
  intErrors[controller] += ((prevErrors[controller] + error)*.5)/delta_t;

  //hal.console->printf_P(PSTR("\n intError: %f \n"), intErrors[controller]);

  // Cut off maximum integral error
  Limit(intErrors[controller], integralLimits[controller], -integralLimits[controller]);

  // Calculate derivative error
  float derError = (error - prevErrors[controller])/delta_t;

  //hal.console->printf_P(PSTR("\n derError: %f \n"), derError);

  // Reassign prevErrors
  prevErrors[controller] = error;

  // Put a saturation limit on the derivative error
  Limit(derError, derivativeLimits[controller], -derivativeLimits[controller]);
  
  // Calculate All Controller Terms
  pTerm = gains[controller][pGain]*error;  // Proportional Controller Term
  
  //hal.console->printf_P(PSTR("\n pTerm: %f \n"), pTerm);

  iTerm = gains[controller][iGain]*intErrors[controller]; // Integral Controller Term

  //hal.console->printf_P(PSTR("\n iTerm: %f \n"), iTerm);

  Limit(iTerm, integralTermLimits[controller], -integralTermLimits[controller]);  // Limit the integral controller
  dTerm = gains[controller][dGain]*derError;  // Derivative Term

  //hal.console->printf_P(PSTR("\n dTerm: %f \n"), dTerm);

  Limit(dTerm, derivativeTermLimits[controller], -derivativeTermLimits[controller]); // Limit the derivative controller
  
  /* Sum all terms */
  command = pTerm + iTerm + dTerm;

  //hal.console->printf_P(PSTR("\n command: %f \n"), command);

  // Put a saturation limit on the output command
  Limit(command, outputLimits[controller], -outputLimits[controller]);

  //hal.console->printf_P(PSTR("\n limited command: %f \n"), command);
    
  return command;
}

/* blah
 *
 */
void SetReference(unsigned char controller, float newValue)
{
  /* Check that the new reference input to command is within limited commands */
  Limit(newValue,referenceLimits[controller][maximum_DEF], referenceLimits[controller][minimum_DEF]);
  
  /* Assumes references is under the total number of controllers */
  references[controller] = newValue;
  
  return;
}

/* blah
 *
 */
void Limit(float &variable, float maximum, float minimum)
{
  if(variable > maximum)
  {
    variable = maximum;
  }
  else if(variable < minimum)
  {
    variable = minimum;
  }
  
  return;
}

/* blah
 *
 */
struct trimState_t ScheduleTrim(float rollCommand, float airspeedCommand, char phaseOfFlight)
{
  trimState_t trimCondition;
  char trimSetting = 0; // used in some of the switch cases

  // Determine what phase of flight we're in and what needs to be set as trim state / reference
  switch (phaseOfFlight)
  {
	  case preMissionLoiter:
		  // Set Trim State
		  trimCondition.pitch = trims[SEVEN_MPS_DEF][pitch_DEF];
          trimCondition.roll  = trims[SEVEN_MPS_DEF][roll_DEF];
          trimCondition.airspeed  = trims[SEVEN_MPS_DEF][airspeed_DEF];		  
		  break;

	  case climb:
		  // Set Trim State - max climb controller puts the aircraft in a huge climb for the beginning of the mission  
		  trimCondition.pitch = trims[MAX_CLIMB_DEF][pitch_DEF];
          trimCondition.roll  = trims[MAX_CLIMB_DEF][roll_DEF];
          trimCondition.airspeed  = trims[MAX_CLIMB_DEF][airspeed_DEF];
		  break;

	  case tSight:
		  // Set Trim State
		  trimCondition.pitch = trims[ELEVEN_MPS_DEF][pitch_DEF];
          trimCondition.roll  = trims[ELEVEN_MPS_DEF][roll_DEF];
          trimCondition.airspeed  = trims[ELEVEN_MPS_DEF][airspeed_DEF];
		  break;

	  case refinement:
		  // Set Trim State
		  trimSetting = determineTrimState(rollCommand, airspeedCommand);

		  trimCondition.pitch = trims[trimSetting][pitch_DEF];
          trimCondition.roll  = trims[trimSetting][roll_DEF];
          trimCondition.airspeed  = trims[trimSetting][airspeed_DEF];

		  break;
		  
	  case glide:
		  // Set Trim State
		  trimCondition.pitch = trims[GLIDE_DEF][pitch_DEF];
          trimCondition.roll  = trims[GLIDE_DEF][roll_DEF];
          trimCondition.airspeed  = trims[GLIDE_DEF][airspeed_DEF];

		  break;

	  case postMissionLoiter:
		  // Set Trim State
		  trimCondition.pitch = trims[SEVEN_MPS_DEF][pitch_DEF];
          trimCondition.roll  = trims[SEVEN_MPS_DEF][roll_DEF];
          trimCondition.airspeed  = trims[SEVEN_MPS_DEF][airspeed_DEF];		  
		  
		  break;
  } // end switch
  
  return trimCondition;
  
}

/* blah
 *
 */
char determineTrimState(float rollCommand, float airspeedCommand)
{
	char trimStateOut = 0;

	// Determine the bounds around the mid roll 
	float midRoll = referenceLimits[rollController_DEF][maximum_DEF]/2.0;
	//float midRollUpper = midRoll + (referenceLimits[rollController_DEF][maximum_DEF] - midRoll)/2.0;
	//float midRollUpper = midRoll - (referenceLimits[rollController_DEF][maximum_DEF] - midRoll)/2.0;
	float midAirspeed = (referenceLimits[airspeedController_DEF][maximum_DEF] - referenceLimits[airspeedController_DEF][minimum_DEF])/2.0;

	// Max Left Bank Case
	if (rollCommand < -midRoll)
	{
		if(airspeedCommand > midAirspeed)
		{
			// Max Airspeed Case
			trimStateOut = MAX_BANK_LEFT_TWELVE_MPS_DEF;

			return trimStateOut;

		} else
		{
			// Min Airspeed Case
			trimStateOut = MAX_BANK_LEFT_SEVEN_MPS_DEF;

			return trimStateOut;

		}
	}
	// Max Right Bank Case
	else if(rollCommand > midRoll)
	{
		if(airspeedCommand > midAirspeed)
		{
			// Max Airspeed Case
			trimStateOut = MAX_BANK_RIGHT_TWELVE_MPS_DEF;
			
			return trimStateOut;

		} else
		{
			// Min Airspeed Case
			trimStateOut = MAX_BANK_RIGHT_SEVEN_MPS_DEF;
			
			return trimStateOut;

		}
	}
	else if(airspeedCommand < 7.5)
	{
		trimStateOut = SEVEN_MPS_DEF;
		
		return trimStateOut;
	}
	else if(airspeedCommand < 8.5)
	{
		trimStateOut = EIGHT_MPS_DEF;

		return trimStateOut;
	}
	else if(airspeedCommand < 9.5)
	{
		trimStateOut = NINE_MPS_DEF;

		return trimStateOut;
	}
	else if(airspeedCommand < 10.5)
	{
		trimStateOut = TEN_MPS_DEF;

		return trimStateOut;
	}
	else if(airspeedCommand < 11.5)
	{
		trimStateOut = ELEVEN_MPS_DEF;

		return trimStateOut;
	}
	else
	{
		trimStateOut = TWELVE_MPS_DEF;

		return trimStateOut;
	}
}