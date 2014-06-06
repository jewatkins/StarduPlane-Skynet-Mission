#include "AA241X_ControllerFunctions.h"
#include "AA241X_ControllerFunctionsConfig.h"
#include <avr/pgmspace.h>

/* Controller Variables */
static float references[numControllers]; // reference inputs to the controllers
static float intErrors[numControllers];  // Need to track iterm over multiple iterations
static float prevErrors[numControllers]; // Need to track integral error over multiple iterations


float StepController(unsigned int controller, float measured, float &delta_t)
{
  float command = 0.0;
  float pTerm = 0.0;
  float iTerm = 0.0;
  //float dTerm = 0.0;

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

	  // If heading is still whacky after integral term limiter much more restrictive, try the quadratic
	  //error = copysignf((error*10)*(error*10)/100, error);
  }

  // Calculate Running Integral of Error
  intErrors[controller] += ((prevErrors[controller] + error)*.5)/delta_t;

  //hal.console->printf_P(PSTR("\n intError: %f \n"), intErrors[controller]);

  // Cut off maximum integral error
  Limit(intErrors[controller], pgm_read_float(&integralLimits[controller]), -pgm_read_float(&integralLimits[controller]));

  // Calculate derivative error
  // float derError = (error - prevErrors[controller])/delta_t;

  //hal.console->printf_P(PSTR("\n derError: %f \n"), derError);

  // Reassign prevErrors
  prevErrors[controller] = error;

  // Put a saturation limit on the derivative error
  // Limit(derError,  pgm_read_float_near(&(derivativeLimits[controller])), -pgm_read_float_near(&(derivativeLimits[controller])));
  
  // Calculate All Controller Terms
  pTerm = pgm_read_float(&gains[controller][pGain])*error;  // Proportional Controller Term
  
  //hal.console->printf_P(PSTR("\n pTerm: %f \n"), pTerm);

  iTerm = pgm_read_float(&gains[controller][iGain])*intErrors[controller]; // Integral Controller Term

  //hal.console->printf_P(PSTR("\n iTerm: %f \n"), iTerm);

  Limit(iTerm, pgm_read_float(&integralTermLimits[controller]), -pgm_read_float(&integralTermLimits[controller]));  // Limit the integral controller
  
  //dTerm = pgm_read_float(&gains[controller][dGain])*derError;  // Derivative Term

  //hal.console->printf_P(PSTR("\n dTerm: %f \n"), dTerm);

  //Limit(dTerm, pgm_read_float_near(&(derivativeTermLimits[controller])), -pgm_read_float_near(&(derivativeTermLimits[controller]))); // Limit the derivative controller
  
  /* Sum all terms */
  command = pTerm + iTerm /*+ dTerm*/;

  //hal.console->printf_P(PSTR("\n command: %f \n"), command);

  // Put a saturation limit on the output command
  Limit(command, pgm_read_float(&outputLimits[controller]), -pgm_read_float(&outputLimits[controller]));

  //hal.console->printf_P(PSTR("\n limited command: %f \n"), command);
    
  return command;
}

/* blah
 *
 */
void SetReference(unsigned int controller, float newValue)
{
  /* Check that the new reference input to command is within limited commands */
  Limit(newValue, pgm_read_float(&referenceLimits[controller][maximum_DEF]), pgm_read_float(&referenceLimits[controller][minimum_DEF]));
  
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

/* This function looks at the current roll state, airspeed state, and desired climb rate. It
 * then outputs a nominal pitch angle for the pitch controller to hold. The altitude controller
 * will augment this nominal pitch angle with a value to perturb this prescribed trim angle depending
 * on the commanded climb rate. In essence, this is just acting as a feed forward controller by knowing
 * a little about the system characteristics.
 */
/*
float SchedulePitchTrim(float rollCommand, float airspeedCommand, float climbRateCommand)
{
  float pitchAngleOut = 0.0; // pitch angle trim state (radians)

  // Straight and level flight, airspeed dependent component
  pitchAngleOut = SEVEN_MPS_PITCH_DEF + PITCH_TRIM_SLOPE_DEF*airspeedCommand;

  // Limit the pitch trim so that it doesn't become unstable in straight and level flight
  if(pitchAngleOut < 0.0)
  {
	  pitchAngleOut = 0.0;
  }

  // Take into account bank angle
  pitchAngleOut += (fabs(rollCommand)/referenceLimits[rollController_DEF][maximum_DEF])*PITCH_TRIM_BANK_MAX_DEF;

  // Take into account climb rate
  if(climbRateCommand > 0.0)
  {
	  pitchAngleOut += (climbRateCommand/MAX_CLIMB_RATE_DEF)*MAX_CLIMB_RATE_PITCH_DEF;
  }
  else if(climbRateCommand < 0.0)
  {
	  pitchAngleOut += (climbRateCommand/MIN_CLIMB_RATE_DEF)*MIN_CLIMB_RATE_PITCH_DEF;
  }
	
  return pitchAngleOut;
  
}
*/

#define EIGHT_MPS_PITCH 0.2442f //14.0 degrees
#define NINE_MPS_PITCH 0.09594f //5.5 degrees
#define TEN_MPS_PITCH 0.008722f //0.5 degrees
#define ELEVEN_MPS_PITCH 0.0f
#define TWELVE_MPS_PITCH 0.0f
#define THIRTEEN_MPS_PITCH 0.0f
#define FOURTEEN_MPS_PITCH -0.01744f // -1 degrees

float SchedulePitchTrim(float rollCommand, float airspeedCommand)
{
	float pitchTrimOut = 0.0;

	if(airspeedCommand < 8.5)
	{
		pitchTrimOut = EIGHT_MPS_PITCH;
	}
	else if(airspeedCommand >= 8.5 && airspeedCommand < 9.5)
	{
		pitchTrimOut = NINE_MPS_PITCH;
	}else if(airspeedCommand >= 9.5 && airspeedCommand < 10.5)
	{
		pitchTrimOut = TEN_MPS_PITCH;
	}else if(airspeedCommand >= 10.5 && airspeedCommand < 11.5)
	{
		pitchTrimOut = ELEVEN_MPS_PITCH;
	}else if(airspeedCommand >= 11.5 && airspeedCommand < 12.5)
	{
		pitchTrimOut = TWELVE_MPS_PITCH;
	}else if(airspeedCommand >= 12.5 && airspeedCommand < 13.5)
	{
		pitchTrimOut = THIRTEEN_MPS_PITCH;
	}else if(airspeedCommand >= 13.5);
	{
		pitchTrimOut = FOURTEEN_MPS_PITCH;
	}

	// Roll Command Schedule
	if(fabs(rollCommand) >= 0.131 && fabs(rollCommand) <= 0.3925 /* 22.5 degrees */)
	{
		if(airspeedCommand >= 11.0)
		{
			// This schedule was determined from experimental data taken. At 13.5 m/s and a 15 degree bank angle
			// the pitch trim must be increased by 0.5 degrees. At 11 m/s and a 15 degree bank angle, the pitch
			// trim must be increased by 5.5 degrees (from airspeed based nominal).
			pitchTrimOut += .095944 - 0.08722*(airspeedCommand - 11.0)/3.0;

			return pitchTrimOut;
		}else
		{
			// This schedule was determined from experimental data taken. At 11 m/s and a 15 degree bank angle
			// the pitch trim must be increased by 5.5 degrees. At 8 m/s and a 15 degree bank angle, the pitch trim
			// must be increased by 4.0 degrees.
			pitchTrimOut += .069777 + 0.02616*(airspeedCommand - 8.0)/3.0;

			return pitchTrimOut;
		}
	}else if(fabs(rollCommand) >= 0.3925)
	{
		if (airspeedCommand >= 11.0)
		{
			// This schedule was determined from experimental data taken. At 13.5 m/s and a 30 degree bank angle
			// the pitch trim must be increased by 4.75 degrees. At 11 m/s and a 15 degree bank angle, the pitch
			// trim must be increased by 9.0 degrees (from airspeed based nominal).
			pitchTrimOut += 0.157 - 0.07414*(airspeedCommand - 11.0)/3.0;

			return pitchTrimOut;
		}else
		{
			// This schedule was determined from experimental data taken. At 11 m/s and a 30 degree bank angle, 
			// the pitch trim must increase by 9.0 degrees to maintain level flight. We're going to leave this value
			// here and let the controller do the work for lower speeds than 11.0
			pitchTrimOut += 0.157;

			return pitchTrimOut;
		}
	}

	return pitchTrimOut;

}


/* blah
 *
 */
float ScheduleThrottleTrim(float airspeedCommand)
{
	float throttleTrim = 0.0;

	//throttleTrim = 50.0 + (50.0/(referenceLimits[airspeedController_DEF][maximum_DEF]-referenceLimits[airspeedController_DEF][minimum_DEF]))*(airspeedCommand-referenceLimits[airspeedController_DEF][minimum_DEF]);
	throttleTrim = 25.0 + 75.0*(airspeedCommand-pgm_read_float(&referenceLimits[airspeedController_DEF][minimum_DEF]))/(pgm_read_float(&referenceLimits[airspeedController_DEF][maximum_DEF])-pgm_read_float(&referenceLimits[airspeedController_DEF][minimum_DEF]));
	return throttleTrim;
}

/* blah
 *
 */
void ScheduleHeadingGain(float airspeedCommand)
{
	//pgm_read_float(&gains[headingController_DEF][pGain]) = .5 + .4*(airspeedCommand - pgm_read_float(&referenceLimits[airspeedController_DEF][minimum_DEF]))/(pgm_read_float(&referenceLimits[airspeedController_DEF][maximum_DEF]) - pgm_read_float(&referenceLimits[airspeedController_DEF][maximum_DEF]));
	//pgm_read_float(&gains[headingController_DEF][iGain]) = .003 + .0035*(airspeedCommand - pgm_read_float(&referenceLimits[airspeedController_DEF][minimum_DEF]))/(pgm_read_float(&referenceLimits[airspeedController_DEF][maximum_DEF]) - pgm_read_float(&referenceLimits[airspeedController_DEF][maximum_DEF]));
}

/* blah
 *
 */
void DiscretizeAirspeedCommand(float &airspeedCommand)
{
	if(airspeedCommand <= 8.5)
	{
		airspeedCommand = 8.5;
	}else if(airspeedCommand <= 9.5 && airspeedCommand > 8.5)
	{
		airspeedCommand = 9.0;
	}else if(airspeedCommand <= 10.5 && airspeedCommand > 9.5)
	{
		airspeedCommand = 10.0;
	}else if(airspeedCommand <= 11.5 && airspeedCommand > 10.5)
	{
		airspeedCommand = 11.0;
	}else if(airspeedCommand <= 12.5 && airspeedCommand > 11.5)
	{
		airspeedCommand = 12.0;
	}else if(airspeedCommand <= 13.5 && airspeedCommand > 12.5)
	{
		airspeedCommand = 13.0;
	}else if( airspeedCommand > 13.5)
	{
		airspeedCommand = 13.5;
	}
}

/* blah
 *
 */
/*
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
}*/