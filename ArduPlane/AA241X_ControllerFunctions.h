#ifndef CONTROLLER_FUNCTIONS_H
#define CONTROLLER_FUNCTIONS_H

#include "AA241X_ControllerFunctionsConfig.h"

// This is the header file for all of the control loop functions that will be used in the ControlLaw file

/* This function takes in a controller ID, a measured value, a reference value, and the amount of time since the last
 * run through the FAST LOOP in the ControlLaw.ino file. The function will determine the error, the derivative error,
 * and the integral error and then determine the controller output to send out to aircraft controls.
 */
float StepController(uint8_t controller, float measured, float &delta_t);

/* blah
 *
 */
void SetReference(uint8_t controller, float newValue);

/* blah
 *
 */
void Limit(float &variable, float maximum, float minimum);

// nominal trim states for the controllers
struct trimState_t
{
  float pitch;
  float roll;
  float airspeed;
};

/* blah
 *
 */
struct trimState_t ScheduleTrim(float rollCommand, float airspeedCommand, char phaseOfFlight);

/* blah
 *
 */
char determineTrimState(float rollCommand, float airspeedCommand);

/* Controller Variables */
static float references[numControllers]; // reference inputs to the controllers
static float intErrors[numControllers];  // Need to track iterm over multiple iterations
static float prevErrors[numControllers]; // Need to track integral error over multiple iterations

#endif /* CONTROLLER_FUNCTIONS_H */
