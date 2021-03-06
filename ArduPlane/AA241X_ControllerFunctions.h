#ifndef CONTROLLER_FUNCTIONS_H
#define CONTROLLER_FUNCTIONS_H

#include "AA241X_ControllerFunctionsConfig.h"

// This is the header file for all of the control loop functions that will be used in the ControlLaw file

/* This function takes in a controller ID, a measured value, a reference value, and the amount of time since the last
 * run through the FAST LOOP in the ControlLaw.ino file. The function will determine the error, the derivative error,
 * and the integral error and then determine the controller output to send out to aircraft controls.
 */
float StepController(unsigned int controller, float measured, float &delta_t);

/* blah
 *
 */
void SetReference(unsigned int controller, float newValue);

/* blah
 *
 */
void Limit(float &variable, float maximum, float minimum);

/* blah
 *
 */
float SchedulePitchTrim(float rollCommand, float airspeedCommand);

/* blah
 *
 */
float ScheduleThrottleTrim(float airspeedCommand);

/* blah
 *
 */
void ScheduleHeadingGain(float airspeedCommand);

/* blah
 *
 */
void DiscretizeAirspeedCommand(float &airspeedCommand);

/* blah
 *
 */
//char determineTrimState(float rollCommand, float airspeedCommand);

#endif /* CONTROLLER_FUNCTIONS_H */
