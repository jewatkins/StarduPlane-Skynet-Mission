#include "AA241X_ControllerFunctions.h"
#include "AA241X_ControllerFunctionsConfig.h"

float StepController(uint8_t controller, float measured, float &delta_t)
{
  float command = 0.0;
  float error = references[controller] - measured;
  float pCommand = 0.0;
  float iCommand = 0.0;
  float dCommand = 0.0;
  
  /* Find proportional term */
  pCommand = gains[controller][pGain]*error; 
  
  /* Sum all terms */
  command = pCommand + iCommand + dCommand;
  
  //Limit(command, limits[controller][maximum], limits[controller][minimum]);
  
  return command;
}

/* blah
 *
 */
void SetReference(uint8_t controller, float newValue)
{
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
