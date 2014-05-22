
#include <math.h>
#include <AP_Math.h>
#include "defines.h"
#include "AA241X_ControlLaw.h"
#include "AA241X_aux.h"

static int8_t     my_signed_8_bit_variable = 10;  // integer numbers between -128 and 127
static uint8_t    my_unsigned_8_bit_variable = 10;  // positive integer numbers between 0 and 255

static int16_t    my_signed_16_bit_variable = 10;  // integer numbers between −32768 and 32767
static uint16_t   my_unsigned_16_bit_variable = 10;  // positive integer numbers between 0 and 65535

static int32_t    my_signed_32_bit_variable = 10;  // integer numbers between −2147483648 and 2147483647
static uint32_t   my_unsigned_32_bit_variable = 10;  // positive integer numbers between 0 and 4294967295
static float      my_float;

// These functions are executed when control mode is in AUTO
// Please read AA241X_aux.h for all necessary definitions and interfaces

// *****   AA241X Fast Loop - @ ~50Hz   ***** //
static void AA241X_AUTO_FastLoop(void) {
  // YOUR CODE HERE
  // Example:
  static float cos_roll;
  static float sin_roll;

//  static struct snapshot mySnapShot = takeASnapshot();
  
  // Using my parameter 1 in the code. Note that the name of the variable
  my_float = my_param_1;
  
  // Checking if we've just switched to AUTO. If more than 100ms have gone past since last time in AUTO, then we are definitely just entering AUTO
  if ((CPU_time_ms - Last_AUTO_stampTime_ms) > 100) {
    // Then we've just switched to AUTO, and need to save the flight 
    // data as the trim condition we want to control around. Used for PS2
  }
  
  cos_roll = cos(roll);
  sin_roll = sin(roll);

  Roll_servo     = RC_Roll_Trim + (2*roll/PI)*100;
  Pitch_servo    = RC_Pitch_Trim + (2*pitch/PI)*100;
  Throttle_servo = 0;
  Rudder_servo   = RC_Rudder_Trim + (2*yaw/PI)*100;
};






// *****   AA241X Medium Loop - @ ~10Hz  *****  //
static void AA241X_AUTO_MediumLoop(void){
  // YOUR CODE HERE
  


  
  
  
  
};





// *****   AA241X Slow Loop - @ ~1Hz  *****  //
static void AA241X_AUTO_SlowLoop(void){
  // YOUR CODE HERE
  
  
  
};





