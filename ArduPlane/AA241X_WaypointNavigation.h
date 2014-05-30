#ifndef WAYPOINT_NAVIGATION_H
#define WAYPOINT_NAVIGATION_H

#include <math.h>

/**** Waypoint Navigation ****/
#define TIME_ESTIMATE 3.5f                     // Time estimate between waypoints (s)
#define Np 4                                   // Number of persons to find
static char persons_found[Np] = {0,0,0,0};     // Number of persons found
static uint8_t init_t_sight_flag = 1;
static uint8_t finalize_t_sight_flag = 1;
static float t_sight_start = 0.0;
static float t_sight_end = 0.0;
static uint16_t no_snap = 0;
static char n_persons_found = 0;

// Wrap angle to interval [0,2*pi]
static float WrapAngle(float angle) {
  float twoPi = 2*PI;
  return angle - twoPi*(float)floor(angle/twoPi);
}

// Returns waypoint number 'wp' for phase-1 assuming that the airspeed is
// maintained at 11 m/s and there are 56 waypoints
#define Nwp 54
static uint8_t init_flag = 1;
static uint8_t trans_flag = 0;
static float t_init = 0.0f;                              // Initial time between waypoints
static uint16_t iwp = 0;                                 // Waypoint iterator
static float x_init = 0.0f; static float y_init = 0.0f;  // Initial start position
static float xwp = 0.0f;    static float ywp = 0.0f;     // Waypoint
static float Hwp = 0.0f;                                 // Heading: waypoint tangent line
static float pos_error = 0.0f;							 // Position Error
static void GetWaypoint() {
  // Set parameters
  uint16_t wp = iwp + (uint16_t)1;
  trans_flag = 0;
  float ts = 3.0;
  float v = 11.0;
  float RC[4] = {140.1838f, 89.3775f, 39.4867f, 9.5493f};
  float tau[4] = {0.1500f, 0.2667f, 0.3833f, 0.5000f};
  float theta = 1.6414f;
  float offset = 1.6546f;
  float rotation = WrapAngle(atan2f(y_init,x_init)) - theta;
  
  // Circle 1
  float omega = v/RC[0];
  float T = (2*PI - tau[0] - theta + offset) / omega;
  uint16_t nTp = (uint16_t)ceilf(T/ts);
  float ang = 0.0;
  if (wp <= nTp) {
      ang = theta + omega*ts*(float)wp;
      ang += rotation;
      xwp = RC[0]*cosf(ang);
      ywp = RC[0]*sinf(ang);
      return;
  }
  ang = theta + omega*ts*(float)nTp;
  theta = WrapAngle(ang);
  uint16_t nTm = nTp;
  
  uint16_t iC = 0;
  for (iC=1; iC<4; iC++) {
    // Transition from circle 1 to circle 2
    omega = v/RC[iC-1];
    T = (2*tau[iC-1]) / omega;
    nTp = nTm + (uint16_t)ceilf(T/ts);
    float rad = 0.0;
    if (wp <= nTp) {
		trans_flag = 1;
        rad = (RC[iC-1] + (RC[iC]-RC[iC-1])*(float)(wp-nTm)/ (float)(nTp-nTm)); // Probably never zero
        ang = theta + omega*ts*(float)(wp-nTm);
        ang += rotation;
        xwp = rad*cosf(ang);
        ywp = rad*sinf(ang);
        return;
    }
    ang = theta + omega*ts*(float)(nTp-nTm);
    theta = WrapAngle(ang);
    nTm = nTp;
    
    // Circle 2
    omega = v/RC[iC];
    T = (2*PI - theta - tau[iC] + offset) / omega;
    nTp = nTm + (uint16_t)ceilf(T/ts);
    if (wp <= nTp) {
        rad = RC[iC];
        ang = theta + omega*ts*(float)(wp-nTm);
        ang += rotation;
        xwp = rad*cosf(ang);
        ywp = rad*sinf(ang);
        return;
    }
    ang = theta + omega*ts*(float)(nTp-nTm);
    theta = WrapAngle(ang);
    nTm = nTp;
  }
}


#endif /* WAYPOINT_NAVIGATION_H */
