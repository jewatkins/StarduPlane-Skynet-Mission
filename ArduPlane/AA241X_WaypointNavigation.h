#ifndef WAYPOINT_NAVIGATION_H
#define WAYPOINT_NAVIGATION_H

/**** Waypoint Navigation ****/
#define MISSION 1                              // controlMode for Mission
#define TIME_ESTIMATE 3.2f                     // Time estimate between waypoints (s)
#define Ndim 2                                 // Dimension of waypoints
#define Np 4                                   // Number of persons to find
static char persons_found[Np] = {0,0,0,0};     // Number of persons found

// Wrap angle to interval [0,2*pi]
static float WrapAngle(float angle) {
  float twoPi = 2*PI;
  return angle - twoPi*(float)floor(angle/twoPi);
}

// Returns waypoint number 'wp' for phase-1 assuming that the airspeed is
// maintained at 11 m/s and there are 62 waypoints
#define Nwp 62
static float t0 = 0;      // Initial time between waypoints
static uint16_t iwp = 0;  // Waypoint iterator
static float xwp = 0.0f; static float ywp = 0.0f;
static float Hwp = 0.0f;
static void GetWaypoint(uint16_t wp, float *xpos, float *ypos) {
  // Set parameters
  wp = wp + 1;
  float ts = 3.0;
  float v = 11.0;
  float RC[4] = {150.8251f, 101.0875f, 52.0025f, 10.5042f};
  float tau[4] = {0.1500f, 0.2667f, 0.3833f, 0.5000f};
  float theta = 1.7146f;
  float offset = 1.6756f;
  
  // Circle 1
  float omega = v/RC[0];
  float T = (2*PI - tau[0] - theta + offset) / omega;
  float nTp = ceilf(T/ts);
  float ang;
  if ((float)wp <= nTp) {
      ang = theta + omega*ts*(float)wp;
      *xpos = RC[0]*cosf(ang);
      *ypos = RC[0]*sinf(ang);
      return;
  }
  ang = theta + omega*ts*nTp;
  theta = WrapAngle(ang);
  float nTm = nTp;
  
  uint16_t iC;
  for (iC=1; iC<4; iC++) {
    // Transition from circle 1 to circle 2
    omega = v/RC[iC-1];
    T = (2*tau[iC-1]) / omega;
    nTp = nTm + ceilf(T/ts);
    float rad;
    if ((float)wp <= nTp) {
        rad = (RC[iC-1] + (RC[iC]-RC[iC-1])*((float)wp-nTm)/ (nTp-nTm));
        ang = theta + omega*ts*((float)wp-nTm);
        *xpos = rad*cosf(ang);
        *ypos = rad*sinf(ang);
        return;
    }
    ang = theta + omega*ts*(nTp-nTm);
    theta = WrapAngle(ang);
    nTm = nTp;
    
    // Circle 2
    omega = v/RC[iC];
    T = (2*PI - theta - tau[iC] + offset) / omega;
    nTp = nTm + ceilf(T/ts);
    if ((float)wp <= nTp) {
        rad = RC[iC];
        ang = theta + omega*ts*((float)wp-nTm);
        *xpos = rad*cosf(ang);
        *ypos = rad*sinf(ang);
        return;
    }
    ang = theta + omega*ts*(nTp-nTm);
    theta = WrapAngle(ang);
    nTm = nTp;
  }
}
#endif /* WAYPOINT_NAVIGATION_H */
