#ifndef WAYPOINT_NAVIGATION_H
#define WAYPOINT_NAVIGATION_H

#include <math.h>
#include "AA241X_ControlLaw.h"
#include "AA241X_MissionPlan_Parameters.h"

// Wrap angle to interval [0,2*pi]
static float WrapAngle(float angle) {
  float twoPi = 2*PI;
  return angle - twoPi*(float)floor(angle/twoPi);
}

// Returns waypoint number 'wp' for phase-1 assuming that the airspeed is
// maintained at 13 m/s and there are entryPts+53 waypoints
static void GetWaypointPhase1() {
  // Set parameters
  uint16_t wp = iwp + (uint16_t)1;
  trans_flag = 0;
  float v = 13.0;
  float RC[4] = {143.4739, 98.3224, 53.8284, 12.4141};
  float tau[4] = {0.1500f, 0.2667f, 0.3833f, 0.5000f};
  float rad0 = sqrt(x_init*x_init+ y_init*y_init);
  float rotation = WrapAngle(atan2f(y_init,x_init));

  // Entry points
  float omega = 1.0*v/rad0;
  float ang = 0.0;
  float rad = 0.0;
  if (wp <= entryPts) {
    ang = omega*ts*(float)wp + rotation;
    rad = rad0 + (RC[0] - rad0)* sqrt((float)wp / (float)entryPts);
    xwp = rad*cosf(ang);
    ywp = rad*sinf(ang);
    return;
  }
  ang = omega*ts*(float)entryPts + rotation;
  float theta = WrapAngle(ang);
  wp = wp - entryPts;

  // Circle 1
  omega = v/RC[0];
  float T = (2*PI - omega*ts) / omega;
  uint16_t nTp = (uint16_t)ceilf(T/ts);
  if (wp <= nTp) {
    ang = theta + omega*ts*(float)wp;
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
    if (wp <= nTp) {
      trans_flag = 1;
      rad = (RC[iC-1] + (RC[iC]-RC[iC-1])*(float)(wp-nTm)/ (float)(nTp-nTm)); // Probably never zero
      ang = theta + omega*ts*(float)(wp-nTm);
      xwp = rad*cosf(ang);
      ywp = rad*sinf(ang);
      return;
    }
    ang = theta + omega*ts*(float)(nTp-nTm);
    theta = WrapAngle(ang);
    nTm = nTp;

    // Circle 2
    omega = v/RC[iC];
    T = (2*PI - omega*ts) / omega;
    nTp = nTm + (uint16_t)ceilf(T/ts);
    if (wp <= nTp) {
      rad = RC[iC];
      ang = theta + omega*ts*(float)(wp-nTm);
      xwp = rad*cosf(ang);
      ywp = rad*sinf(ang);
      return;
    }
    ang = theta + omega*ts*(float)(nTp-nTm);
    theta = WrapAngle(ang);
    nTm = nTp;
  }
}

static float GetNavAirspeed() {
  /*
  // Estimate needed airspeed to reach waypoint at correct time
  float dx = xwp - X_position;
  float dy = ywp - Y_position;
  float ds = sqrtf(dx*dx + dy*dy);
  float dt = TIME_ESTIMATE - (CPU_time_ms - t_init)/1000;
  float ASCommand = ds/dt;

  // Airspeed limiter
  if (ASCommand < 0.0 || ASCommand > 13.0) {
    ASCommand = 13.0;
  }
  else if (ASCommand < 9.0) {
    ASCommand = 9.0;
  }
  ASCommand = 9.0;
  */
  
  // Set constant airspeed
  float ASCommand = 11.0;
  if (phase_flag == 1) {
	  ASCommand = 12.0;
  }
  else if (phase_flag == 2) {
	  ASCommand = v_phase2;
  }
  else if (phase_flag == 3) {
	  ASCommand = 9.5;
  }

  return ASCommand;
}

static float GetNavHeading() {
  // Calculate position error
  float dx = xwp - X_position;
  float dy = ywp - Y_position;
  pos_error = sqrtf(dx*dx + dy*dy);

  // Compute heading (UAV to waypoint)
  float Huav = WrapAngle(atan2f(dy,dx));

  // Compute heading error (rad)
  float Herr = (float)fabs(Huav - Hwp);

  // Determine shortest angle and compute heading command
  // Note: A line tracking gain of "ROUTE_P = 1" means set heading to waypoint.
  float HCommand;
  if (Herr < (2*PI - Herr)) {
    HCommand = WrapAngle(Hwp + copysignf(1.0, Huav - Hwp)*ROUTE_P*Herr);
  }
  else {
    Herr = 2*PI - Herr;
    HCommand = WrapAngle(Hwp - copysignf(1.0, Huav - Hwp)*ROUTE_P*Herr);
  }

  // If Herr is too large or if pos_error is small enough, set heading to waypoint.
  if (Herr >= PI/4 || pos_error <= POSITION_ERROR) {
    HCommand = Huav;
  }
  return HCommand;
}

#endif /* WAYPOINT_NAVIGATION_H */




