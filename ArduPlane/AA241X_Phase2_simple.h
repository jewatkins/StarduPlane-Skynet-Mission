//
//  Phase2_simple.h
//

#ifndef PHASE2_SIMPLE_H
#define PHASE2_SIMPLE_H

#include <math.h>

// Assumed time between snapshots of 3.2s for the search parameters
#define searchRad 25.5071 // Radius of circular path centered around initial target estimate
#define dTheta 0.8782 // 50 degrees

// Calculates circular trajectory around a target
// xwp -> current x position of waypoint in output
// ywp -> current y position of waypoint in output
// x_target and y_target -> Estimated x and y locations of target (Simplest is center of FoV)
void createCircleAroundTarget(){

  float theta;

  //Find current polar angle
  theta = atan2( Y_position - y_target, X_position - x_target );
  if (theta < 0)
    theta += 2*PI;

  // Calculate new polar angle
  theta += dTheta;
  if (theta > 2*PI)
    theta -= 2*PI;

  xwp = x_target + searchRad * cosf(theta);
  ywp = y_target + searchRad * sinf(theta);
}

void GetWaypointPhase2(){

  float theta;


  // If not on circle, go to the quadrature point
  if ( hypotf( Y_position - y_target, X_position - x_target ) > 1.2*searchRad ){
    //Find current polar angle
    theta = atan2( Y_position - y_target, X_position - x_target );
    if (theta < 0)
      theta += 2*PI;

    // Calculate new polar angle
    theta += PI/2;
    if (theta > 2*PI)
      theta -= 2*PI;

    xwp = x_target + searchRad * cosf(theta);
    ywp = y_target + searchRad * sinf(theta);
  }
  else {
    createCircleAroundTarget();
  }
}

static void SetTarget(){
  iTarget = order[iorder];
  x_target = X_person_estimate[iTarget];
  y_target = Y_person_estimate[iTarget];
}

// Estimate all target locations by using a simple average
static void EstimateTargetLocation() {
  uint8_t i,j;
  float sum_x, sum_y;
  for (i=0; i<Ntargets; i++) {
    sum_x = 0.0;
    sum_y = 0.0;
    for (j=0; j < n_snaps[i]; j++) {
      sum_x += G_inc[i][0][j];
      sum_y += G_inc[i][1][j];
    }
    if (n_snaps[i] != 0) {
      X_person_estimate[i] = sum_x/n_snaps[i];
      Y_person_estimate[i] = sum_y/n_snaps[i];
    }
  }
}

#endif /* PHASE2_SIMPLE_H */

