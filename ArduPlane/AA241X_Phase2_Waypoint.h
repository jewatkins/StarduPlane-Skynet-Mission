//
//  Phase2_Waypoint.h
//

#ifndef PHASE2_WAYPOINT_H
#define PHASE2_WAYPOINT_H

#include <math.h>
#include "AA241X_initRefine.h"

// Assumed time between snapshots of 3.2s for the search parameters
static float dTheta = (v_phase2+3) * ts / Rc; // 78 degrees

// Calculates circular trajectory around a target
// xwp -> current x position of waypoint in output
// ywp -> current y position of waypoint in output
// y_centroid[0] and y_centroid[1] -> Estimated x and y locations of target (Simplest is center of FoV)
void createCircleAroundTarget(){

  float theta;

  //Find current polar angle
  theta = atan2( Y_position - y_centroid[1], X_position - y_centroid[0] );
  if (theta < 0)
    theta += 2*PI;

  // Calculate new polar angle
  theta += dTheta;
  if (theta > 2*PI)
    theta -= 2*PI;

  xwp = y_centroid[0] + Rc * cosf(theta);
  ywp = y_centroid[1] + Rc * sinf(theta);
}

void GetWaypointPhase2(){

  float theta;


  // If not on circle, go to the quadrature point
  if ( hypotf( Y_position - y_centroid[1], X_position - y_centroid[0] ) > 1.2*Rc ){
    //Find current polar angle
    theta = atan2( Y_position - y_centroid[1], X_position - y_centroid[0] );
    if (theta < 0)
      theta += 2*PI;

    // Calculate new polar angle
    theta += PI/2;
    if (theta > 2*PI)
      theta -= 2*PI;

    xwp = y_centroid[0] + Rc * cosf(theta);
    ywp = y_centroid[1] + Rc * sinf(theta);
  }
  else {
    createCircleAroundTarget();
  }
}

static void SetTarget(){
  iTarget = order[iorder];
  initRefine();
}

#endif /* PHASE2_WAYPOINT_H */

