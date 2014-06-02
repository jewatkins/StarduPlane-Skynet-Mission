#ifndef MISSION_PLAN_H
#define MISSION_PLAN_H

#include <math.h>
#include "AA241X_ControlLaw.h"
#include "AA241X_WaypointNavigation.h"
#include "AA241X_Phase2_Waypoint.h"
#include "AA241X_parseSnapshot.h"
#include "AA241X_MissionPlan_Parameters.h"

/**** Phase-1: Spiral ****/
// Initialize spiral
static void InitPhase1Spiral() {
  // Initialize G_inc
  uint8_t i,j,k;
  for (i=0; i<Ntargets; i++) {
	  for (j=0; j<Ndim; j++) {
		  for (k=0; k<nG; k++) {
			  G_inc[i][j][k] = 0.0;
		  }
	  }
  }

  // Initialize iorder
  iorder = 0;

  // Snapshot parameters
  no_snap = 0;

  // Mission Planner Parameter
  INIT_SPIRAL = 1.0;

  // Start timer
  t_init = CPU_time_ms;

  // Set initial start position
  x_init = X_position;
  y_init = Y_position;

  // Set waypoint iterator
  iwp = 0;

  // Get first waypoint
  GetWaypointPhase1();

  // Compute heading (waypoint tangent line)
  Hwp = WrapAngle(atan2f(ywp,xwp) + PI/2);
}

// Capture waypoints only when successful snapshots are taken
static void Phase1() {
  // Check to see if waypoint is found
  float dx = xwp - X_position;
  float dy = ywp - Y_position;
  pos_error = sqrtf(dx*dx + dy*dy);

  float error_lim;
  if (iwp < entryPts) {
	  error_lim = INITIAL_ERROR;
  }
  else {
	  error_lim = SNAPSHOT_ERROR;
  }

  if (pos_error <= error_lim) {
    // Take a snapshot
    snapshot mySnapShot = takeASnapshot();

    // Post process results and go to next waypoint if snapshot is taken
    if (mySnapShot.pictureTaken == 1) {
      uint16_t i;
      for (i=0; i<Ntargets; i++) {
        // Check if person found
        if (mySnapShot.personsInPicture[i] == 1) {
          persons_found[i] = 1;
          gcs_send_text_fmt(PSTR("Person %d found"),i+1);
          
          // Collect snapshot data
          uint16_t isnap = n_snaps[i];
          G_inc[i][0][isnap] = mySnapShot.centerOfPictureX;
          G_inc[i][1][isnap] = mySnapShot.centerOfPictureY;
          G_inc[i][2][isnap] = mySnapShot.diameterOfPicture;
          G_inc[i][3][isnap] = mySnapShot.centerOfPersonEstimateX[iTarget];
          G_inc[i][4][isnap] = mySnapShot.centerOfPersonEstimateY[iTarget];

		  // Set order
		  if (isnap == 0) {
		    order[(Ntargets-1)-iorder] = i;
		    iorder++;
		  }
          
          // Add to snapshot counter
          n_snaps[i]++;
          
          // Sum all persons found
          uint16_t ii;
          n_persons_found = 0;
          for (ii=0; ii<Ntargets; ii++) {
            n_persons_found += persons_found[ii];
          }

          // Display message if all persons found
          if (n_persons_found == Ntargets && finalize_t_sight_flag == 1) {
            finalize_t_sight_flag = 0;
            t_sight_end = CPU_time_ms;
            gcs_send_text_P(SEVERITY_LOW, PSTR("All Persons found!"));
          }
        }
      }

      // Go to next waypoint
      iwp++;

      // If all waypoints complete, restart route
      if (iwp >= Nwp || n_persons_found == Ntargets) {
        phase_flag = 2;
        iwp = 0;
        return;
      }

      // Get new waypoints
      float xwp_old = xwp;
      float ywp_old = ywp;
      GetWaypointPhase1();

      // Start timer
      t_init = CPU_time_ms;

      // Compute heading (waypoint to waypoint or waypoint tangent line)
      if (trans_flag == 1) {
        dx = xwp - xwp_old;
        dy = ywp - ywp_old;
        Hwp = WrapAngle(atan2f(dy,dx));
      }
      else {
        Hwp = WrapAngle(atan2f(ywp,xwp) + PI/2);
      }
    }
    else {
      no_snap++;
      gcs_send_text_P(SEVERITY_LOW, PSTR("Snapshot not taken!"));
    }
  }
}

/**** Phase-2: Simple ****/
// Initialize refinement of first target
//static void InitPhase2Simple() {
  /*
  // Test data for phase-2 simple
  X_person_estimate[0] = -100;   X_person_estimate[1] = -100;    X_person_estimate[2] =  100;  X_person_estimate[3] = 100;
  Y_person_estimate[0] = 100;    Y_person_estimate[1] = -100;    Y_person_estimate[2] = -100;  Y_person_estimate[3] = 100;
              order[0] = 3;                  order[1] = 2;                   order[2] = 1;                 order[3] = 0;
  */

  // Mission Planner Parameter
//  INIT_PHASE2 = 1.0;
  
  // Start timer
//  t_init = CPU_time_ms;
  
  // Set target order iterator
  //iorder = 0;
  //SetTarget();

  // Get first waypoint
  //GetWaypointPhase2();

  // Compute heading (waypoint tangent line)
  //float dx = xwp - x_target;
  //float dy = ywp - y_target;
  //Hwp = WrapAngle(atan2f(dy,dx) + PI/2);
//}

// Initialize refinement of first target
static void InitPhase2() {

  // Mission Planner Parameter
  INIT_PHASE2 = 1.0;
  
  // Start timer
  t_init = CPU_time_ms;
  
  // Set target order iterator
  iorder = 0;
  SetTarget();

  // Get first waypoint
  GetWaypointPhase2();

  // Compute heading (waypoint tangent line)
  float dx = xwp - y_centroid[0];
  float dy = ywp - y_centroid[1];
  Hwp = WrapAngle(atan2f(dy,dx) + PI/2);
}

// Capture waypoints, take snapshots every 3 seconds
static void Phase2() 
{  
  /*// Check to see if snapshot is available
  float dt = (CPU_time_ms - t_init)/1000;
  if (dt > 3.0) {
    // Take a snapshot
    snapshot mySnapShot = takeASnapshot();
      
      // Parse snapshots and continue to next target if all snapshots complete
      if (parseSnapshot(mySnapShot)) {
        n_snaps[iTarget] = n_Inc;
        iorder++;
        SetTarget();
      }

	  // 
	  if (iorder >= Ntargets) {
		  phase_flag = 3;
		  return;
	  }
    }
  */
  // Check to see if waypoint is found
  float dx = xwp - X_position;
  float dy = ywp - Y_position;
  pos_error = sqrtf(dx*dx + dy*dy);
  if (pos_error <= SNAPSHOT_ERROR) {
    // Get new waypoint
    GetWaypointPhase2();

    // Compute heading (waypoint tangent line)
    float dx = xwp - y_centroid[0];
    float dy = ywp - y_centroid[1];
    Hwp = WrapAngle(atan2f(dy,dx) + PI/2);
  }
}



/**** Phase-3: Circle center of lake lag ****/
// Circle center of lake lag
static void Phase3() {
	// Set target to center of lake
	y_centroid[0] = 0.0;
	y_centroid[1] = 0.0;
	GetWaypointPhase2();

    // Compute heading (waypoint tangent line)
    float dx = xwp - y_centroid[0];
    float dy = ywp - y_centroid[1];
    Hwp = WrapAngle(atan2f(dy,dx) + PI/2);
}
#endif /* MISSION_PLAN_H */


