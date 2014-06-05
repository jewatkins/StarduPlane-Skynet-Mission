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


/*
// Check tsight and move to phase 2 if all persons found
static void CheckTSight(snapshot &mySnapShot) {
  uint8_t i;
  for (i=0; i<Ntargets; i++) {
    // Check if person found
    if (mySnapShot.personsInPicture[i] == 1 && n_snaps[i] < nG) {
      // Collect snapshot data
      uint16_t isnap = n_snaps[i];
      G_inc[i][0][isnap] = mySnapShot.centerOfPictureX;
      G_inc[i][1][isnap] = mySnapShot.centerOfPictureY;
      G_inc[i][2][isnap] = mySnapShot.diameterOfPicture;
      G_inc[i][3][isnap] = mySnapShot.centerOfPersonEstimateX[i];
      G_inc[i][4][isnap] = mySnapShot.centerOfPersonEstimateY[i];
      
      // Set order
      if (isnap == 0) {
        persons_found[i] = 1;
        //order[(Ntargets-1)-iorder] = i;
        iorder++;
      }
      
      // Add to snapshot counter
      n_snaps[i]++;
    }
  }
  
  // Sum all persons found
  n_persons_found = 0;
  for (i=0; i<Ntargets; i++) {
    n_persons_found += persons_found[i];
  }

  // Go to phase 2 if all persons found
  if (n_persons_found == Ntargets) {
    t_sight = CPU_time_mission_ms;
    for (i=0; i<Ntargets; i++) {
      X_person_estimate[i] = G_inc[i][0][0];
      Y_person_estimate[i] = G_inc[i][1][0];
    }
    phase_flag = 2;
  }
}


// Transition from Climb to Spiral: take snapshots every 3 seconds
static void Phase1Trans() {
  // Check to see if waypoint is found
  float dx = xwp - X_position;
  float dy = ywp - Y_position;
  pos_error = sqrtf(dx*dx + dy*dy);
  if (pos_error <= INITIAL_ERROR) {
    // Go to next waypoint
    iwp++;

    // Get new waypoint
    GetWaypointPhase1();

    // Compute heading (waypoint tangent line)
    Hwp = WrapAngle(atan2f(ywp,xwp) + PI/2);
  }
}


// Capture waypoints only when successful snapshots are taken
static void Phase1() {
  // Check to see if waypoint is found
  float dx = xwp - X_position;
  float dy = ywp - Y_position;
  pos_error = sqrtf(dx*dx + dy*dy);
  if (pos_error <= SNAPSHOT_ERROR) {
    // Take a snapshot
    snapshot mySnapShot = takeASnapshot();

    // Post process results and go to next waypoint if snapshot is taken
    if (mySnapShot.pictureTaken == 1) {
      // Check tsight and move to phase 2 if all persons found
      CheckTSight(mySnapShot);

      // Go to next waypoint
      iwp++;

      // If all waypoints complete, restart route
      if (iwp >= Nwp + entryPts) {
        iwp = entryPts;
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
  }
}
*/


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
      uint8_t i;
      for (i=0; i<Ntargets; i++) {
        // Check if person found
        if (mySnapShot.personsInPicture[i] == 1 && n_snaps[i] < nG) {
          persons_found[i] = 1;
          
          // Collect snapshot data
          uint16_t isnap = n_snaps[i];
          G_inc[i][0][isnap] = mySnapShot.centerOfPictureX;
          G_inc[i][1][isnap] = mySnapShot.centerOfPictureY;
          G_inc[i][2][isnap] = mySnapShot.diameterOfPicture;
          G_inc[i][3][isnap] = mySnapShot.centerOfPersonEstimateX[i];
          G_inc[i][4][isnap] = mySnapShot.centerOfPersonEstimateY[i];

		  // Set order
		  if (isnap == 0) {
		    //order[(Ntargets-1)-iorder] = i;
		    iorder++;
		  }
          
          // Add to snapshot counter
          n_snaps[i]++;
          
          // Sum all persons found
          uint8_t ii;
          n_persons_found = 0;
          for (ii=0; ii<Ntargets; ii++) {
            n_persons_found += persons_found[ii];
          }

          // Display message if all persons found
          if (n_persons_found == Ntargets && finalize_t_sight_flag == 1) {
            finalize_t_sight_flag = 0;
            t_sight_end = CPU_time_ms;
          }
        }
      }

      // Go to next waypoint
      iwp++;

      // If all waypoints complete, restart route
      if (iwp >= Nwp + entryPts || n_persons_found == Ntargets) {
		uint8_t j;
		for (j=0; j<Ntargets; j++) {
		  X_person_estimate[j] = G_inc[j][0][0];
		  Y_person_estimate[j] = G_inc[j][1][0];
		}
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


/**** Phase-2 ****/
// Initialize refinement of first target
static void InitPhase2() {

  // Mission Planner Parameter
  INIT_PHASE2 = 1.0;
  
  // Start timer
  t_init = CPU_time_ms;
  
  /*
  // Set battery energy limit for first target (Joules)
  energy_limit = mission_energy_consumed + (ENERGY_LIMIT - mission_energy_consumed)/Ntargets;
  */
  
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
  // Check to see if waypoint is found
  float dx = xwp - X_position;
  float dy = ywp - Y_position;
  pos_error = sqrtf(dx*dx + dy*dy);
  if (pos_error <= 2.0*SNAPSHOT_ERROR) {
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


