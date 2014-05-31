#ifndef MISSION_PLAN_H
#define MISSION_PLAN_H

#include <math.h>
#include "AA241X_MissionPlan_Parameters.h"
#include "AA241X_ControlLaw.h"
#include "AA241X_WaypointNavigation.h"
#include "AA241X_Phase2_simple.h"

/**** Phase-1: Spiral ****/
// Initialize spiral
static void InitPhase1Spiral() {
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
  if (pos_error <= SNAPSHOT_ERROR) {
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
      if (iwp >= Nwp) {
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
static void InitPhase2Simple() {
  // Test data for phase-2 simple
  X_person_estimate[0] = 65;   X_person_estimate[1] = 110;  X_person_estimate[2] = 50;  X_person_estimate[3] = 10;
  Y_person_estimate[0] = 110;  Y_person_estimate[1] = 20;   Y_person_estimate[2] = 12;  Y_person_estimate[3] = 30;
              order[0] = 3;                order[1] = 2;                order[2] = 1;               order[3] = 0;
  
  // Mission Planner Parameter
  INIT_SIMPLE = 1.0;
  
  // Start timer
  t_init = CPU_time_ms;
  
  // Set target order iterator
  iorder = 0;
  SetTarget();

  // Get first waypoint
  GetWaypointPhase2();

  // Compute heading (waypoint tangent line)
  float dx = xwp - x_target;
  float dy = ywp - y_target;
  Hwp = WrapAngle(atan2f(dy,dx) + PI/2);
}

// Capture waypoints, take snapshots every 3 seconds
static void Phase2() {  
  // Check to see if snapshot is available
  float dt = (CPU_time_ms - t_init)/1000;
  if (dt > 3.0) {
    // Take a snapshot
    snapshot mySnapShot = takeASnapshot();

    // Post process results and go to next target if current target is refined
    if (mySnapShot.pictureTaken == 1 && mySnapShot.personsInPicture[iTarget] == 1) {
      // Collect snapshot data
      uint16_t isnap = n_snaps[iTarget];
      G_inc[iTarget][0][isnap] = mySnapShot.centerOfPictureX;
      G_inc[iTarget][1][isnap] = mySnapShot.centerOfPictureY;
      G_inc[iTarget][2][isnap] = mySnapShot.diameterOfPicture;
      G_inc[iTarget][3][isnap] = mySnapShot.centerOfPersonEstimateX[iTarget];
      G_inc[iTarget][4][isnap] = mySnapShot.centerOfPersonEstimateY[iTarget];
      
      // Reinitialize time
      t_init = CPU_time_ms;
      
      // Add to snapshot counter
      n_snaps[iTarget]++;
      
      // Continue to next target if all snapshots complete
      if (n_snaps[iTarget] >= nG) {
        iorder++;
        SetTarget();
      }
    }
  }
  
  // Check to see if waypoint is found
  float dx = xwp - X_position;
  float dy = ywp - Y_position;
  pos_error = sqrtf(dx*dx + dy*dy);
  if (pos_error <= SNAPSHOT_ERROR) {
    // Get new waypoint
    GetWaypointPhase2();

    // Compute heading (waypoint tangent line)
    float dx = xwp - x_target;
    float dy = ywp - y_target;
    Hwp = WrapAngle(atan2f(dy,dx) + PI/2);
  }
}
#endif /* MISSION_PLAN_H */


