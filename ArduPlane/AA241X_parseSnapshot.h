#ifndef PARSESNAPSHOT_H
#define PARSESNAPSHOT_H

#include <math.h>
#include "AA241X_MissionPlan_Parameters.h"
#include "AA241X_refineTarget.h"


// Returns 1 to move to next target. Else 0.
static uint8_t parseSnapshot(snapshot mySnapShot) {

  if (mySnapShot.pictureTaken == 1) {

    // Reinitialize time if picture was successful
    t_init = CPU_time_ms;

    // Post process results
    if (mySnapShot.personsInPicture[iTarget] == 1) {
      // Collect Inc snapshot data
      Inc[n_Inc][0] = mySnapShot.centerOfPictureX;
      Inc[n_Inc][1] = mySnapShot.centerOfPictureY;
      Inc[n_Inc][2] = 0.5 * mySnapShot.diameterOfPicture;

      cam_est[n_Inc][0] = mySnapShot.centerOfPersonEstimateX[iTarget];
      cam_est[n_Inc][1] = mySnapShot.centerOfPersonEstimateY[iTarget];
      
      // Add to snapshot counter
      n_Inc++;

      // Re-compute the centroid of target region and update target estimate
      if( refineTarget(1, 0) || n_Inc == n_Inc_lim - 1 ) return 1;
      else return 0;
      
    }
    else if (mySnapShot.personsInPicture[iTarget] == 0 && n_Exc_lim > 0 && n_Exc < n_Exc_lim) {
      // Collect Exc snapshot data
      Exc[n_Exc][0] = mySnapShot.centerOfPictureX;
      Exc[n_Exc][1] = mySnapShot.centerOfPictureY;
      Exc[n_Exc][2] = 0.5 * mySnapShot.diameterOfPicture;
      
      // Add to snapshot counter
      n_Exc++;

      // Re-compute the centroid of target region and update target estimate
      if( refineTarget(0, 0)) return 1;
      else return 0;

    }

  }
  return 0;

}

#endif