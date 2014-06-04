#ifndef REFINETARGET_H
#define REFINETARGET_H

#include "AA241X_aux.h"
#include "AA241X_MissionPlan_Parameters.h"
#include "AA241X_polyNorm.h"
#include "AA241X_wMLE_mFOVcorr.h"
#include "AA241X_regionIntDisjCirc.h"
#include "AA241X_regionIntCirc.h"

// Shifting-circle geometric refinement 
// This function is called every time a new snapshot is found
uint8_t refineTarget(uint8_t snap_stat, uint8_t target_comp)
/*
Inputs:
snap_stat   - 1 if successful, 0 if unsuccessful
target_comp - 1 to compute yT for every succesful snap, 0 for only at convergence

Outputs:
wp          - next waypoint location
yT          - current estimate of target

Return:       1 post convergence, 0 otherwise 
*/

{

    uint8_t i, ny;
    float y_region[n_region_lim][2], yT[2];
   
    // Initialize solution
    uint8_t converged = 0;

    // 1. Change level if intersection is found
    // ---
    // Recalculate region
    if (n_Exc > 0) regionIntDisjCirc(n_Inc, n_Exc, Inc, Exc, y_region, &ny);
    else regionIntCirc(n_Inc, Inc, y_region, &ny);

    // Recalculate trajectory and estimate if needed
    if (snap_stat == 1) {
        // Recalculate centroid
	    y_centroid[0] = 0;
	    y_centroid[1] = 0;
	    for(i=0; i<ny; i++){
		    y_centroid[0] += y_region[i][0];
		    y_centroid[1] += y_region[i][1];
	    }
	    y_centroid[0] = y_centroid[0]/ny;
	    y_centroid[1] = y_centroid[1]/ny;
        
        // Refine estimate
        if (target_comp == 1) {
			wMLE_mFOVcorr(ny, y_region, n_Inc, cam_est, Inc, yT);
			X_person_estimate[iTarget] = yT[0];
            Y_person_estimate[iTarget] = yT[1];
			/* X_person_estimate[iTarget] = y_centroid[0];
            Y_person_estimate[iTarget] = y_centroid[1]; */
		}
        
    }
    // ---


    // 2. Check for convergence
    // ---
    // Evaluate norm of intersection region
    if (n_Inc > 2 && polyNorm(ny, y_region) < beta) converged = 1;

    // Re-evaluate target at convergence
    if (converged == 1 || n_Inc == n_Inc_lim-1) {
        wMLE_mFOVcorr(ny, y_region, n_Inc, cam_est, Inc, yT);
        X_person_estimate[iTarget] = yT[0];
        Y_person_estimate[iTarget] = yT[1];
		/* X_person_estimate[iTarget] = y_centroid[0];
        Y_person_estimate[iTarget] = y_centroid[1]; */
    }
    //
    if(converged)
		return 1;
	else
		return 0;
}

#endif
