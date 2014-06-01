#ifndef INITREFINE_H
#define INITREFINE_H

#include "AA241X_MissionPlan_Parameters.h"
#include "AA241X_regionIntCirc.h"

void initRefine()
/*
iTarget - index of iTarget to refine
entry_wp - entry waypoint
*/
{
    uint8_t i, ny;
    float y_region[n_region_lim][2];
    
    // Reset counter for Inc
    n_Inc = n_snaps[iTarget];

    // Reset counter for Exc
    n_Exc = 0;
    
    // Fill in Inc
    for (i=0; i<n_snaps[iTarget]; i++) {
        cam_est[i][0] = G_inc[iTarget][3][i];
        cam_est[i][1] = G_inc[iTarget][4][i];
        Inc[i][0] = G_inc[iTarget][0][i];       
        Inc[i][1] = G_inc[iTarget][1][i];
        Inc[i][2] = G_inc[iTarget][2][i];
    }
    
    // Evaluate FOV region
    regionIntCirc(n_Inc, Inc, y_region, &ny);

    // Calculate centroid
    y_centroid[0] = 0;
    y_centroid[1] = 0;
    for(i=0; i<ny; i++){
	    y_centroid[0] += y_region[i][0];
	    y_centroid[1] += y_region[i][1];
    }
    y_centroid[0] = y_centroid[0]/ny;
    y_centroid[1] = y_centroid[1]/ny;
    
}

#endif
