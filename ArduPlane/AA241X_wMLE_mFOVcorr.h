#ifndef WMLEMFOVCORR_H
#define WMLEMFOVCORR_H

#include "AA241X_isInsideConvexPolygon.h"
#include "AA241X_intWithPolygon.h"

// Evaluates the weighted empirical mean with multi-point FOV correction
void wMLE_mFOVcorr(uint8_t nyR, float yR[][2], uint8_t nGl, float cam[][2], float G[][3], float yT[2])
/* ---
nyR - no. of vertices of region polygon
yR  - vertices of intersection polygon
nGl  - no. of snaps
cam - (y1,y2) - estimate location
G   - (yc1,yc2,R) - FOV center and radius
*/
{

    uint8_t cnt, flag;
    float p[2], yT_corr[2], area_inv, inv_Rsq;

    // Initialize
    area_inv = 0;
    yT[0] = 0; yT[1] = 0;

    for (cnt=0; cnt<nGl; cnt++) {

        // Check if target estimates are outside the intersection of FOVs
        p[0] = cam[cnt][0];
        p[1] = cam[cnt][1];
        flag = isInsideConvexPolygon(nyR, yR, p);

        // Correct target estimates if outside intersection of FOVs
        if (flag == 0) {
            intWithPolygon(nyR, yR, p, yT_corr);
        }
        else {
            yT_corr[0] = cam[cnt][0];
            yT_corr[1] = cam[cnt][1];
        }
        
        // Evaluate the weighted empirical mean
        inv_Rsq = (1.0/(G[cnt][2]*G[cnt][2]));
        yT[0] += yT_corr[0] * inv_Rsq;
        yT[1] += yT_corr[1] * inv_Rsq;
        area_inv += inv_Rsq;
    }
    yT[0] = yT[0] / area_inv;
    yT[1] = yT[1] / area_inv;

}

#endif
