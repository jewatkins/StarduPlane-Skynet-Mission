#ifndef ISINSIDECONVEXPOLYGON_H
#define ISINSIDECONVEXPOLYGON_H

#include <math.h>
#include "AA241X_sortPoly.h"

uint8_t isInsideConvexPolygon(uint8_t n_poly, float poly[][2], float p[2]){

	uint8_t i, flag;
	float angles[n_poly], adj_ang[n_poly];
   
    // Sort polygon in counter-clockwise direction	
	sortPoly(n_poly, poly);

    // Shift origin to p and calculate polar angles 
	for(i = 0; i < n_poly; i++){
		angles[i] = atan2(poly[i][1]-p[1] , poly[i][0]-p[0]);
    }

    // Calculate angles between adjacent points
    for(i = 0; i < n_poly; i++){
		if ( i == n_poly-1){
            adj_ang[i] = angles[0] - angles[i];
        }
		else {
            adj_ang[i] = angles[i+1] - angles[i];
        }
        if (adj_ang[i] < 0) adj_ang[i] += 2*PI;
	}
             
    // Check for interior/exterior location
    flag = 1;
    for (i = 0; i<n_poly; i++){
        if (adj_ang[i] < 0 || adj_ang[i] > PI) {
            flag = 0;
            break;
        }
    }
    
    return flag;

}

#endif
