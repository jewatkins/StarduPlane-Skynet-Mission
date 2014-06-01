#ifndef SORTPOLY_H
#define SORTPOLY_H

#include <math.h>

void sortPoly(uint8_t n_poly, float poly[][2]){

	uint8_t i,j;
    float centroid[2], angles[n_poly], swp;

	// Find centroid of the polygon
	centroid[0] = 0;
	centroid[1] = 0;
	for(i = 0; i < n_poly; i++){
		centroid[0] += poly[i][0];
		centroid[1] += poly[i][1];
	}
	centroid[0] = centroid[0]/n_poly;
	centroid[1] = centroid[1]/n_poly;

    // Shift the origin to the centroid and find the polar angles
	for(i = 0; i < n_poly; i++){
		angles[i] = atan2(poly[i][1]-centroid[1] , poly[i][0]-centroid[0]);
	}

    // Sort the polygon
	for (i=0;i<n_poly-1;i++){
		for (j=i+1;j<n_poly;j++){
			if (angles[j] < angles[i]){

				swp = angles[i];
				angles[i] = angles[j];
				angles[j] = swp;

                swp = poly[i][0];
				poly[i][0] = poly[j][0];
				poly[j][0] = swp;
								
                swp = poly[i][1];
				poly[i][1] = poly[j][1];
				poly[j][1] = swp;
			}
		}
	}
}

#endif
