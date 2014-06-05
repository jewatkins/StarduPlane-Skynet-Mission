#ifndef INTWITHPOLYGON_H
#define INTWITHPOLYGON_H

#include <math.h>
#include "AA241X_sortPoly.h"

void intWithPolygon(uint8_t n_poly, float poly[][2], float p[2], float yInt[2]){


	float c[2], angles[n_poly], yAngle;
	float x1, x2, y1, y2, den;
	uint8_t i;
	int ind1 = 0;
	int ind2 = 0;

	// Find centroid of the polygon
	c[0] = 0;
	c[1] = 0;
	for(i = 0; i < n_poly; i++){
		c[0] += poly[i][0];
		c[1] += poly[i][1];
	}
	c[0] = c[0]/n_poly;
	c[1] = c[1]/n_poly;
	
    // The polygon is already sorted due to the call to isInsideConvexPolygon
    // Shift the origin to the centroid and find the polar angles
	for(i = 0; i < n_poly; i++){
		angles[i] = atan2(poly[i][1]-c[1], poly[i][0]-c[0]);
	}

    // Locate the edge of intersection
	yAngle = atan2(p[1]-c[1], p[0]-c[0]);
	ind1 = -1;
	for(i = 0; i < n_poly-1; i++){
		if (angles[i] <= yAngle && angles[i+1] > yAngle) {
			ind1 = i;
			ind2 = i+1;
	    }
	}
	if (ind1 == -1){
		ind1 = n_poly-1;
		ind2 = 0;
	}

    // Find the intersection pt.
	x1 = poly[ind1][0];
	y1 = poly[ind1][1];
	x2 = poly[ind2][0];
	y2 = poly[ind2][1];
	den = (p[0]-c[0])*(y1-y2)+(x1-x2)*(c[1]-p[1]);
	yInt[0] = (x1*(p[0]*(c[1]-y2)+c[0]*(y2-p[1]))-x2*(p[0]*(c[1]-y1)+c[0]*(y1-p[1])))/den;
	yInt[1] = (y1*(p[1]*(x2-c[0])+c[1]*(p[0]-x2))+y2*(p[1]*(c[0]-x1)+c[1]*(x1-p[0])))/den;
}

#endif
