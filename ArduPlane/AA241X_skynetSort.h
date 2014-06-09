//
//  skynetSort.h
//

#ifndef SKYNETSORT_H
#define SKYNETSORT_H

#include <math.h>
#include "AA241X_regionIntCirc.h"
#include "AA241X_sortPoly.h"
#include "AA241X_MissionPlan_Parameters.h"

#define fB 30.0
#define fT 60.0
#define hB 30.48
#define hT 121.92

#define alpha 200.0
#define gamma 1.0

#define t_refine 35.31
#define dy_refine 1.0981

// Evaluates the FOV radius given a position
float evaluateFOV(float x[3]){

    float f;
    
    return f = fB + (fT-fB)*(x[2]-hB)/(hT-hB);
}

// Evaluates the area of a triangle given the coordinates
float area_triangle(float x1[2], float x2[2], float x3[2]){
    
    float A = x1[0]*(x2[1]-x3[1]) + x2[0]*(x3[1]-x1[1]) + x3[0]*(x1[1]-x2[1]);
    
    if (A < 0)
        return -0.5*A;
    return 0.5*A;
}

// Evaluates the area of a convex polygon given its coordinates
float polyArea(uint8_t n_poly, float poly[][2]){
    
    sortPoly(n_poly, poly);
    
    float x1[2] = {poly[0][0], poly[0][1]};
    
    float A, x2[2], x3[2];
    
    uint8_t i;
    
    A = 0;
    for (i = 1; i < n_poly-1; i++) {
        
        x2[0] = poly[i][0];
        x2[1] = poly[i][1];
        x3[0] = poly[i+1][0];
        x3[1] = poly[i+1][1];
        
        A += area_triangle(x1,x2,x3);
    }
    
    return A;
    
}

// Find the order in which to visit the targets
// G_inc[Ntargets][Ndim][nG] -> data structure to hold all the snapshot data
// Ntargets -> Number of targets
// Ndim     -> 5. 0 and 1 are x and y location returned by camera.
//             2 and 3 are the x and y locations of the center of FoV
//             4 is the FoV radius.
// nG       -> maximum number of snapshots allowed per target in phase 1

void skynetSort(float x_uav[3], float v, float t_rem, uint8_t order[4]){

/*    
    float R_c, y_c[4][2], dy0[4], yR[nG][2], Circ[nG][3], a_r;
    uint8_t nyR, i,j;
    
    R_c = evaluateFOV(x_uav);
    
    for (j=0; j<4; j++) {
        
        for (i=0; i<n_snaps[j]; i++) {
            Circ[i][0] = G_inc[j][0][i];
            Circ[i][1] = G_inc[j][1][i];
            Circ[i][2] = G_inc[j][2][i];
        }
        
        
        regionIntCirc(n_snaps[j], Circ, yR, &nyR);
        
        // Find centroid of the polygon
        y_c[j][0] = 0;
        y_c[j][1] = 0;
        for(i = 0; i < nyR; i++){
            y_c[j][0] += yR[i][0];
            y_c[j][1] += yR[i][1];
        }
        y_c[j][0] = y_c[j][0]/nyR;
        y_c[j][1] = y_c[j][1]/nyR;
        
        if (nyR == 1) {
            a_r = PI * ( G_inc[j][2][0] * G_inc[j][2][0] );
        }
        else {
            a_r = polyArea(nyR, yR);
        }
        dy0[j] = sqrt(a_r / PI);
    }
    
    // Define permutation matrix
    uint8_t p[6][4] = {{0,3,2,1},
        {0,3,1,2},
        {0,2,3,1},
        {0,2,1,3},
        {0,1,2,3},
        {0,1,3,2}};
    
    uint8_t np = 6;

    for (i=0; i<6; i++){
    	for (j=0; j<4; j++){
    		p[i][j] = order[ p[i][j] ];
    	}
    }
    
    float l[6][4], x_tmp[2];
    
    for (i = 0; i<np; i++) {
        for (j=0; j<4; j++) {
            if (j == 0) {
                x_tmp[0] =  y_c[p[i][j]][0] - x_uav[0];
                x_tmp[1] =  y_c[p[i][j]][1] - x_uav[1];
                l[i][j] = normL2(2,x_tmp);
            }
            else
            {
                x_tmp[0] =  y_c[p[i][j]][0] - y_c[p[i][j-1]][0];
                x_tmp[1] =  y_c[p[i][j]][1] - y_c[p[i][j-1]][1];
                l[i][j] = normL2(2,x_tmp);
            }
        }
    }
    
    float score[6], time[6];
    
    float dy[4], den;
    
    for (i=0; i<np; i++) {
        
        time[i] = 0;
        
        for (j=0; j<4; j++) {
            
            dy[p[i][j]] = dy0[p[i][j]];
            time[i] += l[i][j]/v + t_refine;
            
            if (time[i] <= t_rem) {
                dy[ p[i][j] ] = dy_refine;
            }
            else {
                time[i] = t_rem;
                break;
            }
        }
        
        den = 0;
        for (j=0; j<4; j++) {
            if (gamma > dy[p[i][j]])
                den += gamma;
            else
                den += dy[p[i][j]];
        }
        score[i] = alpha / den;
    }
    
    float Escore, Etime;
    uint8_t i_opt;
    
    Escore = 0;
    for (i=0; i<np; i++) {
        if (score[i] > Escore) {
            Escore = score[i];
            i_opt = i;
        }
    }
    
    Etime = t_rem;
    for (i=0; i<np; i++) {
        if (score[i] == Escore) {
            if (Etime > time[i]) {
                Etime = time[i];
                i_opt = i;
            }
        }
    }

    order[0] = p[i_opt][0];
    order[1] = p[i_opt][1];
    order[2] = p[i_opt][2];
    order[3] = p[i_opt][3];
    */

    // Define permutation matrix
    uint8_t p[6][4] = {{0,3,2,1},
        				{0,3,1,2},
        				{0,2,3,1},
        				{0,2,1,3},
        				{0,1,2,3},
        				{0,1,3,2}};
    
    uint8_t np = 6;
    uint8_t i, j;

    for (i=0; i<6; i++){
    	for (j=0; j<4; j++){
    		p[i][j] = order[ p[i][j] ];
    	}
    }
    
    float l[6], min_l;
    uint8_t i_opt, ind1, ind2;

    min_l = 2000;
    for (i=0; i<6; i++){
    	l[i] = 0;
    	for (j=0; j<3; j++){
    		ind1 = p[i][j+1];
    		ind2 = p[i][j];
    		l[i] += hypotf( G_inc[ind2][0][0] - G_inc[ind1][0][0],  G_inc[ind2][1][0] - G_inc[ind1][1][0] ) ;
    	}
    	if (l[i] <= min_l){
    		min_l = l[i];
    		i_opt = i;
    	}
    }

    order[0] = p[i_opt][0];
    order[1] = p[i_opt][1];
    order[2] = p[i_opt][2];
    order[3] = p[i_opt][3];

}

#endif
