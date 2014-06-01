#ifndef POLYNORM_H
#define POLYNORM_H

#include "AA241X_normL2.h"

// Evaluates the average distance between any two points on the periphery of a polygon
float polyNorm(uint8_t ny, float y[][2])
{
    uint8_t i, j;
    float diff[2], mean;
    
    if (ny == 1) return 0;
    
    mean = 0;
    for (i=0; i<ny-1; i++) {
        for (j=i+1; j<ny; j++) {
            diff[0] = y[i][0] - y[j][0];
            diff[1] = y[i][1] - y[j][1];
            mean += normL2(2, diff);
        }
    }
    mean = mean / (ny*(ny-1)/2);

    return mean;

}
#endif
