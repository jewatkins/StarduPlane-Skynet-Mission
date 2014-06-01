#ifndef INTTWOCIRCAUX_H
#define INTTWOCIRCAUX_H

#include <math.h>

// Function for finding the intersection between circles and appending
// auxiliary points 
// - returns 0 if no points of intersection are found
uint8_t IntTwoCircAux(float C0[3], float C1[3], float y[2][2], float aux[2][2])
{
    // Evaluate inter-center distance
    float d = sqrt( powf((C0[0] - C1[0]),2) + powf((C0[1] - C1[1]),2) );

    // Check against repeated circles
    if (d<1e-10) d = 1e-10;

    // Compute intersection points in local coordinates
    float R = C0[2];
    float r = C1[2];
    
    float x, yp, yn;       
    
    // Check against inter-containment and non-intersection
    if ( 4*powf(d*R,2) > powf(d*d - r*r + R*R,2) )
    {
        x = (d*d - r*r + R*R) / (2*d);
        yp = sqrt((4*powf(d*R,2) - powf(d*d - r*r + R*R,2)) / (4*d*d));
        yn = -yp;
    }
    else if (d < R+r)
    {
        if (R>r)
        {
            x = d;
            yp = r; yn = -r;        
        }
        else
        {
            x = 0;
            yp = R; yn = -R;
        }
    }
    else 
    {
        return 0;
    }

    // Rotate to physical coordinates
    float theta = atan2( (C1[1]-C0[1]), (C1[0]-C0[0]) );
    float rotMat[2][2]; 
    rotMat[0][0] = cos(theta);
    rotMat[0][1] = -sin(theta);
    rotMat[1][0] = sin(theta);
    rotMat[1][1] = cos(theta);
    
    y[0][0] = C0[0] + (rotMat[0][0]*x + rotMat[0][1]*yp);
    y[0][1] = C0[1] + (rotMat[1][0]*x + rotMat[1][1]*yp);
    y[1][0] = C0[0] + (rotMat[0][0]*x + rotMat[0][1]*yn);
    y[1][1] = C0[1] + (rotMat[1][0]*x + rotMat[1][1]*yn);
    
    aux[0][0] = C0[0] + rotMat[0][0]*R;
    aux[0][1] = C0[1] + rotMat[1][0]*R;
    aux[1][0] = C0[0] + rotMat[0][0]*(d-r);
    aux[1][1] = C0[1] + rotMat[1][0]*(d-r);
    
    return 1;
}

#endif
