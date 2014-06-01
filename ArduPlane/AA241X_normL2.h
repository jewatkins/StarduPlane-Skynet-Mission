#ifndef NORML2_H
#define NORML2_H

#include <math.h>

// Evaluates the l2 norm of a given vector
float normL2(uint8_t len, float vec[])
/*
len - uint8_t - lenth of vector vec
vec - lenx1 - real valued vector
*/
{
    uint8_t i;
    float sum = 0;
    
    for (i=0; i<len; i++)
    {
        sum = sum + vec[i]*vec[i];
    }
    
    return (sqrt(sum));
}

#endif
