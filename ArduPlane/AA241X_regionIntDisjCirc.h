#ifndef REGIONINTDISJCIRC_H
#define REGIONINTDISJCIRC_H

#include "AA241X_normL2.h"

// Evaluates the region of intersection of circles
void regionIntDisjCirc(uint8_t nCInc, uint8_t nCExc, float CircInc[][3], float CircExc[][3], float y[][2], uint8_t *ny)
/*
nCInc   - uint8_t            - number of included circles
nCExc   - uint8_t            - number of excluded circles
CircInc - (nCIncx3) real - center (yc1,yc2) and radius R of nC circles that are to be included
CircExc - (nCExcx3) real - center (yc1,yc2) and radius R of nC circles that are to be excluded
y       - (nyx2) real    - coordinates of points on the periphery of intersection
ny      - uint8_t            - number of points on the periphery of intersection
*/

{

    uint8_t i, j, n0, n1, n;
    float C0[3], C1[3];

    // Concatenate all circles
    uint8_t nC = nCInc + nCExc;
    float Circ[nC][3];
    for (n=0; n<nCInc; n++)
    {
        for (i=0; i<3; i++)
        {
            Circ[n][i] = CircInc[n][i];
        }
    }
    for (n=0; n<nCExc; n++)
    {
        for (i=0; i<3; i++)
        {
            Circ[nCInc+n][i] = CircExc[n][i];
        }
    }
   
    // Evaluate all intersection points (2 * nC2 pts.)
    float y_IntAll[2][2][nC][nC];
    float yTwo[2][2];
    uint8_t flagInt[nC][nC];
    
    for (n0=0; n0<nC; n0++)
    {
        for (n1=0; n1<nC; n1++)
        {
            if (n1>n0) 
            {
                for (i=0; i<3; i++)
                {
                    C0[i] = Circ[n0][i];
                    C1[i] = Circ[n1][i];
                }
                
                flagInt[n0][n1] = IntTwoCirc(C0, C1, yTwo);
                if (flagInt[n0][n1] == 0) continue;
                
                for (i=0; i<2; i++)
                {
                    for (j=0; j<2; j++)
                    {
                        y_IntAll[i][j][n0][n1] = yTwo[i][j];
                    }
                }
            }
        }
    }
    
    // Test for pts. on the periphery of the intersection
    float y_Int[nC][2], diff[2];
    uint8_t cnt = 0, flag;
    
    for (n0=0; n0<nC; n0++)
    {
        for (n1=0; n1<nC; n1++)
        {
            if (n1 > n0 && flagInt[n0][n1]==1)
            {
                for (i=0; i<2; i++)
                {
                    // Check for interior pts.
                    flag = 1;
                    for (n=0; n<nCInc; n++)
                    {
                        diff[0] = y_IntAll[i][0][n0][n1] - Circ[n][0];
                        diff[1] = y_IntAll[i][1][n0][n1] - Circ[n][1];
                        if ( normL2(2,diff) > Circ[n][2] + 1e-5)
                        {
                            flag = 0;
                            break;
                        }
                    }
                    // Check for exterior pts.
                    for (n=nCInc; n<nC; n++)
                    {
                        diff[0] = y_IntAll[i][0][n0][n1] - Circ[n][0];
                        diff[1] = y_IntAll[i][1][n0][n1] - Circ[n][1];
                        if ( normL2(2,diff) < Circ[n][2] - 1e-5)
                        {
                            flag = 0;
                            break;
                        }
                    }

                    if (flag == 1)
                    {
                        y_Int[cnt][0] = y_IntAll[i][0][n0][n1];
                        y_Int[cnt][1] = y_IntAll[i][1][n0][n1];
                        cnt++;
                    }                    
                }
            }
        }
    }
           
    // Remove degenerate points
    uint8_t logic[cnt];
    for (i=0; i<cnt; i++)
    {
        logic[i] = 1;
    }
    for (j=0; j<cnt-1; j++)
    {
        for (i=j+1; i<cnt; i++)
        {
            diff[0] = y_Int[i][0] - y_Int[j][0]; 
            diff[1] = y_Int[i][1] - y_Int[j][1]; 
            if (normL2(2,diff) <= 1e-5)
            {
                logic[j] = 0;
                break;
            }
        }
    }
    *ny = 0;
    for (i=0; i<cnt; i++)
    {
        if (logic[i] == 1)
        {
            y[*ny][0] = y_Int[i][0];
            y[*ny][1] = y_Int[i][1];
            *ny = *ny + 1;
        }
    }
    

}

#endif
