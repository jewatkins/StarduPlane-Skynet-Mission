#ifndef REGIONINTDISJCIRC_H
#define REGIONINTDISJCIRC_H

#include "AA241X_normL2.h"

// Evaluates the region of uint8_tersection of circles
void regionIntDisjCirc(uint8_t nCInc, uint8_t nCExc, float CircInc[][3], float CircExc[][3], float y[][2], uint8_t *ny)
/*
nCInc   - uint8_t            - number of included circles
nCExc   - uint8_t            - number of excluded circles
CircInc - (nCIncx3) real - center (yc1,yc2) and radius R of nC circles that are to be included
CircExc - (nCExcx3) real - center (yc1,yc2) and radius R of nC circles that are to be excluded
y       - (nyx2) real    - coordinates of pouint8_ts on the periphery of uint8_tersection
ny      - uint8_t            - number of pouint8_ts on the periphery of uint8_tersection
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
   
    // Evaluate all uint8_tersection pouint8_ts (2 * nC2 pts.)
    float yTwo[2][2];
    uint8_t flagInt[nC][nC];
    float y_Int[nC][2], diff[2];
    uint8_t cnt = 0, flag;
    
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
                
                    // Check for uint8_terior pts.
                    flag = 1;
                    for (n=0; n<nCInc; n++)
                    {
                        diff[0] = yTwo[i][0] - Circ[n][0];
                        diff[1] = yTwo[i][1] - Circ[n][1];
                        if ( normL2(2,diff) > Circ[n][2] + 1e-5)
                        {
                            flag = 0;
                            break;
                        }
                    }
                    // Check for exterior pts.
                    for (n=nCInc; n<nC; n++)
                    {
                        diff[0] = yTwo[i][0] - Circ[n][0];
                        diff[1] = yTwo[i][1] - Circ[n][1];
                        if ( normL2(2,diff) < Circ[n][2] - 1e-5)
                        {
                            flag = 0;
                            break;
                        }
                    }

                    if (flag == 1)
                    {
                        y_Int[cnt][0] = yTwo[i][0];
                        y_Int[cnt][1] = yTwo[i][1];
                        cnt++;
                    }
                }
            }
        }
    }
    
    // Remove degenerate pouint8_ts
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
