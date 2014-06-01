#ifndef REGIONINTCIRC_H
#define REGIONINTCIRC_H

#include "AA241X_normL2.h"
#include "AA241X_IntTwoCirc.h"
#include "AA241X_IntTwoCircAux.h"

// Evaluates the region of intersection of circles
void regionIntCirc(uint8_t nC, float Circ[][3], float y[][2], uint8_t *ny)
/*
nC   - uint8_t         - number of circles
Circ - (nCx3) real - center (yc1,yc2) and radius R of nC circles
y    - (nyx2) real - coordinates of points on the periphery of intersection
ny   - uint8_t         - number of points on the periphery of intersection
*/

{

    uint8_t i, j, n0, n1, n;
    float C0[3], C1[3];

    // Check if only one circle is provided
    if (nC == 1)
    {
        *ny = 1;
        y[0][0] = Circ[0][0];
        y[0][1] = Circ[0][1];
        return;
    }
    
    // Check if only two circles are provided
    else if (nC == 2)
    {
        float aux[2][2];
        for (i=0; i<3; i++)
        {
            C0[i] = Circ[0][i];
            C1[i] = Circ[1][i];
        }
        IntTwoCircAux(C0, C1, y, aux);
        
        *ny = 4;
        for (i=0; i<2; i++)
        {
            y[2+i][0] = aux[i][0];
            y[2+i][1] = aux[i][1];
        }
        return;
    } 
        
    // Else return the discrete descriptor of the region
    else
    {
        // Evaluate all intersection points (2 * nC2 pts.)
        float y_IntAll[2][2][nC][nC];
        float yTwo[2][2];       
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
                    IntTwoCirc(C0, C1, yTwo);
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
                if (n1 > n0)
                {
                    for (i=0; i<2; i++)
                    {
                        // Check for interior pts.
                        flag = 1;
                        for (n=0; n<nC; n++)
                        {
                            diff[0] = y_IntAll[i][0][n0][n1] - Circ[n][0];
                            diff[1] = y_IntAll[i][1][n0][n1] - Circ[n][1];
                            if ( normL2(2,diff) > Circ[n][2] + 1e-5)
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
}

#endif
