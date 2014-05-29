#ifndef WAYPOINT_NAVIGATION_H
#define WAYPOINT_NAVIGATION_H

#include <math.h>

/**** Waypoint Navigation ****/
#define TIME_ESTIMATE 3.2f                     // Time estimate between waypoints (s)
#define Np 4                                   // Number of persons to find
static char persons_found[Np] = {0,0,0,0};     // Number of persons found

// Wrap angle to interval [0,2*pi]
static float WrapAngle(float angle) {
  float twoPi = 2*PI;
  return angle - twoPi*(float)floor(angle/twoPi);
}

/*
// Static Waypoint path
#define Nwp 54
static float waypoints[Nwp][2] = {
  {-32.6961,  136.3175},
  {-63.5886,  124.9319},
  {-90.9736,  106.6551},
 {-113.3404,   82.4951},
 {-129.4554,   53.7847},
 {-138.4296,   22.1075},
 {-139.7680,  -10.7891},
 {-133.3968,  -43.0906},
 {-119.6673,  -73.0153},
  {-99.3370,  -98.9123},
  {-73.5273, -119.3534},
  {-43.6618, -133.2109},
  {-11.3878, -139.7205},
   {21.5142, -138.5230},
   {53.2296, -129.6847},
   {82.0088, -113.6928},
  {106.2643,  -91.4297},
  {124.6584,  -64.1233},
  {136.1762,  -33.2798},
  {140.1825,   -0.6006},
  {136.4563,   32.1117},
  {125.2032,   63.0528},
  {107.0439,   90.5158},
   {82.9799,  112.9860},
   {54.3388,  129.2238},
   {22.7004,  138.3336},
  {-10.1902,  139.8129},
  {-34.8138,  109.3737},
  {-46.2253,   76.4955},
  {-70.7165,   54.6582},
  {-85.6763,   25.4540},
  {-89.0886,   -7.1810},
  {-80.4932,  -38.8481},
  {-61.0489,  -65.2792},
  {-33.3762,  -82.9118},
   {-1.2051,  -89.3694},
   {31.1285,  -83.7816},
   {59.2666,  -66.9015},
   {79.4165,  -41.0043},
   {88.8625,   -9.5805},
   {86.3315,   23.1346},
   {72.1645,   52.7316},
   {48.2710,   75.2213},
   {17.8715,   87.5725},
  {-10.7679,   63.5260},
  {-20.2042,   33.9262},
  {-38.7156,    7.7652},
  {-31.7244,  -23.5109},
  { -3.8356,  -39.3000},
  { 26.5797,  -29.2014},
  { 39.4865,    0.1327},
  { 26.3828,   29.3794},
  { -4.0998,   39.2733},
  { -7.7101,    5.6341}
};
*/

// Returns waypoint number 'wp' for phase-1 assuming that the airspeed is
// maintained at 11 m/s and there are 56 waypoints
#define Nwp 54
static uint8_t init_flag = 1;
static uint8_t trans_flag = 0;
static float t_init = 0.0f;                              // Initial time between waypoints
static uint16_t iwp = 0;                                 // Waypoint iterator
static float x_init = 0.0f; static float y_init = 0.0f;  // Initial start position
static float xwp = 0.0f;    static float ywp = 0.0f;     // Waypoint
static float Hwp = 0.0f;                                 // Heading: waypoint tangent line
static float pos_error = 0.0f;							 // Position Error
static void GetWaypoint() {
  // Set parameters
  uint16_t wp = iwp + (uint16_t)1;
  trans_flag = 0;
  float ts = 3.0;
  float v = 11.0;
  float RC[4] = {140.1838f, 89.3775f, 39.4867f, 9.5493f};
  float tau[4] = {0.1500f, 0.2667f, 0.3833f, 0.5000f};
  float theta = 1.6414f;
  float offset = 1.6546f;
  float rotation = WrapAngle(atan2f(y_init,x_init)) - theta;
  
  // Circle 1
  float omega = v/RC[0];
  float T = (2*PI - tau[0] - theta + offset) / omega;
  uint16_t nTp = (uint16_t)ceilf(T/ts);
  float ang = 0.0;
  if (wp <= nTp) {
      ang = theta + omega*ts*(float)wp;
      ang += rotation;
      xwp = RC[0]*cosf(ang);
      ywp = RC[0]*sinf(ang);
      return;
  }
  ang = theta + omega*ts*(float)nTp;
  theta = WrapAngle(ang);
  uint16_t nTm = nTp;
  
  uint16_t iC = 0;
  for (iC=1; iC<4; iC++) {
    // Transition from circle 1 to circle 2
    omega = v/RC[iC-1];
    T = (2*tau[iC-1]) / omega;
    nTp = nTm + (uint16_t)ceilf(T/ts);
    float rad = 0.0;
    if (wp <= nTp) {
		trans_flag = 1;
        rad = (RC[iC-1] + (RC[iC]-RC[iC-1])*(float)(wp-nTm)/ (float)(nTp-nTm)); // Probably never zero
        ang = theta + omega*ts*(float)(wp-nTm);
        ang += rotation;
        xwp = rad*cosf(ang);
        ywp = rad*sinf(ang);
        return;
    }
    ang = theta + omega*ts*(float)(nTp-nTm);
    theta = WrapAngle(ang);
    nTm = nTp;
    
    // Circle 2
    omega = v/RC[iC];
    T = (2*PI - theta - tau[iC] + offset) / omega;
    nTp = nTm + (uint16_t)ceilf(T/ts);
    if (wp <= nTp) {
        rad = RC[iC];
        ang = theta + omega*ts*(float)(wp-nTm);
        ang += rotation;
        xwp = rad*cosf(ang);
        ywp = rad*sinf(ang);
        return;
    }
    ang = theta + omega*ts*(float)(nTp-nTm);
    theta = WrapAngle(ang);
    nTm = nTp;
  }
}


#endif /* WAYPOINT_NAVIGATION_H */
