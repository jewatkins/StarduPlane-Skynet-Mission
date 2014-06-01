#ifndef MISSION_PLAN_PARAMETERS_H
#define MISSION_PLAN_PARAMETERS_H

/**** Mission Plan Parameters ****/
static uint8_t phase_flag = 1;

/**** Phase-1 Parameters ****/
#define Nwp 52                  // Number of waypoints in phase-1 spiral
#define TIME_ESTIMATE 3.5f      // Time estimate between waypoints (s)
static float t_init = 0.0f;     // Initial time between waypoints
static float x_init = 0.0f;     // Initial x-position of spiral
static float y_init = 0.0f;     // Initial y-position of spiral

static uint8_t init_t_sight_flag = 1;
static uint8_t finalize_t_sight_flag = 1;
static float t_sight_start = 0.0;
static float t_sight_end = 0.0;

/**** Phase-2 Parameters ****/
static uint8_t iTarget;
static uint8_t iorder = 0;
static float x_target;
static float y_target;

/**** Waypoint Parameters ****/
static uint16_t iwp = 0;        // Waypoint iterator
static float xwp = 0.0f;        // Waypoint x-position
static float ywp = 0.0f;        // Waypoint y-position
static float Hwp = 0.0f;        // Waypoint heading
static uint8_t init_flag = 1;
static uint8_t trans_flag = 0;

/**** Snapshot Data and Parameters ****/
#define Ntargets 4              // Total number of targets
#define Ndim 5                  // Dimension of snapshot data structure
#define nG 3                    // Maximum number of snapshots that can be saved
static uint16_t no_snap = 0;
static char n_persons_found = 0; // Number of persons found
static char persons_found[Ntargets] = {0,0,0,0};
static uint16_t n_snaps[Ntargets] = {0,0,0,0};
static uint8_t order[Ntargets] = {3,2,1,0};
static float G_inc[Ntargets][Ndim][nG];

/**** Output to Mission Planner ****/
static float pos_error = 0.0f;	// Position Error

#endif /* MISSION_PLAN_PARAMETERS_H */
