#ifndef MYNAVFUNCS_H
#define MYNAVFUNCS_H

#include <inttypes.h>
#include <unistd.h>

// Making amacro to get current heading (phi)
// use ANGLE_FLOAT_OF_BFP for real value??
#define GetCurHeading() (stateGetNedToBodyEulers_f()->psi)

extern uint8_t increase_nav_heading(float increment);
extern uint8_t setHeadingNorth(void);
extern uint8_t moveWaypointForwards(uint8_t waypoint, float distanceMeters);
extern uint8_t moveWaypointLeftwards(uint8_t waypoint, float distanceMeters);
extern uint8_t setAltToWp(uint8_t waypoint_toset, uint8_t waypoint_ref);

extern uint8_t set_loc_and_att(uint8_t waypoint);

extern float toset_xloc, toset_yloc, toset_head;

#endif
