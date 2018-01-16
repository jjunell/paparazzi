// think this contains state information from that file
#include "modules/my_visrl/mynavfuncs.h"
#include "firmwares/rotorcraft/navigation.h"
#include "state.h"

float toset_xloc = 0;
float toset_yloc = 0;
float toset_head = 0;

//Increases the NAV heading. Assumes heading is an INT32_ANGLE. It is bound in this function.
uint8_t increase_nav_heading(float increment)
{
  nav_heading += ANGLE_BFP_OF_REAL(RadOfDeg(increment));
  // Check if your turn made it go out of bounds...
  INT32_ANGLE_NORMALIZE(nav_heading); // HEADING HAS INT32_ANGLE_FRAC....
  return FALSE;
}

/* The following functions are to move the waypoints in 4 directions relative
 * to the craft; these can be factored into one or two
 */
uint8_t moveWaypointForwards(uint8_t waypoint, float distanceMeters)
{
	  struct EnuCoor_i new_coor;
	  struct EnuCoor_i *pos = stateGetPositionEnu_i(); // Get your current position

	  // Calculate the sine and cosine of the heading the drone is keeping
	  float sin_heading = sinf(ANGLE_FLOAT_OF_BFP(nav_heading));
	  float cos_heading = cosf(ANGLE_FLOAT_OF_BFP(nav_heading));

	  // Now determine where to place the waypoint you want to go to
	  new_coor.x = pos->x + POS_BFP_OF_REAL(sin_heading * (distanceMeters));
	  new_coor.y = pos->y + POS_BFP_OF_REAL(cos_heading * (distanceMeters));
	  // new_coor.z = waypoint->z; // Keep the height the same
	  new_coor.z = pos->z; // Keep the height the same

	  // Set the waypoint to the calculated position
	  waypoint_set_xy_i(waypoint, new_coor.x, new_coor.y);
      float wpheight = waypoint_get_alt(waypoint);
      // waypoint_set_here(waypoint);
      waypoint_set_alt(waypoint, wpheight);

	  return FALSE;
}

uint8_t moveWaypointLeftwards(uint8_t waypoint, float distanceMeters)
{
	  struct EnuCoor_i new_coor;
	  struct EnuCoor_i *pos = stateGetPositionEnu_i(); // Get your current position

	  // Calculate the sine and cosine of the heading the drone is keeping
	  float sin_heading = sinf(ANGLE_FLOAT_OF_BFP(nav_heading));
	  float cos_heading = cosf(ANGLE_FLOAT_OF_BFP(nav_heading));

	  // Now determine where to place the waypoint you want to go to
	  new_coor.x = pos->x - POS_BFP_OF_REAL(cos_heading * (distanceMeters));
	  new_coor.y = pos->y + POS_BFP_OF_REAL(sin_heading * (distanceMeters));
	  new_coor.z = pos->z; // Keep the height the same

	  // Set the waypoint to the calculated position
	  waypoint_set_xy_i(waypoint, new_coor.x, new_coor.y);

	  return FALSE;
}

uint8_t setHeadingNorth(void)
{
    float cur_head = GetCurHeading();
    // float cur_navhead = ANGLE_FLOAT_OF_BFP(nav_heading);
    // printf("Nav %f; Curhead %f \n", ANGLE_FLOAT_OF_BFP(nav_heading),ANGLE_FLOAT_OF_BFP(GetCurHeading()));

    if (abs(cur_head) > 1.57) {
        if (cur_head > 0) {
            nav_heading = ANGLE_BFP_OF_REAL(RadOfDeg(90));
        }
        else {
            nav_heading = ANGLE_BFP_OF_REAL(RadOfDeg(270));
        }
        INT32_COURSE_NORMALIZE(nav_heading);
        return TRUE;
    }
    else {
        nav_heading = ANGLE_BFP_OF_REAL(0);
        INT32_COURSE_NORMALIZE(nav_heading);
        return FALSE;
    }
    // if (abs(cur_head) < 0.1
}

/*
uint8_t setHeadingSmooth(float targhead)
{
    float cur_head = GetCurHeading();
    float diff_head = targhead-cur_head
    // float cur_navhead = ANGLE_FLOAT_OF_BFP(nav_heading);
    // printf("Nav %f; Curhead %f \n", ANGLE_FLOAT_OF_BFP(nav_heading),ANGLE_FLOAT_OF_BFP(GetCurHeading()));

    if (abs(diff_head) > 3.14) {
        diff_head = cur_head-targhead
        }

    if (abs(diff_head) > 1.57) {
        if (diff_head > 0) {
            nav_heading = ANGLE_BFP_OF_REAL(targhead+RadOfDeg(90));
        }
        else {
            nav_heading = ANGLE_BFP_OF_REAL(targhead+RadOfDeg(90));
        }
        INT32_COURSE_NORMALIZE(nav_heading);
        return TRUE;
    }
    else {
        nav_heading = ANGLE_BFP_OF_REAL(targhead);
        INT32_COURSE_NORMALIZE(nav_heading);
        return FALSE;
    }
    // if (abs(cur_head) < 0.1
}
*/

uint8_t setAltToWp(uint8_t waypoint_toset, uint8_t waypoint_ref)
{
      float wpheight = waypoint_get_alt(waypoint_ref);
      // waypoint_set_here(waypoint);
      waypoint_set_alt(waypoint_toset, wpheight);
      return FALSE;
}

uint8_t set_loc_and_att(uint8_t waypoint)
{
	waypoint_set_xy_i(waypoint,POS_BFP_OF_REAL(toset_xloc), POS_BFP_OF_REAL(toset_yloc));
    return 0;
}
