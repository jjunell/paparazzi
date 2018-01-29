/*
 * Copyright (C) 2008-2012 The Paparazzi Team
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

#ifndef RLACT_H
#define RLACT_H

#include "std.h"
#include <stdint.h>
#include "firmwares/rotorcraft/navigation.h"

// external variable declarations

// macro defines
//#define IncrementWaypointx(_wp1) ({waypoint_set_xy_i(_wp1, waypoint_get_x(_wp1) + 1, waypoint_get_y(_wp1)); FALSE; })
//#define IncrementWaypointy(_wp1) ({waypoint_set_xy_i(_wp1, waypoint_get_x(_wp1), waypoint_get_y(_wp1)) + 1; FALSE; })
//#define RunLinesWP(_wp1,_wp2,rlact) ({waypoints[_wp2].x = waypoints[_wp1].x + rlact*80; FALSE; })

//******* FUNCTIONS ********//
void rlact_init(void);
extern bool rlact_run(uint8_t wpa, uint8_t wpb);
extern void setWP_wrt00(uint8_t wp_id, uint8_t reward_state_id);
int8_t hitsbounds(uint16_t state_curr, uint8_t act);  /* include with rlact.c_5 and higher*/
int8_t chooseact(uint16_t state_curr, uint8_t ns_curr, uint16_t eps);  /* include with rlact.c_7 and higher*/
//void rlact_periodic(void);
//extern void send_rlact(void);

#endif
