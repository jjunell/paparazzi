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
#include "firmwares/rotorcraft/navigation.h"

// variable declarations

extern int32_t xnew; // variables that you want to use in other files
int RLbool;

#define IncrementWaypointx(_wp1) ({waypoints[_wp1].x = waypoints[_wp1].x + 256; FALSE; })
#define IncrementWaypointy(_wp1) ({waypoints[_wp1].y = waypoints[_wp1].y + 256; FALSE; })
#define RunLinesWP(_wp1,_wp2,rlact) ({waypoints[_wp2].x = waypoints[_wp1].x + rlact*256; FALSE; }) 

//******* FUNCTIONS ********//
void rlact_init(void);
extern bool_t rlact_run(uint8_t wpa, uint8_t wpb);
int8_t hitsbounds(uint16_t state_curr, uint8_t act);
int8_t chooseact(uint16_t state_curr, uint8_t ns_curr ,uint8_t nact, uint16_t eps);
void rlact_periodic(void);
extern void send_rlact(void);

#endif
