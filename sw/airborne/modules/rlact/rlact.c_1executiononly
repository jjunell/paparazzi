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
 
 /* This module creates a flight plan callable function to learn high level guidance via reinforcement learning.  written by Jaime Junell */

#include "rlact.h"
#include "generated/airframe.h"
#include <time.h>
#include <stdlib.h>

#include "messages.h"
#include "mcu_periph/uart.h"
#include "subsystems/datalink/downlink.h"
#include "generated/flight_plan.h"  //needed to use WP_HOME

//*************** DECLARE VARIABLES *****************//
const int16_t del = 256;// distance to move in each action
int8_t act;

//*********************** FUNCTIONS ***********************//
void rlact_init(void) {

printf("init1\n");
	act= 0; //initialize it
printf("init2\n");
	srand(time(NULL));
printf("init3\n");
  //  const int nact = 4; //number of actions = 4 (NESW)

}

///////*  OWN FUCTION TO CALL FROM FLIGHT PLAN *//////
bool_t rlact_run(uint8_t wpa, uint8_t wpb){

 printf("started\n");

	//act = (rand() % 8)+1;
	act = 0;

    	//execute in paparazzi sim/IRL
	switch (act){
	    case 0: /* no movement */
	    waypoints[wpb].x = waypoints[wpa].x;    
		waypoints[wpb].y = waypoints[wpa].y;
		break;
		case 1: /* north */
		waypoints[wpb].x = waypoints[wpa].x;    
		waypoints[wpb].y = waypoints[wpa].y + del;
//		++ka[0];
		break;
		case 2: /* east */
		waypoints[wpb].x = waypoints[wpa].x + del;
		waypoints[wpb].y = waypoints[wpa].y;
//		++ka[1];
		break;
		case 3: /* south */
		waypoints[wpb].x = waypoints[wpa].x;
		waypoints[wpb].y = waypoints[wpa].y - del;
//		++ka[2];
		break;      
		case 4: /* west */
		waypoints[wpb].x = waypoints[wpa].x - del;
		waypoints[wpb].y = waypoints[wpa].y;
//		++ka[3];
		break;    
		case 5: /* northwest */
		waypoints[wpb].x = waypoints[wpa].x - del;    
		waypoints[wpb].y = waypoints[wpa].y + del;
//		++ka[4];
		break;
		case 6: /* northeast */
		waypoints[wpb].x = waypoints[wpa].x + del;
		waypoints[wpb].y = waypoints[wpa].y + del;
//		++ka[5];
		break;
		case 7: /* southeast */
		waypoints[wpb].x = waypoints[wpa].x + del;
		waypoints[wpb].y = waypoints[wpa].y - del;
//		++ka[6];
		break;      
		case 8: /* southwest */
		waypoints[wpb].x = waypoints[wpa].x - del;
		waypoints[wpb].y = waypoints[wpa].y - del;
//		++ka[7];
		break;
		default: /* no movement */
	    waypoints[wpb].x = waypoints[wpa].x;
		waypoints[wpb].y = waypoints[wpa].y;
//		state_next = state_curr; ns_next = ns_curr;
		//printf("Error: no valid action taken\n");
		break;
	}
		return FALSE;
}  // end of rlact_run function

