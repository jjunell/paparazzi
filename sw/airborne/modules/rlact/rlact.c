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

#include "rlact.h"
#include "generated/airframe.h"
#include <time.h>
#include <stdlib.h>

#include "messages.h"
#include "subsystems/datalink/downlink.h"

void send_rlact(void) {
  DOWNLINK_SEND_RLACT(DefaultChannel, DefaultDevice,
                                            &xnew);
}

int32_t xnew; //declare variable

void rlact_init(void) {
	xnew = 5; //initialize it
	srand(time(NULL));


}

void rlact_periodic(void) {

  xnew = rand();
  send_rlact();

//print in a file the x-y location
fprintf(rlact_log, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
    counter,
    imu.gyro_unscaled.p,
    imu.gyro_unscaled.q,
    imu.gyro_unscaled.r,
    imu.accel_unscaled.x,
    imu.accel_unscaled.y,
    imu.accel_unscaled.z,
    imu.mag_unscaled.x,
    imu.mag_unscaled.y,
    imu.mag_unscaled.z,
    stabilization_cmd[COMMAND_THRUST],
    stabilization_cmd[COMMAND_ROLL],
    stabilization_cmd[COMMAND_PITCH],
    stabilization_cmd[COMMAND_YAW],
    quat->qi,
    quat->qx,
    quat->qy,
    quat->qz
  );

 //define size of space and distance between each square
 
 
  //xnew = xnew + 1; // use it

 // initialize location
 
 
 //wpx = wp1
 
 //get location of wpx and discretize location within an RxC grid
 
 //Look up V^pi 
 
 //choose action based on pi
 
 // move wpy
 
 //

}
