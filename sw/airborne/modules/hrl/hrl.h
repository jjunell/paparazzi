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

#ifndef HRL_H
#define HRL_H

#include "std.h"
#include <stdint.h>
#include "firmwares/rotorcraft/navigation.h"

// external variable declarations

// macro defines

//******* FUNCTIONS ********//
void hrl_init(void);
extern bool hrl_run(uint8_t wpa, uint8_t wpb);
extern void setWP_wrt00(uint8_t wp_id, uint8_t reward_state_id);
int8_t hitsbounds(uint16_t state_curr, uint8_t act);
int8_t chooseopt(uint16_t state_curr, uint8_t ns_curr, uint8_t lvs_curr, uint16_t eps);
int8_t primact(uint8_t opt_sf3, uint8_t optT0_sf3);

#endif
