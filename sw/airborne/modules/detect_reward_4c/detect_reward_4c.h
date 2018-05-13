/*
 * Copyright (C)2018
 *
 * This file is part of paparazzi
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file "modules/detect_reward/detect_reward.h"
 * Detect a specific colour and identify as reward if past threshold
 */

#ifndef DETECT_REWARD_H
#define DETECT_REWARD_H

#include <stdint.h>
#include "modules/computer_vision/cv.h"

// macro defines

//******* FUNCTIONS ********//
extern void detect_reward_init(void);
extern void detect_reward_periodic(void);
uint16_t image_yuv422_color_counter(struct image_t *input, uint8_t y_m, uint8_t y_M, uint8_t u_m, uint8_t u_M, uint8_t v_m, uint8_t v_M);

extern uint8_t detected_reward;
//extern uint color_count;

extern struct video_listener *listener;
#endif






