/*
 * Copyright (C) 2015
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/**
 * @file modules/computer_vision/colorfilter.h
 */

#ifndef VIS_AP_H
#define VIS_AP_H

#include <stdint.h>
#include "modules/computer_vision/cv.h"

// Module functions
uint8_t colmax(uint32_t colarr[3][3],uint8_t maxcolarr[3]);
void my_image_yuv422_colorcounter(struct image_t *input);
extern struct image_t *colorfilter_func(struct image_t *img);

extern struct video_listener *listener;

extern uint8_t print_cvarrs(void);
extern uint8_t cv_3grids(void);

extern void vis_ap_init(void);


#endif /* VIS_AP_H */
