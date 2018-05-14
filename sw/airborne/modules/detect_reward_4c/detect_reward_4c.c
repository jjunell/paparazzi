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
 *  @file modules/detect_reward_4c/detect_reward_4c.c
 */


#include "detect_reward_4c.h"

#include "generated/airframe.h"
#include "modules/computer_vision/lib/vision/image.h"
//#include "subsystems/abi.h"

#include <stdio.h>

#ifndef DETECTREWARD_FPS
#define DETECTREWARD_FPS 10       ///< Default FPS (zero means run at camera fps)
#endif
PRINT_CONFIG_VAR(DETECTREWARD_FPS)


struct video_listener *listener = NULL;

uint8_t detected_reward = 0;
const uint32_t thresholdColorCount = 0.05 * 124800;   //520 x 240 = 124.800 total pixels  (fwd facing)
static int detect_array[4] = { 0, 0, 0, 0 };  //logical for each color detection (Y1 = hive, R1 = F1, B1=F2, P1=F3)

uint8_t color_lum_min[4];
uint8_t color_lum_max[4];
uint8_t color_cb_min[4];
uint8_t color_cb_max[4];
uint8_t color_cr_min[4];
uint8_t color_cr_max[4];

const int8_t nc = 4;  // number of colors

// Result
uint32_t color_count;

static uint32_t image_yuv422_color_counter(struct image_t *input,
    uint8_t y_m, uint8_t y_M,
    uint8_t u_m, uint8_t u_M,
    uint8_t v_m, uint8_t v_M);

static struct image_t *detect_reward_process_img(struct image_t *img);

/*
 * Initialize function
 */

void detect_reward_init(void)
{
 listener = cv_add_to_device(&DETECTREWARD_CAMERA, detect_reward_process_img, DETECTREWARD_FPS);

 // yellow color
 color_lum_min[0] = 126;
 color_lum_max[0] = 255;
 color_cb_min[0]  = 0;
 color_cb_max[0]  = 120;
 color_cr_min[0]  = 91;
 color_cr_max[0]  = 180;

 // red color
 color_lum_min[1] = 41;
 color_lum_max[1] = 230;
 color_cb_min[1]  = 82;
 color_cb_max[1]  = 137;
 color_cr_min[1]  = 160;
 color_cr_max[1]  = 249;

 // blue color
 color_lum_min[2] = 75;
 color_lum_max[2] = 255;
 color_cb_min[2]  = 138;
 color_cb_max[2]  = 255;
 color_cr_min[2]  = 0;
 color_cr_max[2]  = 124;

 // purple color
 color_lum_min[3] = 36;
 color_lum_max[3] = 239;
 color_cb_min[3]  = 134;
 color_cb_max[3]  = 218;
 color_cr_min[3]  = 152;
 color_cr_max[3]  = 255;
}

static struct image_t *detect_reward_process_img(struct image_t *img)
{
  uint8_t i, imax=0;
  uint32_t countmax;

  // print to see uyvy color of center pixel
   //  uint8_t *img_buf = (uint8_t*)(img->buf);
   //  printf("uyvy %d %d %d %d\n", img_buf[img->w * 2 * img->h/2 + img->w], img_buf[img->w * 2 * img->h/2 + img->w + 1], img_buf[img->w * 2 * img->h/2 + img->w + 2], img_buf[img->w * 2 * img->h/2 + img->w + 3]);

  countmax = 0;
  for (i = 0; i < nc; i++) {
    color_count = image_yuv422_color_counter(img,
        color_lum_min[i], color_lum_max[i],
        color_cb_min[i], color_cb_max[i],
        color_cr_min[i], color_cr_max[i]
    );
    detect_array[i] = color_count > thresholdColorCount;
    //printf("count color %d: %d \n", i, color_count);

    if (color_count>countmax){
      imax=i;  // index of color with max count in case there are multiple POI detected
    }
  }

  //
  if ((detect_array[0] + detect_array[1] + detect_array[2] + detect_array[3])>=1){
  printf("[y,r,b,p]: [ %d %d %d %d ] ", detect_array[0],detect_array[1],detect_array[2],detect_array[3]);
  }

 //if more than one POI detected, set to detect only max count color
  if ((detect_array[0] + detect_array[1] + detect_array[2] + detect_array[3])>1)
  {
    printf("more than 1 POI detected\n");
    for (i = 0; i < nc; i++) {  //reset all to 0
      detect_array[i]=0;
    }
    detect_array[imax]= 1 ;  //set to detect only max count color
  }

  //
  if (detect_array[0]) { detected_reward = 4; }
  else if (detect_array[1]) { detected_reward = 1; }
  else if (detect_array[2]) { detected_reward = 2; }
  else if (detect_array[3]) { detected_reward = 3; }
  else { detected_reward = 0;}

  if ((detect_array[0] + detect_array[1] + detect_array[2] + detect_array[3])>=1){
  printf(", POI: %d \n", detected_reward);
  }

  return img;
}

/* // Function
struct image_t *colorcount_func(struct image_t *img);
struct image_t *colorcount_func(struct image_t *img)
{


// print to see uyvy color of center pixel
//  uint8_t *img_buf = (uint8_t*)(img->buf);
//  printf("uyvy %d %d %d %d\n", img_buf[img->w * 2 * img->h/2 + img->w], img_buf[img->w * 2 * img->h/2 + img->w + 1], img_buf[img->w * 2 * img->h/2 + img->w + 2], img_buf[img->w * 2 * img->h/2 + img->w + 3]);

  // Filter
  for (i = 0; i < nc; i++) {
  color_count[i] = image_yuv422_color_counter(img,
                                       color_lum_min[i], color_lum_max[i],
                                       color_cb_min[i], color_cb_max[i],
                                       color_cr_min[i], color_cr_max[i]
                                      );

  }


  return img; // Colorfilter did not make a new image
} */


/**
 * Counts pixels within  colors in an YUV422 image
 * @param[in] *input The input image to filter
 * @param[in] y_m The Y minimum value
 * @param[in] y_M The Y maximum value
 * @param[in] u_m The U minimum value
 * @param[in] u_M The U maximum value
 * @param[in] v_m The V minimum value
 * @param[in] v_M The V maximum value
 * @return The amount of filtered pixels
 */
static uint32_t image_yuv422_color_counter(struct image_t *input,
    uint8_t y_m, uint8_t y_M,
    uint8_t u_m, uint8_t u_M,
    uint8_t v_m, uint8_t v_M)
{
  uint32_t cnt = 0;
  uint8_t *source = (uint8_t *)input->buf;

  // Go trough all the pixels
  for (uint16_t y = 0; y < input->h; y++) {
    for (uint16_t x = 0; x < input->w; x += 2) {
      // Check if the color is inside the specified values
      if (
          (source[1] >= y_m)
          && (source[1] <= y_M)
          && (source[0] >= u_m)
          && (source[0] <= u_M)
          && (source[2] >= v_m)
          && (source[2] <= v_M)
      ) {
        cnt++;
      }

      // Go to the next 2 pixels
      source += 4;
    }
  }
  return cnt;
}
