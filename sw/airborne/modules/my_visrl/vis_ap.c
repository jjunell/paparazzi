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
 * @file modules/computer_vision/colorfilter.c
 */

// Own header
#include "modules/my_visrl/vis_ap.h"
#include <stdio.h>

#include "modules/computer_vision/lib/vision/image.h"

#include "visrl.h" //Contains the defined variables that stores CV output

PRINT_CONFIG_VAR(VISRL_FPS)

struct video_listener *listener = NULL;

// Filter Settings

uint32_t count_arr[2][3] = {{0}};
uint32_t sumcount_arr[2] = {0,0};
uint32_t redcount_arr[3] = {0,0,0};
uint32_t redsum = 0;

#ifdef VISRL_TWOGOALS
uint32_t bluecount_arr[3] = {0,0,0};
uint32_t bluesum = 0;
#endif

uint8_t domcol_arr[3] = {0,0,0};

// dont understand why i have to define again here when they are included in
// visrl.h
float red_thresh = 0.46;
float blue_thresh = 0.46;
uint16_t min_pix_thresh = 1000;


uint8_t colmax(uint32_t colarr[2][3],uint8_t maxcolarr[3])
{
    for (int i = 0; i < 3; i++) {
        uint32_t curcolmax = 0;
        uint8_t curcolmax_ind = 0;
        for (int j = 0; j < 2; j++) {
            if (colarr[j][i] > curcolmax) {
                curcolmax = colarr[j][i];
                curcolmax_ind = j+1;
            }
        }
        // change the array only if (pixels above the threshold) are above a
        // threshold
        if (curcolmax > min_pix_thresh) {
            maxcolarr[i] = curcolmax_ind;
        }
        else {
            maxcolarr[i] = 0;
        }
    }
    return 0;
}

/* THIS CAN BE WRITTEN MUCH BETTER
 * taken from image_yuv422_colorfilter from image.c file.modified to
 * convert it to an encoder that takes vision information and uses the pixel
 * data to come up with a state decription. Three columns are defined on the
 * vision stream and the number of pixels above a pre-defined threshold is
 * counted for the RBG channels in each of the columns. A 3x3 array consisting
 * of this information is outputted
 * */
void my_image_yuv422_colorcounter(struct image_t *input)
{
    uint8_t *source = input->buf;
    float red,green,blue,pixcolsum;
    float f_red,f_blue;
    // Reset the output array

    uint16_t w_first, w_second;
    uint8_t coltoapp;
    w_first = input->w/3;
    w_second = 2*w_first;
    uint16_t whitecount = 0;

    //reset relevant arrays and memory to 0
    for (int i = 0; i < 3; i++) {
        redcount_arr[i] = 0;
#ifdef VISRL_TWOGOALS
        bluecount_arr[i] = 0;
    }
    bluesum = 0;
#else
    }
#endif
    redsum = 0;

  // Go trough all the pixels
    for (uint16_t y = 0; y < input->h; y++) {
        for (uint16_t x = 0; x < input->w; x += 2) {

            // converting to RGB
            //http://softpixel.com/~cwright/programming/colorspace/yuv/
            red = (float )source[1] + 1.4075*((float )source[2] - (float )128);
            green = (float )source[1] - 0.3455*((float )source[0] - (float )128) - ((float )0.7169 * ((float )source[2] - (float )128));
            blue = (float )source[1] + 1.779*((float )source[0] - (float )128);
            pixcolsum = red+green+blue;

            // estimating fractions of rgb
            f_red = red/pixcolsum;
            f_blue = blue/pixcolsum;

            // setting column in output that should be appended
            if (x < w_first) {
                coltoapp = 0;
            } else if (x < w_second) {
                coltoapp = 1;
            } else {
                coltoapp = 2;
            }

            // colorize pixel above threshold and colorize; otherwise make BW
//             if (((f_red > red_thresh) && (f_green > green_thresh)) ||
//                 ((f_red > red_thresh) && (f_blue > blue_thresh)) ||
//                 ((f_blue > blue_thresh) && (f_green > green_thresh))) {
            if ((f_red > red_thresh) && (f_blue > blue_thresh)) {
                source[0] = 240;        // U
                source[2] = 240;        // V
                whitecount++;
                source[1] = 254;
                source[3] = 254;
            }
            else if (f_red > red_thresh) {
                source[0] = 10;        // U
                source[2] = 240;        // V
                redcount_arr[coltoapp]++;
            }
#ifdef VISRL_TWOGOALS
            else if (f_blue > blue_thresh) {
                source[0] = 240;        // U
                source[2] = 10;        // V
                bluecount_arr[coltoapp]++;
            }
#endif
            else {
                source[0] = 127;        // U
                source[2] = 127;        // V
            }

            // Go to the next 2 pixels
            source += 4;
        }
    }

    // Copying red CV output to count_arr
    for (int i = 0; i < 3; i++) {
        count_arr[0][i] = redcount_arr[i];
        redsum += redcount_arr[i];
#ifdef VISRL_TWOGOALS
        count_arr[1][i] = bluecount_arr[i];
        bluesum += bluecount_arr[i];
    }
    sumcount_arr[1] = bluesum;
#else
    }
#endif
    sumcount_arr[0] = redsum;
    // printf("white:%d \n",whitecount);
    colmax(count_arr,domcol_arr);
    printf("\n%d,%d,%d ", domcol_arr[0], domcol_arr[1], domcol_arr[2]);
    printf("|R%d,%d,%d ", count_arr[0][0], count_arr[0][1], count_arr[0][2]);
#ifdef VISRL_TWOGOALS
    printf("B%d,%d,%d", count_arr[1][0], count_arr[1][1], count_arr[1][2]);
#endif
}

// Function
struct image_t *colorfilter_func(struct image_t *img)
{
  // Filter
  my_image_yuv422_colorcounter(img);

    //Print the statez
    /* Prints the color state information
  char colors[] = "RGB";
    printf("\tcol1 \tcol2 \tcol3\n");
    for (int ri = 0; ri < 3; ri++) {
        printf("%c:",colors[ri]);
        for (int ci = 0; ci <3; ci++) {
            printf("\t%d",count_arr[ri][ci]);
        }
        printf("\n");
    }
    */


  // Draw lines for the columns
//     uint16_t w_first, w_second;
// 
//     w_first = img->w/3;
//     w_second = 2*w_first;

//     struct point_t myfrom = { w_first, 0 };
//     struct point_t myto = { w_first, img->h };
    // printf("w_first:%d img->h:%d\n",w_first,img->h);
    static uint8_t linecol[4] = {200, 100, 200, 100};
    struct point_t myfrom = { 425, 0 };
    struct point_t myto = { 425, 720 };
    struct point_t myfrom2 = { 900, 0 };
    struct point_t myto2 = { 900, 720 };

//     image_draw_line_color(img, &myfrom, &myto, linecol);
//     image_draw_line_color(img, &myfrom2, &myto2, linecol);

    image_draw_line_color(img, &myfrom, &myto, linecol);
    image_draw_line_color(img, &myfrom2, &myto2, linecol);

//     struct point_t myfrom3 = { 920, 0 };
//     struct point_t myto3 = { 920, 720 };
    // image_draw_line_color(img, &myfrom3, &myto3, linecol);
    /*
    */
    // image_free(&img2);

  return img;
}

void vis_ap_init(void)
{
  listener = cv_add_to_device(&VISRL_CAMERA, colorfilter_func, VISRL_FPS);
}

uint8_t cv_3grids(void) { return 0; }
