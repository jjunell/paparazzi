/*
 * Copyright (C) paki
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/manan_test/manan_test.h"
 * @author paki
 * testing if nps can compile custom modules
 */

#ifndef VIS_NPS_H
#define VIS_NPS_H

#include <inttypes.h>
#include <unistd.h>

struct MemoryStruct
{
  char *memory;
  size_t size;
};

struct BmpStruct
{
    unsigned char *buffer;
    unsigned long size;
    uint16_t width;
    uint16_t height;
    uint8_t pixel_size;
    uint16_t row_stride;
};

// static size_t WriteMemoryCallback(void *contents, size_t size, size_t nmemb, void *userp);

void curl2mem(struct MemoryStruct *chunk);

uint8_t get_bmp(unsigned char *jpg_buffer, unsigned long jpg_size, struct BmpStruct *bmp);

unsigned char* get_pointertopix(unsigned char *startpointer , int row, int col, int rowwidth);
uint8_t colmax(uint32_t colarr[2][3],uint8_t maxcolarr[3]);
uint8_t count_pixels_in_three_grids(struct BmpStruct *bmpstructPtr);

extern uint8_t cv_3grids(void);

#endif
