/*


*/

#include "detect_reward.h"
#include "modules/computer_vision/colorfilter.h"
#include "firmwares/rotorcraft/navigation.h"
#include "generated/flight_plan.h"
#include "generated/airframe.h"
#include "state.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>


#ifndef R1_LUM_MIN
#define R1_LUM_MIN 41
#endif
#ifndef R1_LUM_MAX
#define R1_LUM_MAX 183
#endif
#ifndef R1_CB_MIN
#define R1_CB_MIN 82
#endif
#ifndef R1_CB_MAX
#define R1_CB_MAX 137
#endif
#ifndef R1_CR_MIN
#define R1_CR_MIN 160
#endif
#ifndef R1_CR_MAX
#define R1_CR_MAX 249
#endif



uint8_t detected_reward   = false;
const uint32_t thresholdColorCount   = 0.05 * 124800;   //520 x 240 = 124.800 total pixels  (fwd facing)

uint8_t color_lum_min;
uint8_t color_lum_max;

uint8_t color_cb_min;
uint8_t color_cb_max;

uint8_t color_cr_min;
uint8_t color_cr_max;

/*
 * Initialize function
 */

void detect_reward_init()
{
// Initialize the variables of the colorfilter
  color_lum_min = R1_LUM_MIN;
  color_lum_max = R1_LUM_MAX;
  color_cb_min  = R1_CB_MIN;
  color_cb_max  = R1_CB_MAX;
  color_cr_min  = R1_CR_MIN;
  color_cr_max  = R1_CR_MAX;

}

void detect_reward_periodic()
{
	detected_reward = color_count > thresholdColorCount;

	printf("Color_count: %d  threshold: %d reward_flag: %d \n", color_count, thresholdColorCount, detected_reward);
}
