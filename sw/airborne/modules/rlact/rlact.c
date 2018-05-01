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
#include <stdio.h>

#include "pprzlink/messages.h"
#include "mcu_periph/uart.h"
#include "subsystems/datalink/downlink.h"
#include "generated/flight_plan.h"  //needed to use WP_HOME

/** Set the default File logger path to the USB drive for ardrone, other for */
#if RLACT_NPS
#define RLACT_FILEPATH "./sw/airborne/modules/rlact/"   // for simulation (nps)
const int8_t detected_reward;
#else
#define RLACT_FILEPATH "/data/video/usb/"     // for ardrone (ap)
#include "modules/detect_reward/detect_reward.h"

#endif



//*************** DECLARE VARIABLES *****************//
// environment and states for RL algorithm
int8_t drow, dcol;
int8_t state_curr, state_next;
int8_t ns_curr, ns_next;  //current nectar state 0-5 (base 0)
int8_t vs;  //vision state, 0= nothing seen, 1= flower seen, 2= hive seen
const int8_t nns = 5;  //maximum nectar state (base 0)
const int8_t ndim = 6;  //number of elements in a dimension = 6 (nrows,ncols, nns)
const int8_t nstates = 36;  //=ndim*ndim

// flags
int8_t nsflag;  //nectar state flag = nectar full
int8_t hbflag;      //hitbounds flag
int8_t falseposflag;  // false positive reward detection

// Bellman equation
double V_old, alphav;
const double belgam = 0.9;  // gamma for belman equation
static double V[36][6] = { { 0 } };  // Value function initialized to zeros
static int kv[36][6] = { { 0 } };  // number of times a state has been visited
static int ka[8] = { 0 };  //number of times an action has been taken, initialized as 0
// with ka declared as 'int', as long as the GCS is not stopped, the value will not reinitialize
// with ka declared as 'static int', same thing.  GCS and server processes can be stopped, but stopping the simulator process, will cause the ka variable to be reinitialized.
double reward;

// Storing value function:  Value function, V, is stored in V[36][6] for algorithm purposes.  But each iteration the value function and kv matrices will be written to a file so as to be saved and not as easily rewritten.
FILE *file_Vfcn, *file_kv;
FILE *file_reg, *file_regw, *file_actw, *file_act;

int16_t i, j, k;

char filename_regen[200];
char filename_act[200];
char filename_Vfcn[200];
char filename_kv[200];

// counting false positive and false negative reward detection
static int falsepos[36] = { 0 };
static int falseneg[36] = { 0 };

FILE *file_falseneg, *file_falsepos;
char filename_falsepos[200];
char filename_falseneg[200];

// for execution of RL in paparazzi
struct EnuCoor_f my_wp;
const double del = 0.75;  // distance to move in each action
static int32_t pass;

// policy decisions - just random for now
const int8_t nact = 8;  //number of actions possible (NESW + diagonals)
int8_t act;
int16_t eps;  //eps=0 is random, eps=100 is full greedy.

// subfunctions
//int8_t hitsbounds(uint16_t state_curr_sf1, uint8_t act_sf1)

//int8_t chooseact(uint16_t state_curr_sf2, uint8_t ns_curr_sf2 ,uint8_t nact_sf2, uint16_t eps_sf2)
int8_t a, act_sf2;
//int8_t index[8] = {-1};
double Vact[8], max;

//*********************** FUNCTIONS ***********************//
void rlact_init(void)
{
//initialize variables
  printf("init1\n");
  state_curr = 0;
  state_next = 0;
  ns_curr = 0;
  ns_next = 0;
  vs = 0;

  nsflag = 0;
  hbflag = 0;

  reward = 0.0;

  act = 0;
  pass = 0;
  eps = 0;

  my_wp.z = NAV_DEFAULT_ALT;

  printf("init2\n");
  srand(time(NULL));  //initialize random number generator just once
  printf("init3\n");
  //create filenames for files created
  sprintf(filename_regen, "%sregen_rand.txt", RLACT_FILEPATH);
  sprintf(filename_act, "%sactions_rand.txt", RLACT_FILEPATH);

  // Just in case file has left over numbers from last time, clear file
  file_actw = fopen(filename_act, "w");
  fclose(file_actw);
  file_regw = fopen(filename_regen, "w");
  fclose(file_regw);

  printf("tert: %s\n", RLACT_FILEPATH);
}

///////*  OWN FUCTION TO CALL FROM FLIGHT PLAN *//////
bool rlact_run(uint8_t wpa, uint8_t wpb)
{
  pass++;
  printf("pass = %d\n",pass);

  // first pass?  choose initial state randomly, and move wpb wrt p00
  if (pass == 1) {

    state_curr = rand() % nstates;  //Random state between 0-35;
    drow = state_curr % ndim;  // remainder = #increments to move in y from home
    dcol = (int) state_curr / ndim;  // rounded down = #increments to move in x from home
    my_wp.x = waypoint_get_x(WP_p00) + dcol * del;
    my_wp.y = waypoint_get_y(WP_p00) + drow * del;
    waypoint_set_enu(wpb, &my_wp);

    //reopen regeneration file and save the first random generation state
    file_reg = fopen(filename_regen, "a");
    fprintf(file_reg, "%d ", state_curr);
    fclose(file_reg);

  } else {
    //choose initial action and print to appendable file
    act = chooseact(state_curr, ns_curr, eps);
    file_act = fopen(filename_act, "a");
    fprintf(file_act, "%d ", act);

    file_reg = fopen(filename_regen, "a");

    printf("state = %d, ns= %d, visit# %d, Vold= %.4f, ", state_curr, ns_curr,
        (kv[state_curr][ns_curr]+1), V[state_curr][ns_curr]);

    // FOR flight test - use camera to detect flower
    if (!RLACT_NPS) {
      vs = detected_reward;
    }

    switch (state_curr) {
      case 2:
      case 22:
      case 30:  // flowers
        if (RLACT_NPS) {
          vs = 1;
          printf("/in simulated flower state/ ");
        }  //for sim- define vision state based on location state
        if (vs == 0) {  //false negative
          ++falseneg[state_curr];
          printf(
              "false negative: reward detection missed at flower in state: %d",
              state_curr);
        }
        else {
          printf("/reward state succesfully detected/");
        }

        break;
      case 35:      // if in hive state
        vs = 2;
        break;  // assume perfect detection of hive
      default:         // not a reward space (code done for random policy)
        if (RLACT_NPS) {
          vs = 0;
        }  // for sim
        if (vs == 1) {  //false positive
          falseposflag = 1;
          ++falsepos[state_curr];
          printf("false positive: unwarranted reward given in state: %d",
              state_curr);
        }
    }  // switch statement - vision state

    // give rewards for current state BASED ON VISION and calculate next state
    switch (vs) {
      case 1:  // flowers
        reward = 8.0;

        ns_next = ns_curr + 1;
        if (falseposflag == 1) {
          ns_next = ns_curr;
        }  //if reward is false positive, don't go to next nectar state
        nsflag = 0;

        if (ns_next > nns) {  //if full of nectar, no reward
          ns_next = nns;
          reward = 0.0;
          nsflag = 1;
        }
        // if action results in out-of-bounds, decrease reward, choose new action
        hbflag = hitsbounds(state_curr, act);
        while (hbflag) {
          printf("//wallhit//");
          reward = reward - 1.0;
          act = chooseact(state_curr, ns_curr, eps);
          fprintf(file_act, "%d ", act);
          hbflag = hitsbounds(state_curr, act);
        }

        // calculate next state
        // if random policy or last nectar state, execute actions
        // if ep-greedy then randomly generate anywhere on the board
        if (nsflag || eps == 0) {
          switch (act) {
            case 1:
              state_next = state_curr + 1;
              break;    //north
            case 2:
              state_next = state_curr + ndim;
              break;  //east
            case 3:
              state_next = state_curr - 1;
              break;     //south
            case 4:
              state_next = state_curr - ndim;
              break;  //west
            case 5:
              state_next = state_curr + 1 - ndim;
              break;  //northwest
            case 6:
              state_next = state_curr + 1 + ndim;
              break;  //northeast
            case 7:
              state_next = state_curr - 1 + ndim;
              break;  //southeast
            case 8:
              state_next = state_curr - 1 - ndim;
              break;  //southwest
            default:
              break;
          }
        } else {  //if ep-greedy (eps>0)
          state_next = rand() % nstates;
          fprintf(file_reg, "%d ", state_next);
          act = 9;  // special regen implementation for flowers during e-greedy
        }
        break;

      case 2:      // if in hive state
        reward = 0.99 * (double) ns_curr * (double) ns_curr;  //hive reward function base 0
        // reset at nectar state 0, and a random grid location.
        ns_next = 0;
        state_next = rand() % nstates;
        fprintf(file_reg, "%d ", state_next);
        act = 9;  //special hive implementation for random regeneration
        break;

      default:         // not a reward space (code done for random policy)
        // if not in reward spot: check if boundary is hit, if so give negative reward and stay in same state, otherwise, calculate next state and give no reward.
        reward = 0.0;
        ns_next = ns_curr;

        hbflag = hitsbounds(state_curr, act);
        if (hbflag) {
          reward = -1.0;
          state_next = state_curr;
          act = 0;  // special action for not moving; (not really a chosen action, but needed for implemention in paparazzi.)
        } else {
          switch (act) {
            case 1:
              state_next = state_curr + 1;
              break;    //north
            case 2:
              state_next = state_curr + ndim;
              break;  //east
            case 3:
              state_next = state_curr - 1;
              break;     //south
            case 4:
              state_next = state_curr - ndim;
              break;  //west
            case 5:
              state_next = state_curr + 1 - ndim;
              break;  //northwest
            case 6:
              state_next = state_curr + 1 + ndim;
              break;  //northeast
            case 7:
              state_next = state_curr - 1 + ndim;
              break;  //southeast
            case 8:
              state_next = state_curr - 1 - ndim;
              break;  //southwest
            default:
              break;
          }
        }
    }  // switch statement - reward function

    //override stay in same nectar state:
    ns_curr = 0;
    ns_next = 0;

    /* Now with reward and next state calculated:
     1) update value function for current state using belman eqn
     2) take action in paparazzi sim/IRL
     3) reset "next state" to "current state" for next iteration */

    // update Value function
    V_old = V[state_curr][ns_curr];
    ++kv[state_curr][ns_curr];

    alphav = 1.0 / (double) kv[state_curr][ns_curr];
    V[state_curr][ns_curr] = V_old
        + alphav * (reward + belgam * V[state_next][ns_next] - V_old);
    printf(" Vnew= %.4f\n", V[state_curr][ns_curr]);

    ////////// update value function file ////////
    if (pass == 11  || pass == 21  || pass == 31  || pass == 41  || pass == 51  ||
        pass == 61  || pass == 71  || pass == 81  || pass == 91  || pass == 101 ||
        pass == 111 || pass == 121 || pass == 131 || pass == 141 || pass == 151 ||
        pass == 161 || pass == 171 || pass == 181 || pass == 191 || pass == 201 ||
        pass == 211 || pass == 221 || pass == 231 || pass == 241 || pass == 251 ||
        pass == 261 || pass == 271 || pass == 281 || pass == 291 || pass == 301 ||
        pass == 311 || pass == 321 || pass == 331 || pass == 341 || pass == 351 ||
        pass == 361 || pass == 371 || pass == 381 || pass == 391 || pass == 401 ||
        pass == 411 || pass == 421 || pass == 431 || pass == 441 || pass == 451 ||
        pass == 461 || pass == 471 || pass == 481 || pass == 491 || pass == 501 ||
        pass == 511 || pass == 521 || pass == 531 || pass == 541 || pass == 551 ||
        pass == 561 || pass == 571 || pass == 581 || pass == 591 || pass == 601 ||
        pass == 611 || pass == 621 || pass == 631 || pass == 641 || pass == 651 ||
        pass == 661 || pass == 671 || pass == 681 || pass == 691 || pass == 701 ||
        pass == 711 || pass == 721 || pass == 731 || pass == 741 || pass == 751 ||
        pass == 761 || pass == 771 || pass == 781 || pass == 791 || pass == 801 ||
        pass == 811 || pass == 821 || pass == 831 || pass == 841 || pass == 851 ||
        pass == 861 || pass == 871 || pass == 881 || pass == 891 || pass == 901 ||
        pass == 911 || pass == 921 || pass == 931 || pass == 941 || pass == 951 ||
        pass == 961 || pass == 971 || pass == 981 || pass == 991 || pass == 1001 )  {

      // create a filename for file
      sprintf(filename_Vfcn, "%sVfcn%d.txt", RLACT_FILEPATH, (pass - 1));
      sprintf(filename_kv, "%skv%d.txt", RLACT_FILEPATH, (pass - 1));
      sprintf(filename_falsepos, "%sfalsepos%d.txt", RLACT_FILEPATH, (pass - 1));
      sprintf(filename_falseneg, "%sfalseneg%d.txt", RLACT_FILEPATH, (pass - 1));

      // open the file
      file_Vfcn = fopen(filename_Vfcn, "w");
      file_kv = fopen(filename_kv, "w");
      file_falsepos = fopen(filename_falsepos, "w");
      file_falseneg = fopen(filename_falseneg, "w");

      if (file_Vfcn == NULL) {
        printf(
            "Error! 'Vfcn.txt' NULL. No Value Function updates written to file.\n");
      }

      else {
        for (i = 0; i < ndim; i++) {
          for (j = 0; j < nstates; j++) {
            fprintf(file_Vfcn, "%.10f ", V[j][i]);
            fprintf(file_kv, "%d ", kv[j][i]);

            if (i == 0) {
              fprintf(file_falsepos, "%d ", falsepos[j]);
              fprintf(file_falseneg, "%d ", falseneg[j]);
            }
          }
        }  // end loops to print falsepos, Vfcn and kv in file
      }  //end security check

      fclose(file_Vfcn);
      fclose(file_kv);
      fclose(file_falsepos);
      fclose(file_falseneg);
    }  //end update value function

    //execute in paparazzi sim/IRL
    switch (act) {
      case 0: /* no movement */
        my_wp.z = waypoint_get_alt(WP_p00);
        my_wp.x = waypoint_get_x(wpa);
        my_wp.y = waypoint_get_y(wpa);
        waypoint_set_enu(wpb, &my_wp);

        break;
      case 1: /* north */
        my_wp.z = waypoint_get_alt(WP_p00);
        my_wp.x = waypoint_get_x(wpa);
        my_wp.y = waypoint_get_y(wpa) + del;
        waypoint_set_enu(wpb, &my_wp);

        ++ka[0];
        break;
      case 2: /* east */
        my_wp.z = waypoint_get_alt(WP_p00);
        my_wp.x = waypoint_get_x(wpa) + del;
        my_wp.y = waypoint_get_y(wpa);
        waypoint_set_enu(wpb, &my_wp);

        ++ka[1];
        break;
      case 3: /* south */
        my_wp.z = waypoint_get_alt(WP_p00);
        my_wp.x = waypoint_get_x(wpa);
        my_wp.y = waypoint_get_y(wpa) - del;
        waypoint_set_enu(wpb, &my_wp);

        ++ka[2];
        break;
      case 4: /* west */
        my_wp.z = waypoint_get_alt(WP_p00);
        my_wp.x = waypoint_get_x(wpa) - del;
        my_wp.y = waypoint_get_y(wpa);
        waypoint_set_enu(wpb, &my_wp);

        ++ka[3];
        break;
      case 5: /* northwest */
        my_wp.z = waypoint_get_alt(WP_p00);
        my_wp.x = waypoint_get_x(wpa) - del;
        my_wp.y = waypoint_get_y(wpa) + del;
        waypoint_set_enu(wpb, &my_wp);

        ++ka[4];
        break;
      case 6: /* northeast */
        my_wp.z = waypoint_get_alt(WP_p00);
        my_wp.x = waypoint_get_x(wpa) + del;
        my_wp.y = waypoint_get_y(wpa) + del;
        waypoint_set_enu(wpb, &my_wp);

        ++ka[5];
        break;
      case 7: /* southeast */
        my_wp.z = waypoint_get_alt(WP_p00);
        my_wp.x = waypoint_get_x(wpa) + del;
        my_wp.y = waypoint_get_y(wpa) - del;
        waypoint_set_enu(wpb, &my_wp);

        ++ka[6];
        break;
      case 8: /* southwest */
        my_wp.z = waypoint_get_alt(WP_p00);
        my_wp.x = waypoint_get_x(wpa) - del;
        my_wp.y = waypoint_get_y(wpa) - del;
        waypoint_set_enu(wpb, &my_wp);

        ++ka[7];
        break;
      case 9: /* special hive/flower random regeneration */
        drow = state_next % ndim;  // remainder = #increments to move in y from home
        dcol = (int) state_next / ndim;  // rounded down = #increments to move in x from home
        my_wp.z = waypoint_get_alt(WP_p00);
        my_wp.x = waypoint_get_x(WP_p00) + dcol * del;
        my_wp.y = waypoint_get_y(WP_p00) + drow * del;
        waypoint_set_enu(wpb, &my_wp);

        break;
      default: /* no movement */
        my_wp.z = waypoint_get_alt(WP_p00);
        my_wp.x = waypoint_get_x(wpa);
        my_wp.y = waypoint_get_y(wpa);
        waypoint_set_enu(wpb, &my_wp);
        state_next = state_curr;
        ns_next = ns_curr;
        printf("default action stay still: no valid action taken\n");
        break;
    }

    // reset "next state" to "current state" for next iteration
    state_curr = state_next;
    ns_curr = ns_next;

    //close all open files
    fclose(file_act);
    fclose(file_reg);

  }  //if not first pass

  return FALSE;
}  // end of rlact_run function

///////*  OWN FUCTION TO CALL FROM FLIGHT PLAN *//////
/*** moves waypoint to state location with respect to WP_p00 */

void setWP_wrt00(uint8_t wp_id, uint8_t reward_state_id)
{

  //state must be between 0-35;  use for flower (2,22,30) and hive (35) states

  drow = reward_state_id % ndim;  // remainder = #increments to move in y from home
  dcol = (int) reward_state_id / ndim;  // rounded down = #increments to move in x from home
  my_wp.x = waypoint_get_x(WP_p00) + dcol * del;
  my_wp.y = waypoint_get_y(WP_p00) + drow * del;
  waypoint_set_enu(wp_id, &my_wp);

}

/********************* hitsbounds subfunction *************/
/*** determines if boundary is hit within a 6x6 grid environment */
/*** inputs: current state, action *** output: 1/0 integer */
int8_t hitsbounds(uint16_t state_curr_sf1, uint8_t act_sf1)
{
  drow = state_curr_sf1 % 6;  //= #increments current state is from home in y
  dcol = (int) state_curr_sf1 / 6;  //= #increments current state is from home in x

  int sol = 0;
  switch (act_sf1) {  //going north or south?
    case 1:
    case 5:
    case 6:  //north, northwest, northeast
      if (drow >= (6 - 1)) {
        sol = 1;
      }
      break;
    case 3:
    case 7:
    case 8:  //south, southeast, southwest
      if (drow == 0) {
        sol = 1;
      }
      break;
    default:
      break;
  }  //end switch north-south

  switch (act_sf1) {  //going east or west?
    case 2:
    case 6:
    case 7:  //east, northeast, southeast
      if (dcol >= (6 - 1)) {
        sol = 1;
      }
      break;
    case 4:
    case 5:
    case 8:  //west, northwest, southwest
      if (dcol == 0) {
        sol = 1;
      }
      break;
    default:
      break;
  }  //end switch east-west

  if (sol == 1) {
    return 1;
  } else {
    return 0;
  }
}

/********************* chooseact subfunction *************/
/*** chooses an action between 1-8 to take.  */
/*** input: Value function, policy, current state, nectar state*/
/*** output: action 1-8 (North, east, south, west, NW, NE, SE, SW) */
int8_t chooseact(uint16_t state_curr_sf2, uint8_t ns_curr_sf2, uint16_t eps_sf2)
{
//int8_t a,i,act_sf2;
//double Vact[8], max;
//Declare up above instead of inside.  and give them different names so they are not shadowed?
// question: why don't I have to bring in V[36][6], or nact, or ndim?
  int8_t index_sf2[8] = { -1 };  //for some reason I cannot declare this above.

//initialize
  a = 0;

  if (eps == 0) {  //random
    act_sf2 = (rand() % nact) + 1;
  } else {  //ep_greedy or full greedy depending on eps value
    if (eps_sf2 < (rand() % 100)) {
      act_sf2 = (rand() % nact) + 1;
    } else {
      //for each action fill in goodness value or NaN
      for (a = 1; a < nact + 1; a++) {
        hbflag = hitsbounds(state_curr_sf2, a);
        if (hbflag) {
          Vact[(a - 1)] = 0.0 / 0.0;  //if hits bounds, not an option for movement
        } else {
          switch (a) {
            case 1:
              Vact[0] = V[state_curr_sf2 + 1][ns_curr_sf2];
              break;			//north
            case 2:
              Vact[1] = V[state_curr_sf2 + ndim][ns_curr_sf2];
              break;		//east
            case 3:
              Vact[2] = V[state_curr_sf2 - 1][ns_curr_sf2];
              break;			//south
            case 4:
              Vact[3] = V[state_curr_sf2 - ndim][ns_curr_sf2];
              break;  		//west
            case 5:
              Vact[4] = V[state_curr_sf2 + 1 - ndim][ns_curr_sf2];
              break;		//NW
            case 6:
              Vact[5] = V[state_curr_sf2 + 1 + ndim][ns_curr_sf2];
              break;		//NE
            case 7:
              Vact[6] = V[state_curr_sf2 - 1 + ndim][ns_curr_sf2];
              break;		//SE
            case 8:
              Vact[7] = V[state_curr_sf2 - 1 - ndim][ns_curr_sf2];
              break;		//SW
            default:
              break;
          }  //switch each state
        }  //else
      }  //for each action

//find max value of Vact
      max = Vact[0];
      for (a = 1; a < nact; a++) {
        if (isnan(max)) {
          max = Vact[a]; /*printf("\nmax is nan, new max= %f\n",max);*/
        } else if (Vact[a] > max) {
          max = Vact[a];
        }
      }

//find all the elements that have max value
      i = 0;
      for (a = 0; a < nact; a++) {
        if (Vact[a] == max) {
          index_sf2[i] = a;
          i++;
        }
      }
//there are i elements with the same max value.  Choose randomly between them
      a = rand() % i;
      act_sf2 = index_sf2[a] + 1;

    }  //else if not randomly chosen, then greedy
  }  // if eps=0(random), else it is ep_greedy

  return act_sf2;

}  // end chooseact subfuction

