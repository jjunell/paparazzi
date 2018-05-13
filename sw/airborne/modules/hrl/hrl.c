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

/* This module creates a flight plan callable function to learn high level guidance via hierarchical reinforcement learning.  written by Jaime Junell */
/* version for cyberzoo with vision (version 1) */
/* can also be compiled with sim airframe and nps target */

#include "hrl.h"
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
#define RLACT_FILEPATH "./sw/airborne/modules/hrl/"   // for simulation (nps)
int8_t detected_reward;
#else
#define RLACT_FILEPATH "/data/video/usb/"     // for ardrone (ap)
#include "modules/detect_reward/detect_reward.h"

#endif



//*************** DECLARE VARIABLES *****************//
// environment and states for RL algorithm
int8_t drow, dcol;
int8_t state_curr, state_next, state_opt0;
int8_t ns_curr, ns_next, ns_opt0;   //nectar state 0-3 (base 0)
int8_t lvs_curr, lvs_next, lvs_opt0;    //history state 0-3 (0 = hive, 1=F1, 2=F2, 3=F3)
const int8_t Nns = 3;  //last nectar state (base 0)
const int8_t nlvs = 4;  //number lvs (states)
const int8_t ndim = 6; //number of elements in a dimension = 6 (nrows,ncols)
const int8_t nstates=36; //=ndim*ndim

//vision version 1:  vs, 0= nothing seen, 1= flower seen, 2= hive seen
//vision version 2:  vs, 0= nothing seen, 1= F1 seen, 2= F2 seen, 3= F3 seen, 4= hive seen
int8_t vs;

// flags
int8_t nsflag;  //nectar state flag = nectar full
int8_t hbflag;      //hitbounds flag
int8_t optflag;   //option terminal flag
int8_t falseposflag;  // false positive reward detection

// Bellman equation
double Q_old, alpha;
const double belgam = 0.9;  // gamma for belman equation
static double Q[36][4][4][8]= {{{{0}}}}; // Q Value function initialized to zeros
static int kq[36][4][4][8]= {{{{0}}}}; // number of times a state has been visited
static int ka[4]= {0};  //number of times an action has been taken, initialized as 0
double reward;
const double rhive[4] = { 0 , 1 , 11.3137 , 46.7654 };  //reward at hive as function of ns (=ns^3.5).

// Storing Q value function:
FILE *file_Qfcn, *file_kq;

int16_t i, j, k, n;

char filename_Qfcn[5000];
char filename_kq[5000];

// counting false positive and false negative reward detection
static int falsepos[36] = { 0 };
static int falseneg[36] = { 0 };

// for execution of RL in paparazzi
struct EnuCoor_f my_wp;
const double del = .5;// distance to move in each action
static int32_t pass;
static int32_t idec;

// policy decisions
const int8_t nopt = 8; //number of actions possible (NESW + diagonals)
int8_t act, opt;
int16_t eps;  //eps=0 is random, eps=100 is full greedy.
int8_t optT0;
double optr;

// nectar time counter
const int16_t tfnr = 12;
static int fnr_count[4] = {15 , 15 , 15 , 15};

// subfunctions
static int8_t a ;
static int8_t opt_sf2;
//int8_t opt_sf3 , optT0_sf3;
static int8_t a_sf3;
static double max;


//*********************** FUNCTIONS ***********************//
void hrl_init(void) {

  //initialize variables
  printf("init1: begin hrl_init\n");
  state_curr = 0;  //p00
  state_next = 0;
  ns_curr = 0;      //no nectar
  ns_next = 0;
  lvs_curr = 0;     //last visited hive
  lvs_next = 0;

  vs = 0;  //seeing no POI

  state_opt0 = 0;
  ns_opt0 = 0;
  lvs_opt0 = 0;

  nsflag = 0;
  hbflag = 0;

  optflag = 1;  // option termination flag
  optT0 = 1;
  falseposflag = 0;

  reward = 0.0;

  opt = 0;
  act = 0;
  pass = 0;
  idec = 0;
  eps = 50;

  my_wp.z = NAV_DEFAULT_ALT;

  printf("init2: initialized variables \n");

  srand(time(NULL)); //initialize random number generator just once

  printf("init3: \n");

}

///////*  OWN FUCTION TO CALL FROM FLIGHT PLAN *//////
bool hrl_run(uint8_t wpa, uint8_t wpb){
  pass++;
  printf("pass %d:  ",pass);

  /* first pass?  init state is hive, and move wpb wrt p00 */
  if(pass==1){

    state_curr = 35; //init at hive
    drow = state_curr % ndim; // remainder = #increments to move in y from home
    dcol = (int)state_curr/ndim;  // rounded down = #increments to move in x from home
    my_wp.x = waypoint_get_x(WP_p00) + dcol*del;
    my_wp.y = waypoint_get_y(WP_p00) + drow*del;
    waypoint_set_enu(wpb, &my_wp);
    printf("first pass. del = %f\n",del);
  }
  else{

    printf("state = %d, lvs= %d,  ns= %d\n", state_curr, lvs_curr, ns_curr);
    printf("nr counter = [ %d, %d,  %d,  %d]\n", fnr_count[0], fnr_count[1], fnr_count[2], fnr_count[3]);

    //choose option if optflag
    if(optflag){
      opt = chooseopt(state_curr , lvs_curr , ns_curr  , eps);
      optT0 = 1;
      optflag = 0;
      idec++;

      printf("option = %d\n",opt);

      //remember initial state of option
      state_opt0 = state_curr;
      lvs_opt0 = lvs_curr;
      ns_opt0 = ns_curr;
      optr = 0;  //cumulated reward starts at 0

    }

    //choose primitive action based on option (hardcoded in subfunction for optionset A3d)
    act = primact(opt, optT0);
    printf("act = %d\n",act);

    // flag if option is terminated (hardcoded for optionset A3d)
    switch(opt){
      case 1 :  case 2 :  case 3 :  case 4 :
        if(optT0==2){optflag = 1;}
        break;
      case 5 :  case 6 :  case 7 :  case 8 :
        if(optT0==3){optflag = 1;}
        break;
      default :
        printf("warning: no valid option selected. Check for error here in hrl.c\n"); break;
    }


    // flag if action results in hiting the boundary
    hbflag = hitsbounds(state_curr, act);

    if(hbflag){
      printf("\n hitbound with act = %d; therefore stay still\n",act);
      act = 0;  // special action for not moving; (not really a chosen action, but needed for implemention in paparazzi.)
      optflag = 1;
    }


    //execute primitive action in paparazzi sim/IRL
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
      default: /* no movement */
        my_wp.z = waypoint_get_alt(WP_p00);
        my_wp.x = waypoint_get_x(wpa);
        my_wp.y = waypoint_get_y(wpa);
        waypoint_set_enu(wpb, &my_wp);
        printf("warning: no valid action selected\n");
        break;
    }




    //calculate next location state
    if(hbflag){
      state_next = state_curr;
    }
    else{
      switch(act){
        case 1: state_next = state_curr + 1; break;    //north
        case 2: state_next = state_curr + ndim; break; //east
        case 3: state_next = state_curr -1; break;     //south
        case 4: state_next = state_curr - ndim; break; //west
        default :
          printf("warning: no valid primitive action selected. Check for error here in hrl.c\n"); break;
      }  //switch act
    }  // calculate next location state


    // FOR flight test - use camera to detect flower
    if (!RLACT_NPS) {
      vs = detected_reward;
//vision version 1:  vs, 0= nothing seen, 1= flower seen, 2= hive seen
    }

    switch (state_next) {
      case 2:
      case 22:
      case 30:  // flowers
        if (RLACT_NPS) {
          vs = 1;
          printf("/in simulated flower state/ ");
        }  //for sim- define vision state based on location state
        if (vs == 0) {  //false negative
          ++falseneg[state_next];
          printf(
              "false negative: reward detection missed at flower in state: %d",
              state_next);
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



    // Env: calculate (t+1) reward and next xlv and xns states  BASED ON VISION 
    switch (vs) {
      case 1:  // flowers
        if(ns_curr<Nns && state_next==22 && fnr_count[1]>tfnr){
          //at flower 1 and there is nectar
          reward = -1;
          ns_next = ns_curr + 1;
          lvs_next = 1;

          fnr_count[1] = 0;
          optflag = 1;
          printf("F1 next state \n");
        }
        else if(ns_curr<Nns && state_next==2 && fnr_count[2]>tfnr){
          //at flower 2 and there is nectar
          reward = -1;
          ns_next = ns_curr + 1;
          lvs_next = 2;

          fnr_count[2] = 0;
          optflag = 1;
          printf("F2 next state \n");
        }
        else if(ns_curr<Nns && state_next==30 && fnr_count[3]>tfnr){
          //at flower 3 and there is nectar
          reward = -1;
          ns_next = ns_curr + 1;
          lvs_next = 3;

          fnr_count[3] = 0;
          optflag = 1;
          printf("F3 next state \n");
        }
        else {  //no nectar available even though color seen
              reward = -1;
              ns_next = ns_curr;
              lvs_next = lvs_curr;
        }
               break;

      case 2:  //hive
        if(ns_curr>0){
          //at hive and there is nectar to collect reward
          reward = rhive[ns_curr];  //hive reward function base 0
          ns_next = 0;
          lvs_next = 0;
          optflag = 1;
          printf("hive next state \n");
        }
        else{
          // If no nectar to bring to hive, no reward
          reward = -1;
          ns_next = ns_curr;
          lvs_next = lvs_curr;
        }
        break;

      default:  // POI not detected
        reward = -1;
        ns_next = ns_curr;
        lvs_next = lvs_curr;
        break;
    }  //switch vs

    if (falseposflag == 1) {
      ns_next = ns_curr;
    }  //if reward is false positive, don't go to next nectar state


    if(hbflag){
      // if wall hit then reward overwritten
      reward = -3;
    }

    optr = optr + reward;  // cumulated reward for option


    /* Now with reward and next state calculated:
   1) update Q value function for current state using belman eqn
   2) take action in paparazzi sim/IRL
   3) reset "next state" to "current state" for next iteration */

    // update Q
    // if option is terminal or flower/hive found
    if(optflag){
      Q_old = Q[state_opt0][lvs_opt0][ns_opt0][opt-1];
      ++kq[state_opt0][lvs_opt0][ns_opt0][opt-1];
      alpha =  0.3 ; //1.0/(double pow((double)k[state_opt0][lvs_opt0][ns_opt0][opt-1], double .25));
      Q[state_opt0][lvs_opt0][ns_opt0][opt-1] = Q_old + alpha*(reward + belgam* Q[state_next][lvs_next][ns_next][opt-1] - Q_old);
      printf("end option %d from state. [ %d, %d, %d ], reward = %.4f\n", opt, state_opt0, lvs_opt0, ns_opt0, optr);
      printf(" Qold = %.4f,  Qnew= %.4f\n", Q_old, Q[state_opt0][lvs_opt0][ns_opt0][opt-1]);
    }
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
        pass == 961 || pass == 971 || pass == 981 || pass == 991 || pass == 1001 )
    {

      // create a filename for file
      sprintf(filename_Qfcn, "%sQfcn%d.txt", RLACT_FILEPATH, (pass - 1));
      sprintf(filename_kq, "%sk%d.txt", RLACT_FILEPATH, (pass - 1));

      // open the file
      file_Qfcn = fopen(filename_Qfcn, "w");
      file_kq = fopen(filename_kq, "w");

      if (file_Qfcn == NULL) {
        printf(
            "Error! 'Qfcn.txt' NULL. No Value Function updates written to file.\n");
      }

      else {
        printf("Writing to files......\n");
        k = 1 ; n = 1 ;  //nectarstate 1, opt1
        for (n = 0; n < nopt; n++) {
          for (k = 0; k < Nns+1; k++) {
            for (i = 0; i < nlvs; i++) {
              for (j = 0; j < nstates; j++) {
                fprintf(file_Qfcn, "%.4f ", Q[j][i][k][n]);
                fprintf(file_kq, "%d ", kq[j][i][k][n]);

              }
            }
          }
        }  // end loops to print falsepos, Qfcn and kv in file


      }  //end security check

      //printf("bug check: print to file finished\n");

      fclose(file_Qfcn);
      fclose(file_kq);
      //printf("bug check: files closed\n");

    } // end print value function to file



    falseposflag = 0;

    // reset "next state" to "current state" for next iteration
    state_curr = state_next;
    ns_curr = ns_next;
    lvs_curr = lvs_next;

    optT0++;

    fnr_count[1]++;
    fnr_count[2]++;
    fnr_count[3]++;

  } //if not first pass




  return FALSE;

} // end of hrl_run function


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
int8_t hitsbounds(uint16_t state_curr_sf1, uint8_t act_sf1){
  drow = state_curr_sf1 % 6; //= #increments current state is from home in y
  dcol = (int)state_curr_sf1/6;  //= #increments current state is from home in x

  int sol = 0;
  switch(act_sf1){  //going north or south?
    case 1 : case 5 : case 6 :  //north, northwest, northeast
      if(drow>=(6-1)){sol =1;}
      break;
    case 3 : case 7 : case 8 : //south, southeast, southwest
      if(drow==0){sol =1;}
      break;
    default :
      break;
  } //end switch north-south

  switch(act_sf1){  //going east or west?
    case 2 : case 6 : case 7 : //east, northeast, southeast
      if(dcol>=(6-1)){sol =1;}
      break;
    case 4 : case 5 : case 8 : //west, northwest, southwest
      if(dcol==0){sol =1;}
      break;
    default :
      break;
  } //end switch east-west

  if(sol==1){return 1;}
  else{return 0;}
}

/********************* chooseopt subfunction *************/
/*** chooses an option between 1-8 to take.  */
/*** input: Q Value function, policy, current state, nectar state, history state*/
/*** output: option 1-Nopt */
int8_t chooseopt(uint16_t state_curr_sf2 , uint8_t lvs_curr_sf2 , uint8_t ns_curr_sf2 , uint16_t eps_sf2){

  int8_t index_sf2[8]={-1};  //for some reason I cannot declare this above.


  //initialize
  a = 0;

  //ep_greedy
  if(eps_sf2<(rand() % 100)){opt_sf2 = (rand() % nopt) +1;}  //random
  else{
    //greedy  - find max Q over options

    max = Q[state_curr_sf2][lvs_curr_sf2][ns_curr_sf2][0];
    for(a=1; a <nopt; a++){
      if(Q[state_curr_sf2][lvs_curr_sf2][ns_curr_sf2][a]>max){ max=Q[state_curr_sf2][lvs_curr_sf2][ns_curr_sf2][a]; }
    }
    //find all the elements that have max value
    i=0;
    for(a=0; a<nopt; a++){
      if(Q[state_curr_sf2][lvs_curr_sf2][ns_curr_sf2][a]==max){index_sf2[i] = a; i++; }
    }
    //there are i elements with the same max value.  Choose randomly between them
    a = rand() % i;
    opt_sf2 = index_sf2[a] + 1;

  }  //else if not randomly chosen, then greedy

  return opt_sf2;

}  // end chooseopt subfuction


/********************* primact subfunction *************/
/*** selects primitive action to take (using specific optionset A3d*/
/*** input: opt (option), optT0(option timestep)*/
/*** output: action 1-4, option termination flag */
int8_t primact(uint8_t opt_sf3, uint8_t optT0_sf3){


  switch(opt_sf3){
    case 1 :               //NE
      switch(optT0_sf3){
        case 1 : a_sf3 = 1; break;  //N
        case 2 : a_sf3 = 2; break;  //E
        default: printf("warning: check for error here in primact hrl.c"); break;
      }
      break;

    case 2 :               //SE
      switch(optT0_sf3){
        case 1 : a_sf3 = 2; break;  //E
        case 2 : a_sf3 = 3; break;  //S
        default: printf("warning: check for error here in primact hrl.c"); break;
          }
      break;

    case 3 :               //SW
      switch(optT0_sf3){
        case 1 : a_sf3 = 3; break;  //S
        case 2 : a_sf3 = 4; break;  //W
        default: printf("warning: check for error here in primact hrl.c"); break;
      }
      break;

    case 4 :               //NW
      switch(optT0_sf3){
        case 1 : a_sf3 = 4; break;  //W
        case 2 : a_sf3 = 1; break;  //N
        default: printf("warning: check for error here in primact hrl.c"); break;
      }
      break;

    case 5 :  a_sf3 = 1; break;  //Nx3
    case 6 :  a_sf3 = 2; break;  //Ex3
    case 7 :  a_sf3 = 3; break;  //Sx3
    case 8 :  a_sf3 = 4; break;  //Wx3
    default :
      printf("warning: check for error here in hrl.c");
      break;

  } //switch option number


  return a_sf3;

}  // end primact subfuction

