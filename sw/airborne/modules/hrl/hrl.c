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

#include "hrl.h"
#include "generated/airframe.h"
#include <time.h>
#include <stdlib.h>
#include <stdio.h>

#include "pprzlink/messages.h"
#include "mcu_periph/uart.h"
#include "subsystems/datalink/downlink.h"
#include "generated/flight_plan.h"  //needed to use WP_HOME

//*************** DECLARE VARIABLES *****************//
// environment and states for RL algorithm
int8_t drow, dcol;
int8_t state_curr, state_next;
int8_t ns_curr, ns_next;//nectar state 0-3 (base 0)
int8_t lvs_curr, lvs_next;//history state 0-3 (0 = hive, 1=F1, 2=F2, 3=F3)
const int8_t nns = 3;  //last nectar state (base 0)
const int8_t ndim = 6; //number of elements in a dimension = 6 (nrows,ncols, nns)
const int8_t nstates=36; //=ndim*ndim

// flags
int8_t nsflag;  //nectar state flag = nectar full
int8_t hbflag;      //hitbounds flag

// Bellman equation
    double Q_old, alpha;
    const double belgam = 0.9;  // gamma for belman equation
    static double Q[36][4][4]= {{0}}; // Q Value function initialized to zeros
    static int k[36][4][4]= {{0}}; // number of times a state has been visited
    static int ka[4]= {0};  //number of times an action has been taken, initialized as 0
  double reward;

// for execution of RL in paparazzi
struct EnuCoor_f my_wp;
const int16_t del = 1;// distance to move in each action
static int32_t pass;

// policy decisions - just random for now
const int8_t nact = 8; //number of actions possible (NESW + diagonals)
int8_t act; opt
int16_t eps;  //eps=0 is random, eps=100 is full greedy.

// subfunctions
//int8_t hitsbounds(uint16_t state_curr_sf1, uint8_t act_sf1)

//int8_t chooseact(uint16_t state_curr_sf2, uint8_t ns_curr_sf2 ,uint8_t nact_sf2, uint16_t eps_sf2)
int8_t a,i,act_sf2;
//int8_t index[8] = {-1};
double Vact[8], max;


//*********************** FUNCTIONS ***********************//
void hrl_init(void) {
//initialize variables
printf("init1\n");
  state_curr = 0;
  state_next = 0;
  ns_curr = 0;
  ns_next = 0;
  lvs_curr = 0;
  lvs_next = 0;

  nsflag = 0;
  hbflag = 0;

  reward = 0.0;

  opt=0;
  act=0;
  pass=0;
  eps=0;

  my_wp.z = NAV_DEFAULT_ALT;

printf("init2\n");
  srand(time(NULL)); //initialize random number generator just once
printf("init3\n");
}

///////*  OWN FUCTION TO CALL FROM FLIGHT PLAN *//////
bool rlact_run(uint8_t wpa, uint8_t wpb){
pass++;
// printf("pass = %d\n",pass);

// first pass?  choose initial state randomly, and move wpb wrt p00
if(pass==1){

  state_curr = rand() % nstates; //Random state between 0-35;
  drow = state_curr % ndim; // remainder = #increments to move in y from home
  dcol = (int)state_curr/ndim;  // rounded down = #increments to move in x from home
  my_wp.x = waypoint_get_x(WP_p00) + dcol*del;
  my_wp.y = waypoint_get_y(WP_p00) + drow*del;
  waypoint_set_enu(wpb, &my_wp);
    printf("first pass. del = %d\n",del);
  }
else{
  //choose initial option
  opt = chooseopt(state_curr, ns_curr,lvs_curr,eps);
  printf("state = %d, ns= %d, visit# %d, Qold= %.4f, ",state_curr, ns_curr, k[state_curr][ns_curr][lvs_curr],Q[state_curr][ns_curr][lvs_curr]);


  //choose primitive action based on option


  // give rewards for current state and calculate next state
  switch(state_curr){
  case 2 :  case 22 : case 30 : // flowers
    reward = 8.0;
    ns_next = ns_curr + 1;
    nsflag = 0;

      if(ns_next>nns){ //if full of nectar, no reward
        ns_next = nns;
        reward = 0.0;
        nsflag = 1;
      }
      // if action results in out-of-bounds, decrease reward, choose new action
      hbflag = hitsbounds(state_curr, act);
      while(hbflag){
        reward = reward - 1.0;
        act = chooseact(state_curr, ns_curr, eps);
        hbflag =  hitsbounds(state_curr, act);
      }

    // calculate next state
      // if random policy or last nectar state, execute actions
      // if ep-greedy then randomly generate anywhere on the board
      if(nsflag || eps==0){
        switch(act){
        case 1: state_next = state_curr + 1; break;    //north
        case 2: state_next = state_curr + ndim; break; //east
        case 3: state_next = state_curr -1; break;     //south
        case 4: state_next = state_curr - ndim; break; //west
        case 5: state_next = state_curr + 1 - ndim; break;  //northwest
        case 6: state_next = state_curr + 1 + ndim; break;  //northeast
        case 7: state_next = state_curr - 1 + ndim; break;  //southeast
        case 8: state_next = state_curr - 1 - ndim; break;  //southwest
      }
      }
      else {  //if ep-greedy (eps>0)
        state_next = rand() % nstates;
      }
    break;

  case 35 :      // if in hive state
    reward = 0.99*(double)ns_curr*(double)ns_curr;  //hive reward function base 0
    // reset at nectar state 0, and a random grid location.
    ns_next = 0;
    lvs_next = 0;
    state_next = rand() % nstates;
    act = 9; //special hive implementation for random regeneration
    break;

  default :         // not a reward space (code done for random policy)
  // if not in reward spot: check if boundary is hit, if so give negative reward and stay in same state, otherwise, calculate next state and give no reward.
    reward = 0.0;
    ns_next = ns_curr;
    lvs_next = lvs_curr;

    hbflag = hitsbounds(state_curr, act);
    if(hbflag){
      reward = -1.0;
      state_next = state_curr;
      printf("\n hitbound with act = %d; therefore stay still\n",act);
      act = 0;  // special action for not moving; (not really a chosen action, but needed for implemention in paparazzi.)
    }
    else{
      switch(act){
        case 1: state_next = state_curr + 1; break;    //north
        case 2: state_next = state_curr + ndim; break; //east
        case 3: state_next = state_curr -1; break;     //south
        case 4: state_next = state_curr - ndim; break; //west
      }
    }
    } // switch statement - reward function


/* Now with reward and next state calculated:
   1) update Q value function for current state using belman eqn
   2) take action in paparazzi sim/IRL
   3) reset "next state" to "current state" for next iteration */

    // update Value function
    Q_old = Q[state_curr][ns_curr][lvs_curr];
     ++k[state_curr][ns_curr];
     alpha = 1.0/(double)k[state_curr][ns_curr][lvs_curr];
    Q[state_curr][ns_curr][lvs_curr] = Q_old + alpha*(reward + belgam* Q[state_next][ns_next][lvs_next] - Q_old);
  printf(" Qnew= %.4f\n",Q[state_curr][ns_curr][lvs_curr]);

      //execute in paparazzi sim/IRL
  switch (act){
          case 0: /* no movement */
    my_wp.x = waypoint_get_x(wpa);
    my_wp.y = waypoint_get_y(wpa);
          waypoint_set_enu(wpb, &my_wp);
    printf("act = %d\n",act);
    break;
    case 1: /* north */
          my_wp.x = waypoint_get_x(wpa);
    my_wp.y = waypoint_get_y(wpa) + del;
    waypoint_set_enu(wpb, &my_wp);
         printf("act = %d\n",act);
    ++ka[0];
    break;
    case 2: /* east */
    my_wp.x = waypoint_get_x(wpa) + del;
    my_wp.y = waypoint_get_y(wpa);
    waypoint_set_enu(wpb, &my_wp);
         printf("act = %d\n",act);
    ++ka[1];
    break;
    case 3: /* south */
    my_wp.x = waypoint_get_x(wpa);
    my_wp.y = waypoint_get_y(wpa) - del;
    waypoint_set_enu(wpb, &my_wp);
         printf("act = %d\n",act);
    ++ka[2];
    break;
    case 4: /* west */
    my_wp.x = waypoint_get_x(wpa) - del;
    my_wp.y = waypoint_get_y(wpa);
    waypoint_set_enu(wpb, &my_wp);
         printf("act = %d\n",act);
    ++ka[3];
    break;
    case 9:  /* special hive/flower random regeneration */
    drow = state_next % ndim; // remainder = #increments to move in y from home
    dcol = (int)state_next/ndim;  // rounded down = #increments to move in x from home
    my_wp.x = waypoint_get_x(WP_p00) + dcol*del;
    my_wp.y = waypoint_get_y(WP_p00) + drow*del;
    waypoint_set_enu(wpb, &my_wp);
    printf("act = %d\n",act);
    break;
    default: /* no movement */
    my_wp.x = waypoint_get_x(wpa);
    my_wp.y = waypoint_get_y(wpa);
    waypoint_set_enu(wpb, &my_wp);
    state_next = state_curr; ns_next = ns_curr;
    printf("default action stay still: no valid action taken\n");
    break;
  }


  // reset "next state" to "current state" for next iteration
  state_curr = state_next;
  ns_curr = ns_next;

} //if not first pass

    return FALSE;
}  // end of rlact_run function


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
    } //end switch north-south

    switch(act_sf1){  //going east or west?
      case 2 : case 6 : case 7 : //east, northeast, southeast
      if(dcol>=(6-1)){sol =1;}
      break;
      case 4 : case 5 : case 8 : //west, northwest, southwest
      if(dcol==0){sol =1;}
      break;
    } //end switch east-west

    if(sol==1){return 1;}
    else{return 0;}
}

/********************* chooseact subfunction *************/
/*** chooses an option between 1-8 to take.  */
/*** input: Value function, policy, current state, nectar state*/
/*** output: option 1-8 */
int8_t chooseopt(uint16_t state_curr_sf2, uint8_t ns_curr_sf2 , uint16_t eps_sf2){
//int8_t a,i,act_sf2;
//double Vact[8], max;
//Declare up above instead of inside.  and give them different names so they are not shadowed?
// question: why don't I have to bring in Q[36][6], or nact, or ndim?
int8_t index_sf2[8]={-1};  //for some reason I cannot declare this above.


//initialize
a = 0;

if(eps==0){ //random
  act_sf2 = (rand() % nact) + 1;
}
else{ //ep_greedy or full greedy depending on eps value
  if(eps_sf2<(rand() % 100)){act_sf2 = (rand() % nact) +1;}
  else{
      //for each action fill in goodness value or NaN
    for(a= 1; a < nact+1; a++){
        hbflag = hitsbounds(state_curr_sf2, a);
      if(hbflag){
        Vact[(a-1)] = 0.0/0.0; //if hits bounds, not an option for movement
        }
      else{
        switch(a){
        case 1 : Vact[0] = Q[state_curr_sf2+1][ns_curr_sf2]; break;     //north
        case 2 : Vact[1] = Q[state_curr_sf2+ndim][ns_curr_sf2]; break;    //east
        case 3 : Vact[2] = Q[state_curr_sf2-1][ns_curr_sf2]; break;     //south
        case 4 : Vact[3] = Q[state_curr_sf2-ndim][ns_curr_sf2]; break;      //west
        case 5 : Vact[4] = Q[state_curr_sf2+1-ndim][ns_curr_sf2]; break;    //NW
        case 6 : Vact[5] = Q[state_curr_sf2+1+ndim][ns_curr_sf2]; break;    //NE
        case 7 : Vact[6] = Q[state_curr_sf2-1+ndim][ns_curr_sf2]; break;    //SE
        case 8 : Vact[7] = Q[state_curr_sf2-1-ndim][ns_curr_sf2]; break;    //SW
        }  //switch each state
      } //else
    } //for each action

//find max value of Vact
max = Vact[0];
for(a=1; a <nact; a++){
  if(isnan(max)){max = Vact[a]; /*printf("\nmax is nan, new max= %f\n",max);*/}
  else if(Vact[a]>max){ max=Vact[a]; }
}
//find all the elements that have max value
i=0;
for(a=0; a<nact; a++){
  if(Vact[a]==max){index_sf2[i] = a; i++; }
}
//there are i elements with the same max value.  Choose randomly between them
a = rand() % i;
act_sf2 = index_sf2[a] + 1;

}  //else if not randomly chosen, then greedy
} // if eps=0(random), else it is ep_greedy

return act_sf2;

}  // end chooseact subfuction

