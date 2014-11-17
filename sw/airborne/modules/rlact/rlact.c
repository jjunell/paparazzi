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

#include "messages.h"
#include "mcu_periph/uart.h"
#include "subsystems/datalink/downlink.h"
#include "generated/flight_plan.h"  //needed to use WP_HOME

void send_rlact(void) {
  DOWNLINK_SEND_RLACT(DefaultChannel, DefaultDevice,
                                            &xnew);
}


//*************** DECLARE VARIABLES *****************//
int32_t xnew; //declare variable

// environment and states for RL algorithm
int8_t drow, dcol;
int16_t state_curr = 0;
int16_t state_next = 0;
int8_t ns_curr = 0; //current nectar state 0-5 (base 0)
int8_t ns_next = 0;
const int8_t nns = 5;  //last nectar state (base 0)
const int8_t ndim = 6; //number of elements in a dimension = 6 (nrows,ncols, nns)
const int nstates=36; //=ndim*ndim
double reward;

// flags
int8_t nsflag = 0;  //nectar state flag = nectar full
int8_t hbflag;      //hitbounds flag


// Bellman equation
    double V_old, alphav;
    const double belgam = 0.9;  // gamma for belman equation
    static double V[36][6] = {{0}}; // Value function initialized to zeros
    static int kv[36][6] = {{0}}; // number of times a state has been visited
    static int ka[8]={0};  //number of times an action has been taken, initialized as 0
   // with ka declared as 'int', as long as the GCS is not stopped, the value will not reinitialize
  // with ka declared as 'static int', same thing.  GCS and server processes can be stopped, but stopping the simulator process, will cause the ka variable to be reinitialized.  

// Storing value function:  Value function, V, is stored in V[36][6] for algorithm purposes.  But each iteration the value function and kv matrices will be written to a file so as to be saved and not as easily rewritten.
    //static FILE* kv_store;
//    FILE *file_Vfcn, *file_kv;
//    FILE *file_reg, *file_regw, *file_actw, *file_act;
    int16_t i, j, k;
    
// for execution of RL in paparazzi
const int del = 128;// distance to move in each action
static int16_t pass = 0;

// policy decisions - just random for now
int16_t eps = 0;  // eps contains information on policy from random(eps = 0) to full greedy(eps=100).  If eps is between 1 and 99, it is ep_greedy.
const int8_t nact = 8; //number of actions = 4 (NESW)
uint8_t act;


//*********************** FUNCTIONS ***********************//
void rlact_init(void) {
	xnew = 5; //initialize it
	srand(time(NULL));
  //  const int nact = 4; //number of actions = 4 (NESW)

}

///////*  OWN FUCTION TO CALL FROM FLIGHT PLAN *//////
bool_t rlact_run(uint8_t wpa, uint8_t wpb){

pass++;

// first pass?  choose initial state randomly, and move wpb wrt home
if(pass==1){
	srand(time(NULL));  //initialize random number generator just once
	
	state_curr = rand() % nstates; //Random state between 0-35;
	drow = state_curr % ndim; // remainder = #increments to move in y from home
	dcol = (int)state_curr/ndim;  // rounded down = #increments to move in x from home
	waypoints[wpb].x = waypoints[WP_HOME].x + dcol*del;
	waypoints[wpb].y = waypoints[WP_HOME].y + drow*del;
	
	// Just in case file has left over numbers from last time, clear file
//	file_actw = fopen("./sw/airborne/modules/rlact/actions_rand.txt","w");
//	fclose(file_actw);
//	file_regw = fopen("./sw/airborne/modules/rlact/regen_rand.txt","w");
//	fclose(file_regw);
	
	//reopen regeneration file and save the first random generation state
//	file_reg = fopen("./sw/airborne/modules/rlact/regen_rand.txt","a");
//	fprintf(file_reg,"%d ", state_curr);
//	fclose(file_reg);
}
else{
	//choose initial action and print to appendable file
	act = chooseact(state_curr, ns_curr, nact, eps);
//	file_act = fopen("./sw/airborne/modules/rlact/actions_rand.txt","a");
//	fprintf(file_act,"%d ", act);
	
//	file_reg = fopen("./sw/airborne/modules/rlact/regen_rand.txt","a");

	// give rewards for current state and calculate next state
	switch(state_curr){
	case 2 :  case 22 : case 30 : // flowers  (code done for random policy)
		reward = 8.0;
		ns_next = ns_curr + 1;
		nsflag = 0;
		
			if(ns_next>nns){ //if bee is already full of nectar, no reward
				ns_next = nns;
				reward = 0.0;
				nsflag = 1;
			}
			// if action results in out-of-bounds, decrease reward, choose new action
			hbflag = hitsbounds(state_curr, act);
			while(hbflag){
				reward = reward - 1.0;
				act = chooseact(state_curr, ns_curr, nact, eps);
//				fprintf(file_act,"%d ", act);
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
//		  	fprintf(file_reg,"%d ", state_next);
		  }
		break;
		
	case 35 :      // if in hive state  
		reward = 0.99*(double)ns_curr*(double)ns_curr;  //hive reward function base 0
		// reset at nectar state 0, and a random grid location.
		ns_next = 0;
		state_next = rand() % nstates;
//		fprintf(file_reg,"%d ", state_next);
		act = 9; //special hive implementation for random regeneration
		break;
		
	default :         // not a reward space (code done for random policy)
	// if not in reward spot: check if boundary is hit, if so give negative reward and stay in same state, otherwise, calculate next state and give no reward.
		reward = 0.0;
		ns_next = ns_curr;
		
		hbflag = hitsbounds(state_curr, act);
		if(hbflag){
			reward = -1.0;
			state_next = state_curr;
			act = 0;  // special action for not moving; (not really a chosen action, but needed for implemention in paparazzi.)
		}
		else{
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
    } // switch statement - reward function
    
/* Now with reward and next state calculated:
   1) update value function for current state using belman eqn
   2) take action in paparazzi sim/IRL
   3) reset "next state" to "current state" for next iteration */

    // update Value function
    printf("At grid spot %d, nectar state %d. visited %d times\n",state_curr,ns_curr,kv[state_curr][ns_curr]);
    V_old = V[state_curr][ns_curr];
     ++kv[state_curr][ns_curr];   
     alphav = 1.0/(double)kv[state_curr][ns_curr];
    V[state_curr][ns_curr] = V_old + alphav*(reward + belgam* V[state_next][ns_next] - V_old);
printf("alphav = %.4f,  old V = %.4f, new V = %.4f \n", alphav, V_old, V[state_curr][ns_curr]);

/*/ update value function file
if(pass==11 || pass==101 || pass==201 || pass==251 || pass==1001 || pass==5001){
switch(pass){
	case 11: file_Vfcn = fopen("./sw/airborne/modules/rlact/Vfcn10.txt","w");
		file_kv = fopen("./sw/airborne/modules/rlact/kv10.txt","w"); break;
		
	case 101: file_Vfcn = fopen("./sw/airborne/modules/rlact/Vfcn100.txt","w");
		file_kv = fopen("./sw/airborne/modules/rlact/kv100.txt","w"); break;
	
	case 201: file_Vfcn = fopen("./sw/airborne/modules/rlact/Vfcn200.txt","w");
		file_kv = fopen("./sw/airborne/modules/rlact/kv200.txt","w"); break;
		
	case 251: file_Vfcn = fopen("./sw/airborne/modules/rlact/Vfcn250.txt","w");
		file_kv = fopen("./sw/airborne/modules/rlact/kv250.txt","w"); break;
				
	case 1001: file_Vfcn = fopen("./sw/airborne/modules/rlact/Vfcn1000.txt","w"); 
		file_kv = fopen("./sw/airborne/modules/rlact/kv1000.txt","w"); break;
		
    case 5001: file_Vfcn = fopen("./sw/airborne/modules/rlact/Vfcn5000.txt","w");
    	file_kv = fopen("./sw/airborne/modules/rlact/kv5000.txt","w"); break;
  }
    if(file_Vfcn==NULL){printf("Error! 'Vfcn.txt' NULL. No Value Function updates written to file.\n");}
    else{
    for(i=0;i<ndim;i++){
    	for(j=0;j<nstates;j++){ 
    	fprintf(file_Vfcn, "%.10f ", V[j][i]);
    	fprintf(file_kv, "%d ", kv[j][i]);
    	}}// end loops to print Vfcn and kv in file


    } //end security check
        fclose(file_Vfcn);
        fclose(file_kv);
   } //end update value function */
    
    	//execute in paparazzi sim/IRL
	switch (act){
	    case 0: /* no movement */
	    waypoints[wpb].x = waypoints[wpa].x;    
		waypoints[wpb].y = waypoints[wpa].y;
		break;
		case 1: /* north */
		waypoints[wpb].x = waypoints[wpa].x;    
		waypoints[wpb].y = waypoints[wpa].y + del;
		++ka[0];
		break;
		case 2: /* east */
		waypoints[wpb].x = waypoints[wpa].x + del;
		waypoints[wpb].y = waypoints[wpa].y;
		++ka[1];
		break;
		case 3: /* south */
		waypoints[wpb].x = waypoints[wpa].x;
		waypoints[wpb].y = waypoints[wpa].y - del;
		++ka[2];
		break;      
		case 4: /* west */
		waypoints[wpb].x = waypoints[wpa].x - del;
		waypoints[wpb].y = waypoints[wpa].y;
		++ka[3];
		break;    
		case 5: /* northwest */
		waypoints[wpb].x = waypoints[wpa].x - del;    
		waypoints[wpb].y = waypoints[wpa].y + del;
		++ka[4];
		break;
		case 6: /* northeast */
		waypoints[wpb].x = waypoints[wpa].x + del;
		waypoints[wpb].y = waypoints[wpa].y + del;
		++ka[5];
		break;
		case 7: /* southeast */
		waypoints[wpb].x = waypoints[wpa].x + del;
		waypoints[wpb].y = waypoints[wpa].y - del;
		++ka[6];
		break;      
		case 8: /* southwest */
		waypoints[wpb].x = waypoints[wpa].x - del;
		waypoints[wpb].y = waypoints[wpa].y - del;
		++ka[7];
		break;
		case 9:  /* special hive random regeneration */
		drow = state_next % ndim; // remainder = #increments to move in y from home
		dcol = (int)state_next/ndim;  // rounded down = #increments to move in x from home
		waypoints[wpb].x = waypoints[WP_HOME].x + dcol*del;
		waypoints[wpb].y = waypoints[WP_HOME].y + drow*del;
		break;
		default: /* no movement */
	    waypoints[wpb].x = waypoints[wpa].x;
		waypoints[wpb].y = waypoints[wpa].y;
		state_next = state_curr; ns_next = ns_curr;
		printf("Error: no valid action taken\n");
		break;
	}
	
	
  // reset "next state" to "current state" for next iteration
  state_curr = state_next;
  ns_curr = ns_next;	
  
  /*/close all open files
  fclose(file_act);
  fclose(file_reg);*/

} //if not first pass

		return FALSE;
}  // end of rlact_run function

void rlact_periodic(void) {
if(RLbool){xnew = rand(); send_rlact();}
}

/********************* hitsbounds subfunction *************/
/*** determines if boundary is hit within a 6x6 grid environment */
/*** inputs: current state, action *** output: 1/0 integer */
int8_t hitsbounds(uint16_t state_curr, uint8_t act){
    drow = state_curr % 6; //= #increments current state is from home in y
	dcol = (int)state_curr/6;  //= #increments current state is from home in x
    
    int sol = 0;
    switch(act){  //going north or south?
    	case 1 : case 5 : case 6 :  //north, northwest, northeast
    	if(drow>=(6-1)){sol =1;}
    	break;
    	case 3 : case 7 : case 8 : //south, southeast, southwest
    	if(drow==0){sol =1;}
    	break;
    } //end switch north-south

    switch(act){  //going east or west?
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
/*** chooses an action between 1-8 to take.  */
/*** input: Value function, policy, current state, nectar state*/
/*** output: action 1-8 (North, east, south, west, NW, NE, SE, SW) */
int8_t chooseact(uint16_t state_curr, uint8_t ns_curr ,uint8_t nact, uint16_t eps){
int8_t a,i,act;
int8_t index[8]={-1};
double Vact[8], max;


if(eps==0){ //random
	act = (rand() % nact) + 1; 
}
else{ //ep_greedy or full greedy depending on eps value
	if(eps<(rand() % 100)){act = (rand() % nact) +1;}
	else{
			//for each action fill in goodness value or NaN
		for(a= 0; a < nact; a++){
  			hbflag = hitsbounds(state_curr, a);
			if(hbflag){
				Vact[a] = 0.0/0.0; //if hits bounds, not an option for movement
				} 
			else{
				switch(a){
				case 1 : Vact[a] = V[state_curr+1][ns_curr]; break;			//north
				case 2 : Vact[a] = V[state_curr+ndim][ns_curr]; break;		//east
				case 3 : Vact[a] = V[state_curr-1][ns_curr]; break;			//south
				case 4 : Vact[a] = V[state_curr-ndim][ns_curr]; break;  		//west
				case 5 : Vact[a] = V[state_curr+1-ndim][ns_curr]; break;		//NW
				case 6 : Vact[a] = V[state_curr+1+ndim][ns_curr]; break;		//NE
				case 7 : Vact[a] = V[state_curr-1+ndim][ns_curr]; break;		//SE
				case 8 : Vact[a] = V[state_curr-1-ndim][ns_curr]; break;		//SW
				}  //switch each state  
			} //else
		} //for each action

//find max value of Vact
max = Vact[0];
for(a=1; a <nact; a++){
	if(Vact[a]>max){ max=Vact[a]; }
}
//find all the elements that have max value
i=0;
for(a=0; a<nact; a++){
	if(Vact[a]==max){index[i] = a; i++; }
}
//there are i elements with the same max value.  Choose randomly between them
a = rand() % i;
act = index[a];

}  //else if not randomly chosen, then greedy
} // if eps=0(random), else it is ep_greedy

return act;

}  // end chooseact subfuction

