/*
 * PI Loop.cpp
 *
 *  Created on: Oct 1, 2019
 *      Author: stayo
 */
#include <PI_Loop.h>
#define MAX_ERROR
#define KI_FOC 10
#define KP_FOC 10
#define KI_SPEED 0.1
#define KP_SPEED 1.2
#define KD_SPEED 0.03
#define IQ_MAX 50
#define MAX_V 6000
static float interrorq=0;
static float interrord=0;
static float interrorspeed=0;
static float errorq_prev = 0;
static float errord_prev = 0;
static float errorspeed_prev = 0;
static float speed_diff = 0;
static uint32_t last_tic=0;
static uint32_t last_tic_speed=0;
int max_speed_int = IQ_MAX/KI_SPEED;

void update_foc_pi(float *target_iq, float *target_id,float *cur_iq,float *cur_id,int *Vq,int *Vd){
	float errorq = *target_iq-*cur_iq;
	float errord = *target_id-*cur_id;
	interrorq += errorq/20000;
	interrord += errord/20000;
    errorq_prev = errorq; //update previous error term
    errord_prev = errord; //update previous error term
	if(interrorq>200){
		interrorq=200;
	}
	else if(interrorq<-200){
		interrorq=-200;
	}
	if(interrord>150){
		interrord=-150;
	}
	else if(interrord<-150){
		interrord=-150;
	}
	//TODO: pwm saturates at 100% but drivers should not operate 100% so some protection must be added
	*Vq = (int)(interrorq*KI_FOC+errorq*KP_FOC);
	*Vd = (int)(interrord*KI_FOC+errord*KP_FOC);
	if(*Vq>MAX_V){
			*Vq = MAX_V;
		}
		else if(*Vq<-MAX_V){
				*Vq = -MAX_V;
		}
}
void update_speed_pi(float *target_speed, float *cur_speed,float *Iq){
	float error_speed = *target_speed-*cur_speed;
	float output,speed_error_der;
	last_tic_speed = HAL_GetTick();
	interrorspeed += error_speed/20000;
    float diff = (error_speed - errorspeed_prev)/20000;
    speed_diff = 0.3*speed_diff + 0.7*diff;
    errorspeed_prev = error_speed;
	if(interrorspeed>max_speed_int){
		interrorspeed=max_speed_int;
	}
	else if(interrorspeed<-max_speed_int){
		interrorspeed=-max_speed_int;
	}
	output = (interrorspeed*KI_SPEED+error_speed*KP_SPEED+speed_diff*KD_SPEED);
	if(output>IQ_MAX){
		*Iq = IQ_MAX;
	}
	else if(output<-IQ_MAX){
			*Iq = -IQ_MAX;
	}
	else{
		*Iq = output;
	}
}
/*
	Pi_loop(float maxval,float minval,float kp_in,float ki_in,int ticks_per_second_in){
		max_val = maxval;
		min_val = minval;
		kp = kp_in;
		ki = ki_in;
		ticks_per_second = ticks_per_second_in;
	}
	void target(float newtarget){
		target = newtarget;
	}
    float get_val(void){
    	return(curr_val);
    }
    void update_loop(float measured){
    	long time_delta=HAL_GetTick()-last_tick;
    	last_tick+=delta;
    	float error = target-measured;
    	error_intgral = error*time_delta/TICKS_PER_SECOND;
    	curr_val=error_intgral*kp+error*ki;
    	if(curr_val>max_val){
    		curr_val=max_val;
    	}
    	if(curr_val<min_val){
    		curr_val=min_val;
		}
    }
 */
