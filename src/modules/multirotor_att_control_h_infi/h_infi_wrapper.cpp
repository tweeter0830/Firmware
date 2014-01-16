/****************************************************************************
 *
 *   Copyright (C) 2008-2012 PX4 Development Team. All rights reserved.
 *   Author: Thomas Gubler <thomasgubler@student.ethz.ch>
 *           Julian Oes <joes@student.ethz.ch>
 *           Laurens Mackay <mackayl@student.ethz.ch>
 *           Tobias Naegeli <naegelit@student.ethz.ch>
 *           Martin Rutschmann <rutmarti@student.ethz.ch>
 *           Lorenz Meier <lm@inf.ethz.ch>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/*
 * @file multirotor_attitude_control.c
 *
 * Implementation of attitude controller for multirotors.
 *
 * @author Thomas Gubler <thomasgubler@student.ethz.ch>
 * @author Julian Oes <joes@student.ethz.ch>
 * @author Laurens Mackay <mackayl@student.ethz.ch>
 * @author Tobias Naegeli <naegelit@student.ethz.ch>
 * @author Martin Rutschmann <rutmarti@student.ethz.ch>
 * @author Lorenz Meier <lm@inf.ethz.ch>
 */


#include <stdio.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <float.h>
#include <math.h>
#include <systemlib/pid/pid.h>
#include <systemlib/param/param.h>
#include <systemlib/err.h>
#include <drivers/drv_hrt.h>

#include "h_infi_wrapper.hpp"
#include "multirotor_attitude_control_h_infi.hpp"
#include "body_torque_conversion.h"

#include <uORB/uORB.h>
//#include <uORB/topics/vehicle_torque_command.h>

#define H_INFI_WRAPPER_DEBUG 

#ifdef H_INFI_WRAPPER_DEBUG
#include <uORB/topics/debug_key_value.h>
static struct debug_key_value_s dbg_tr;
static struct debug_key_value_s dbg_t_r;
static struct debug_key_value_s dbg_t_p;
static struct debug_key_value_s dbg_t_y;
static orb_advert_t pub_dbg_tr;
#endif

//ORB_DEFINE(vehicle_torque_command, struct vehicle_torque_command_s);
//static struct vehicle_torque_command_s torque_comm;
//static orb_advert_t pub_torque_comm;

static Multirotor_Attitude_Control_H_Infi h_infi_controller;

struct mc_att_control_h_infi_params {
	float w_rate;
	float w_position;
	float w_integral;
	float w_control;

	float Ixx;
	float Iyy;
	float Izz;
	/* XX Implement this */
	float integral_max;

	float arm_length;
	float forward_ang;
	float torque_fract;
};

struct mc_att_control_h_infi_param_handles {
	param_t w_rate;
	param_t w_position;
	param_t w_integral;
	param_t w_control;

	param_t Ixx;
	param_t Iyy;
	param_t Izz;

	param_t integral_max;

	param_t arm_length;
	param_t forward_ang;
	param_t torque_fract;
};

/**
 * Initialize all parameter handles and values
 *
 */
static int parameters_init(struct mc_att_control_h_infi_param_handles *h);

/**
 * Update all parameters
 *
 */
static int parameters_update(const struct mc_att_control_h_infi_param_handles *h, struct mc_att_control_h_infi_params *p);


static int parameters_init(struct mc_att_control_h_infi_param_handles *h)
{
	/* Control Weight Parameters */
	h->w_rate 	=	param_find("MC_H_WEIGHT_R");
	h->w_position 	=	param_find("MC_H_WEIGHT_P");
	h->w_integral 	=	param_find("MC_H_WEIGHT_I");
	h->w_control    =       param_find("MC_H_WEIGHT_U");
	/* Moment of inertia parameters */                 
	h->Ixx  	= 	param_find("MC_H_IXX");	   
	h->Iyy  	= 	param_find("MC_H_IYY");	   
	h->Izz  	= 	param_find("MC_H_IZZ");	   
	/* Integral parameters */	                   
	h->integral_max =	param_find("MC_H_INT_MAX");

	/* torque to pwm parameters */
	h->arm_length  =        param_find("MC_T_ARM_LENGTH");
	h->forward_ang =        param_find("MC_T_FOR_ANGLE");
	h->torque_fract=        param_find("MC_T_TORQUE_FRACT");
	return OK;
}

static int parameters_update(const struct mc_att_control_h_infi_param_handles *h, struct mc_att_control_h_infi_params *p)
{
	param_get(h->w_rate, &(p->w_rate));
	param_get(h->w_position, &(p->w_position));
	param_get(h->w_integral, &(p->w_integral));
	param_get(h->w_control, &(p->w_control));

	param_get(h->Ixx, &(p->Ixx));
	param_get(h->Iyy, &(p->Iyy));
	param_get(h->Izz, &(p->Izz));

	param_get(h->integral_max, &(p->integral_max));

	param_get(h->arm_length  , &(p->arm_length));
	param_get(h->forward_ang , &(p->forward_ang));
	param_get(h->torque_fract, &(p->torque_fract));
	
	return OK;
}

void h_infi_wrapper(
	const struct vehicle_attitude_setpoint_s	*att_sp,
	const struct vehicle_attitude_s			*att,
	const struct vehicle_rates_setpoint_s		*rates_sp,
	struct actuator_controls_s                      *actuators,
	bool						 control_pos,
	bool						 control_yaw_pos, 
	bool						 reset_integral)
{
	static uint64_t last_run = 0;
	static uint64_t last_input = 0;
	last_run = hrt_absolute_time();

	if (last_input != att_sp->timestamp) {
		last_input = att_sp->timestamp;
	}

	static int motor_skip_counter = 0;
	
	static struct mc_att_control_h_infi_params p;
	static struct mc_att_control_h_infi_param_handles h;

	static bool initialized = false;
	/* initialize the controller when the function is called for the first time */
	if (initialized == false) {
		parameters_init(&h);
		parameters_update(&h, &p);

		h_infi_controller.set_phys_params(p.Ixx, p.Iyy, p.Izz);
		h_infi_controller.set_weights(p.w_rate, p.w_position, p.w_integral, p.w_control);

#ifdef H_INFI_WRAPPER_DEBUG
		strcpy(dbg_tr.key,  "Tr_com");
		strcpy(dbg_t_r.key, "T_r");
		strcpy(dbg_t_p.key,  "T_p");
		strcpy(dbg_t_y.key,  "T_y");

		dbg_tr.value = 0.0f;
		dbg_t_r.value = 0.0f;
		dbg_t_p.value = 0.0f;
		dbg_t_y.value = 0.0f;

		pub_dbg_tr = orb_advertise(ORB_ID(debug_key_value), &dbg_tr);
#endif
		// torque_comm.roll = 0.0f;
		// torque_comm.pitch= 0.0f;
		// torque_comm.yaw  = 0.0f;
		// strcpy(torque_comm.key, "Tor_comm");
		// pub_torque_comm = orb_advertise(ORB_ID(vehicle_torque_command), &torque_comm);

		initialized = true;
	}
	bool updated = false;
	/* load new parameters with lower rate */
	if (motor_skip_counter % 500 == 0) {
		/* update parameters from storage */
		parameters_update(&h, &p);
		
		h_infi_controller.set_phys_params(p.Ixx, p.Iyy, p.Izz);
		h_infi_controller.set_weights(p.w_rate, p.w_position, p.w_integral, p.w_control);
		updated = true;
	}

	/* reset integrals if needed */
	if (reset_integral) {
		h_infi_controller.reset_integrator();
	}
	Multirotor_Attitude_Control_H_Infi::State 
		meas_state, 
		meas_rate,
		torque_out,
		set_state,
		set_rate,
		set_accel;
	meas_state.r = att->roll;
	meas_state.p = att->pitch;
	meas_state.y = att->yaw;
	meas_rate.r = att->rollspeed;
	meas_rate.p = att->pitchspeed;
	meas_rate.y = att->yawspeed;
	set_state.r = att_sp->roll_body;
	set_state.p = att_sp->pitch_body;
	set_state.y = att_sp->yaw_body;
	set_rate.r = rates_sp->roll;
	set_rate.p = rates_sp->pitch;
	set_rate.y = rates_sp->yaw;
	set_accel.r = 0.0f;
	set_accel.p = 0.0f;
	set_accel.y = 0.0f;
	/* calculate current control outputs */
	hrt_abstime start_time = 0;

	h_infi_controller.set_setpoints(set_state,set_rate,set_accel);
	h_infi_controller.set_mode(control_pos, true, true, control_yaw_pos);
	h_infi_controller.control(meas_state, meas_rate, torque_out, last_run/1000000.0f);

	// torque_comm.roll = torque_out.r;
	// torque_comm.pitch= torque_out.p;
	// torque_comm.yaw  = torque_out.y;
	
	// orb_publish(ORB_ID(vehicle_torque_command), pub_torque_comm , &torque_comm);
#ifdef H_INFI_WRAPPER_DEBUG 
	dbg_tr.value = att_sp->thrust;
	dbg_t_r.value = torque_out.r;
	dbg_t_p.value = torque_out.p;
	dbg_t_y.value = torque_out.p;
	orb_publish(ORB_ID(debug_key_value), pub_dbg_tr, &dbg_tr);
	orb_publish(ORB_ID(debug_key_value), pub_dbg_tr, &dbg_t_r);
	orb_publish(ORB_ID(debug_key_value), pub_dbg_tr, &dbg_t_p);
	orb_publish(ORB_ID(debug_key_value), pub_dbg_tr, &dbg_t_y);
#endif
	struct body_torque_params body_t_p;
	body_t_p.arm_length   = p.arm_length;
	body_t_p.forward_ang  = p.forward_ang;
	body_t_p.torque_fract = p.torque_fract;

	struct body_torque body_t;
	body_t.r  = torque_out.r;
	body_t.p  = torque_out.p;
	body_t.y  = torque_out.y;

	float pwm_fract[4];
        body_torque_to_pwm( &body_t,
			    &body_t_p,
			    att_sp->thrust,
			    updated,
			    pwm_fract);

	actuators->control[0] = pwm_fract[0];
	actuators->control[1] = pwm_fract[1];
	actuators->control[2] = pwm_fract[2];
	actuators->control[3] = pwm_fract[3];

	motor_skip_counter++;
}
