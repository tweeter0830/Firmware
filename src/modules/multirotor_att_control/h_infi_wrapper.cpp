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

#include "multirotor_attitude_control.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <float.h>
#include <math.h>
#include <systemlib/pid/pid.h>
#include <systemlib/param/param.h>
#include <drivers/drv_hrt.h>

PARAM_DEFINE_FLOAT("MC_H_WEIGHT_R", 1.0f)
PARAM_DEFINE_FLOAT("MC_H_WEIGHT_P", 1.0f)
PARAM_DEFINE_FLOAT("MC_H_WEIGHT_I", 1.0f)
PARAM_DEFINE_FLOAT("MC_H_WEIGHT_U", 1.0f)

PARAM_DEFINE_FLOAT("MC_H_IXX", 1.0f);	   
PARAM_DEFINE_FLOAT("MC_H_IYY", 1.0f);	   
PARAM_DEFINE_FLOAT("MC_H_IZZ", 1.0f);	   

PARAM_DEFINE_FLOAT("MC_H_INT_MAX", 1.0f);

struct mc_att_control_h_infi_params {
	float w_rate;
	float w_position;
	float w_integral;
	float w_control;

	float Ixx;
	float Iyy;
	float Izz;

	float integral_max;
};

struct mc_att_control_h_infi_params_handles {
	param_t w_rate;
	param_t w_position;
	param_t w_integral;
	param_t w_control;

	param_t Ixx;
	param_t Iyy;
	param_t Izz;

	param_t integral_max;
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


static int parameters_init(struct mc_att_control_param_handles *h)
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

	return OK;
}

void h_infi_wrapper(
	const struct vehicle_attitude_setpoint_s	*att_sp,
	const struct vehicle_attitude_s			*att,
	const struct vehicle_rates_setpoint_s		*rates_sp,
	const struct vehicle_rates_s			*rates,
	bool						 control_pos,
	bool						 control_yaw, 
	bool						 reset_integral);
{
	static uint64_t last_run = 0;
	static uint64_t last_input = 0;
	float deltaT = (hrt_absolute_time() - last_run) / 1000000.0f;
	last_run = hrt_absolute_time();

	if (last_input != att_sp->timestamp) {
		last_input = att_sp->timestamp;
	}

	static int motor_skip_counter = 0;

	static PID_t pitch_controller;
	static PID_t roll_controller;

	static struct mc_att_control_h_infi_params p;
	static struct mc_att_control_h_infi_param_handles h;

	static bool initialized = false;

	static float yaw_error;

	/* initialize the pid controllers when the function is called for the first time */
	if (initialized == false) {
		parameters_init(&h);
		parameters_update(&h, &p);

		pid_init(&pitch_controller, p.att_p, p.att_i, p.att_d, 1000.0f, 1000.0f, PID_MODE_DERIVATIV_SET, 0.0f);
		pid_init(&roll_controller, p.att_p, p.att_i, p.att_d, 1000.0f, 1000.0f, PID_MODE_DERIVATIV_SET, 0.0f);

		initialized = true;
	}

	/* load new parameters with lower rate */
	if (motor_skip_counter % 500 == 0) {
		/* update parameters from storage */
		parameters_update(&h, &p);

		/* apply parameters */
		pid_set_parameters(&pitch_controller, p.att_p, p.att_i, p.att_d, 1000.0f, 1000.0f);
		pid_set_parameters(&roll_controller, p.att_p, p.att_i, p.att_d, 1000.0f, 1000.0f);
	}

	/* reset integrals if needed */
	if (reset_integral) {
		pid_reset_integral(&pitch_controller);
		pid_reset_integral(&roll_controller);
		//TODO pid_reset_integral(&yaw_controller);
	}

	/* calculate current control outputs */

	/* control pitch (forward) output */
	rates_sp->pitch = pid_calculate(&pitch_controller, att_sp->pitch_body ,
					att->pitch, att->pitchspeed, deltaT);

	/* control roll (left/right) output */
	rates_sp->roll = pid_calculate(&roll_controller, att_sp->roll_body ,
				       att->roll, att->rollspeed, deltaT);

	if (control_yaw_position) {
		/* control yaw rate */
		// TODO use pid lib

		/* positive error: rotate to right, negative error, rotate to left (NED frame) */
		// yaw_error = _wrap_pi(att_sp->yaw_body - att->yaw);

		yaw_error = att_sp->yaw_body - att->yaw;

		if (yaw_error > M_PI_F) {
			yaw_error -= M_TWOPI_F;

		} else if (yaw_error < -M_PI_F) {
			yaw_error += M_TWOPI_F;
		}

		rates_sp->yaw = p.yaw_p * (yaw_error) - (p.yaw_d * att->yawspeed);
	}

	rates_sp->thrust = att_sp->thrust;
    //need to update the timestamp now that we've touched rates_sp
    rates_sp->timestamp = hrt_absolute_time();

	motor_skip_counter++;
}
