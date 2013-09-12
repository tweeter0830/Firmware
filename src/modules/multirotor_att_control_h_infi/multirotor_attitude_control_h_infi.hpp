/****************************************************************************
 *
 *   Copyright (c) 2013 Estimation and Control Library (ECL). All rights reserved.
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
 * 3. Neither the name APL nor the names of its contributors may be
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

/**
 * @file multirotor_attitude_control_h_infi.h
 * Definition of a simple orthogonal roll PID controller.
 *
 */

#ifndef MULTIROTOR_ATTITUDE_CONTROL_H_INFI_H
#define MULTIROTOR_ATTITUDE_CONTROL_H_INFI_H

#include <stdbool.h>
#include <stdint.h>

#include <mathlib/mathlib.h>

class Multirotor_Attitude_Control_H_Infi{ //__EXPORT
public:
	Multirotor_Attitude_Control_H_Infi();
        
	typedef struct State
	{
		//Body angles in radians
		float r;
		float p;
		float y;
       	};

	float control();

	void reset_integrator();

	void set_time_constant(float time_constant) {
		if (time_constant > 0.1f && time_constant < 3.0f) {
			_tc = time_constant;
		}
	}
	void set_k_p(float k_p) {
		//_k_p = k_p;
	}
	void set_k_i(float k_i) {
		//_k_i = k_i;
	}
	void set_k_d(float k_d) {
		//_k_d = k_d;
	}
	void set_integrator_max(float max) {
		//_integrator_max = max;
	}
	void set_max_rate(float max_rate) {
		//_max_rate = max_rate;
	}

	float get_rate_error() {
		//return _rate_error;
	}

	float get_desired_rate() {
		//return _rate_setpoint;
	}

private:
	uint64_t _last_run;
	float _tc;
	float _weight_error_deriv;
	float _weight_error;
	float _weight_error_integral;
	float _weight_torque;
	float _Ixx;
	float _Iyy;
	float _Izz;
	State _setpoint;
	State _setpoint_rate;
	State _setpoint_accel;
	float _command_torque [3];
	
	math::Vector _integral;
	math::Matrix _M;
	math::Matrix _M_inv;
	math::Matrix _C;
	
	void calc_gains(const math::Matrix& M,const math::Matrix& C, float* k_p, float* k_i, float* k_d);
	void make_M(const State& St, math::Matrix& M);
	void make_C(const State& St, const State& Rate, math::Matrix& C);
};

#endif // MULTIROTOR_ATTITUDE_CONTROL_H_INFI_H
