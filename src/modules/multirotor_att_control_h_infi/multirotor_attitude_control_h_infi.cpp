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
 * @file ecl_roll_controller.cpp
 * Implementation of a simple orthogonal roll PID controller.
 *
 */

#include "../ecl.h"
#include "multirotor_attitude_control_h_infi.hpp"
#include <stdint.h>
#include <math.h>
#include <float.h>
#include <geo/geo.h>
#include <ecl/ecl.h>
#include <mathlib/mathlib.h>

Multirotor_Attitude_Control_H_Infi::Multirotor_Attitude_Control_H_Infi() :
	_last_run(0),
	_tc(0.1f),
	_last_output(0.0f),
	_integrator(0.0f),
	_rate_error(0.0f),
	_rate_setpoint(0.0f),
	_max_deflection_rad(math::radians(45.0f))
{

}

float Multirotor_Attitude_Control_H_Infi::control()
{
}

void Multirotor_Attitude_Control_H_Infi::reset_integrator()
{
}

void Multirotor_Attitude_Control_H_Infi::calc_gains(const math::Matrix& M,const math::Matrix& C, float* k_p, float* k_i, float* k_d) {
	const static float I_vals[][] =  {{1,0,0},{0,1,0},{0,0,1}};
	const static math::Matrix I(3,3,I_vals);
	const math::Matrix Dynamics_weights = M.inverse();//*(C+f1.0/_weight_torque*I);
	const float long_expr = sqrt(_weight_error*_weight_error + f2.0*_weight_error_deriv*_weight_error_integral)/weight_error;

	*k_d=long_expr*I+Dynamics_weights;
	*k_p=_weight_error_integral/_weight_error_deriv*I+long_expr*Dynamics_weights;
	*k_i=_weight_error_integral/_weight_error_deriv*Dynamics_weights;
}

void Multirotor_Attitude_Control_H_Infi::make_M(const State& St, math::Matrix& M) {
	float M_vals [3][3] = {0};
	float sin_R=sin(St.r);
	float cos_R=cos(St.r);
	float sin_P=sin(St.p);
	float cos_P=cos(St.p);
	//First Row
	M_vals[0][0]=_Ixx;
	M_vals[0][2]=-_Ixx*sin_P;
	// Second Row
	M_vals[1][1]=_Iyy*cos_R*cos_R+_Izz*sin_R*sin_R;
	M_vals[1][2]=(_Iyy-_Izz)*cos_R*sin_R*sin_P;
	// Third row
	M_vals[2][0]=-_Ixx*sin_P;
	M_vals[2][1]=(_Iyy-_Izz)*cos_R*sin_R*cos_P;
	M_vals[2][2]=_Ixx*sin_P*sin_P + _Iyy*sin_R*sin_R*cos_P*cos_P +
		     _Izz*cos_R*cos_R*cos_P*cos_P;

	M = math::Matrix(3,3,M_vals);
}

void Multirotor_Attitude_Control_H_Infi::make_C(const State& St, const State& Rate, math::Matrix& C) {
	float C_vals [3][3] = {0};
	float sin_R=sin(St.r);
	float cos_R=cos(St.r);
	float sin_P=sin(St.p);
	float cos_P=cos(St.p);
	float long_factor = Rate.p*cos_R*sin_R + Rate.y*sin_R*sin_R*cos_P;
	//First Row
	C_vals[0][0]=0;
	C_vals[0][1]=(_Iyy-_Izz)*(long_factor) + 
		     (_Izz-_Iyy)*Rate.y*cos_R*cos_R*cos_P -
		     _Ixx*Rate.y*cos_P;
	C_vals[0][2]=(_Izz-_Iyy)*Rate.p*cos_R*sin_R*cos_P*cos_P;
	//Second Row
	C_vals[1][0]=(_Izz-_Iyy)*(long_factor) + 
	             (_Iyy-_Izz)*Rate.y*cos_R*cos_R*cos_P +
		     _Ixx*Rate.y*cos_P;
	C_vals[1][1]=(_Izz-_Iyy)*Rate.r*cos_R*cos_R;
	C_vals[1][2]=-_Ixx*Rate.y*sin_P*cos_P +
		      _Iyy*Rate.y*sin_R*sin_R*cos_P*sin_P +
		      _Izz*Rate.y*cos_R*cos_R*sin_P*cos_P;
	//Third Row
	C_vals[2][0]=(_Iyy-_Izz)*Rate.y*cos_P*cos_P*sin_R*cos_R - 
		     _Ixx*Rate.p*cos_P;
	C_vals[2][1]=(_Izz-_Iyy)*(Rate.p*cos_R*sin_R*sin_P+Rate.r*sin_R*sin_R*cos_P) +
		     (_Iyy-_Izz)*Rate.r*cos_R*cos_R*cos_P + 
		     _Ixx*Rate.y*sin_P*cos_P - 
		     _Iyy*Rate.y*sin_R*sin_R*sin_P*cos_P - 
		     _Izz*Rate.y*cos_R*cos_R*sin_P*cos_P;
	C_vals[2][2]=(_Iyy-_Izz)*Rate.r*cos_R*sin_R*cos_P*cos_P - 
		     _Iyy*Rate.p*sin_R*sin_R*cos_P*sin_P - 
		     _Izz*Rate.p*cos_R*cos_R*cos_P*sin_P + 
		     _Ixx*Rate.p*cos_P*sin_P;
	C= math::Matrix(3,3,C_vals);
}
