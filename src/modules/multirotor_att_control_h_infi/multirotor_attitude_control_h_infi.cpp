#include <stdint.h>
#include <math.h>
#include <float.h>
#include <stdio.h>
#include <stdlib.h> 

#include "multirotor_attitude_control_h_infi.hpp"

// #define H_INFI_CORE_DEBUG

#ifndef RT_RIF
#define RT_PIF	3.1415927F
#endif 

static float angle_error( float ref, float meas);

Multirotor_Attitude_Control_H_Infi::
Multirotor_Attitude_Control_H_Infi() : 
	_M(3,3),
	_M_inv(3,3),
	_Cor(3,3)
{
	_last_run	       = 0;
	_tc		       = 0.1f;
	_weight_error_deriv    = 1;
	_weight_error_state    = 1;
	_weight_error_integral = 1;
	_weight_torque	       = 1;
	_Ixx		       = 1;
	_Iyy		       = 1;
	_Izz		       = 1;
	_old_time	       = 0;
	_modes_set	       = false;
	_state_track	       = true;
	_rate_track	       = true;
	_accel_track	       = true;
	_yaw_track	       = true;
	_setpoint_state[0]     = 0;
	_setpoint_state[1]     = 0;
	_setpoint_state[2]     = 0;
	_setpoint_rate[0]      = 0;
	_setpoint_rate[1]      = 0;
	_setpoint_rate[2]      = 0;
	_setpoint_accel[0]     = 0;
	_setpoint_accel[1]     = 0;
	_setpoint_accel[2]     = 0;
	_integral(0)	       = 0.0f;
	_integral(1)	       = 0.0f;
	_integral(2)	       = 0.0f;
	_int_sat	       = 10.0f;
	_max_p                 = 1.0f;
	_max_r                 = 1.0f;
	_max_y                 = 0.5f;
}

void Multirotor_Attitude_Control_H_Infi::
set_mode(bool state_track, bool rate_track, bool accel_track, bool yaw_track){
	_state_track = state_track;
	_rate_track  = rate_track;
	_accel_track = accel_track;
	_yaw_track   = yaw_track;
	_modes_set   = true;
}
void Multirotor_Attitude_Control_H_Infi::
set_setpoints(const float state[],const float rate[],const float accel[]) {
	_setpoint_state[0] = state[0];
	_setpoint_state[1] = state[1];
	_setpoint_state[2] = state[2];
	_setpoint_rate[0] = rate[0];
	_setpoint_rate[1] = rate[1];
	_setpoint_rate[2] = rate[2];
	_setpoint_accel[0] = accel[0];
	_setpoint_accel[1] = accel[1];
	_setpoint_accel[2] = accel[2];
}

bool Multirotor_Attitude_Control_H_Infi::
control(const float meas_state[], const float meas_rate[], double time, float torque_out[] ){
	if(!_modes_set) {
		return false;
	}
	// TODO: check inputs here
	float dt = time-_old_time;
	Matrix k_p(3,3);
	Matrix k_i(3,3);
	Matrix k_d(3,3);
	make_M(meas_state,_M);
	make_C(meas_state, meas_rate, _Cor);
	calc_gains(_M,_Cor, k_p, k_i, k_d);
	Vector error_state;
	if( _state_track ){
		error_state(0) = angle_error( _setpoint_state[0], meas_state[0]);
		error_state(1) = angle_error( _setpoint_state[1], meas_state[1]);
		error_state(2) = angle_error( _setpoint_state[2], meas_state[2]);
	}else{
		error_state.setAll(0.0f);
	}
	if( !_yaw_track ){
		error_state(2)=0;
	}
	Vector error_rate;
	if( _rate_track ) {
		error_rate(0) = angle_error( _setpoint_rate[0], meas_rate[0]);
		error_rate(1) = angle_error( _setpoint_rate[1], meas_rate[1]);
		error_rate(2) = angle_error( _setpoint_rate[2], meas_rate[2]);
	} else {
		error_rate.setAll(0.0f);
	}
	Vector setpoint_accel;
	if( _accel_track ) {
		setpoint_accel(0) = _setpoint_accel[0];
		setpoint_accel(1) = _setpoint_accel[1];
		setpoint_accel(2) = _setpoint_accel[2];
	} else {
		setpoint_accel.setAll(0.0f);
	}
	Vector meas_rate_vect;
	meas_rate_vect(0)=meas_rate[0];
	meas_rate_vect(1)=meas_rate[1];
	meas_rate_vect(2)=meas_rate[2];
#ifdef H_INFI_CORE_DEBUG
	std::cout << "error_state " << error_state << std::endl;
	std::cout << "error_rate " << error_rate << std::endl;
	std::cout << "setpoint_accel " << setpoint_accel << std::endl;
#endif
	Vector control_accel = k_d*error_rate + k_p*error_state + k_i*_integral;
	Vector control_torque = _M*setpoint_accel + _Cor*meas_rate_vect - _M*control_accel;
	float sat_vals[3] = {_max_r, _max_p, _max_y};
	for( int i = 0; i < 3; i++){
		float cur_val = control_torque(i);
		if( cur_val < -sat_vals[i] || sat_vals[i] < cur_val ){
			int sign_c = (cur_val > 0) - (cur_val < 0);
			int sign_i = (_integral(i) > 0) - (_integral(i) < 0);
			int sign_er= (error_state(i) > 0) - (error_state(i) < 0);
			control_torque(i) = sat_vals[i]*sign_c;
			if( sign_i != sign_er){
				_integral(i) = _integral(i) + error_state(i)*dt;
			}
		}else{
			_integral(i) = _integral(i) + error_state(i)*dt;
		}
		if( _integral(i) > _int_sat )
		        _integral(i) = _int_sat;
		else if( _integral(i) < -_int_sat )
		        _integral(i) = -_int_sat;
	}
	// control_torque(2) = _integral(1); //DEBUG HACK
	if( !_yaw_track ){
		_integral(2) = 0;
	}

#ifdef H_INFI_CORE_DEBUG
	std::cout << "Time Diff: " << dt << std::endl;
	std::cout << "_integral " << _integral << std::endl;
	std::cout << "control_accel " << control_accel << std::endl;
	std::cout << "control_torque "<< control_torque<< std::endl;
#endif
	torque_out[0] = control_torque(0); //roll
	torque_out[1] = control_torque(1); //pitch
	torque_out[2] = control_torque(2); //yaw
	_old_time = time;
	return true;
}

void Multirotor_Attitude_Control_H_Infi::
calc_gains(const Matrix& M,const Matrix& C, Matrix& k_p, Matrix& k_i, Matrix& k_d) {
	const float w_1 = _weight_error_deriv;
	const float w_2 = _weight_error_state;
	const float w_3 = _weight_error_integral;
	const float w_u = _weight_torque;
	
	float I_data[] = { 1,0,0, 0,1,0, 0,0,1 };
	Matrix I(3,3,I_data);
	Matrix M_inv(3,3);
	M_inv = M.inverse();
	Matrix Dynamics_weights = M_inv*( C+I*( 1.0f/(w_u*w_u) ) );
	float long_expr = sqrt(w_2*w_2 + 2.0f*w_1*w_3)/w_1;
	
	k_d = (I*long_expr)+Dynamics_weights;
	k_p = I*(w_3/w_1)+Dynamics_weights*long_expr;
	k_i = Dynamics_weights*(w_3/w_1);
#ifdef H_INFI_CORE_DEBUG
	std::cout<< "Calculated Gains:---------"<< std::endl;
	std::cout<< "I\n" << I << std::endl;
	std::cout<< "M_Inverse\n" << M_inv << std::endl;
	std::cout<< "long_expr\n " << long_expr << std::endl;
	std::cout<< "Dynamics_weights\n" <<Dynamics_weights << std::endl;
	std::cout<< "k_d\n"<< k_d << std::endl;
	std::cout<< "k_p\n"<< k_p << std::endl;
	std::cout<< "k_i\n"<< k_i << std::endl;
#endif
}

void Multirotor_Attitude_Control_H_Infi::
make_M(const float St[], Matrix& M){
	float sin_R=sin(St[0]);
	float cos_R=cos(St[0]);
	float sin_P=sin(St[1]);
	float cos_P=cos(St[1]);
	//First Row
	M(0,0)=_Ixx;
	M(0,1)=0;
	M(0,2)=-_Ixx*sin_P;
	// Second Row
	M(1,0)=0;
	M(1,1)=_Iyy*cos_R*cos_R + _Izz*sin_R*sin_R;
	M(1,2)=(_Iyy-_Izz)*cos_R*sin_R*cos_P;
	// Third row
	M(2,0)=-_Ixx*sin_P;
	M(2,1)=(_Iyy-_Izz)*cos_R*sin_R*cos_P;
	M(2,2)=_Ixx*sin_P*sin_P + _Iyy*sin_R*sin_R*cos_P*cos_P +
		     _Izz*cos_R*cos_R*cos_P*cos_P;
	
#ifdef H_INFI_CORE_DEBUG
	std::cout<< "M " << M << std::endl;
#endif
}

void Multirotor_Attitude_Control_H_Infi::
make_C(const float St[], const float Rate[], Matrix& C){
	float s_ph=sin(St[0]);
	float c_ph=cos(St[0]);
	float s_th=sin(St[1]);
	float c_th=cos(St[1]);
	// theta -> Pitch
	// phi -> roll
	// lamba -> yaw
	// state[0] = roll
	// state[1] = pitch
	// state[2] = yaw
	float long_factor = Rate[1]*c_ph*s_ph + Rate[2]*s_ph*s_ph*c_th;
	//First Row
	C(0,0)=0;
	C(0,1)=(_Iyy-_Izz)*(long_factor) + 
		     (_Izz-_Iyy)*Rate[2]*c_ph*c_ph*c_th -
		     _Ixx*Rate[2]*c_th;
	C(0,2)=(_Izz-_Iyy)*Rate[2]*c_ph*s_ph*c_th*c_th;
	//Second Row
	C(1,0)=(_Izz-_Iyy)*(long_factor) + 
	             (_Iyy-_Izz)*Rate[2]*c_ph*c_ph*c_th +
		     _Ixx*Rate[2]*c_th;
	C(1,1)=(_Izz-_Iyy)*Rate[0]*c_ph*c_ph;
	C(1,2)=-_Ixx*Rate[2]*s_th*c_th +
		      _Iyy*Rate[2]*s_ph*s_ph*c_th*s_th +
		      _Izz*Rate[2]*c_ph*c_ph*s_th*c_th;
	//Third Row
	C(2,0)=(_Iyy-_Izz)*Rate[2]*c_th*c_th*s_ph*c_ph - 
		     _Ixx*Rate[1]*c_th;
	C(2,1)=(_Izz-_Iyy)*(Rate[1]*c_ph*s_ph*s_th+Rate[0]*s_ph*s_ph*c_th) +
		     (_Iyy-_Izz)*Rate[0]*c_ph*c_ph*c_th + 
		     _Ixx*Rate[2]*s_th*c_th - 
		     _Iyy*Rate[2]*s_ph*s_ph*s_th*c_th - 
		     _Izz*Rate[2]*c_ph*c_ph*s_th*c_th;
	C(2,2)=(_Iyy-_Izz)*Rate[0]*c_ph*s_ph*c_th*c_th - 
		     _Iyy*Rate[1]*s_ph*s_ph*c_th*s_th - 
		     _Izz*Rate[1]*c_ph*c_ph*c_th*s_th + 
		     _Ixx*Rate[1]*c_th*s_th;
#ifdef H_INFI_CORE_DEBUG
	std::cout<< "C " << C << std::endl;
#endif
}

void Multirotor_Attitude_Control_H_Infi::
reset_integrator()
{
	_integral(0)=0.0f;
	_integral(1)=0.0f;
	_integral(2)=0.0f;
}

static float angle_error( float ref, float meas){
	if(  abs( meas-ref) <= RT_PIF )
		return  meas-ref;
	else if( meas > ref)
		return -RT_PIF*2 + meas - ref;
	else //ref > meas
		return RT_PIF*2 + meas - ref;
}
