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

typedef math::Matrix  Matrix;
typedef math::Vector3  Vector;

// #if !defined(CONFIG_ARCH_CORTEXM4) && !defined(CONFIG_ARCH_FPU)
// typedef long long uint64_t;
// #endif

// TODO: scale the integral to timestep
class __EXPORT Multirotor_Attitude_Control_H_Infi{ //__EXPORT
public:
	Multirotor_Attitude_Control_H_Infi();
        
	void set_phys_params(float Ixx, float Iyy, float Izz){
		_Ixx = Ixx;
		_Iyy = Iyy;
		_Izz = Izz;
	};
	
	bool control(const float meas_state[], const float meas_rate[], double time, float torque_out[] );

	void reset_integrator();

	void set_mode(bool state_track, bool rate_track, bool accel_track, bool yaw_track);

	void set_setpoints(const float state[],const float rate[],const float accel[]);

	void set_weights(float w_deriv, float w_error, float w_int, float w_torque){
		_weight_error_state = w_error;
		_weight_error_integral = w_int;
		_weight_error_deriv = w_deriv;
		_weight_torque = w_torque;
	}
	void set_weight_error_deriv(float weight_in) {
		_weight_error_deriv = weight_in;
	}
	void set_weight_error_state(float weight_in) {
		_weight_error_state = weight_in;
	}
	void set_weight_integral(float weight_in) {
		_weight_error_integral = weight_in;
	}
	void set_integrator_max(float max) {
		_int_sat = max;
	}
	void set_max_moments(float r_max, float p_max, float y_max){
		_max_p = p_max;
		_max_r = r_max;
		_max_y = y_max;
	}
	// void set_max_rate(float max_rate) {
	// 	//_max_rate = max_rate;
	// }

	// float get_rate_error() {
	// 	//return _rate_error;
	// }

	// float get_desired_rate() {
	// 	//return _rate_setpoint;
	// }
	float get_integral(int num){
		return _integral(num);
	}

private:
	int _last_run;
	float _tc;
	float _weight_error_deriv;
	float _weight_error_state;
	float _weight_error_integral;
	float _weight_torque;
	float _Ixx;
	float _Iyy;
	float _Izz;
	float _setpoint_state[3];
	float _setpoint_rate[3];
	float _setpoint_accel[3];
	float _command_torque[3];
	bool _modes_set;
	bool _state_track;
	bool _rate_track;
	bool _accel_track;
	bool _yaw_track;
	float _int_sat;

	float _max_p;
	float _max_r;
	float _max_y;

        Vector _integral;
	Matrix _M;
        Matrix _M_inv;
	Matrix _Cor;

	double _old_time;

	void calc_gains(const Matrix& M,const Matrix& C, Matrix& k_p, Matrix& k_i, Matrix& k_d);
	void make_M(const float St[], Matrix& M);
	void make_C(const float St[], const float Rate[], Matrix& C);
};
#endif // MULTIROTOR_ATTITUDE_CONTROL_H_INFI_H
