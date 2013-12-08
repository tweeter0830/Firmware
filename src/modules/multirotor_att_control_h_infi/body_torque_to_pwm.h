#ifndef H_INFI_WRAPPER_H_
#define H_INFI_WRAPPER_H_

struct body_torque{
	float roll;
	float pitch;
	float yaw;
};

struct body_torque_params{
	float arm_length;
	float forward_ang;
	float torque_fract;
};

int body_torque_to_pwm(const struct body_torque * torques,
		       const struct body_torque_params * p,
		       const float thrust,
		       const bool updated,
		       float *pwm_fract);
#endif /* H_INFI_WRAPPER_ */
