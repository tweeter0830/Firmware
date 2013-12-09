#ifndef BODY_TORQUE_TO_PWM_H_
#define BODY_TORQUE_TO_PWM_H_

struct body_torque{
	float r;
	float p;
	float y;
};

struct body_torque_params{
	float arm_length;
	float forward_ang;
	float torque_fract;
};

extern void body_torque_to_pwm(struct body_torque * torques,
			struct body_torque_params * p,
			float thrust,
		        bool updated,
			float * pwm_fract);
#endif /* H_INFI_WRAPPER_ */
