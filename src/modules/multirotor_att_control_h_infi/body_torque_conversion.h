#ifndef BODY_TORQUE_TO_PWM_H_
#define BODY_TORQUE_TO_PWM_H_

#ifdef __cplusplus
extern "C" {
#endif

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

//extern void body_torque_to_pwm(void);
__EXPORT void body_torque_to_pwm(struct body_torque * torques,
				 struct body_torque_params * p,
				 float thrust,
				 bool updated,
				 float * pwm_fract,
				 bool debug_loop);
#ifdef __cplusplus
}
#endif
#endif /* H_INFI_WRAPPER_ */
