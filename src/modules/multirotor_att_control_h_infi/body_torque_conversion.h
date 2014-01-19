#ifndef BODY_TORQUE_TO_PWM_H_
#define BODY_TORQUE_TO_PWM_H_

#include <stdbool.h> 

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

struct body_torque_params{
	float arm_length;
	float forward_ang;
	float torque_fract;
};

//extern void body_torque_to_pwm(void);
#ifdef __CC_ARM
__EXPORT
#endif
void body_torque_to_pwm(const float torques[],
			const struct body_torque_params * p,
			float thrust,
			bool updated,
			float * pwm_fract);
#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* H_INFI_WRAPPER_ */
