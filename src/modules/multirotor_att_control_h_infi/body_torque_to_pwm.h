#ifndef H_INFI_WRAPPER_H_
#define H_INFI_WRAPPER_H_


struct body_torque{
	float roll;
	float pitch;
	float yaw;
};

struct up_to_mapping{
	unsigned int data_points;
	float 

};
int body_torque_to_pwm(const struct body_torque * torques, float thrust, float *pwm);
bool gluInvertMatrix(const double m[16], double invOut[16]);

float simple_interp(const float * x, const float * y, const int length, const float x0);

#endif /* H_INFI_WRAPPER_ */
