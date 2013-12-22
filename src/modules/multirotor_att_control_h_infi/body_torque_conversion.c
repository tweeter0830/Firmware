#include <math.h>
#include <nuttx/config.h>
#include <debug.h>
#include <systemlib/err.h>
#include <systemlib/param/param.h>

#include "body_torque_conversion.h"

#define BODY_TORQUE_DEBUG

static void make_conversion_mat( const struct body_torque_params * p, float out_mat[16] );
static float simple_interp(const float * x, const float * y, const int length, const float x0);
static bool invert_4by4_matrix(const float m[16], float invOut[16]);
static void print_4by4_matrix(const float m[16]);

__EXPORT void body_torque_to_pwm(struct body_torque * torques,
				 struct body_torque_params * p,
				 float thrust,
				 bool updated,
				 float * pwm_fract,
				 bool debug_loop){
	static float pwm_val[]    = { 950,1000,1050,1100,1150,1200,1250,1350,1450,1550,1650,1750,1850,1900};
	static float thrust_val[] = {0.03f,0.21f,0.45f,0.73f,1.06f,1.39f,1.82f,2.85f,4.05f,5.40f,6.65f,7.69f,8.05f,8.35f};
	static int table_length = sizeof(thrust_val)/sizeof(thrust_val[0]);

	static bool initialized = false;
	
	static float body_to_motor[16] = {0};
	float motor_to_body[16] = {0};
	bool zero_det = false;
	if( updated || ~initialized ){
		make_conversion_mat( p, motor_to_body);
		zero_det = invert_4by4_matrix(motor_to_body, body_to_motor);
		if( zero_det ){
			warnx("Zero determinant");
		}
		initialized = true;
	}

	float rot_thrust[4] = {0};
	rot_thrust[0] = torques->p*body_to_motor[0]+torques->r*body_to_motor[1]+torques->y*body_to_motor[2]+thrust*body_to_motor[3];
	rot_thrust[1] = torques->p*body_to_motor[4]+torques->r*body_to_motor[5]+torques->y*body_to_motor[6]+thrust*body_to_motor[7];
 	rot_thrust[2] = torques->p*body_to_motor[8]+torques->r*body_to_motor[9]+torques->y*body_to_motor[10]+thrust*body_to_motor[11];
	rot_thrust[3] = torques->p*body_to_motor[12]+torques->r*body_to_motor[13]+torques->y*body_to_motor[14]+thrust*body_to_motor[15];
	
	float rot_high_time[4] = {0};
	for( int i = 0; i<4; i++ ){
		rot_high_time[i] = simple_interp(thrust_val, pwm_val, table_length, rot_thrust[i]);
		pwm_fract[i] = ( rot_high_time[i]-pwm_val[0] )/( pwm_val[table_length-1]-pwm_val[0] );
	}
#ifdef BODY_TORQUE_DEBUG
	if (debug_loop){
		warnx("Initialized: %b",initialized);
		warnx("motor_to_body:");
		print_4by4_matrix(motor_to_body);
		warnx("body_to_motor:");
		print_4by4_matrix(body_to_motor);
		warnx("rot_thrust:");
		warnx("[%4.2f\t%4.2f\t%4.2f\t%4.2f]",
		      rot_thrust[0], rot_thrust[1], rot_thrust[2], rot_thrust[3]);
		warnx("rot_high_time:");
		warnx("[%4.2f\t%4.2f\t%4.2f\t%4.2f]",
		      rot_high_time[0], rot_high_time[1], rot_high_time[2], rot_high_time[3]);
		warnx("pwm_fract:");
		warnx("[%4.2f\t%4.2f\t%4.2f\t%4.2f]",
		      pwm_fract[0], pwm_fract[1], pwm_fract[2], pwm_fract[3]);
	}
#endif
}

static void make_conversion_mat( const struct body_torque_params * p, float out_mat[16] ){
        float	l   = p->arm_length;
	float	the = p->forward_ang;
	float	lam = p->torque_fract;
	float   c0  = cos(the);
	float   s0  = sin(the);

	out_mat[0] =  l*c0;
	out_mat[1] = -l*c0;
	out_mat[2] =  l*s0;
	out_mat[3] = -l*s0;

	out_mat[4] = -l*s0;
	out_mat[5] =  l*s0;
	out_mat[6] =  l*c0;
	out_mat[7] = -l*c0;

	out_mat[8] =  lam;
	out_mat[9] =  lam;
	out_mat[10]= -lam;
	out_mat[11]= -lam;

	out_mat[12] = 1;
	out_mat[13] = 1;
	out_mat[14] = 1;
	out_mat[15] = 1;
}

static float simple_interp(const float * x, const float * y, const int length, float x0){
	float bot_x, top_x;
	if( x0 <= x[0] )
		return y[0];
	if( x0 >= x[length - 1] )
		return y[length - 1];
	for(int i = 0; i<length-1; i++){
		bot_x = x[i];
		top_x = x[i+1];
		if( bot_x <= x0 && x0 < top_x ){
			float y_0 = y[i]+(y[i+1]-y[i])*(x0-x[i])/(x[i+1]-x[i]);
			return y_0;
		}
	}
	return y[0];
}

static bool invert_4by4_matrix(const float m[16], float invOut[16])
{
    float tol = 0.000000001f;
    float inv[16], det;
    int i;

    inv[0] = m[5]  * m[10] * m[15] - 
             m[5]  * m[11] * m[14] - 
             m[9]  * m[6]  * m[15] + 
             m[9]  * m[7]  * m[14] +
             m[13] * m[6]  * m[11] - 
             m[13] * m[7]  * m[10];

    inv[4] = -m[4]  * m[10] * m[15] + 
              m[4]  * m[11] * m[14] + 
              m[8]  * m[6]  * m[15] - 
              m[8]  * m[7]  * m[14] - 
              m[12] * m[6]  * m[11] + 
              m[12] * m[7]  * m[10];

    inv[8] = m[4]  * m[9] * m[15] - 
             m[4]  * m[11] * m[13] - 
             m[8]  * m[5] * m[15] + 
             m[8]  * m[7] * m[13] + 
             m[12] * m[5] * m[11] - 
             m[12] * m[7] * m[9];

    inv[12] = -m[4]  * m[9] * m[14] + 
               m[4]  * m[10] * m[13] +
               m[8]  * m[5] * m[14] - 
               m[8]  * m[6] * m[13] - 
               m[12] * m[5] * m[10] + 
               m[12] * m[6] * m[9];

    inv[1] = -m[1]  * m[10] * m[15] + 
              m[1]  * m[11] * m[14] + 
              m[9]  * m[2] * m[15] - 
              m[9]  * m[3] * m[14] - 
              m[13] * m[2] * m[11] + 
              m[13] * m[3] * m[10];

    inv[5] = m[0]  * m[10] * m[15] - 
             m[0]  * m[11] * m[14] - 
             m[8]  * m[2] * m[15] + 
             m[8]  * m[3] * m[14] + 
             m[12] * m[2] * m[11] - 
             m[12] * m[3] * m[10];

    inv[9] = -m[0]  * m[9] * m[15] + 
              m[0]  * m[11] * m[13] + 
              m[8]  * m[1] * m[15] - 
              m[8]  * m[3] * m[13] - 
              m[12] * m[1] * m[11] + 
              m[12] * m[3] * m[9];

    inv[13] = m[0]  * m[9] * m[14] - 
              m[0]  * m[10] * m[13] - 
              m[8]  * m[1] * m[14] + 
              m[8]  * m[2] * m[13] + 
              m[12] * m[1] * m[10] - 
              m[12] * m[2] * m[9];

    inv[2] = m[1]  * m[6] * m[15] - 
             m[1]  * m[7] * m[14] - 
             m[5]  * m[2] * m[15] + 
             m[5]  * m[3] * m[14] + 
             m[13] * m[2] * m[7] - 
             m[13] * m[3] * m[6];

    inv[6] = -m[0]  * m[6] * m[15] + 
              m[0]  * m[7] * m[14] + 
              m[4]  * m[2] * m[15] - 
              m[4]  * m[3] * m[14] - 
              m[12] * m[2] * m[7] + 
              m[12] * m[3] * m[6];

    inv[10] = m[0]  * m[5] * m[15] - 
              m[0]  * m[7] * m[13] - 
              m[4]  * m[1] * m[15] + 
              m[4]  * m[3] * m[13] + 
              m[12] * m[1] * m[7] - 
              m[12] * m[3] * m[5];

    inv[14] = -m[0]  * m[5] * m[14] + 
               m[0]  * m[6] * m[13] + 
               m[4]  * m[1] * m[14] - 
               m[4]  * m[2] * m[13] - 
               m[12] * m[1] * m[6] + 
               m[12] * m[2] * m[5];

    inv[3] = -m[1] * m[6] * m[11] + 
              m[1] * m[7] * m[10] + 
              m[5] * m[2] * m[11] - 
              m[5] * m[3] * m[10] - 
              m[9] * m[2] * m[7] + 
              m[9] * m[3] * m[6];

    inv[7] = m[0] * m[6] * m[11] - 
             m[0] * m[7] * m[10] - 
             m[4] * m[2] * m[11] + 
             m[4] * m[3] * m[10] + 
             m[8] * m[2] * m[7] - 
             m[8] * m[3] * m[6];

    inv[11] = -m[0] * m[5] * m[11] + 
               m[0] * m[7] * m[9] + 
               m[4] * m[1] * m[11] - 
               m[4] * m[3] * m[9] - 
               m[8] * m[1] * m[7] + 
               m[8] * m[3] * m[5];

    inv[15] = m[0] * m[5] * m[10] - 
              m[0] * m[6] * m[9] - 
              m[4] * m[1] * m[10] + 
              m[4] * m[2] * m[9] + 
              m[8] * m[1] * m[6] - 
              m[8] * m[2] * m[5];

    det = m[0] * inv[0] + m[1] * inv[4] + m[2] * inv[8] + m[3] * inv[12];

    if (-tol < det && det < tol)
        return false;

    det = 1.0f / det;

    for (i = 0; i < 16; i++)
        invOut[i] = inv[i] * det;

    return true;
}

static void print_4by4_matrix(const float m[16]){
	for (int i = 0 ; i < 4 ; i++){
		warnx("Row:%d, [%3.4f\t%3.4f\t%3.4f\t%3.4f]",
		      i, m[i], m[i+1], m[i+2], m[i+3]);
	}
}
