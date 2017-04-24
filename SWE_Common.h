/*******************************************************************
** Types for SWE code
********************************************************************/

#define SWE_GAIN_AP 0.01f
#define SWE_GAIN_AI 0.00005f

#define SWE_GAIN_VP 0.25f
#define SWE_GAIN_VI 0.025f


/*
** TYPE: SWE_STATE_TYPE
** This holds all the state variables
** for the walking speed and estimation code
*/
typedef struct
{
	float pitch_mem;
	float pitch_delta;
	float pitch_delta_total;
	
	/* [a_x, a_y, a_z]
	** Leg acceleration vector
	** wrt leg coordinate reference frame */
	float accel[3];
	float accel_total[3];
	
	float accel_delta[3];
	float omega_ap[3];
	float omega_ai[3];
	
	/* [v_x, v_y, v_Z]
	** Leg velocity vector
	** wrt leg coordinate reference frame */
	float vel[3];
	float vel_total[3];
	
	float vel_delta[3];
	float omega_vp[3];
	float omega_vi[3];
	
	float N;
} SWE_STATE_TYPE;
