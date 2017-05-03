/*******************************************************************
** Types for SWE code
********************************************************************/

//#define SWE_GAIN_AD 0.025f
#define SWE_GAIN_AD 0.05f
//#define SWE_GAIN_AP 0.00005f
#define SWE_GAIN_AP 0.0f

//#define SWE_GAIN_VD 0.25f
#define SWE_GAIN_VD 0.3f
#define SWE_GAIN_VP 0.03f
//#define SWE_GAIN_VP 0.0f


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
	float accel_ave[3];
	
	float accel_delta[3];
	float omega_ad[3];
	float omega_ap[3];
	
	/* [v_x, v_y, v_Z]
	** Leg velocity vector
	** wrt leg coordinate reference frame */
	float vel[3];
	float vel_total[3];
	float vel_ave[3];
	
	float vel_delta[3];
	float omega_vd[3];
	float omega_vp[3];
	
	float pe[3];
	float pave;
	
	float N;
} SWE_STATE_TYPE;




