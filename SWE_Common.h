/*******************************************************************
** Types for SWE code
********************************************************************/

#define SWE_GAIN_AP 0.1f
#define SWE_GAIN_AI 0.00005f
#define SWE_GAIN_VP 0.01f
#define SWE_GAIN_VI 0.000005f


/*
** TYPE: SWE_STATE_TYPE
** This holds all the state variables
** for the walking speed and estimation code
*/
typedef struct
{
	/* TRUE at heel strike only
	** used for poincare mapping */
  bool HeelStrikeEvent;
	
	/* Swing State:   0
	** Planted State: 1 */
	bool LegStanceState; 
	
	/* Pitch Memory to get Pitch Delta */
	float PitchMem[2];
	
	/* Positive pitch delta: Upswing (Swing-Leg)
	** Negative pitch delta: Downswing (Planted-Leg) */
	float PitchDelta;
	
	/* [a_x, a_y, a_z]
	** Leg acceleration vector
	** wrt leg coordinate reference frame */
	float accel[3];
	float accel_total[3];
	
	/* [v_x, v_y, v_Z]
	** Leg velocity vector
	** wrt leg coordinate reference frame */
	float vel[3];
	
	/* Initial velocity for each gait */
	float vel_init[3];
	
	float vel_delta[3];
	float vel_delta_total[3];
	
	float N;
} SWE_STATE_TYPE;
