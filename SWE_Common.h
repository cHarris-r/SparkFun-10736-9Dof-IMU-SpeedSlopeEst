/*******************************************************************
** Types for SWE code
********************************************************************/

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
	
	/* [v_x, v_y, v_Z]
	** Leg velocity vector
	** wrt leg coordinate reference frame */
	float vel[3];
	
	/* Initial velocity for each gait */
	float vel_init[3];
	
} SWE_STATE_TYPE;
