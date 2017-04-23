/*************************************************
** FILE: SWE_Functions
** This file contains all functions which are
** specific to the speed and walking incline 
** estimation capabilities
**************************************************/


/* 
** Function: SWE_Init
** This function initializes the SWE state 
** variables.
*/
void SWE_Init ( void )
{
  /* Initialize poincare event */
	g_swe_state.HeelStrikeEvent = FALSE;

	/* Assume planted leg to start */
  g_swe_state.LegStanceState = 1;
	
	/* Initialize Pitch Memory */
	//g_swe_state.PitchMem[] = { 0.0f,0.0f };
	
	/* Initialize Pitch delta */
	g_swe_state.PitchDelta = 0;
	
	/* Initialize SWE Acceleration state vector */
	//g_swe_state.accel[] = {0.0f,0.0f,0.0f};
	
	/* Initialize SWE Velocity state vector */
	//g_swe_state.vel[] = {0.0f,0.0f,0.0f};
}

/* 
** Function: SWE_Update
** This code executes the speed and 
** walking incline estimation state update
*/
void SWE_Update ( void )
{
	Map_Accel_2D();
	Integrate_accel_2D();
}

/*
** Function: Map_Accel_2D
** This function maps a_t and a_n to a_x and a_y 
** using the filtered pitch assuming 2D motion.
** This Does not account for roll.
** NOTE: There may be a better way of extracting
**       this from the mid-filter DCM 
*/
void Map_Accel_2D ( void )
{

	/* Accel x:Fore y:Port z:Zenith */
	g_swe_state.accel[0] = -g_sensor_state.accel[0]*sin(g_sensor_state.pitch) + g_sensor_state.accel[1]*cos(g_sensor_state.pitch);
	g_swe_state.accel[1] = g_sensor_state.accel[0]*cos(g_sensor_state.pitch) + g_sensor_state.accel[1]*sin(g_sensor_state.pitch);
}

/* 
** Function: Integrate_accel_2D
** Integrate acceleration (wrt leg ref coordinates)
** to get velocity (wrt leg ref coordinates) 
** Assumes 2D motion
*/
void Integrate_accel_2D ( void )
{
	int i;
	for( i=0; i<=2; i++)	
	{
		g_swe_state.vel[i] = g_swe_state.vel[i] + g_swe_state.accel[i]*g_control_state.G_Dt + g_swe_state.vel_init[i];
	}
}










