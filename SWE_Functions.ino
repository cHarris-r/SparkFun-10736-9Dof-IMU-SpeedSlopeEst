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
	int i;

	g_swe_state.pitch_mem   = 0.0f;
	g_swe_state.pitch_delta = 0.0f;
	g_swe_state.pitch_delta_total = 0.0f;
	
	for( i=0;i<3;i++ )
	{
		/* Initialize SWE Acceleration state vector */
		g_swe_state.accel[i] = 0.0f;
		g_swe_state.accel_total[i] = 0.0f;
		
		/* Initialize SWE Velocity state vector */
		g_swe_state.vel[i] = 0.0f;
		g_swe_state.vel_total[i] = 0.0f;
		
		g_swe_state.vel_delta[i] = 0.0f;
		g_swe_state.omega_vp[i]  = 0.0f;
		g_swe_state.omega_vi[i]  = 0.0f;
	}
	g_swe_state.N = 1.0f;
}

/* 
** Function: SWE_Reset
** This function resets the SWE  
** state variables. In particular,
** the integrated variables.
*/
void SWE_Reset ( void )
{
	int i;
	for( i=0; i<=2; i++)	
	{
		g_swe_state.vel[i] = 0.0f;
	}
}

/* 
** Function: SWE_Update
** This code executes the speed and 
** walking incline estimation state update
*/
void SWE_Update ( void )
{
	g_swe_state.N++;
	g_swe_state.pitch_delta = g_sensor_state.pitch - g_swe_state.pitch_mem;
	g_swe_state.pitch_delta_total += g_swe_state.pitch_delta;
	
	Map_Accel_2D();
	Integrate_Accel_2D();
	Estimate_Error();
	
	
	
	g_swe_state.pitch_mem = g_sensor_state.pitch;
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
	g_swe_state.accel_delta[0] = g_swe_state.accel[0];
	g_swe_state.accel_delta[1] = g_swe_state.accel[1];
	
	
	/* Calc Ax wrt world coordinate system */
	g_swe_state.accel[0]   = -g_sensor_state.accel[0]*sin(g_sensor_state.pitch) + g_sensor_state.accel[1]*cos(g_sensor_state.pitch);
	g_swe_state.accel[0]  -= (g_swe_state.omega_ap[0] + g_swe_state.omega_ai[0]);
		
	g_swe_state.accel_delta[0] = g_swe_state.accel[0] - g_swe_state.accel_delta[0];
	g_swe_state.omega_ap[0]    = g_swe_state.accel_delta[0]*SWE_GAIN_AP;
	g_swe_state.omega_ai[0]   += g_swe_state.accel_delta[0]*SWE_GAIN_AI;
	
	g_swe_state.accel_total[0] += g_swe_state.accel[0];
	
	
	/* Calc Ay wrt world coordinate system */
	g_swe_state.accel[1]  = g_sensor_state.accel[0]*cos(g_sensor_state.pitch) + g_sensor_state.accel[1]*sin(g_sensor_state.pitch) - GRAVITY;
	g_swe_state.accel[1] -= g_swe_state.omega_ap[1] - g_swe_state.omega_ai[1];
	
	g_swe_state.accel_delta[1] = g_swe_state.accel[1] - g_swe_state.accel_delta[1];
	g_swe_state.omega_ap[1]    = g_swe_state.accel_delta[1]*SWE_GAIN_AP;
	g_swe_state.omega_ai[1]   += g_swe_state.accel_delta[1]*SWE_GAIN_AI;
	
	g_swe_state.accel_total[1] += g_swe_state.accel[1];
}

/* 
** Function: Integrate_Accel_2D
** Integrate acceleration (wrt leg ref coordinates)
** to get velocity (wrt leg ref coordinates) 
** Assumes 2D motion
*/
void Integrate_Accel_2D ( void )
{
	int i;
	for( i=0; i<=2; i++)	
	{
		g_swe_state.vel_delta[i] = g_swe_state.vel[i];
		
		g_swe_state.vel[i]  = g_swe_state.vel[i] + g_swe_state.accel[i]*g_control_state.G_Dt;
		g_swe_state.vel[i] -= (g_swe_state.omega_vp[i] + g_swe_state.omega_vi[i]);
		
		g_swe_state.vel_delta[i] = g_swe_state.vel[i] - g_swe_state.vel_delta[i];
		g_swe_state.omega_vp[i]  = g_swe_state.vel_delta[i]*SWE_GAIN_VP;
		g_swe_state.omega_vi[i] += g_swe_state.vel_delta[i]*SWE_GAIN_VI;
		
		g_swe_state.vel_total[i] += g_swe_state.vel[i];
	}
}


/*
** Function: Estimate_Error
** This function is inteded to estimate the
** error in the intitial velocity estimates.
** There are several ways we can estimate the
** error in the velocity:
** 	1. We assume there is a relationship between the 
**     velocity error and the pitch delta. I.e. at 
**     small theta_dot, there is a high probability 
**     of error in the estimate
**  2. We can take the value of the velocity feedback 
**     intergation term as an approximation of the error.
**  3. We can take the difference between the final velocity and
**     the average velocity. This assumes that the
**     velocity should be a constant. Therefore, a large 
**     difference may indicate that the velocity has error. */
void Estimate_Error ( void )
{
	int i;
	float pave,pe1,pe2,pe3;
	float m1_pho, m1_ave_pitch_delta;
	float m2_ave_feedback;
	float m3_ave_vel, m3_delta;
	
	String imuLog;
	
	/* Method 1 
	** Estimate error from relationship to pitch delta 
	** m1_pho: Strength of relationship */
	m1_pho = 0.1f;
	m1_ave_pitch_delta = abs(g_swe_state.pitch_delta_total/g_swe_state.N);
	//pe1 = m1_ave_pitch_delta;
	pe1 = exp(-m1_ave_pitch_delta/m1_pho);
	
	/* Method 2:
	** Estimate error based on velocity feedback values */
	m2_ave_feedback = 0.0f;
	for( i=0;i<2;i++ ) { m2_ave_feedback += abs(g_swe_state.omega_vi[i]/g_swe_state.vel[i]); }
	//pe2 = m2_ave_feedback;
	pe2 = min( abs(m2_ave_feedback/2), 1);
	
	/* Method 3:
	** Estimate error based on the difference between
	** the average velocity and the final velocity */
	m3_delta = 0.0f;
	for( i=0;i<2;i++ ) 
	{ 
		m3_ave_vel = (g_swe_state.vel_total[i]/g_swe_state.N);
		m3_delta += abs((m3_ave_vel - g_swe_state.vel[i])/m3_ave_vel);
	}
	//pe3 = m3_delta;
	pe3 = min(m3_delta/2,1);
	
	/* Get final error estimate
	** Final estimate can be the avereage p */
	pave = (pe1 + pe2 + pe3)/3;
	
	if ( millis() > (g_control_state.g_LastBlinkTime + UART_BLINK_RATE) )
  {
		imuLog = "\t\t err est (p1/p2/p3/pave): ";
		imuLog += String( pe1,7 ) + "/" + String( pe2,7 ) + "/" + String( pe3,7 ) + "/" +String( pave,7 );
		imuLog += "\r\n\n"; 
		LOG_PORT.print( imuLog );
	}
}








