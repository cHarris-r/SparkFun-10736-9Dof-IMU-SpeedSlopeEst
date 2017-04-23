/*************************************************
** FILE: CAL_Functions
** This file contains the functions which will help 
** calibration the sensors
**************************************************/


/* 
** Function: Calibration_Init
** This function initializes the calibration state 
** variables.
*/
void Calibration_Init ( void )
{
  int i;
	for( i=0; i<3; i++ )
	{
		g_calibration.accel_total[i] = 0.0f;
		g_calibration.accel_max[i]   = -9999.0f;
		g_calibration.accel_min[i]   = 9999.0f;
		
		g_calibration.gyro_total[i] = 0.0f;
		g_calibration.gyro_max[i]   = -9999.0f;
		g_calibration.gyro_min[i]   = 9999.0f;
	}
	g_calibration.N = 0;
}



/* 
** Function: Calibrate
*/
void Calibrate ( void )
{
	int i;
	for( i=0; i<3; i++ )
	{
		g_calibration.accel_total[i] += g_sensor_state.accel[i];
		if( g_sensor_state.accel[i] > g_calibration.accel_max[i] ) { g_calibration.accel_max[i] = g_sensor_state.accel[i]; }
		if( g_sensor_state.accel[i] < g_calibration.accel_min[i] ) { g_calibration.accel_min[i] = g_sensor_state.accel[i]; }
	
		g_calibration.gyro_total[i] += g_sensor_state.gyro[i];
		if( g_sensor_state.gyro[i] > g_calibration.gyro_max[i] ) { g_calibration.gyro_max[i] = g_sensor_state.gyro[i]; }
		if( g_sensor_state.gyro[i] < g_calibration.gyro_min[i] ) { g_calibration.gyro_min[i] = g_sensor_state.gyro[i]; }
	}
	
	g_calibration.N++;
}









