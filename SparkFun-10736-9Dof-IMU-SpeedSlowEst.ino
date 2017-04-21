
#include "IMU10736_Config.h"
#include "IMU_Common.h"
#include <Wire.h>


/*******************************************************************
** Globals *********************************************************
********************************************************************/

/* DCM variables */
DCM_STATE_TYPE     g_dcm_state;
SENSOR_STATE_TYPE  g_sensor_state;
CONTROL_STATE_TYPE g_control_state;


/*******************************************************************
** START ***********************************************************
********************************************************************/

/*************************************************
** Setup Function 
** This function contains the setup functions 
** including the initialization of the hardware
** and the initialization of the serial ports
*/
void setup()
{
  /* Initialize the hardware */
  Init_Hardware();

  /* Initialize the IMU */
  if ( !Init_IMU() ) 
  {
    LOG_PORT.println("Error connecting to MPU-9250");
    while(1){ }  // Loop forever if we fail to connect
  }
  LOG_PORT.println("> IMU Initialized");
  delay(20);
  
  /* Set the initial roll/pitch/yaw from 
  ** initial accel/gyro */
  Reset_Sensor_Fusion(); 
}

// Main loop
void loop()
{ 
  /* Update sensor readings */
  Read_Sensors();
	
	/* Apply the DCM Filter */
  Update_Time();
	DCM_Filter();

  /* Blink LED 
  ** TO DO: It would be nice to have a blink code
  **        to communicate during operation */
  Blink_LED();
}






