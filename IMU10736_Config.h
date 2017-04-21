
/******************************************************************
** User defined 
** General Settings
** Here we define the general settings for the system
** These can be changed to suit the users needs
******************************************************************/


/* Communication Parameters
*******************************************************************/
/* Serial Port Configuration */
#define DEBUG 1 
#define LOG_PORT_BAUD 115200
#define COMM_PORT_BAUD 9600

/* The LED can be used for external debugging */
#define UART_BLINK_RATE 100



/* DCM parameters
*******************************************************************/

/* DCM gain */
#define Kp_ROLLPITCH 0.1f //0.02f
#define Ki_ROLLPITCH 0.00005f //0.00002f
#define Kp_YAW 1.2f
#define Ki_YAW 0.00002f

/* "1G reference" used for DCM filter and accelerometer calibration */
#define GRAVITY 256.0f 


/* Define pitch orientation convention
** Range: -90:90
** With PITCH_ROT_CONV==1 :
** PITCH_O:1 - Pitch orientation #1. Angle x-axis w/ Horizontal Plane  +Rot:Aft-Down    0:Nadir0/Zenith down. +90:Aft down   -90:Fore down
** PITCH_O:2 - Pitch orientation #2. Angle y-axis w/ Horizontal Plane  +Rot:Port-Down   0:Fore/Aft down       +90:Port down  -90:Starboard down
** PITCH_O:3 - Pitch orientation #3. Angle z-axis w/ Horizontal Plane  +Rot:Nadir-Down  0:Fore/Aft down       +90:Nadir down -90:Zenith down */
#define PITCH_O  1

/* Pitch rotation convention
** This sets the sign of rotation for pitch
**   1 :+Towards +axis
**  -1 :+Towards -axis */
#define PITCH_ROT_CONV 1

/* NOTE:
** Since Pitch is -90:+90, there is no "PITCH_ZREF"
** as this would have the exact role as flipping 
** "PITCH_ROT_CONV" */

/* Roll rotation convention
** This sets the sign of rotation for roll
**   1 :+Towards +axis
**  -1 :+Towards -axis */
#define ROLL_ROT_CONV 1

/* Roll rotation reference direction
** This will reverse the direction of the "0" reference for roll
**   1 :"0" is in direction of +axis
**  -1 :"0" is in direction of -axis */
#define ROLL_ZREF 1





/******************************************************************
** User dependant
** DON'T TOUCH! 
** These are created from user variables
*******************************************************************/

/* Define roll orientation convention
** We should only be using 3,4,5 orientations, but they are all available for hacking.
** Range: -180:180
** With ROLL_ROT_CONV==1  ROLL_ZREF==1
** ROLL_O:1 - Roll orientation #1. Rotation around z-axis (Nadir-Zenith) +Rot:Aft-Port    0:Port down     +-180:stbd down 
** ROLL_O:2 - Roll orientation #2. Rotation around y-axis (Port-Stbd)    +Rot:Aft-Nadir   0:Nadir down    +-180:Zenith down 
** ROLL_O:3 - Roll orientation #3. Rotation around x-axis (Fore-Aft)     +Rot:Port-Nadir  0:Nadir down    +-180:Zenith down  
** ROLL_O:4 - Roll orientation #4. Rotation around z-axis (Nadir-Zenith) +Rot:Aft-Port    0:Aft down      +-180:Fore down  
** ROLL_O:5 - Roll orientation #5. Rotation around y-axis (Port-Stbd)    +Rot:Aft-Nadir   0:Aft down      +-180:Fore down  
** ROLL_O:6 - Roll orientation #6. Rotation around x-axis (Fore-Aft)     +Rot:Port-Nadir  0:Port down     +-180:Stbd down  
** WARNING: ROLL should be determined from pitch orientation, not set manually */
#if    PITCH_O==1  /* Rotation around y-axis  +Rot:Aft-Down    0:Nadir0/Zenith down. +90:Aft down -90:Fore down */
  #define ROLL_O 3 
#elif  PITCH_O==2  /* Rotation around z-axis  +Rot:Port-Down   0:Fore/Aft down       +90:Port down -90:Starboard down */
  /* This is offset 90, need to find another option */
  #define ROLL_O 5
#elif PITCH_O==3   /* Rotation around y-axis  +Rot:Nadir-Down  0:Fore/Aft down       +90:Nadir down -90:Zenith down */
  /* This is offset 90, need to find another option */
  #define ROLL_O 4
#endif

/* Set which sensors to read */
#define ACCEL_ON 1
#define GYRO_ON  1
#define MAGN_ON  0 /* We removed support for the mag in the DCM! */







/******************************************************************
** HW specific
** SparkFun "9DOF Razor IMU" version "SEN-10736" (HMC5883L magnetometer)
*******************************************************************/

/* COMM Ports 
*******************************************************************/
#define LOG_PORT if(DEBUG)Serial
//#define LOG_PORT if(DEBUG)SERIAL_PORT_USBVIRTUAL
#define COMM_PORT Serial


/* Sampling resolution
*******************************************************************/
/* Set the system sampling rate */
#define TIME_SR         200.0f    /* Warning: depends on sensor values! */

/* Resolution of system time
** Used to set delta T - see Update_Time */
//#define TIME_RESOLUTION 1000.0f
#define TIME_RESOLUTION 1000000.0f

/* TIME_RESOLUTION should match TIME_FUPDATE !! */
//#define TIME_FUPDATE    millis()
#define TIME_FUPDATE    micros()


/* Board LED 
*******************************************************************/
#define HW_LED_PIN 13 


/* Accelerometer I2C addresses (Regeister Map)
******************************************************************/
#define ACCEL_ADDRESS ((int16_t) 0x53) /* 0x53 = 0xA6 / 2 (this is local) */
#define ACCEL_RATE     0x2C            /* Name:Data rate and power mode control BW_RATE  - Access:R/W */
#define ACCEL_POWER    0x2D            /* Name:Power-Saving features control POWER_CTL   - Access:R/W */
#define ACCEL_FORMAT   0x31            /* Name:Data format control                       - Access:R/W */
#define ACCEL_DATA     0x32            /* Name:Start of data registers (6 bytes)         - Access:R   */


/* Magnometer I2C addresses (Regeister Map)
******************************************************************/
#define MAGN_ADDRESS  ((int16_t) 0x1E) /* 0x1E = 0x3C / 2 (this is local) */
#define MAGN_CONFIG_A  0x00            /* Name:Configuration Register A 		- Access:Read/Write */
#define MAGN_CONFIG_B  0x01            /* Name:Configuration Register B 		- Access:Read/Write */
#define MAGN_MODE      0x02            /* Name:Mode Register 								- Access:Read/Write */
#define MAGN_DATA_MSBX 0x03            /* Name:Data Output X MSB Register 	- Access:Read */
#define MAGN_DATA_LSBX 0x04            /* Name:Data Output X LSB Register 	- Access:Read */
#define MAGN_DATA_MSBZ 0x05            /* Name:Data Output Z MSB Register 	- Access:Read */
#define MAGN_DATA_LSBZ 0x06            /* Name:Data Output Z LSB Register 	- Access:Read */
#define MAGN_DATA_MSBY 0x07            /* Name:Data Output Y MSB Register 	- Access:Read */
#define MAGN_DATA_LSBY 0x08            /* Name:Data Output Y LSB Register 	- Access:Read */
#define MAGN_STATUS    0x09            /* Name:Status Register 							- Access:Read */
#define MAGN_ID_A      0x0A            /* Name:Identification Register A 		- Access:Read */
#define MAGN_ID_B      0x0B            /* Name:Identification Register B 		- Access:Read */
#define MAGN_ID_C      0x0C            /* Name:Identification Register C 		- Access:Read */



/* Gyroscope addresses I2C addresses (Regeister Map)
******************************************************************/
#define GYRO_ADDRESS  ((int16_t) 0x68) /* 0x68 = 0xD0 / 2 (this is local) */
#define GYRO_ID        0x00            /* Name:verify identity WHO_AM_I						- Access:R/W */
#define GYRO_RATE      0x15            /* Name:Sample rate divider            		- Access:R/W */
#define GYRO_DLPF      0x16            /* Name:Configures several parameters 	 		- Access:R/W */
#define GYRO_CONFIG    0x17            /* Name:INT_CFG 														- Access:R/W */
#define GYRO_STATUS    0x1A            /* Name:INT_STATUS 												- Access:R/W */
#define GYRO_DATA      0x1D            /* Name:Start of data registers (6 bytes) 	- Access:R   */
#define GYRO_POWER     0x3E            /* Name:PWR_MGM 														- Access:R/W */


/* I2C Macros I2C addresses 
******************************************************************/
#define WIRE_SEND(b) Wire.write((byte) b) 
#define WIRE_RECEIVE() Wire.read() 







/******************************************************************
** Sensor Calibration
*******************************************************************

/* Calibration Macros
******************************************************************/
#define TO_RAD(x) (x * 0.01745329252)  // *pi/180
#define TO_DEG(x) (x * 57.2957795131)  // *180/pi

/* Accelerometer Calibration
******************************************************************/
#define ACCEL_X_MIN ((float) -250)
#define ACCEL_X_MAX ((float) 250)
#define ACCEL_Y_MIN ((float) -250)
#define ACCEL_Y_MAX ((float) 250)
#define ACCEL_Z_MIN ((float) -250)
#define ACCEL_Z_MAX ((float) 250)
#define ACCEL_X_OFFSET ((ACCEL_X_MIN + ACCEL_X_MAX) / 2.0f)
#define ACCEL_Y_OFFSET ((ACCEL_Y_MIN + ACCEL_Y_MAX) / 2.0f)
#define ACCEL_Z_OFFSET ((ACCEL_Z_MIN + ACCEL_Z_MAX) / 2.0f)
#define ACCEL_X_GAIN (GRAVITY / (ACCEL_X_MAX - ACCEL_X_OFFSET))
#define ACCEL_Y_GAIN (GRAVITY / (ACCEL_Y_MAX - ACCEL_Y_OFFSET))
#define ACCEL_Z_GAIN (GRAVITY / (ACCEL_Z_MAX - ACCEL_Z_OFFSET))
#define ACCEL_X_SCALED(x) ( (x - ACCEL_X_OFFSET)*ACCEL_X_GAIN )
#define ACCEL_Y_SCALED(x) ( (x - ACCEL_Y_OFFSET)*ACCEL_Y_GAIN )
#define ACCEL_Z_SCALED(x) ( (x - ACCEL_Z_OFFSET)*ACCEL_Z_GAIN )

/* Magnetometer Calibration
******************************************************************/
#define MAGN_X_MIN ((float) -600)
#define MAGN_X_MAX ((float) 600)
#define MAGN_Y_MIN ((float) -600)
#define MAGN_Y_MAX ((float) 600)
#define MAGN_Z_MIN ((float) -600)
#define MAGN_Z_MAX ((float) 600)
#define MAGN_X_OFFSET ((MAGN_X_MIN + MAGN_X_MAX) / 2.0f)
#define MAGN_Y_OFFSET ((MAGN_Y_MIN + MAGN_Y_MAX) / 2.0f)
#define MAGN_Z_OFFSET ((MAGN_Z_MIN + MAGN_Z_MAX) / 2.0f)
#define MAGN_X_SCALE (100.0f / (MAGN_X_MAX - MAGN_X_OFFSET))
#define MAGN_Y_SCALE (100.0f / (MAGN_Y_MAX - MAGN_Y_OFFSET))
#define MAGN_Z_SCALE (100.0f / (MAGN_Z_MAX - MAGN_Z_OFFSET))

/* Gyroscope Calibration
******************************************************************/
// Gain for gyroscope (ITG-3200)
#define GYRO_GAIN 0.06957 // Same gain on all axes
#define GYRO_AVERAGE_OFFSET_X ((float) 0.0)
#define GYRO_AVERAGE_OFFSET_Y ((float) 0.0)
#define GYRO_AVERAGE_OFFSET_Z ((float) 0.0)
#define GYRO_X_SCALED(x) ( GYRO_SCALED_RAD(x - GYRO_AVERAGE_OFFSET_X) )
#define GYRO_Y_SCALED(x) ( GYRO_SCALED_RAD(x - GYRO_AVERAGE_OFFSET_Y) )
#define GYRO_Z_SCALED(x) ( GYRO_SCALED_RAD(x - GYRO_AVERAGE_OFFSET_Z) )
#define GYRO_SCALED_RAD(x) (x * TO_RAD(GYRO_GAIN)) 














