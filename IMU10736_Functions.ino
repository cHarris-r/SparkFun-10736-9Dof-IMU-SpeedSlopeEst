
/*************************************************
** FILE: IMU10736_Functions
** This file contains some MPU 9250 (HW specific)
** functions. Specifically, for initializing and 
** reading the sensor registeres
**************************************************/

/*************************************************
** NOTES on orientation for the 10736 IMU
**   Terms: 
**     Fore:       (Front) Edge oposite of the power port
**     Aft:        (Rear) Edge of the power port
**     Starboard:  (Right) Edge with reset switch
**     Port:       (Left) Edge oposite of reset switch
**     Zenith:     (Up) Clean face of board 
**     Nadir:      (Down) Populated face of board
**   Contrary to the silk, the axis are positioned as follows:
**     +x is Fore,       -x is Aft
**     +y is Starboard,  -y is Port
**     +z is Zenith,     -z is Nadir
**   This means, placing the board on a flat surface with the 
**   unpopulated side (Zenith) down will result in an acceleration 
**   of about -256 (1xg) for accel[2] (z) since the acceleration  
**   from gravity with be acting along -z.
**************************************************/



/*************************************************
** Read_Sensors 
** This function calls the read functions
** which read the data registers from the 
** sensors.
*/
void Read_Sensors() 
{
  if(GYRO_ON) { Read_Gyro(); }  
  if(ACCEL_ON){ Read_Accel(); } 
  if(MAGN_ON) { Read_Magn(); } 
}

/*************************************************
** Init_IMU
** This function initiates I2C communicatino with
** the gyro/magn/accel sensors. Further, it initializes
** the sensors (setting sampling rate, data format, etc.)
*/
bool Init_IMU(void)
{
  /* Initialize sensors */
  delay(50);  // Give sensors enough time to start
  I2C_Init();
  if(GYRO_ON) { Gyro_Init();  }
  if(ACCEL_ON){ Accel_Init(); }
  if(MAGN_ON) { Magn_Init();  }
  
  /* Read sensors, initialize DCM algorithm */
  delay(20);  // Give sensors enough time to collect data
	Read_Sensors();
  Reset_Sensor_Fusion();
  
  return true; // Return success
}

/*************************************************
** I2C_Init 
** This function initiates I2C communication 
** with the gyro/magn/accel. This board only
** has the one wire port available, so here we
** simply initate wire.
*/
void I2C_Init() 
{ Wire.begin(); }

/*************************************************
** Accel_Init 
** This function initializes the accelerometer
*/
void Accel_Init()
{
	/* Set measurement mode */
  Wire.beginTransmission(ACCEL_ADDRESS);
  WIRE_SEND(ACCEL_POWER);
  WIRE_SEND(0b00001000);   // Measurement mode
  Wire.endTransmission();
  delay(5);
	
	/* Set data resolution */
  Wire.beginTransmission(ACCEL_ADDRESS);
  WIRE_SEND(ACCEL_FORMAT);
  WIRE_SEND(0b00001000);    // Set to full resolution
  Wire.endTransmission();
  delay(5);
  
  /* Set sample rate 
	** Accepts 0x06-0x0F
	** 0x06:6.25  0x07:12.5  0x08:25   0x09:50    0x0A:100
	** 0x0B:200   0x0C:400   0x0D:800  0x0E:1600  0x0F:3200 
	** Hz  */
  Wire.beginTransmission(ACCEL_ADDRESS);
  WIRE_SEND(ACCEL_RATE);
  //WIRE_SEND(0x0F);
  WIRE_SEND(0x0E);
  Wire.endTransmission();
  delay(5);
}

/*************************************************
** Read_Accel 
** This function reads x/y/z data from the 
** accelerometer
*/
void Read_Accel()
{
  int i = 0;
  uint8_t buff[6];
  
	/* Send address to read from */
  Wire.beginTransmission(ACCEL_ADDRESS); 
  WIRE_SEND(ACCEL_DATA); 
  Wire.endTransmission();
  
	/* Read 6 bytes */
  Wire.beginTransmission(ACCEL_ADDRESS);
  Wire.requestFrom(ACCEL_ADDRESS, 6); 
  while(Wire.available()) 
  { 
    buff[i] = WIRE_RECEIVE();
    i++;
  }
  Wire.endTransmission();
  
	/* Unpack Data */
  if (i == 6)
  {
    /* No multiply by -1 for coordinate system transformation here, because of double negation:
    ** We want the gravity vector, which is negated acceleration vector. */
    g_sensor_state.accel[0] = (int16_t)((((uint16_t) buff[3]) << 8) | buff[2]);  // X axis (internal sensor y axis)
    g_sensor_state.accel[1] = (int16_t)((((uint16_t) buff[1]) << 8) | buff[0]);  // Y axis (internal sensor x axis)
    g_sensor_state.accel[2] = (int16_t)((((uint16_t) buff[5]) << 8) | buff[4]);  // Z axis (internal sensor z axis)
  }
  else
  {
    Serial.println("!ERR: reading accelerometer");
  }
}

/*************************************************
** Magn_Init 
** This function initializes the magnetometer
*/
void Magn_Init()
{
	/* Set continuous sampling mode */
  Wire.beginTransmission(MAGN_ADDRESS);
  WIRE_SEND(MAGN_MODE);  
  WIRE_SEND(0x00); 
  Wire.endTransmission();
  delay(5);
	
  /* Set sample rate 
	** Offset 2 bits ( xxx###xx )
	** Accepts 000-110
	** 000:0.75  001:1.5  010:3   011:7.5
	** 100:15    101:30   110:75 
	** Hz  */
  Wire.beginTransmission(MAGN_ADDRESS);
  WIRE_SEND(MAGN_CONFIG_A);
  WIRE_SEND(0b00011000); /* 75 Hz */
  Wire.endTransmission();
  delay(5);
}

/*************************************************
** Read_Magn 
** This function reads x/y/z data from the 
** magnetometer.
*/
void Read_Magn()
{
  int i = 0;
  uint8_t buff[6];
 
  /* Send address to read from */
  Wire.beginTransmission(MAGN_ADDRESS); 
  WIRE_SEND(MAGN_DATA_MSBX);  
  Wire.endTransmission();
  
	/* Read 6 bytes */
  Wire.beginTransmission(MAGN_ADDRESS); 
  Wire.requestFrom(MAGN_ADDRESS, 6); 
  while(Wire.available())
  { 
    buff[i] = WIRE_RECEIVE(); 
    i++;
  }
  Wire.endTransmission();
  
	/* Unpack data */
  if (i == 6)
  {
    /* 9DOF Razor IMU SEN-10736 using HMC5883L magnetometer
    ** Data 2 byte width, MSB byte first then LSB; Y and Z reversed: X, Z, Y */
    g_sensor_state.mag[0] = -1 * (int16_t)(((((uint16_t) buff[4]) << 8) | buff[5]));  // X axis (internal sensor -y axis)
    g_sensor_state.mag[1] = -1 * (int16_t)(((((uint16_t) buff[0]) << 8) | buff[1]));  // Y axis (internal sensor -x axis)
    g_sensor_state.mag[2] = -1 * (int16_t)(((((uint16_t) buff[2]) << 8) | buff[3]));  // Z axis (internal sensor -z axis)
  }
  else
  {
    Serial.println("!ERR: reading magnetometer");
  }
}

/*************************************************
** Gyro_Init 
** This function initializes the gyroscope
*/
void Gyro_Init()
{
  /* Power up reset defaults */
  Wire.beginTransmission(GYRO_ADDRESS);
  WIRE_SEND(GYRO_POWER);
  WIRE_SEND(0x80);
  Wire.endTransmission();
  delay(5);
  
  /* Full-scale range of the gyro sensors
	** Set LP filter bandwidth B0-B2  
	**  NOTE:The LPF determines Finternal! 
	**  DLPF_CFG:LPF BW(Hz): Finternal(Hz)
	**  000:256:8  001:188:1  010:98:1  011:42:1
	**  100:20:1   101:10:1   110:5:1 
  ** FS_SEL: Full scale range, B3-B4, only accepts 11 */
  Wire.beginTransmission(GYRO_ADDRESS);
  WIRE_SEND(GYRO_DLPF);
  //WIRE_SEND(0x1B);  // DLPF_CFG = 3:LPF 42Hz,Fi 1kHz , FS_SEL = 3:+-2000deg/sec
  WIRE_SEND(0x03);  // DLPF_CFG = 3:LPF 42Hz,Fi 1kHz , FS_SEL = 3:+-2000deg/sec
  Wire.endTransmission();
  delay(5);
  
  /* Set sample rato divider
	** Fsample = Finternal / (divider+1), where Finternal is either 1kHz or 8kHz 
	** 8 bit field (0-255) */
  Wire.beginTransmission(GYRO_ADDRESS);
  WIRE_SEND(GYRO_RATE);
  //WIRE_SEND(0x0A);  //  SMPLRT_DIV = 10 (90Hz w/ Fi=1kHz)
  WIRE_SEND(0x00);  //  SMPLRT_DIV = 0 (1000Hz w/ Fi=1kHz)
  Wire.endTransmission();
  delay(5);

  /* Set clock to PLL with z gyro reference */
  Wire.beginTransmission(GYRO_ADDRESS);
  WIRE_SEND(GYRO_POWER);
  //WIRE_SEND(0x00); /* Internal occilator */
  WIRE_SEND(0x01); /* Gyro x ref */
  Wire.endTransmission();
  delay(5);
}

/*************************************************
** Read_Gyro 
** This function reads x/y/z data from the 
** gyroscope.
*/
void Read_Gyro()
{
  int i = 0;
  uint8_t buff[6];
  
	/* Send address to read from */
  Wire.beginTransmission(GYRO_ADDRESS); 
  WIRE_SEND(GYRO_DATA);
  Wire.endTransmission();
  
	/* Read 6 bytes */
  Wire.beginTransmission(GYRO_ADDRESS);
  Wire.requestFrom(GYRO_ADDRESS, 6);  
  while(Wire.available()) 
  { 
    buff[i] = WIRE_RECEIVE();
    i++;
  }
  Wire.endTransmission();
  
	/* Unpack Data */
  if (i == 6)
  {
    g_sensor_state.gyro[0] = -1 * (int16_t)(((((uint16_t) buff[2]) << 8) | buff[3]));    // X axis (internal sensor -y axis)
    g_sensor_state.gyro[1] = -1 * (int16_t)(((((uint16_t) buff[0]) << 8) | buff[1]));    // Y axis (internal sensor -x axis)
    g_sensor_state.gyro[2] = -1 * (int16_t)(((((uint16_t) buff[4]) << 8) | buff[5]));    // Z axis (internal sensor -z axis)
  }
  else
  {
    Serial.println("!ERR: reading gyroscope");
  }
}
