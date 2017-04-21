/*************************************************
** FILE: DCM_Functions
** This file contains the DCM filter functions
** These functions take the accel/magn/gyro data
** and (via the DCM filter) output filtered 
** Euler angles.+
**************************************************/



/*************************************************
** UpdateTime
** Update the time state
** Delta time (s) is used to determine the state
** estimate in the filter.
*/
void Update_Time( void )
{
  float temp = (float) (TIME_RESOLUTION / (TIME_SR+1.0) );

  /* Ensure sample rate */
	//while( (TIME_FUPDATE - g_control_state.timestamp) < (temp) ) {}

  /* Update delta T */
  g_control_state.timestamp_old = g_control_state.timestamp;
  g_control_state.timestamp     = TIME_FUPDATE; 
  if( g_control_state.timestamp_old > 0 ) { g_control_state.G_Dt = (float) ( (g_control_state.timestamp - g_control_state.timestamp_old) / TIME_RESOLUTION ) ; }
  else { g_control_state.G_Dt = 0.0f; }
}


/*************************************************
** Init_Rotation_Matrix
** Initialize the DCM rotation matrix using 
** euler angles 
*/
void Init_Rotation_Matrix(float m[3][3], float yaw, float pitch, float roll)
{
  float c1 = cos(roll);
  float s1 = sin(roll);
  float c2 = cos(pitch);
  float s2 = sin(pitch);
  float c3 = cos(yaw);
  float s3 = sin(yaw);

  /* Euler angles, right-handed, intrinsic, XYZ convention
  ** (which means: rotate around body axes Z, Y', X'')  */
  m[0][0] = c2 * c3;
  m[0][1] = c3 * s1 * s2 - c1 * s3;
  m[0][2] = s1 * s3 + c1 * c3 * s2;

  m[1][0] = c2 * s3;
  m[1][1] = c1 * c3 + s1 * s2 * s3;
  m[1][2] = c1 * s2 * s3 - c3 * s1;

  m[2][0] = -s2;
  m[2][1] = c2 * s1;
  m[2][2] = c1 * c2;
}


/******************************************************************
** FUNCTION: DCM_Filter
** There are 4 parts to the DCM filter
**   1. Matrix_Update    - Update the DCM
**   2. Normalize        - Normalize the DCM
**   3. Drift_Correction - Correct for drift in orientation 
**   4. Get_Euler_Angles - Extract Euler angles from DCM */
void DCM_Filter( void )
{
  float error = 0;
  float renorm = 0;
	
  float TempM[3][3];
	
	float Accel_Vector[3];
  float Accel_magnitude;
  float Accel_weight;
  
	float Omega_Vector[3];
	float ErrorGain[3];
  float errorRollPitch[3];
  float errorYaw[3];

	float *DCM_Matrix   = &g_dcm_state.DCM_Matrix[0][0];

  /******************************************************************
  ** DCM 1. Update the Direction Cosine Matrix
  ** We set the DCM matrix for this iteration.
  ** We update the states assuming the IMU is 
  ** traveling along the direction described by the
  ** previous iterations DCM orientation.
  ** Apply the feedback gains from the last iteration
  ** in order to account for any drift.
  ******************************************************************/
	
  /* Convert the acceleration values
  ** Note: Values read from sensor are fixed point */
  Accel_Vector[0] = ACCEL_X_SCALED( g_sensor_state.accel[0] );
  Accel_Vector[1] = ACCEL_Y_SCALED( g_sensor_state.accel[1] );
  Accel_Vector[2] = ACCEL_Z_SCALED( g_sensor_state.accel[2] );
  
  /* Apply prop and int gain to rotation
	** Need to convert the Gyro values to radians
	**    Note: Values read from sensor are fixed point */
  Omega_Vector[0] = GYRO_SCALED_RAD( g_sensor_state.gyro[0] ) + g_dcm_state.Omega_I[0] + g_dcm_state.Omega_P[0];
  Omega_Vector[1] = GYRO_SCALED_RAD( g_sensor_state.gyro[1] ) + g_dcm_state.Omega_I[1] + g_dcm_state.Omega_P[1];
  Omega_Vector[2] = GYRO_SCALED_RAD( g_sensor_state.gyro[2] ) + g_dcm_state.Omega_I[2] + g_dcm_state.Omega_P[2];

  /* Update the state matrix
  ** We are essentially applying a rotation */
  g_dcm_state.DCM_Matrix[0][0] = g_control_state.G_Dt * (g_dcm_state.DCM_Matrix[0][1]*Omega_Vector[2] - g_dcm_state.DCM_Matrix[0][2]*Omega_Vector[1]) + g_dcm_state.DCM_Matrix[0][0];
  g_dcm_state.DCM_Matrix[0][1] = g_control_state.G_Dt * (g_dcm_state.DCM_Matrix[0][2]*Omega_Vector[0] - g_dcm_state.DCM_Matrix[0][0]*Omega_Vector[2]) + g_dcm_state.DCM_Matrix[0][1];
  g_dcm_state.DCM_Matrix[0][2] = g_control_state.G_Dt * (g_dcm_state.DCM_Matrix[0][0]*Omega_Vector[1] - g_dcm_state.DCM_Matrix[0][1]*Omega_Vector[0]) + g_dcm_state.DCM_Matrix[0][2];
  g_dcm_state.DCM_Matrix[1][0] = g_control_state.G_Dt * (g_dcm_state.DCM_Matrix[1][1]*Omega_Vector[2] - g_dcm_state.DCM_Matrix[1][2]*Omega_Vector[1]) + g_dcm_state.DCM_Matrix[1][0];
  g_dcm_state.DCM_Matrix[1][1] = g_control_state.G_Dt * (g_dcm_state.DCM_Matrix[1][2]*Omega_Vector[0] - g_dcm_state.DCM_Matrix[1][0]*Omega_Vector[2]) + g_dcm_state.DCM_Matrix[1][1];
  g_dcm_state.DCM_Matrix[1][2] = g_control_state.G_Dt * (g_dcm_state.DCM_Matrix[1][0]*Omega_Vector[1] - g_dcm_state.DCM_Matrix[1][1]*Omega_Vector[0]) + g_dcm_state.DCM_Matrix[1][2];

	

  /******************************************************************
  ** DCM 2. Normalize DCM
  ** We must normalize the DCM matrix
  ** After each update in order to keep 
  ** each vector in the DCM orthogonal
  ******************************************************************/
  
  /* Determine vector overlap
  ** Each vector should be orthogonal */
  error = -Vector_Dot_Product(&g_dcm_state.DCM_Matrix[0][0],&g_dcm_state.DCM_Matrix[1][0]) * 0.5; 
  
  /* temp = V .* e */
  Vector_Scale( &g_dcm_state.DCM_Matrix[1][0], error, &TempM[0][0] ); 
  Vector_Scale( &g_dcm_state.DCM_Matrix[0][0], error, &TempM[1][0] ); 
  
  /* temp = temp .* DCM[0][:] */
  Vector_Add( &TempM[0][0], &g_dcm_state.DCM_Matrix[0][0], &TempM[0][0] );
  Vector_Add( &TempM[1][0], &g_dcm_state.DCM_Matrix[1][0], &TempM[1][0] );
   
  /* Force orthogonality */
  Vector_Cross_Product( &TempM[0][0], &TempM[1][0], &TempM[2][0] ); 
  
  /* Normalize each vector
  ** DCM[i][:] = temp ./ ( 0.5*(3 - sum(temp.^2)) )
  ** Note that the sum of the DCM vectors should have length of 1 */
	
  renorm = .5 *(3 - Vector_Dot_Product(&TempM[0][0],&TempM[0][0]));
  Vector_Scale( &TempM[0][0], renorm, &g_dcm_state.DCM_Matrix[0][0] );
  
  renorm = .5 *(3 - Vector_Dot_Product(&TempM[1][0],&TempM[1][0]));
  Vector_Scale( &TempM[1][0], renorm, &g_dcm_state.DCM_Matrix[1][0] );
  
  renorm = .5 *(3 - Vector_Dot_Product(&TempM[2][0],&TempM[2][0])); 
  Vector_Scale( &TempM[2][0], renorm, &g_dcm_state.DCM_Matrix[2][0] );



  /******************************************************************
  ** DCM 3. Drift_Correction
  ** Drift correction basically looks at the difference in 
  ** the orientation described by the DCM matrix and the
  ** orientation described by the current acceleration vector.
  ** Essentially, the acceleration is the input and the DCM
  ** matrix is the current state. 
  ** NOTE: We are applying a drift correction by adjusting the
  **       proportional and integral feedback. So, this will not
  **       have an effect until the next iteration!
  ******************************************************************/

  /* Roll and Pitch
  ** Calculate the magnitude of the accelerometer vector 
	** Scale to gravity */
	Accel_magnitude = sqrt( Vector_Dot_Product( &Accel_Vector[0], &Accel_Vector[0] ) ) / GRAVITY;
	
  /* Dynamic weighting of accelerometer info (reliability filter)
	** Weight for accelerometer info (<0.5G = 0.0, 1G = 1.0 , >1.5G = 0.0) */
  Accel_weight = constrain(1 - 2*abs(1 - Accel_magnitude),0,1); 

  /* Adjust the ground of reference 
  ** errorRP = accel x DCM[2][:] 
  ** The error is essentially the amount that the DCM 
  ** vector, the vector which describes our current orientation
  ** in space, and the current acceleration vector differ. The accel
  ** vector is naturally very noisy, but it is our input for each cycle 
  ** and serves as our state estimate. Therefore, we scale the error 
  ** by a integral and proportional gain in each cycle */
  Vector_Cross_Product( &Accel_Vector[0], &g_dcm_state.DCM_Matrix[2][0], &errorRollPitch[0] ); 
	
  Vector_Scale( &errorRollPitch[0], Kp_ROLLPITCH*Accel_weight, &g_dcm_state.Omega_P[0] );
  Vector_Scale( &errorRollPitch[0], Ki_ROLLPITCH*Accel_weight, &ErrorGain[0] );
  Vector_Add( g_dcm_state.Omega_I, ErrorGain, g_dcm_state.Omega_I );     

  /* Note:
  ** Roll and pitch have been lumped here, to simplify the math
  ** since the orientation is only described by a 3-dim point. 
  ** We could split the roll/pitch gains (since the roll is 
  ** described by DCM[2][1-2] and pitch is described by DCM[2,0])
  ** however, we would have to be clever about how we apply the
  ** acceleration vector */

  /* YAW
  ** We make the gyro YAW drift correction based on compass magnetic heading 
  ** The yaw is an estimate, and will drift towards some equilibrium as time 
  ** progresses. However, presuming the drift is not constant towards any 
  ** particular direction, this should be ok */
 
  Vector_Scale( &g_dcm_state.DCM_Matrix[2][0], g_dcm_state.DCM_Matrix[0][0], errorYaw ); /* Applys the yaw correction to the XYZ rotation of the aircraft, depeding the position. */
  
  /* Update the proportional and integral gains per yaw error */
  Vector_Scale( &errorYaw[0], Kp_YAW, &ErrorGain[0] ); /* proportional of YAW. */
  Vector_Add( g_dcm_state.Omega_P, ErrorGain, g_dcm_state.Omega_P ); /* Adding  Proportional. */
  
  Vector_Scale( &errorYaw[0], Ki_YAW, &ErrorGain[0] ); /* Adding Integrator */
  Vector_Add( g_dcm_state.Omega_I, ErrorGain, g_dcm_state.Omega_I ); /* Adding integrator to the Omega_I */


  
  /******************************************************************
  ** DCM 4. Extract Euler Angles from DCM
  ** Calculate the euler angles from the orintation 
  ** described by the DCM matrix.
  ** DCM[2][:] is essentially a vector describing the
  ** orientation of the IMU in space.
  ******************************************************************/
  
  /* Pitch Conventions (set in config):
  ** Range: -90:90
  ** With PITCH_ROT_CONV==1 : 
  ** PITCH_O:1 - Pitch orientation #1. Angle x-axis w/ Horizontal Plane  +Rot:Aft-Down    0:Nadir0/Zenith down. +90:Aft down   -90:Fore down
  ** PITCH_O:2 - Pitch orientation #2. Angle y-axis w/ Horizontal Plane  +Rot:Port-Down   0:Fore/Aft down       +90:Port down  -90:Starboard down
  ** PITCH_O:3 - Pitch orientation #3. Angle z-axis w/ Horizontal Plane  +Rot:Nadir-Down  0:Fore/Aft down       +90:Nadir down -90:Zenith down */
  switch ( PITCH_O )
  {
    case 1 :
      g_sensor_state.pitch = -PITCH_ROT_CONV*f_asin( g_dcm_state.DCM_Matrix[2][0] );
      break;
    case 2 : 
      g_sensor_state.pitch = -PITCH_ROT_CONV*f_asin( g_dcm_state.DCM_Matrix[2][1] );
      break;
    case 3 :
      g_sensor_state.pitch = -PITCH_ROT_CONV*f_asin( g_dcm_state.DCM_Matrix[2][2] );
      break;
  }
  
  /* Define roll orientation convention (set in config):
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
  switch ( ROLL_O )
  {
    case 1 :
      g_sensor_state.roll = -ROLL_ROT_CONV*f_atan2( g_dcm_state.DCM_Matrix[2][0], -ROLL_ZREF*g_dcm_state.DCM_Matrix[2][1] );
      break;
    case 2 :
      g_sensor_state.roll = -ROLL_ROT_CONV*f_atan2( g_dcm_state.DCM_Matrix[2][0], -ROLL_ZREF*g_dcm_state.DCM_Matrix[2][2] ); 
      break;
    case 3 :
      g_sensor_state.roll = -ROLL_ROT_CONV*f_atan2( g_dcm_state.DCM_Matrix[2][1], -ROLL_ZREF*g_dcm_state.DCM_Matrix[2][2] );
      break;
    case 4 :
      g_sensor_state.roll =  ROLL_ROT_CONV*f_atan2( g_dcm_state.DCM_Matrix[2][1], -ROLL_ZREF*g_dcm_state.DCM_Matrix[2][0] );
      break;
    case 5 :
      g_sensor_state.roll =  ROLL_ROT_CONV*f_atan2( g_dcm_state.DCM_Matrix[2][2], -ROLL_ZREF*g_dcm_state.DCM_Matrix[2][0] );
      break;
    case 6 :
      g_sensor_state.roll =  ROLL_ROT_CONV*f_atan2( g_dcm_state.DCM_Matrix[2][2], -ROLL_ZREF*g_dcm_state.DCM_Matrix[2][1] );
      break;
  }
  g_sensor_state.yaw   =  f_atan2( g_dcm_state.DCM_Matrix[1][0], g_dcm_state.DCM_Matrix[0][0] ); // A faster atan2
}


/*************************************************
** Reset_Sensor_Fusion
** Read every sensor and record a time stamp.
** Init DCM with unfiltered orientation 
** I.e. set inital roll/pitch from inital guess
*       and initialize the DCM arrays.
*/
void Reset_Sensor_Fusion() {
  float temp1[3];
  float temp2[3];
  float xAxis[] = {1.0f, 0.0f, 0.0f};

  /* GET PITCH
  ** Using y-z-plane-component/x-component of gravity vector */
  g_sensor_state.pitch = -f_atan2(g_sensor_state.accel[0], sqrt(g_sensor_state.accel[1] * g_sensor_state.accel[1] + g_sensor_state.accel[2] * g_sensor_state.accel[2]));

  /* GET ROLL
  ** Compensate pitch of gravity vector */
  Vector_Cross_Product( g_sensor_state.accel, xAxis, temp1);
  Vector_Cross_Product( xAxis, temp1, temp2);
  
	/* Normally using x-z-plane-component/y-component of compensated gravity vector
  ** roll = atan2(temp2[1], sqrt(temp2[0] * temp2[0] + temp2[2] * temp2[2]));
  ** Since we compensated for pitch, x-z-plane-component equals z-component: */
  g_sensor_state.roll = f_atan2(temp2[1], temp2[2]);

  /* GET YAW */
  g_sensor_state.yaw = 0;

  /* Init rotation matrix */
  Init_Rotation_Matrix(g_dcm_state.DCM_Matrix, g_sensor_state.yaw, g_sensor_state.pitch, g_sensor_state.roll);
}


/*************************************************
** Set_Sensor_Fusion
** Similar to Reset_Sensor_Fusion, except we do not 
** re-initialize the DCM arrays. This is used to 
** reset the roll/pitch values without touching the
** DCM filter.
*/
void Set_Sensor_Fusion()
{
  float temp1[3];
  float temp2[3];
  float xAxis[] = {1.0f, 0.0f, 0.0f};

  /* GET PITCH
  ** Using y-z-plane-component/x-component of gravity vector */
  g_sensor_state.pitch = -f_atan2(g_sensor_state.accel[0], sqrt(g_sensor_state.accel[1] * g_sensor_state.accel[1] + g_sensor_state.accel[2] * g_sensor_state.accel[2]));

  /* GET ROLL
	** Compensate pitch of gravity vector */
  Vector_Cross_Product( g_sensor_state.accel, xAxis, temp1 );
  Vector_Cross_Product( xAxis, temp1, temp2 );
  
	/* Normally using x-z-plane-component/y-component of compensated gravity vector
	** roll = atan2(temp2[1], sqrt(temp2[0] * temp2[0] + temp2[2] * temp2[2]));
	** Since we compensated for pitch, x-z-plane-component equals z-component: */
  g_sensor_state.roll = f_atan2(temp2[1], temp2[2]);

  /* GET YAW */
  g_sensor_state.yaw = 0;
}








