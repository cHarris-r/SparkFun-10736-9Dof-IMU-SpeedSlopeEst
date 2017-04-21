/*******************************************************************
** Tyedefs 
********************************************************************/


/*
** TYPE: DCM_STATE_TYPE
** This type is used to hold the DCM 
** specific arrays and variables */
typedef struct
{
  float Omega_P[3]; 
  float Omega_I[3];
  float DCM_Matrix[3][3];
} DCM_STATE_TYPE;

/*
** TYPE: SENSOR_STATE_TYPE
** This type is used to hold the sensor
** variables */
typedef struct
{
  float yaw;
  float pitch;
  float roll;
  float mag[3];
  float accel[3];
  float gyro[3];
} SENSOR_STATE_TYPE;

/*
** TYPE: CONTROL_STATE_TYPE
** This type is used to hold all control
** variables. */
typedef struct
{
  unsigned long timestamp;
  unsigned long timestamp_old;
  float G_Dt;

  /* Serial communication globals */
  //static bool g_BaudLock; /* Used to set baud rate */
  bool g_BaudLock; /* Used to set baud rate */
  
  /* LED state globals */
  bool      g_LedState; /* Used to set LED state */
  uint32_t  g_LastBlinkTime;   /* Used to set LED state */
} CONTROL_STATE_TYPE;


/* 
** TYPE: RESPONSE_TYPE
** Used to store temporary resonse data
** for responding to request from master */
typedef struct
{
  uint16_t       Packet_nBytes;  /* Length of entire packet, minus this variable, in bytes */
  uint16_t       PacketType;     /* Type code of packet */
  uint16_t       Buffer_nBytes;  /* Length of data buffer in bytes (0-50) */
  unsigned char  Buffer[50];     /* Data buffer */
  unsigned char  CheckSum;       /* CheckSum of data buffer only */
} RESPONSE_TYPE;

