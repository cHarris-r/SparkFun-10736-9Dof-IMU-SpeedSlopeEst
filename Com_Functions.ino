/*************************************************
** FILE: Com_Functions
** This file contains the helper functions which 
** allow us to communicate with either the user
** (via LOG_PORT) or another processor (via COM_PORT)
** It handles data packing, command interpretation, 
** and other such protocol-level operations.
**************************************************/



/*************************************************
** Debug_LogOut
** This function just prints a standard string 
** to the log_port serial port.
** It prints the rpy as well as the timestamp and
** and estimate of the sample rate
*/
void Debug_LogOut(void)
{
	/* Output Euler angles as strings to LOG_PORT */
	
	String imuLog = ""; // Create a fresh line to log
	
  imuLog += "Time: " + String( g_control_state.timestamp ) + ", "; // Add time to log string
  imuLog += "DT: " + String( g_control_state.G_Dt,5 ) + ", ";
	imuLog += "SR: " + String( (1/g_control_state.G_Dt),5 ) + ", "; // Add delta time to log string

  imuLog += "Roll 1:" + String( TO_DEG( g_sensor_state.roll ),5 ) + ", ";
  imuLog += "Pitch:" + String( TO_DEG( g_sensor_state.pitch ),5 ) + ", ";
	//imuLog += "Yaw:" + String( TO_DEG( g_sensor_state.yaw ),5 ) + ", ";
  
  //imuLog += "accel:" + String( g_sensor_state.accel[0],5 ) + ", " + String( g_sensor_state.accel[1],5 ) + ", " + String( g_sensor_state.accel[2],5 ) + ", ";
  //imuLog += "gyro:" + String( g_sensor_state.gyro[0],5 ) + ", " + String( g_sensor_state.gyro[1],5 ) + ", " + String( g_sensor_state.gyro[2],5 ) + ", ";
 
	imuLog += "\r\n"; // Add a new line
	LOG_PORT.print( imuLog ); // Print log line to serial port
}



/*************************************************
** f_SendData
** We have recieved a request
** The request will be a single character (one Byte)
** which will coorespond to a given type of data being 
** requested by the master */
void f_SendData( int nBytesIn )
{
	int i; 
	unsigned char RequestByte;
  unsigned char Garbage;

	/* uint16_t Packet_nBytes;
	** uint16_t nBytes;
	** uint8_t Buffer[50];
	** uint8_t CheckSum; */
	RESPONSE_TYPE Response;

	/* Some Log Output (usb) */
  LOG_PORT.println("> Recieved " + String(nBytesIn) + " Bytes");

	/* We must read the request and respond appropriately
	** If there is more than one request, we will respond
	** in a FIFO fashion 
	** Each request is a single byte code character
	** A response can be variable length and depends on the 
	** data being requested */
	for( i=0; i<nBytesIn; i++)
	{
		/* Read request from the master */
		RequestByte = COMM_PORT.read(); 

		/* Some Log outputs (usb) */
		LOG_PORT.print("> Request code: ");
		LOG_PORT.println(RequestByte, HEX);

		/* Respond the the request appropriately 
		** The packet architecture allows for mulitple 
		** requestes and responses via FIFO */
		switch( RequestByte )
		{
			/* TO DO:
			**   Need to add cases for things like re-locking and 
			**   other error codes for rhobustness */

      case 0xB1: /* 0xB# : Debug */
        /* Packet type 11
        ** Debug test byte
        ** Data buffer
        **   1 x 16 bit integer
        **   Ints are signed */
        
        /* Some Log outputs (usb) */
        LOG_PORT.print("\tRecieved Debug Request: ");
        LOG_PORT.println(RequestByte, HEX);
        Response.PacketType     = 11;
        Response.Buffer_nBytes  = sizeof(uint8_t)*2*1;
        Response.Packet_nBytes  = sizeof(uint16_t)*2 + sizeof(uint8_t)*(1 + Response.Buffer_nBytes);
        f_WriteIToPacket( &Response.Buffer[0], 0xB1 );
        Response.CheckSum       = f_CheckSum( &Response.Buffer[0], Response.Buffer_nBytes ); 
        f_SendPacket( Response );
        break;

      case 0xB2:
        /* Packet type 12
        ** Debug test 32 bit float
        ** Data buffer
        **   1 x 32 bit float
        **   Float is sent bit for bit */
        LOG_PORT.print("\tRecieved Debug Request: ");
        LOG_PORT.println(RequestByte, HEX);
        Response.PacketType     = 12;
        Response.Buffer_nBytes  = sizeof(uint8_t)*4*1;
        Response.Packet_nBytes  = sizeof(uint16_t)*2 + sizeof(uint8_t)*(1 + Response.Buffer_nBytes);
        f_WriteFToPacket_s32( &Response.Buffer[0], -2.0 );
        Response.CheckSum       = f_CheckSum( &Response.Buffer[0], Response.Buffer_nBytes ); 
        f_SendPacket( Response );
        break;

			case 0xA1:
        /* Packet type 1
        ** Roll pitch yaw data
        ** Data buffer:
        **    3 x 16 bit fixed point floats
        **    Each element is shifted 7 bits
        **    floats are signed */
    
				/* Some Log outputs (usb) */
				LOG_PORT.print("\tRecieved Roll Pitch request ... Case : ");
				LOG_PORT.println(RequestByte, DEC);
				Response.PacketType     = 1;
				Response.Buffer_nBytes  = sizeof(uint8_t)*2*3;
        Response.Packet_nBytes  = sizeof(uint16_t)*2 + sizeof(uint8_t)*(1 + Response.Buffer_nBytes);
        f_WriteFToPacket_u16( &Response.Buffer[sizeof(uint16_t)*0], TO_DEG(g_sensor_state.roll) );
        f_WriteFToPacket_u16( &Response.Buffer[sizeof(uint16_t)*1], TO_DEG(g_sensor_state.pitch) );
        f_WriteFToPacket_u16( &Response.Buffer[sizeof(uint16_t)*2], TO_DEG(g_sensor_state.yaw) );
        Response.CheckSum       = f_CheckSum( &Response.Buffer[0], Response.Buffer_nBytes ); 
        f_SendPacket( Response );
				break;

      case 0xA2:
        /* Packet type 2
        ** Roll pitch yaw data 
        ** Data buffer:
        **    3 x 32 bit floats
        **    floats are packed bit for bit */ 
        /* Some Log outputs (usb) */
        LOG_PORT.print("\tRecieved Roll Pitch request ... Case : ");
        LOG_PORT.println(RequestByte, DEC);
        Response.PacketType     = 2;
        Response.Buffer_nBytes  = sizeof(uint8_t)*4*3;
        Response.Packet_nBytes  = sizeof(uint16_t)*2 + sizeof(uint8_t)*(1 + Response.Buffer_nBytes);
        f_WriteFToPacket_s32( &Response.Buffer[sizeof(uint32_t)*0], TO_DEG(g_sensor_state.roll) );
        f_WriteFToPacket_s32( &Response.Buffer[sizeof(uint32_t)*1], TO_DEG(g_sensor_state.pitch) );
        f_WriteFToPacket_s32( &Response.Buffer[sizeof(uint32_t)*2], TO_DEG(g_sensor_state.yaw) );
        Response.CheckSum       = f_CheckSum( &Response.Buffer[0], Response.Buffer_nBytes ); 
        f_SendPacket( Response );
        break;

        
			default:
				LOG_PORT.println("\t ERROR: I don't understand the request!");
				break;
		}
		
		/* Reset the input buffer */
		RequestByte = 0;
	}
} /* End f_SendData */


/*************************************************
** f_SendPacket
** This code builds the contiguous byte array 
** from the defined "Response" packet then sends 
** the data as a singly stream over the UART line 
*/
void f_SendPacket( RESPONSE_TYPE Response )
{
  uint8_t Packet[100];
  int ret;
  int i;

  /* Initialize the array */
  for( ret=0; ret<100; ret++) Packet[ret] = 0;

  /* Build the transmit packet */
  f_WriteIToPacket( &Packet[0], Response.Packet_nBytes );
  f_WriteIToPacket( &Packet[2], Response.PacketType );
  f_WriteIToPacket( &Packet[4], Response.Buffer_nBytes );
  
  for( i=0; i<Response.Buffer_nBytes; i++ ) Packet[6+i] = Response.Buffer[i];
  Packet[6+Response.Buffer_nBytes] = Response.CheckSum;
  
  for( i=0; i<Response.Packet_nBytes+2; i++ )
  {
    LOG_PORT.print(Packet[i],HEX);
    LOG_PORT.print(" , ");
    COMM_PORT.write(&Packet[i],1);
  }
  LOG_PORT.println();
}

/*************************************************
** f_WriteIToPacket
** This is a helper function which copies an 2 byte integer
** into an array of single bytes 
*/
void f_WriteIToPacket( uint8_t *Packet, uint16_t InputBuffer )
{
  int i;
  int nBytes = sizeof(uint16_t);

  for( i=0; i<nBytes; i++ )
  {
    Packet[i] = (uint8_t)(InputBuffer >> ((nBytes-1-i)*8));
  }
}

/*************************************************
** f_WriteFToPacket_u16
** This is a helper function which copies an 4 byte float
** into an array of single bytes 
** Because the master (the C200) uses half precision floats, 
** we cannot send the full 4 byte float. 
** Instead, we:
**    1) Convert the 4 byte float into a 2 byte unsigned integer
**    2) Pack the 2 byte integer into an array of single bytes
** When the data is recieved by the master, it is converted to 
** a 2 byte float. 
** This function assumes that we are sending signed floats. 
** If we can assume that the data is unsigned, we would be able
** to shift an additional bit. This could be implemented as another
** packet "Type" 
*/
void f_WriteFToPacket_u16( unsigned char *Packet, float Input )
{
  uint16_t hpFloat = 0;
  int nBytes = 2;
  int i;

  /* Convert 4 byte float into a 2 byte unsigned int */
  hpFloat = (uint16_t)(Input * pow(2,7) + 0.5);

  /* Pack the 2 byte int into a byte array */
  for( i=0; i<nBytes; i++ )
  { 
    Packet[i] = (uint8_t)( hpFloat >> ((nBytes-1-i)*8) );
  }
}

/*************************************************
** f_WriteFToPacket_s32
** This function writes a float to the packet 
** in a bit for bit fashion. Ie. we pack the float
** as it is stored in memory
*/
void f_WriteFToPacket_s32( unsigned char *Packet, float Input )
{
  unsigned char *p_byte;
  int nBytes = 4;
  int i;
  
  p_byte =(unsigned char *)&Input;

  /* Pack the 32 bytes int into a byte array */
  for( i=0; i<nBytes; i++ ) { Packet[i] = ( p_byte[nBytes-1-i] ); }
}



/*************************************************
** f_Handshake
** The Handshake code waits for the TI board to 
** initiate the handshake. Handshake is initiated 
** by recieving any character(s). Once Initiated, 
** we must send the baud lock character "a" or "A" 
** and then wait for the confirmation charaacter 
*/
void f_Handshake( void )
{
	/* ASCII 'A' :: DEC:65 HEX:0x41 */
	/* ASCII 'a' :: DEC:97 HEX:0x61 */
	const char BaudLockChar = 'a';
	/* 0xAA in Binary: 1010 1010 */
	uint8_t ConfirmChar = 0xAA; 
	/* 0x7E in Binary 0111 1110 */
	uint8_t FailChar = 0xAB;
	
	/* Rx/Tx variabels */
	uint8_t IncomingByte;
	int     nBytesIn;
 
  LOG_PORT.print("> Using BaudLockChar (int):");
  LOG_PORT.println(BaudLockChar,DEC);
  LOG_PORT.print("> Using ConfirmChar (int):");
  LOG_PORT.println(ConfirmChar,DEC);
  LOG_PORT.print("> Using FailChar (int):");
  LOG_PORT.println(FailChar,DEC);
	
	/* We continue to attempt a handshake
	** until there is a lock */
	while( g_control_state.g_BaudLock==false )
	{
		/* Some Log Output (usb) */
		LOG_PORT.println("> Beginning Handshake");

		/* Wait for initiation 
		** Master can send any character(s)
		** NOTE: The first data sent from the master 
		**       is assumed to be garbage */
		while( Serial.available()==0 ) {} 
		
    LOG_PORT.println("> Recieved Initiazation");
    
		/* Clear the input buffer 
		** Since the data sent from the master will be garbage,
		** we need to clear the buffer to ensure further reads
		** are not corrupted.
		** We delay a few ms to ensure all garbage is in FIFO buffer 
		** This allows for a proper clear */
		delay( 5 );
		nBytesIn = Serial.available();
    LOG_PORT.println("> Clearing " + String(nBytesIn) + " characters from buffer");
		while( nBytesIn-- > 0 ) { Serial.read(); }
		
		/* Once handshake is initated by the master,
		** we send the lock character 
		** The master should respond with the confirmation 
		** character */
		Serial.print( BaudLockChar ); // Serial.print to Tx pin
    LOG_PORT.println("> BaudLockChar \"" + String(BaudLockChar) + "\" sent");
    
		/* We delay a few ms to allow the 
		** master to detect and answer the handshake */
		delay( 5 );
	
		/* Read incomming characters 
		** If confirmation character is detected,
		** toggle baud lock variable */
		while( Serial.available()==0 ) {} 
		nBytesIn = Serial.available(); /* nBytes should == 1 */
		if( nBytesIn>0 ) { IncomingByte = Serial.read(); }
		
		/* Some Log Output (usb) */
		LOG_PORT.println("> Recieved " + String(nBytesIn) + " Bytes");
		LOG_PORT.print("  Character (int): ");
		LOG_PORT.println(IncomingByte, DEC);

		/* If confirmation character detected, Baud is locked 
		** Reply with confimation character to end handshake
		** If confirmation character is not detected, lock failed
		** We then send an error character */
		if( IncomingByte==ConfirmChar ) 
		{  
			/* Baud lock successful */
      LOG_PORT.println("> Baud Lock Successful");
		
			/* Toggle Boud lock */
			g_control_state.g_BaudLock = true; 
		
			/* Reply with confimation char to 
			** complete the handshake with the master */
			Serial.print( ConfirmChar );
      LOG_PORT.println("> Confirmation Character Sent");
		}
		else 
		{
			/* Baud Lock failed */
      LOG_PORT.println("> Baud Lock Fail");

      /* Clear input buffer */
      delay( 5 );
      nBytesIn = Serial.available();
      LOG_PORT.println("> Clearing " + String(nBytesIn) + " characters from buffer");
      while( nBytesIn-- > 0 ) { Serial.read(); }
    
			/* Reply with Error char
			** If the Baud lock truely failed, then
			** the master will likely not understand 
			** this response at all. This is more of 
			** a symbolic check */
      //Serial.print( FailChar );
      //Serial.print( IncomingByte );
      LOG_PORT.println("> Fail Character Sent");
		}
		/* Reset Input Buffer */
		IncomingByte = 0;
	}
} /* End f_Handshake */


/*************************************************
** f_CheckSum 
** This function gets a simple checksum for
** the response packet. This checksum simply summs
** the response buffer (modulated to keep within 1 byte)
** This allows for proper data transmission 
*/
uint8_t f_CheckSum( unsigned char *p_Buffer, uint16_t nBytes )
{
	int i;
	uint8_t checksum = 0; 
	
	/* Our check sum is a simmple summation */
	for( i=0; i<nBytes; i++) { checksum += p_Buffer[i]; }
	
	return( checksum & 0xFF );
} /* End f_CheckSum */




