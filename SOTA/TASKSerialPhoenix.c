/* -------------------------------------------------------------------------- *
 *  Module for Turbochef Oven Software - Phoenix Project                      *
 *  TASKSerialPhoenix.c                                                       *
 *                                                                            *
 *  This task handles communication from this controller to the Phoenix UI    *
 *  computer.                                                                 *
 *                                                                            *
 *                                                                            *
 * ************************************************************************** *
 * FileName:        TASKSerialPhoenix.c                                       *
 * Dependencies:    fire.h                                                    *
 *                  <><><><><<.h                                              *
 * Processor:       dsPIC33FJ256GP710                                         *
 * Compiler:        MPLAB C30                                                 *
 * Company:         Turbochef Technology Incorporated                         *
 *                                                                            *
 * Copyright ?2013 Turbochef Technology Inc.  All rights reserved.           *
 *                                                                            *
 *                                                                            *
 * Changes:                                                                   *
 * 10/18/2013   JMWare  Move from uart2 to uart1 (485) port                   *
 *                                                                            *
 * ---------------------------------------------------------------------------*/
#include "p33FJ256GP710.h"
#include "common.h"
#include "timer1.h"
#include "timer23.h"
#include "i2c1.h"
#include "TASKio.h"
#include "TASKHeater.h"
#include "uart1.h"
#include "TASKPhoenix.h"
#include "FIRESerialProtocol.h"

//etu added
#include <string.h>
#include <xc.h>
#include <libpic30.h>
#include <stdio.h>

#include "modbusSlave.h"

//                      [9] = {'X','6','0','8','1','3',0};	// Version number. Changes to Heat-engine, blowers, HF160606(PM)
                                                            // WAS XB0617. Setting to release 16/06/17 HF160705

// see if it can transmit 8 characters to Phoenix
unsigned char SWVERSION [9] = {'1','6','0','8','3','0', 0};	// Version number. Changes to Heat-engine, blowers, HF160606(PM)
                                                            // no blower amps factored in and modulation 208/240 : 15:30
                                                            // 1P818P remove modulation from 208VAC per DC
                                                            // 1P726P increase 208VAC current (change (-)20 to (-)17 HF160826
//etu added
unsigned char newCmd = 0;

extern volatile char U2_txbuf[ 10 ]; 
extern unsigned char TxData[16];
extern unsigned char RxData[8];

extern int iTopMeasTempF;
extern int iBotMeasTempF;

int sendData_Flag = 1;
unsigned char gRxTemp; 
extern unsigned char gHoldingWCnt;
extern unsigned char gHoldingRCnt;
extern unsigned char gInputWCnt;
extern unsigned char gInputRCnt;
extern unsigned char gSingleRegWCnt;
extern unsigned char gSingleRegRCnt;
extern unsigned char gSingleCoilWCnt;
extern unsigned char gSingleCoilRCnt;
extern unsigned char gMulCoilsRCnt;
extern unsigned char gMulCoilsWCnt;

extern unsigned char rx2_Rcv;

extern void u2_SendDemo( int packetsize );
extern uint8_t readHoldingRegs (void);
extern uint8_t readInputRegs (void);
extern uint8_t UART2_Read( void);

//etu
                                                            
extern volatile unsigned char U1_txbuf[ TXBUFSIZE ];      	// Allow access to the Transmit character buffer
extern volatile unsigned char U1_rxbuf[ RXBUFSIZE ];      	// Allow access to the receive buffer
extern volatile unsigned int  RxStateMachine;        		// The receive state location for the state machine...
extern volatile unsigned int u1_rxIndex;				    // Index to next address to store incoming data
extern volatile int u1_txBufSending;	  	        	  	// Sending/idle - transmitting a packet flag ... 1=sending 0=idle
extern volatile int serialTimer;                          	// Serial Timer - used to timeout packets
extern volatile int commTimer;

int OperationMode       = 0;
int DownLoadInProgress  = 0;
int newdldrequest       = 1;

union CmdPacket   CmdPk;
union StatPacket  StatPk;

/* ------------------------------------------------------------------------- *
 *                                                                           *
 *  CopyRxBufferTOCommandStructure()                                         *
 *      This proceedure moves data from the Receive buffer to the command    *
 *  structure.                                                               *
 *                                                                           *
 *                                                                           *
 *  RETURNS: None                                                            *
 *                                                                           *
 * ------------------------------------------------------------------------- */
void CopyRxBufferTOCommandStructure( void )
{
  int i, j;
  i = 6;
  // Copy command data TO Command structure
  // Setup where to start copy and limit:
	for(j = 6; j < (NumHeaderBytes + NumCommandBytes); j++)
	{
    	CmdPk.byte[i] = U1_rxbuf[j];        // Move data from rx buffer to command array!
		i++;																// increment index in to transmit buffer
	}
}

// --------------------------------------- Real header cksum -----------------------------------------
// Calculate checksum that when on receive is calculated as:  SUM(0, 1, 2, 3, 4) % 0x100 = 0x0000 !!!
// ----------------------------------------------------------------------------------------------------
unsigned char CkSum( void )
{
  int i = 0;
  i = i + U1_txbuf[0];  // Add in start byte 0
  i = i + U1_txbuf[1];  // Add in start byte 1
  i = i + U1_txbuf[2];  // Add in packet type:  See Packet types as defined in fire.h
  i = i + U1_txbuf[3];  // Add in DataSize.  Size of the rest of the packet excluding bytes 0 thr 4
  i = 0x100 - (i % 0x100);
  return( (unsigned char) i);
}

/* ------------------------------------------------------------------------- *
 *                                                                           *
 *  generateChecksum()                                                       *
 *      This function returns a 16-bit number that when run through the      *
 *  checksum (for a receive packet)routine will return 0x0000.  This is      *
 *  based on the Fletcher's checksum.  Since the data in the serial stream   *
 *  can force the oven to operate in different modes, a better checksum      *
 *  was implemented.                                                         *
 *                                                                           *
 *                                                                           *
 *  numBytes  : The number of bytes in the packet to run the checksum on     *
 *  *data     : Pointer to packet data array                                 *
 *  retType   : 1 - Returns checksum for transmit routine (ckSumTransmit)    *
 *              2 - Returns checksum for receive check packet routine        *
 *                  (ckSumReceive)                                           *
 *                                                                           *
 *  RETURNS: Checksum to store in the transmit packet or examine in the      *
 *  receive packet.                                                          *
 *                                                                           *
 * ------------------------------------------------------------------------- */
unsigned int generateChecksum( char retType, volatile unsigned char *data, int numBytes )
{
  unsigned int sum1, sum2;
  unsigned char myData;
  int index;
  unsigned char c0,c1;

  sum1 = 0;                                                         // Clear sum's
  sum2 = 0;

  for( index = 0; index < numBytes; index++ )
  {
    myData = data[index];                                           // Get next data byte
    sum1   = ( sum1 + myData );

    if( sum1 >= 255 ) sum1 -= 255;                                  // This is faster than MOD(%)

    sum2   = ( sum2 + sum1 );

    if( sum2 >= 255 ) sum2 -= 255;
  }

  // Turn checksum in to something to run checksum ON!
  // For the receive packet test, this makes the checksum
  // routine return 0x0000, and allows the checksum to be
  // checked for correctness too!
  if( retType == ckSumTransmit )
  {
    // Make result 0x0000:
    c0 = 0xff - ( ( sum1 + sum2) % 0xff );
    c1 = 0xff - ( ( sum1 + c0  ) % 0xff );

    sum2 = c1;
    sum1 = c0;
  }

  // Create code to return checksum of 0x0000 when it is checked on the receiving end
  return (unsigned int)( ( sum2 << 8 ) | sum1 );
}

 // --------------------------SEND VERSION TO PHOENIX ------------------------------- -
 //                                                                                   -
 // Send the version string for this firmware to the Phoenix UI                       -
 // NOTE: Version string is stored in common.h and is a variable length string        -
 //                                                                                   -
 // --------------------------------------------------------------------------------- -
 void send_Version( void )
 {
	int i = 0;
	int j = 5;
	unsigned char f0,f1;
	unsigned int cs1;

//memset (U1_txbuf, 0, sizeof(U1_txbuf));

	// Create the HEADER:
	// Step 1.  Start bytes:
	U1_txbuf[0] = 0xaa;
	U1_txbuf[1] = 0x55;

	// Step 2. Enter the status packet ID:
	U1_txbuf[2] = versionPacket;                                    // Packet is a Status packet

	// Step 3. DataSize.  Size of the packet excluding the header.
	U1_txbuf[3] = 2;                                                // Checksum ONLY!

    // -- End of Header -----------------------------------------
    // Now - put in version string payload!
    while ( i < 6)
		U1_txbuf[ j++ ] = SWVERSION[ i++ ];

  	i++;
  	U1_txbuf[j] = 0;                                                // Insert terminating null
  	U1_txbuf[3] = i + 2;                                            // String length (+ null) + checksum bytes(2)

  	// Step 4. Checksum.  Sum bytes 0, 1, 2, 3 -> store in 4
	U1_txbuf[4] = CkSum();

  	j++;

  	// (J) Length of the string including the end NULL
  	cs1 = generateChecksum( ckSumTransmit, &U1_txbuf[0], ( j ) );   // (i+2)

  	// Get the high and low order byte:
  	f0 = cs1 & 0xff;
  	f1 = (cs1 >> 8) & 0xff;

  	// Save in transmit packet
  	U1_txbuf[j]     = f0;                                           // Save checksum in transmit buffer
  	U1_txbuf[j+1]   = f1;

//	u1_SendPacket( U1_txbuf[ 3 ] );									// Transmit buffer out Serial Port...(if we can!)
	u2_SendPacket( U1_txbuf[ 3 ] );									// Transmit buffer out Serial Port...(if we can!)

}

/* ---------------------- STATUS PACKET RESPONSE ??? YES ------------------- *
 *                                                                           *
 *  Send_Response()                                                          *
 *      This proceedure assembles the response packet and sets in motion     *
 *  the sending of the response packet data.  After this, then next step is  *
 *  to try and send the packet.                                              *
 *                                                                           *
 *  U2_txbuf[]     : The data packet being sent                              *
 *  txbufindex     : Index to which byte to send next                        *
 *  NumStatusBytes : Count of the number of bytes needed to transmit         *
 *                                                                           *
 *  RETURNS: None                                                            *
 *                                                                           *
 * ------------------------------------------------------------------------- */
void Send_Response( void )
{
	int i = 6;
	int j;
	unsigned char f0,f1;
	unsigned int cs1;

	// ----------------------------------------------------------
	// Create the HEADER:

	// Step 1.  Start bytes:
	U1_txbuf[0] = 0xaa;
	U1_txbuf[1] = 0x55;

	// Step 2. Enter the status packet ID:
	U1_txbuf[2] = statusPacket;                                     // Packet is a Status packet

	// Step 3. DataSize.  Size of the packet excluding the header.
	U1_txbuf[3] = NumStatusBytes;

	// Step 4. Checksum.  Sum bytes 0, 1, 2, 3 -> store in 4
	U1_txbuf[4] = CkSum();

	// clear filler:
	U1_txbuf[5] = 0;                                                // Filler byte

	// -- End of Header -----------------------------------------

	// Copy response data TO transmit buffer
	for( j = 0; j < NumStatusBytes; j++ )                           // Copy RESPONSE packet data to transmit buffer
	{
    	U1_txbuf[i] = StatPk.byte[j+6];
		i++;														// increment index in to transmit buffer
	}

  // ------------------------------  Generate checksum: ----------------------------------------------- .
  //                     Get Tx Cksum   Buffer         Number of bytes to run checksum on!              .
  //                                                   the magic number '-2' is to look only at         .
  // __________________________________________________the_packet_not_the_two_checksum_byte_locations__ .
  cs1 = generateChecksum( ckSumTransmit, &U1_txbuf[0], ( NumHeaderBytes + NumStatusBytes - 2 ) );

  // Get the high and low order byte:
  f0 = cs1 & 0xff;
  f1 = ( cs1 >> 8 ) & 0xff;
  i  = NumHeaderBytes + NumStatusBytes - 2;                         // Points to checksum location in transmit packet

  // Save in transmit packet
  U1_txbuf[i]   = f0;                                               // Save checksum in transmit buffer
  U1_txbuf[i+1] = f1;

	// Setup auto SEND
//	u1_SendPacket( NumStatusBytes );								// Transmit buffer out Serial Port...(if we can!)
	u2_SendPacket( NumStatusBytes );								// Transmit buffer out Serial Port...(if we can!)

}

/* ------------------------------------------------------------------------- *
 *                                                                           *
 *  handleCmdPacket()     Handle the Command Packet type...                  *
 *                                                                           *
 *                                                                           *
 *                                                                           *
 *  RETURNS: None                                                            *
 *                                                                           *
 * ------------------------------------------------------------------------- */
void handleCmdPacket( void )
{
 	CopyRxBufferTOCommandStructure();                               // Move the packet into the structure for use by oven

	if( CmdPk.BinXfer == 0xA5 )
	{															    // bin file download request received
		if( !DownLoadInProgress && newdldrequest )
		{
			// Think about disabling all non essential stuff below:
			// T1CONbits.TON    = 0;           // disable Timer 1
			// T2CONbits.TON    = 0;           // disable Timer 2
			// T3CONbits.TON    = 0;           // disable Timer 3
			// AD1CON1bits.ADON = 0; 		   // turn off ADC module

			DownLoadInProgress = 1;
			iTasksActivated    = 0;
			newdldrequest      = 0;
			EnableRapidBlink( 1 );
			initbuffers();
		}

		return;
	}
	else if( CmdPk.BinXfer == 0x5A )
	{
		newdldrequest = 1;
	}

	if( u1_txBufSending == 0 && U1STAbits.UTXBF == 0 )              // We are not sending a packet and the hw buffer is emput
	{
		Send_Response();                                            // Create and send the RESPONSE packet
		uart1_ReadyForNextPacket();                                 // Clear the input buffer for the next COMMAND packet
	}
}

/* ------------------------------------------------------------------------- *
 *                                                                           *
 *  TASKSerialPhoenix()     Task that handles the Serial Protocol from       *
 *                          Phoenix UI to SAGE                               *
 *                                                                           *
 *                                                                           *
 *  RETURNS: None                                                            *
 *                                                                           *
 * ------------------------------------------------------------------------- */
void TASKSerialPhoenix( void )
{
    	if( commTimer == 0 )
      	{
        	// USART not seeing any data, assume we have received a packet - receive it!
        	u1_rxPacketReady  =  TRUE;
        	RxStateMachine    =  0;
        	u1_rxIndex        =  0;
        	commTimer         = -1;
      	}

		if( u1_rxPacketReady == TRUE )
		{
			serialTimer = SERIALTIMEOUT;                            // RESET the serial timer to it's max trigger level.
		   	// Did we receive a COMMAND packet? ( not ping, not boot, not version, not resetSOTA )
  			if( PacketID == commandPacket )
  		  	{
    			// determine if packet is good and process if OK
            	if( U1_rxbuf[0] != 0xAA || U1_rxbuf[1] != 0x55 ||
            	    ( generateChecksum( ckSumReceive, &U1_rxbuf[0], ( NumHeaderBytes + NumCommandBytes ) ) ) )
    			{	// Something is WRONG with this packet - abort and wait for re-try
      		  		uart1_ReadyForNextPacket();                     // Clear the input buffer for the next COMMAND packet
    		  	}
    		  	else
    		  	{
   		   			handleCmdPacket();                              // Handle and send response packet
    		  	}
  		  	}
    		else if( PacketID == requestVersion )
    		{	//Handle the request for SAGE's version string
  			  	if( u1_txBufSending == 0 && U1STAbits.UTXBF == 0 )  // We are not sending a packet and the hw buffer is emput
   			  	{
          		   	send_Version();                                 // Create and send the RESPONSE packet
          		   	uart1_ReadyForNextPacket();                     // Clear the input buffer for the next COMMAND packet
          		}
    		}
			else
			{
				uart1_ReadyForNextPacket();                         // Clear the input buffer for the next COMMAND packet
			}
		}

		//------------------------------------------------------------------------------------------------------------
		// Check packet timeout. Oven has not seen a command packet
		if( serialTimer <= 0 ) OperationMode = 0; else OperationMode = 1;
}


/* ------------------------------------------------------------------------- *
 *                                                                           *
 *  TASKSerialPhoenix()     Task that handles the Serial 2 Mod Bus Protocol  *
 *                                                                           *
 *                                                                           *
 *  RETURNS: None                                                            *
 *                                                                           *
 * ------------------------------------------------------------------------- */
void TASKSerial2Modbus( void )
{
	int i,j;
	int cmdLen;
	//init MeasTemp
//	iTopMeasTempF = 388; //0x0184
//	iBotMeasTempF = 412; //0x019c

//if (rx2_Rcv)
//{
	if (!newCmd)
	{
		if( SLAVE_ID == UART2_Read())
		{	// recevive new command,
			RxData[0] = SLAVE_ID;

			gRxTemp = UART2_Read(); // check function code to determin length of cmd
			switch (gRxTemp)
			{
				case 0x01:	//Read Coils
				case 0x03:	//Read Holding
				case 0x04:	//Read Input Registers
				case 0x05:	//Write Single Coil
				case 0x06:	//Write Single Registers
					RxData[1] = gRxTemp;
					cmdLen = 6;
					break;
				
				case 0x0f:	//Write Multiple Coils
					RxData[1] = gRxTemp;
					cmdLen = 9;
					break;
			
				case 0x10:	//Write Multiple Registers
					RxData[1] = gRxTemp;
					cmdLen = 9;
					break;

				default:
					break;

			} // end of switch
		}	

		for (i =0; i < cmdLen; i++)
			RxData[2+i] = UART2_Read();
		newCmd = 0;
	}
		
//	iTopMeasTempF = 388;
//	iBotMeasTempF = 412;

	if (cmdLen > 0)
	{
		if( RxData[0] == SLAVE_ID )
		{
		    switch( RxData[1] )
		    {
		      case 0x03:  
					readHoldingRegs();
					u2_SendPacket( gHoldingRCnt);
					for( i = 0; i < 1000; i++ ){                      // Small Delay
						for( j = 0; j < 10000; j++ );
					}
	              break;
	
	      	case 0x04:
					readInputRegs();
					u2_SendPacket( gInputRCnt);
					for( i = 0; i < 1000; i++ ){                      // Small Delay
						for( j = 0; j < 10000; j++ );
					}
	              break;

			case 0x01:
					readCoils();
					u2_SendPacket( gSingleCoilRCnt);
					for( i = 0; i < 1000; i++ ){                      // Small Delay
						for( j = 0; j < 10000; j++ );
					}
				break;

			case 0x02:
					readInputs();
					u2_SendPacket( gInputRCnt);
					for( i = 0; i < 1000; i++ ){                      // Small Delay
						for( j = 0; j < 10000; j++ );
					}
				break;

			case 0x06:
					writeSingleReg();
					u2_SendPacket( gSingleRegWCnt);
					for( i = 0; i < 1000; i++ ){                      // Small Delay
						for( j = 0; j < 10000; j++ );
					}
				break;

			case 0x10:
					writeHoldingRegs();
					u2_SendPacket( gHoldingWCnt);
					for( i = 0; i < 1000; i++ ){                      // Small Delay
						for( j = 0; j < 10000; j++ );
					}

				break;

			case 0x05:
					writeSingleCoil();
					u2_SendPacket( gSingleCoilWCnt);
					for( i = 0; i < 1000; i++ ){                      // Small Delay
						for( j = 0; j < 10000; j++ );
					}
				break;

			case 0x0F:
					writeMultiCoils();
					u2_SendPacket(gMulCoilsWCnt);
					for( i = 0; i < 1000; i++ ){                      // Small Delay
						for( j = 0; j < 10000; j++ );
					}
				break;

			default:
				  break;
	
			}
			cmdLen = 0;
		}

	rx2_Rcv = 0;
	}
//  } //end of if(rx2_Rcv)	

}