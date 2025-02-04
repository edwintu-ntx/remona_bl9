/* -------------------------------------------------------------------------- *
 *  Module for Turbochef Oven Software - Phoenix Project                      *
 *  uart1.c                                                                   *
 *                                                                            *
 *  Low level driver firmware for access to Serial Port #2                    *
 *                                                                            *
 * -------------------------------------------------------------------------- *
 *                                                                            *
 * ************************************************************************** *
 * FileName:        uart2.c                                                   *
 * Dependencies:    uart2.h                                                   *
 * Processor:       dsPIC33FJ256GP710                                         *
 * Compiler:        MPLAB XC16                                                 *
 * Company:         Turbochef Technology Incorporated                         *
 *                                                                            *
 * Copyright ?2013 Turbochef Technology Inc.  All rights reserved.           *
 *                                                                            *
 * PIO PortF                                                                  *
 * 049 U2RX/CN16/RF4   SERIAL_RX                                              *
 * 050 U2TX/CN18/RF5   SERIAL_TX                                              *
 *                                                                            *
 * 040 U2CTS/RF12      SERIAL_DSR (output)                                    *
 * 039 U2RTS/RF13      SERIAL_DTR (input)                                     *
 * ---------------------------------------------------------------------------*/

// ****************************************************************************
// Ver 0.0
// History:
// --------
// Ver		Who made change(s)		Description
// ---------------------------------------------------------------
//  0.1   JMWare          4/20/2013  - Making document better fit software
//                        guidance standards
//  0.1A  JMWare          10/16/2013 - Created uart1.x from uart2.x
//                        to allow protocol transfer using RS-485
//  0.2   JMWare          04/15/2014 - Additional support for expanded packet size
//                        to allow for Boot loader packets.
//
// ****************************************************************************
#include "p33FJ256GP710.h"
#include "common.h"
#include "uart2.h"
#include "FIRESerialProtocol.h"               // Allow access to "FIRE" specific information for data protocol
#include "i2c1.h"

/* Module Variables */

// Transmit:
volatile char U2_txbuf[ TXBUFSIZE ];     	    // The  Transmit  buffer
volatile unsigned int  u2_txbufdatasize;	    // Amount of data in buffer
volatile unsigned int  u2_txbufindex;	 		    // index  into U2_txbuf[] for next byte to send...
volatile int  u2_txBufSending = 0;  	 	    // Sending/idle - transmitting a packet flag (1-Sending 0-Idle/receive)

// Receive:
volatile unsigned char U2_rxbuf[ RXBUFSIZE ]; // The Recieve buffer
volatile unsigned int u2_rxIndex;				      // Index to next address to store incoming data
volatile int 		      u2_rxPacketReady;			  // When == TRUE ;; Packet is full and ready to parse/test/move/... new command packet

#define NumIncomingPacket2	(NumHeaderBytes2 + NumCommandBytes2)

extern volatile int  iRxTm;                   // Receive timer...

volatile unsigned int  RxStateMachine2;        // The receive state location for the state machine...

extern volatile int commTimer;

static unsigned char *pWorkBuffer2;
unsigned char scReadBuff2[24000];
int indicator2 = 0;
int	savefwdata2 = 0;
int writefwtoEE2 = 0;
int receivefwbinary2 = 0;
int totalbytesreceived2 = 0;

//char Ed_txbuf[16];
//char Ed_txbuf[] = "Hello Remona";
//void UART_Write(char data);

//void UART_Write(char data){
//	while(!U2TRMT);
//	U2TXREG = data;
//}

/* ------------------------------------------------------------------------- *
 *                                                                           *
 *  U2_Init()     Initilizes serial port #2                                  *
 *                                                                           *
 *  RETURNS: None                                                            *
 *                                                                           *
 * ------------------------------------------------------------------------- */
void U2_Init2( void )
{
    U2MODE             = 1;            // From Phoenix Code:
    U2STA              = 0;            //?
    U2_TX_PIN          = 0;            //?
    U2_RTS_PIN         = 0;            //?
    U2_RX_PIN          = 0;            //?
    U2_TX_TRIS         = 0;            // Output
    U2_RX_TRIS         = 1;            // Input
    U2_RTS_TRIS        = 1;            // Inputs
    U2_CTS_TRIS        = 1;            //?
//    U2MODEbits.BRGH    = 0;            // High Speed
    U2BRG              = BAUDRATEREG1; // Set baud rate
    U2MODEbits.UEN     = 2;            // TX, RX, CTS & RTS under hardware control
    U2STAbits.URXISEL  = 0;            // IRQ on any byte received
    U2STAbits.UTXISEL0 = 0;            // IRQ when TX NOT full
    U2STAbits.UTXISEL1 = 0;            //?
    IFS1bits.U2TXIF    = 0;            // Reset TX ISR Flag
    IFS1bits.U2RXIF    = 0;            //?
    IFS4bits.U2EIF     = 0;            //?
    IEC1bits.U2TXIE    = 1;            // Enable TX Interrupt
    IEC1bits.U2RXIE    = 1;            // Enable RX Interrupt
    U2MODEbits.RTSMD   = 1;            // Flow-control=0   Simplex=1
    U2MODEbits.STSEL   = 0;            // 1-stop bit
    U2MODEbits.PDSEL   = 0b00;         // 00 = 8-bit data with no Parity
    U2MODEbits.LPBACK  = 0;            // DISABLE loop-back mode
    U2MODEbits.UARTEN  = 1;            // Enable UART
    U2STAbits.UTXEN    = 1;            // TX Enabled, UART controls TX pin
}

/* ------------------------------------------------------------------------- *
 *                                                                           *
 *  void SendPacket(int packetsize)                                          *
 *                                                                           *
 *  Packetsize   = How large is the packet this is being requested to send?  *
 *                                                                           *
 *  U1_txbuf[]    : The data packet being sent                               *
 *  txbufindex    : Index to which byte to send next                         *
 *  txbufdatasize : Count of the number of bytes needed to transmit          *
 *                                                                           *
 *  RETURNS: None                                                            *
 *                                                                           *
 * JMWare:                                                                   *
 * Send packet to Phoenix computer.  Start sending packet,                   *
 * then let interrupt finish sending the rest of the packet!                 *
 * ------------------------------------------------------------------------- */
// -- JMWare: Send packet to Phoenix computer.  Start sending packet, then let interrupt finish...
#if 0
void u2_SendPacket2( int packetsize )
{
  int i;

	while( u2_txBufSending == 1 );                          //return; // Don't allow steping on a currently sending packet

//	for( i = 0; i < 10000; i++ );                           // Small Delay
	for( i = 0; i < 100; i++ ); 
	u2_txBufSending       = 1;								// Show we are sending!
	u2_txbufdatasize      = packetsize + NumHeaderBytes2;	// set the packet size to send (generic) - could be used to send other packets later...if needed.
	u2_txbufindex         = 1;								// Point to first byte to transmit ( after sending byte zero(0) below: )
	U2TXREG               = U2_txbuf[0];					// Send the first byte, let Tx interrupt send all the others...
	u2_txbufdatasize--;	                                    // dec data size counter to show that the first byte has been sent.
}
#endif

/* ------------------------------------------------------------------------- *
 *                                                                           *
 *  _U2TXInterrupt(void)     Handles hardware interrupts when sending a      *
 *                           packet of data.                                 *
 *                                                                           *
 *  Handle UART #2's Transmit interrupt                                      *
 *                                                                           *
 *  U2_txbuf[]    : The data packet being sent                               *
 *  txbufindex    : Index to which byte to send next                         *
 *  txbufdatasize : Count of the number of bytes needed to transmit          *
 *                                                                           *
 *  RETURNS: None                                                            *
 *                                                                           *
 * ------------------------------------------------------------------------- */
void __attribute__((__interrupt__, no_auto_psv)) _U2TXInterrupt( void )
{
  int txtemp;

  if( u2_txbufdatasize == 0)                        // are there any charaters left in buffer?
    {                                               //
        Nop();                                      // NO, do nothing (No transmitter to turn off)
        u2_txBufSending  = 0;						// Show we are finished sending packet!  Ready for Rx
    }                                               //
  else                                              // OK - we have more data to send!
    {                                               //
	    txtemp = U2_txbuf[ u2_txbufindex ];			// Get the next byte
	    u2_txbufindex++;							// increment index in to transmit buffer
	    u2_txbufdatasize--;							// decrement data size to account for sending the next byte, no not need to
	                                                // check for txbufdatasize being zero (to protect decrement to -1) because above
	                                                // 'if' statement checked for that issue.
	    U2TXREG = txtemp;							// Send the data
    }                                               //
                                                    //
  //IFS0bits.U2TXIF = 0;                              // Clear UART2 Transmit Irq flag
  IFS1bits.U2TXIF= 0;								// Clear UART2 Transmit Irq flag

}

/* -------------------------------------------------------------------------  *
 *                                                                            *
 *  uart2_ReadyForNextPacket()   This proceedure clears the SW receive buffer *
 *                               and clears the rxPacketReady flag, also it   *
 *                               sets the index in to the receive buffer to   *
 *                               the beginning of the buffer/array.           *
 *                                                                            *
 *  U2_rxbuf[]        : Serial receive buffer (Cleared in this proceedure)    *
 *  NumIncomingPacket : Total number of bytes in an Command packet            *
 *                                                                            *
 *  RETURNS: None                                                             *
 *                                                                            *
 * -------------------------------------------------------------------------  */
#if 0
void uart2_ReadyForNextPacket2()
{
	int i;

	for(i = 0; i< NumIncomingPacket2; i++)
	{
		U2_rxbuf[u2_rxIndex] = 0;					// Clear the rx array
	}

	u2_rxPacketReady = FALSE;						// Show packet is clear
	u2_rxIndex       = 0;							// Get ready for the next packet
}

#define PacketID2 U2_rxbuf[2]                        // Define where PACKET ID is un the header!
#endif

/* ------------------------------------------------------------------------- *
 *                                                                           *
 *  _U2RXInterrupt(void)     Handles hardware interrupts when receiving a    *
 *                           packet of data.                                 *
 *                                                                           *
 *  Handle UART #2's Receive interrupt                                       *
 *                                                                           *
 *  I1_rxbuf[] :  Has the received packet when rxPacketReady is TRUE         *
 *                                                                           *
 *  RETURNS: None                                                            *
 *                                                                           *
 * ------------------------------------------------------------------------- */
void __attribute__((__interrupt__, no_auto_psv)) _U2RXInterrupt( void )
{
  int rxtemp;                                   	//
                                                    //
  if( U2STAbits.OERR )                              // Hardware Buffer Overflow Error
  {                                                 //
    U2STAbits.OERR = 0;                           	// Clear Error
  }

  // Receive state machine:  0- Nothing; Waiting for header to start!, 0,1- Looking for 0xaa, 0x55  2- The rest of the packet
  while( U2STAbits.URXDA )                        	// While we have data...
  {

   if( receivefwbinary2 )
   {
     rxtemp = U2RXREG ;

      if( rxtemp == 0xA5 )
      {
          if( indicator2 == 0xA5 )
          {
              savefwdata2 = 1;
			  goto EXITIF;
  		  }
      }
      else if( rxtemp == 0x5A )
      {
          if( indicator2 == 0x5A )
          {
              savefwdata2  = 0;
			  writefwtoEE2 = 1;
  		  }
	  }


	  if( savefwdata2 )
	  {
	  	  *pWorkBuffer2++ = rxtemp;
	       totalbytesreceived2++;
	  }

	EXITIF:
      indicator2 = rxtemp;
   }
   else
   {
    // ------------
    // Idle here, removing data from the serial stream while we are processing
    // a packet. Any serial data sent in while the main routine is processing
    // packets will be discarded.
    // ------------
    if( u2_rxPacketReady == TRUE )
    {
      while( U2STAbits.URXDA )
      {
        rxtemp = U2RXREG;
      }

      return;
    }

    // -------------
    // OK - Normal processing of an incomming packet
    // -------------
    rxtemp    = U2RXREG;                          // Get a byte from the data stream...
    iRxTm     = RESET_TIME;                       // reset timer so it does not reset the communications stream!
    commTimer = 60;                               // Default = 200ms

    switch( RxStateMachine2 )
    {
      case 0: // Waiting for 0xaa
              u2_rxIndex = 0;                     // 0xaa goes in @ index 0

              if( rxtemp == 0xaa )
              {
                U2_rxbuf[u2_rxIndex] = rxtemp;    // Save to indexed rxbuffer
                RxStateMachine2       = 1;         // Move to next state
                u2_rxIndex++;                     // Increment receive array index
              }

              break;

      case 1:
              if( rxtemp == 0x55 )
              {
                U2_rxbuf[u2_rxIndex] = rxtemp;    // Save to indexed rxbuffer
                RxStateMachine2       = 2;
                u2_rxIndex++;
              }

              break;

      case 2: //
              // Receive all other bytes here:
              //
              U2_rxbuf[u2_rxIndex] = rxtemp;      // Save to indexed rxbuffer

              if( u2_rxIndex < 250 )
                  u2_rxIndex++; // array index overbounds protection
              else
              {
                // LOG ERROR!                     // Could log an error here if needed.
              }

              break;
    }
   }
  }

//  IFS0bits.U1RXIF = 0;                              // Clear UART1 Recive Irq flag
  IFS1bits.U2RXIF = 0;                              // Clear UART2 Recive Irq flag

}


 /* ------------------------------------------------------------------------- *
 *                                                                            *
 *  _U1ErrInterrupt()     Handles hardware interrupts when receiving a        *
 *                        error when dealing with the serial port.            *
 *                                                                            *
 *  Handle UART #1's Receive interrupt                                        *
 *                                                                            *
 *  RETURNS: None                                                             *
 *                                                                            *
 * -------------------------------------------------------------------------  */
void __attribute__((__interrupt__, no_auto_psv)) _U2ErrInterrupt( void )
{
  if( U2STAbits.OERR )                    // Hardware Buffer Overflow Error
  {
      U2STAbits.OERR  = 0;                // Clear Error
  }                                       //
  IFS4bits.U2EIF      = 0;                // Clear UART1 Error Irq flag
}

/* -------------------------------------------------------------------------  *
 *                                                                            *
 *  initbuffers()     This proceedure initialized all registers               *
 *                    necessary for the start and archival of the bin file    *
 *                    to be transmitted over the uart                         *
 *                                                                            *
 *  RETURNS: none                                                             *
 *                                                                            *
 * -------------------------------------------------------------------------  */
#if 0
void initbuffers2( void )
{
    indicator2 = 0;
	writefwtoEE2 = 0;
	savefwdata2 = 0;
	pWorkBuffer2 = scReadBuff2;
	totalbytesreceived2 = 0;
    receivefwbinary2 = 1;
}
#endif

/* -------------------------------------------------------------------------  *
 *                                                                            *
 *  isDownloadDone()  This proceedure starts the process to write to ee       *
 *                    the bin file already stored in scReadBuff.              *
 *                    totalbytesreceived is decremented to take out the 0xA5  *
 *                                                                            *
 *  RETURNS: true when the entire bin file has been written to ee             *
 *                                                                            *
 * -------------------------------------------------------------------------  */
#if 0
int isDownloadDone2( void )
{
   if( writefwtoEE2 )
   {
	 // Disable UART so that anything on the uart will not disturb ee writes
     IEC1bits.U2TXIE    = 0;            // Disable TX Interrupt
     IEC1bits.U2RXIE    = 0;            // Disable RX Interrupt
     U2MODEbits.UARTEN  = 0;            // Disable UART
     writefwtoEE2        = 0;

     totalbytesreceived2--;				// cut out the last byte 0x5A saved that was actually a delimiter-
                                        //- and not part of the bin file
	 totalbytesreceived2--;				// cut out another byte since we are going to write "B" into the -
                                        //-0th position after the entire bin file has been writte to the EE
     // start from position 1 of scReadBuff
	 if( i2c1_EEWriteArray( 0xA8, 1, &scReadBuff2[1], totalbytesreceived2 ) ) 
		return 1;
   }
   return 0;
}
#endif

/* -------------------------------------------------------------------------  *
 *                                                                            *
 *  writeBLFS()  Well actually just write "B" in the 0th address of the EE    *
 *                                                                            *
 * -------------------------------------------------------------------------  */
#if 0
void writeBLFS2( void )
{
	i2c1_EEWriteArray( 0xA8, 0, &scReadBuff2[0], 4 );
}
#endif

/* -------------------------------------------------------------------------  *
 *                                                                            *
 *  clearBLFS()  Well actually just write "B" in the 0th address of the EE    *
 *                                                                            *
 * -------------------------------------------------------------------------  */
#if 0
void clearBLFS2( void )
{
	unsigned char ucBuff2[5] = {0,0,0,0};

	i2c1_EEWriteArray( 0xA8, 0, &ucBuff2[0], 4 );
}
#endif

/* -------------------------------------------------------------------------  *
 *                                                                            *
 *  iswritefwtoEE()   This proceedure returns the status of writefwtoEE,      *
 *                    thereby, indicated whether the bin file download over   *
 *                    uart is complete and write to EE can proceed            *
 *                                                                            *
 *                                                                            *
 *  RETURNS: Current state of writefwtoEE                                     *
 *                                                                            *
 * -------------------------------------------------------------------------  */
#if 0
int iswritefwtoEE2( void )
{
	return writefwtoEE2;
}
#endif