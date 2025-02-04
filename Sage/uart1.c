/* -------------------------------------------------------------------------- *
 *  Module for Turbochef Oven Software - Phoenix Project                      *
 *  uart1.c                                                                   *
 *                                                                            *
 *  Low level driver firmware for access to Serial Port #1                    *
 *                                                                            *
 * -------------------------------------------------------------------------- *
 *                                                                            *
 * ************************************************************************** *
 * FileName:        uart1.c                                                   *
 * Dependencies:    uart1.h                                                   *
 * Processor:       dsPIC33FJ256GP710                                         *
 * Compiler:        MPLAB C30                                                 *
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
#include "uart1.h"
#include "FIRESerialProtocol.h"               // Allow access to "FIRE" specific information for data protocol
#include "i2c1.h"

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include "xc.h"

//etu
#define UART2_CONFIG_TX_BYTEQ_LENGTH (8+1)
#define UART2_CONFIG_RX_BYTEQ_LENGTH (8+1)

/* Module Variables */

//etu
static unsigned char *volatile rxTail;
static unsigned char *rxHead;
static unsigned char *txTail;
static unsigned char *volatile txHead;
//static _Bool volatile rxOverflowed;
static bool volatile rxOverflowed;

// Transmit:
volatile char U1_txbuf[ TXBUFSIZE ];     	    // The  Transmit  buffer
volatile unsigned int  u1_txbufdatasize;	    // Amount of data in buffer
volatile unsigned int  u1_txbufindex;	 		    // index  into U2_txbuf[] for next byte to send...
volatile int  u1_txBufSending = 0;  	 	    // Sending/idle - transmitting a packet flag (1-Sending 0-Idle/receive)

// Receive:
volatile unsigned char U1_rxbuf[ RXBUFSIZE ]; // The Recieve buffer
volatile unsigned int u1_rxIndex;				      // Index to next address to store incoming data
volatile int 		      u1_rxPacketReady;			  // When == TRUE ;; Packet is full and ready to parse/test/move/... new command packet

#define NumIncomingPacket	(NumHeaderBytes + NumCommandBytes)
//etu added
#define Num2IncomingPacket	(Num2HeaderBytes + Num2CommandBytes)

extern volatile int  iRxTm;                   // Receive timer...

volatile unsigned int  RxStateMachine;        // The receive state location for the state machine...
//etu added

volatile unsigned char U2_rxbuf[ RXBUFSIZE ]; // The Recieve buffer
volatile unsigned int u2_rxIndex;				      // Index to next address to store incoming data
volatile int 	      u2_rxPacketReady;			  // When == TRUE ;; Packet is full and ready to parse/test/move/... new command packet

volatile unsigned int  RxStateMachine2;        // The uart2 receive state location for the state machine...

extern volatile int commTimer;

static unsigned char *pWorkBuffer;
unsigned char scReadBuff[24000];
int indicator = 0;
int	savefwdata = 0;
int writefwtoEE = 0;
int receivefwbinary = 0;
int totalbytesreceived = 0;

//etu added
#include "modbusSlave.h"

unsigned char TxData[16];
unsigned char RxData[16];
int modCmd_Flag = 0;
unsigned char modIndex = 0;
extern unsigned char newCmd;
unsigned char rx2_Rcv = 0;

// Transmit UART2:
volatile unsigned int  u2_txbufdatasize;	    // Amount of data in buffer
volatile unsigned int  u2_txbufindex;	 		// index  into U2_txbuf[] for next byte to send...
volatile int  u2_txBufSending = 0;  	 	    // Sending/idle - transmitting a packet flag (1-Sending 0-Idle/receive)

// Receive UART2:
volatile unsigned char U2_rxbuf[ RXBUFSIZE ]; // The Recieve buffer
//volatile unsigned int u2_rxIndex;				  // Index to next address to store incoming data
//volatile int      u2_rxPacketReady;			  // When == TRUE ;; Packet is full and ready to parse/test/move/... new command packet

extern unsigned char U2_testbuf[ TXBUFSIZE ];

//etu
static uint8_t txQueue[UART2_CONFIG_TX_BYTEQ_LENGTH];
static uint8_t rxQueue[UART2_CONFIG_RX_BYTEQ_LENGTH];

void (*UART2_TxDefaultInterruptHandler)(void);
void (*UART2_RxDefaultInterruptHandler)(void);


/* ------------------------------------------------------------------------- *
 *                                                                           *
 *  U1_Init()     Initilizes serial port #1                                  *
 *                                                                           *
 *  RETURNS: None                                                            *
 *                                                                           *
 * ------------------------------------------------------------------------- */
void U1_Init( void )
{
    U1MODE             = 1;            // From Phoenix Code:
    U1STA              = 0;            //?
    U1_TX_PIN          = 0;            //?
    U1_RTS_PIN         = 0;            //?
    U1_RX_PIN          = 0;            //?
    U1_TX_TRIS         = 0;            // Output
    U1_RX_TRIS         = 1;            // Input
    U1_RTS_TRIS        = 1;            // Inputs
    U1_CTS_TRIS        = 1;            //?
    U1MODEbits.BRGH    = 0;            // High Speed
    U1BRG              = BAUDRATEREG1; // Set baud rate
    U1MODEbits.UEN     = 2;            // TX, RX, CTS & RTS under hardware control
    U1STAbits.URXISEL  = 0;            // IRQ on any byte received
    U1STAbits.UTXISEL0 = 0;            // IRQ when TX NOT full
    U1STAbits.UTXISEL1 = 0;            //?
    IFS0bits.U1TXIF    = 0;            // Reset TX ISR Flag
    IFS0bits.U1RXIF    = 0;            //?
    IFS4bits.U1EIF     = 0;            //?
    IEC0bits.U1TXIE    = 1;            // Enable TX Interrupt
    IEC0bits.U1RXIE    = 1;            // Enable RX Interrupt
    U1MODEbits.RTSMD   = 1;            // Flow-control=0   Simplex=1
    U1MODEbits.STSEL   = 0;            // 1-stop bit
    U1MODEbits.PDSEL   = 0b00;         // 00 = 8-bit data with no Parity
    U1MODEbits.LPBACK  = 0;            // DISABLE loop-back mode
    U1MODEbits.UARTEN  = 1;            // Enable UART
    U1STAbits.UTXEN    = 1;            // TX Enabled, UART controls TX pin
}

/* ------------------------------------------------------------------------- *
 *                                                                           *
 *  U1_Init()     Initilizes serial port #2                                  *
 *                                                                           *
 *  RETURNS: None                                                            *
 *                                                                           *
 * ------------------------------------------------------------------------- */
void U2_Init( void )
{
 	U2MODE             = 1;//1= two stop bit, 0= one stop bit  // From Phoenix Code:
    U2STA              = 0;            //?
    U2_TX_PIN          = 0;            //?
//    U2_RTS_PIN         = 0;            //?
    U2_RX_PIN          = 0;            //?
    U2_TX_TRIS         = 0;            // Output
    U2_RX_TRIS         = 1;            // Input
//    U2_RTS_TRIS        = 1;            // Inputs
//    U2_CTS_TRIS        = 1;            // Output
#if 1
//	U2MODEbits.BRGH    = 0;            // High Speed
    U2BRG              = BAUDRATEREG2; // Set baud rate
    U2MODEbits.UEN     = 0; //2;            // TX, RX, CTS & RTS under hardware control
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

//	U2BRG = 259;     // 40Mhz osc, 9600 Baud
	U2BRG = 64;     // 40Mhz osc, 38400 Baud

#endif
// configure U2MODE 
//etu added
#if 0
    U2MODEbits.UARTEN = 0;  // Bit15 TX, RX DISABLED, ENABLE at end of func
    //U2MODEbits.notimplemented;    // Bit14
    U2MODEbits.USIDL = 0;   // Bit13 Continue in Idle
    U2MODEbits.IREN = 0;    // Bit12 No IR translation
    U2MODEbits.RTSMD = 0;   // Bit11 Simplex Mode
    //U2MODEbits.notimplemented;    // Bit10
    U2MODEbits.UEN = 0;     // Bits8,9 TX,RX enabled, CTS,RTS not
    U2MODEbits.WAKE = 0;    // Bit7 No Wake up (since we don't sleep here)
    U2MODEbits.LPBACK = 0;  // Bit6 No Loop Back
    U2MODEbits.ABAUD = 0;   // Bit5 No Autobaud (would require sending '55')
//    U2MODEbits.URXINV = 0;  // Bit4 IdleState = 1  (for dsPIC)
//    U2MODEbits.BRGH = 0;    // Bit3 16 clocks per bit period
    U2MODEbits.PDSEL = 0;   // Bits1,2 8bit, No Parity
    U2MODEbits.STSEL = 0;   // Bit0 One Stop Bit

//	IPC7 = 0x4400;  // Mid Range Interrupt Priority level, no urgent reason

    IFS1bits.U2TXIF = 0;    // Clear the Transmit Interrupt Flag
    IEC1bits.U2TXIE = 1;    // Enable Transmit Interrupts
    IFS1bits.U2RXIF = 0;    // Clear the Recieve Interrupt Flag
    IEC1bits.U2RXIE = 1;    // Enable Recieve Interrupts

// Load a value into Baud Rate Generator.  Example is for 9600.
    // See section 19.3.1 of datasheet.
    //  U2BRG = (Fcy/(16*BaudRate))-1
    //  U2BRG = (40M/(16*9600))-1
	//  U2BRG = 259;     // 40Mhz osc, 9600 Baud
	U2BRG = 64;     // 40Mhz osc, 38400 Baud
#endif
// etu

	txHead = txQueue;
    txTail = txQueue;
    rxHead = rxQueue;
    rxTail = rxQueue;
   
    rxOverflowed = FALSE;
 }

void __attribute__ ((weak)) UART2_Receive_CallBack(void)
{

}

void UART2_SetRxInterruptHandler(void (* interruptHandler)(void))
{
    if(interruptHandler == NULL)
    {
        UART2_RxDefaultInterruptHandler = &UART2_Receive_CallBack;
    }
    else
    {
        UART2_RxDefaultInterruptHandler = interruptHandler;
    }
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
void u1_SendPacket( int packetsize )
{
  int i;

	while( u1_txBufSending == 1 );                          //return; // Don't allow steping on a currently sending packet

	for( i = 0; i < 10000; i++ );                           // Small Delay
	u1_txBufSending       = 1;								// Show we are sending!
	u1_txbufdatasize      = packetsize + NumHeaderBytes;	// set the packet size to send (generic) - could be used to send other packets later...if needed.
	u1_txbufindex         = 1;								// Point to first byte to transmit ( after sending byte zero(0) below: )
	U1TXREG               = U1_txbuf[0];					// Send the first byte, let Tx interrupt send all the others...
	u1_txbufdatasize--;	                                    // dec data size counter to show that the first byte has been sent.
}

void u2_SendPacket( int packetsize )
{
  int i;

	while( u1_txBufSending == 1 );                          //return; // Don't allow steping on a currently sending packet

	for( i = 0; i < 500; i++ );                           // Small Delay
	u1_txBufSending       = 1;								// Show we are sending!
	u1_txbufdatasize      = packetsize; // + Num2HeaderBytes;	// set the packet size to send (generic) - could be used to send other packets later...if needed.
	u1_txbufindex         = 1;								// Point to first byte to transmit ( after sending byte zero(0) below: )
//	modIndex++;
	U2TXREG               = TxData[0];					// Send the first byte, let Tx interrupt send all the others...
	u1_txbufdatasize--;	                                    // dec data size counter to show that the first byte has been sent.
}

#if 0
void u2_SendDemo( int packetsize )
{
  int i;

	while( u1_txBufSending == 1 );                          //return; // Don't allow steping on a currently sending packet

	for( i = 0; i < 1000; i++ );                           // Small Delay
	u1_txBufSending       = 1;								// Show we are sending!
	u1_txbufdatasize      = packetsize; // + NumHeaderBytes;	// set the packet size to send (generic) - could be used to send other packets later...if needed.
	u1_txbufindex         = 1;								// Point to first byte to transmit ( after sending byte zero(0) below: )
//	U2TXREG               = U2_txbuf[0];					// Send the first byte, let Tx interrupt send all the others...
	U2TXREG               = U2_testbuf[0];					// Send the first byte, let Tx interrupt send all the others...
	u1_txbufdatasize--;	                                    // dec data size counter to show that the first byte has been sent.
}
#endif

/* ------------------------------------------------------------------------- *
 *                                                                           *
 *  _U1TXInterrupt(void)     Handles hardware interrupts when sending a      *
 *                           packet of data.                                 *
 *                                                                           *
 *  Handle UART #1's Transmit interrupt                                      *
 *                                                                           *
 *  U1_txbuf[]    : The data packet being sent                               *
 *  txbufindex    : Index to which byte to send next                         *
 *  txbufdatasize : Count of the number of bytes needed to transmit          *
 *                                                                           *
 *  RETURNS: None                                                            *
 *                                                                           *
 * ------------------------------------------------------------------------- */
void __attribute__((__interrupt__, no_auto_psv)) _U1TXInterrupt( void )
{
  int txtemp;

  if( u1_txbufdatasize == 0)                        // are there any charaters left in buffer?
    {                                               //
        Nop();                                      // NO, do nothing (No transmitter to turn off)
        u1_txBufSending  = 0;						// Show we are finished sending packet!  Ready for Rx
    }                                               //
  else                                              // OK - we have more data to send!
    {                                               //
	    txtemp = U1_txbuf[ u1_txbufindex ];			// Get the next byte
	    u1_txbufindex++;							// increment index in to transmit buffer
	    u1_txbufdatasize--;							// decrement data size to account for sending the next byte, no not need to
	                                                // check for txbufdatasize being zero (to protect decrement to -1) because above
	                                                // 'if' statement checked for that issue.
	    U1TXREG = txtemp;							// Send the data
    }                                               //
                                                    //
  IFS0bits.U1TXIF = 0;                              // Clear UART1 Transmit Irq flag
}


/* ------------------------------------------------------------------------- *
 *                                                                           *
 *  _U1TXInterrupt(void)     Handles hardware interrupts when sending a      *
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

  if( u1_txbufdatasize == 0)                        // are there any charaters left in buffer?
    {                                               //
        Nop();                                      // NO, do nothing (No transmitter to turn off)
        u1_txBufSending  = 0;						// Show we are finished sending packet!  Ready for Rx
    }                                               //
  else                                              // OK - we have more data to send!
    {                                               //
//	    txtemp = U2_txbuf[ u1_txbufindex ];			// Get the next byte
	    txtemp = TxData[ u1_txbufindex ];			// Get the next byte
//	    txtemp = U2_testbuf[ u1_txbufindex ];		// Get the next byte
	    u1_txbufindex++;							// increment index in to transmit buffer
	    u1_txbufdatasize--;							// decrement data size to account for sending the next byte, no not need to
	                                                // check for txbufdatasize being zero (to protect decrement to -1) because above
	                                                // 'if' statement checked for that issue.
	    U2TXREG = txtemp;							// Send the data
    }                                               //
                                                    //
  IFS1bits.U2TXIF = 0;                              // Clear UART2 Transmit Irq flag
}

/* -------------------------------------------------------------------------  *
 *                                                                            *
 *  uart1_ReadyForNextPacket()   This proceedure clears the SW receive buffer *
 *                               and clears the rxPacketReady flag, also it   *
 *                               sets the index in to the receive buffer to   *
 *                               the beginning of the buffer/array.           *
 *                                                                            *
 *  U1_rxbuf[]        : Serial receive buffer (Cleared in this proceedure)    *
 *  NumIncomingPacket : Total number of bytes in an Command packet            *
 *                                                                            *
 *  RETURNS: None                                                             *
 *                                                                            *
 * -------------------------------------------------------------------------  */
void uart1_ReadyForNextPacket()
{
	int i;

	for(i = 0; i< NumIncomingPacket; i++)
	{
		U1_rxbuf[u1_rxIndex] = 0;					// Clear the rx array
	}

	u1_rxPacketReady = FALSE;						// Show packet is clear
	u1_rxIndex       = 0;							// Get ready for the next packet
}

//#define PacketID U1_rxbuf[2]                        // Define where PACKET ID is un the header!

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
void uart2_ReadyForNextPacket()
{
	int i;

	for(i = 0; i< Num2IncomingPacket; i++)
	{
		U2_rxbuf[u2_rxIndex] = 0;					// Clear the rx array
	}

	u2_rxPacketReady = FALSE;						// Show packet is clear
	u2_rxIndex       = 0;							// Get ready for the next packet
}
#endif

/* ------------------------------------------------------------------------- *
 *                                                                           *
 *  _U1RXInterrupt(void)     Handles hardware interrupts when receiving a    *
 *                           packet of data.                                 *
 *                                                                           *
 *  Handle UART #1's Receive interrupt                                       *
 *                                                                           *
 *  I1_rxbuf[] :  Has the received packet when rxPacketReady is TRUE         *
 *                                                                           *
 *  RETURNS: None                                                            *
 *                                                                           *
 * ------------------------------------------------------------------------- */
void __attribute__((__interrupt__, no_auto_psv)) _U1RXInterrupt( void )
{
  int rxtemp;                                   	//
                                                    //
  if( U1STAbits.OERR )                              // Hardware Buffer Overflow Error
  {                                                 //
    U1STAbits.OERR = 0;                           	// Clear Error
  }

  // Receive state machine:  0- Nothing; Waiting for header to start!, 0,1- Looking for 0xaa, 0x55  2- The rest of the packet
  while( U1STAbits.URXDA )                        	// While we have data...
  {

   if( receivefwbinary )
   {
     rxtemp = U1RXREG ;

      if( rxtemp == 0xA5 )
      {
          if( indicator == 0xA5 )
          {
              savefwdata = 1;
			  goto EXITIF;
  		  }
      }
      else if( rxtemp == 0x5A )
      {
          if( indicator == 0x5A )
          {
              savefwdata  = 0;
			  writefwtoEE = 1;
  		  }
	  }


	  if( savefwdata )
	  {
	  	  *pWorkBuffer++ = rxtemp;
	       totalbytesreceived++;
	  }

	EXITIF:
      indicator = rxtemp;
   }
   else
   {
    // ------------
    // Idle here, removing data from the serial stream while we are processing
    // a packet. Any serial data sent in while the main routine is processing
    // packets will be discarded.
    // ------------
    if( u1_rxPacketReady == TRUE )
    {
      while( U1STAbits.URXDA )
      {
        rxtemp = U1RXREG;
      }

      return;
    }

    // -------------
    // OK - Normal processing of an incomming packet
    // -------------
    rxtemp    = U1RXREG;                          // Get a byte from the data stream...
    iRxTm     = RESET_TIME;                       // reset timer so it does not reset the communications stream!
    commTimer = 60;                               // Default = 200ms

    switch( RxStateMachine )
    {
      case 0: // Waiting for 0xaa
              u1_rxIndex = 0;                     // 0xaa goes in @ index 0

              if( rxtemp == 0xaa )
              {
                U1_rxbuf[u1_rxIndex] = rxtemp;    // Save to indexed rxbuffer
                RxStateMachine       = 1;         // Move to next state
                u1_rxIndex++;                     // Increment receive array index
              }

              break;

      case 1:
              if( rxtemp == 0x55 )
              {
                U1_rxbuf[u1_rxIndex] = rxtemp;    // Save to indexed rxbuffer
                RxStateMachine       = 2;
                u1_rxIndex++;
              }

              break;

      case 2: //
              // Receive all other bytes here:
              //
              U1_rxbuf[u1_rxIndex] = rxtemp;      // Save to indexed rxbuffer

              if( u1_rxIndex < 250 )
                  u1_rxIndex++; // array index overbounds protection
              else
              {
                // LOG ERROR!                     // Could log an error here if needed.
              }

              break;
    }
   }
  }

  IFS0bits.U1RXIF = 0;                              // Clear UART1 Recive Irq flag
}




/* ------------------------------------------------------------------------- *
 *                                                                           *
 *  _U1RXInterrupt(void)     Handles hardware interrupts when receiving a    *
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
//  int rxtemp;                                   	//
                                                    //
  if( U2STAbits.OERR )                              // Hardware Buffer Overflow Error
  {                                                 //
    U2STAbits.OERR = 0;                           	// Clear Error
  }

#if 0
	if( u2_rxPacketReady == TRUE )
	{
	      while( U2STAbits.URXDA )
	      {
	        rxtemp = U2RXREG;
	      }
	
	      return;
	}
#endif


#if 1
while((U2STAbits.URXDA == 1))
    {
        *rxTail = U2RXREG;

        // Will the increment not result in a wrap and not result in a pure collision?
        // This is most often condition so check first
        if ( ( rxTail    != (rxQueue + UART2_CONFIG_RX_BYTEQ_LENGTH-1)) &&
             ((rxTail+1) != rxHead) )
        {
            rxTail++;
        } 
        else if ( (rxTail == (rxQueue + UART2_CONFIG_RX_BYTEQ_LENGTH-1)) &&
                  (rxHead !=  rxQueue) )
        {
            // Pure wrap no collision
            rxTail = rxQueue;
        } 
        else // must be collision
        {
            rxOverflowed = true;
        }
    }
#endif
	
//etu added
////	rx2_Rcv = 0;
rx2_Rcv = 1;

  IFS1bits.U2RXIF = 0;                              // Clear UART2 Recive Irq flag
}

/**
  Section: UART Driver Client Routines
*/

uint8_t UART2_Read( void)
{
    uint8_t rxtemp = 0;

    while (rxHead == rxTail )
	{
	}

	    rxtemp = *rxHead;
	
	    rxHead++;
	
	    if (rxHead == (rxQueue + UART2_CONFIG_RX_BYTEQ_LENGTH))
	    {
	        rxHead = rxQueue;
	    }
		newCmd = 1;
	    return rxtemp;
}

bool UART2_IsTxReady(void)
{
    uint16_t size;
    uint8_t *snapshot_txHead = (uint8_t*)txHead;
    
    if (txTail < snapshot_txHead)
    {
        size = (snapshot_txHead - txTail - 1);
    }
    else
    {
        size = ( UART2_CONFIG_TX_BYTEQ_LENGTH - (txTail - snapshot_txHead) - 1 );
    }
    
    return (size != 0);
}

void UART2_Write( uint8_t byte)
{
    while(UART2_IsTxReady() == 0)
    {
    }

    *txTail = byte;

    txTail++;
    
    if (txTail == (txQueue + UART2_CONFIG_TX_BYTEQ_LENGTH))
    {
        txTail = txQueue;
    }

    IEC1bits.U2TXIE = 1;
}

bool UART2_IsRxReady(void)
{    
    return !(rxHead == rxTail);
}


bool UART2_IsTxDone(void)
{
    if(txTail == txHead)
    {
        return (bool)U2STAbits.TRMT;
    }
    
    return false;
}

#if 0
int __attribute__((__section__(".libc.write"))) write(int handle, void *buffer, unsigned int len) 
{
    unsigned int i;
    uint8_t *data = buffer;

    for(i=0; i<len; i++)
    {
        while(UART2_IsTxReady() == false)
        {
        }

        UART2_Write(*data++);
    }
  
    return(len);
}
#endif

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
void __attribute__((__interrupt__, no_auto_psv)) _U1ErrInterrupt( void )
{
  if( U1STAbits.OERR )                    // Hardware Buffer Overflow Error
  {
      U1STAbits.OERR  = 0;                // Clear Error
  }                                       //
  IFS4bits.U1EIF      = 0;                // Clear UART1 Error Irq flag
}

 /* ------------------------------------------------------------------------- *
 *                                                                            *
 *  _U1ErrInterrupt()     Handles hardware interrupts when receiving a        *
 *                        error when dealing with the serial port.            *
 *                                                                            *
 *  Handle UART #2's Receive interrupt                                        *
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
  IFS4bits.U2EIF      = 0;                // Clear UART2 Error Irq flag
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
void initbuffers( void )
{
    indicator = 0;
	writefwtoEE = 0;
	savefwdata = 0;
	pWorkBuffer = scReadBuff;
	totalbytesreceived = 0;
    receivefwbinary = 1;
}

/* -------------------------------------------------------------------------  *
 *                                                                            *
 *  isDownloadDone()  This proceedure starts the process to write to ee       *
 *                    the bin file already stored in scReadBuff.              *
 *                    totalbytesreceived is decremented to take out the 0xA5  *
 *                                                                            *
 *  RETURNS: true when the entire bin file has been written to ee             *
 *                                                                            *
 * -------------------------------------------------------------------------  */
int isDownloadDone( void )
{
   if( writefwtoEE )
   {
	 // Disable UART so that anything on the uart will not disturb ee writes
     IEC0bits.U1TXIE    = 0;            // Disable TX Interrupt
     IEC0bits.U1RXIE    = 0;            // Disable RX Interrupt
     U1MODEbits.UARTEN  = 0;            // Disable UART
     writefwtoEE        = 0;

     totalbytesreceived--;				// cut out the last byte 0x5A saved that was actually a delimiter-
                                        //- and not part of the bin file
	 totalbytesreceived--;				// cut out another byte since we are going to write "B" into the -
                                        //-0th position after the entire bin file has been writte to the EE
     // start from position 1 of scReadBuff
	 if( i2c1_EEWriteArray( 0xA8, 1, &scReadBuff[1], totalbytesreceived ) ) 
		return 1;
   }
   return 0;
}

/* -------------------------------------------------------------------------  *
 *                                                                            *
 *  writeBLFS()  Well actually just write "B" in the 0th address of the EE    *
 *                                                                            *
 * -------------------------------------------------------------------------  */
void writeBLFS( void )
{
	i2c1_EEWriteArray( 0xA8, 0, &scReadBuff[0], 4 );
}

/* -------------------------------------------------------------------------  *
 *                                                                            *
 *  clearBLFS()  Well actually just write "B" in the 0th address of the EE    *
 *                                                                            *
 * -------------------------------------------------------------------------  */
void clearBLFS( void )
{
	unsigned char ucBuff[5] = {0,0,0,0};

	i2c1_EEWriteArray( 0xA8, 0, &ucBuff[0], 4 );
}

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
int iswritefwtoEE( void )
{
	return writefwtoEE;
}
