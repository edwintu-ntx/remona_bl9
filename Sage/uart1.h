/* -------------------------------------------------------------------------- *
 *  Module for Turbochef Oven Software - Phoenix Project                      *
 *  uart1.h                                                                   *
 *                                                                            *
 *  Low level driver firmware for access to Serial Port #1                    *
 *                                                                            *
 * PIO PortF                                                                  *
 * 049 U2RX/CN16/RF4   SERIAL_RX                                              *
 * 050 U2TX/CN18/RF5   SERIAL_TX                                              *
 *                                                                            *
 * 040 U2CTS/RF12      SERIAL_DSR (output)                                    *
 * 039 U2RTS/RF13      SERIAL_DTR (input)                                     *
 * -------------------------------------------------------------------------- *
 *                                                                            *
 * ************************************************************************** *
 * FileName:        uart1.h                                                   *
 * Dependencies:    None                                                      *
 * Processor:       dsPIC33FJ256GP710                                         *
 * Compiler:        MPLAB C30                                                 *
 * Company:         Turbochef Technology Incorporated                         *
 *                                                                            *
 * Copyright ?2013 Turbochef Technology Inc.  All rights reserved.           *
 *                                                                            *
 * ---------------------------------------------------------------------------*/

/* Module Defines */
#define TXBUFSIZE       (64)  	          /* TX Buffer Size, MUST be greater than 6! */
#define RXBUFSIZE       (64)  	          /* RX Buffer Size, MUST be greater than 6! */
#define BAUDRATE1       115200            // Baudrate, in Hz 115200   57600
//etu
#define BAUDRATE2       38400            // Baudrate, in Hz 115200   57600

/* Module IOs  */

// Data Direction resisters:
#define U1_TX_TRIS      TRISFbits.TRISF3
#define U1_RX_TRIS      TRISFbits.TRISF2
#define U1_CTS_TRIS     TRISDbits.TRISD14
#define U1_RTS_TRIS     TRISDbits.TRISD15
// Input/Output pins
#define U1_TX_PIN       PORTFbits.RF3
#define U1_RX_PIN       PORTFbits.RF2
#define U1_RTS_PIN      PORTDbits.RD15
#define U1_CTS_PIN      PORTDbits.RD14

//etu added
#define U2_TX_TRIS      TRISFbits.TRISF5 
#define U2_RX_TRIS      TRISFbits.TRISF4 
//#define U2_CTS_TRIS     TRISDbits.TRISD14
//#define U2_RTS_TRIS     TRISDbits.TRISD15
// Input/Output pins
#define U2_TX_PIN       PORTFbits.RF5 
#define U2_RX_PIN       PORTFbits.RF4 
//#define U2_RTS_PIN      PORTDbits.RD15
//#define U2_CTS_PIN      PORTDbits.RD14
#define BAUDRATEREG2 (SYSCLK/(16*BAUDRATE2))-1
//etu

#define BAUDRATEREG1 (SYSCLK/(16*BAUDRATE1))-1

extern volatile int u1_rxPacketReady;			// When == TRUE ;; Packet is full and ready to parse/test/move/... new command packet

//* * *  Prototypes
//void U1_Init();
void U2_Init();
void u1_SendPacket(int packetsize);			    // Send a packet of data...
//etu
void u2_SendPacket(int packetsize);			    // Send a packet of data...

void uart1_ReadyForNextPacket();		        // Clears internal var's to get ready for next rx packet
void initbuffers(void);
int iswritefwtoEE(void);
int isDownloadDone(void);
void writeBLFS(void);
void clearBLFS(void);
