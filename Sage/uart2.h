/* -------------------------------------------------------------------------- *
 *  Module for Turbochef Oven Software - Phoenix Project                      *
 *  uart1.h                                                                   *
 *                                                                            *
 *  Low level driver firmware for access to Serial Port #2                    *
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
 * FileName:        uart2.h                                                   *
 * Dependencies:    None                                                      *
 * Processor:       dsPIC33FJ256GP710                                         *
 * Compiler:        MPLAB XC16                                                 *
 * Company:         Turbochef Technology Incorporated                         *
 *                                                                            *
 * Copyright ?2013 Turbochef Technology Inc.  All rights reserved.           *
 *                                                                            *
 * ---------------------------------------------------------------------------*/

/* Module Defines */
#define TXBUFSIZE       (64)  	          /* TX Buffer Size, MUST be greater than 6! */
#define RXBUFSIZE       (64)  	          /* RX Buffer Size, MUST be greater than 6! */
#define BAUDRATE1       115200            // Baudrate, in Hz 115200   57600

/* Module IOs  */

// Data Direction resisters:
#define U2_TX_TRIS      TRISFbits.TRISF5 //TRISFbits.TRISF3
#define U2_RX_TRIS      TRISFbits.TRISF4 //TRISFbits.TRISF2
#define U2_CTS_TRIS     TRISDbits.TRISD14
#define U2_RTS_TRIS     TRISDbits.TRISD15
// Input/Output pins
#define U2_TX_PIN       PORTFbits.RF5 //PORTFbits.RF3
#define U2_RX_PIN       PORTFbits.RF4 //PORTFbits.RF2
#define U2_RTS_PIN      PORTDbits.RD15
#define U2_CTS_PIN      PORTDbits.RD14

#define BAUDRATEREG1 (SYSCLK/(16*BAUDRATE1))-1

extern volatile int u1_rxPacketReady;			// When == TRUE ;; Packet is full and ready to parse/test/move/... new command packet

//* * *  Prototypes
void U2_Init();
void u2_SendPacket(int packetsize);			    // Send a packet of data...
void uart2_ReadyForNextPacket2();		        // Clears internal var's to get ready for next rx packet
void initbuffers2(void);
int iswritefwtoEE2(void);
int isDownloadDone2(void);
void writeBLFS2(void);
void clearBLFS2(void);
