/*********************************************************************
 *
 *	Hardware specific definitions
 *
 *********************************************************************
 * FileName:        HardwareProfile.h
 * Dependencies:    None
 * Processor:       PIC18, PIC24F, PIC24H, dsPIC30F, dsPIC33F, PIC32
 * Compiler:        Microchip C32 v1.00 or higher
 *					Microchip C30 v3.01 or higher
 *					Microchip C18 v3.13 or higher
 *					HI-TECH PICC-18 STD 9.50PL3 or higher
 * Company:         Microchip Technology, Inc.
 *
 * Software License Agreement
 *
 * Copyright (C) 2002-2008 Microchip Technology Inc.  All rights 
 * reserved.
 *
 * Microchip licenses to you the right to use, modify, copy, and 
 * distribute: 
 * (i)  the Software when embedded on a Microchip microcontroller or 
 *      digital signal controller product ("Device") which is 
 *      integrated into Licensee's product; or
 * (ii) ONLY the Software driver source files ENC28J60.c and 
 *      ENC28J60.h ported to a non-Microchip device used in 
 *      conjunction with a Microchip ethernet controller for the 
 *      sole purpose of interfacing with the ethernet controller. 
 *
 * You should refer to the license agreement accompanying this 
 * Software for additional information regarding your rights and 
 * obligations.
 *
 * THE SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT 
 * WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT 
 * LIMITATION, ANY WARRANTY OF MERCHANTABILITY, FITNESS FOR A 
 * PARTICULAR PURPOSE, TITLE AND NON-INFRINGEMENT. IN NO EVENT SHALL 
 * MICROCHIP BE LIABLE FOR ANY INCIDENTAL, SPECIAL, INDIRECT OR 
 * CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF 
 * PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY OR SERVICES, ANY CLAIMS 
 * BY THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY DEFENSE 
 * THEREOF), ANY CLAIMS FOR INDEMNITY OR CONTRIBUTION, OR OTHER 
 * SIMILAR COSTS, WHETHER ASSERTED ON THE BASIS OF CONTRACT, TORT 
 * (INCLUDING NEGLIGENCE), BREACH OF WARRANTY, OR OTHERWISE.
 *
 *
 * Author               Date		Comment
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * Howard Schlunder		10/03/06	Original, copied from Compiler.h
 ********************************************************************/
#ifndef __HARDWARE_PROFILE_H
#define __HARDWARE_PROFILE_H

// Choose which hardware profile to compile for here.  See 
// the hardware profiles below for meaning of various options.  
#define SAGE_BOARD

// Set configuration fuses (but only once)
#if defined(THIS_IS_STACK_APPLICATION)
	#if defined(__dsPIC33F__) || defined(__PIC24H__)
		// Explorer 16 board
///		_FOSCSEL(FNOSC_PRIPLL)			// PLL enabled
///		_FOSC(OSCIOFNC_OFF & POSCMD_XT)	// XT Osc
///		_FWDT(FWDTEN_OFF)				// Disable Watchdog timer
		// JTAG should be disabled as well
	#endif
#endif // Prevent more than one set of config fuse definitions

// Clock frequency value.
// This value is used to calculate Tick Counter value
#if  defined(__dsPIC33F__)	
	// dsPIC33F processor
	#define GetSystemClock()		(80000000ul)      // Hz
	#define GetInstructionClock()	(GetSystemClock()/2)
	#define GetPeripheralClock()	GetInstructionClock()
#endif

// Hardware mappings
#if defined(SAGE_BOARD)
/* TurboChef's SAGE Controls board          2006 10/25 -- MJP  
   Updated to SAGE Rev 6                    2008  7/16 -- MJP  

	 ENC28J60 I/O pins on SAGE rev 6
       SPI2 is used 
         SDI = RG7, SDO = RG8, SCK = RG6
       ENC SPI select = RG9
       ENC INT        = RA14 (INT3)
       ENG RESET      = MCLR
     */
       
//	#define ENC_RST_TRIS		(TRISBbits.TRISB5)
//	#define ENC_RST_IO			(LATBbits.LATB5)
	#define ENC_CS_TRIS			(TRISGbits.TRISG9)
	#define ENC_CS_IO			(LATGbits.LATG9)
	#define ENC_SCK_TRIS		(TRISGbits.TRISG6)
	#define ENC_SDI_TRIS		(TRISGbits.TRISG7)
	#define ENC_SDO_TRIS		(TRISGbits.TRISG8)
	#define ENC_SPI_IF			(IFS2bits.SPI2IF)
	#define ENC_SSPBUF			(SPI2BUF)
	#define ENC_SPISTAT			(SPI2STAT)
	#define ENC_SPISTATbits		(SPI2STATbits)
	#define ENC_SPICON1			(SPI2CON1)
	#define ENC_SPICON1bits		(SPI2CON1bits)
	#define ENC_SPICON2			(SPI2CON2)


	/* SAGE has an I2C EEPROM */
// Too bad the new stack doesn't support it... 
///#define MPFS_USE_EEPROM

/*
 * This is the address for U19, Microchip 24LC512, a  64KB serial EEPROM
 */
#define EEPROM_CONTROL                  (0xA8)

	/*   I2C EEPROM I/O pins on I2C#2  (Not really SPI... sigh...) */
///	#define EEPROM_CS_TRIS		(TRISBbits.TRISB4)
///	#define EEPROM_CS_IO		(LATBbits.LATB4)
///	#define EEPROM_SCK_TRIS		(TRISCbits.TRISC3)
///	#define EEPROM_SDI_TRIS		(TRISCbits.TRISC4)
///	#define EEPROM_SDO_TRIS		(TRISCbits.TRISC5)
	#define EEPROM_SCL_TRIS		(TRISGbits.TRISG2)
	#define EEPROM_SDA_TRIS		(TRISGbits.TRISG3)
	#define EEPROM_I2C_IF		(PIR1bits.SSPIF)
	#define EEPROM_I2CBRG		(I2C1BRG)
	#define EEPROM_I2CCON		(I2C1CON)
	#define EEPROM_I2CCONbits	(I2C1CONbits)
	#define EEPROM_I2CSTAT		(I2C1STAT)
	#define EEPROM_I2CSTATbits	(I2C1STATbits)
	#define EEPROM_I2CTX		(I2C1TRN)
	#define EEPROM_I2CRX		(I2C1RCV)


/* SAGE KEYBOARD
   Rows and columns are spread across 3 ports.

	ROW's are outputs, COL's are inputs (with pull down resistors)


Pin dsPIC discription	SAGE board discription	(Secondary use)	
--- ------------------  -----------------------------------------			
        PIO PortA
017 TMS/RA0				KEYROW1 	   (JTAG test mode select)
038 TCK/RA1				KEYROW2 	   (JTAG test clock)
060 TDI/RA4             KEYROW3        (JTAG Test data in)
061 TD0/RA5             KEYROW4        (JTAG Test data out)
091 AN22/CN22/RA6       KEYROW5
092 AN23/CN23/RA7       KEYCOL8

		PIO PortB
025 PGD3/MNUD3/AN0/CN2/RB0	KEYCOL1
024 PGC3/EMUC3/AN1/CN3/RB1  KEYCOL2	
023 AN2//SS1/CN4/RB2		KEYCOL3
022 AN3/CN5/RB3		        KEYCOL4
021 AN4/CN6/RB4		        KEYCOL5  
020 AN5/CN7/RB5	            KEYCOL6  

		PIO PortD
080 IC6/CN19/RD13       KEYCOL7

*/
	#define BUTTON0_TRIS		(TRISDbits.TRISD10)	// BELT1
	#define	BUTTON0_IO			(PORTDbits.RD10)
	#define BUTTON1_TRIS		(TRISDbits.TRISD11)	// BELT2
	#define	BUTTON1_IO			(PORTDbits.RD11)
	#define BUTTON2_TRIS		(TRISDbits.TRISD12)	// BELT3
	#define	BUTTON2_IO			(PORTDbits.RD12)
	#define BUTTON3_TRIS		(TRISAbits.TRISA14)	// HX_Status
	#define	BUTTON3_IO			(PORTAbits.RA14)


		/* Note, these ARE actually hooked up to stuff and in use! (UART2) */
	#define UARTTX_TRIS			(TRISFbits.TRISF5)
	#define UARTTX_IO			(PORTFbits.RF5)
	#define UARTRX_TRIS			(TRISFbits.TRISF4)
	#define UARTRX_IO			(PORTFbits.RF4)


#else
	#error "Hardware profile not defined.  See available profiles in HardwareProfile.h.  Add the appropriate macro definition to your application configuration file ('TCPIPConfig.h', etc.)"
#endif


#if defined(__18CXX)	// PIC18
	// UART mapping functions for consistent API names across 8-bit and 16 or 
	// 32 bit compilers.  For simplicity, everything will use "UART" instead 
	// of USART/EUSART/etc.
	#define BusyUART()				BusyUSART()
	#define CloseUART()				CloseUSART()
	#define ConfigIntUART(a)		ConfigIntUSART(a)
	#define DataRdyUART()			DataRdyUSART()
	#define OpenUART(a,b,c)			OpenUSART(a,b,c)
	#define ReadUART()				ReadUSART()
	#define WriteUART(a)			WriteUSART(a)
	#define getsUART(a,b,c)			getsUSART(b,a)
	#define putsUART(a)				putsUSART(a)
	#define getcUART()				ReadUSART()
	#define putcUART(a)				WriteUSART(a)
	#define putrsUART(a)			putrsUSART((far rom char*)a)

#else	 // PIC24F, PIC24H, dsPIC30, dsPIC33, PIC32
	// Some A/D converter registers on dsPIC30s are named slightly differently 
	// on other procesors, so we need to rename them.
	#if defined(__dsPIC30F__)
		#define ADC1BUF0			ADCBUF0
		#define AD1CHS				ADCHS
		#define	AD1CON1				ADCON1
		#define AD1CON2				ADCON2
		#define AD1CON3				ADCON3
		#define AD1PCFGbits			ADPCFGbits
		#define AD1CSSL				ADCSSL
		#define AD1IF				ADIF
		#define AD1IE				ADIE
		#define _ADC1Interrupt		_ADCInterrupt
	#endif

	// Select which UART the STACK_USE_UART and STACK_USE_UART2TCP_BRIDGE 
	// options will use.  You can change these to U1BRG, U1MODE, etc. if you 
	// want to use the UART1 module instead of UART2.
	#define UBRG					U2BRG
	#define UMODE					U2MODE
	#define USTA					U2STA
	#define BusyUART()				BusyUART2()
	#define CloseUART()				CloseUART2()
	#define ConfigIntUART(a)		ConfigIntUART2(a)
	#define DataRdyUART()			DataRdyUART2()
	#define OpenUART(a,b,c)			OpenUART2(a,b,c)
	#define ReadUART()				ReadUART2()
	#define WriteUART(a)			WriteUART2(a)
	#define getsUART(a,b,c)			getsUART2(a,b,c)
	#if defined(__C32__)
		#define putsUART(a)			putsUART2(a)
	#else
		#define putsUART(a)			putsUART2((unsigned int*)a)
	#endif
	#define getcUART()				getcUART2()
	#define putcUART(a)				WriteUART(a)
	#define putrsUART(a)			putsUART(a)
#endif

#endif
