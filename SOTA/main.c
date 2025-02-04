/* ---------------------------------- main.c --------------------------------------------------- *
 * Copyright (C) 2006 Turbochef Technologies Inc                                                 *
 *                                                                                               *
 *    Source: C:\Dev\dsPIC33\base\main.                                                          *
 *    Author: Mark Philipp                                                                       *
 * Chief storyboard author, co-author, executive producer, "Grip"  Sanity -                      *
 * checker, : Harold F.                                                                          *
 * Processer: Microchip dsPIC33FJ256GP710                                                        *
 *       IDE: Microchip's MPLAB IDE                                                              *
 *  Compiler: MPLAB. C30 v2.01 or higher                                                         *
 *  Assembly: Microchip's "Explorer 16 Developer Board"                                          *
 *  Debugger: MPLAB ICD 2                                                                        *
 *  Revision:  (pre-alpha)                                                                       *
 *      Date:  2006 06/26                                                                        *
 *                                                                                               *
 * SAGE Board - MPU Pins by Port        MPU = dsPIC33FJ256GP710                                  *
 *                                                                                               *
 * --------------------------------------------------------------------------------------------- */
#include "p33FJ256GP710.h"
#include "common.h"
#include "timer1.h"
#include "timer23.h"
#include "adc.h"
#include "pwm.h"
#include "i2c1.h"           // added with move from TASKio.c
#include "uart1.h"
#include "TASKio.h"
#include "uart1.h"
#include "TASKBlowers.h"
#include "TASKHeater.h"
#include "TASKPhoenix.h"

#include "pid.h"

/* Configuration Bits */

 _FGS(GWRP_OFF & GCP_OFF);
 _FOSCSEL(FNOSC_PRIPLL);
 _FOSC(FCKSM_CSDCMD & OSCIOFNC_OFF & POSCMD_XT);
/// _FWDT( FWDTEN_ON & WDTPRE_PR32 &WDTPOST_PS1024); // Watchdog ON, 1mS prescaler, Timeout = 1 Sec
 _FWDT(FWDTEN_OFF);	//Watchdog "OFF" (Controlable by Software)

	/*Ops, Microchip seems to have forgot JTAG in CONFIG3 in the dsPIC33! */
#define _CONFIG3(x) __attribute__((section("__CONFIG3.sec,code"))) int _CONFIG3 = (x);
_CONFIG3(0x00CB); // BKBUG_ON, COE_ON, JTAG_off, 0, 1, 0, ICS=PGC/EMUC&PGD/EMUD

int iInitHardware = 1; //0;
int iTasksActivated = 1;

int TASKBlowersSleep 		= TASKBLOWERSLEEP;
int TASKHeatersSleep		= TASKHEATERSLEEP;
int TASKInterPhoenixSleep 	= TASKINTERPHSLEEP;
int TASKSerialPhoenixSleep 	= TASKSERIALPHSLEEP;

int BLFSwriteDelay 			= 0;
int ResetDelay 				= 0;

unsigned char ucWriteBLFS 	= 0;
unsigned char ucResetSage 	= 0;

//etu added
extern volatile char U2_txbuf[ 10 ]; 
extern unsigned char TxData[16];
extern unsigned char RxData[8];
#if 0
	int i,j;
	int cmdLen;
	//init MeasTemp
	iTopMeasTempF = 388; //0x0184
	iBotMeasTempF = 412; //0x019c
#endif
extern Pid_Remona pidParam;
//etu

extern void send_Version( void );


void InitHardware( void )
{
  i2c1_init();
  pwm_init();       /* Initialize all Output Compares to PWM mode */
  Init_ADC();       /* Initalize Analog Inputs */
//  U1_Init();		/* Initialize Uart #1 - RS-485 port */
  U2_Init();		/* Initialize Uart #2 - RS-485 port */
  InitLSIO();       /* Initialize U16 with PortA */
//  InitializePIDs();	/* Default PID values */
  PID_param_init(&pidParam);

  LSIn.Port  = 0;                           // Start out with clean data and init blower status to off
  LSOut.Port = 0;                           // Setup output low - all off

  WriteIOPort(0x42,0x00,0x00);		    	// Initiallize port expander - Port A is output and Port B is input
  WriteIOPort(0x42,0x00,0x00);				// Port B is defaulted to all ones so don't need to program it explicitly
  WriteIOPort(0x42,0x00,0x00);				// just send the command for Port A repeatedly. This is because in cases
											// where sage would reset several times, port expander wasn't getting this command
  // Heat OFF
  HX1TOP = OFF;
  HX2TOP = OFF;
  HX1BOT = OFF;
  HX2BOT = OFF;

  WriteIOPort(0x9A,0x01,0x00);	     		// Initiallize EC Thermometer
}


/* -----------------10/7/2006 9:51PM-----------------*
 *  The Primary Startup Task: main()                 *
 *             Crystal Frequency  * (DIVISOR+2)      *
 *  Fcy =     ---------------------------------      *
 *               PLLPOST * (PRESCLR+2) * 4           *
 *                                                   *
 *  For 16 MIPS off of a 8.0 MHz Xtal                *
 *  PLLFBD = 0x00A0;                                 *
 *  CLKDIV = 0x0048;                                 *
 *  For 40 MIPS off of a 8.0 MHz Xtal                *
 * --------------------------------------------------*/
int main( void )
{

  // For 40 MIPS off of a 8.0 MHz Xtal
  PLLFBD = 0x0026;
  CLKDIV = 0x0000;
  OSCCONbits.LPOSCEN = 0;		// Disable Secondary Oscillator
  RCONbits.SWDTEN    = 0;		// Disable Watch Dog Timer


//U2RxRBits.RXR = 0b0001;
//RPB7Rbits.RPB7R = 0b0001;

  /* Inintiallize Hardware needed before starting tasks */
  Init_Timer1();    /* Initialize Timer 1 for 1000 Hz from Fcy Clock */
  Init_Timer2();    /* Initialize Timer 2 */
  Init_Timer3();    /* Initialize Timer 3 */

  /* All hardware should be Init'd, it should be safe to handle IRQ's, Start Ints!*/
  __asm__ volatile( "disi #0x0000" ); // All IRQ's ON

  /* OS Repeat... */
  while( 1 )
  {
	if( iInitHardware == 1 )
	{
		clearBLFS();			// clear BLFS again in case the bootloader didn't do its job
		InitHardware();
		iInitHardware = 2;
	}

	if(iTasksActivated )
	{
		if( TASKBlowersSleep == 0 ) 
		{
			TASKBlowers();
			TASKBlowersSleep = TASKBLOWERSLEEP;
		}

		if( TASKHeatersSleep == 0 )  
		{
			TASKHeaters();
			TASKHeatersSleep = TASKHEATERSLEEP;
		}

		if( TASKInterPhoenixSleep == 0 ) 
		{
			TASKInterfacePhoenix();
			TASKInterPhoenixSleep = TASKINTERPHSLEEP;
		}

		if( TASKSerialPhoenixSleep == 0 ) 
		{
			TASKSerialPhoenix();
			//etu added
			TASKSerial2Modbus();
			TASKSerialPhoenixSleep = TASKSERIALPHSLEEP;
		}
	}

    if( DownLoadInProgress )
	{
		if( iswritefwtoEE() )
	  	{
 			if( isDownloadDone() )
 		 	{
				EnableRapidBlink( 0 );
				DownLoadInProgress = 0;
				BLFSwriteDelay     = BLFSWRITEDELAY;
				ucWriteBLFS        = 1;			        // move onto the next stage for writing BLFS
				ALIVE_LED          = 0;
		 	}
	  	}
    }

	if( ucWriteBLFS )
	{
		if( BLFSwriteDelay == 0 )
		{
			ALIVE_LED   = 1;
			ucWriteBLFS = 0;
			writeBLFS();
			ResetDelay  = RESETDELAY;
			ucResetSage	= 1;				            // move onto the next stage for resetting sage
		}
	}

	if( ucResetSage )
	{
		if( ResetDelay < 1500 )
			ALIVE_LED = 0;

		if( ResetDelay == 0 )
		{
			ucResetSage	= 0;
			__asm__ volatile ( "reset" );	            // Soft Reset
		}
  	}
  }
}
