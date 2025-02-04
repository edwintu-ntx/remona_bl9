/* -------------------------------------------------------------------------- *
 *  Module for Turbochef Oven Software - Phoenix Project                      *
 *  TASKPhoenix_Interface.c                                                   *
 *                                                                            *
 *  This TASK handles interfacing SAGE to Phoenix using the Phoenix serial    *
 *  protocol.                                                                 *
 *                                                                            *
 *                                                                            *
 * ************************************************************************** *
 * FileName:        TASKPhoenix_Interface.c                                   *
 * Dependencies:                                                              *
 *                                                                            *
 * Processor:       dsPIC33FJ256GP710                                         *
 * Compiler:        MPLAB C30                                                 *
 * Company:         Turbochef Technology Incorporated                         *
 *                                                                            *
 * Copyright ) 2013 Turbochef Technology Inc.  All rights reserved.           *
 *                                                                            *
 * Date         Who       Notes                                               *
 * 2013-08-17   JMWare    Creating of SAGE interface to Phoenix               *
 *                                                                            *
 * ---------------------------------------------------------------------------*/
#include "p33FJ256GP710.h"
#include "common.h"
#include "timer1.h"
#include "timer23.h"
#include "TASKio.h"
#include "TASKHeater.h"
#include "TASKBlowers.h"
#include "pid.h"
#include "adc.h"                                              // Access analog channels
#include "advolts.h"                                          // Access conversion routines
#include "i2c1.h"                                             // Access I2C (intra-board communications)
#include "pwm.h"
#include "FIRESerialProtocol.h"                               // Access to structures/unions and Defines for the serial comm link
#include "TASKPhoenix.h"

unsigned char  ucVoltSET;                  			          // Check voltage setting on unit.
unsigned short acVoltage;

volatile union LSInPortUnion  LSIn;                           // GLOBAL input of MCP23017's Port B data goes
volatile union LSOutPortUnion LSOut;                          // GLOBAL output MCP23017's Port A data comes from
unsigned char ECTemp     = 0;                                 // GLOBAL where TC74's temp is put
unsigned char ECTempCntr = 0;
char cReadOrWrite        = 0xFF;
int iServiceMode = FALSE;

volatile unsigned int  uiTopStaleDtect = 0;                   // determine if Top Cook seconds is within 0-10 AND not counting.
volatile unsigned int  uiBotStaleDtect = 0;                   // "" ""


/* ---------------------------------------------------------- Init LSIO ---------------------------------------------------- *
 *                                                                                                                           *
 * ------------------------------------------------------------------------------------------------------------------------- */
void InitLSIO( void )
{
  /* INIT High Speed I/O  Preload all outputs before setting I/O direction */
  LATEbits.LATE2 = 0;                                         // HX2BOT     - output
  LATEbits.LATE3 = 0;                                         // HX1TOP     - output
  LATEbits.LATE4 = 0;                                         // RACK_BOT   - output
  LATEbits.LATE7 = 0;                                         // HSI0_1-OUT - output
  LATFbits.LATF8 = 0;                                         // ALIVE_LED  - output
  LATGbits.LATG0 = 0;                                         // IR_Enable  - output

  /*Disable @$%@#%$ Analog inputs from screwing up Parallel I/O !!! */
  /* When will Microchip fix this, and have default as analog off ? */
  AD1PCFGH |= 0x9C00 ;                                                                  // AN31, A28, A27, A26 are digitals AD1)

   /*   Pin I/O Direction   */

    TRISAbits.TRISA14 = 1;  // HX_Status - input

    TRISCbits.TRISC14 = 1;  // RTC_Pulse - input

    TRISDbits.TRISD7  = 0;  // HX Drive re-enabled making 2nd heater work. HF091409
    TRISDbits.TRISD9  = 1;  // HSI_VOLT_MODULE - input
    TRISDbits.TRISD10 = 1;  // BeltPulse1 - input
    TRISDbits.TRISD11 = 1;  // BeltPulse2 - input
    TRISDbits.TRISD12 = 1;  // BeltPulse3 - input

    TRISDbits.TRISD15 = 1;  // AC_Phase - input

    TRISEbits.TRISE2 = 0;   // HX2BOT - output
    TRISEbits.TRISE3 = 0;   // HX1TOP - output
    TRISEbits.TRISE4 = 0;   // RACK_BOT - output
    TRISEbits.TRISE7 = 0;   // HSIO_4_OUT - output (Port-bit-E7 mislabeled. was HSIO_1-OUT HF160225)

    TRISFbits.TRISF0 = 1;   // CDP_Status (K1 Relay) - input
    TRISFbits.TRISF1 = 1;   // CDS_Status (K2 Relay) - input
    TRISFbits.TRISF8 = 0;   // ALIVE_LED - output

    TRISGbits.TRISG0 = 0;   // IR_Enable - output
    TRISGbits.TRISG1 = 1;   // CDM_Status - input

    /* Make sure that the pins on the J12 SD-card port is all outputs. This should be modified when Ethernet is enabled */
    TRISDbits.TRISD8 = 0;   // SD card enable - output

    TRISGbits.TRISG6 = 0;   // SPI-Clock - output
    TRISGbits.TRISG7 = 0;   // SPI-DI - output
    TRISGbits.TRISG8 = 0;   // SPI-D0 - output
    TRISGbits.TRISG9 = 0;   // Ethernet_SEL - output

    /*******************************************/
}

/* ------------------------------------------------------------------------- *
 *                                                                           *
 * TASKPhoenix_Interface.c                                                   *
 *  Entry point to the Phoenix_Interface TASK routine                        *
 *                                                                           *
 * ------------------------------------------------------------------------- */
void TASKInterfacePhoenix( void )
{
	// ----------------------------------------------------------------------------------------------------------------
    // Step 1. "Talk" to I2C connected port expander's data port
    if( iInitHardware == 2 )
	{
    	if( cReadOrWrite )
    		ReadIOPort( 0x42, 0x13, (unsigned char*)&LSIn.Port );                       // Read GPIOB of U16 into HSInPort
		else
    		WriteIOPort( 0x42, 0x14, LSOut.Port );            	                        // Write HSOutPort into OLATA of U16

		cReadOrWrite = ~cReadOrWrite;                                                   // toggle between read-port and write port
	}

    // Normal operating mode as long as Phoenix and SAGE are in communications.
    if( OperationMode )
    {
        // ------------------------------------- Oven temperature data -----------------------------------------------------
        // TC 1 (Top):
        StatPk.TopTempF = iTopMeasTempF;                                                // Save to serial communications routins data structure

        // TC 2 (Bot):
        StatPk.BotTempF = iBotMeasTempF;                                                // Save to serial communications routins data structure

        // --------------------------------------- Oven Voltage ------------------------------------------------------------
		acVoltage = GetVoltageRMS();

		if( acVoltage == 0 )
		{
			StatPk.VOLTSENCE_Stat = VOLTSENCE_STAT;     	                            // volt module in legacy mode
	    }
		else
		{
	    	if( acVoltage > 2240 )       					                            // if the voltage is greater than 226 Vac
    			StatPk.VOLTSENCE_Stat = 1;
	    	else
    			StatPk.VOLTSENCE_Stat = 0;
  		}


		StatPk.VoltsIN = acVoltage;

        // Take Oven Voltage setting from Phoenix, which gets it's voltage info from VOLTSENCE_STAT sent to Phoenix
		VOLTSEL = CmdPk.VOLTSel;

        // init the local variable.  Set 208 base PWM modulation for power share HF160223
        if( CmdPk.VOLTSel ) 			                                                // CmdPk.VOLTSel high is 208V
        {                                                                               //
			ucVoltSET = VUS208;                                                         //
                                                                                        //
            if( CmdPk.bSingleMulti )                                                    // LO = 3phase 208
            {                                                                           //
                cPWMpwrBase = MOD3P208;                                                 //
            }                                                                           //
            else                                                                        // HI = 1phase 208
            {                                                                           //
                cPWMpwrBase = MOD1P208;                                                 //
            }                                                                           //
		}
		else  // else set 240 base PWM modulation for power share                       ..
		{		                                                                        //
			ucVoltSET = VUS240;                                                         // CmdPk.VOLTSel low is 240V
                                                                                        //
            if( CmdPk.bSingleMulti )                                                    //
            {                                                                           //
                cPWMpwrBase = MOD3P240;                                                 // HI = 1phase 240
            }                                                                           //
            else                                                                        //
            {                                                                           //
                cPWMpwrBase = MOD1P240;                                                 // LO = 3phase 240
            }                                                                           //
                                                                                        //
		}                                                                               //

		// Get Single/multi from Phoenix
        ucPhase = CmdPk.ucOvenType;

        // -------------------------------------- Oven Door Switches: ------------------------------------------------------
        StatPk.TopDoorStat = DOOR_TOP_STAT;
		StatPk.BotDoorStat = DOOR_BOT_STAT;

        // ----------------------------------- Blower Control and Status: --------------------------------------------------
        // HF160622 Blower status 1 = offline, 0 = blower ready. Can tell Phoenix blowers always ready (StatPk.TopBlowerStat = 0;)
        StatPk.TopBlowerStat  = BLOWER_TOP_STAT;
        StatPk.BotBlowerStat  = BLOWER_BOT_STAT;

    	SetTOPair( CmdPk.TopAirSetpoint );
		SetBOTair( CmdPk.BotAirSetpoint );

		// ----------------------------------- Heater Control and Status ---------------------------------------------------
		if( CmdPk.ServiceMode )
		{
			iServiceMode = TRUE;
			HX1TOP = CmdPk.TopHeater1;
			HX2TOP = CmdPk.TopHeater2;
			HX1BOT = CmdPk.BotHeater1;
			HX2BOT = CmdPk.BotHeater2;
		}
		else
		{
			iServiceMode = FALSE;

       		if( CmdPk.TopHeatEnabled ) TopSetPoint = CmdPk.TopSetpoint; else TopSetPoint = 0;
	        if( CmdPk.BotHeatEnabled ) BotSetPoint = CmdPk.BotSetpoint; else BotSetPoint = 0;
		}

		StatPk.TopHeater1Stat = HX1TOP;
		StatPk.TopHeater2Stat = HX2TOP;
		StatPk.BotHeater1Stat = HX1BOT;
		StatPk.BotHeater2Stat = HX2BOT;

        // --------------------------------------- Miscellaneous Controls -------------------------------------------------
		// Light
		LIGHT_TOP = CmdPk.TopLight;
		LIGHT_BOT = CmdPk.BotLight;

		// Alive Led
        ALIVE_LED = CmdPk.ALIVE_Led;

		// Top and Bottom Racks
		RACK_TOP = CmdPk.TopRack;
		RACK_BOT = CmdPk.BotRack;

        // ---------------------------- Get the EC temperature and report to Phoenix ---------------------------------------
		if( ( ECTempCntr++ > 100 ) && ( iInitHardware == 2 ) )                          // no need to read ECTemp everytime
		{
       		ReadIOPort( 0x9A, 0x00, &ECTemp );
       		StatPk.ECTempC = ECTemp;                                                    // Temperature is read in Celcius
			ECTempCntr     = 0;
		}


		StatPk.nu10 = CmdPk.usTopCookCTR;
        StatPk.nu11 = CmdPk.usBotCookCTR;

    }
    else // ------------------ NOT normal operating mode. Handle graceful Shutdown mode ("GSDown")here ---------------------
    {
  		// Handle gracefull Shutdown mode ("GSDown")here...
        if( iTopMeasTempF >= 150 && iTopMeasTempF <= 645 )
			SetTOPair( 50 );		                                                    // Fans on 50%
        else
			SetTOPair( 0 );			                                                    // "cool" can turn off blowers: Fans on 0%

        if( iBotMeasTempF >= 150 && iBotMeasTempF <= 645 )
			SetBOTair( 50 );		                                                    // Fans on 50%
        else
			SetBOTair( 0 );			                                                    // "cool" can turn off blowers: Fans on 0%

        // Heat OFF
		TopSetPoint = 0;
		BotSetPoint = 0;

		// Top and Bottom Racks
		RACK_TOP = 0;
		RACK_BOT = 0;

		// Top and Bottom Racks
		RACK_TOP = 0;
		RACK_BOT = 0;

        // Clear all incoming data packet information to reset conditions. I guess the compiler would gripe if needed.
		int i = 6;

        for( i = 6; i < ( NumHeaderBytes + NumCommandBytes ); i++ ) CmdPk.byte[i] = 0;  // Clear all data!
    }
}

