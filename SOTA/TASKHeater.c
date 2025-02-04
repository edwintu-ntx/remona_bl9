/* ---------------------------------------------------------------------------**
 *                                                                             *
 *   TASKHeaters.c                                                             *
 *                                                                             *
 *   This task maintains the heater and flame control.                         *
 *                                                                             *
 *                                                                             *
 * ---------------------------------------------------------------------------**/
#include "p33FJ256GP710.h"
#include "common.h"
#include "TASKHeater.h"
#include "TASKio.h"
#include "timer1.h"
#include "TASKBlowers.h"
#include "pid.h"
#include "adc.h"
#include "advolts.h"
#include "pwm.h"
#include "TASKPhoenix.h"

//etu added
#include <stdlib.h>  /* for rand() and srand() */
#define temp_Ht_on_Margin 5
#define temp_Ht_off_Margin 1

extern PIDController pid_Temp;
extern float setpoint;
extern float measurement;
//extern int pid_Cal(void);

Pid_Remona pidParam;
float setpoint = 1.0f;
float measurement = 0.0f;
float rtnActual = 0.0; // return actua value

int rtdRawTop;
int rtdRawBot;

int iTopMeasTempF;
int iBotMeasTempF;

int iStartModTop = FALSE;                                       // HF161208 NEW to establish sync
int iStartModBot = FALSE;

int  tempErr;                                                   // Tempreture Error
int  iFlameDrive;                                               // Burner Drive level (32767 = full on, 0 or less = minimum on)

int  TopSetPoint = 0;                                           // Cook Chamber Set Point, Deg F
int  BotSetPoint = 0;                                           // Lower Cook Chamber Set Point, Deg F

int iTopTempDiff = 0;                                           // used to determine how close Top oven cavity is to setpoint
int iBotTempDiff = 0;                                           // used to determine how close Bottom oven cavity is to setpoint

unsigned char ucPhase;

int gTopFanPwm = 0;
int gBotFanPwm = 0;

/*========================================================================*
 *                                                                        *
 *                                                                        *
 *      MAIN TASK for Heat Control                                        *
 * Added Heater Safety Relay to firmware for Dave R.                      *
 * Functionally the relay is by default off, unless one or both of the-   *
 * heaters are commanded on.                                              *
 *                                                                        *
 * ______________________________________________________________________ */
void TASKHeaters( void )
{
    // Convert raw TOP a2d input F to control top heaters and to report to Phoenix
	rtdRawTop       = adc[0];                                   // top temp [ 0,4 ] RTD1,TC1
	iTopMeasTempF   = rtd_deg_F( rtdRawTop );                   // Measure CC temperature 

// etu added
TopSetPoint = 65.1;
iTopMeasTempF = 60.9;

// *** Performing PID heater control with reference Temp
int n = rand() % 5;  /* generate a number from 0 - 9 */
setpoint = TopSetPoint;
measurement = iTopMeasTempF + n ;
pidParam.target_val = setpoint;
//pid_Cal();
rtnActual = PID_Realize( measurement, &pidParam);
iTopMeasTempF += (int)rtnActual;    
// etu
                                                        //
    iTopTempDiff = TopSetPoint - iTopMeasTempF;		            // get (negative value) distance BELOW top cavity set-point
                                                                //
	if( iTopTempDiff < 0 ) iTopTempDiff = 0;                    // don't allow neg temp differences as we can only bleed heat.
	
    //etu added
	//check if top temp diff drop lower than 5 deg F of setpoint
	//turn the top heater on
	if ( iTopTempDiff >= temp_Ht_on_Margin)
		//assert heater safety relay
		HSIO_4_OUT = 0;	

//		HX1TOP = ON;
//		TRISEbits.TRISE2 = 0;   // HX2BOT - output
//    	TRISEbits.TRISE3 = 0;   // HX1TOP - output

	//check if temp diff less than 1 deg of setpoint
	//turn the top heater off
	if (((TopSetPoint - iTopMeasTempF) < 0) && abs(TopSetPoint - iTopMeasTempF) >= temp_Ht_off_Margin)
		//de-assert heater safety relay 
		HSIO_4_OUT = 1;	

//		HX1TOP = OFF;
//		TRISEbits.TRISE2 = 1;   // HX2BOT - output
//    	TRISEbits.TRISE3 = 1;   // HX1TOP - output

// etu

// etu added 
// *** control flow 1 , issue by Bob at Jan 22 2025 
// Default set temp to 0F when power on.
TopSetPoint = 0.0;
BotSetPoint = 0.0;
// If the set temp > 120F (minimum) turn on the fan at 20% if the fan is not already set higher than 20%
if (TopSetPoint > 120)
{
	if (gTopFanPwm < 20)
		gTopFanPwm = 20; // set pwm to 20% duty cycle
}

// If the actual temp is <120 and the set temp is <120, turn off the fan
if ((iTopMeasTempF < 120) && (TopSetPoint > 120))
		gTopFanPwm = 0; // set pwm to 0% duty cycle, turn off the fan
// When the heater is commanded to turn on, if the fan is not on, set it to 60% 
	if(iStartModTop)
	{
		if (!gTopFanPwm)
			gTopFanPwm = 60;
	}
// etu
			                                                //
	// Convert raw BOTTOM a2d input F to control bottom heaters and to report to Phoenix
	rtdRawBot       = adc[1];                                   // bot temp [ 1,5 ] RTD2,TC2
    iBotMeasTempF   = rtd_deg_F( rtdRawBot );                   // Converted temp from raw data
                                                                //
    iBotTempDiff    = BotSetPoint - iBotMeasTempF;              // don't allow neg temp differences as we can only bleed heat.

    //etu added
	//check if bot temp diff drop lower than 5 deg F of setpoint
	//turn the bot heater on
	if ( iBotTempDiff >= temp_Ht_on_Margin)
		//assert heater safety relay
		HSIO_4_OUT = 0;	//??? same as TOP ???

		HX1BOT = ON;
//		TRISEbits.TRISE2 = 0;   // HX2BOT - output
//    	TRISEbits.TRISE3 = 0;   // HX1TOP - output

	//check if temp diff less than 1 deg of setpoint
	//turn the bot heater off
	if (((BotSetPoint - iBotMeasTempF) < 0) && abs(BotSetPoint - iBotMeasTempF) >= temp_Ht_off_Margin)
		//de-assert heater safety relay 
		HSIO_4_OUT = 1;	//??? same as TOP ???

		HX1BOT = OFF;
//		TRISEbits.TRISE2 = 1;   // HX2BOT - output
//    	TRISEbits.TRISE3 = 1;   // HX1TOP - output

// etu

                                                                //
	if( iBotTempDiff < 0 ) iBotTempDiff = 0;                    // don't allow negative result to affect operation

	// ------------------------------------------- Top Heater Control ------------------------------------------
	if( iServiceMode )
	{
		HSIO_4_OUT   = 1;                                       // Assert Heater Safety Relay ON
		iStartModTop = FALSE;
		iStartModBot = FALSE;
	}
	else
	{   // ------------------------------- Heater Safety Relay -----------------------------------------------------
		if( TopSetPoint || BotSetPoint ) HSIO_4_OUT = 1; else HSIO_4_OUT = 0; // Assert/de-assert Heater Safety Relay
		
    	// ------------------------------------------- Top Heater Control ------------------------------------------
		if( TopSetPoint )
		{	// Control SSR if we have a valid (non-zero) top oven set point.
			if( iTopMeasTempF < TopSetPoint ) iStartModTop = TRUE; else iStartModTop = FALSE;
		}
		else
		{   // check each time through (loop-through is continuous) to determine if we should shut off the SSR Relay.
			iStartModTop = FALSE;
		}

	    // --------------------------------------- Bottom Heater Control -------------------------------------------
		if( BotSetPoint )
		{   // Control SSR if we have a valid (non-zero) bottom oven set point.
			if( iBotMeasTempF < BotSetPoint ) iStartModBot = TRUE; else iStartModBot = FALSE;
		}
		else
		{   // check each time through (loop-through is continuous) to determine if we should shut off the SSR Relay.
			iStartModBot = FALSE;
		}
	}
}

