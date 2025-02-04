/* -------------------------------------------------------------------*
 *     pwm.c        Output Capture controls (PWM, Analog outs)        *
 *                                                                    *
 * -------------------------------------------------------------------*
 * NOTES:                                                             *
 *  All PWM run from Timer3                                           *
 *  Currently, BEEP PWM, and HX_DRIVE PMW are not used and are        *
 *  controlled elseware as GPIO Port pins. Later they may be driven   *
 *  as PWM's                                                          *
 * -------------------------------------------------------------------*/
#include "p33FJ256GP710.h"
#include "common.h"
#include "timer23.h"
#include "pwm.h"

/* * *  Defines */
#define PWMT2DIV ( 0xFFFFul / (TIMER2PR+1) )
#define PWMT3DIV (0xFFFF/TIMER3PR)              // #define TIMER3PR 0x03FE // for 39kHz, for 10bit PWM cntl

/* ---------------------------------------------------------------*
 *      pwm_init()                                                *
 *                                                                *
 *  Init all Output Compare pins for use as PWM                   *
 *                                                                *
 * ---------------------------------------------------------------*/
void pwm_init( void )
{   /* OC1: AnalogOut1 */
	OC1CON				= 0;	// Disable OC and clear all controls
	OC1R				= 0;	// Preload PWM to 0% (always low)
	OC1CONbits.OCTSEL 	= 1;	// Use Timer3
	OC1RS				= 0;	// Set next PWM cycle to 0% (always low)
	OC1CONbits.OCM 		= 6;	// PWM mode without Fault Protection

	/* OC2: AnalogOut2 */
	OC2CON				= 0;	// Disable OC and clear all controls
	OC2R				= 0;	// Preload PWM to 0% (always low)
	OC2CONbits.OCTSEL 	= 1;	// Use Timer3
	OC2RS				= 0;	// Set next PWM cycle to 0% (always low)
	OC2CONbits.OCM 		= 6;	// PWM mode without Fault Protection

	/* OC3: AnalogOut3 */
	OC3CON				= 0;	// Disable OC and clear all controls
	OC3R				= 0;	// Preload PWM to 0% (always low)
	OC3CONbits.OCTSEL 	= 1;	// Use Timer3
	OC3RS				= 0;	// Set next PWM cycle to 0% (always low)
	OC3CONbits.OCM 		= 6;	// PWM mode without Fault Protection

	/* OC4: AnalogOut4 */
	OC4CON				= 0;	// Disable OC and clear all controls
	OC4R				= 0;	// Preload PWM to 0% (always low)
	OC4CONbits.OCTSEL 	= 1;	// Use Timer3
	OC4RS				= 0;	// Set next PWM cycle to 0% (always low)
	OC4CONbits.OCM 		= 6;	// PWM mode without Fault Protection

	/* OC5: AnalogOut5 */
	OC5CON				= 0;	// Disable OC and clear all controls
	OC5R				= 0;	// Preload PWM to 0% (always low)
	OC5CONbits.OCTSEL 	= 1;	// Use Timer3
	OC5RS				= 0;	// Set next PWM cycle to 0% (always low)
	OC5CONbits.OCM 		= 6;	// PWM mode without Fault Protection

	/* OC6: Iout */
	OC6CON				= 0;	// Disable OC and clear all controls
	OC6R				= 0;	// Preload PWM to 0% (always low)
	OC6CONbits.OCTSEL 	= 1;	// Use Timer3
	OC6RS				= 0;	// Set next PWM cycle to 0% (always low)
	OC6CONbits.OCM 		= 6;	// PWM mode without Fault Protection

	/* OC7: Beep PWM */
	OC7CON				= 0;	// Disable OC and clear all controls
	OC7R				= 0;	// Preload PWM to 0% (always low)
	OC7CONbits.OCTSEL 	= 1;	// Use Timer3
	OC7RS				= 0;	// Set next PWM cycle to 0% (always low)
	OC7CONbits.OCM 		= 6;	// PWM mode without Fault Protection

    /* OC8: NOW a high speed digital out (HX_Drive)  */
    //OC8CON             = 0;    // Disable OC and clear all controls
    //OC8R               = 0;    // Preload PWM to 0% (always low)
    //OC8CONbits.OCTSEL  = 1;    // Use Timer3
    //OC8RS              = 0;    // Set next PWM cycle to 0% (always low)
    //OC8CONbits.OCM         = 6;    // PWM mode without Fault Protection
}

/* * ----------------- ------------------* ------------------*
 *                                                           *
 *      pwm_X( Width )                                       *
 *  Set PWM width for PWM #X                                 *
 *  With timer 3 running with a PR of 0xFFFE:                *
 *       Width                                               *
 *      0x0000  =   Output always Low                        *
 *      0x1999  =   10% duty cycle                           *
 *      0x3FFF  =   25% duty cycle                           *
 *      0x7FFF  =   50% duty cycle                           *
 *      0xAFFF  =   75% duty cycle                           *
 *      0xE661  =   90% duty cycle                           *
 *      0xFFFF  =   Output always High                       *
 *                                                           *
 * ----------------- ------------------* ------------------* */
/* AnalogOut1 */
void pwm_1( unsigned int width )
{
  OC1RS = width/PWMT3DIV;   // Set PWM
}

/* AnalogOut2 */
void pwm_2( unsigned int width )
{
  OC2RS = width/PWMT3DIV;   // Set PWM
}
