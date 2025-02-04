/*--------------------------------------------------------------------- *
 * FileName:        adc.c                                               *
 * Dependencies:    p33FJ256GP710.h                                     *
 * Processor:       dsPIC33F                                            *
 * Compiler:        MPLAB. C30 v2.01 or higher                          *
 *                                                                      *
 *                                                                      *
 * Port pins used:                                                      *
 *                                                                      *
 *        PIO PortA                                                     *
 * 018 AN20/INT1/RA12      Thermocouple1, Analog input                  *
 * 019 AN21/INT2/RA13      Thermocouple2, Analog input                  *
 *                                                                      *
 *         PIO PortC                                                    *
 * 006 AN16/T2CK/T7CK/RC1  RTD1, Analog input                           *
 * 007 AN17/T3CK/T6CK/RC2  RTD2, Analog input                           *
 * 008 AN18/T4CK/T9CK/RC3  AC_Current, Analog input                     *
 * 009 AN19/T4CK/T8CK/RC4  ACPhase-A2D                                  *
 *                                                                      *
 *                                                                      *
 *         PIO PortE                                                    *
 * 003 AN29/RE5            Supply_Volt,  Analog input                   *
 * 004 AN30/RE6            HSI_VOLT_MODULE (ACVoltsIN)                  *
 *                                                                      *
 * -------------------------------------------------------------------- */
#include "p33FJ256GP710.h"
#include "adc.h"

int TCLUT[11][2] =
   {  { 4095,  2000 },    // 2000F
      { 3986,  1750 },    // 1750F  (Flat from here down to 50F)
      { 2727,  1200 },    // 1200F
      { 2251,  1000 },    // 1000F
      { 1776,   800 },    // 800F
      { 1308,   600 },    // 600F
      {  851,   400 },    // 400F
      {  427,   200 },    // 200F
      {  167,   100 },    // 100F
      {   53,    50 },    //  50F (Starts to devate from here down)
      {    0,     0 }  }; //   0F

/*---------Private-Variables------------------------------------------*/
unsigned int ADCState;

/*---------Defines----------------------------------------------------*/
/* Constants */

/* ADC channels numbers */
static unsigned char ADCChannel[8] = { 16, 17, 18, 19, 20, 21, 29, 30 };

/* Public Variables */

volatile unsigned int adc[8];          // Smoothed ADC values stored here.


/*---------------------------------------------------------------------/
  Function Name: Init_ADC
  Description:   Initialize ADC module
  Inputs:        None
  Returns:       None
-----------------------------------------------------------------------*/
void Init_ADC( void )
{
  /* set port configuration here */

  AD1PCFGHbits.PCFG16 = 0;		  // force RTD1 (AN16) into analog input
  AD1PCFGHbits.PCFG17 = 0;		  // force RTD2 (AN17) into analog input
  AD1PCFGHbits.PCFG18 = 0;		  // force AC_Current (AN18) into analog input
  AD1PCFGHbits.PCFG19 = 0;		  // force ACPhase-A2D (AN19) into analog input
  AD1PCFGHbits.PCFG20 = 0;		  // force Thermocouple1 (AN20) into analog input
  AD1PCFGHbits.PCFG21 = 0;		  // force Thermocouple2 (AN21) into analog input
  AD1PCFGHbits.PCFG29 = 0;		  // force SupplyVolt (AN29) into analog input
  AD1PCFGHbits.PCFG30 = 0;		  // force ACVoltIN (AN30) into analog input

  /* set channel scanning here, auto sampling and convert, with default read-format mode */
///  AD1CON1 = 0x00E4;

  /* set channel scanning here, TIMER3 sampling and convert, with default read-format mode */
  AD1CON1 = 0x0044;

  /* select 12-bit, 1 channel ADC operation */
  AD1CON1bits.AD12B = 1;

  /* enable DMA mode (ADC module sends all results to ADBUF0 and interrupts on each result */
  ///ADCON1bits.ADDMAEN = 1;

  /* No channel scan for CH0+, Use MUX A, SMPI = 1 per interrupt, Vref= External Vref+ & Vref- */
  AD1CON2 = 0x6000;

  /* Set Samples and bit conversion time */
  AD1CON3 = 0x032F;

  /* set channel scanning here for AN4 and AN5 */
  AD1CSSL = 0x0000;

  /* channel select first channel */
  AD1CHS0 = ADCChannel[0];

  /* reset ADC interrupt flag */
  IFS0bits.AD1IF = 0;

  /* Set interrupt priority to 7 (hightest) */
  IPC3bits.AD1IP = 7;

  /* enable ADC interrupts, disable this interrupt if DMA is enabled */
  IEC0bits.AD1IE = 1;

  /* Set state machine to init */
  ADCState = 0;

  /* Pick first conversion channel */
  AD1CHS0 = ADCChannel[0];

  /* turn on ADC module */
  AD1CON1bits.ADON = 1;
}



/*---------------------------------------------------------------------
  Function Name: ADCInterrupt
  Description:   ADC Interrupt Handler
-----------------------------------------------------------------------*/

volatile unsigned long adclong[8];

void __attribute__((__interrupt__, no_auto_psv)) _ADC1Interrupt( void )
{
	unsigned int adctemp;

    /* Save off latest ADC data */
    adctemp = ADC1BUF0;

	/* Smooth it out then save it */
	adclong[ ADCState ] += (long) adctemp - (adclong[ ADCState ])/512L;  // Running average.
	adc[ ADCState ] = (unsigned int) ( adclong[ ADCState ]/512L );	 // Scale and save running ave.

    ADCState++;     // Advance to next state
    if ( 7 < ADCState )     // Have we rolled over top?
	{
		ADCState = 0;		// YES, roll back over to first
	}

	/* prepare to read next voltage */
	AD1CHS0 = ADCChannel[ ADCState ];

  /* reset ADC interrupt flag */
  IFS0bits.AD1IF = 0;
}
