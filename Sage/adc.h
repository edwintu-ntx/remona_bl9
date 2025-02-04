/* -------------------------------------------------------------------- *
 * FileName:         adc.h                                              *
 * Dependencies:    none                                                *
 * Processor:       dsPIC33F                                            *
 * Compiler:        MPLAB. C30 v2.01 or  higher                         *
 *                                                                      *
 *      Common variables used in isr_ADC.c                              *
 *                                                                      *
 *  ADC[0] = CCMeaADC            RTD1    AN16  Read RAW                 *
 *  ADC[1] =                     RTD2    AN17                           *
 *  ADC[2] = Mag current                 AN18  Read RAW                 *
 *  ADC[3] = ACPhase                     AN19                           *
 *  ADC[4] = Thermocouple 1              AN20                           *
 *  ADC[5] = Thermocouple 2              AN21                           *
 *  ADC[6] = DC supply volt              AN29                           *
 *  ADC[7] = ACVoltIN                    AN30  Read RAW                 *
 *                                                                      *
 *                                                                      *
 * -------------------------------------------------------------------- */
/*  Defines  */
#define MAXADCCOUNT 	(4095)	/* Highest ADC value for 12 bit adc */


extern int TCLUT[11][2];

/* Prototypes */
void Init_ADC( void );
extern volatile unsigned int adc[8];

