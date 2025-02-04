/*--------------------------------------------------------------------*
* FileName:         advolts.h
* Dependencies:    none
* Processor:       dsPIC33F
* Compiler:        MPLAB. C30 v2.01 or higher
*
*		Common items used in advolts.c
*
*--------------------------------------------------------------------*/
/* Prototypes */
int rtd_deg_F ( unsigned int adc_count );
unsigned int adc_deg_F( int tempF );
int tc_deg_F( unsigned int x, const int* lut[][2] );
int tc_degF_ADC( int x, const int* lut[][2] );
