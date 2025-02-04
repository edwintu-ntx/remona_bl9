/* ------------------------------------------------------------------- *
 * FileName:        advolts.c                                          *
 * Dependencies:    p33FJ256GP710.h                                    *
 * Processor:       dsPIC33F                                           *
 * Compiler:        MPLAB. C30 v2.01 or higher                         *
 * ------------------------------------------------------------------- */
#include "TASKHeater.h"
#include "fire.h"               // Allow access to fire serial protocol
#include "adc.h"
#include "advolts.h"

/* ------------------------------------------------------------------- *
 *     rtd_deg_F ( adc_count )      Convert ADC count into DegF        *
 *   From a curve fit of these values in Excel...                      *
 *           RTD avg     F                                             *
 *           3929.166667 600                                           *
 *           3754.333333 550                                           *
 *           3575.25     500                                           *
 *           3395.25     450                                           *
 *           3212.25     400                                           *
 *           3028.166667 350                                           *
 *           2842.25     300                                           *
 *           2652.416667 250                                           *
 *           2462.5      200                                           *
 *           2268.416667 150                                           *
 *           2075.25     100                                           *
 *           1880.666667 50                                            *
 *           1684.083333 0                                             *
 *   ... gives folling "best fit" linear approximation:                *
 *       rtd_deg_F = 0.2668 * adc - 454.47 or for better fit from 450F *
 *       to 550F rtd_deg_F = 0.2786 * adc - 495.73                     *
 *       = (2786 * adc)/10000 - 495.73                                 *
 *       = ( (2786 * adc) - 49573000 )/10000                           *
 *    - - - - - - - - - - - - - - - - - - - -                          *
 *   ... also gives folling "best fit" binomial approximation:         *
 *       rtd_deg_F = 7.31814E-6 * adc^2 + 0.22568718 * adc - 400.2249151
 *       rtd_deg_F = adc^2 / 136647 + 2257 * adc / 10000 - 400.2249    *
 *       rtd_deg_F = adc^2 / 136647 + (2257 * adc -4002249)/ 10000     *
 *           Doing a little change, gives an emperically better number:*
 *       rtd_deg_F = adc^2 / 135200 + (2256 * adc -4002249)/ 10000     *
 * ------------------------------------------------------------------- */
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
int rtd_deg_F( unsigned int adc )
{
  long ltemp;

  //Binomial Emperically accurate, 6 board, RTD Avg based
  ltemp = ( ( (long)adc * (long)adc ) / 135200L) + ( ( 2256L * (long)adc ) - 4002249L ) / 10000L;

  /* Check to see if iCCMea is a bad temperature! */
  if( adc > 4090)  ltemp = (long)BADTEMP; // Open RTD, or over about 645 F
  if( adc < 1500 ) ltemp = (long)BADTEMP;  // Shorted RTD, or under around 0 F

  return (int)ltemp;
}

/*--------------------------------------------------------------------*
 *                                                                    *
 *     adc_deg_F( int tempF )       Find ADC count for a tempreture   *
 *                                                                    *
 *      returns an ADC count for a temp,                              *
 *              or MAXADCCOUNT if it's a bad temp.                    *
 *                                                                    *
 *--------------------------------------------------------------------*/
unsigned int adc_deg_F( int tempF )
{
  unsigned int adcval;
  int error;
  int loopcounter;

  if( BADTEMP == tempF )        // is it a failed temp?
  {
		return MAXADCCOUNT;		// YES, return max ADC count;
  }

  adcval = tempF * 4 + 1600;    /* A guess to get us close */
  loopcounter = 100;            /* We will only do this 100 times, max, then give up! */
  error = 1;                    /* force it to go though loop at least once */

  while( error )     // While there is some error,
	{
      error = tempF - rtd_deg_F( adcval ); // Calulate error,
      adcval += error;                     // Add it back in
      loopcounter--;

      if( !loopcounter )  //Have we tried to find it for too long?
          break;          // YES, give up while we are close!
    }

  return adcval;
}

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

                                                                                                   
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *
 *  tc_deg_F( adc count, pointer to lookup table )
 *
 *  takes adc count from thermocuple, and a pointer to a lookup table (in tc.h)
 *
 *   (The last entry MUST be {0, 0} so that the function can detect the end of 
 *    the table.)
 *
 *  Returns Degrees FOR
 *          -1 if error in table (Double entries)
 *          -2 if Deg_F is out of range of table
 *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
# if 0
int tc_deg_F( unsigned int x, const int* lut[][2] )
{
	unsigned int i = 0;			// Start at the begining of the table
	int x1, x2, y, y1, y2;

	while(1)  // while not done
	{
		x1 = (int)lut[i+1][0];
		y1 = (int)lut[i+1][1];
		x2 = (int)lut[ i ][0];
		y2 = (int)lut[ i ][1];

		if( x2 == x1) // ERROR IN TABLE! About to Divide by Zero!)
			return -1;  // Report FAILURE!

		if(x == x1)      // Is it right on the first point?
			return( y1 ); 

		if(x == x2)      // Is it right on the second point?
			return( y2 );

		if( (x > x1) && (x < x2) ) // Is x between the two table entries?
		{
			y =(int)( ( (long)(x-x1) * (long)(y2-y1) ) / (long)(x2-x1)) + y1; // YES, to a Linear Interpolation.

			return y;
		}

		if( ( 0 == y2) || (0 == y1) ) // We are at end of table and x wasn't inside!
		{
			return -2;  // Report FAILURE!
		}
		else
		{
			i++; // Advace the index to the next pair in table
			if ( i > 11 )
				return -2;
		}
	}
	// never should reach here.
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *
 *  tc_degF_ADC( Deg_F, pointer to lookup table)
 *
 *  Converts Tempereture (Degs F) to an ADC count for use in Heater PID
 *  
 *
 *   (The last entry MUST be {0,0} so that the function can detect the end of 
 *    the table.)
 *
 *  Returns ADC count OR
 *          -1 if error in table (Double entries)
 *          -2 if Deg_F is out of range of table
 *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
int tc_degF_ADC( int x, const int* lut[][2] )
{
	unsigned int i = 0;		// Start at the begining of the table
	int x1, x2, y, y1, y2;

	while(1)  // while not done
	{
		y1 = (int)lut[i+1][0];
		x1 = (int)lut[i+1][1];
		y2 = (int)lut[ i ][0];
		x2 = (int)lut[ i ][1];

		if( x2 == x1) // ERROR IN TABLE! About to Divide by Zero!)
			return -1;  // Report FAILURE!

		if(x == x1)      // Is it right on the first point?
			return( y1 ); 

		if(x == x2)      // Is it right on the second point?
			return( y2 );

		if( (x > x1) && (x < x2) ) // Is x between the two table entries?
		{
			y =(int)( ( (long)((int)x-x1) * (long)(y2-y1) ) / (long)(x2-x1)) + y1; // YES, to a Linear Interpolation.

			return y;
		}

		if( ( 0 == x2) || (0 == x1) ) // We are at end of table and x wasn't inside!
		{
			return -2;  // Report FAILURE!
		}
		else
		{
			i++; // Advace the index to the next pair in table
			if ( i > 11 )
				return -2;
		}
	}
	// never should reach here.
}
#endif
