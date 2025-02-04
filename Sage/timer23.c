/* ---------------------------------------------------------------------------------------------*
* FileName:        isr_timer23.c                                                                *
*                                                                                               *
*   Timer2 and Timer3 are grouped together in same file, because                                *
*   it is possable to run them as a combined, 32 bit timer.                                     *
*                                                                                               *
*   Timer2 and Timer3 are special, as they are only timers that                                 *
*   can be used for PWM, A2D sampling, and other periodic hardware.                             *
* ---------------------------------------------------------------------------------------------**/
#include "p33FJ256GP710.h"
#include "common.h"
#include "timer23.h"
#include "pwm.h"
#include "TASKio.h"

char cAliveStatus           = 0xFF;
char cBlinkAliveLED         = 0;
int iTimer2Cntr             = 0;

// Following registers are all for the purpose of receiving rms voltage from the ac volt module
char startsampling          = 0;
char acVoltEnhanceMode      = 0;
volatile int bitno          = 0;
volatile int sampletick     = 0;
volatile int lookforlow     = 0;
volatile int checkbittick   = 0;
unsigned short voltagerms   = 0;
volatile long voltage       = 0;

/* -------------------------------------------------------------------- *
 *   Function Name: GetVoltageRMS                                       *
 *   Description:   Returns the RMS voltage                             *
 *   Inputs:        None                                                *
 *   Returns:       RMS votlage in the format 1204 indicating 120.4     *
 * -------------------------------------------------------------------- */
unsigned short GetVoltageRMS( void )
{
  if( acVoltEnhanceMode )
  {
    if ( voltagerms > 1000 && voltagerms < 3000 )
    {
      return voltagerms;
    }
  }

  return 0;
}

/* -------------------------------------------------------------------- *
 *   Function Name: Enable Rapid Blink                                  *
 *   Description:                                                       *
 *   Inputs:        Char                                                *
 *   Returns:                                                           *
 * -------------------------------------------------------------------- */
void EnableRapidBlink( char action )
{
	cBlinkAliveLED = action;
}

/* ---------------------------------------------------------------------------------------------*
 *   Function Name: Init_Timer2                                                                 *
 *   Description:                                                                               *
 *   Inputs:        None                                                                        *
 *   Returns:       None                                                                        *
 * ---------------------------------------------------------------------------------------------*/
void Init_Timer2( void )                        //
{                                               //
  T2CON           = 0;                          // ensure Timer 2 is in reset state
  IFS0bits.T2IF   = 0;                          // reset Timer2 interrupt flag
  IPC1bits.T2IP   = 4;                          // set Timer2 interrupt priority level to 4
  IEC0bits.T2IE   = 1;                          // enable Timer2 interrupt
  T2CONbits.TCS   = 0;                          // select internal timer clock
  T2CONbits.TCKPS = 2;                          // divide Fcy by 64
  PR2             = TIMER2PR;                   // Defined in timer23.h or calulated above
  T2CONbits.T32   = 0;                          // 16 bit mode
  T2CONbits.TON   = 1;                          // enable Timer 2 and start count
}                                               //

/* ---------------------------------------------------------------------------------------------*
 *   Function Name: Init_Timer3                                                                 *
 *   Description:   Initialize Timer3 (Only if not combined with Timer2)                        *
 *   Inputs:        None                                                                        *
 *   Returns:       None                                                                        *
------------------------------------------------------------------------------------------------*/
void Init_Timer3( void )                        //
{                                               //
  T3CON           = 0;                          // ensure Timer 3 is in reset state
  IFS0bits.T3IF   = 0;                          // reset Timer3 interrupt flag
  IPC2bits.T3IP   = 4;                          // set Timer2 interrupt priority level to 3
  IEC0bits.T3IE   = 1;                          // enable Timer3 interrupt
  T3CONbits.TCS   = 0;                          // select internal timer clock
//uncomment for 1ms interrupt
  //T3CONbits.TCKPS = 2;                          // divide Fcy by 64
  T3CONbits.TCKPS = 0;                          // divide Fcy by 64
  PR3             = TIMER3PR;                   // Defined in timer23.h
  T3CONbits.TON   = 1;                          // enable Timer3 and start count
}                                               //

/* ------------------------------------------------------------------------------------------------------------------------- T2
 * __T2Interrupt                                                                                                             T2
 * Description:   Timer1 Interrupt Handler                                                                                   T2
 * ------------------------------------------------------------------------------------------------------------------------- */
void __attribute__((__interrupt__, no_auto_psv)) _T2Interrupt( void )  //                                                    T2
{
	if( iInitHardware == 0 ) 
	{
		// Here only at startup since the port expander needed a delay between resets
		if( iTimer2Cntr > 1000 ) // one second delay
		{	
			iInitHardware = 1;
			iTimer2Cntr   = 0;
		}
		
		iTimer2Cntr++;
	}

	if( cBlinkAliveLED ) 
	{
		// rapid blinking of led engaged during download of the bin file
		if( iTimer2Cntr > 50 ) 
		{
    		if( cAliveStatus )
				ALIVE_LED = 1;
			else
				ALIVE_LED = 0;

			cAliveStatus = ~cAliveStatus;
			iTimer2Cntr  = 0;
		}
		
		iTimer2Cntr++;
	}

	if( TASKBlowersSleep )	        TASKBlowersSleep--;
	if( TASKHeatersSleep )	        TASKHeatersSleep--;
	if( TASKInterPhoenixSleep )	    TASKInterPhoenixSleep--;
	if( TASKSerialPhoenixSleep )    TASKSerialPhoenixSleep--;
	if( BLFSwriteDelay )    		BLFSwriteDelay--;
	if( ResetDelay )        		ResetDelay--;

  	IFS0bits.T2IF = 0;                                                 // reset Timer 2 interrupt flag                       T2
}                                                                      //                                                    T2

/* ------------------------------------------------------------------------------------------------------------------------- T3
 *                  Function Name: _T3Interrupt                                                                              T3

 * --------------------------------------------------------------------------------------------------------------------------*/
void __attribute__((__interrupt__, no_auto_psv)) _T3Interrupt( void )  //                                                    T3
{                                                                      //                                                    T3

  if( startsampling )                          // has the bit-banging by volt module started,                                .
  {                                            // and should we start sampling the bits coming in                            .
      sampletick++;                            // bit banging started and first low edge detected                            .
                                               // start sampling and increment 1ms counter,                                  .
      acVoltEnhanceMode = 1;                   // init the mode flag since we are in enhance mode and not the legacy mode    .
      
      if( sampletick > checkbittick )          // ac volt module baud rate is 50 bauds so each bit is present for 20ms       .
      {                                        // check to see if it has been 20ms since the last read                       .
	      checkbittick += 20;                  // this sample has been given 20ms to stabilize, roll this bit in             .
	      bitno++;                             // increment the bit counter that keeps track of how many bits read so far    .

          if( ( bitno < 9 ) || ( bitno >10 ) ) // bits 9 & 10 are stop and start bits so ignore them                         .
          {
              if( HSI_VOLT_MODULE )            // if input is high then this is a 1 ofcourse                                 .
                  voltage = voltage | 0x01;

              voltage = voltage << 1;          // left shift this bit                                                        .
          }

          if( bitno > 18 )                     // check if two bytes + 1 start + 1 stop have been received                   .
          {
	          voltagerms = voltage >> 2;       // 2 bytes received, right shift 2 to account for eliminating start and stop bits
              startsampling = 0;               // initialize for the next transmission from the volt module                  .
          }
      }
  }
  else                                         // sampling has not started, check to determine the falling edge of the       .
  {                                            // input pin that will initiate sampling of the bits indicating the voltage   .
      if( HSI_VOLT_MODULE )                    // is the input still high                                                    .
      {
	      if( sampletick > 300 )               // input is high, each transmission comes after atleast one second            .
              lookforlow = 1;                  // so wait for atleast 300ms before looking for a low edge on the input pin   .
      }
      else
      {
	      sampletick = 0;                      // input is low                                                               .
	      
	      if( lookforlow )                     // only proceed further if we have already waited for atleast 300 ms so that  .
          {                                    // we are sure this is a totally new transmission from the volt module        .
              bitno = 0;                       // initialize bitno that keeps a track of total bits read                     .
              voltage = 0;                     // each bit read is left shifted into this as received                        .
              lookforlow = 0;                  // initialize for next time                                                   .
              checkbittick = 30;               // first falling edge has been detected, it will be there for the next ~20ms, .
                                               // since we want to sample only when the edges have stabilized, so read the   .
                                               // next bit after it has had time to stabilize for atleast 10ms, therefore,   .
                                               // sample after 30 seconds and then 20ms for each bit, thereafter.            .
              startsampling = 1;               // set the start sampling flag                                                .
  	      }
      }

      if( sampletick > 3000 )                  // if at any given time we havent recieved anything for 3 seconds, then we are in legacy mode
      {
          sampletick = 0;                      // 3 seconds have elapsed and nothing received, we must be in legacy mode    .
          acVoltEnhanceMode = 0;
      }

      sampletick++;                            // increment samping tick counter                                            .
  }

   IFS0bits.T3IF = 0;                          // reset Timer 3interrupt flag                        T3
}                                              //                                                    T3

