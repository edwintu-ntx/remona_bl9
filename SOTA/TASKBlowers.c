/* ---------------------------------------------------------------------------
 *   TASKBlowers.c                                                           *
 *  This task maintains both lower and upper blower                          *
 *                                                                           *
 * -------------------------------------------------------------------------**/
#include "p33FJ256GP710.h"
#include "common.h"
#include "TASKBlowers.h"
#include "TASKio.h"
#include "timer1.h"
#include "pwm.h"
#include "FIRESerialProtocol.h"                        // Allow access to serial Link Protocol to Phoenix

enum FanSTATE      TOPFanSTATE = FOFF;                 //
enum FanSTATE      BOTFanSTATE = FOFF;                 //
                                                       //
int iTOPairSpeed = 0;                                  // Top Blower speed set point, in percentage (100 - 0)
int iBOTairSpeed = 0;                                  // Bottom Blower speed set point, in percentage (100 - 0)
char topAirON = FALSE;                                 // TRUE when blower is to be running, otherwize FALSE
char botAirON = FALSE;                                 // TRUE when blower is to be running, otherwize FALSE

unsigned int FtopCNTdwn = 0;                           //
unsigned int FbotCNTdwn = 0;                           //

/* ------------------------------------------------------------------------*
 *      SetTOPair( int speed )                                             *
 *   Set's Top Air blower speed                                            *
 *    speed is between MAXAIR (100) and MINAIR (10)                        *
 *           Anything outside those are clipped to                         *
 *           closest                                                       *
 *    speed is also set to nearest divisable of 5.                         *
 *  RETURNS: speed as saved in topAir                                      *
 * ------------------------------------------------------------------------*/
void SetTOPair( int speed )                            //
{                                                      //
  int error;                                           //
                                                       //
  if( speed == 0 )                                     //
  {                                                    //
    topAirON    = FALSE;                               //
    TOPFanSTATE = FOFF;                                // AND force state
  }                                                    //
  else                                                 //
  {                                                    //
    topAirON = TRUE;                                   //
                                                       //
    if( FOFF == TOPFanSTATE )                          //IF the current fans state is OFF,
        TOPFanSTATE = FSTART;                          // Then change it to START
  }                                                    //
                                                       //
  if( speed > MAXAIR ) speed = MAXAIR;                 // Busted top of Blower speed? YES, clip it to Max
                                                       //
  error = speed % 5;                                   //
                                                       //
  if( error != 0 )                                     // Is speed a multiple of 5?
    {                                                  //
      if( error > 2 ) error = error - 5;               // round up (Negive, negitive error)
                                                       //
      speed -= error;                                  // fix speed to be mulple of 5
    }                                                  //
                                                       //
  iTOPairSpeed = speed;                                //
                                                       //
}                                                      //

/* ------------------------------------------------------------------------*
 *      SetBOTair( int speed )                                             *
 *   Set's Bottom Air blower speed                                         *
 *    speed is between MAXAIR (100) and MINAIR (10)                        *
 *           Anything outside those are clipped to                         *
 *           closest                                                       *
 *    speed is also set to nearest divisable of 5.                         *
 *  RETURNS: speed as saved in bottomAir                                   *
 * ------------------------------------------------------------------------*/
void SetBOTair( int speed )                          //
{                                                    //
  int error;                                         //
                                                     //
  if( speed == 0 )                                   //
  {                                                  //
    botAirON    = FALSE;                             //
    BOTFanSTATE = FOFF;                              // AND force state
  }                                                  //
  else                                               //
  {                                                  //
    botAirON = TRUE;                                 //
    if( FOFF == BOTFanSTATE )                        // IF the current fans state is OFF,
        BOTFanSTATE = FSTART;                        // Then change it to START
  }                                                  //
                                                     //
  if( speed > MAXAIR ) speed = MAXAIR;               // Busted top of Blower speed? YES, clip it to Max
                                                     //
  error = speed % 5;                                 //
                                                     //
  if( error != 0 )                                   // Is speed a multiple of 5?
  {                                                  //
    if( error > 2 ) error = error - 5;               // round up (Negive, negitive error)
                                                     //
    speed -= error;                                  // fix speed to be mulple of 5
  }                                                  //
                                                     //
  iBOTairSpeed = speed;                              //
                                                     //
}                                                    //

/* ------------------------------------------------------------------------*
 *      isTopAirOn ( void)                                                *
 *   Tests to see if both blowers are up and running                       *
 *  RETURNS: TRUE if a blower have started                                 *
 *           FALSE if both blowers aren't                                  *
 * ------------------------------------------------------------------------*/
char isTopAirOn( void )                         	 //
{                                                    //
 	return( topAirON );                              //
}                                                    //

/* ------------------------------------------------------------------------*
 *      isBotAirOn ( void)                                                *
 *   Tests to see if both blowers are up and running                       *
 *  RETURNS: TRUE if a blower have started                                 *
 *           FALSE if both blowers aren't                                  *
 * ------------------------------------------------------------------------*/
char isBotAirOn( void )                         	 //
{                                                    //
 	return( botAirON );                              //
}                                                    //

/* ========================================================================*
 *                                                                         *
 *     MAIN TASK for Blower Control                                        *
 *                                                                         *
 * NOTES: BM1_OUT and BM2_OUT are sourced from the same pins on the CPU.   *
 * Each has two possible connection points found on the B and C Molex      *
 * connectors; B1 = BM1-OUT-OC, B2 = BM2-OUT-OC both are OPEN COLLECTOR.   *
 * Legacy NGC requirements caused the inclusion of BM1_EN (pin C-2) and    *
 * BM2_EN (pin B-11). BM1_OUT (GPA1 pin-22) and BM2_OUT (GPA0 pin-21) are  *
 * outputs on U15 MCP23017 port expander. I chose to have David move the   *
 * harness wires to the "OC" version because the default based on NGC is to*
 * use the OC (Open Collector) variant. It's better to err on the side of  *
 * convention and the historically proven (successful) approach. HF030207  *
 * ----------------------------------------------------------------------- */
void TASKBlowers( void )
{
      //
      // Only run this code if BLOWER #1 Engine has been asked to turn on:
      //                                                                              //
        //========== 1st Statemachine maintains top fan ==================================
        if( topAirON )                                   // If top blower should be running,
        {                                                //
        	// if status indicators from motor control are wrong for state, re-start
			if( BLOWER_TOP_STAT && TOPFanSTATE == FRUNING ){ TOPFanSTATE = FSTART; }     

            switch( TOPFanSTATE )                        //
              {                                          //
                 case FOFF: //__| How appripoe'          ..
                      pwm_1( MINAIR );                   //
                  	  BM1_OUT     = 0;                   // Make sure that enable is OFF
                      break;                             //
  
                 case FSTART: //__|                      ..
                      BM1_OUT     = 1;                   // Pull enable line LOW (NGC)
                      FtopCNTdwn  = 0;                   // delay 10 divisions of TASKBlower
                      TOPFanSTATE = FTEST;               //
                      break;                             //
  
                 case FTEST: //__|                       ..
                      if( FtopCNTdwn++ > 15 )            //
                      {                                  //   Test for good stat response
						  if( BLOWER_TOP_STAT )          //
                          {                              //
                              BM1_OUT     = 0;           // Set to OFF to be cycled ON during retry (FSTART
                              pwm_1( MINAIR );           //
                              TOPFanSTATE = FSTART;      // go round and kick it all again
                          }                              //
                          else                           //  if input PULLED LOW then FAN responded READY (LOW)
                          {                              //
                              TOPFanSTATE = FRUNING;     // success
                                                         //
                              // 62059 PWM = 10V out ( 0xFFFF * ( 10/10.56) set PWM to 10V * topair% / 100%
                              pwm_1( (unsigned int)( AIRSPD10L * (long)iTOPairSpeed / 100L ) );
                          }                              //
                      }                                  //                                                         //
                      break;                             //
  
                 case FRUNING: //__| constantly tst for fan self-disengaged
                      if( BLOWER_TOP_STAT )              // report fan no longer running and KILL heaters
                      {                                  // (for some internal to fan controller reason)
                          TOPFanSTATE = FOFF;            //
                      }                                  //
                      else                               //
                      {                                  //
                          // intercept and scale last 20% to reduce current to acceptable values and still provide deflection.
                          switch( iTOPairSpeed )
                          {
                              case  80: iTOPairSpeed = 77; break;
                              case  85: iTOPairSpeed = 79; break;
                              case  90: iTOPairSpeed = 81; break;
                              case  95: iTOPairSpeed = 83; break;
                              case 100: iTOPairSpeed = 85; break;
                              default :                    break; // all other 5% increments remain unchanged. HF160812
                          }
 
                          pwm_1( (unsigned int)( AIRSPD10L * (long)iTOPairSpeed / 100L ) );
                      }                                  //
                                                         //
                      break;                             //
              }                                          //
        }
        else // default condition passed whenever fan not on
        {                                                //
            pwm_1( 0 );                                  // ZERO volts out
           	BM1_OUT     = 0;                       	     // AND Make sure that enable is OFF
            TOPFanSTATE = FOFF;                          // AND force state
        }                                                //
      
        // ============== 2nd Statemachine maintains state of Bottom Fan =========================
        if( botAirON )                                   // If top blower should be running,
        {                                                //
        	// Only allow to be set once per "failed" event, from ANY F1 source    .
        	if( BLOWER_BOT_STAT && BOTFanSTATE == FRUNING ){ BOTFanSTATE = FSTART; } 

            switch( BOTFanSTATE )                        //
              {                                          // Note: case FOFF has no exit to FSTART.
                 case FOFF:
                      pwm_2( MINAIR );                   //
                      BM2_OUT     = 0;                   // Make sure that enable is OFF
                      break;                             //
  
                 case FSTART: //__|    Attempt the Fan ON
                      BM2_OUT     = 1;                   // Pull enable line LOW (NGC)
                      FbotCNTdwn  = 0;                   // delay 50 divisions of TASKBlower
                      BOTFanSTATE = FTEST;               //
                      break;                             //
  
                 case FTEST: //__|                       ..
                      if( FbotCNTdwn++ > 15 )            //
                      {                                  //   Test for good stat response
                          if( BLOWER_BOT_STAT )          //
                          {                              //
                              BM2_OUT     = 0;           // Set to OFF to be cycled ON during retry (FSTART
                              pwm_2( MINAIR );           //
                              BOTFanSTATE = FSTART;      // go round and kick it all again
                          }                              //
                          else                           //
                          {                              // if input PULLED LOW then FAN responded READY (LOW)
                              BOTFanSTATE = FRUNING;     // success
                                                         //
                              // 62059 PWM = 10V out     ( 0xFFFF * ( 10/10.56) set PWM to 10V * topair% / 100%
                              pwm_2( (unsigned int)( AIRSPD10L * (long)iBOTairSpeed / 100L ) );
                          }                              //
                      }                                  //
                      break;                             //
  
                 case FRUNING: //__| constantly test If fan self-disengaged |__
                      if( BLOWER_BOT_STAT )              // report fan no longer running and KILL heaters
                      {                                  // (for some internal to fan controller reason)
                          BOTFanSTATE = FOFF;            //
                      }                                  //
                      else                               //
                      {                                  //
                          // intercept and scale last 20% to reduce current to acceptable values and still provide deflection.
                          switch( iBOTairSpeed )
                          {
                             case  80: iBOTairSpeed = 77; break;
                             case  85: iBOTairSpeed = 79; break;
                             case  90: iBOTairSpeed = 81; break;
                             case  95: iBOTairSpeed = 83; break;
                             case 100: iBOTairSpeed = 85; break;
                             default :                    break; // all other 5% increments remain unchanged. HF160812
                          }
                              
                          pwm_2( (unsigned int)( AIRSPD10L * (long)iBOTairSpeed / 100L ) );
                      }                                  //
                                                         //
                      break;                             //
              }                                          //
        }                                                //
        else  // default conditions passed whenever fan  not on
        {                                                //
            pwm_2( 0 );                                  // NO volts out
           	BM2_OUT     = 0;                         	 // AND Make sure that enable is OFF
            BOTFanSTATE = FOFF;                          // AND force state
        }
}

