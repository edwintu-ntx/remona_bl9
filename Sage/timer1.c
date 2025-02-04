/* ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| *
 *  TABS ARE SET TO 4                                                                                                        *
 * FileName:        isr_timer1.c                                                                                             *
 * Dependencies:    p24FJ128GA010.h                                                                                          *
 * Processor:       PIC24Fj                                                                                                  *
 * Compiler:        MPLABR C30 v2.01 or higher                                                                               *
 *                                                                                                                           *
 * ------------------------------------------------------------------------------------------------------------------------- */
#include "p33FJ256GP710.h" 
#include "common.h"
#include "timer1.h"
#include "TASKio.h"
#include "TASKHeater.h"
#include "TASKBlowers.h"
#include "pwm.h"
#include "adc.h"
#include "FIRESerialProtocol.h"                                     // Allow access to Serial Protocol interface functions
#include "TASKPhoenix.h"                                            //
#include "fire.h"                                                   // HF140619 include Packet unions for protocol to ID Oven type
#include "stdlib.h"

       volatile unsigned int  ticks                 = 0;            //
                                                                    // pre-divisor that allows 1% values -
       volatile          int  iHeatPeriodTop        = 0;            // Period reg for HX heaters
       volatile          int  iHeaterTopdcyc        = 0;            // On time of HX heater in percent (Duty Cycle)
       volatile          int  iHeatPeriodBot        = 0;            // Period reg for IR heaters
       volatile          int  iHeaterBotdcyc        = 0;            // On time of IR heater in percent (Duty Cycle)
       volatile          int  iRxTm;                                // Receive timer...
       volatile          int  commTimer             = 0;            // Communications protocol timer!
       volatile          int  serialTimer           = 0;            // Serial Timer - used to timeout packets
       volatile          int  iLimit;

extern volatile unsigned int  RxStateMachine;                       // The receive state location for the state machine...
extern volatile unsigned int  u1_rxIndex;                           // Index to next address to store incoming data

       volatile unsigned int  uiTopDoorTmr          = 0;            // accrue time door is open for alarm.
       volatile unsigned int  uiBotDoorTmr          = 0;            // accrue time door is open for alarm.

       volatile          char cTopHTRmodTmr         = 0;	        // initialize
       volatile          char cBotHTRmodTmr         = 0;		    // initialize

       volatile unsigned char ucTopDoorHtrBLK       = 0;            // Flags to interrupt heaters   HF160514
       volatile unsigned char ucBotDoorHtrBLK       = 0;            //

       volatile unsigned char ucTopCookHtrBLK       = 0;            // Flag to block heaters during last 10 seconds of cook to bleed-
       volatile unsigned char ucBotCookHtrBLK       = 0;            //-off large thermal mass of calrods HF160825


	   volatile unsigned short usTopCompOffTime;                    // Calculated point on cook time-line (seconds) to shutdown heater
       volatile unsigned short usBotCompOffTime;                    //

       volatile unsigned char  cTopAirComp          = 0;            // HF160212 Added to offset increase top fan current.
       volatile unsigned char  cBotAirComp          = 0;            // HF160212 Added to offset increase bot fan current.

       volatile unsigned char  cPWMpwrShare;                        // var for base offset for heaters.
       volatile unsigned char  cPWMpwrBase;                         // global used to set 208/240 from CmdPk.VOLTSel  HF

       // improving the heat-engine without adding PID back, as it takes a bit more code space.
       volatile          int   iLastTopTemp;                        // As there's no timing in TASKHeater now I have to put -
       volatile          int   iDeltaTopRate;                       // this in the interrupt.

       volatile          int   iLastBotTemp;                        //
       volatile          int   iDeltaBotRate;                       //

       volatile          int   iSetTopPWM;                          // new target register for PWM duty cycle
       volatile          int   iSetBotPWM;                          //

       volatile          int   iTopAccruErrr;                       // additive to absolutle error.
       volatile          int   iBotAccruErrr;                       // additive to absolutle error.

       volatile unsigned char  ucSampleSecs;                        // measurement window for delta temp numbers.

       volatile unsigned char ucTopEndCookBLK = FALSE;              // heaters ENABLED until cook seconds [ 9,8,7,6,5,4,3,2,1 ] end of cook secs
       volatile unsigned char ucBotEndCookBLK = FALSE;              // heaters ENABLED until cook seconds [ 9,8,7,6,5,4,3,2,1 ] end of cook secs


/* -------------------------------------------------------------------- *
 *   Function Name: Init_Timer1                                         *
 *   Description:   Initialize Timer1 for 1 second intervals            *
 *   Inputs:        None                                                *
 *   Returns:       None                                                *
 * -------------------------------------------------------------------- */
void Init_Timer1( void )
{
  T1CON           = 0;           /* ensure Timer 1 is in reset state         */
  IFS0bits.T1IF   = 0;           /* reset Timer 1 interrupt flag             */
  IPC0bits.T1IP   = 1;           /* set Timer1 interrupt priority level to 1 */
  IEC0bits.T1IE   = 1;           /* enable Timer 1 interrupt                 */
  T1CONbits.TCS   = 0;           /* select internal timer clock              */
  T1CONbits.TCKPS = 2;           /* divide Fcy by 64                         */
  PR1             = TIMER1PR;
  T1CONbits.TON   = 1;           /* enable Timer 1 and start count           */
}

/* -------------------------------------------------------------------- *
 *   Function Name: SetBotHeaterPWM                                     *
 *   Description:   Sets the IR heater PWM percentage                   *
 *   Inputs:        None                                                *
 *   Returns:       IR percentage (0 - 100)                             *
 * -------------------------------------------------------------------- */
void SetBotHeaterPWM( int val )
{
     iHeaterBotdcyc = val;	                                            // Set heater PWM
}

/* ========================================================================================================================= *
 * Function Name: _T1Interrupt                                                                                               *
 * 1/10th sec period for heater modulation. Follow on tests always have a value greater than ZERO in period counter, as this *
 * increment is performed 1st. This allows the ON time to be set to ZERO, thus shutting off the PWM output. always running,  *
 * never quits, but it's it's not what's controlling the heaters.                                                            *
 *                                                                                                                           *
 * ========================================================================================================================= */
void __attribute__((__interrupt__, no_auto_psv)) _T1Interrupt( void )
{
  
// +++++++++++++++++++++++++++++++++++++++ Heater Operation Logic ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ .
    //_________________________________________________________________________________                                                  .
    // Heater modulation period is 100 ms giving 10 Hz with 0 to 100% duty cycle                                                         .
    if( cTopHTRmodTmr     ) cTopHTRmodTmr--;                            //                                                               .
    if( cTopHTRmodTmr < 1 ) cTopHTRmodTmr = 100;                        //                                                               .
                                                                        //                                                               .
    if( cBotHTRmodTmr     ) cBotHTRmodTmr--;                            //                                                               .
    if( cBotHTRmodTmr < 1 ) cBotHTRmodTmr = 100;                        // HF160203                                                      .

    // _____________                                                 __________________
    // _____________PWM control for BOTH top heater elements together__________________ ucTopDoorHtrBLK can override iStartModTop        .
    //                    LO = enabled        LO = enabled                                                                               .
    if( iStartModTop && !ucTopDoorHtrBLK )                              //                                                               .
    {                                                                   //                                                               .
        // control ON/OFF dutycycle, counting down (bottom of module)                                                                    .
        if( cTopHTRmodTmr < iSetTopPWM - cPWMpwrBase )                  //                                                               .
        {                                                               //                                                               .
            HX1TOP = ON;  HX2TOP = ON;                                  //                                                               .
        }                                                               //                                                               .
        else                                                            //                                                               .
        {                                                               //                                                               .
            HX1TOP = OFF; HX2TOP = OFF;                                 //                                                               .
        }                                                               //                                                               .
    }                                                                   //                                                               .
    else  // TOP heaters OFF. ensure we don't start or continue un-commanded.                                                            .
    {                                                                   //                                                               .
        if( !iServiceMode )                                             //                                                               .
        {                                                               //                                                               .
            HX1TOP = OFF; HX2TOP = OFF;                                 //                                                               .
        }                                                               //                                                               .
                                                                        //                                                               .
        cTopHTRmodTmr = 0;                                              // ensure we don't start or continue uncomanded                  .
        iTopAccruErrr = 0;                                              // If heat off (measured = setpoint) clear accrual or roll-up.   .
    }                                                                   // part of expert system heat plant below HF160523               .

    // _____________                                                    _______________                                                  .
    // _____________PWM control for BOTH bottom heater elements together_______________ ucBotDoorHtrBLK can override iStartModBot        .
    //                    LO = enabled         LO = enabled                                                                              .
    if( iStartModBot && !ucBotDoorHtrBLK )                              // heater task enabled AND not blocked -                         .
    {                                                                   // -(blocked = 1) HF160514                                       .
        // control ON/OFF dutycycle, counting down (bottom of module)                                                                    .
        if( cBotHTRmodTmr < iSetBotPWM - cPWMpwrBase )                  // We will add back 208/240 and Fan comp later.                  .
        {                                                               //                                                               .
            HX1BOT = ON;  HX2BOT = ON;                                  //                                                               .
        }                                                               //                                                               .
        else                                                            //                                                               .
        {                                                               //                                                               .
   		    HX1BOT = OFF; HX2BOT = OFF;                                 //                                                               .
        }                                                               //                                                               .
    }                                                                   //                                                               .
    else // BOTTOM heaters OFF. Ensure we don't start or continue un-commanded.                                                          .
    {                                                                   //                                                               .
        if( !iServiceMode )                                             //                                                               .
        {                                                               //                                                               .
            HX1BOT = OFF; HX2BOT = OFF;                                 //                                                               .
        }                                                               //                                                               .
                                                                        //                                                               .
        cBotHTRmodTmr = 0;                                              //                                                               .
        iBotAccruErrr = 0;                                              // If heat off (measured = setpoint) clear accrual or roll-up.   .
    }                                                                   // part of expert system heat plant below HF160523               .

    // +++++++++++++++++++++++++++++++++++++++  Serial Communication with Phoenix Logic  +++++++++++++++++++++++++++++++++++++++++++++++ .
    // Receive state machine timer...                                                                                                    .
    if( iRxTm > 0 )                                                     //                                                               .
    {                                                                   //                                                               .
        iRxTm--;                                                        //                                                               .
    }                                                                   //                                                               .
    else                                                                //                                                               .
    {                                                                   //                                                               .
        if( u1_rxIndex != 0 )                                           //                                                               .
        {                                                               //                                                               .
            iRxTm           = -1;                                       //                                                               .
            RxStateMachine  =  0;                                       // Restart watching for the start of the header!                 .
        }                                                               //                                                               .
    }                                                                   //                                                               .
                                                                        //                                                               .
    if( serialTimer > 0 ) serialTimer--;                                // Count of sub-secs for serial timer (timeouts)                 .
    if( commTimer   > 0 ) commTimer--;                                  // Count of sub-seconds for network timer                        .

    // ++++++++++++++++++++++++++++++++ Accrue milliseconds until = ONE_second +++++++++++++++++++++++++++++++++++++++++++++++++++++++++ .
    if( ticks < TICKSPERSEC )				                            // ticks/sec = 1000 1ms accrual of ONE SEC time                  .
    {                                                                   //                                                               .
        ticks++;                                                        //                                                               .
    }                                                                   //                                                               .
    else // 11111111 Do all ONE_second activities 1111111111111111111111111111111111111111111111111111111111111111111111111111111111111 1.
    {                                                                   //                                                              1.
        ticks = 0;                                                      //                                                              1.
                                                                        //                                                              1.
        // Top Door Timer                                                                                                               1.
        if( DOOR_TOP_STAT )                                             // TRUE/1 = OPENED                                              1.
        {                                                               //                                                              1.
            uiTopDoorTmr++;                                             // Increment because the door is open                           1.
                                     // disable                 enable                                                                  1.
            if( uiTopDoorTmr > 120 ) ucTopDoorHtrBLK = 1; else ucTopDoorHtrBLK = 0; // @ 120 sec turn heaters off                       1.
            if( uiTopDoorTmr > 240 ) uiTopDoorTmr    = 0;               // @ 240 secs RST timer, allow uiTopDoorTmr to kill heat again  1.
        }                                                               //                                                              1.
        else                                                            //                                                              1.
        {                                                               //                                                              1.
            uiTopDoorTmr    = 0;                                        // Reset time during door closed, so we start fresh at zero secs1.
            ucTopDoorHtrBLK = 0;                                        // re-enable heat-engine.  HF160829                             1.
        }                                                               //                                                              1.
                                                                        //                                                              1.
        // Bot Door Timer                                                                                                               1.
        if( DOOR_BOT_STAT )                                             // TRUE/1 = OPENED                                              1.
        {                                                               //                                                              1.
            uiBotDoorTmr++;                                             //                                                              1.
                                     // disable                 enable                                                                  1.
            if( uiBotDoorTmr > 120 ) ucBotDoorHtrBLK = 1; else ucBotDoorHtrBLK = 0;   // @ 120sec turn heaters off                      1.
            if( uiBotDoorTmr > 240 ) uiBotDoorTmr    = 0;               // while door open count up to max of 240 sec and repeat        1.
        }                                                               // @ 240 secs RST timer, allow uiTopDoorTmr to kill heat again  1.
        else                                                            //                                                              1.
        {                                                               //                                                              1.
            uiBotDoorTmr    = 0;                                        // Reset time during door closed, so we start fresh at zero secs1.
            ucBotDoorHtrBLK = 0;                                        // re-enable heat-engine.                                       1.
        }                                                               //                                                              1.

        // ================================ Calculate Error and final PWM settings ==================================================== 1.
        // Once every 8 seconds calculate error delta for measured temperature to be used to affect PWM ( 5sec didn't work well)        1.
        // Positive Delata values represent increasing temps. Negative Delta values represent decreasing temps                          1.
        // Top Oven Error rate compensation for both rate of change (Integral) and distance from set-temp (Proportional)                1.
        //                                                                                                                              1.
        // Enlarge time window to account for slow changes in measured temperature.                                                     1.
        if( ucSampleSecs++ > 7 )                                        //                                                              1.
        {                                                               // NOTE: Delta and Diff from top of TASKHeater.c                1.
            ucSampleSecs  = 0;                                          //                                                              1.
                                                                        //                                                              1.
        // ________________________________________                          _________________________________________________________  1.
        // ________________________________________ TOP TOP Heater calc comp _________________________________________________________  1.
            iDeltaTopRate = iTopMeasTempF - iLastTopTemp;               // e.g last = 2, current = 4, Delta -2 or increasing.           1.
                                                                        //     last = 4, current = 2, Delta +2 or increasing            1.
            // (-)(-)(-)(-)(-)(-)(-)(-)(-)(-)(-)(-)(-)(-)(-)(-)(-)(-)(-)(-)(-)(-)(-)(-)Temp dropping. Negative numbers; turn ON 100%    1.
            if( iDeltaTopRate < 0 || iTopTempDiff > 10 )                // if temp-diff is neg or going neg XF/Sec                      1.
            {                                                           // OR error > 10F                                               1.
                if( iTopMeasTempF < TopSetPoint + 1 ) iSetTopPWM = 100; // run at maximum power (IF below set-point)                    1.
            }                                                           //                                                              1.
            else // (+)(+)(+)(+)(+)(+)(+)(+)(+)(+)(+)(+)(+)(+)(+)(+)(+))(+)(+)(+)(+)(+)                                                 1.
            {                                                           //                                                              1.
                // moving toward set-point fast enough (+ delta value)? if not accrue more error to add to proportional PWM value       1.
                if( iDeltaTopRate > 0 )                                 // Check for positive delta change (empty oven) 1F or more      1.
                {                                                       //                                                              1.
                    iTopAccruErrr = 0;                                  // If recovering >= than 1F/5secs                               1.
                }                                                       //                                                              1.
                else                                                    // slower than 1F/5secs                                         1.
                {                                                       //                                                              1.
                    iTopAccruErrr += iTopTempDiff;                      // add proportional compensation value                          1.
                    if( iTopAccruErrr > 100 ) iTopAccruErrr = 100;      // Limit to max value for PWM%                                  1.
                }                                                       // iTopTempDiff is only positive value (see TASKHeater.c)       1.
                                                                        // 00 01 04 09 16 25 36 49 64 81 10*10 handled above            1.
                iSetTopPWM  = iTopTempDiff * iTopTempDiff;              //  0  1  2  3  4  5  6  7  8  9 10 non-linear rate             1.
                iSetTopPWM += iTopAccruErrr;                            //                                                              1.
                if( iSetTopPWM > 100 ) iSetTopPWM = 100;                //  clip maximum value pushed to PWM                            1.
            }                                                           //                                                              1.
                                                                        // Delta and Diff from top of TASKHeater.c                      1.
            //___________________________________                          ____________________________________________________________ 1.
            // __________________________________ BOT boT heater Calc comp ___________________________________________________________  1.
            iDeltaBotRate = iBotMeasTempF - iLastBotTemp;               // neg values indicated dropping measured delta temp (TIME 5S)  1.
                                                                        // pos values indicate rising measured delta temperatures       1.
            // (-)(-)(-)(-)(-)(-)(-)(-)(-)(-)(-)(-)(-)(-)(-)(-)(-)(-)(-)(-)(-)(-)(-)(-) Temp dropping. Negative numbers; turn ON 100%   1.
            if( iDeltaBotRate < 0 || iBotTempDiff > 10 )                // if temp-diff is neg or going neg XF/5sec                     1.
            {                                                           // OR error > 10F                                               1.
                if( iBotMeasTempF < BotSetPoint + 1 ) iSetBotPWM = 100; // run at maximum power (IF below set-point)                    1.
            }                                                           //                                                              1.
            else // (+)(+)(+)(+)(+)(+)(+)(+)(+)(+)(+)(+)(+)(+)(+)(+)(+))(+)(+))(+)(+)(+)                                                1.
            {                                                           //                                                              1.
                // moving toward set-point fast enough (+ delta value)? if not accrue more error to add to proportional PWM value       1.
                if( iDeltaBotRate > 0 )                                 // Check for positive temp swing (empty oven) 1F or more        1.
                {                                                       //                                                              1.
                    iBotAccruErrr = 0;                                  //                                                              1.
                }                                                       //                                                              1.
                else                                                    //                                                              1.
                {                                                       //                                                              1.
                    iBotAccruErrr += iBotTempDiff;                      //                                                              1.
                    if( iBotAccruErrr > 100 ) iBotAccruErrr = 100;      //                                                              1.
                }                                                       // iBotTempDiff is only positive value (see TASKHeater.c)       1.
                                                                        // 00 01 04 09 16 25 36 49 64 81 10*10 handled above            1.
                iSetBotPWM  = iBotTempDiff * iBotTempDiff;              //  0  1  2  3  4  5  6  7  8  9 10 non-linear rate             1.
                iSetBotPWM += iBotAccruErrr;                            //                                                              1.
                if( iSetBotPWM > 100 ) iSetBotPWM = 100;                // clip maximum value pushed to PWM                             1.
            }                                                           //                                                              1.

            // _________________________Capture current temp to use next pass as last (remembered) temp _______________________________ 1.
            iLastTopTemp  = iTopMeasTempF;                              // current_temp will be last_temp next_time                     1.
            iLastBotTemp  = iBotMeasTempF;                              // current_temp will be last_temp next_time                     1.
        }                                                               //                                                              1.
    }                                                                   //________________ end of 1sec__________________________________1.
                                                                        //                                                               .
     IFS0bits.T1IF = 0;                                                 // reset Timer 1 interrupt flag                                  .
}                                                                       //                                                               .

