/* ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||| *
 *  TABS ARE SET TO 4                                                                                                        *
 * FileName:        isr_timer1.h                                                                                             *
 * Dependencies:    none                                                                                                     *
 * Processor:       dsPIC33F                                                                                                 *
 * Compiler:        MPLAB. C30 v2.01 or  higher                                                                              *
 *                                                                                                                           *
 *       Common variables used in isr_ADC.c                                                                                  *
 * HF020307 forces correct order. moved in from timer1.c                                                                     *
 * ------------------------------------------------------------------------------------------------------------------------  */
/* Constants */
#define TIMER1PRESCALE	    (64)        /* Calulate PR1 register value. This is set by TCKPS */
#define TIMER1SOURCE	    SYSCLK      // set by TCS, can be values like 32768 if run SOSCO, or SYSCLK
#define TICKSPERSEC		    (1000)      // 1000 Hz (1 mS) system tick
#define TIMER1PR		    ((TIMER1SOURCE/TIMER1PRESCALE)/TICKSPERSEC)

// HF160830D validated below settings for 3-Phase 
#define MOD1P208            14          // 1-phase 50~ Amps
#define MOD1P240            24          // 1-phase 50~ Amps

#define MOD3P208            35          // 3-phase 31~ Amps
#define MOD3P240            39          // 3-phase 31~ Amps

#define AMPAIRTHRESH        50

/* Check to see if PR value is small enough to actually fit into register! */
#if TIMER1PR > 65535
  #error Cannot set up Timer1 for the SYSCLK and TICKSPERSEC.\
   Correct values in common.h and timer1.h files.
#endif

//_____________________________________
enum ePWRshareV                         //
{                                       //
  VUNKNOWN	    = 0,                    //
  VUS208        = 1,                    //
  VUS240        = 2,                    //
  TOTALNUMVOLT                          //
};                                      //

/*  Variables (externals)   */

extern volatile unsigned int   iDoorTimerTop;            // exposing to restore function HF160818
extern volatile unsigned int   iDoorTimerBot;            // '' ''
extern volatile unsigned char  cPWMpwrBase;              // global used to set 208/240 from CmdPk.VOLTSel  HF160514

extern volatile unsigned short usTopCompOffTime;         // Calculated point on cook time-line (seconds) to shutdown heater
extern volatile unsigned short usBotCompOffTime;         //  make visible to TASKInterfacePhoenix and timer1

extern volatile unsigned char ucTopEndCookBLK;           // heaters ENABLED until cook seconds [ 9,8,7,6,5,4,3,2,1 ] end of cook secs
extern volatile unsigned char ucBotEndCookBLK;           // heaters ENABLED until cook seconds [ 9,8,7,6,5,4,3,2,1 ] end of cook secs


extern unsigned char ucPhase;

/* Prototypes TFT*/
void Init_Timer1( void );
void SetTopHeaterPWM( int val );
void SetBotHeaterPWM( int val );
