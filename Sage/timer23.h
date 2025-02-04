/* ------------------------------------------------------------------- *
 * FileName:        isr_timer23.h
 * Dependencies:    none
 * Processor:       dsPIC33F
 * Compiler:        MPLAB. C30 v2.01 or  higher
 * Constants
 *                  ***** IMPORTANT *****
 *   See start of timer23.c for details on what these defines actually
 *   are doing in preprocessor code!!!
 *	If you don't, change anything in this header file at you own risk!!!
 *             ***** You have been warned! ***
 * ------------------------------------------------------------------- */
#undef TIMER2PR        /* Force calulation of TIMER3PR from TIMER2TPS */
#define TIMER2TPS 8000 /* Timer2 speed in Ticks Per Sec (TPS) */

/* TIMER3PR is used for PWM (also see pwm.c), should be set to
     only values that are 2^n -2,  for example...
      0xFFFE (N=16), Ox7FFE (N=15), Ox3FFE (N=14), ... 0x001E (N=5) */

//#define TIMER3PR  0xFFFE; / * for 153Hz, for full PWM control * /
//#define TIMER3PR	0x0FFE	/ * for 9.77kHz, for 12bit PWM control * /
#define TIMER3PR	0x03FE	/* for 39kHz, for 10bit PWM control */

/* ---------------------------------------------------------------------------*
 * Calulate  PR2 register value, TIMER2                                       *
 *                                                                            *
 *                                                                            *
 *                          ***** IMPORTANT *****                             *
 *  If TIMER2PR is already defined, thats whats used in TIMER2's PR register. *
 *                                                                            *
 *  If TIMER2PR NOT already defined in timer23.h, is calulated from TIMER2TPS *
 *  in preprocessor code below. TIMER2PRESCALE and TIMER2SOURCE might need    *
 *  to be changed in the below code, to get it to work correctly.             *
 *                                                                            *
 *  The TIMER2PR calulated is the closest value within 0.5% of the speed in   *
 *  TIMER2TPS. If it can't, or if TIMER2PR value is impossable to do, errors  *
 *  will be generated during the build.                                       *
 *                                                                            */
#ifndef TIMER2PR  /* if PR for TIMER2 is not defined, make one now! */

  // This is set by TCKPS, can only be 1, 8, 64, or 256
  #define TIMER2PRESCALE (8)

  // This is set by TCS, can be values like 32768 if run SOSCO, or SYSCLK
  #define TIMER2SOURCE SYSCLK

  #define TIMER2PR ((TIMER2SOURCE/TIMER2TPS)/TIMER2PRESCALE)

  /* Check to see if PR value is small enough to actually fit into registar! */
  #if TIMER2PR > 65535
    #error Cannot set up Timer2 for the SYSCLK and TIMER2TPS. \
           Correct values in common.h and timer2.h files.
  #endif

  /*  Check for less that 0.5% error, and fail if too large.  */
  #define TIMER2PR_MISTAKE 1000*(TIMER2TPS-(TIMER2SOURCE/TIMER2PRESCALE/TIMER2PR))/TIMER2TPS

  #if (TIMER2PR_MISTAKE > 5)||(TIMER2PR_MISTAKE < -5)
    #error TIMER2 rate mistake is too big for the SYSCLK \
           and TIMER2TPS. Correct values in timer23.c file.
  #endif
#endif  /* #ifndef TIMER2PR  */

/* Prototypes */
void Init_Timer2( void );
void Init_Timer3( void );

unsigned short GetVoltageRMS();
void EnableRapidBlink (char action);
