/* --------------------------------*          Truely GLOBAL items            *------------------------------------ *
 *                                                                                                                 *
 * HF100809 rc9 fixed volt select problem brought on by fixing the little flash of volt select.                    *
 *                                                                                                                 *
 * Cooking range is 300F to 540F                                                                                   *
 * Soak threshold is 200 (to allow 300F)                                                                           *
 * Cooking time limited to 10 minutes.      1.00.01rc6 HF030110                                                    *
 * Release 01.00.01                         Per David it's ready for release.  HF031610                            *
 *                                                                                                                 *
 * Release Candidate 01.00.02rc2            Intermediate step to help disipate heat into cavity-                   *
 *                                         -wall by speeding up air in cavity.                                     *
 *                                                                                                                 *
 * Release  01.00.02     Fixed Temp select so that you can change temps and then select either.                    *
 *                        HF050210                                                                                 *
 *                                                                                                                 *
 *                                                                                                                 *
 * Release Candidate 01.00.03rc1            Intermediate step to help disipate heat into cavity-                   *
 *                                         -wall by speeding up air in cavity.                                     *
                                                                                                                   *
   Sota 01.00.03rc1        HF052610                                                                                *
                                                                                                                   *
   Fix Temp select for Group page.                                                                                 *
                                                                                                                   *
                                                                                                                   *
   --------------------------------------------------------------------------------                                *
   SOTA 1.00.03rc2    MJP 2010 07/26                                                                               *
   --------------------------------------------------------------------------------                                *
                                                                                                                   *
   Changes from 1.00.03rc1                                                                                         *
                                                                                                                   *
    Added F9 date time stamps to ethernet opf.txt file                                                             *
    Fixed F9 date time stamp view on screen, if up or down buttons were pressed.                                   *
  --------------------------------------------------------------------------------                                 *
 *                                                                                                                 *
 *                                                                                                                 *
 * Sota 01.00.03     MJP 2010 12/22       RTM per verbal request from David Castillo.                              *
 *                                                                                                                 *
                                                                                                                   *
  >>>>>> Branch: New version information below:                                                                    *
  -----------------                                                                                                *
  Branch:                                                                                                          *
    8/17/2013                                                                                                      *
  -----------------                                                                                                *
  FIRE - Part of Phoenix project                                                                                   *
  ----                                                                                                             *
  10/22/2013        1.00.10!              Development Version - First version of FIRE with 485                     *
                                          enabled.                                                                 *
  11/17/2013        1.00.22!              Handle event if door opens - setback blower speed                        *
  12/??/2013        1.00.23!              Compressed FIRE to fit on one card                                       *
  12/10/2013        1.00.24!              Serial protocol change to allow for multi-packet types                   *
  01/01/2014        1.00.25!              Mutli-Packet ability & MAG1&2 status output (see FIRE.h)                 *
  ----------        1.00.26!              Boot Loader development (still in development)                           *
  04/02/2014        1.00.27!              Adding Version packet and gracefull shutdown mode                        *
  04/30/2014        1.00.30!              Skipping 28 & 29.  Working boot-loader code - rdy for testing            *
  05/08/2014        1.00.31               Added RESET packet+Harolds three mods in Phoenix_interface.              *
  05/15/2014        1.01.00               Changed gracefull shutdown to clear incoming packet data too.            *
  06/03/2014        1.01.01               Corrected number of bytes being sent from the version send routine       *
  06/04/2014        1.01.02               Corrected minor header checksum.  Corrected major packet checksum.       *
                                          Added read and display version string                                    *
  06/05/2014        1.01.03               Correctly sending version packet without extra bytes at end of packet.   *
  HF140822          1.00.00.P             CA30 Blower control and F10 LED combo R                                  *
  HF140822P        01.00.00.S             CA32                                                                     *
  HF141004Pz       01.01.00.M             Refelect blower stat bits with heater stat bits.                         *
  HF141006Pz       01.01.00.N             Added back missed functionality after purge of bad blower version.       *
  HF141007P        01.01.00.P             Cleaned out all unneeded code fro SAGE-FIRE                              *
  HF160223         xx.xx.xx.?             working in heater control and power sharing for HHDual                   *
 *                                                                                                                 *
 * HF160830        160830                 Dusty's Modulation values No end of cook comp, and door timer rework     *
 *                                                                                                                 *
 *                                                                                                                 *
 *                                                                                                                 *
 *                                                                                                                 *
 *                                                                                                                 *
 * --------------------------------------------------------------------------------------------------------------- */
/* System frequency (Fcy) in Hz,  as setup by SFR's PLLFBD & CLKDIV in main.c */
#define SYSCLK      (40000000)          //

#define ON          0x01                //
#define OFF         0x00                //

#define TRUE		1
#define FALSE		0

#define TASKBLOWERSLEEP   	250         //	half a second delay
#define TASKHEATERSLEEP     250         // now 2X / sec as original 3000 / 6 = 500; 1/2 sec = 250  3000 // 6 sec dly
#define TASKINTERPHSLEEP    50          //	100ms delay  (have to trust it's close...
#define TASKSERIALPHSLEEP   3	        //	6ms delay
#define BLFSWRITEDELAY	   	1000	    //	2 second delay
#define RESETDELAY	  		3000        //	6 second delay

extern int iTasksActivated;
extern int iInitHardware;

extern int TASKBlowersSleep;
extern int TASKHeatersSleep;
extern int TASKInterPhoenixSleep;
extern int TASKSerialPhoenixSleep;

extern int BLFSwriteDelay;
extern int ResetDelay;

/* Defines */
