/* ------------------------------------------------------------------- *
 *                                                                     *
 *     TASKBlower.h     External references for TASKBlower.c           *
 *                                                                     *
 * Measured results of blower speed vs input voltage from SAGE:        *
 * Reverified by Craig Towne 16/06/13.                                 *
 *                                                                     *
 *                                                                     *
 *  percent voltage   RPM   Amps                                       *
 *                                                                     *
 *  10%     0.985	  734   -.--                                       *
 *  20%     1.973	1,483   -.--                                       *
 *  30%     2.962	2,170   -.--                                       *
 *  40%     3.952	2,856                                              *
 *  50%     4.943	3,547                                              *
 *  60%     5.963	4,437                                              *
 *  70%     6.93	4,934                                              *
 *  80%     7.93	5,614                                              *
 *  85%     8.43    5,841            <--- MAX RPM, MAX V-OUT           *
 *  90%     8.92	5,871                                              *
 * 100%     -.--    -,---                                              *
 *                                                                     *
 * ------------------------------------------------------------------- */


 
 
 
 
  
/* * *  Defines */
// #define AIRSPD10L       62059L                     // Original 10V full scale value
// 62059 / 100 = 620.59  per 1-percent change       .. Per DC 160525 change full scale to be -
// 620.59 * 85 = 52750 (new max)                    ..-only 85% of current full scale (62059)
// 52750 / 100 = 527.5 per 1-percent change         ..
// so 52750/62059 = 0.849 (or 85%)                  ..
// #define AIRSPD10L       52750L                      // modified for David to 85% now equals 100%
#define AIRSPD10L       62059L                      // moved back to 62059L = 100% HF160606
#define MAXAIR          (100)                       /* Max Blower speed. 85% 90% 95% 100% = 85% HF160606 */
#define MINAIR          (10)                        /* Minumum Blower speed, in percent */

enum FanSTATE       {   FOFF    = 0,
                        FSTART  = 1,
                        FTEST   = 2,
                        FRUNING = 3
                    };

extern int iTOPairSpeed;                            // Top Blower speed set point, in percentage (100 - 0)
extern int iBOTairSpeed;                            // Bottom Blower speed set point, in percentage (100 - 0)


/* * *  Prototypes */
void SetTOPair( int speed );
void SetBOTair( int speed );
char isTopAirOn( void );
char isBotAirOn( void );
void TASKBlowers( void );
