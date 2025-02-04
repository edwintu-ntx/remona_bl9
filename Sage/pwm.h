/* -------------------------------------------------------------------*
 *     pwm.h        Output Capture  controls                          *
 *                                                                    *
 * -------------------------------------------------------------------*/
/* * *  Defines  */

/* * * Variables ( externals) */

/* * *  Prototypes */

void pwm_init( void );
void pwm_1 ( unsigned int width );        // TOP AIR
void pwm_2 ( unsigned int width );        // BOT AIR
//void pwm_3 ( unsigned int width );        //
//void pwm_4 ( unsigned int width );        //
//void pwm_5 ( unsigned int width );        //
//void pwm_6 ( unsigned int width );        // GAS VALVE
//void pwm_7 ( unsigned int width );        // AUDIO
//void pwm_8 ( unsigned int width );        // HX Relay occupies as digital I/O

