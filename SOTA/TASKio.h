/* -------------------------------------------------------------------------------------------------------------------------- *
 *     TASKio.h     External referances into TASKio.c                                                                         *
 *         TABS ARE SET TO 4                                                                                                  *
 * -------------------------------------------------------------------------------------------------------------------------- */
//#define KEYCLICKLENGTH  (140)   /* HF12181906 was 40 Click lenngh in Ticks */
//#define BEEPLENGTH      (333)   /* Beep lenngh in Ticks */

/* -------------Defines for IO pins access----------------- */

/* HIGH SPEED Inputs */
#define RTC_PULSE       PORTCbits.RC14  /* not used - input */

#define HX2TOP      	PORTDbits.RD7   /* Top heater 2 control - output*/
#define BELT1_STAT      PORTDbits.RD10  /* not used - input */
#define BELT2_STAT      PORTDbits.RD11  /* not used - input */
#define BELT3_STAT      PORTDbits.RD12  /* not used - input */
#define HX_STAT         PORTDbits.RD14  /* not used - input */
#define AC_PHASE        PORTDbits.RD15  /* not used - input */

/* HIGH SPEED Outputs */
#define HX2BOT          PORTEbits.RE2   /* Bottom heater 2 control - output    */
#define HX1TOP          PORTEbits.RE3   /* Top heater 1 control - output    */
#define RACK_BOT        PORTEbits.RE4   /* RACK_BOT - output    */
#define HSI_VOLT_MODULE PORTDbits.RD9   /* HSI_VOLT_MODULE - input  */
#define HSIO_4_OUT      PORTEbits.RE7   /* Now used for Heater disarm relay HF160225 (was "not used") - output */

#define DOOR_BOT_STAT   PORTFbits.RF0   /* CDP_Status (K1 Relay) - input, HI = door open */
#define CDS_STAT        PORTFbits.RF1   /* not used */
#define ALIVE_LED       PORTFbits.RF8   /* ALIVE_LED - output        */

#define HX1BOT          PORTGbits.RG0   /* Bottom heater 1 control - output */
#define DOOR_TOP_STAT   PORTGbits.RG1   /* CDM_Status - input, HI = door open */

/*  LSIn Inputs     Defines of Low Speed IO bits, from inside LSIn, and LSOut union */
#define LSIO_1          LSIn.LSIO_1bit               // 7 not used
#define LSIN_1          LSIn.LSIN_1bit               // 6 not used
#define MAG_FAN_STAT    LSIn.MAG_FAN_STATbit         // 5 not used
#define MAGOT_STAT      LSIn.MAG_OT_STATbit          // 4 not used                                       .
#define VOLTSENCE_STAT  LSIn.VOLTSENCE_STATbit       // 3 
#define PRODUCT_STAT    LSIn.PRODUCT_STATbit         // 2 not used
#define BLOWER_BOT_STAT LSIn.BM2_STATbit             // 1 
#define BLOWER_TOP_STAT LSIn.BM1_STATbit             // 0 

/* LSOut Outputs                                                                                 */
#define BM1_OUT         LSOut.BM1_OUTbit             // 0 
#define BM2_OUT         LSOut.BM2_OUTbit             // 1 
#define VOLTSEL         LSOut.VOLTSEL_ENbit          // 2                                        .
#define LIGHT_BOT       LSOut.LIGHT_BOT_CNT          // 3                                        .
#define MAG1_EN         LSOut.MAG1_ENbit             // 4 not used 
#define MAG2_EN         LSOut.MAG2_ENbit             // 5 not used
#define LIGHT_TOP       LSOut.LIGHT_TOP_CNT          // 6                                        .
#define RACK_TOP        LSOut.RACK_TOP_CNT           // 7                                        .

/* Union to remap Low Speed IO bits, on output side */
extern volatile union     LSOutPortUnion
{
    unsigned char Port;

    struct
    {
        unsigned BM1_OUTbit:    1;   // GPA0 pin-21 U15 MCP23017
        unsigned BM2_OUTbit:    1;   // GPA1 pin-22
        unsigned VOLTSEL_ENbit: 1;   // GPA2 pin-23
        unsigned LIGHT_BOT_CNT: 1;   // GPA3 pin-24
        unsigned MAG1_ENbit:    1;   // GPA4 pin-25
        unsigned MAG2_ENbit:    1;   // GPA5 pin-26
        unsigned LIGHT_TOP_CNT: 1;   // GPA6 pin-27
        unsigned RACK_TOP_CNT:  1;   // GPA7 pin-28

    };
} LSOut;

/* Union to remap Low Speed IO bits, on input side */
extern volatile union     LSInPortUnion
{
    unsigned char Port;

    struct
    {
        unsigned BM1_STATbit:      1;    // GPB0 pin-1 U15 MCP23017
        unsigned BM2_STATbit:      1;    // GPB1 pin-2
        unsigned PRODUCT_STATbit:  1;    // GPB2 pin-3
        unsigned VOLTSENCE_STATbit:1;    // GPB3 pin-4
        unsigned MAG_OT_STATbit:   1;    // GPB4 pin-5
        unsigned MAG_FAN_STATbit:  1;    // GPB5 pin-6
        unsigned LSIN_1bit:        1;    // GPB6 pin-7
        unsigned LSIO_1bit:        1;    // GPB7 pin-8
    };
} LSIn;

void InitLSIO( void );
