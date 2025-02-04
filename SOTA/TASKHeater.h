/* -------------------------------------------------------------------*
 *
 *     TASKHeater.h     External referances into  TASKHeater.c
 *
 * -------------------------------------------------------------------*/

/* * * Variables (externals)   */
extern int rtdRawTop;
extern int rtdRawBot;

extern int iTopMeasTempF;
extern int iBotMeasTempF;

extern int TopSetPoint;                         // Upper Cook Chamber Set Point, Deg F
extern int BotSetPoint;                         // Lower Cook Chamber Set Point, Deg F

extern int iStartModTop;                        // HF161208
extern int iStartModBot;

extern int iTopTempDiff;                        // 
extern int iBotTempDiff;                        // 

// experimental to see if we can use raw (3X more granular temp)
extern int  iTopRawDiff;                        //X
extern int  iBotRawDiff;                        //X

extern unsigned char ucPhase;


/* * *  Prototypes  */
void TASKHeaters( void );
