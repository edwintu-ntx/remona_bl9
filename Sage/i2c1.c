/*------------------------------------------------------------------------------------*
 * I2C bus device list SAGE Rev 4                                                     *
 *  This page needs to be cleaned up and make sure all addresses match all devices-   *
 *-used on the SAGE2 board.                                                           *
 *-------i2c1.c-------I2C bus #1------------------------------------------------------*
 * part      type      #bit   subaddr  full_addr  TopSpeed                            *
 * ----    ----------   ---    -----    -------    ----                               *
 * U16     MCP23017      7       1       0x42      1.7MHz                             *
 * U20     MCP9802       7       5       0x9A      400kHz                             *
 * U21     DS1338        7    (fixed)    0xD0      400kHz                             *
 * U24     AT24C64       7       2       0xA4      400kHz                             *
 * U25     AT24C64       7       3       0xA6      400kHz                             *
 *                                               ----------                           *
 *                                               400kHz = Max Freq.                   *
 *-----------------------------I2C buss 2---------------------------------------------*
 *                                                                                    *
 * part            type      #bit   subaddr  full_addr  TopSpeed                      *
 * ----          ----------   ---    -----    -------    ----                         *
 * Smartcard     AT24C512      7       0       0xA0      1MHz                         *
 *------------------------------------------------------------------------------------*
 * I2C bus device list SAGE Rev X5                                                    *
 * I2C buss 1                                                                         *
 * part      type      #bit   subaddr  full_addr  TopSpeed                            *
 * ----    ----------   ---    -----    -------    ----                               *
 * U15     MCP23017      7       1       0x42      1.7MHz                             *
 * U22     MCP9802       7       5       0x9A      400kHz                             *
 * U23     DS1337        7    (fixed)    0xD0      400kHz                             *
 * U19 === AT24C512 ==== 7 ===== 4 ===== 0xA8 ==== 400kHz =======*                    *
 *         (2nd bank)    7      (0)     (0xA0)                   |                    *
 * U20 === AT24C512 ==== 7 ===== 6 ===== 0xAC ==== 400kHz =======*=====               *
 *         (2nd bank)    7      (4)     (0xA4)                   |                    *
 * U21 === AT24C512 ==== 7 ===== 7 ===== 0xAE ==== 400kHz =======*                    *
 *         (2nd Bank)    7      (3)     (0xA6)                                        *
 *                                                400kHz = Max Freq.                  *
 *------------------------------------------------------------------------------------*
 *  Smartcard      0xA0                                                               *
 *  1st EEPROM     0xA8                                                               *
 *  2nd            0xAC                                                               *
 *  3rd            0xAE                                                               *
 *  EC therm       0x9A                                                               *
 *  RTC            0xD0                                                               *
 * ---------------------------------------------------------------------------------- */
#include "p33FJ256GP710.h"
#include "common.h"
#include "i2c1.h"


/* ----------------------------- i2c1_init( ) -------------------------------------- *
 *   initate I2C port #1, speed = 400 for 400Khz clock =  all others, 100Khz clock   *
 * --------------------------------------------------------------------------------- */
void i2c1_init( void )
{                                               //
    I2C1CONbits.I2CEN   = 0;                    // FORCE I2C port 1 modlule Off before changing settings!!!
    IEC1bits.SI2C1IE    = 0;                    // Disable SI2C Interrupt
    IEC1bits.MI2C1IE    = 0;                    // Disable MI2C Interrupt
    TRISGbits.TRISG3    = 1;                    // SDA1 pin to Input
    TRISFbits.TRISF6    = 1;                    // SCK1 pin to Input
    I2C1CONbits.I2CSIDL = 0;                    // Continue module even in Idle mode
    I2C1CONbits.IPMIEN  = 0;                    // IPMI mode off
    I2C1CONbits.A10M    = 0;                    // 7 bit address mode
    I2C1CONbits.DISSLW  = 1;                    // Slew rate control, disable
    I2C1CONbits.SMEN    = 1;                    // SMB levels enable
                                                //
    //I2C1BRG             = I2CBRG_400KHZ;        // 400kHz
    I2C1BRG             = I2CBRG_100KHZ;        // 400kHz
    I2C1CONbits.DISSLW  = 0;                    // Slew rate control, enable
                                                //
    IFS1bits.SI2C1IF  = 0;                      // clear MI2C & SI2C Interrupts
    IFS1bits.MI2C1IF  = 0;                      //
    IPC4bits.SI2C1IP  = 7;                      // SI2C priority
    IPC4bits.MI2C1IP  = 4;                      // MI2C priority
    IEC1bits.SI2C1IE  = 0;                      // Disable SI2C Interrupt
    IEC1bits.MI2C1IE  = 1;                      // Enable MI2C Interrupt
    I2C1CONbits.I2CEN = 1;                      // Enable I2C port 1
}


/*  Interrupt's aren't used in this version, but are captured here for future use */

/* ================== i2c1_isr() =========================== *
 *      Handle I2C port #1 interrupt  IRQ handler            *
 * ========================================================= */
void __attribute__((__interrupt__, no_auto_psv)) _MI2C1Interrupt( void )
{
    IFS1bits.MI2C1IF = 0;                   // Clear IRQ flag
}


/* ======== i2c1_isr() Slave interrupt (abnormal) ========== *
 * Handle I2C port #1 slave interrupt Abnormal this design   *
 * ========================================================= */
void __attribute__((__interrupt__, no_auto_psv)) _SI2C1Interrupt( void )
{
    IFS1bits.SI2C1IF = 0;                   // Clear IRQ flag
    while( 1 );                             // There should be no Slave ints - Trap It!
}


/* +++++++++++++  Low Level I2C Bus #1 Routines  ++++++++++++++++ */

/* ====================== i2c1_busy() ============================ *
 * TRUE if I2C bus is busy, otherwise returns 0; (End of TX,) or   *
 * (end of Ack, end of Rec, end Stop, end of Repeted Start, or     *
 * end of Start)                                                   *
 * =============================================================== */
int i2c1_busy( void )
{
    return ( ( I2C1STAT & 0x4000 ) | ( I2C1CON & 0x001F ) ) ;
}


/* ====================== i2c1_idlewait() ======================== *
 *  Spin wheels until I2C bus #1 is not busy any more or timeout   *
 *  very TEMPORTY fix, will be made obsolete by ISR based control  *
 *
 * RETURNS 0 = timed out waiting for idle (= FALSE)
 *  all else = bus idle.
 * =============================================================== */
void i2c1_idlewait( void )
{                                           //
  unsigned int busycount;                   //
                                            //
  busycount = 60000;                        // Set busycount to some rediculous amount

  // while I2C buss busy, && counter hasn't timed out
  while( i2c1_busy() && busycount )       //
  {                                       //
     busycount--;                         //
     Nop(); Nop(); Nop(); Nop();          //
  }                                       //
}                                         //

/* ===================== i2c1_start() ============================ *
 *      Send start sequence out I2C bus #1                         *
 * =============================================================== */
void i2c1_start()
{                                           //
    i2c1_idlewait();                        //
    I2C1CONbits.SEN = 1;                    // Send a Start;
}                                           //

/*==================== i2c1_restart() ============================ *
 *      Send re-start sequence out I2C bus #1                      *
 * =============================================================== */
void i2c1_restart()
{                                           //
    i2c1_idlewait();                        //
    I2C1CONbits.RSEN = 1;                   // Send a Start;
}                                           //

/* ===================== i2c1_stop() ============================== *
 *      Send start sequence out I2C bus #1                          *
 * ================================================================ */
void i2c1_stop()
{                                           //
  i2c1_idlewait();                          //
  I2C1CONbits.PEN = 1;                      // Send a Stop;
}                                           //

/* ====================== i2c1_write( byte ) ====================== *
 *      Send a byte out I2C bus #1                                  *
 *                                                                  *
 * RETURNS FALSE = NACK was received (Bad Write)                    *
 *         TRUE  = ACH was received (GOOD Write)                    *
 * ================================================================ */
int i2c1_write( unsigned char b )
{                                           //
  i2c1_idlewait();                          //
  I2C1TRN = b;                              //
  i2c1_idlewait();                          //
  return( !I2C1STATbits.ACKSTAT );          //
}                                           //

/* ==================== i2c1_read( ack ) ========================== *
 *      Receieve a byte in from I2C bus #1                          *
 *      if "act" is true, respond with ACK after transfer           *
 * ================================================================ */
int i2c1_read( int ack )
{
  int recbyte;                              //
                                            //
  i2c1_idlewait();                          //
  I2C1CONbits.RCEN  = 1;                    // Recieve Mode Enable
  i2c1_idlewait();                          //
  recbyte           = I2C1RCV;              // Get recieved byte
                                            //
  if( ack )                                 //
      I2C1CONbits.ACKDT = 0;                // Prepare to send an ACK
  else                                      //
      I2C1CONbits.ACKDT = 1;                // Prepare to send a NACK
                                            //
  I2C1CONbits.ACKEN = 1;                    // Start sending ACK or NACK
  return( recbyte );                        //
}                                           //

/* ====================  ReadIOPort(  ) =========================== *
 *     Read a byte from Low Speed I/O chip, TC74, and RTC.          *
 *     Address = I2C address                                        *
 *     Command = buffer address in extrnal chip                     *
 *  Example ReadIOPort(0x44, 0x14, &ucInByte);                      *
 * ================================================================ */
void ReadIOPort( unsigned char Address, unsigned char Command, unsigned char* readbyte )
{
	int	ack;            // Storage of ACK or NACK was recieved;

    i2c1_start();                            // Generate START condition
    ack =  i2c1_write( Address & 0xFE );     // Transmit ADDRESS with WRITE command
    ack &= i2c1_write( Command );            // Transmit COMMAND byte
	if( ack )
	{
    	i2c1_restart();                          // Generate a REPEATED-START condition
    	ack &= i2c1_write( Address | 0x01 );     // Transmit ADDRESS with READ command
    	*readbyte = i2c1_read( 0 );              // Receive second DATA byte (MSB) and don't acknowledge
	}
    i2c1_stop();                             // Generate a STOP condition
}

/* ===================== WriteIOPort( ) =========================== *
 *  Writes two bytes into Low Speed I/O chip.                       *
 *  Address = I2C address        Command = buffer address in MC23017*
 *  Example WriteIOPorts(0x42, 0x14, ucOutput0, ucOutput1);         *
 *  Example WriteIOPorts(0x44, 0x14, ucOutput2, ucInput0);          *
 * ================================================================ */
void WriteIOPort( unsigned char address, unsigned char cmd,unsigned char data )
{
    i2c1_start();                          	// Generate a START condition

    i2c1_write( address & 0xFE );     		// Transmit "ADDRESS and WRITE" byte

 	i2c1_write( cmd );            			// Transmit COMMAND byte

	i2c1_write( data );      				// Transmit first DATA byte

    i2c1_stop(); 		   					// Close the bus
}


int i2c1_EEWriteArray( unsigned char id, unsigned int addr, unsigned char* array, unsigned int size )
{
  unsigned int index; 
  unsigned int addr2;                                            // Index in array
  unsigned int i,j;
  unsigned int istart;
  unsigned char pageindex;

  unsigned char c;

  if( 0 == size || 0 == id || array == 0 )                      // Is there is a reason not to write?
  {
     return( FALSE );                                           // YES, then don't write!
  }

  pageindex = 0;
  istart = 0;
  addr2 = addr;

  i2c1_redo:;

  i2c1_start();                                                 // Start a I2C transation

  if( i2c1_write( id & 0xFE ) )                                 // Transmit I2C chip ID with WRITE mode
  {
    if( i2c1_write( (unsigned char)( addr >> 8 ) ) )            // Send the upper byte of addr
    {
      if( i2c1_write( (unsigned char)( addr & 0xFF ) ) )        // Send the lower byte of addr
      {
        for( index = istart; index < size; index++ )
        {
          if(pageindex != 0)
          {
            if((addr2 % 128) == 0)
            {
              i2c1_stop();
              //
              for(i=0; i<50; i++)
              {
                for(j=0; j<30000; j++);
              }
              // re-aline to continue in to next page!
              istart = index;
              addr = addr2;
              pageindex = 0;
              goto i2c1_redo;

            }
          } 
          c = array[0];
          if( ! i2c1_write( array[ index ] ) )                  // Write a byte, checking that we got an ACK
          {
            i2c1_stop();                                        // Failed sending a byte from array
            return( FALSE );                                    // Report FAILED!
          }
          addr2++; 
          pageindex++;
        }

        for(j=0; j<30000; j++);

        i2c1_stop();                                            // Close the bus and
        return( TRUE );                                         // Report successful write!
      }
    }
  }

  // Can only get here if we had a NACK - WE FAILED!
  i2c1_stop();                                                  // Close the bus and
  return( FALSE );                                              // Report failure
}

