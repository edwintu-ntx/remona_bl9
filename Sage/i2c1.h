/* --------------------------------------------------------------------*
 *  I2C bus device list                                                *
 *  SAGE Rev 4                                                         *
 *                                                                     *
 *  I2C buss 1                                                         *
 *                                                                     *
 *  part      type      #bit   subaddr  full_addr  TopSpeed            *
 *  ----    ----------   ---    -----    -------    ----               *
 *  U16     MCP23017      7       1       0x42      1.7MHz             *
 *  U20     MCP9802       7       5       0x9A      100kHz             *
 *  U21     DS1338        7    (fixed)    0xD0      400kHz             *
 *  U24     AT24C64       7       2       0xA4      400kHz             *
 *  U25     AT24C64       7       3       0xA6      400kHz             *
 *                                                ----------           *
 *                                                  100kHz = Max Freq. *
 *                                                                     *
 *                                                                     *
 *     i2c1.h       I2C bus #1                                         *
 * --------------------------------------------------------------------*/
/* * *  Defines   */
/*      Module Constants    */
#define I2CBRG_100KHZ   362
#define I2CBRG_400KHZ    62

	/* I2C address's on bus */
#define EECB1       0xA8             //  EEPROM (C)ontrol (B)oard - 1
#define EECB2       0xAC             //  EEPROM (C)ontrol (B)oard - 2
#define EECB3       0xAE             //  EEPROM (C)ontrol (B)oard - 3
	//  I2C Part addresses
#define I2C1RTC     0xD0             //  DS1307 RTC
#define I2CTEMP     0x9A             //  TC-74


#if 0
	/***  RTC Registars  ***/
// --------------- Real Time Clock Defines -----------------------
#define RTC_REG_SECOND    0x00       // RTC Secnds Reg Offset
#define RTC_REG_MINUTE    0x01       // RTC Minutes Reg Offset
#define RTC_REG_HOUR      0x02       // RTC Hours Reg Offset
#define RTC_REG_DOW       0x03       // RTC Day of Week Reg Offset
#define RTC_REG_DATE      0x04       // RTC Day of Month Reg Offset
#define RTC_REG_MONTH     0x05       // RTC Month Reg Offset
#define RTC_REG_YEAR      0x06       // RTC Year Reg Offset
#define RTC_REG_CONTROL   0x07       // RTC Control Reg Offset

#define RTC_CH_BIT        0x80       // RTC Secnds Reg Oscillator Enable
#define RTC_SECOND_10     0x70       // RTC Secnds Reg Most Sgnfcnt Digit
#define RTC_SECOND        0x0f       // RTC Secnds Reg Least Sgnfcnt Digit
#define RTC_MINUTE_10     0x70       // RTC Minutes Reg Most Sgnfcnt Digit
#define RTC_MINUTE        0x0f       // RTC Minutes Reg Least Sgnfcnt Digit
#define RTC_HOUR_12       0x40       // RTC Hours Reg 12/24 mode
#define RTC_HOUR_AM_PM    0x20       // RTC Hours Reg AM/PM if 12 hr mode, else MSBit of Hrs
#define RTC_HOUR_12_10    0x10       // RTC Hours Reg Most Sgnfcnt Digit (12Hour)
#define RTC_HOUR_24_10    0x30       // RTC Hours Reg Most Sgnfcnt Digit (24Hour)
#define RTC_HOUR          0x0f       // RTC Hours Reg Least Sgnfcnt Digit
#define RTC_DAY_OF_WEEK   0x07       // RTC Day of Week Reg 0-7
#define RTC_DATE_10       0x30       // RTC Day of Month Reg Most Sgnfcnt Digit
#define RTC_DATE          0x0f       // RTC Day of Month Reg Least Sgnfcnt Digit
#define RTC_MONTH_10      0x10       // RTC Month Reg Most Sgnfcnt Digit
#define RTC_MONTH         0x0f       // RTC Month Reg Least Sgnfcnt Digit
#define RTC_YEAR_10       0xf0       // RTC Year Reg Most Sgnfcnt Digit
#define RTC_YEAR          0x0f       // RTC Year Reg Least Sgnfcnt Digit
#define RTC_CONTROL_OUT   0x80       // RTC Control Reg SquareWave Output (High/Low)
#define RTC_CONTROL_SQWV  0x10       // RTC Control Reg Oscillator Output Enable
#define RTC_CONTROL_RS1   0x02       // RTC Control Reg Freq Rate Select
#define RTC_CONTROL_RS0   0x01       // RTC Control Reg Freq Rate Select
#define RTC_CONTROL_FS    0x03       // RTC Control Reg Freq Select(Combine previous 2 bits)
#endif


/* * * Variables (externals) */
//extern const unsigned char EEADDR[4]; 	// EEPROM ADDRESSES, index 1 = EEPROM #1... 0 for no EEPROM

void i2c1_init( void );

void ReadIOPort(unsigned char, unsigned char, unsigned char* );
void WriteIOPort(unsigned char, unsigned char ,unsigned char );
int i2c1_EEWriteArray( unsigned char id, unsigned int addr, unsigned char* array, unsigned int size );
