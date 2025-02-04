/* --------------------------------------------------------------------------             *
 *  Module for Turbochef Oven Software - Phoenix Project                                  *
 *  TASKPhoenix_Interface.h                                                               *
 *                                                                                        *
 *  Header file for TASKPhoenix_Interface.c                                               *
 *                                                                                        *
 * **************************************************************************             *
 * FileName:        TASKPhoenix.h                                                         *
 * Dependencies:    None                                                                  *
 *																			              *
 * Processor:       dsPIC33FJ256GP710                                                     *
 * Compiler:        MPLAB C30                                                             *
 * Company:         Turbochef Technology Incorporated                                     *
 *                                                                                        *
 * Copyright ?2013 Turbochef Technology Inc.  All rights reserved.                       *
 *                                                                                        *
 * Date         Who       Comments/Notes                                                  *
 * 2013-08-17   JMWare    Initial File                                                    *
 *                                                                                        *
 * ---------------------------------------------------------------------------------------*/
extern int OperationMode;
extern unsigned char ucVoltSET;                // Check voltage setting on unit.
extern int DownLoadInProgress;
extern int iServiceMode;

void TASKInterfacePhoenix( void );
void TASKSerialPhoenix( void );
void TASKSerial2Modbus( void );