/* ****************************************************************************
 *  Module for Turbochef Oven Software - Phoenix Project                      *
 *                                                                            *
 *  taskIO.h                                                                  *
 *                                                                            *
 *                                                                            *
 *                                                                            *
 * ************************************************************************** *
 * FileName:        fire.h        Common .h between FIRE and Phoenix          *
 * Dependencies:    None                                                      *
 *                                                                            *
 * Processor:       dsPIC33FJ256GP710                                         *
 * Compiler:        MPLAB C30                                                 *
 * Company:         Turbochef Technology Incorporated                         *
 *                                                                            *
 *  Ver 0.1                                                                   *
 *  History:                                                                  *
 *  --------                                                                  *
 *  Ver	   Who	    Description                                               *
 *  ---    ---      --------------------------------------------------------- *
 * 0.01    JMWare   Initial version                                           *
 * 0.02    HF091513 Modification to match taskIO.h SAGE to Phoenix            *
 *                  The 1st version was based on OvenLan.odt, which will have *
 *                  to be amended. This is 2nd Initial version after John     *
 *                  redesigned taskIO.h to be identical in both AND Phoenix.  *
 *                  This version has the API calls with headers documenting   *
 *                  how they are to be used.                                  *
 * 0.03    JMWare   Moved items that ONLY needed to be in FIRE and not in     *
 *                  Phoenix to to this FIRESerialProtocol.h (this file)       *
 *                  to support FIRE protocol.  2013-09-23                     *
 * 0.04    JMWare   Made union structures for protocol common to FIRE and to  *
 *                  Phoenix called fire.h.  This .h is now the main .h to     *
 *                  include when needing access to serial protocol unions.    *
 * 0.05    JMWare   Changed to using interface functions to access comm data  *
 *                                                                            *
 * ************************************************************************** *
 *                                                                            *
 * Copyright © 2013 Turbochef Technology Inc.  All rights reserved.           *
 * ************************************************************************** */
#ifndef _FIREPROTOCOL_EXTRA_H
#define _FIREPROTOCOL_EXTRA_H 
 
// Common file between FIRE and PHOENIX:
#include "fire.h"                           // Packet unions for protocol
 
#define PacketID U1_rxbuf[2]                // Define where PACKET ID is un the header!
#define RESET_TIME 500                      // Packet reset timeout!  rx'ing data, then no data resets state machine!

// JMWare:
// The following union(s) allow access
// to each byte of the Header, Command, and Status/Response packets - for easy transmission thru serial port
// >> Convert from direct access to indirect access thru interface functions (see below)
extern union CmdPacket   CmdPk;
extern union StatPacket  StatPk;
 
// *************************************************************************
// Define if checksum routine is computing a transmit packet or
// a receive packet.  The difference is how the final checksum is
// either included in the computation (as in from a receive packet) or
// is not included in the computations (as in from a transmit packet).
// *************************************************************************
typedef enum _CHECKSUM_METHOD
{
    ckSumTransmit = 1,
    ckSumReceive,
} CHECKSUM_METHOD;

// *************************************************************************
// Define timeout when waiting for the next command packet to arrive
// Counts down in the ms timer area - so 1000 = 1000ms = 1 Second
// Waiting for a command packet - if no packet, oven can
// shutdown heating system till command packet is received again
// *************************************************************************
#define SERIALTIMEOUT      5000

#endif  // _FIREPROTOCOL_EXTRA_H



