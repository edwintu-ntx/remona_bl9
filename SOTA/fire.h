/* ****************************************************************************
 *  Module for Turbochef Oven Software - Phoenix Project                      *
 *                                                                            *
 * ************************************************************************** *
 * FileName:        fire.h                                                    *
 * Dependencies:    GenericTypeDefs.h                                         *
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
 * ************************************************************************** *
 *                                                                            *
 * https://www.allacronyms.com/CTR/Counter                                    *
 *                                                                            *
 * Copyright ?2013 Turbochef Technology Inc.  All rights reserved.           *
 * ************************************************************************** */
#ifndef _FIREPROTOCOL_H
#define _FIREPROTOCOL_H

// **************************************************************************** *
// Define number of bytes for each structure                                    *
// Need to define since we have a array inside a union - compiler does not      *
// allow using flexable arrays in unions.                                       *
// **************************************************************************** *
#define NumHeaderBytes     5               // Header
#define NumCommandBytes   18               // Command Packet = (5 + 18 = 23) 0-22  (18 - NEW) (16 - Working)
#define NumStatusBytes    27               // Status Packet  = (5 + 27 = 32) 0-31  (27 - NEW) (25 - Working)

//etu added
#define Num2HeaderBytes     5               // Header
#define Num2CommandBytes   18               // Command Packet = (5 + 18 = 23) 0-22  (18 - NEW) (16 - Working)
#define Num2StatusBytes    27               // Status Packet  = (5 + 27 = 32) 0-31  (27 - NEW) (25 - Working)

// **************************************************************************** *
//  Invalid Temperature (for SetPoint or to return from a bad sensor reading)   *
//  MJP: Added                                                                  *
//  JMW: Removed in FIRE - already define in TASKHeater.h                       *
//       Left in Phoenix - Does this belong in this .h file ????                *
//       Does it need to be signed?????                                         *
// **************************************************************************** *
// input from TC team oh how to handle this.
#define BADTEMP           (-9999)

// **************************************************************************** *
// Enum for packet types.                                                       *
// commandPacket - Packet sent from Phoenix to FIRE, giving FIRE data/commands  *
//                 to be used to control the oven.                              *
// statusPacket  - Packet sent from FIRE to Phoenix, sending back data from     *
//                 FIRE's ports or data from oven systems (blowers, ...)        *
// NOTE: Other packet types can be defined here.                                *
// Ver:0.00.22 - Swap status and command packet types for Mark                  *
//                                                                              *
// Per Harold: Command Packets are ODD numbered; Resp/Stat are even numbered    *
// **************************************************************************** *
typedef enum _PACKET_TYPES
{
    // From Phoenix to Sage
    commandPacket  = 1,        // Direct Command data to the Fire
    requestVersion = 13,       // Phoenix Requests SAGE to send it's version packet!

    // From Sage to Phoenix
    statusPacket   = 0,        // Return data to Phoenix from Fire
    versionPacket  = 12        // Sending the version of the SAGE firmware
} PACKET_TYPES;

// **************************************************************************** *
// COMMAND PACKET (From Phoenix to Oven)                                        *
// **************************************************************************** *
union CmdPacket
{                                           //
    struct                                  //
    {                                       //
        unsigned short Start;               // (1,2)    Start signature (Defined as 0xAA 0x55)
        unsigned char  PacketType;          // (3)      Packet type
        unsigned char  DataSize;            // (4)      Size of packet
        unsigned char  Checksum;            // (5)      Simple Checksum of HEADER only
        unsigned char  H_Spacer;            // (6)      Spacer for header to align on 16-bit boundary
        unsigned int   TopHeatEnabled:  1;  // (7) 1    Top Heat engine enabled
        unsigned int   BotHeatEnabled:  1;  // (7) 2    Bottom Heat engine enabled
        unsigned int   TopRack:         1;  // (7) 3
        unsigned int   BotRack:         1;  // (7) 4
        unsigned int   TopLight:        1;  // (7) 5
        unsigned int   BotLight:        1;  // (7) 6
        unsigned int   ALIVE_Led:       1;  // (7) 7
        unsigned int   VOLTSel:         1;  // (7) 8
        unsigned int   ServiceMode:     1;  // (8) 1
        unsigned int   TopHeater1:      1;  // (8) 2
        unsigned int   TopHeater2:      1;  // (8) 3
        unsigned int   BotHeater1:      1;  // (8) 4
        unsigned int   BotHeater2:      1;  // (8) 5
        unsigned int   bSingleMulti:    1;  // (8) 6
        unsigned int   nu11:            1;  // (8) 7
        unsigned int   nu12:            1;  // (8) 8
        unsigned char  nu2;                 // (9)
        unsigned char  BinXfer;             // (10)     do not change the location of this byte
        unsigned char  ucOvenType;          // (11)     Added to allow single/multi-phase selection HF160822
        unsigned char  TopAirSetpoint;      // (12)     Top Air Setpoint % (0-100)
        unsigned char  BotAirSetpoint;      // (13)     Bottom Air Setpoint %(0-100)
        unsigned short TopSetpoint;         // (14,15)  Top Heater Setpoint
        unsigned short BotSetpoint;         // (16,17)  Bottom Heater Setpoint
        unsigned short usTopCookCTR;        // (18,19)   https://www.allacronyms.com/CTR/Counter
        unsigned short usBotCookCTR;        // (20,21)
        unsigned short FChecksum;           // (22,23)
    } __attribute__ ((packed));

    // Command Packet Array
    unsigned char   byte[ NumHeaderBytes + NumCommandBytes ];
};

// ******************************************************************************* *
// STATUS PACKET (From Oven to Phoenix)                                            *
// ******************************************************************************* *
union StatPacket
{                                           //
    struct                                  //
    {                                       //
        unsigned short Start;               // (1,2)   Start signature (Defined as 0xAA 0x55)
        unsigned char  PacketType;          // (3)     Packet types, defined above
        unsigned char  DataSize;            // (4)     Size of data packet (CMD OR RESP) includes Data cksum byte
        unsigned char  Checksum;            // (5)     Checksum of HEADER only
        unsigned char  H_Filler;            // (6)     "H"eader Filler: Save 1 byte for WW2 version 2.0
        unsigned char  nu1;                 // (7)
        unsigned int   TopDoorStat:     1;  // (8) 1
        unsigned int   BotDoorStat:     1;  // (8) 2
        unsigned int   TopHeater1Stat:  1;  // (8) 3
        unsigned int   TopHeater2Stat:  1;  // (8) 4
        unsigned int   BotHeater1Stat:  1;  // (8) 5
        unsigned int   BotHeater2Stat:  1;  // (8) 6
        unsigned int   TopBlowerStat:   1;  // (8) 7
        unsigned int   BotBlowerStat:   1;  // (8) 8
        unsigned int   VOLTSENCE_Stat:  1;  // (9) 1
        unsigned int   nu2:             1;  // (9) 2
        unsigned int   nu3:             1;  // (9) 3
        unsigned int   nu4:             1;  // (9) 4
        unsigned int   nu5:             1;  // (9) 5
        unsigned int   nu6:             1;  // (9) 6
        unsigned int   nu7:             1;  // (9) 7
        unsigned int   nu8:             1;  // (9) 8
        unsigned char  nu9;                 // (10)
        unsigned short ECTempC;             // (11,12)   Deg's C
        unsigned short TopTempF;            // (13,14)   Deg's F
        unsigned short BotTempF;            // (15,16)   Deg's F
        unsigned short VoltsIN;             // (17,18)
        unsigned short nu10;                // (19,20)
        unsigned short nu11;                // (21,22)
        unsigned short nu12;                // (23,24)
        unsigned short nu13;                // (25,26)
        unsigned short nu14;                // (27,28)
        unsigned short nu15;                // (29,30)
        unsigned short FChecksum;           // (31,32)
    } __attribute__ ((packed));

    // Status Packet Array
    unsigned char   byte[ NumHeaderBytes + NumStatusBytes ];
};

#endif  // _FIREPROTOCOL_H
