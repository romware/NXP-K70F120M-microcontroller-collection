/*! @file packet.h
 *
 *  @brief Routines to implement packet encoding and decoding for the serial port.
 *
 *  This contains the functions for implementing the "Tower to PC Protocol" 5-byte packets.
 *
 *  @author 12403756, 12551519
 *  @date 2018-04-13
 */

#ifndef PACKET_H
#define PACKET_H

// New types
#include "types.h"

// Packet structure
#define PACKET_NB_BYTES 5

#pragma pack(push)
#pragma pack(1)

typedef union
{
  uint8_t bytes[PACKET_NB_BYTES];     /*!< The packet as an array of bytes. */
  struct
  {
    uint8_t command;		      /*!< The packet's command. */
    union
    {
      struct
      {
        uint8_t parameter1;	      /*!< The packet's 1st parameter. */
        uint8_t parameter2;	      /*!< The packet's 2nd parameter. */
        uint8_t parameter3;	      /*!< The packet's 3rd parameter. */
      } separate;
      struct
      {
        uint16_t parameter12;         /*!< Parameter 1 and 2 concatenated. */
        uint8_t parameter3;
      } combined12;
      struct
      {
        uint8_t paramater1;
        uint16_t parameter23;         /*!< Parameter 2 and 3 concatenated. */
      } combined23;
    } parameters;
    uint8_t checksum;
  } packetStruct;
} TPacket;

#pragma pack(pop)

#define Packet_Command     Packet.packetStruct.command
#define Packet_Parameter1  Packet.packetStruct.parameters.separate.parameter1
#define Packet_Parameter2  Packet.packetStruct.parameters.separate.parameter2
#define Packet_Parameter3  Packet.packetStruct.parameters.separate.parameter3
#define Packet_Parameter12 Packet.packetStruct.parameters.combined12.parameter12
#define Packet_Parameter23 Packet.packetStruct.parameters.combined23.parameter23
#define Packet_Checksum    Packet.packetStruct.checksum

extern TPacket Packet;

// Acknowledgment bit mask
extern const uint8_t PACKET_ACK_MASK;

/*! @brief Initializes the packets by calling the initialization routines of the supporting software modules.
 *
 *  @param baudRate The desired baud rate in bits/sec.
 *  @param moduleClk The module clock rate in Hz.
 *  @return bool - TRUE if the packet module was successfully initialized.
 */
bool Packet_Init(const uint32_t baudRate, const uint32_t moduleClk);

/*! @brief Attempts to get a packet from the received data.
 *
 *  @return bool - TRUE if a valid packet was received.
 */
bool Packet_Get(void);

/*! @brief Builds a packet and places it in the transmit FIFO buffer.
 *
 *  @return bool - TRUE if a valid packet was sent.
 */
bool Packet_Put(const uint8_t command, const uint8_t parameter1, const uint8_t parameter2, const uint8_t parameter3);

#endif
