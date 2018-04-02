/*! @file
 *
 *  @brief Routines to implement packet encoding and decoding for the serial port.
 *
 *  This contains the functions for implementing the "Tower to PC Protocol" 5-byte packets.
 *
 *  @author PMcL, 12403756, 12551519
 *  @date 2015-07-23
 *  @modified 2018-04-02
 */

#ifndef PACKET_H
#define PACKET_H

// new types
#include "types.h"

// Listed command bits of a packet
#define TOWER_STARTUP 0x04
#define TOWER_VER 0x09
#define TOWER_NUM 0x0B

// Tower major version and minor version
#define TOWER_VER_MAJ 1
#define TOWER_VER_MIN 0

// Tower number get and set bits of packet parameter 1
#define TOWER_NUM_GET 1
#define TOWER_NUM_SET 2

// Packet structure
extern uint8_t 	Packet_Command,		/*!< The packet's command */
		Packet_Parameter1, 	/*!< The packet's 1st parameter */
		Packet_Parameter2, 	/*!< The packet's 2nd parameter */
		Packet_Parameter3,	/*!< The packet's 3rd parameter */
		Packet_Checksum;	/*!< The packet's checksum */

// Acknowledgment bit mask
extern const uint8_t PACKET_ACK_MASK;

// Tower number most and least significant bits
extern uint8_t Tower_Num_MSB;
extern uint8_t Tower_Num_LSB;

/*! @brief Initializes the packets by calling the initialization routines of the supporting software modules.
 *
 *  @param baudRate The desired baud rate in bits/sec.
 *  @param moduleClk The module clock rate in Hz
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

/*! @brief Generates a checksum from the given packet values
 *
 *  @return uint8_t - XOR value of all parameters.
 */
uint8_t Checksum_Generate(const uint8_t command, const uint8_t parameter1, const uint8_t parameter2, const uint8_t parameter3);

/*! @brief Sends the startup packets to the PC
 *
 *  @return bool - TRUE if all packets are sent
 */
bool Packet_Startup(void);

/*! @brief Executes a command depending on what packet has been received
 *
 *  @return void
 */
void Packet_Handle(void);

#endif
