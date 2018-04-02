/*! @file packet.c
 *
 *  @brief Routines to implement packet encoding and decoding for the serial port.
 *
 *  This contains the functions for implementing the "Tower to PC Protocol" 5-byte packets.
 *
 *  @author PMcL, 12403756, 12551519
 *  @date 2015-07-23
 *  @modified 2018-04-02
 */

#include "packet.h"

/*! @brief Initializes the packets by calling the initialization routines of the supporting software modules.
 *
 *  @param baudRate The desired baud rate in bits/sec.
 *  @param moduleClk The module clock rate in Hz
 *  @return bool - TRUE if the packet module was successfully initialized.
 */
bool Packet_Init(const uint32_t baudRate, const uint32_t moduleClk)
{
  // Sets up the UART interface before first use.
  return UART_Init(baudRate, moduleClk);
}

/*! @brief Attempts to get a packet from the received data.
 *
 *  @return bool - TRUE if a valid packet was received.
 */
bool Packet_Get(void)
{
  // Check if the RxFIFO is not empty
  if(RxFIFO.NbBytes > 0)
  {
    // Shift all packet bytes up and insert last byte from RxFIFO
    Packet_Command = Packet_Parameter1;
    Packet_Parameter1 = Packet_Parameter2;
    Packet_Parameter2 = Packet_Parameter3;
    Packet_Parameter3 = Packet_Checksum;
    UART_InChar(&Packet_Checksum);

    // Check if packet checksum is equal to received checksum
    if(Checksum_Generate(Packet_Command,Packet_Parameter1,Packet_Parameter2,Packet_Parameter3) == Packet_Checksum)
    {
      return true;
    }
  }
  return false;
}

/*! @brief Builds a packet and places it in the transmit FIFO buffer.
 *
 *  @return bool - TRUE if a valid packet was sent.
 */
bool Packet_Put(const uint8_t command, const uint8_t parameter1, const uint8_t parameter2, const uint8_t parameter3)
{
  // Generates the XOR checksum of a packet.
  uint8_t checksum = Checksum_Generate(command, parameter1, parameter2, parameter3);
  // Put each byte of the packet in the transmit FIFO if it is not full.
  return (
    UART_OutChar(command) && 
    UART_OutChar(parameter1) && 
    UART_OutChar(parameter2) && 
    UART_OutChar(parameter3) && 
    UART_OutChar(checksum)
  );
}

/*! @brief Generates the XOR checksum of a packet.
 *
 *  @return uint8_t - XOR value of parameters.
 */
uint8_t Checksum_Generate(const uint8_t command, const uint8_t parameter1, const uint8_t parameter2, const uint8_t parameter3)
{
  // XOR each byte.
  return command ^ parameter1 ^ parameter2 ^ parameter3;
}

/*! @brief Sends the startup packets to the PC
 *
 *  @return bool - TRUE if all packets are sent
 */
bool Packet_Startup(void)
{
  // Sends the tower startup values, version and number to the PC
  return (
    Packet_Put(TOWER_STARTUP,0x00,0x00,0x00) &&
    Packet_Put(TOWER_VER,'v',TOWER_VER_MAJ,TOWER_VER_MIN) &&
    Packet_Put(TOWER_NUM,TOWER_NUM_GET,Tower_Num_LSB,Tower_Num_MSB)
  );
}

/*! @brief Executes the command depending on what packet has been received
 *
 *  @return void
 */
void Packet_Handle(void)
{
  // Initializes the success status of the received packet to false
  bool success = false;

  // AND the packet command byte with the bitwise inverse ACK MASK to ignore if ACK is requested
  switch (Packet_Command & ~PACKET_ACK_MASK)
  {
    // Check if command byte matches tower startup byte
    case TOWER_STARTUP:
      // Check if parameters match tower startup parameters
      if(Packet_Parameter1 == 0 && Packet_Parameter2 == 0 && Packet_Parameter3 == 0)
      {
	  // Send tower startup packets
	  success = Packet_Startup();
      }
      break;

    // Check if command byte matches tower version byte
    case TOWER_VER:
      // Check if parameters match tower version parameters
      if(Packet_Parameter1 == 'v' && Packet_Parameter2 == 'x' && Packet_Parameter3 == 13)
      {
	  // Send tower version packet
	  success = Packet_Put(TOWER_VER,'v',TOWER_VER_MAJ,TOWER_VER_MIN);
      }
      break;

    // Check if command byte matches tower number byte
    case TOWER_NUM:
      // Check if parameters match tower number GET or SET parameters
      if(Packet_Parameter1 == TOWER_NUM_GET && Packet_Parameter2 == 0 && Packet_Parameter3 == 0)
      {
	  // Send tower number packet
	  success = Packet_Put(TOWER_NUM,TOWER_NUM_GET,Tower_Num_LSB,Tower_Num_MSB);
      }
      else if (Packet_Parameter1 == TOWER_NUM_SET)
      {
	// Change tower number
        Tower_Num_LSB = Packet_Parameter2;
        Tower_Num_MSB = Packet_Parameter3;
        success = true;
      }
      break;

    default:
      // Sets success status to false if command bit is invalid
      success = false;
      break;
  }

  // AND the packet command byte with the ACK MASK to check if ACK is requested
  if(Packet_Command & PACKET_ACK_MASK)
  {
    // Check the success status of the packet which was sent
    if(success == false)
    {
      // Return the sent packet with the ACK command if successful
      Packet_Put(Packet_Command & ~PACKET_ACK_MASK,Packet_Parameter1,Packet_Parameter2,Packet_Parameter3);
    }
    else
    {
      // Return the sent packet with the NACK command if unsuccessful
      Packet_Put(Packet_Command,Packet_Parameter1,Packet_Parameter2,Packet_Parameter3);
    }
  }

  // Reset the packet variables to 0
  Packet_Command = 0;
  Packet_Parameter1 = 0;
  Packet_Parameter2 = 0;
  Packet_Parameter3 = 0;
  Packet_Checksum = 0;
}
