/*! @file packet.c
 *
 *  @brief Routines to implement packet encoding and decoding for the serial port.
 *
 *  This contains the functions for implementing the "Tower to PC Protocol" 5-byte packets.
 *
 *  @author 12403756, 12551519
 *  @date 2018-04-13
 */
/*!
**  @addtogroup packet_module packet module documentation
**  @{
*/
/* MODULE packet */

#include "types.h"
#include "UART.h"
#include "packet.h"

const uint8_t PACKET_ACK_MASK = 0x80; /*!< Bit 7 of byte set to 1 */

TPacket Packet;                       /*!< Packet structure */

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
  uint8_t nextByte;

  // Check if the RxFIFO is not empty
  if(UART_InChar(&nextByte))
  {
    // Shift all packet bytes up and insert last byte from RxFIFO
    Packet_Command = Packet_Parameter1;
    Packet_Parameter1 = Packet_Parameter2;
    Packet_Parameter2 = Packet_Parameter3;
    Packet_Parameter3 = Packet_Checksum;
    Packet_Checksum = nextByte;

    // Check if packet checksum is equal to received checksum
    if((Packet_Command ^ Packet_Parameter1 ^ Packet_Parameter2 ^ Packet_Parameter3) == Packet_Checksum)
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
  uint8_t checksum = command ^ parameter1 ^ parameter2 ^ parameter3;
  // Put each byte of the packet in the transmit FIFO if it is not full.
  return (
    UART_OutChar(command) &&
    UART_OutChar(parameter1) &&
    UART_OutChar(parameter2) &&
    UART_OutChar(parameter3) &&
    UART_OutChar(checksum)
  );
}
/* END packet */
/*!
** @}
*/
