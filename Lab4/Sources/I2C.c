/*! @file I2C.c
 *
 *  @brief I/O routines for the K70 I2C interface.
 *
 *  This contains the functions for operating the I2C (inter-integrated circuit) module.
 *
 *  @author 12403756, 12551519
 *  @date 2018-04-13
 */
/*!
**  @addtogroup I2C_module median module documentation
**  @{
*/
/* MODULE I2C */

#include "I2C.h"
#include "MK70F12.h"
#include "Cpu.h"
#include "PE_Types.h"

static void StartCondition();
static void StopCondition();
static void WaitCondition();
static void BusyCondition();
static void RepeatCondition();

static void (*ReadCompleteCallbackFunction)(void*);  /*!<  Callback functions for I2C. */
static void* ReadCompleteCallbackArguments;          /*!< Callback parameters for I2C. */

static uint8_t SlaveDeviceAddress = 0x1D;            /*!< Current Slave address for I2C. */

/*! @brief Start condition on I2C Bus
 */
static void StartCondition()
{
  // Wait until the I2C bus is idle
  BusyCondition();
  I2C0_C1 |= I2C_C1_MST_MASK;
  I2C0_C1 |= I2C_C1_TX_MASK;
}

/*! @brief Stop condition on I2C Bus
 */
static void StopCondition()
{
  I2C0_C1 &= ~I2C_C1_MST_MASK;
}

/*! @brief Wait condition on I2C Bus
 */
static void WaitCondition()
{
  while(I2C0_S & ~I2C_S_IICIF_MASK) {}
  I2C0_S = I2C_S_IICIF_MASK;
}

/*! @brief Wait while busy condition on I2C Bus
 */
static void BusyCondition()
{
  while(I2C0_S & I2C_S_BUSY_MASK) {}
}

/*! @brief Repeat Start condition on I2C Bus
 */
static void RepeatCondition()
{
  I2C0_C1 |= I2C_C1_MST_MASK;
  I2C0_C1 |= I2C_C1_TX_MASK;
  I2C0_C1 |= I2C_C1_RSTA_MASK;
}

/*! @brief Sets up the I2C before first use.
 *
 *  @param aI2CModule is a structure containing the operating conditions for the module.
 *  @param moduleClk The module clock in Hz.
 *  @return BOOL - TRUE if the I2C module was successfully initialized.
 */
bool I2C_Init(const TI2CModule* const aI2CModule, const uint32_t moduleClk)
{
  // Store parameters for interrupt routine
  ReadCompleteCallbackFunction = aI2CModule->readCompleteCallbackFunction;
  ReadCompleteCallbackFunction = aI2CModule->readCompleteCallbackArguments;

  // Ensure global interrupts are disabled
  EnterCritical();

  /* Address     | Vector | IRQ  | NVIC non-IPR register | NVIC IPR register | Source module | Source description
   * 0x0000_00A0 | 40     | 24   | 0                     | 6                 | I2C0          | ---
   * IRQ modulo 32 = 24
   */

  // Clear any pending interrupts on I2C0
  NVICICPR0 |= (1 << 24);

  // Enable interrupts from I2C0 module
  NVICISER0 |= (1 << 24);

  // Return global interrupts to how they were
  ExitCritical();

  // Enable the I2C0 in System Clock Gating Control Register 4
  SIM_SCGC4 |= SIM_SCGC4_IIC0_MASK;

  // Enable PORTE in System Clock Gating Control Register 5
  SIM_SCGC5 |= SIM_SCGC5_PORTE_MASK;

  // Set PTE18 (BGA Map 'L4') to be the I2C0 SDA pin by setting to ALT4
  PORTE_PCR18 = PORT_PCR_MUX(4);

  // Set PTE19 (BGA Map 'M3') to be the I2C0 SCL pin by setting to ALT4
  PORTE_PCR19 = PORT_PCR_MUX(4);

  /* Set I2C Frequency Divider Register  TODO: Write a private function that performs an exhaustive search for these parameters
   * MULT: 0x02;   mul = 4;
   * ICR:  0x12;  SCL divider = 64;  SDA hold value = 13;  SCL hold (start) = 26;  SCL hold (stop) = 33;
   * I2C baud rate = 25Mhz / (4 * 640 = 97,656 bits/second
   */
  I2C0_F = I2C_F_MULT(0x2);
  I2C0_F = I2C_F_ICR(0x12);

  // Set I2C Enable
  I2C0_C1 |= I2C_C1_IICEN_MASK;

  // Enable I2C interrupts
  I2C0_C1 |= I2C_C1_IICIE_MASK;

  return true;
}

/*! @brief Selects the current slave device
 *
 * @param slaveAddress The slave device address.
 */
void I2C_SelectSlaveDevice(const uint8_t slaveAddress)
{
  // select slave address
  SlaveDeviceAddress = slaveAddress;
}

/*! @brief Write a byte of data to a specified register
 *
 * @param registerAddress The register address.
 * @param data The 8-bit data to write.
 */
void I2C_Write(const uint8_t registerAddress, const uint8_t data)
{
  BusyCondition();

  StartCondition();

  I2C0_D = SlaveDeviceAddress << 1;

  WaitCondition();

  I2C0_D = registerAddress;

  WaitCondition();

  I2C0_D = data;

  WaitCondition();

  StopCondition();
}

/*! @brief Reads data of a specified length starting from a specified register
 *
 * Uses polling as the method of data reception.
 * @param registerAddress The register address.
 * @param data A pointer to store the bytes that are read.
 * @param nbBytes The number of bytes to read.
 */
void I2C_PollRead(const uint8_t registerAddress, uint8_t* const data, const uint8_t nbBytes)
{

}

/*! @brief Reads data of a specified length starting from a specified register
 *
 * Uses interrupts as the method of data reception.
 * @param registerAddress The register address.
 * @param data A pointer to store the bytes that are read.
 * @param nbBytes The number of bytes to read.
 */
void I2C_IntRead(const uint8_t registerAddress, uint8_t* const data, const uint8_t nbBytes)
{

}

/*! @brief Interrupt service routine for the I2C.
 *
 *  Only used for reading data.
 *  At the end of reception, the user callback function will be called.
 *  @note Assumes the I2C module has been initialized.
 */
void __attribute__ ((interrupt)) I2C_ISR(void)    // TODO: We need to work out what this callback function needs to do
{
  if (ReadCompleteCallbackFunction)
    (*ReadCompleteCallbackFunction)(ReadCompleteCallbackArguments);
}
