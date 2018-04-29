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
static uint8_t SlaveDeviceReadAddress;               /*!< Current Slave device read address for I2C. */
static uint8_t NbBytes = 0;                          /*!< Number of bytes in current read. */
static uint8_t* DataPtr;                             /*!< Pointer to where to store read bytes. */

/*! @brief Start condition on I2C Bus
 */
static void StartCondition()
{
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
  while(!(I2C0_S & I2C_S_IICIF_MASK)) {}
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
  ReadCompleteCallbackArguments = aI2CModule->readCompleteCallbackArguments;

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

  // Enable open drain
  PORTE_PCR18 |= PORT_PCR_ODE_MASK;

  // Set PTE19 (BGA Map 'M3') to be the I2C0 SCL pin by setting to ALT4
  PORTE_PCR19 = PORT_PCR_MUX(4);

  // Enable open drain
  PORTE_PCR19 |= PORT_PCR_ODE_MASK;

  /* Set I2C Frequency Divider Register  TODO: Write a private function that performs an exhaustive search for these parameters
   * MULT: 0x02;   mul = 4;
   * ICR:  0x12;  SCL divider = 64;  SDA hold value = 13;  SCL hold (start) = 26;  SCL hold (stop) = 33;
   * I2C baud rate = 25Mhz / (4 * 640 = 97,656 bits/second
   */ //TODO: Test properly, neaten up, remove comments above, also maybe check if result if acceptable otherwise return false??

  // Array to store mul and icr

  uint8_t mulArray[]  = {1,2,4};
  uint16_t sclArray[] = {20,22,24,26,28,30,34,40,28,32,36,40,44,48,56,68,48,
                         56,64,72,80,88,104,128,80,96,112,128,144,160,192,
			 240,160,192,224,256,288,320,384,448,512,576,640,768,
			 960,640,768,896,1024,1152,1280,1536,1920,1280,1536,
			 1792,2048,2304,2560,3072,3840};

  // Variables to store best matches, the difference and the test difference
  uint8_t bestMul = 0;
  uint8_t bestScl = 0;
  uint32_t baudRateDifference = 0xFFFFFFFF;
  uint32_t testBaudRate;

  for(uint8_t i = 0; i < 3; i++)
  {
    for(uint8_t j = 0; j < 64; j++)
    {
      testBaudRate = moduleClk / (mulArray[i] * sclArray[j]);

      // If the testBaudRate is smaller than or equal to the desired baud rate
      if(aI2CModule->baudRate >= testBaudRate)
      {
        if((aI2CModule->baudRate - testBaudRate) < baudRateDifference)
        {
          baudRateDifference = (aI2CModule->baudRate - testBaudRate);
          bestMul = i;
          bestScl = j;
        }
      }
      // IF the testBaudRate is larger than the desired baud rate
      else if (aI2CModule->baudRate < testBaudRate)
      {
        if((testBaudRate - aI2CModule->baudRate) < baudRateDifference)
        {
          baudRateDifference = (aI2CModule->baudRate - testBaudRate);
          bestMul = i;
          bestScl = j;
        }
      }
    }
  }

  I2C0_F = I2C_F_MULT(bestMul);
  I2C0_F = I2C_F_ICR(bestScl);

  // Set I2C Enable
  I2C0_C1 |= I2C_C1_IICEN_MASK;

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
  // Wait until the I2C bus is idle
  BusyCondition();

  // Initiate communications by sending start signal
  StartCondition();

  // Send slave device address with write bit
  I2C0_D = SlaveDeviceAddress << 1;

  // Wait for AK from slave
  WaitCondition();

  // Send read register address
  I2C0_D = registerAddress;

  // Wait for AK from slave
  WaitCondition();

  // Send Repeat start signal to slave
  RepeatCondition();

  // Send slave device address with read bit
  I2C0_D = (SlaveDeviceAddress << 1) | 1;

  // Wait for AK from slave device
  WaitCondition();

  // Change to RX mode
  I2C0_C1 &= ~I2C_C1_TX_MASK;

  // Ensure TXAK is clear
  I2C0_C1 &= ~I2C_C1_TXAK_MASK;

  // Dummy read from register
  data[0] = I2C0_D;
  WaitCondition();

  // Read until second last byte of data
  for (int i = 0; i < nbBytes-2; i++)
  {
    // Load received byte into data
    data[i] = I2C0_D;

    WaitCondition();
  }

  // Set TXACK prior to reading second last byte
  I2C0_C1 |= I2C_C1_TXAK_MASK;

  // Read second last of data
  data[nbBytes-2] = I2C0_D;
  WaitCondition();

  // Generate stop signal prior to reading last byte of data
  StopCondition();

  // Read last byte of data
  data[nbBytes-1] = I2C0_D;
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
  // Store the number of bytes to be read
  NbBytes = nbBytes;

  // Store the pointer to data
  DataPtr = data;

  // Store the slave device read register address
  SlaveDeviceReadAddress = registerAddress;

  // Wait until the I2C bus is idle
  BusyCondition();

  // Initiate communications by sending start signal
  StartCondition();

  // Send slave device address with write bit
  I2C0_D = SlaveDeviceAddress << 1;

  // Clear any interrupts
  I2C0_S = I2C_S_IICIF_MASK;

  // Enable interrupts
  I2C0_C1 |= I2C_C1_IICIE_MASK;
}

/*! @brief Interrupt service routine for the I2C.
 *
 *  Only used for reading data.
 *  At the end of reception, the user callback function will be called.
 *  @note Assumes the I2C module has been initialized.
 */
void __attribute__ ((interrupt)) I2C_ISR(void)    // TODO: We need to work out what this callback function needs to do
{
  // To keep track of the current stage in the sequence
  static uint8_t stage = 0;

  // To keep track of bytes read
  static uint8_t readCount = 0;

  // Clear the interrupt
  I2C0_S = I2C_S_IICIF_MASK;

  switch(stage)
  {
    case 0:
      // Send read register address
      I2C0_D = SlaveDeviceReadAddress;

      stage++;
      break;

    case 1:
      // Send Repeat start signal to slave
      RepeatCondition();

      // Send slave device address with read bit
      I2C0_D = (SlaveDeviceAddress << 1) | 1;

      stage++;
      break;

    case 2:
      // Change to RX mode
      I2C0_C1 &= ~I2C_C1_TX_MASK;

      // Ensure TXAK is clear
      I2C0_C1 &= ~I2C_C1_TXAK_MASK;

      // Dummy read from register
      DataPtr[0] = I2C0_D;

      stage++;
      break;

    case 3:
      if(readCount < NbBytes - 2)
      {
        // Load received byte into data
        DataPtr[readCount] = I2C0_D;

        readCount++;
      }
      else if(readCount == NbBytes - 2)
      {
        // Set TXACK prior to reading second last byte
        I2C0_C1 |= I2C_C1_TXAK_MASK;

        // Read second last of data
        DataPtr[readCount] = I2C0_D;

        readCount++;
      }
      else if(readCount == NbBytes - 1)
      {
        // Generate stop signal prior to reading last byte of data
        StopCondition();

        // Read last byte of data
        DataPtr[readCount] = I2C0_D;

        // Disable interrupts
        I2C0_C1 &= ~I2C_C1_IICIE_MASK;

        stage = 0;
        readCount = 0;

        // Call user callback function
        if (ReadCompleteCallbackFunction)
          (*ReadCompleteCallbackFunction)(ReadCompleteCallbackArguments);
      }
      break;
  }
}

