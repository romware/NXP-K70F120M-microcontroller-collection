/*! @file accel.c
 *
 *  @brief HAL for the accelerometer.
 *
 *  This contains the functions for interfacing to the MMA8451Q accelerometer.
 *
 *  @author 12403756, 12551519
 *  @date 2018-04-29
 */
/*!
**  @addtogroup accel_module accel module documentation
**  @{
*/
/* MODULE accel */

// Accelerometer functions
#include "accel.h"

// Inter-Integrated Circuit
#include "I2C.h"

// Median filter
#include "median.h"

// K70 module registers
#include "MK70F12.h"

// Periodic Interrupt Timer
#include "PIT.h"

// CPU and PE_types are needed for critical section variables and the definition of NULL pointer
#include "CPU.h"
#include "PE_types.h"

// Accelerometer registers
#define ADDRESS_OUT_X_MSB 0x01
#define ADDRESS_OUT_X_LSB 0x02
#define ADDRESS_OUT_Y_MSB 0x03
#define ADDRESS_OUT_Y_LSB 0x04
#define ADDRESS_OUT_Z_MSB 0x05
#define ADDRESS_OUT_Z_LSB 0x06

#define ADDRESS_INT_SOURCE 0x0C

static union
{
  uint8_t byte;			/*!< The INT_SOURCE bits accessed as a byte. */
  struct
  {
    uint8_t SRC_DRDY   : 1;	/*!< Data ready interrupt status. */
    uint8_t               : 1;
    uint8_t SRC_FF_MT  : 1;	/*!< Freefall/motion interrupt status. */
    uint8_t SRC_PULSE  : 1;	/*!< Pulse detection interrupt status. */
    uint8_t SRC_LNDPRT : 1;	/*!< Orientation interrupt status. */
    uint8_t SRC_TRANS  : 1;	/*!< Transient interrupt status. */
    uint8_t SRC_FIFO   : 1;	/*!< FIFO interrupt status. */
    uint8_t SRC_ASLP   : 1;	/*!< Auto-SLEEP/WAKE interrupt status. */
  } bits;			/*!< The INT_SOURCE bits accessed individually. */
} INT_SOURCE_Union;

#define INT_SOURCE     		INT_SOURCE_Union.byte
#define INT_SOURCE_SRC_DRDY	INT_SOURCE_Union.bits.SRC_DRDY
#define INT_SOURCE_SRC_FF_MT	CTRL_REG4_Union.bits.SRC_FF_MT
#define INT_SOURCE_SRC_PULSE	CTRL_REG4_Union.bits.SRC_PULSE
#define INT_SOURCE_SRC_LNDPRT	CTRL_REG4_Union.bits.SRC_LNDPRT
#define INT_SOURCE_SRC_TRANS	CTRL_REG4_Union.bits.SRC_TRANS
#define INT_SOURCE_SRC_FIFO	CTRL_REG4_Union.bits.SRC_FIFO
#define INT_SOURCE_SRC_ASLP	CTRL_REG4_Union.bits.SRC_ASLP

#define ADDRESS_CTRL_REG1 0x2A

typedef enum
{
  DATE_RATE_800_HZ,
  DATE_RATE_400_HZ,
  DATE_RATE_200_HZ,
  DATE_RATE_100_HZ,
  DATE_RATE_50_HZ,
  DATE_RATE_12_5_HZ,
  DATE_RATE_6_25_HZ,
  DATE_RATE_1_56_HZ
} TOutputDataRate;

typedef enum
{
  SLEEP_MODE_RATE_50_HZ,
  SLEEP_MODE_RATE_12_5_HZ,
  SLEEP_MODE_RATE_6_25_HZ,
  SLEEP_MODE_RATE_1_56_HZ
} TSLEEPModeRate;

static union
{
  uint8_t byte;			/*!< The CTRL_REG1 bits accessed as a byte. */
  struct
  {
    uint8_t ACTIVE    : 1;	/*!< Mode selection. */
    uint8_t F_READ    : 1;	/*!< Fast read mode. */
    uint8_t LNOISE    : 1;	/*!< Reduced noise mode. */
    uint8_t DR        : 3;	/*!< Data rate selection. */
    uint8_t ASLP_RATE : 2;	/*!< Auto-WAKE sample frequency. */
  } bits;			/*!< The CTRL_REG1 bits accessed individually. */
} CTRL_REG1_Union;

#define CTRL_REG1     		    CTRL_REG1_Union.byte
#define CTRL_REG1_ACTIVE	    CTRL_REG1_Union.bits.ACTIVE
#define CTRL_REG1_F_READ  	  CTRL_REG1_Union.bits.F_READ
#define CTRL_REG1_LNOISE  	  CTRL_REG1_Union.bits.LNOISE
#define CTRL_REG1_DR	    	  CTRL_REG1_Union.bits.DR
#define CTRL_REG1_ASLP_RATE	  CTRL_REG1_Union.bits.ASLP_RATE

#define ADDRESS_CTRL_REG2 0x2B

#define ADDRESS_CTRL_REG3 0x2C

static union
{
  uint8_t byte;			/*!< The CTRL_REG3 bits accessed as a byte. */
  struct
  {
    uint8_t PP_OD       : 1;	/*!< Push-pull/open drain selection. */
    uint8_t IPOL        : 1;	/*!< Interrupt polarity. */
    uint8_t WAKE_FF_MT  : 1;	/*!< Freefall/motion function in SLEEP mode. */
    uint8_t WAKE_PULSE  : 1;	/*!< Pulse function in SLEEP mode. */
    uint8_t WAKE_LNDPRT : 1;	/*!< Orientation function in SLEEP mode. */
    uint8_t WAKE_TRANS  : 1;	/*!< Transient function in SLEEP mode. */
    uint8_t FIFO_GATE   : 1;	/*!< FIFO gate bypass. */
  } bits;			/*!< The CTRL_REG3 bits accessed individually. */
} CTRL_REG3_Union;

#define CTRL_REG3     		    CTRL_REG3_Union.byte
#define CTRL_REG3_PP_OD		    CTRL_REG3_Union.bits.PP_OD
#define CTRL_REG3_IPOL		    CTRL_REG3_Union.bits.IPOL
#define CTRL_REG3_WAKE_FF_MT	CTRL_REG3_Union.bits.WAKE_FF_MT
#define CTRL_REG3_WAKE_PULSE	CTRL_REG3_Union.bits.WAKE_PULSE
#define CTRL_REG3_WAKE_LNDPRT	CTRL_REG3_Union.bits.WAKE_LNDPRT
#define CTRL_REG3_WAKE_TRANS	CTRL_REG3_Union.bits.WAKE_TRANS
#define CTRL_REG3_FIFO_GATE	  CTRL_REG3_Union.bits.FIFO_GATE

#define ADDRESS_CTRL_REG4 0x2D

static union
{
  uint8_t byte;			/*!< The CTRL_REG4 bits accessed as a byte. */
  struct
  {
    uint8_t INT_EN_DRDY   : 1;	/*!< Data ready interrupt enable. */
    uint8_t               : 1;
    uint8_t INT_EN_FF_MT  : 1;	/*!< Freefall/motion interrupt enable. */
    uint8_t INT_EN_PULSE  : 1;	/*!< Pulse detection interrupt enable. */
    uint8_t INT_EN_LNDPRT : 1;	/*!< Orientation interrupt enable. */
    uint8_t INT_EN_TRANS  : 1;	/*!< Transient interrupt enable. */
    uint8_t INT_EN_FIFO   : 1;	/*!< FIFO interrupt enable. */
    uint8_t INT_EN_ASLP   : 1;	/*!< Auto-SLEEP/WAKE interrupt enable. */
  } bits;			/*!< The CTRL_REG4 bits accessed individually. */
} CTRL_REG4_Union;

#define CTRL_REG4            		CTRL_REG4_Union.byte
#define CTRL_REG4_INT_EN_DRDY	  CTRL_REG4_Union.bits.INT_EN_DRDY
#define CTRL_REG4_INT_EN_FF_MT	CTRL_REG4_Union.bits.INT_EN_FF_MT
#define CTRL_REG4_INT_EN_PULSE	CTRL_REG4_Union.bits.INT_EN_PULSE
#define CTRL_REG4_INT_EN_LNDPRT	CTRL_REG4_Union.bits.INT_EN_LNDPRT
#define CTRL_REG4_INT_EN_TRANS	CTRL_REG4_Union.bits.INT_EN_TRANS
#define CTRL_REG4_INT_EN_FIFO	  CTRL_REG4_Union.bits.INT_EN_FIFO
#define CTRL_REG4_INT_EN_ASLP	  CTRL_REG4_Union.bits.INT_EN_ASLP

#define ADDRESS_CTRL_REG5 0x2E

static union
{
  uint8_t byte;			/*!< The CTRL_REG5 bits accessed as a byte. */
  struct
  {
    uint8_t INT_CFG_DRDY   : 1;	/*!< Data ready interrupt enable. */
    uint8_t                : 1;
    uint8_t INT_CFG_FF_MT  : 1;	/*!< Freefall/motion interrupt enable. */
    uint8_t INT_CFG_PULSE  : 1;	/*!< Pulse detection interrupt enable. */
    uint8_t INT_CFG_LNDPRT : 1;	/*!< Orientation interrupt enable. */
    uint8_t INT_CFG_TRANS  : 1;	/*!< Transient interrupt enable. */
    uint8_t INT_CFG_FIFO   : 1;	/*!< FIFO interrupt enable. */
    uint8_t INT_CFG_ASLP   : 1;	/*!< Auto-SLEEP/WAKE interrupt enable. */
  } bits;			/*!< The CTRL_REG5 bits accessed individually. */
} CTRL_REG5_Union;

#define CTRL_REG5     		      	CTRL_REG5_Union.byte
#define CTRL_REG5_INT_CFG_DRDY		CTRL_REG5_Union.bits.INT_CFG_DRDY
#define CTRL_REG5_INT_CFG_FF_MT		CTRL_REG5_Union.bits.INT_CFG_FF_MT
#define CTRL_REG5_INT_CFG_PULSE		CTRL_REG5_Union.bits.INT_CFG_PULSE
#define CTRL_REG5_INT_CFG_LNDPRT	CTRL_REG5_Union.bits.INT_CFG_LNDPRT
#define CTRL_REG5_INT_CFG_TRANS		CTRL_REG5_Union.bits.INT_CFG_TRANS
#define CTRL_REG5_INT_CFG_FIFO		CTRL_REG5_Union.bits.INT_CFG_FIFO
#define CTRL_REG5_INT_CFG_ASLP		CTRL_REG5_Union.bits.INT_CFG_ASLP

extern const uint32_t PERIOD_I2C_POLL; /*!< Period of the I2C polling in polling mode */

static OS_ECB* DataReadySemaphore;     /*!< Data ready semaphore for accel */

/*! @brief Initializes the accelerometer by calling the initialization routines of the supporting software modules.
 *
 *  @param accelSetup is a pointer to an accelerometer setup structure.
 *  @return bool - TRUE if the accelerometer module was successfully initialized.
 */
bool Accel_Init(const TAccelSetup* const accelSetup)
{
  // Set up the accelerometer as an I2C module
  TI2CModule aTI2CModule;
  aTI2CModule.baudRate = accelSetup->moduleClk;
  aTI2CModule.primarySlaveAddress = 0x1D;
  aTI2CModule.readCompleteSemaphore = accelSetup->readCompleteSemaphore;
  DataReadySemaphore = accelSetup->dataReadySemaphore;

  // Initialize the accelerometer as an I2C module
  I2C_Init(&aTI2CModule, accelSetup->moduleClk);

  // Ensure global interrupts are disabled
  EnterCritical();

  /* Address     | Vector | IRQ  | NVIC non-IPR register | NVIC IPR register | Source module       | Source description
   * 0x0000_01A0 | 104    | 88   | 2                     | 22                | Port control module | Pin detect (Port B)
   * IRQ modulo 32 = 24
   */

  // Clear any pending interrupts on pin detect B
  NVICICPR2 |= (1 << 24);

  // Enable interrupts from pin detect B
  NVICISER2 |= (1 << 24);

  // Return global interrupts to how they were
  ExitCritical();

  // Enable PORTB in System Clock Gating Control Register 5
  SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;

  // Set PTB4 (BGA Map 'N15') to be the I2C data ready interrupt pin by setting to ALT1
  PORTB_PCR4 = PORT_PCR_MUX(1);

  // Disable interrupt flag as it will be enabled in Accel_SetMode if Interrupt mode is selected
  PORTB_PCR4 &= ~PORT_PCR_IRQC_MASK;

  // Set up a 1 second Periodic Interrupt Timer for use in I2C polling mode.
  // PIT_Init(accelSetup->moduleClk, DataReadySemaphore);

  return true;
}

/*! @brief Reads X, Y and Z accelerations.
 *  @param data is a an array of 3 bytes where the X, Y and Z data are stored.
 */
void Accel_ReadXYZ(uint8_t data[3])
{
  // Reads X, Y and Z accelerations via interrupt reads
  I2C_IntRead(ADDRESS_OUT_X_MSB, data, 3);
}

/*! @brief Set the mode of the accelerometer.
 *  @param mode specifies either polled or interrupt driven operation.
 */
void Accel_SetMode(const TAccelMode mode)
{
  // Ensure the Accelerometer is out of active mode to allow setting of control registers
  CTRL_REG1_ACTIVE = 0;
  I2C_Write(ADDRESS_CTRL_REG1, CTRL_REG1);

  // Check which mode is to be set
  if(mode == ACCEL_POLL)
  {
    // Disable the DRDY interrupt from the accelerometer
    CTRL_REG4_INT_EN_DRDY = 0;
    I2C_Write(ADDRESS_CTRL_REG4, CTRL_REG4);

    // Disable the interrupt source of PTB4
    PORTB_PCR4 &= ~PORT_PCR_IRQC_MASK;

    // Set to fast read, 12.5 Hz and put back into active mode
    CTRL_REG1_F_READ = 1;
    CTRL_REG1_DR     = DATE_RATE_12_5_HZ;   // Set the 12.5 as we want this faster than our read rate to ensure we read new data
    CTRL_REG1_ACTIVE = 1;
    I2C_Write(ADDRESS_CTRL_REG1, CTRL_REG1);

    // Set the Periodic Interrupt Timer for use with I2C polling
    PIT_Set(PERIOD_I2C_POLL, true);
  }
  else if(mode == ACCEL_INT)
  {
    // Disable the PIT used for Poll
    PIT_Enable(false);

    // Route the DRDY interrupt to INT1
    CTRL_REG5_INT_CFG_DRDY = 1;
    I2C_Write(ADDRESS_CTRL_REG5, CTRL_REG5);

    // Enable/disable the DRDY interrupt
    CTRL_REG4_INT_EN_DRDY = mode;
    I2C_Write(ADDRESS_CTRL_REG4, CTRL_REG4);

    // Set to fast read, 1.56 Hz and put back into active mode
    CTRL_REG1_F_READ = 1;
    CTRL_REG1_DR     = DATE_RATE_1_56_HZ;
    CTRL_REG1_ACTIVE = 1;
    I2C_Write(ADDRESS_CTRL_REG1, CTRL_REG1);

    // Enable flag and interrupt on falling edge for PTB4
    PORTB_PCR4 |= PORT_PCR_IRQC(0b1010);
  }
}

/*! @brief Interrupt service routine for the accelerometer.
 *
 *  The accelerometer has data ready.
 *  The user callback function will be called.
 *  @note Assumes the accelerometer has been initialized.
 */
void __attribute__ ((interrupt)) AccelDataReady_ISR(void)
{
  // Notify RTOS of start of ISR
  OS_ISREnter();

  // Check the interrupt is from PTB4
  if(PORTB_ISFR & (1 << 4))
  {
    // Write 1 to clear flag
    PORTB_ISFR = (1 << 4);

    // Signal DataReady semaphore
    OS_SemaphoreSignal(DataReadySemaphore);
  }

  // Notify RTOS of exit of ISR
  OS_ISRExit();
}
/* END accel */
/*!
** @}
*/
