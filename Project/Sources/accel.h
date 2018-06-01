/*! @file accel.h
 *
 *  @brief HAL for the accelerometer.
 *
 *  This contains the functions for interfacing to the MMA8451Q accelerometer.
 *
 *  @author 12403756, 12551519
 *  @date 2018-04-29
 */

#ifndef ACCEL_H
#define ACCEL_H

// New types
#include "types.h"
#include "OS.h"

typedef enum
{
  ACCEL_POLL,
  ACCEL_INT
} TAccelMode;

typedef struct
{
  uint32_t moduleClk;				/*!< The module clock rate in Hz. */
  OS_ECB* dataReadySemaphore;                   /*!< The user's data ready semaphore */
  OS_ECB* readCompleteSemaphore;                /*!< The user's read complete semaphore */
} TAccelSetup;

#pragma pack(push)
#pragma pack(1)

typedef union
{
  uint8_t bytes[3];				/*!< The accelerometer data accessed as an array. */
  struct
  {
    uint8_t x, y, z;				/*!< The accelerometer data accessed as individual axes. */
  } axes;
} TAccelData;

#pragma pack(pop)


/*! @brief Initializes the accelerometer by calling the initialization routines of the supporting software modules.
 *
 *  @param accelSetup is a pointer to an accelerometer setup structure.
 *  @return bool - TRUE if the accelerometer module was successfully initialized.
 */
bool Accel_Init(const TAccelSetup* const accelSetup);

/*! @brief Reads X, Y and Z accelerations.
 *  @param data is a an array of 3 bytes where the X, Y and Z data are stored.
 */
void Accel_ReadXYZ(uint8_t data[3]);

/*! @brief Set the mode of the accelerometer.
 *  @param mode specifies either polled or interrupt driven operation.
 */
void Accel_SetMode(const TAccelMode mode);

/*! @brief Interrupt service routine for the accelerometer.
 *
 *  The accelerometer has data ready.
 *  The user callback function will be called.
 *  @note Assumes the accelerometer has been initialized.
 */
void __attribute__ ((interrupt)) AccelDataReady_ISR(void);

#endif
