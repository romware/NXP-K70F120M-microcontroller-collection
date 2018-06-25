/*! @file  VRR.h
 *
 *  @brief Routines for voltage regulating relay.
 *
 *  This contains the functions needed for accessing the VRR.
 *
 *  @author 12551519
 *  @date 2018-06-13
 */

#ifndef VRR_H
#define VRR_H

// new types
#include "types.h"
#include "OS.h"

#define ANALOG_CHANNEL_1 0                                   /*!< 1st analog channel */
#define ANALOG_CHANNEL_2 1                                   /*!< 2nd analog channel */
#define ANALOG_CHANNEL_3 2                                   /*!< 3rd analog channel */

#define NB_ANALOG_CHANNELS 3                                 /*!< Number of analog channels */

#define ANALOG_SAMPLE_SIZE 16                                /*!< Number of analog samples per cycle */

extern const uint64_t PERIOD_TIMER_DELAY;  /*!< Period of analog timer delay (5 seconds) */
extern const uint64_t PERIOD_MIN_DELAY;    /*!< Period of analog timer delay minium (1 second) */

extern const int16_t VRR_OUT_ZERO;         /*!< VRR_VOLT x0 */
extern const int16_t VRR_VOLT_HALF;        /*!< VRR_VOLT x0.5 */
extern const int16_t VRR_VOLT;             /*!< One volt ((2^15 - 1)/10) */
extern const int16_t VRR_LIMIT_FREQUENCY;  /*!< VRR_VOLT x1.5 */
extern const int16_t VRR_LIMIT_LOW;        /*!< VRR_VOLT x2 */
extern const int16_t VRR_LIMIT_HIGH;       /*!< VRR_VOLT x3 */
extern const int16_t VRR_OUT_FIVE;         /*!< VRR_VOLT x5 */

/*! @brief Timing mode parameters for tower to PC protocol
 *
 */
typedef enum
{
  TIMING_GET,
  TIMING_DEFINITE,
  TIMING_INVERSE
} TTimingMode;

/*! @brief Deviation parameters for tower to PC protocol
 *
 */
typedef enum
{
  DEVIATION_GET,
  DEVIATION_RESET
} TDeviation;

/*! @brief Data structure used to pass Analog configuration to a user thread
 *
 */
typedef struct AnalogThreadData
{
  OS_ECB* semaphoreRMS;
  uint8_t channelNb;
  int16_t sampleData[ANALOG_SAMPLE_SIZE];
  int16_t prevSampleData[ANALOG_SAMPLE_SIZE];
  uint16_t sampleDataIndex;
  uint16_t currentRMS;
} TAnalogThreadData;

/*! @brief Sets up the VRR before first use.
 *
 *  @return bool - TRUE if the VRR was successfully initialized.
 */
bool VRR_Init(void);

/*! @brief Sends a value to an analog input channel.
 *
 *  @param channelNb is the number of the analog output channel to send the value to.
 *  @param value is the value to write to the analog channel.
 *  @return bool - true if the analog value was output successfully.
 */
bool VRR_ProtectedAnalogPut(uint8_t const channelNb, int16_t const value);

/*! @brief Insert into from one array into another TODO: params + comments
 *
 *  @return void
 */
void VRR_ArrayCopy(int16_t array1[], int16_t array2[], const uint8_t length);

/*! @brief Checks if any item in array is true TODO:params + comments
 *
 *  @return bool - TRUE if any item is true
 */
bool VRR_ArrayAnyTrue(const bool data[], const uint8_t length);

/*! @brief Calculates the square root of a given value efficiently TODO:params + comments
 *
 *  @note Tested speed of Math.h sqrt function - 38us whereas this is 25us. Therefore on average 13us faster
 *  @return uint16 - Square root value
 */
uint32_t VRR_QuickSquareRoot(const uint32_t targetSquare, uint32_t prevRoot, uint32_t prevSquare, const uint32_t lowLimit, const uint32_t highLimit);

/*! @brief Calculates RMS of given values TODO:params + comments
 *
 *  @return uint16_t - RMS value
 */
uint16_t VRR_CalculateRMS(const int16_t data[], const uint8_t length, const uint16_t prevRoot);

/*! @brief  TODO:brief + params + comments
 *
 *  @return
 */
void VRR_CheckAlarm(int64_t* timerDelay, int64_t* minDelay, int64_t* timerRate, const uint32_t defaultRate, const uint8_t mode, const uint16_t deviation, bool* alarm, bool* adjustment, OS_ECB* semaphore, const int8_t channel);


void VRR_SetFrequency(OS_ECB* frequencyAccess, uint32_t * frequencyPtr, const uint32_t frequency, const uint16_t rms, uint32_t * analogPollPtr);


uint32_t VRR_CalculateFrequency(const int16_t sampleData[], const uint16_t rms, const uint32_t analogPoll);

#endif
