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

#define VRR_CHANNEL_1 0                    /*!< 1st VRR analog channel */
#define VRR_CHANNEL_2 1                    /*!< 2nd VRR analog channel */
#define VRR_CHANNEL_3 2                    /*!< 3rd VRR analog channel */

#define VRR_NB_CHANNELS 3                  /*!< Number of VRR analog channels */

#define VRR_SAMPLE_SIZE 16                 /*!< Number of VRR analog samples per cycle */

extern const int16_t VRR_OUT_ZERO;         /*!< VRR_VOLT x0 */
extern const int16_t VRR_VOLT_HALF;        /*!< VRR_VOLT x0.5 */
extern const int16_t VRR_VOLT;             /*!< One volt ((2^15 - 1)/10) */
extern const int16_t VRR_LIMIT_FREQUENCY;  /*!< VRR_VOLT x1.5 */
extern const int16_t VRR_VOLT_LIMIT_LOW;   /*!< VRR_VOLT x2 */
extern const int16_t VRR_VOLT_NOMINAL;     /*!< VRR_VOLT x2.5 */
extern const int16_t VRR_VOLT_LIMIT_HIGH;  /*!< VRR_VOLT x3 */
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
  int16_t sampleData[VRR_SAMPLE_SIZE];
  int16_t prevSampleData[VRR_SAMPLE_SIZE];
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

/*! @brief Insert samples from one array into another.
 *
 *  @param array1 is the array transferring from.
 *  @param array2 is the array transferring to.
 *  @param length is the length of both arrays.
 *  @return void
 */
void VRR_ArrayCopy(int16_t array1[], int16_t array2[], const uint8_t length);

/*! @brief Checks if any alarm in an array is true.
 *
 *  @param data is the array.
 *  @param length is the length of the array.
 *  @return bool - TRUE if any item is true
 */
bool VRR_ArrayAnyTrue(const bool data[], const uint8_t length);

/*! @brief Calculates the square root of a given value efficiently using previous roots.
 *
 *  @param targetSquare is the number to find the root of.
 *  @param prevRoot is the previously calculated root.
 *  @param prevSquare is the previously calculated square.
 *  @param lowLimit is estimated lowest root.
 *  @param highLimit is estimated highest root.
 *  @note Tested speed of Math.h sqrt function - 38us whereas this is 25us. Therefore on average 13us faster.
 *  @return uint32 - Square root value
 */
uint32_t VRR_QuickSquareRoot(const uint32_t targetSquare, uint32_t prevRoot, uint32_t prevSquare, const uint32_t lowLimit, const uint32_t highLimit);

/*! @brief Calculates RMS of given values.
 *
 *  @param data is the sample data from the ADC.
 *  @param length is the length of the data.
 *  @param prevRoot is the previously calculated RMS.
 *  @return uint16 - RMS value
 */
uint16_t VRR_CalculateRMS(const int16_t data[], const uint8_t length, const uint16_t prevRoot);

/*! @brief Triggers the alarm, adjustment and timer when the RMS is out of bounds.
 *
 *  @param timerDelay is the five second counter when an alarm has been triggered.
 *  @param minDelay is the one second minimum counter when an alarm has been triggered on inverse mode.
 *  @param timerRate is the rate which the timerDelay is decremented.
 *  @param defaultRate is the initial timerRate.
 *  @param mode is the timing mode setting.
 *  @param deviation is how much the RMS is away from the boundaries.
 *  @param alarm is whether it has been triggered on this channel.
 *  @param adjustment is whether it has been triggered on this channel.
 *  @param semaphore is to signal lowers or raises in Flash.
 *  @param channel is the the channel number on the ADC.
 *  @return void
 */
void VRR_CheckAlarm(int64_t* timerDelay, int64_t* minDelay, int64_t* timerRate, const uint32_t defaultRate, const uint8_t mode, const uint16_t deviation, bool* alarm, bool* adjustment, OS_ECB* semaphore, const int8_t channel);

/*! @brief Sets the frequency to a new value.
 *
 *  @param frequencyAccess the mutex to protect frequency.
 *  @param frequencyPtr is the pointer to the current frequency.
 *  @param frequency is the new frequency to set.
 *  @param rms is the current RMS value.
 *  @param periodAnalogSamplePtr is the pointer to the current sample rate.
 *  @return void
 */
void VRR_SetFrequency(OS_ECB* frequencyAccess, uint32_t * frequencyPtr, const uint32_t frequency, const uint16_t rms, uint32_t * periodAnalogSamplePtr);

/*! @brief Calculates the frequency from sample data.
 *
 *  @param sampleData is the sample data taken from the ADC.
 *  @param rms is value of the calculated RMS on the selected channel.
 *  @param periodAnalogSample is the period at which the ADC is sampling.
 *  @return uint32_t - The frequency
 */
uint32_t VRR_CalculateFrequency(const int16_t sampleData[], const uint16_t rms, const uint32_t periodAnalogSample);

#endif
