/*! @file  VRR.c
 *
 *  @brief Routines for voltage regulating relay.
 *
 *  This contains the functions needed for accessing the VRR.
 *
 *  @author 12551519
 *  @date 2018-06-13
 */
/*!
**  @addtogroup VRR_module VRR module documentation
**  @{
*/
/* MODULE VRR */

#include "types.h"
#include "MK70F12.h"
#include "analog.h"
#include "VRR.h"
#include "OS.h"
#include "PIT.h"

const uint64_t VRR_PERIOD_TIMER_DELAY   = 5000000000;             /*!< Period of analog timer delay (5 seconds) */
const uint64_t VRR_PERIOD_MIN_DELAY     = 1000000000;             /*!< Period of analog timer delay minium (1 second) */

const uint32_t VRR_FREQUENCY_DEFAULT    =        500;             /*!< Default frequency (50Hz) */
const uint32_t VRR_FREQUENCY_LIMIT_LOW  =        475;             /*!< Low limit frequency (47.5Hz) */
const uint32_t VRR_FREQUENCY_LIMIT_HIGH =        525;             /*!< High limit frequency (52.5Hz) */

const int16_t VRR_OUT_ZERO              =          0;             /*!< VRR_VOLT x0 */
const int16_t VRR_VOLT_HALF             =       1638;             /*!< VRR_VOLT x0.5 */
const int16_t VRR_VOLT                  =       3277;             /*!< One volt ((2^15 - 1)/10) */
const int16_t VRR_LIMIT_FREQUENCY       =       4915;             /*!< VRR_VOLT x1.5 */
const int16_t VRR_VOLT_LIMIT_LOW        =       6553;             /*!< VRR_VOLT x2 */
const int16_t VRR_VOLT_NOMINAL          =       8192;             /*!< VRR_VOLT x2.5 */
const int16_t VRR_VOLT_LIMIT_HIGH       =       9830;             /*!< VRR_VOLT x3 */
const int16_t VRR_OUT_FIVE              =      16384;             /*!< VRR_VOLT x5 */


/*! @brief Sets up the VRR before first use.
 *
 *  @return bool - TRUE if the VRR was successfully initialized.
 */
bool VRR_Init(void)
{
  return true;
}

/*! @brief Sends a value to an analog input channel.
 *
 *  @param channelNb is the number of the analog output channel to send the value to.
 *  @param value is the value to write to the analog channel.
 *  @return bool - true if the analog value was output successfully.
 */
bool VRR_ProtectedAnalogPut(uint8_t const channelNb, int16_t const value)
{
  OS_DisableInterrupts();
  Analog_Put(channelNb, value);
  OS_EnableInterrupts();
}

/*! @brief Calculates the square root of a given value efficiently using previous roots.
 *
 *  @param targetSquare is the number to find the root of.
 *  @param prevRoot is the previously calculated root.
 *  @note tested speed of Math.h sqrt function - 38us whereas this is 25us. Therefore on average 13us faster.
 *  @return uint32 - Square root value
 */
uint32_t VRR_QuickSquareRoot(const uint32_t targetSquare, uint32_t prevRoot)
{
  int32_t distance;
  uint32_t targetRoot = 1;

  // If there is a previous estimate use it as a starting point
  if(prevRoot)
  {
    targetRoot = prevRoot;
  }

  // Ensure the square is not 0
  if(targetSquare)
  {
    // Keep track of last estimate
    prevRoot = targetRoot;

    // Use Newton's method to adjust root estimate
    targetRoot = (targetRoot + (targetSquare / targetRoot)) / 2;

    // Find distance between root estimates
    distance = targetRoot - prevRoot;

    // Repeat until estimates distance is 1 or less
    while(distance > 1 || distance < -1)
    {
      prevRoot = targetRoot;
      targetRoot = (targetRoot + (targetSquare / targetRoot)) / 2;
      distance = targetRoot - prevRoot;
    }
    return targetRoot;
  }
  return 0;
}

/*! @brief Calculates RMS of given values.
 *
 *  @param data is the sample data from the ADC.
 *  @param length is the length of the data.
 *  @param prevRoot is the previously calculated RMS.
 *  @return uint16 - RMS value
 */
uint16_t VRR_CalculateRMS(const int16_t data[], const uint8_t length, const uint16_t prevRoot)
{
  // Add all the squared samples in the array
  int64_t sum = 0;
  for(uint8_t i = 0; i < length; i++)
  {
    sum += (data[i])*(data[i]);
  }

  // Calculate efficiently the square root of the sum squared
  return (uint16_t)VRR_QuickSquareRoot((sum/length), prevRoot);
}

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
void VRR_CheckAlarm(int64_t* timerDelay, int64_t* minDelay, int64_t* timerRate, const uint32_t defaultRate, const uint8_t mode,
                    const uint16_t deviation, bool* alarm, bool* adjustment, OS_ECB* semaphore, const int8_t channel)
{
  // Check if alarm has been triggered, or timer is still decrementing or adjustment has been signaled
  if(!(*alarm))
  {
    // Turn alarm on and initialize the timers
    *alarm = true;
    *timerDelay = VRR_PERIOD_TIMER_DELAY;
    *minDelay = VRR_PERIOD_MIN_DELAY;
    *timerRate = defaultRate;
    VRR_ProtectedAnalogPut(VRR_CHANNEL_3, VRR_OUT_FIVE);
  }
  else if(*timerDelay >= *timerRate || *minDelay >= *timerRate)
  {
    if(mode == TIMING_INVERSE)
    {
      // Adjust the rate for inverse timing to be relative to the deviation
      *timerRate = (deviation/(uint16_t)VRR_VOLT_HALF) * defaultRate;
    }

    // Decrement timers
    *timerDelay -= *timerRate;
    *minDelay -= defaultRate;
  }
  else if(!*adjustment)
  {
    // Turn raise or lower on and increment Flash
    *adjustment = true;
    VRR_ProtectedAnalogPut(channel, VRR_OUT_FIVE);
    OS_SemaphoreSignal(semaphore);
  }
}

/*! @brief Sets the frequency to a new value.
 *
 *  @param frequencyAccess the mutex to protect frequency.
 *  @param frequencyPtr is the pointer to the current frequency.
 *  @param frequency is the new frequency to set.
 *  @param rms is the current RMS value.
 *  @param periodAnalogSamplePtr is the pointer to the current sample rate.
 *  @return void
 */
void VRR_SetFrequency(OS_ECB* frequencyAccess, uint32_t * frequencyPtr, const uint32_t frequency, const uint16_t rms, uint32_t * periodAnalogSamplePtr)
{
  // Ensure protection for reading and writing the frequency
  (void)OS_SemaphoreWait(frequencyAccess, 0);
  OS_DisableInterrupts();

  // Check if the RMS has dropped below the minimum for frequency tracking
  if(rms < VRR_LIMIT_FREQUENCY)
  {
    // Set the frequency to default
    *frequencyPtr = VRR_FREQUENCY_DEFAULT;
    *periodAnalogSamplePtr = (VRR_PERIOD_MIN_DELAY / (*frequencyPtr/10)) / VRR_SAMPLE_SIZE;
    PIT_Set(*periodAnalogSamplePtr / VRR_NB_CHANNELS, true);
  }
  else if(*frequencyPtr != frequency && frequency >= VRR_FREQUENCY_LIMIT_LOW && frequency <= VRR_FREQUENCY_LIMIT_HIGH)
  {
    // Set the frequency to the newly calculated value
    *frequencyPtr = frequency;
    *periodAnalogSamplePtr = (VRR_PERIOD_MIN_DELAY / (*frequencyPtr/10)) / VRR_SAMPLE_SIZE;
    PIT_Set(*periodAnalogSamplePtr / VRR_NB_CHANNELS, true);
  }

  OS_EnableInterrupts();
  (void)OS_SemaphoreSignal(frequencyAccess);
}

/*! @brief Calculates the frequency from sample data.
 *
 *  @param sampleData is the sample data taken from the ADC.
 *  @param rms is value of the calculated RMS on the selected channel.
 *  @param periodAnalogSample is the period at which the ADC is sampling.
 *  @return uint32_t - The frequency
 */
uint32_t VRR_CalculateFrequency(const int16_t sampleData[], const uint16_t rms, const uint32_t periodAnalogSample)
{
  bool negCrossingFound = false;
  uint32_t negCrossing1 = 0;
  uint32_t negCrossing2 = 0;

  // Loop through the two most recent cycles
  for(uint8_t i = 0; i < VRR_SAMPLE_SIZE*2 - 1; i++)
  {
    // Read two samples directly next to each other
    int16_t leftVal = sampleData[i];
    int16_t rightVal = sampleData[i+1];

    // Check if the samples are a negative crossing and if one has already been found or not
    if(leftVal >= 0 && 0 > rightVal && !negCrossingFound)
    {
      negCrossingFound = true;

      // Find the gradient over one sample (rise / run, run = 1 sample)
      int32_t m = (rightVal - leftVal);

      // Find the point of interception to improve accuracy of the distance (x = -b / m)
      int32_t x = ((-leftVal) << 10) / m;
      negCrossing1 = (i << 10) + x;
    }
    else if(leftVal >= 0 && 0 > rightVal && negCrossingFound)
    {
      // Find the gradient over one sample (rise / run, run = 1 sample)
      int32_t m = (rightVal - leftVal);

      // Find the point of interception to improve accuracy of the distance (x = -b / m)
      int32_t x = ((-leftVal) << 10) / m;
      negCrossing2 = (i << 10) + x;

      // Find the distance between the two crossings and multiple it by the current sampling rate
      uint32_t distance = negCrossing2 - negCrossing1;
      uint32_t oldPoll = periodAnalogSample >> 10;
      uint64_t newPoll = distance * oldPoll;

      // Calculate the frequency * 10 by dividing the sample rate from 1 second multiplied by 10
      uint64_t frequency = (10000000000 / newPoll);
      return frequency;
    }
  }
  return 0;
}

/* END VRR */
/*!
** @}
*/
