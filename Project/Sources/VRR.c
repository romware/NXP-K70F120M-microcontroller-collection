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

const uint64_t PERIOD_TIMER_DELAY  = 5000000000;             /*!< Period of analog timer delay (5 seconds) */
const uint64_t PERIOD_MIN_DELAY    = 1000000000;             /*!< Period of analog timer delay minium (1 second) */

const int16_t VRR_OUT_ZERO         =          0;             /*!< VRR_VOLT x0 */
const int16_t VRR_VOLT_HALF        =       1638;             /*!< VRR_VOLT x0.5 */
const int16_t VRR_VOLT             =       3277;             /*!< One volt ((2^15 - 1)/10) */
const int16_t VRR_LIMIT_FREQUENCY  =       4915;             /*!< VRR_VOLT x1.5 */
const int16_t VRR_LIMIT_LOW        =       6553;             /*!< VRR_VOLT x2 */
const int16_t VRR_LIMIT_HIGH       =       9830;             /*!< VRR_VOLT x3 */
const int16_t VRR_OUT_FIVE         =      16384;             /*!< VRR_VOLT x5 */


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

/*! @brief Insert into from one array into another TODO: params + comments
 *
 *  @return void
 */
void VRR_ArrayCopy(int16_t array1[], int16_t array2[], const uint8_t length)
{
  for(uint8_t i = 0; i < length; i++)
  {
    array2[i] = array1[i];
  }
}

/*! @brief Checks if any item in array is true TODO:params + comments
 *
 *  @return bool - TRUE if any item is true
 */
bool VRR_ArrayAnyTrue(const bool data[], const uint8_t length)
{
  for(uint8_t i = 0; i < length; i++)
  {
    if(data[i])
    {
      return true;
    }
  }
  return false;
}

/*! @brief Calculates the square root of a given value efficiently TODO:params + comments
 *
 *  @note Tested speed of Math.h sqrt function - 38us whereas this is 25us. Therefore on average 13us faster
 *  @return uint16 - Square root value
 */
uint32_t VRR_QuickSquareRoot(const uint32_t targetSquare, uint32_t prevRoot, uint32_t prevSquare, const uint32_t lowLimit, const uint32_t highLimit)
{
  if(!prevRoot || !prevSquare)
  {
    prevRoot = (lowLimit + highLimit) / 2;
    prevSquare = prevRoot * prevRoot;
  }

  bool increment = false;
  bool decrement = false;

  while((prevSquare != targetSquare) && !(increment && decrement))
  {
    if(prevSquare < targetSquare)
    {
      prevRoot++;
      increment = true;
    }
    else
    {
      prevRoot--;
      decrement = true;
    }
    prevSquare = prevRoot * prevRoot;
  }

  return prevRoot;
}

/*! @brief Calculates RMS of given values TODO:params + comments
 *
 *  @return uint16_t - RMS value
 */
uint16_t VRR_CalculateRMS(const int16_t data[], const uint8_t length, const uint16_t prevRoot)
{
  int64_t sum = 0;
  for(uint8_t i = 0; i < length; i++)
  {
    sum += (data[i])*(data[i]);
  }
  uint32_t prevSquare = prevRoot*prevRoot;
  return (uint16_t)VRR_QuickSquareRoot((sum/length), prevRoot, prevSquare, VRR_OUT_ZERO, VRR_OUT_FIVE);
}

/*! @brief  TODO:brief + params + comments
 *
 *  @return
 */
void VRR_CheckAlarm(int64_t* timerDelay, int64_t* minDelay, int64_t* timerRate, const uint32_t defaultRate, const uint8_t mode, const uint16_t deviation, bool* alarm, bool* adjustment, OS_ECB* semaphore, const int8_t channel)
{
  if(!(*alarm))
  {
    *alarm = true;
    *timerDelay = PERIOD_TIMER_DELAY;
    *minDelay = PERIOD_MIN_DELAY;
    *timerRate = defaultRate;
    VRR_ProtectedAnalogPut(ANALOG_CHANNEL_3, VRR_OUT_FIVE);
  }
  else if(*timerDelay >= *timerRate || *minDelay >= *timerRate)
  {
    if(mode == TIMING_INVERSE)
    {
      *timerRate = (deviation/(uint16_t)VRR_VOLT_HALF) * defaultRate;
    }
    *timerDelay -= *timerRate;
    *minDelay -= defaultRate;
  }
  else if(!*adjustment)
  {
    *adjustment = true;
    VRR_ProtectedAnalogPut(channel, VRR_OUT_FIVE);
    OS_SemaphoreSignal(semaphore);
  }
}

void VRR_SetFrequency(OS_ECB* frequencyAccess, uint32_t * frequencyPtr, const uint32_t frequency, const uint16_t rms, uint32_t * analogPollPtr)
{
  (void)OS_SemaphoreWait(frequencyAccess, 0);
  OS_DisableInterrupts();

  if(rms < VRR_LIMIT_FREQUENCY)
  {
    *frequencyPtr = 500;
    *analogPollPtr = (1000000000 / (*frequencyPtr/10)) / ANALOG_SAMPLE_SIZE;
    PIT_Set(*analogPollPtr / NB_ANALOG_CHANNELS, true);
  }
  else if(*frequencyPtr != frequency && frequency >= 475 && frequency <= 525)
  {
    *frequencyPtr = frequency;
    *analogPollPtr = (1000000000 / (*frequencyPtr/10)) / ANALOG_SAMPLE_SIZE; // 1 billion
    PIT_Set(*analogPollPtr / NB_ANALOG_CHANNELS, true);
  }

  OS_EnableInterrupts();
  (void)OS_SemaphoreSignal(frequencyAccess);
}

uint32_t VRR_CalculateFrequency(const int16_t sampleData[], const uint16_t rms, const uint32_t analogPoll)
{
  bool negCrossingFound = false;
  uint32_t negCrossing1 = 0;
  uint32_t negCrossing2 = 0;

  for(uint8_t i = 0; i < ANALOG_SAMPLE_SIZE*2 - 1; i++)
  {
    int16_t leftVal = sampleData[i];
    int16_t rightVal = sampleData[i+1];

    if(leftVal >= 0 && 0 > rightVal && !negCrossingFound)
    {
      negCrossingFound = true;
      // m = rise over run, run = 1 sample
      int32_t m = (rightVal - leftVal);
      // x = -b/m
      int32_t x = ((-leftVal) << 10) / m;
      negCrossing1 = (i << 10) + x;
    }
    else if(leftVal >= 0 && 0 > rightVal && negCrossingFound)
    {
      // m = rise over run, run = 1 sample
      int32_t m = (rightVal - leftVal);
      // x = -b/m
      int32_t x = ((-leftVal) << 10) / m;

      negCrossing2 = (i << 10) + x;

      uint32_t distance = negCrossing2 - negCrossing1;
      uint32_t oldPoll = analogPoll >> 10;
      uint64_t newPoll = distance * oldPoll;
      uint64_t frequency = (10000000000 / newPoll); // 10 bill

      return frequency;
    }
  }
  return 0;
}

/* END VRR */
/*!
** @}
*/
