/* ###################################################################
**     Filename    : main.c
**     Project     : Lab2
**     Processor   : MK70FN1M0VMJ12
**     Version     : Driver 01.01
**     Compiler    : GNU C Compiler
**     Date/Time   : 2018-04-12, 13:27, # CodeGen: 0
**     Abstract    :
**         Main module.
**         This module contains user's application code.
**     Settings    :
**     Contents    :
**         No public methods
**     Authors     : 12403756, 12551519
**
** ###################################################################*/
/*!
** @file main.c
** @version 2.0
** @brief
**         Main module.
**         This module contains user's application code.
** @author 12551519
** @date 2018-05-13
*/         
/*!
**  @addtogroup main_module main module documentation
**  @{
*/         
/* MODULE main */

// Included header files
#include "types.h"
#include "Cpu.h"
#include "OS.h"
#include "Events.h"
#include "PE_Types.h"
#include "PE_Error.h"
#include "PE_Const.h"
#include "IO_Map.h"
#include "LEDs.h"
#include "MK70F12.h"
#include "UART.h"
#include "FIFO.h"
#include "packet.h"
#include "Flash.h"
#include "PE_Types.h"
#include "PIT.h"
#include "FTM.h"
#include "kiss_fft.h"
#include "kiss_fftr.h"

// Analog functions
#include "analog.h"

#include <math.h>

#define BAUD_RATE 115200                                     /*!< UART2 Baud Rate */

#define ANALOG_CHANNEL_1 0                                   /*!< 1st analog channel */
#define ANALOG_CHANNEL_2 1                                   /*!< 2nd analog channel */
#define ANALOG_CHANNEL_3 2                                   /*!< 3rd analog channel */

#define NB_ANALOG_CHANNELS 3                                 /*!< Number of analog channels */

#define ANALOG_SAMPLE_SIZE 16                                   /*!< Number of analog samples per cycle */

const uint64_t PERIOD_TIMER_DELAY  = 5000000000;             /*!< Period of analog timer delay (5 seconds) */
const uint64_t PERIOD_MIN_DELAY    = 1000000000;             /*!< Period of analog timer delay minium (1 second) */

const uint8_t COMMAND_TIMING       =       0x10;             /*!< The serial command byte for tower timing */
const uint8_t COMMAND_RAISES       =       0x11;             /*!< The serial command byte for tower raises */
const uint8_t COMMAND_LOWERS       =       0x12;             /*!< The serial command byte for tower lowers */
const uint8_t COMMAND_FREQUENCY    =       0x17;             /*!< The serial command byte for tower frequency */
const uint8_t COMMAND_VOLTAGE      =       0x18;             /*!< The serial command byte for tower voltage */
const uint8_t COMMAND_SPECTRUM     =       0x19;             /*!< The serial command byte for tower spectrum */

const int16_t VRR_ZERO             =          0;             /*!< VRR_VOLT x0 */
const int16_t VRR_VOLT_HALF        =       1638;             /*!< VRR_VOLT x0.5 */
const int16_t VRR_VOLT             =       3277;             /*!< One volt ((2^15 - 1)/10) */
const int16_t VRR_LIMIT_FREQUENCY  =       4915;             /*!< VRR_VOLT x1.5 */
const int16_t VRR_LIMIT_LOW        =       6553;             /*!< VRR_VOLT x2 */
const int16_t VRR_LIMIT_HIGH       =       9830;             /*!< VRR_VOLT x3 */
const int16_t VRR_OUTPUT_5V        =      16384;             /*!< VRR_VOLT x5 */

static uint32_t PeriodAnalogPoll   =    1250000;             /*!< Period of analog polling (16 samples per cycle, 50 Hz default) */
static uint32_t Frequency          =        500;             /*!< The frequency of the VRR */
static uint8_t TimingMode          =          1;             /*!< The timing mode of the VRR */

volatile uint8_t* NvCountRaises;                             /*!< Number of raises pointer to flash */
volatile uint8_t* NvCountLowers;                             /*!< Number of lowers pointer to flash */
volatile uint8_t* NvTimingMode;                              /*!< Timing Mode Setting pointer to flash */

static OS_ECB* LEDOffSemaphore;                              /*!< LED off semaphore for FTM */
static OS_ECB* RaisesSemaphore;                              /*!< Raises semaphore for Flash */
static OS_ECB* LowersSemaphore;                              /*!< Lowers semaphore for Flash */
static OS_ECB* FlashAccessSemaphore;                         /*!< Mutex semaphore for Flash */
static OS_ECB* FrequencySemaphore;                           /*!< Frequency semaphore for channel 1*/

// Thread stacks
OS_THREAD_STACK(InitModulesThreadStack, THREAD_STACK_SIZE);  /*!< The stack for the Tower Init thread. */
OS_THREAD_STACK(FTMLEDsOffThreadStack, THREAD_STACK_SIZE);   /*!< The stack for the FTM thread. */
OS_THREAD_STACK(PacketThreadStack, THREAD_STACK_SIZE);       /*!< The stack for the Packet thread. TODO:size */
OS_THREAD_STACK(RaisesThreadStack, THREAD_STACK_SIZE);       /*!< The stack for the Flash Raises thread. */
OS_THREAD_STACK(LowersThreadStack, THREAD_STACK_SIZE);       /*!< The stack for the Flash Lowers thread. */
OS_THREAD_STACK(FrequencyThreadStack, THREAD_STACK_SIZE);    /*!< The stack for the frequency thread. */

static uint32_t RMSThreadStacks[NB_ANALOG_CHANNELS][THREAD_STACK_SIZE] __attribute__ ((aligned(0x08)));

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
  uint16 currentRMS;
  uint16 currentRMSSquared;
} TAnalogThreadData;

/*! @brief Analog thread configuration data
 *
 */
static TAnalogThreadData AnalogThreadData[NB_ANALOG_CHANNELS] =
{
  {
    .semaphoreRMS = NULL,
    .sampleDataIndex = 0,
    .channelNb = ANALOG_CHANNEL_1
  },
  {
    .semaphoreRMS = NULL,
    .sampleDataIndex = 0,
    .channelNb = ANALOG_CHANNEL_2
  },
  {
    .semaphoreRMS = NULL,
    .sampleDataIndex = 0,
    .channelNb = ANALOG_CHANNEL_3
  }
};

/*! @brief Sends a value to an analog input channel.
 *
 *  @param channelNb is the number of the analog output channel to send the value to.
 *  @param value is the value to write to the analog channel.
 *  @return bool - true if the analog value was output successfully.
 */
bool ProtectedAnalogPut(uint8_t const channelNb, int16_t const value)
{
  OS_DisableInterrupts();
  Analog_Put(channelNb, value);
  OS_EnableInterrupts();
}

/*! @brief Insert into from one array into another TODO: params + comments
 *
 *  @return void
 */
void ArrayCopy(int16_t array1[], int16_t array2[], const uint8_t length)
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
bool ArrayAnyTrue(const bool data[], const uint8_t length)
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
uint32_t QuickSquareRoot(const uint32_t targetSquare, uint32_t prevRoot, uint32_t prevSquare, const uint32_t lowLimit, const uint32_t highLimit)
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
uint16_t CalculateRMS(const int16_t data[], const uint8_t length, const uint16_t prevRoot)
{
  int64_t sum = 0;
  for(uint8_t i = 0; i < length; i++)
  {
    sum += (data[i])*(data[i]);
  }

  //return (uint16_t)sqrt( sum / length );
  uint32_t prevSquare = prevRoot*prevRoot;
  return (uint16_t)QuickSquareRoot((sum/length), prevRoot, prevSquare, VRR_ZERO, VRR_OUTPUT_5V);


}

/*! @brief  TODO:brief + params + comments
 *
 *  @return
 */
void CheckRMS(int64_t* timerDelay, int64_t* minDelay, int64_t* timerRate, const uint16_t deviation, bool* alarm, bool* adjustment, OS_ECB* semaphore, const int8_t channel)
{
  if(!(*alarm))
  {
    *alarm = true;
    *timerDelay = PERIOD_TIMER_DELAY;
    *minDelay = PERIOD_MIN_DELAY;
    *timerRate = PeriodAnalogPoll;
    ProtectedAnalogPut(ANALOG_CHANNEL_3, VRR_OUTPUT_5V);
  }
  else if(*timerDelay >= *timerRate || *minDelay >= *timerRate)
  {
    if(TimingMode == TIMING_INVERSE)
    {
      *timerRate = (deviation/(uint16_t)VRR_VOLT_HALF) * PeriodAnalogPoll;
    }
    *timerDelay -= *timerRate;
    *minDelay -= PeriodAnalogPoll;
  }
  else if(!*adjustment)
  {
    *adjustment = true;
    ProtectedAnalogPut(channel, VRR_OUTPUT_5V);
    OS_SemaphoreSignal(semaphore);
  }
}

/*! @brief Sets the tower timing mode or sends the packet to the PC
 *
 *  @return bool - TRUE if packet is sent or timing mode is set
 */
static bool HandleTowerTiming(void)
{
  // Check if parameters match timing GET, DEFINITE or INVERSE parameters
  if(Packet_Parameter1 == TIMING_GET && Packet_Parameter2 == 0 && Packet_Parameter3 == 0)
  {
    // Sends the timing mode packet
    return Packet_Put(COMMAND_TIMING,TIMING_GET,TimingMode,0);
  }
  else if((Packet_Parameter1 == TIMING_DEFINITE || Packet_Parameter1 == TIMING_INVERSE) && Packet_Parameter2 == 0 && Packet_Parameter3 == 0)
  {
    // Sets the timing mode
    TimingMode = Packet_Parameter1;
    return Flash_Write((uint32_t*)NvTimingMode,Packet_Parameter1,8);
  }
  return false;
}

/*! @brief Resets the number of raises or sends the number to the PC
 *
 *  @return bool - TRUE if packet is sent or number of raises is reset
 */
static bool HandleTowerRaises(void)
{
  // Check if parameters match tower raises GET or RESET parameters
  if(Packet_Parameter1 == DEVIATION_GET && Packet_Parameter2 == 0 && Packet_Parameter3 == 0)
  {
    // Sends the number of raises packet
    return Packet_Put(COMMAND_RAISES,DEVIATION_GET,Flash_Read8(NvCountRaises),0);
  }
  else if(Packet_Parameter1 == DEVIATION_RESET && Packet_Parameter2 == 0 && Packet_Parameter3 == 0)
  {
    // Resets the number of raises to flash
    return Flash_Write((uint32_t*)NvCountRaises,0,8);
  }
  return false;
}

/*! @brief Resets the number of lowers or sends the number to the PC
 *
 *  @return bool - TRUE if packet is sent or number of lowers is reset
 */
static bool HandleTowerLowers(void)
{
  // Check if parameters match tower raises GET or RESET parameters
  if(Packet_Parameter1 == DEVIATION_GET && Packet_Parameter2 == 0 && Packet_Parameter3 == 0)
  {
    // Sends the number of raises packet
    return Packet_Put(COMMAND_LOWERS,DEVIATION_GET,Flash_Read8(NvCountLowers),0);
  }
  else if(Packet_Parameter1 == DEVIATION_RESET && Packet_Parameter2 == 0 && Packet_Parameter3 == 0)
  {
    // Resets the number of lowers to flash
    return Flash_Write((uint32_t*)NvCountLowers,0,8);
  }
  return false;
}

/*! @brief Sends the tower frequency packet for phase A
 *
 *  @return bool - TRUE if frequency packet is sent
 */
static bool HandleTowerFrequency(void)
{
  int16union_t frequency;

  OS_DisableInterrupts();
  frequency.l = (int16_t)Frequency;
  OS_EnableInterrupts();

  // Sends the frequency packet
  return Packet_Put(COMMAND_FREQUENCY,frequency.s.Lo,frequency.s.Hi,0);
}

/*! @brief Sends the tower voltage packet for a selected phase
 *
 *  @return bool - TRUE if voltage packet is sent
 */
static bool HandleTowerVoltage(void)
{
  if(Packet_Parameter1 > ANALOG_CHANNEL_1 && Packet_Parameter1 <= NB_ANALOG_CHANNELS && Packet_Parameter2 == 0 && Packet_Parameter3 == 0)
  {
    int16union_t rms;
    TAnalogThreadData* phase = &AnalogThreadData[Packet_Parameter1-1];

    // Calculates the RMS based on received phase
    OS_DisableInterrupts();
    rms.l = phase->currentRMS;
    OS_EnableInterrupts();

    // Sends the RMS voltage packet
    return Packet_Put(COMMAND_VOLTAGE,Packet_Parameter1,rms.s.Lo,rms.s.Hi);
  }
  return false;
}

/*! @brief Sends the tower spectrum packet for a selected harmonic
 *
 *  @note Tested speed of Math.h sqrt and float FFT speed - 242us whereas this is 26us. Therefore on average 216us faster
 *  @return bool - TRUE if spectrum packet is sent
 */
static bool HandleTowerSpectrum(void)
{
  // Check if harmonic number is between 0 and 7
  if(Packet_Parameter1 >= 0 && Packet_Parameter1 <= 7)
  {
    uint8_t memory[500];
    size_t length = 500;
    kiss_fftr_cfg config = kiss_fftr_alloc(ANALOG_SAMPLE_SIZE, 0, memory, &length);

    TAnalogThreadData* channelData = &AnalogThreadData[ANALOG_CHANNEL_1];

    kiss_fft_scalar timedata[ANALOG_SAMPLE_SIZE];

    OS_DisableInterrupts();
    for(uint8_t i = 0; i < ANALOG_SAMPLE_SIZE; i++)
    {
      timedata[i] = channelData->prevSampleData[i];
    }
    OS_EnableInterrupts();

    kiss_fft_cpx spectrum[(ANALOG_SAMPLE_SIZE/2)+1];

    kiss_fftr(config, timedata, spectrum);

    static uint32_t prevRoots[7];
    uint32_t prevRoot = prevRoots[Packet_Parameter1];
    uint32_t prevSquare = prevRoot * prevRoot;

    int64_t real = spectrum[Packet_Parameter1].r / (ANALOG_SAMPLE_SIZE/2);
    int64_t imaginary = spectrum[Packet_Parameter1].i / (ANALOG_SAMPLE_SIZE/2);
    int64_t targetSquare = (real * real) + (imaginary * imaginary);

    prevRoots[Packet_Parameter1] = QuickSquareRoot(targetSquare,prevRoot,prevSquare,VRR_LIMIT_LOW,VRR_OUTPUT_5V);

    uint16union_t magnitude;
    magnitude.l = prevRoots[Packet_Parameter1];

    return Packet_Put(COMMAND_SPECTRUM,Packet_Parameter1,magnitude.s.Lo,magnitude.s.Hi);
  }
  return false;
}

/*! @brief Executes the command depending on what packet has been received
 *
 *  @return void
 */
void ReceivedPacket(void)
{
  bool success = false;                                         /*!< The success status of the received packet */
  uint8_t commandIgnoreAck = Packet_Command & ~PACKET_ACK_MASK; /*!< The command byte ignoring the ACK mask */
  uint8_t commandAck = Packet_Command & PACKET_ACK_MASK;        /*!< The command byte with the ACK mask */

  // AND the packet command byte with the bitwise inverse ACK MASK to ignore if ACK is requested
  if(commandIgnoreAck == COMMAND_TIMING)
  {
    // Check if parameters match tower timing GET, DEFINITE or INVERSE parameters
    success = HandleTowerTiming();
  }
  else if(commandIgnoreAck == COMMAND_RAISES)
  {
    // Check if parameters match number of raises GET or RESET parameters
    success = HandleTowerRaises();
  }
  else if(commandIgnoreAck == COMMAND_LOWERS)
  {
    // Check if parameters match number of lowers GET or RESET parameters
    success = HandleTowerLowers();
  }
  else if(commandIgnoreAck == COMMAND_FREQUENCY)
  {
    // Send the tower frequency packet
    success = HandleTowerFrequency();
  }
  else if(commandIgnoreAck == COMMAND_VOLTAGE)
  {
    // Send the tower voltage packet for phase A, B or C
    success = HandleTowerVoltage();
  }
  else if(commandIgnoreAck == COMMAND_SPECTRUM)
  {
    // Send the tower spectrum packet
    success = HandleTowerSpectrum();
  }

  // AND the packet command byte with the ACK MASK to check if ACK is requested
  if(commandAck)
  {
    // Check the success status of the packet which was sent
    if(success == false)
    {
      // Return the sent packet with the NACK command if unsuccessful
      Packet_Put(commandIgnoreAck,Packet_Parameter1,Packet_Parameter2,Packet_Parameter3);
    }
    else
    {
      // Return the sent packet with the ACK command if successful
      Packet_Put(Packet_Command,Packet_Parameter1,Packet_Parameter2,Packet_Parameter3);
    }
  }

  // Reset the packet variables to 0
  Packet_Command    = 0;
  Packet_Parameter1 = 0;
  Packet_Parameter2 = 0;
  Packet_Parameter3 = 0;
  Packet_Checksum   = 0;
}

/*! @brief Reads analog data every time it is called
 *
 *  @return void
 */
void PITCallback(void* arg)
{
  static uint8_t channel;

  int16_t analogInputValue;

  TAnalogThreadData* channelData = &AnalogThreadData[channel];

  Analog_Get(channelData->channelNb, &analogInputValue);

  channelData->sampleData[channelData->sampleDataIndex] = analogInputValue;
  channelData->sampleDataIndex++;
  if(channelData->sampleDataIndex >= ANALOG_SAMPLE_SIZE)
  {
    channelData->sampleDataIndex = 0;
  }

  (void)OS_SemaphoreSignal(AnalogThreadData[channel].semaphoreRMS);

  channel++;
  if(channel >= NB_ANALOG_CHANNELS)
  {
    channel = 0;
  }

  if(channel == 0 && channelData->sampleDataIndex == 0)
  {
    (void)OS_SemaphoreSignal(FrequencySemaphore);
  }
}

/*! @brief Initializes the main tower components by calling the initialization routines of the supporting software modules.
 *
 *  @return void
 */
bool TowerInit(void)
{
  // Success status of writing default values to Flash and FTM
  bool success = false;

  if(Flash_Init(FlashAccessSemaphore)
  && LEDs_Init()
  && Packet_Init(BAUD_RATE, CPU_BUS_CLK_HZ)
  && FTM_Init()
  && PIT_Init(CPU_BUS_CLK_HZ, PITCallback, NULL))
  {
    success = true;

    // Allocates addresses in Flash memory
    if(Flash_AllocateVar((volatile void**)&NvCountRaises, sizeof(*NvCountRaises))
    && Flash_AllocateVar((volatile void**)&NvCountLowers, sizeof(*NvCountLowers))
    && Flash_AllocateVar((volatile void**)&NvTimingMode, sizeof(*NvTimingMode)));
    {
      // Checks if timing mode is clear
      if(Flash_Read8(NvTimingMode) == 0xFF)
      {
        // Sets the tower mode to the default number
        if(!Flash_Write((uint32_t*)NvTimingMode,(uint8_t)TIMING_DEFINITE,8))
        {
          success = false;
        }
      }
      // Checks if number of raises is clear
      if(Flash_Read8(NvCountRaises) == 0xFF)
      {
        // Sets the number of raises to the default number
        if(!Flash_Write((uint32_t*)NvCountRaises,(uint8_t)0,8))
        {
          success = false;
        }
      }
      // Checks if number of lowers is clear
      if(Flash_Read8(NvCountLowers) == 0xFF)
      {
        // Sets the number of lowers to the default number
        if(!Flash_Write((uint32_t*)NvCountLowers,(uint8_t)0,8))
        {
          success = false;
        }
      }
    }
  }
  return success;
}

/*! @brief Initializes the modules
 *
 *  @param pData is not used but is required by the OS to create a thread.
 *  @note This thread deletes itself after running for the first time.
 */
static void InitModulesThread(void* pData)
{
  OS_DisableInterrupts();

  // Initialize the analog library
  (void)Analog_Init(CPU_BUS_CLK_HZ);

  // Generate the global analog semaphores
  for (uint8_t analogNb = ANALOG_CHANNEL_1; analogNb < NB_ANALOG_CHANNELS; analogNb++)
  {
    AnalogThreadData[analogNb].semaphoreRMS = OS_SemaphoreCreate(0);
  }

  // Create semaphores for threads
  FlashAccessSemaphore = OS_SemaphoreCreate(1);
  LEDOffSemaphore      = OS_SemaphoreCreate(0);
  RaisesSemaphore      = OS_SemaphoreCreate(0);
  LowersSemaphore      = OS_SemaphoreCreate(0);
  FrequencySemaphore   = OS_SemaphoreCreate(0);

  // Initializes the main tower components and sets the default or stored values
  if(TowerInit())
  {
    // Turn on the orange LED to indicate the tower has initialized successfully
    LEDs_On(LED_ORANGE);
  }
  
  // Set the PIT with the analog polling period
  PIT_Set(PeriodAnalogPoll / NB_ANALOG_CHANNELS, true);

  // Put initial zero values to tower output channels
  ProtectedAnalogPut(ANALOG_CHANNEL_1, VRR_ZERO);
  ProtectedAnalogPut(ANALOG_CHANNEL_2, VRR_ZERO);
  ProtectedAnalogPut(ANALOG_CHANNEL_3, VRR_ZERO);

  OS_EnableInterrupts();

  // We only do this once - therefore delete this thread
  OS_ThreadDelete(OS_PRIORITY_SELF);
}

/*! @brief Checks if any packets have been received then handles it based on its contents
 *
 *  @param pData is not used but is required by the OS to create a thread.
 *  @note Assumes that Packet_Init has been called successfully.
 */
static void PacketThread(void* pData)
{
  TFTMChannel receivedPacketTmr;          /*!< FTM Channel for received packet timer */
  receivedPacketTmr.channelNb             = 0;
  receivedPacketTmr.delayCount            = CPU_MCGFF_CLK_HZ_CONFIG_0;
  receivedPacketTmr.ioType.inputDetection = TIMER_INPUT_ANY;
  receivedPacketTmr.ioType.outputAction   = TIMER_OUTPUT_DISCONNECT;
  receivedPacketTmr.timerFunction         = TIMER_FUNCTION_OUTPUT_COMPARE;
  receivedPacketTmr.userSemaphore         = LEDOffSemaphore;

  // Set FTM Channel for received packet timer
  FTM_Set(&receivedPacketTmr);

  for (;;)
  {
    // Check if a packet has been received
    if(Packet_Get())
    {
      // Turn on the blue LED if the 1 second timer is set
      if(FTM_StartTimer(&receivedPacketTmr))
      {
        LEDs_On(LED_BLUE);
      }

      // Execute a command depending on what packet has been received
      ReceivedPacket();
    }
  }
}

/*! @brief Turns the Blue LED off after the timer is complete
 *
 *  @param pData is not used but is required by the OS to create a thread.
 *  @note Assumes that FTM_Init has been called successfully.
 */
static void FTMLEDsOffThread(void* pData)
{
  for (;;)
  {
      // Wait until signaled to turn LED off
     OS_SemaphoreWait(LEDOffSemaphore,0);

     // Turn off blue LED
     LEDs_Off(LED_BLUE);
  }
}

/*! @brief Calculates the RMS value on an ADC channel. TODO:comments
 *
 */
void CalculateRMSThread(void* pData)
{
  // Make the code easier to read by giving a name to the typecast'ed pointer
  #define analogData ((TAnalogThreadData*)pData)

  static bool allAlarms[NB_ANALOG_CHANNELS];
  static bool adjustment;

  int64_t timerDelay;
  int64_t minDelay;
  int64_t timerRate;
  bool alarm;

  for (;;)
  {
    (void)OS_SemaphoreWait(analogData->semaphoreRMS, 0);

    uint16_t prevRoot = analogData->currentRMS;

    uint16_t rms = CalculateRMS(analogData->sampleData, ANALOG_SAMPLE_SIZE, prevRoot);
    analogData->currentRMS = rms;

    if(rms < VRR_LIMIT_LOW)
    {
      CheckRMS(&timerDelay, &minDelay, &timerRate, (VRR_LIMIT_LOW-rms), &alarm, &adjustment, RaisesSemaphore, ANALOG_CHANNEL_1);
    }
    else if(rms > VRR_LIMIT_HIGH)
    {
      CheckRMS(&timerDelay, &minDelay, &timerRate, (rms-VRR_LIMIT_HIGH), &alarm, &adjustment, LowersSemaphore, ANALOG_CHANNEL_2);
    }
    else
    {
      alarm = false;
    }

    allAlarms[analogData->channelNb] = alarm;
    if(!ArrayAnyTrue(allAlarms, NB_ANALOG_CHANNELS))
    {
      adjustment = false;
      ProtectedAnalogPut(ANALOG_CHANNEL_1, VRR_ZERO);
      ProtectedAnalogPut(ANALOG_CHANNEL_2, VRR_ZERO);
      ProtectedAnalogPut(ANALOG_CHANNEL_3, VRR_ZERO);
    }
  }
}

/*! @brief Calculates the Frequency value on an ADC channel. TODO:comments
 *
 */
void CalculateFrequencyThread(void* pData)
{
  for (;;)
  {
    (void)OS_SemaphoreWait(FrequencySemaphore, 0);

    OS_DisableInterrupts(); // TODO: remove interrupts disable

    bool negCrossingFound = false;
    uint32_t negCrossing1 = 0;
    uint32_t negCrossing2 = 0;
    TAnalogThreadData* channelData = &AnalogThreadData[ANALOG_CHANNEL_1];

    if(channelData->currentRMS > VRR_LIMIT_FREQUENCY)
    {
      for(uint8_t i = 0; i < ANALOG_SAMPLE_SIZE*2 - 1; i++)
      {
        int16_t leftVal;
        int16_t rightVal;

        if(i < ANALOG_SAMPLE_SIZE)
        {
          leftVal = channelData->prevSampleData[i];
        }
        else
        {
          leftVal = channelData->sampleData[i - ANALOG_SAMPLE_SIZE];
        }

        if(i+1 < ANALOG_SAMPLE_SIZE)
        {
          rightVal = channelData->prevSampleData[i+1];
        }
        else
        {
          rightVal = channelData->sampleData[i+1 - ANALOG_SAMPLE_SIZE];
        }

        if(leftVal >= 0 && 0 > rightVal && negCrossingFound)
        {
          // m = rise over run, run = 1 sample
          int32_t m = (rightVal - leftVal);
          // x = -b/m
          int32_t x = ((-leftVal) << 10) / m;
          negCrossing2 = (i << 10) + x;

          uint32_t distance = negCrossing2 - negCrossing1;

          uint32_t oldPoll = PeriodAnalogPoll >> 10;

          uint64_t newPoll = distance * oldPoll;

          uint64_t frequency = (10000000000 / newPoll); // 10 bill

          if(Frequency != frequency && frequency >= 475 && frequency <= 525)
          {
            Frequency = frequency;
            PeriodAnalogPoll = (1000000000 / (Frequency/10)) / ANALOG_SAMPLE_SIZE; // 1 billion
            PIT_Set(PeriodAnalogPoll / NB_ANALOG_CHANNELS, true);
          }

          break;
        }
        else if(leftVal >= 0 && 0 > rightVal && !negCrossingFound)
        {
          negCrossingFound = true;

          // m = rise over run, run = 1 sample
          int32_t m = (rightVal - leftVal);
          // x = -b/m
          int32_t x = ((-leftVal) << 10) / m;
          negCrossing1 = (i << 10) + x;
        }
      }

      ArrayCopy(channelData->sampleData, channelData->prevSampleData, ANALOG_SAMPLE_SIZE);

      OS_EnableInterrupts();
    }
    else
    {
      OS_DisableInterrupts();
      Frequency = 500;
      PeriodAnalogPoll = (1000000000 / (Frequency/10)) / ANALOG_SAMPLE_SIZE;
      PIT_Set(PeriodAnalogPoll / NB_ANALOG_CHANNELS, true);
      OS_EnableInterrupts();
    }
  }
}

/*! @brief Increments the raises count and writes to Flash.
 *
 */
void RaisesThread(void* pData)
{
  for (;;)
  {
    (void)OS_SemaphoreWait(RaisesSemaphore, 0);
    Flash_Write((uint32_t*)NvCountRaises,Flash_Read8(NvCountRaises)+1,8);
  }
}

/*! @brief Increments the lowers count and writes to Flash.
 *
 */
void LowersThread(void* pData)
{
  for (;;)
  {
    (void)OS_SemaphoreWait(LowersSemaphore, 0);
    Flash_Write((uint32_t*)NvCountLowers,Flash_Read8(NvCountLowers)+1,8);
  }
}

/*lint -save  -e970 Disable MISRA rule (6.3) checking. */
int main(void)
/*lint -restore Enable MISRA rule (6.3) checking. */
{
  /* Write your local variable definition here */
  OS_ERROR error;

  /*** Processor Expert internal initialization. DON'T REMOVE THIS CODE!!! ***/
  PE_low_level_init();
  /*** End of Processor Expert internal initialization.                    ***/

  // Initialize the RTOS
  OS_Init(CPU_CORE_CLK_HZ, false);

  // Highest priority
  error = OS_ThreadCreate(InitModulesThread,
                          NULL,
                          &InitModulesThreadStack[THREAD_STACK_SIZE - 1],
  		                    0);

  // Create threads for analog RMS calculations
  for (uint8_t threadNb = ANALOG_CHANNEL_1; threadNb < NB_ANALOG_CHANNELS; threadNb++)
  {
    error = OS_ThreadCreate(CalculateRMSThread,
                            &AnalogThreadData[threadNb],
                            &RMSThreadStacks[threadNb][THREAD_STACK_SIZE - 1],
                            3+threadNb);
  }

  // 6th Highest priority
  error = OS_ThreadCreate(CalculateFrequencyThread,
                          NULL,
                          &FrequencyThreadStack[THREAD_STACK_SIZE - 1],
                          6);
  // 7th Highest priority
  error = OS_ThreadCreate(PacketThread,
                          NULL,
                          &PacketThreadStack[THREAD_STACK_SIZE - 1],
                          7);
  // 8th Highest priority
  error = OS_ThreadCreate(FTMLEDsOffThread,
                          NULL,
                          &FTMLEDsOffThreadStack[THREAD_STACK_SIZE - 1],
                          8);
  // 9th Highest priority
  error = OS_ThreadCreate(RaisesThread,
                          NULL,
                          &RaisesThreadStack[THREAD_STACK_SIZE - 1],
                          9);
  // 10th Highest priority
  error = OS_ThreadCreate(LowersThread,
                          NULL,
                          &LowersThreadStack[THREAD_STACK_SIZE - 1],
                          10);

  // Start multithreading - never returns!
  OS_Start();
}

/* END main */
/*!
** @}
*/
/*
** ###################################################################
**
**     This file was created by Processor Expert 10.5 [05.21]
**     for the Freescale Kinetis series of microcontrollers.
**
** ###################################################################
*/
