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
** @author 12403756, 12551519
** @date 2018-04-13
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
#include "RTC.h"
#include "accel.h"
#include "I2C.h"
#include "median.h"
#include "analog.h"
#include "math.h"

#define BAUD_RATE 115200                        /*!< UART2 Baud Rate */

#define ADC_SAMPLES_PER_CYCLE 16                /*!< ADC samples per cycle */

#define ADC_BUFFER_SIZE 64                     // Note: Must be an integral number of ADC_SAMPLES_PER_CYCLE

#define NB_ANALOG_CHANNELS 3

#define ADC_DEFAULT_FREQUENCY 50

#define RMS_UPPER_LIMIT    9830
#define RMS_LOWER_LIMIT    6554
#define RMS_FREQUENCY_MIN  4915
#define DAC_5V_OUT         16384
#define DAC_0V_OUT          0

typedef struct
{
  int16_t ADC_Data[ADC_BUFFER_SIZE];
  uint8_t LatestData;
}TVoltageData;

typedef enum
{
  TIMING_GET,
  TIMING_DEFINITE,
  TIMING_INVERSE
}TTimingMode;
/*! @brief Data structure used to pass Analog configuration to a user thread
 *
 */
typedef struct AnalogThreadData
{
  OS_ECB* semaphore;
  uint8_t channelNb;
} TAnalogThreadData;

/*! @brief Analog thread configuration data
 *
 */
static TAnalogThreadData AnalogThreadData[NB_ANALOG_CHANNELS] =
{
  {
    .semaphore = NULL,
    .channelNb = 0
  },
  {
    .semaphore = NULL,
    .channelNb = 1
  }
};

const uint32_t PERIOD_I2C_POLL    = 1000000000; /*!< Period of the I2C polling in polling mode */

const uint8_t COMMAND_STARTUP     = 0x04;       /*!< The serial command byte for tower startup */
const uint8_t COMMAND_VER         = 0x09;       /*!< The serial command byte for tower version */
const uint8_t COMMAND_NUM         = 0x0B;       /*!< The serial command byte for tower number */
const uint8_t COMMAND_PROGRAMBYTE = 0x07;       /*!< The serial command byte for tower program byte */
const uint8_t COMMAND_READBYTE    = 0x08;       /*!< The serial command byte for tower read byte */
const uint8_t COMMAND_MODE        = 0x0D;       /*!< The serial command byte for tower mode */
const uint8_t COMMAND_TIME        = 0x0C;       /*!< The serial command byte for tower time */
const uint8_t COMMAND_ACCEL       = 0x10;       /*!< The serial command byte for tower accelerometer */
const uint8_t COMMAND_TIMINGMODE  = 0x10;       /*!< The serial command byte for tower timing mode */
const uint8_t COMMAND_RAISES      = 0x11;       /*!< The serial command byte for tower raises */
const uint8_t COMMAND_LOWERS      = 0x12;       /*!< The serial command byte for tower lowers */
const uint8_t COMMAND_FREQUENCY   = 0x17;       /*!< The serial command byte for tower frequency */
const uint8_t COMMAND_VOLTAGE     = 0x18;       /*!< The serial command byte for tower voltage */
const uint8_t COMMAND_SPECTRUM    = 0x19;       /*!< The serial command byte for tower spectrum */

// ----------------------------------------
// Thread priorities
// 0 = highest priority
// ----------------------------------------
const uint8_t ANALOG_THREAD_PRIORITIES[NB_ANALOG_CHANNELS] = {7, 8, 9};

const uint8_t PARAM_GET           = 1;          /*!< Get bit of packet parameter 1 */
const uint8_t PARAM_SET           = 2;          /*!< Set bit of packet parameter 1 */

const uint8_t TOWER_VER_MAJ       = 1;          /*!< Tower major version */
const uint8_t TOWER_VER_MIN       = 0;          /*!< Tower minor version */

volatile uint16union_t* NvTowerNb;              /*!< Tower number union pointer to flash */
volatile uint16union_t* NvTowerMd;              /*!< Tower mode union pointer to flash */
volatile uint8_t* NvNbRaises;                   /*!< Raise count byte pointer to flash */
volatile uint8_t* NvNbLowers;                   /*!< Lower count byte pointer to flash */
volatile uint8_t* NvTimingMd;                   /*!< Timing Mode byte pointer to flash */



static TVoltageData VoltageSamples[NB_ANALOG_CHANNELS];
static uint16_t RMS[NB_ANALOG_CHANNELS];
static float Frequency;
static uint16_t TimingMode;
static uint32_t SamplePeriod;
static uint32_t NewSamplePeriod;
static bool Alarm[NB_ANALOG_CHANNELS];
static bool Adjusting[NB_ANALOG_CHANNELS];
static uint64_t LastSumOfSquares[NB_ANALOG_CHANNELS];
static int16_t OldestData[NB_ANALOG_CHANNELS];


static OS_ECB* LEDOffSemaphore;                 /*!< LED off semaphore for FTM */
static OS_ECB* RTCReadSemaphore;                /*!< Read semaphore for RTC */
static OS_ECB* OutOfRangeSemaphore;             /*!< Out of range semaphore for RMS  */
static OS_ECB* WithinRangeSemaphore;            /*!< Within range semaphore for RMS  */
static OS_ECB* NewADCDataSemaphore;             /*!< New ADC data semaphore for ADC Process thread */
static OS_ECB* FrequencyCalculateSemaphore;     /*!< Frequency Calculate semaphore */
static OS_ECB* FrequencyTrackSemaphore;         /*!< Frequency track semaphore */
static OS_ECB* LogRaisesSemaphore;              /*!< Log Raises semaphore */
static OS_ECB* LogLowersSemaphore;              /*!< Log Lowers semaphore */
static OS_ECB* FlashAccessMutex;                /*!< Flash Access Mutex */

// Thread stacks
OS_THREAD_STACK(InitModulesThreadStack, THREAD_STACK_SIZE);       /*!< The stack for the Tower Init thread. */
OS_THREAD_STACK(RTCThreadStack, THREAD_STACK_SIZE);               /*!< The stack for the RTC thread. */
OS_THREAD_STACK(FTMLEDsOffThreadStack, THREAD_STACK_SIZE);        /*!< The stack for the FTM thread. */
OS_THREAD_STACK(ADCDataProcessThreadStack, THREAD_STACK_SIZE);    /*!< The stack for the ADCDataProcess thread. */
OS_THREAD_STACK(FrequencyCalculateThreadStack, THREAD_STACK_SIZE);/*!< The stack for the FrequencyCalculate thread. */
OS_THREAD_STACK(FrequencyTrackThreadStack, THREAD_STACK_SIZE);    /*!< The stack for the FrequencyTrack thread. */
OS_THREAD_STACK(PacketThreadStack, THREAD_STACK_SIZE);            /*!< The stack for the Packet thread. */
OS_THREAD_STACK(LogRaisesThreadStack, THREAD_STACK_SIZE);         /*!< The stack for the Log Raises thread. */
OS_THREAD_STACK(LogLowersThreadStack, THREAD_STACK_SIZE);         /*!< The stack for the Log Lowers thread. */
static uint32_t RMSThreadStacks[NB_ANALOG_CHANNELS][THREAD_STACK_SIZE] __attribute__ ((aligned(0x08)));

/*! @brief Sends the startup packets to the PC
 *
 *  @return bool - TRUE if all packets are sent
 */
bool HandleTowerStartup(void)
{
  // Sends the tower startup values, version, number and mode to the PC
  return (
    Packet_Put(COMMAND_STARTUP,0x00,0x00,0x00) &&
    Packet_Put(COMMAND_VER,'v',TOWER_VER_MAJ,TOWER_VER_MIN) &&
    Packet_Put(COMMAND_NUM,PARAM_GET,NvTowerNb->s.Lo,NvTowerNb->s.Hi) &&
    Packet_Put(COMMAND_MODE,PARAM_GET,NvTowerMd->s.Lo,NvTowerMd->s.Hi));
}

/*! @brief Sends the tower version packet to the PC
 *
 *  @return bool - TRUE if packet is sent
 */
bool HandleTowerVersion(void)
{
  // Sends the tower number packet
  return Packet_Put(COMMAND_NUM,PARAM_GET,NvTowerNb->s.Lo,NvTowerNb->s.Hi);
}

/*! @brief Sets the tower number or sends the packet to the PC
 *
 *  @return bool - TRUE if packet is sent or number is set
 */
bool HandleTowerNumber(void)
{
  // Check if parameters match tower number GET or SET parameters
  if(Packet_Parameter1 == PARAM_GET && Packet_Parameter2 == 0 && Packet_Parameter3 == 0)
  {
    // Sends the tower number packet
    return Packet_Put(COMMAND_NUM,PARAM_GET,NvTowerNb->s.Lo,NvTowerNb->s.Hi);
  }
  else if(Packet_Parameter1 == PARAM_SET)
  {
    // Sets the tower number
    return Flash_Write16((uint16_t*)NvTowerNb,(uint16_t)Packet_Parameter23);
  }
  return false;
}

/*! @brief Erases or programs a byte to Flash based on packet received
 *
 *  @return bool - TRUE if Flash was modified
 */
bool HandleTowerProgramByte(void)
{
  // Check if offset is erase or set
  if(Packet_Parameter1 == 0x08 && Packet_Parameter2 == 0)
  {
    // Erase Flash
    return Flash_Erase();
  }
  else if(Packet_Parameter1 >= 0x00 && Packet_Parameter1 <= 0x07 && Packet_Parameter2 == 0)
  {
    // Program byte to Flash
    volatile uint8_t* nvAddress = (uint8_t*)(FLASH_DATA_START + Packet_Parameter1);
    return Flash_Write8( (uint8_t*)nvAddress, Packet_Parameter3 );
  }
  return false;
}

/*! @brief Sends the byte read from Flash packet to the PC
 *
 *  @return bool - TRUE if packet is sent
 */
bool HandleTowerReadByte(void)
{
  // Check if offset is within range
  if(Packet_Parameter1 >= 0x00 && Packet_Parameter1 <= 0x07 && Packet_Parameter2 == 0 && Packet_Parameter3 == 0)
  {
    // Send read byte packet
    return Packet_Put(COMMAND_READBYTE,Packet_Parameter1,0,_FB(FLASH_DATA_START + (uint32_t)Packet_Parameter1));
  }
  return false;
}

/*! @brief Sets the tower mode or sends the packet to the PC
 *
 *  @return bool - TRUE if packet is sent or mode is set
 */
bool HandleTowerMode(void)
{
  // Check if parameters match tower mode GET or SET parameters
  if(Packet_Parameter1 == PARAM_GET && Packet_Parameter2 == 0 && Packet_Parameter3 == 0)
  {
    // Sends the tower mode packet
    return Packet_Put(COMMAND_MODE,PARAM_GET,NvTowerMd->s.Lo,NvTowerMd->s.Hi);
  }
  else if (Packet_Parameter1 == PARAM_SET)
  {
    // Sets the tower mode
    return Flash_Write16((uint16_t*)NvTowerMd,(uint16_t)Packet_Parameter23);
  }
  return false;
}

/*! @brief Sets the Real Time Clock time as requested by PC
 *
 *  @return bool - TRUE if Real Clock Time is set successfully
 */
bool HandleTowerSetTime(void)
{
  // Check if parameters are within tower time limits
  if(Packet_Parameter1 < 24 && Packet_Parameter2 < 60 && Packet_Parameter3 < 60)
  {
    // Sets the Real Time Clock
    RTC_Set(Packet_Parameter1,Packet_Parameter2,Packet_Parameter3);
    return true;
  }
  return false;
}

/*! @brief Sets or gets timing mode as requested by PC
 *
 *  @return bool - TRUE if mode is set or sent
 */
bool HandleTowerTimingMode(void)
{
  // Check if parameters are within tower time limits and get or set definite or inverse
  if(Packet_Parameter1 == TIMING_GET && Packet_Parameter2 == 0 && Packet_Parameter3 == 0)
  {
    // Send Timing Mode
    Packet_Put(COMMAND_TIMINGMODE, _FB(NvTimingMd), 0, 0);
    return true;
  }
  else if((Packet_Parameter1 == TIMING_DEFINITE || Packet_Parameter1 == TIMING_INVERSE) && Packet_Parameter2 == 0 && Packet_Parameter3 == 0)
  {
    // Set Timing Mode
    if (Flash_Write8((uint8_t*)NvTimingMd,(uint8_t)Packet_Parameter1))
    {
      OS_DisableInterrupts();
      TimingMode = _FB(NvTimingMd);
      OS_EnableInterrupts();
      return true;
    }
  }
  return false;
}

/*! @brief Sets the number of raises to 0 or sends number of raises to PC
 *
 *  @return bool - TRUE if packet is sent or number is set
 */
bool HandleTowerRaises(void)
{
  // Check if parameters match tower number GET or SET parameters
  if(Packet_Parameter1 == 0 && Packet_Parameter2 == 0 && Packet_Parameter3 == 0)
  {
    // Sends the number of raises
    return Packet_Put(COMMAND_RAISES,_FB(NvNbRaises), 0, 0);
  }
  else if(Packet_Parameter1 == 1 && Packet_Parameter2 == 0 && Packet_Parameter3 == 0)
  {
    // Resets the number of raises
    return Flash_Write8((uint8_t*)NvNbRaises,(uint8_t)0);
  }
  return false;
}

/*! @brief Sets the number of lowers to 0 or sends number of lowers to PC
 *
 *  @return bool - TRUE if packet is sent or number is set
 */
bool HandleTowerLowers(void)
{
  // Check if parameters match tower number GET or SET parameters
  if(Packet_Parameter1 == 0 && Packet_Parameter2 == 0 && Packet_Parameter3 == 0)
  {
    // Sends the number of lowers
    return Packet_Put(COMMAND_LOWERS,_FB(NvNbLowers), 0, 0);
  }
  else if(Packet_Parameter1 == 1 && Packet_Parameter2 == 0 && Packet_Parameter3 == 0)
  {
    // Resets the number of lowers
    return Flash_Write8((uint8_t*)NvNbLowers,(uint8_t)0);
  }
  return false;
}

/*! @brief Sends the frequency to the PC
 *
 *  @return bool - TRUE if packet sent
 */
bool HandleTowerFrequency(void)
{
  // Local storage of variable as union
  uint16union_t frequency;

  // Check if parameters are within limits
  if(Packet_Parameter1 == 0 && Packet_Parameter2 == 0 && Packet_Parameter3 == 0)
  {
    // Disable interrupts to read variable.
    OS_DisableInterrupts();
    float localFrequency = Frequency;
    OS_EnableInterrupts();

    // Round to the nearest 0.1Hz
    float roundedFrequencyBy10 = roundf((localFrequency * (float)10));

    // Type-cast  as a uint16 to be sent to PC.
    frequency.l = (uint16_t)roundedFrequencyBy10;

    // Send frequency to PC
    return Packet_Put(COMMAND_FREQUENCY, frequency.s.Hi, frequency.s.Lo, 0);
  }
  return false;
}

/*! @brief Sends the frequency to the PC
 *
 *  @return bool - TRUE if packet sent
 */
bool HandleTowerVoltage(void)
{
  // Local storage of variable as union
  uint16union_t rms;

  // Check if parameters are within limits
  if(Packet_Parameter1 > 0 && Packet_Parameter1 <= NB_ANALOG_CHANNELS && Packet_Parameter2 == 0 && Packet_Parameter3 == 0)
  {
    // Disable interrupts to read variable.
    OS_DisableInterrupts();

    rms.l = RMS[Packet_Parameter1 -1];

    OS_EnableInterrupts();

    // Send Voltage to PC
    return Packet_Put(COMMAND_VOLTAGE, Packet_Parameter1, rms.s.Hi, rms.s.Lo);
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
  if(commandIgnoreAck == COMMAND_STARTUP && Packet_Parameter1 == 0 && Packet_Parameter2 == 0 && Packet_Parameter3 == 0)
  {
    // Send tower startup packets
    success = HandleTowerStartup();
  }
  else if(commandIgnoreAck == COMMAND_VER && Packet_Parameter1 == 'v' && Packet_Parameter2 == 'x' && Packet_Parameter3 == 13)
  {
    // Send tower version packet
    success = HandleTowerVersion();
  }
  else if(commandIgnoreAck == COMMAND_NUM)
  {
    // Check if parameters match tower number GET or SET parameters
    success = HandleTowerNumber();
  }
  else if(commandIgnoreAck == COMMAND_MODE)
  {
    // Check if parameters match tower mode GET or SET parameters
    success = HandleTowerMode();
  }
  else if(commandIgnoreAck == COMMAND_PROGRAMBYTE)
  {
    // Send tower program byte packet
    success = HandleTowerProgramByte();
  }
  else if(commandIgnoreAck == COMMAND_READBYTE)
  {
    // Send tower read byte packet
    success = HandleTowerReadByte();
  }
  else if(commandIgnoreAck == COMMAND_TIME)
  {
    // Set the Real Time Clock time
    success = HandleTowerSetTime();
  }
  else if(commandIgnoreAck == COMMAND_TIMINGMODE)
  {
    // Handle timing mode command
    success = HandleTowerTimingMode();
  }
  else if(commandIgnoreAck == COMMAND_RAISES)
  {
    // Set of Get number of raises
    success = HandleTowerRaises();
  }
  else if(commandIgnoreAck == COMMAND_LOWERS)
  {
    // Set of Get number of lowers
    success = HandleTowerLowers();
  }
  else if(commandIgnoreAck == COMMAND_FREQUENCY)
  {
    // Send Frequency to Tower
    success = HandleTowerFrequency();
  }
  else if(commandIgnoreAck == COMMAND_VOLTAGE)
  {
    // Send Voltage to Tower
    success = HandleTowerVoltage();
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

void ADCReadCallback(void* arg)  //TODO: Info
{
  static uint8_t phase = 0;

  switch (phase){
    case 0:
      Analog_Get(phase, &(VoltageSamples[phase].ADC_Data[VoltageSamples[phase].LatestData]));
      VoltageSamples[phase].LatestData++;
      if(VoltageSamples[phase].LatestData == ADC_BUFFER_SIZE)
      {
        VoltageSamples[phase].LatestData = 0;
        OS_SemaphoreSignal(FrequencyCalculateSemaphore);
      }
      OS_SemaphoreSignal(AnalogThreadData[phase].semaphore);
      phase ++;
      break;
    case 1:
      Analog_Get(phase, &(VoltageSamples[phase].ADC_Data[VoltageSamples[phase].LatestData]));
      VoltageSamples[phase].LatestData++;
      if(VoltageSamples[phase].LatestData == ADC_BUFFER_SIZE)
      {
        VoltageSamples[phase].LatestData = 0;
      }
      OS_SemaphoreSignal(AnalogThreadData[phase].semaphore);
      phase ++;
      break;
    case 2:
      Analog_Get(phase, &(VoltageSamples[phase].ADC_Data[VoltageSamples[phase].LatestData]));
      VoltageSamples[phase].LatestData++;
      if(VoltageSamples[phase].LatestData == ADC_BUFFER_SIZE - 1)
      {
        OS_SemaphoreSignal(FrequencyTrackSemaphore);
      }
      if(VoltageSamples[phase].LatestData == ADC_BUFFER_SIZE)
      {
        VoltageSamples[phase].LatestData = 0;
      }
      OS_SemaphoreSignal(AnalogThreadData[phase].semaphore);
      phase = 0;
      break;
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

  if (Flash_Init() &&  LEDs_Init() && Packet_Init(BAUD_RATE, CPU_BUS_CLK_HZ) && FTM_Init() &&
      RTC_Init(RTCReadSemaphore) && Analog_Init(CPU_BUS_CLK_HZ) && PIT_Init(CPU_BUS_CLK_HZ, ADCReadCallback, NULL)/*&& Accel_Init(&accelSetup)*/)
  {
    success = true;

    // Allocates an address in Flash memory to the tower number, tower mode, timing mode, number of raises and number of lowers
    if(Flash_AllocateVar((volatile void**)&NvTowerNb, sizeof(*NvTowerNb))
    && Flash_AllocateVar((volatile void**)&NvTowerMd, sizeof(*NvTowerMd))
    && Flash_AllocateVar((volatile void**)&NvTimingMd, sizeof(*NvTimingMd))
    && Flash_AllocateVar((volatile void**)&NvNbRaises, sizeof(*NvNbRaises))
    && Flash_AllocateVar((volatile void**)&NvNbLowers, sizeof(*NvNbLowers)));
    {
      // Checks if tower number is clear
      if(_FH(NvTowerNb) == 0xFFFF)
      {
        // Sets the tower number to the default number
        if(!Flash_Write16((uint16_t*)NvTowerNb,(uint16_t)3756))
        {
          success = false;
        }
      }

      // Checks if tower mode is clear
      if(_FH(NvTowerMd) == 0xFFFF)
      {
        // Sets the tower mode to the default mode
        if(!Flash_Write16((uint16_t*)NvTowerMd,(uint16_t)1))
        {
          success = false;
        }
      }

      // Checks if timing mode is valid
      if((_FB(NvTimingMd) != TIMING_DEFINITE) && (_FB(NvTimingMd) != TIMING_INVERSE))
      {
        // Sets the timing mode to the default mode
        if(!Flash_Write8((uint8_t*)NvTimingMd, (uint8_t)TIMING_DEFINITE))
        {
          success = false;
        }
        TimingMode = _FB(NvTimingMd);
      }

      else if((_FB(NvTimingMd) == TIMING_DEFINITE) || (_FB(NvTimingMd) == TIMING_INVERSE))
      {
        TimingMode = _FB(NvTimingMd);
      }

      // Checks if number of raises is clear
      if(_FB(NvNbRaises) == 0xFF)
      {
        // Sets the number of raises to 0
        if(!Flash_Write8((uint8_t*)NvNbRaises,(uint8_t)0))
        {
          success = false;
        }
      }

      // Checks if number of lowers is clear
      if(_FB(NvNbLowers) == 0xFF)
      {
        // Sets the number of raises to 0
        if(!Flash_Write8((uint8_t*)NvNbLowers,(uint8_t)0))
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
  // Create semaphores for threads
  LEDOffSemaphore = OS_SemaphoreCreate(0);
  RTCReadSemaphore = OS_SemaphoreCreate(0);
  FrequencyCalculateSemaphore = OS_SemaphoreCreate(0);
  FrequencyTrackSemaphore = OS_SemaphoreCreate(0);
  LogRaisesSemaphore = OS_SemaphoreCreate(0);
  LogLowersSemaphore = OS_SemaphoreCreate(0);
  FlashAccessMutex = OS_SemaphoreCreate(1);
  for (uint8_t i = 0; i < NB_ANALOG_CHANNELS; i++)
  {
    AnalogThreadData[i].semaphore = OS_SemaphoreCreate(0);
  }

  // Initializes the main tower components and sets the default or stored values
  if(TowerInit())
  {
    // Turn on the orange LED to indicate the tower has initialized successfully
    LEDs_On(LED_ORANGE);
  }
  
  // Send startup packets to PC
  HandleTowerStartup();

  SamplePeriod = (uint32_t)(1000000000/(ADC_DEFAULT_FREQUENCY * ADC_SAMPLES_PER_CYCLE) / 20) * 20;    //TODO: Round to how pit does. 1000000000/modclk
  NewSamplePeriod = SamplePeriod;
  Frequency = ADC_DEFAULT_FREQUENCY;

  PIT_Set(SamplePeriod / NB_ANALOG_CHANNELS, false);    //TODO: Check how this division plays out
  PIT_Enable(true);
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
  TFTMChannel receivedPacketTmr;                   /*!< FTM Channel for received packet timer */
  receivedPacketTmr.channelNb                      = 0;
  receivedPacketTmr.delayCount                     = CPU_MCGFF_CLK_HZ_CONFIG_0;
  receivedPacketTmr.ioType.inputDetection          = TIMER_INPUT_ANY;
  receivedPacketTmr.ioType.outputAction            = TIMER_OUTPUT_DISCONNECT;
  receivedPacketTmr.timerFunction                  = TIMER_FUNCTION_OUTPUT_COMPARE;
  receivedPacketTmr.userSemaphore                  = LEDOffSemaphore;

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

/*! @brief Sends the time from the real time clock to the PC
 *
 *  @param pData is not used but is required by the OS to create a thread.
 *  @note Assumes that RTC_Init has been called successfully.
 */
static void RTCThread(void* pData)
{
  for (;;)
  {
    // Wait for RTCRead semaphore
    OS_SemaphoreWait(RTCReadSemaphore,0);

    // Toggle the yellow LED
    LEDs_Toggle(LED_YELLOW);

    // Declare variable for hours, minutes seconds
    uint8_t hours, minutes, seconds;

    // Get the current time values
    RTC_Get(&hours, &minutes, &seconds);

    // Send time to PC
    Packet_Put(COMMAND_TIME, hours, minutes, seconds);
  }
}
static void LogRaisesThread(void* pData)
{
  uint8_t events;
  for (;;)
  {
    // Wait for Log Raise semaphore
    OS_SemaphoreWait(LogRaisesSemaphore,0);

    // Gain access to flash
    OS_SemaphoreWait(FlashAccessMutex, 0);

    events = _FB(NvNbRaises);
    events ++;
    Flash_Write8((uint8_t*)NvNbRaises, events);

    // Release access to flash
    OS_SemaphoreSignal(FlashAccessMutex);

  }
}

static void LogLowersThread(void* pData)
{
  uint8_t events;
  for (;;)
  {
    // Wait for Log Raise semaphore
    OS_SemaphoreWait(LogLowersSemaphore,0);

    // Gain access to flash
    OS_SemaphoreWait(FlashAccessMutex, 0);

    events = _FB(NvNbLowers);
    events ++;
    Flash_Write8((uint8_t*)NvNbLowers, events);

    // Release access to flash
    OS_SemaphoreSignal(FlashAccessMutex);

  }
}

static void FrequencyTrackThread(void* pData)
{
  uint8_t events;
  for (;;)
  {
    // Wait for Frequency Track semaphore
    OS_SemaphoreWait(FrequencyTrackSemaphore,0);

    // Set New PIT sample period
    OS_DisableInterrupts();
    PIT_Set((uint32_t)(NewSamplePeriod / NB_ANALOG_CHANNELS), false);
    OS_EnableInterrupts();
  }
}

static void FrequencyCalculateThread(void* pData)
{
  TVoltageData localSamples;
  uint32_t newSamplePeriod;
  uint16_t m;   // Gradient (units voltage/sample period. This should always be positive)
  int16_t b;    // y-intercept crossing
  uint16_t rms;
  uint8_t crossingCount;
  float risingCrossings[10];
  float lastCrossing;
  float frequency;
  float period;

  for (;;)
  {
    // Wait for FrequencyTrack semaphore
    OS_SemaphoreWait(FrequencyCalculateSemaphore,0);

    // Get a local copy of phase A RMS,
    OS_DisableInterrupts();
    rms = RMS[0];
    OS_DisableInterrupts();

    // Check if RMS is in the frequency reading range
    if(rms > RMS_FREQUENCY_MIN)
    {
      // Get a local copy that cannot be modified by another tread
      OS_DisableInterrupts();
      for(uint8_t i = 0; i < ADC_BUFFER_SIZE; i++)
      {
        localSamples.ADC_Data[i] = VoltageSamples[0].ADC_Data[i];
      }
      OS_EnableInterrupts();

      // Reset array element counter
      crossingCount = 0;

      // Find rising crossings from sample data and store interpolated result in array (units are in sample periods)
      for(uint8_t i = 0; i < ADC_BUFFER_SIZE - 1; i++)
      {
        if((localSamples.ADC_Data[i + 1] > 0) && (localSamples.ADC_Data[i] <= 0))
        {
          m = (uint16_t)(localSamples.ADC_Data[i + 1]) - (localSamples.ADC_Data[i]);
          b = localSamples.ADC_Data[i];
          risingCrossings[crossingCount] = (float)(i + (((float)-b) / ((float)m)));
          crossingCount ++;
          //i += (ADC_SAMPLES_PER_CYCLE / 2);
        }
      }

      // If we have a buffer size equal to the number of samples per cycle, we could get only one crossing and need to reference last cycles crossing
      if(crossingCount == 1)
      {
        // Calculate the period
        period = (float)(((float)ADC_SAMPLES_PER_CYCLE + risingCrossings[crossingCount -1]) - lastCrossing);

        // Update the last crossing
        lastCrossing = risingCrossings[crossingCount - 1] -((crossingCount -1) * ((float)ADC_SAMPLES_PER_CYCLE));
      }

      // If we have a larger buffer, or if it is equal but the frequency has increased we could get more than 1 rising crossing from the sample
      else if(crossingCount > 1)
      {
        period = ((risingCrossings[crossingCount - 1]) - risingCrossings[0]) / (float)(crossingCount - 1);

        // Update the last crossing
        lastCrossing = risingCrossings[crossingCount - 1] -((crossingCount -1) * ((float)ADC_SAMPLES_PER_CYCLE));
      }

      // If we have a buffer size equal to the number of samples per cycle, we could get no crossings if the frequency has dropped.
      else if(crossingCount == 0)
      {
        //Update the last crossing as a negative number so it can be used normally next time around. Do not update period in this round.
        lastCrossing -= (float)ADC_SAMPLES_PER_CYCLE;
      }

      // Find the period in nanoseconds.
      // Truncate the same way PIT does to ensure no difference to actual sample rate (including when pit samples faster for several channels)
      uint32_t nanoSecondPerTick = 1000000000 / CPU_BUS_CLK_HZ;
      newSamplePeriod = (uint32_t)(((((uint32_t)(((float)(period * SamplePeriod) / (float)ADC_SAMPLES_PER_CYCLE))
                         / NB_ANALOG_CHANNELS)/ nanoSecondPerTick) * nanoSecondPerTick) * NB_ANALOG_CHANNELS);


      // From the period (in number of sample periods) and the sample period, work out the frequency.
      OS_DisableInterrupts();
      frequency = (float)1000000000 / (period * SamplePeriod);
      Frequency = frequency;
      OS_DisableInterrupts();

      // Update sample periods (for use by Frequency track thread and this thread)
      OS_DisableInterrupts();
      SamplePeriod = NewSamplePeriod;
      NewSamplePeriod = newSamplePeriod;
      OS_DisableInterrupts();
    }
  }
}

uint16_t UpdateRMSFast(int16_t *RemovedData, uint64_t *PreviousSumOfSquares, const TVoltageData Data, uint8_t DataSize)
{
  // Update the sum of squares, removing old data and adding new data.
  int32_t newestData;
  if(Data.LatestData == 0)
  {
    newestData = Data.ADC_Data[DataSize - 1];
  }
  else
  {
    newestData = Data.ADC_Data[Data.LatestData - 1];
  }

  int64_t newSumOfSquares = (uint64_t)(*PreviousSumOfSquares - ((int64_t)(*RemovedData) * (int64_t)(*RemovedData))) + (newestData * newestData);

  if(newSumOfSquares <= 0)
  {
    newSumOfSquares = 0;
  }

  // Update the removed data and sum of squares for next time
  *RemovedData = Data.ADC_Data[Data.LatestData];    // Note: Latest data is where the latest data WILL be put.
  *PreviousSumOfSquares = newSumOfSquares;

  return (uint16_t)sqrtf((float)newSumOfSquares / (float)DataSize);
}


uint16_t GetRMS(const TVoltageData Data, const uint8_t DataSize)
{
  float sum = 0;      //TODO: use different data type?? keep the sum of the squares.
  for (uint8_t i = 0; i < DataSize; i ++)
  {
    sum += (Data.ADC_Data[i]) * (Data.ADC_Data[i]);
  }
  return (uint16_t)sqrtf(sum / DataSize);
}

float GetAverage(const TVoltageData Data, const uint8_t DataSize)
{
  float sum = 0;
  for(uint8_t i = 0; i < DataSize; i ++)
  {
    sum += Data.ADC_Data[i];
  }
  return sum / DataSize;
}

bool CheckOutputsOff(const bool  Outputs[], const uint8_t NbOutputs)
{
  bool set = false;
  for (uint8_t i = 0; i < NbOutputs; i++)
  {
    set |= Outputs[i];
  }
  return !set;
}


/*! @brief Samples a value on an ADC channel and sends it to the corresponding DAC channel.
 *
 */
void RMSThread(void* pData)
{
  // Make the code easier to read by giving a name to the typecast'ed pointer
  #define analogData ((TAnalogThreadData*)pData)

  uint16_t rms;
  uint16_t rmsTest;
  bool alarm;
  bool alarmSet;
  bool overVoltage;
  bool underVoltage;
  bool adjusting;
  uint16_t deltaVoltage;
  int64_t OutOfRangeTimer = 5000000000;
  uint32_t minTimeCheck;

  for (;;)
  {
    // Wait for channel semaphore
    (void)OS_SemaphoreWait(analogData->semaphore, 0);

    rms = GetRMS(VoltageSamples[analogData->channelNb], ADC_BUFFER_SIZE);

    //rms = UpdateRMSFast(&(OldestData[analogData->channelNb]), &(LastSumOfSquares[analogData->channelNb]), VoltageSamples[analogData->channelNb], ADC_BUFFER_SIZE);

    // Update global RMS variable, disabling interrupts to restrict access during operation.
    OS_DisableInterrupts();

    RMS[analogData->channelNb] = rms;

    OS_EnableInterrupts();

    // Check if data is in limits
    if(rms > RMS_UPPER_LIMIT)
    {
      alarm = true;
      overVoltage = true;
    }
    else if(rms < RMS_LOWER_LIMIT)
    {
      alarm = true;
      underVoltage = true;
    }

    // Set alarm if it is not set
    if(alarm && !alarmSet)
    {
      OS_DisableInterrupts();
      if(CheckOutputsOff(Alarm, (uint8_t)NB_ANALOG_CHANNELS))
      {
        Analog_Put(3, DAC_5V_OUT);
        Alarm[analogData->channelNb] = true;
      }
      OS_EnableInterrupts();
      alarmSet = true;
    }

    if(alarm && !adjusting)
    {
      if(TimingMode == TIMING_DEFINITE)
      {
        OutOfRangeTimer -= (uint64_t)SamplePeriod;

        // Keep track of actual time
        minTimeCheck += SamplePeriod;
      }
      else if(TimingMode == TIMING_INVERSE)
      {
        if(overVoltage)
        {
          deltaVoltage = rms - RMS_UPPER_LIMIT;
        }
        else if(underVoltage)
        {
          deltaVoltage = RMS_LOWER_LIMIT - rms;
        }

        // Decrement counter
        OutOfRangeTimer -= (uint64_t)SamplePeriod;

        // Keep track of actual time
        minTimeCheck += SamplePeriod;
      }

      // Check if raise or lower signals should be set
      if(OutOfRangeTimer <= 0 && !adjusting && minTimeCheck >= 1000000000)
      {
        // Set lower signal and store count in Nv if voltage high and not already set by another channel.
        if(overVoltage)
        {
          OS_DisableInterrupts();
          if(CheckOutputsOff(Adjusting, (uint8_t)NB_ANALOG_CHANNELS))
          {
            Analog_Put(2, DAC_5V_OUT);
            Adjusting[analogData->channelNb] = true;
            OS_SemaphoreSignal(LogLowersSemaphore);
          }
          OS_EnableInterrupts();
          adjusting = true;
        }

        // Set raise signal and store count in Nv if voltage low and not already set by another channel.
        else if(underVoltage)
        {
          OS_DisableInterrupts();
          if(CheckOutputsOff(Adjusting, (uint8_t)NB_ANALOG_CHANNELS))
          {
            Analog_Put(1, DAC_5V_OUT);
            Adjusting[analogData->channelNb] = true;
            OS_SemaphoreSignal(LogRaisesSemaphore);
          }
          OS_EnableInterrupts();
          adjusting = true;
        }
      }
    }

    // Check if raise / lower should be decremented
    else if(!alarm)
    {
      bool switchOffAdjusting;

      OS_DisableInterrupts();
      Alarm[analogData->channelNb] = false;
      if(CheckOutputsOff(Alarm, (uint8_t)NB_ANALOG_CHANNELS))
      {
        Analog_Put(3, DAC_0V_OUT);
      }
      OS_EnableInterrupts();

      OS_DisableInterrupts();
      Adjusting[analogData->channelNb] = false;
      if(CheckOutputsOff(Adjusting, (uint8_t)NB_ANALOG_CHANNELS))
      {
        switchOffAdjusting = true;
      }

      if(switchOffAdjusting)
      {
        OS_DisableInterrupts();
        Analog_Put(1, DAC_0V_OUT);
        OS_EnableInterrupts();

        OS_DisableInterrupts();
        Analog_Put(2, DAC_0V_OUT);
        OS_EnableInterrupts();
      }
      OutOfRangeTimer = 5000000000;
      adjusting = false;
      alarmSet = false;
    }

    // Reset variables
    alarm = false;
    overVoltage = false;
    underVoltage = false;
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
  // Create threads for RMS channels
  for (uint8_t threadNb = 0; threadNb < NB_ANALOG_CHANNELS; threadNb++)
  {
    error = OS_ThreadCreate(RMSThread,
                            &AnalogThreadData[threadNb],
                            &RMSThreadStacks[threadNb][THREAD_STACK_SIZE - 1],
                            ANALOG_THREAD_PRIORITIES[threadNb]);
  }
  // 6th Highest priority
  error = OS_ThreadCreate(FrequencyCalculateThread,
                          NULL,
                          &FrequencyCalculateThreadStack[THREAD_STACK_SIZE - 1],
                          6);
  // 2nd Highest priority
  error = OS_ThreadCreate(FrequencyTrackThread,
                          NULL,
                          &FrequencyTrackThreadStack[THREAD_STACK_SIZE - 1],
                          2);

  // 10th Highest priority
  error = OS_ThreadCreate(PacketThread,
			  NULL,
                          &PacketThreadStack[THREAD_STACK_SIZE - 1],
                          10);
  // 17th Highest priority
  error = OS_ThreadCreate(LogRaisesThread,
        NULL,
                          &LogRaisesThreadStack[THREAD_STACK_SIZE - 1],
                          17);
  // 18th Highest priority
  error = OS_ThreadCreate(LogLowersThread,
        NULL,
                          &LogLowersThreadStack[THREAD_STACK_SIZE - 1],
                          18);
  // 29th Highest priority
  error = OS_ThreadCreate(FTMLEDsOffThread,
                          NULL,
                          &FTMLEDsOffThreadStack[THREAD_STACK_SIZE - 1],
                          29);
  // 30th Highest priority
  error = OS_ThreadCreate(RTCThread,
                          NULL,
                          &RTCThreadStack[THREAD_STACK_SIZE - 1],
                          30);



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
