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

// Analog functions
#include "analog.h"

#include <math.h>

#define BAUD_RATE 115200                        /*!< UART2 Baud Rate */

#define ANALOG_CHANNEL_1 0
#define ANALOG_CHANNEL_2 1
#define ANALOG_CHANNEL_3 2

#define NB_ANALOG_CHANNELS 3

#define VRR_SAMPLE_PERIOD 16

const uint32_t PERIOD_ANALOG_POLL = 1250000;    /*!< Period of the Analog polling (1 second) 1,000,000,000 / (f) 50,000 / cycles (16) */

const uint8_t COMMAND_TIMING      =    0x10;    /*!< The serial command byte for tower timing */
const uint8_t COMMAND_RAISES      =    0x11;    /*!< The serial command byte for tower raises */
const uint8_t COMMAND_LOWERS      =    0x12;    /*!< The serial command byte for tower lowers */
const uint8_t COMMAND_FREQUENCY   =    0x17;    /*!< The serial command byte for tower frequency */
const uint8_t COMMAND_VOLTAGE     =    0x18;    /*!< The serial command byte for tower voltage */
const uint8_t COMMAND_SPECTRUM    =    0x19;    /*!< The serial command byte for tower spectrum */

const uint8_t PARAM_PHASE_A       =       1;    /*!< Phase A bit of packet parameter 1 */
const uint8_t PARAM_PHASE_B       =       2;    /*!< Phase B bit of packet parameter 1 */
const uint8_t PARAM_PHASE_C       =       3;    /*!< Phase C bit of packet parameter 1 */

const int16_t VRR_ZERO            =       0;    // 0
const int16_t VRR_VOLT            =    3277;    // (2^15 - 1) / 10
const int16_t VRR_LIMIT_LOW       =    6553;    // VRR_VOLT * 2
const int16_t VRR_LIMIT_HIGH      =    9830;    // VRR_VOLT * 3 TDO: clean up doxygen and naming conventions
const int16_t VRR_OUTPUT_5V       =   16384;    // VRR_VOLT * 5

uint8_t Timing_Mode = 1;

volatile uint8_t* NvCountRaises;                /*!< Number of raises pointer to flash */
volatile uint8_t* NvCountLowers;                /*!< Number of lowers pointer to flash */

// Thread stacks
OS_THREAD_STACK(InitModulesThreadStack, THREAD_STACK_SIZE);       /*!< The stack for the Tower Init thread. */
OS_THREAD_STACK(RTCThreadStack, THREAD_STACK_SIZE);               /*!< The stack for the RTC thread. */
OS_THREAD_STACK(FTMLEDsOffThreadStack, THREAD_STACK_SIZE);        /*!< The stack for the FTM thread. */
OS_THREAD_STACK(PITThreadStack, THREAD_STACK_SIZE);               /*!< The stack for the PIT thread. */
OS_THREAD_STACK(PacketThreadStack, THREAD_STACK_SIZE);            /*!< The stack for the Packet thread. */

static uint32_t AnalogThreadStacks[NB_ANALOG_CHANNELS][THREAD_STACK_SIZE] __attribute__ ((aligned(0x08)));
static uint32_t RMSThreadStacks[NB_ANALOG_CHANNELS][THREAD_STACK_SIZE] __attribute__ ((aligned(0x08)));

typedef enum
{
  TIMING_GET,
  TIMING_DEFINITE,
  TIMING_INVERSE
} TTimingMode;

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
  OS_ECB* semaphoreRead;
  OS_ECB* semaphoreRMS;
  uint8_t channelNb;
  int16_t sampleData[VRR_SAMPLE_PERIOD];
  uint16_t sampleDataIndex;
} TAnalogThreadData;

/*! @brief Analog thread configuration data
 *
 */
static TAnalogThreadData AnalogThreadData[NB_ANALOG_CHANNELS] =
{
  {
    .semaphoreRead = NULL,
    .semaphoreRMS = NULL,
    .sampleDataIndex = 0,
    .channelNb = ANALOG_CHANNEL_1
  },
  {
    .semaphoreRead = NULL,
    .semaphoreRMS = NULL,
    .sampleDataIndex = 0,
    .channelNb = ANALOG_CHANNEL_2
  },
  {
    .semaphoreRead = NULL,
    .semaphoreRMS = NULL,
    .sampleDataIndex = 0,
    .channelNb = ANALOG_CHANNEL_3
  }
};

static OS_ECB* LEDOffSemaphore;                          /*!< LED off semaphore for FTM */

/*! @brief Calculates the RMS value on an ADC channel.
 *
 */
void CalculateRMSThread(void* pData)
{
  // Make the code easier to read by giving a name to the typecast'ed pointer
  #define analogData ((TAnalogThreadData*)pData)

  for (;;)
  {
    (void)OS_SemaphoreWait(analogData->semaphoreRMS, 0);

    int64_t sum = 0;

    for(uint8_t i = 0; i < VRR_SAMPLE_PERIOD; i++) //TODO: Make sliding window of 16 samples not every 16 samples
    {
      sum += (analogData->sampleData[i])*(analogData->sampleData[i]);
    }

    float rms = sqrt((float)sum/(float)VRR_SAMPLE_PERIOD);

    OS_DisableInterrupts();

    static uint32 counter = 1000;

    static bool alarmTriggered = false;

    if(rms < VRR_LIMIT_LOW)
    {
      if(!counter)
      {
        if(!alarmTriggered)
        {
          Flash_Write8((uint8_t*)NvCountRaises,_FB(NvCountRaises)+1);
        }

        Analog_Put(ANALOG_CHANNEL_1, VRR_OUTPUT_5V);
        Analog_Put(ANALOG_CHANNEL_2, VRR_ZERO);
        Analog_Put(ANALOG_CHANNEL_3, VRR_OUTPUT_5V);
        alarmTriggered = true;
      }
      else
      {
        counter--;
      }
    }
    else if(rms > VRR_LIMIT_HIGH)
    {
      if(!counter)
      {
        if(!alarmTriggered)
        {
          Flash_Write8((uint8_t*)NvCountLowers,_FB(NvCountLowers)+1);
        }

        Analog_Put(ANALOG_CHANNEL_1, VRR_ZERO);
        Analog_Put(ANALOG_CHANNEL_2, VRR_OUTPUT_5V);
        Analog_Put(ANALOG_CHANNEL_3, VRR_OUTPUT_5V);
        alarmTriggered = true;
      }
      else
      {
        counter--;
      }
    }
    else
    {
      counter = 1000;
      Analog_Put(ANALOG_CHANNEL_1, VRR_ZERO);
      Analog_Put(ANALOG_CHANNEL_2, VRR_ZERO);
      Analog_Put(ANALOG_CHANNEL_3, VRR_ZERO);
      alarmTriggered = false;
    }

    analogData->sampleDataIndex = 0;

    OS_EnableInterrupts();
  }
}

/*! @brief Samples a value on an ADC channel and sends it to the corresponding DAC channel.
 *
 */
void AnalogReadThread(void* pData)
{
  // Make the code easier to read by giving a name to the typecast'ed pointer
  #define analogData ((TAnalogThreadData*)pData)

  for (;;)
  {
    int16_t analogInputValue;

    (void)OS_SemaphoreWait(analogData->semaphoreRead, 0);

    OS_DisableInterrupts();

    Analog_Get(analogData->channelNb, &analogInputValue);

    OS_EnableInterrupts();

    if(analogData->sampleDataIndex == VRR_SAMPLE_PERIOD)
    {
      (void)OS_SemaphoreSignal(AnalogThreadData[analogData->channelNb].semaphoreRMS);
    }
    else
    {
      analogData->sampleData[analogData->sampleDataIndex] = analogInputValue;
      analogData->sampleDataIndex++;
    }
  }
}

/*! @brief Reads analog data every time it is called
 *
 *  @return void
 */
void PITCallback(void* arg)
{
  // Signal the analog channels to take a sample
  for (uint8_t analogNb = ANALOG_CHANNEL_1; analogNb < NB_ANALOG_CHANNELS; analogNb++)
    (void)OS_SemaphoreSignal(AnalogThreadData[analogNb].semaphoreRead);
}

/*! @brief Sets the tower timing mode or sends the packet to the PC
 *
 *  @return bool - TRUE if packet is sent or timing mode is set
 */
bool HandleTowerTiming(void)
{
  // Check if parameters match timing GET, DEFINITE or INVERSE parameters
  if(Packet_Parameter1 == TIMING_GET && Packet_Parameter2 == 0 && Packet_Parameter3 == 0)
  {
    // Sends the timing mode packet
    return Packet_Put(COMMAND_TIMING,TIMING_GET,Timing_Mode,0);
  }
  else if((Packet_Parameter1 == TIMING_DEFINITE || Packet_Parameter1 == TIMING_INVERSE) && Packet_Parameter2 == 0 && Packet_Parameter3 == 0)
  {
    // Sets the timing mode
    Timing_Mode = Packet_Parameter1;
    return true;
  }
  return false;
}

/*! @brief Resets the number of raises or sends the number to the PC
 *
 *  @return bool - TRUE if packet is sent or number of raises is reset
 */
bool HandleTowerRaises(void)
{
  // Check if parameters match tower raises GET or RESET parameters
  if(Packet_Parameter1 == DEVIATION_GET && Packet_Parameter2 == 0 && Packet_Parameter3 == 0)
  {
    // Sends the number of raises packet
    return Packet_Put(COMMAND_RAISES,DEVIATION_GET,_FB(NvCountRaises),0);
  }
  else if(Packet_Parameter1 == DEVIATION_RESET && Packet_Parameter2 == 0 && Packet_Parameter3 == 0)
  {
    // Resets the number of raises to flash
    return Flash_Write8((uint8_t*)NvCountRaises,0);
  }
  return false;
}

/*! @brief Resets the number of lowers or sends the number to the PC
 *
 *  @return bool - TRUE if packet is sent or number of lowers is reset
 */
bool HandleTowerLowers(void)
{
  // Check if parameters match tower raises GET or RESET parameters
  if(Packet_Parameter1 == DEVIATION_GET && Packet_Parameter2 == 0 && Packet_Parameter3 == 0)
  {
    // Sends the number of raises packet
    return Packet_Put(COMMAND_LOWERS,DEVIATION_GET,_FB(NvCountLowers),0);
  }
  else if(Packet_Parameter1 == DEVIATION_RESET && Packet_Parameter2 == 0 && Packet_Parameter3 == 0)
  {
    // Resets the number of lowers to flash
    return Flash_Write8((uint8_t*)NvCountLowers,0);
  }
  return false;
}

/*! @brief Sets the tower frequency packet to the PC
 *
 *  @return bool - TRUE if frequency is set
 */
bool HandleTowerFrequency(void)
{
  // Check if parameters frequency low byte and high byte is valid

  return false;
}

/*! @brief Sets the tower voltage
 *
 *  @return bool - TRUE if voltage is set
 */
bool HandleTowerVoltage(void)
{
  // Check half word

  // Check if parameters match phase A, B or C
  if(Packet_Parameter1 == PARAM_PHASE_A)
  {
    // Sends the tower voltage packet
  }
  else if(Packet_Parameter1 == PARAM_PHASE_B)
  {
    // Sends the tower voltage packet
  }
  else if(Packet_Parameter1 == PARAM_PHASE_C)
  {
    // Sends the tower voltage packet
  }
  return false;
}

/*! @brief Sets the tower spectrum
 *
 *  @return bool - TRUE if spectrum is set
 */
bool HandleTowerSpectrum(void)
{
  // Check half word

  // Check if harmonic number is between 0 and 7
  if(Packet_Parameter1 >= 0 && Packet_Parameter1 <= 7)
  {
    // Sends the spectrum packet
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

/*! @brief Initializes the main tower components by calling the initialization routines of the supporting software modules.
 *
 *  @return void
 */
bool TowerInit(void)
{
  // Success status of writing default values to Flash and FTM
  bool success = false;

  if (Flash_Init() &&  LEDs_Init() && Packet_Init(BAUD_RATE, CPU_BUS_CLK_HZ) && FTM_Init())
  {
    success = true;

    // Allocates an address in Flash memory to the tower number and tower mode
    if(Flash_AllocateVar((volatile void**)&NvCountRaises, sizeof(*NvCountRaises))
    && Flash_AllocateVar((volatile void**)&NvCountLowers, sizeof(*NvCountLowers)));
    {
      // Checks if number of raises is clear
      if(_FB(NvCountRaises) == 0xFF)
      {
        // Sets the number of raises to the default number
        if(!Flash_Write8((uint8_t*)NvCountRaises,(uint8_t)0))
        {
          success = false;
        }
      }
      // Checks if number of lowers is clear
      if(_FB(NvCountLowers) == 0xFF)
      {
        // Sets the number of lowers to the default number
        if(!Flash_Write8((uint8_t*)NvCountLowers,(uint8_t)0))
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
  // Analog
  (void)Analog_Init(CPU_BUS_CLK_HZ);

  // Generate the global analog semaphores
  for (uint8_t analogNb = ANALOG_CHANNEL_1; analogNb < NB_ANALOG_CHANNELS; analogNb++)
  {
    AnalogThreadData[analogNb].semaphoreRead = OS_SemaphoreCreate(0);
    AnalogThreadData[analogNb].semaphoreRMS = OS_SemaphoreCreate(0);
  }

  // Initialize the Periodic Interrupt Timer with the PIT Callback Function
  PIT_Init(CPU_BUS_CLK_HZ, PITCallback, NULL);

  // Create semaphores for threads
  LEDOffSemaphore = OS_SemaphoreCreate(0);

  // Initializes the main tower components and sets the default or stored values
  if(TowerInit())
  {
    // Turn on the orange LED to indicate the tower has initialized successfully
    LEDs_On(LED_ORANGE);
  }
  
  // Set the PIT with the analog polling period
  PIT_Set(PERIOD_ANALOG_POLL, true);

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
  // Create threads for analog read channels
  for (uint8_t threadNb = ANALOG_CHANNEL_1; threadNb < NB_ANALOG_CHANNELS; threadNb++)
  {
    error = OS_ThreadCreate(AnalogReadThread,
                            &AnalogThreadData[threadNb],
                            &AnalogThreadStacks[threadNb][THREAD_STACK_SIZE - 1],
                            3+threadNb);
  }
  // Create threads for analog RMS channels
  for (uint8_t threadNb = ANALOG_CHANNEL_1; threadNb < NB_ANALOG_CHANNELS; threadNb++)
  {
    error = OS_ThreadCreate(CalculateRMSThread,
                            &AnalogThreadData[threadNb],
                            &RMSThreadStacks[threadNb][THREAD_STACK_SIZE - 1],
                            6+threadNb);
  }
  // 9th Highest priority
  error = OS_ThreadCreate(PacketThread,
                          NULL,
                          &PacketThreadStack[THREAD_STACK_SIZE - 1],
                          9);
  // 10th Highest priority
  error = OS_ThreadCreate(FTMLEDsOffThread,
                          NULL,
                          &FTMLEDsOffThreadStack[THREAD_STACK_SIZE - 1],
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
