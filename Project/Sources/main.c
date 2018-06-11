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

#define NB_ANALOG_CHANNELS 3

#define ADC_DEFAULT_FREQUENCY 50

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
static uint16_t Frequency;
static uint64_t OutOfRangeTimer = 5000000000;

static OS_ECB* LEDOffSemaphore;                 /*!< LED off semaphore for FTM */
static OS_ECB* RTCReadSemaphore;                /*!< Read semaphore for RTC */
static OS_ECB* OutOfRangeSemaphore;             /*!< Out of range semaphore for RMS voltage out of range */
static OS_ECB* NewADCDataSemaphore;             /*!< New ADC data semaphore for ADC Process thread */

// Thread stacks
OS_THREAD_STACK(InitModulesThreadStack, THREAD_STACK_SIZE);       /*!< The stack for the Tower Init thread. */
OS_THREAD_STACK(RTCThreadStack, THREAD_STACK_SIZE);               /*!< The stack for the RTC thread. */
OS_THREAD_STACK(FTMLEDsOffThreadStack, THREAD_STACK_SIZE);        /*!< The stack for the FTM thread. */
//OS_THREAD_STACK(PITThreadStack, THREAD_STACK_SIZE);               /*!< The stack for the PIT thread. */
OS_THREAD_STACK(ADCDataProcessThreadStack, THREAD_STACK_SIZE); /*!< The stack for the AccelReadComplete thread. */
OS_THREAD_STACK(OutOfRangeThreadStack, THREAD_STACK_SIZE);    /*!< The stack for the Alarm thread. */
OS_THREAD_STACK(PacketThreadStack, THREAD_STACK_SIZE);            /*!< The stack for the Packet thread. */

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
  if(Packet_Parameter1 == 0 && Packet_Parameter2 == 0 && Packet_Parameter3 == 0)
  {
    // Send Timing Mode
    Packet_Put(COMMAND_TIMINGMODE, _FB(NvTimingMd), 0, 0);
    return true;
  }
  else if((Packet_Parameter1 == 1 || Packet_Parameter1 == 2) && Packet_Parameter2 == 0 && Packet_Parameter3 == 0)
  {
    // Set Timing Mode
    return Flash_Write8((uint8_t*)NvTowerMd,(uint8_t)Packet_Parameter1);
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
    return Packet_Put(COMMAND_RAISES,_FB(NvNbRaises), 0, 0);            //TODO:Check with Jack if he thinks this should be in param 1
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
    return Packet_Put(COMMAND_RAISES,_FB(NvNbLowers), 0, 0);            //TODO:Check with Jack if he thinks this should be in param 1
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

    frequency.l = Frequency;

    OS_EnableInterrupts();

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
  for(uint8_t i = 0; i < NB_ANALOG_CHANNELS; i++)
  {
    Analog_Get(i, &(VoltageSamples[i].ADC_Data[VoltageSamples[i].LatestData]));
    VoltageSamples[i].LatestData++;
    if(VoltageSamples[i].LatestData == ADC_BUFFER_SIZE)
    {
      VoltageSamples[i].LatestData = 0;
      OS_SemaphoreSignal(NewADCDataSemaphore);
    }
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

      // Checks if timing mode is clear
      if(_FH(NvTimingMd) == 0xFF)
      {
        // Sets the timing mode to the default mode
        if(!Flash_Write8((uint8_t*)NvTimingMd,(uint8_t)1))
        {
          success = false;
        }
      }

      // Checks if number of raises is clear
      if(_FH(NvTowerMd) == 0xFF)
      {
        // Sets the number of raises to 0
        if(!Flash_Write8((uint8_t*)NvNbRaises,(uint8_t)0))
        {
          success = false;
        }
      }

      // Checks if number of lowers is clear
      if(_FH(NvTowerMd) == 0xFF)
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
  NewADCDataSemaphore = OS_SemaphoreCreate(0);
  OutOfRangeSemaphore = OS_SemaphoreCreate(0);
  RTCReadSemaphore = OS_SemaphoreCreate(0);

  // Initializes the main tower components and sets the default or stored values
  if(TowerInit())
  {
    // Turn on the orange LED to indicate the tower has initialized successfully
    LEDs_On(LED_ORANGE);
  }
  
  // Send startup packets to PC
  HandleTowerStartup();

  PIT_Set((uint32_t)1000000000/(ADC_DEFAULT_FREQUENCY * ADC_SAMPLES_PER_CYCLE), false);
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

uint16_t GetRMS(const TVoltageData Data, const uint8_t DataSize)
{
  float sum = 0;
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

float GetFrequency(const TVoltageData Data,  const uint8_t DataSize,  const uint32_t FreqTimesTen)
{
  // Array to store calculated crossings.
  float crossings[8];
  float mean;
  uint8_t count = 0;
  uint8_t arrayCrossing;      // To store array crossing to be used to find zero crossings

  if(Data.LatestData == 0)
  {
    arrayCrossing = (DataSize - 1);
  }
  else
  {
    arrayCrossing = (Data.LatestData - 1);
  }

  //float average = GetAverage(Data, DataSize);

  // Find crossings from LatestData to element 0 of array
  for(uint8_t i = arrayCrossing; i > 0; i --)
  {
    // Check for crossing
    if((((float)Data.ADC_Data[i] >= 0) && ((float)Data.ADC_Data[i - 1] < 0)) || (((float)Data.ADC_Data[i] <= 0) && ((float)Data.ADC_Data[i - 1] >= 0)))
    {
      // Calculate accurate crossing and store in array
      crossings[count] = (float)((i - 1) + (float)((- (float)Data.ADC_Data[i - 1]) / ((float)Data.ADC_Data[i] - (float)Data.ADC_Data[i - 1])));
      count ++;
      //i -= 10;
    }
  }

  // Check for crossing between ends of array
  if(Data.LatestData)
  {
    if((((float)Data.ADC_Data[0] >= 0) && ((float)Data.ADC_Data[DataSize - 1] < 0)) || (((float)Data.ADC_Data[0] <= 0) && ((float)Data.ADC_Data[DataSize - 1] >= 0)))
    {
      // Calculate accurate crossing and store in array
      crossings[count] = (float)((float)(((float)Data.ADC_Data[DataSize - 1]) / ((float)Data.ADC_Data[0] - (float)Data.ADC_Data[DataSize - 1])));
      count ++;
    }
  }


  for(uint8_t i = DataSize - 1; i > arrayCrossing + 1; i --)
  {
    // Check for crossing
    if((((float)Data.ADC_Data[i] >= 0) && ((float)Data.ADC_Data[i - 1] < 0)) || (((float)Data.ADC_Data[i] <= 0) && ((float)Data.ADC_Data[i - 1] >= 0)))
    {
      // Calculate accurate crossing and store in array
      crossings[count] = (float)(((i - 1) + (float)((- (float)Data.ADC_Data[i - 1]) / ((float)Data.ADC_Data[i] - (float)Data.ADC_Data[i - 1])))- ((float)DataSize - 1));
      count ++;
      //i += 10;
    }
  }

  mean = (float)((crossings[0] - crossings[count - 1]) / (count - 1));

  // Return value as frequency
  return (float)((((float)FreqTimesTen * (float)ADC_SAMPLES_PER_CYCLE) / ((float)2 * mean)) / 10);
}

static void ADCDataProcessThread(void* pData)
{
  TVoltageData currentSamples[NB_ANALOG_CHANNELS];
  static float frequency = 50;
  uint16_t rms;

  for (;;)
  {
    // Wait for New ADC Data Semaphore
    OS_SemaphoreWait(NewADCDataSemaphore,0);

    // Load new PIT period
    PIT_Set((uint32_t)((uint64_t)(10000000000 /((uint32_t)(frequency * 10) * ADC_SAMPLES_PER_CYCLE))), false);

    // Store a local copy of data for analysis, disabling interrupts to restrict access during operation.
    //OS_DisableInterrupts();

    for(uint8_t i = 0; i < NB_ANALOG_CHANNELS; i++)
    {
      currentSamples[i] = VoltageSamples[i];
    }

   //OS_EnableInterrupts();

    // Calculate RMS for all channels
    for(uint8_t i = 0; i < NB_ANALOG_CHANNELS; i++)
    {
      rms = GetRMS(currentSamples[i], ADC_BUFFER_SIZE);

      // Update global RMS variable, disabling interrupts to restrict access during operation.
      OS_DisableInterrupts();

      RMS[i] = rms;

      OS_EnableInterrupts();
    }

    // Get frequency if VRMS > 1.5 V
    if(RMS[0] > 4915)
    {
      frequency = GetFrequency(currentSamples[0], ADC_BUFFER_SIZE, (uint32_t)(frequency * 10));
    }

    // Limit lowest frequency
    if(frequency < 47.5)
    {
      frequency = 47.5;
    }

    // Limit highest frequency
    else if(frequency > 52.5)
    {
      frequency = 52.5;
    }
    
    // Update global frequency variable, disabling interrupts to restrict access during operation.
    OS_DisableInterrupts();

    Frequency = (uint16_t)(frequency * 10);           //TODO: Does this typecast always round down??

    OS_EnableInterrupts();
  }
}

static void OutOfRangeThread(void* pData)
{
  for (;;)
  {
    // Wait until signaled as out of range
    OS_SemaphoreWait(OutOfRangeSemaphore,0);

    // Set alarm output on channel 3 to 5 volts
    Analog_Put(3, 16384);

    // Check mode then subtract amount from




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
  // 3rd Highest priority
  error = OS_ThreadCreate(ADCDataProcessThread,
                          NULL,
                          &ADCDataProcessThreadStack[THREAD_STACK_SIZE - 1],
                          3);
  // 4th Highest priority
  error = OS_ThreadCreate(OutOfRangeThread,
                          NULL,
                          &OutOfRangeThreadStack[THREAD_STACK_SIZE - 1],
                          5);
  // 5th Highest priority
  error = OS_ThreadCreate(PacketThread,
			  NULL,
                          &PacketThreadStack[THREAD_STACK_SIZE - 1],
                          4);
  // 6th Highest priority
  error = OS_ThreadCreate(FTMLEDsOffThread,
                          NULL,
                          &FTMLEDsOffThreadStack[THREAD_STACK_SIZE - 1],
                          6);
  // 7th Highest priority
  error = OS_ThreadCreate(RTCThread,
                          NULL,
                          &RTCThreadStack[THREAD_STACK_SIZE - 1],
                          7);



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
