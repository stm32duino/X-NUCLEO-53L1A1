/**
 ******************************************************************************
 * @file    X_NUCLEO_53L1A1_People_Counting.ino
 * @author  AST
 * @version V1.1.0
 * @date    8 January 2021
 * @brief   Arduino test application for the STMicrolectronics X-NUCLEO-53L1A1
 *          proximity sensor expansion board based on FlightSense.
 *          This application makes use of C++ classes obtained from the C
 *          components' drivers.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2021 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include <Arduino.h>
#include <Wire.h>
#include <vl53l1x_x_nucleo_53l1a1_class.h>
#include <stmpe1600_class.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <assert.h>
#include <stdlib.h>

#ifdef ARDUINO_SAM_DUE
#define DEV_I2C Wire1
#elif defined(ARDUINO_ARCH_STM32)
#define DEV_I2C Wire
#elif defined(ARDUINO_ARCH_AVR)
#define DEV_I2C Wire
#else
#define DEV_I2C Wire
#endif
#define SerialPort Serial

//For AVR compatibility where D8 and D2 are undefined
#ifndef D8
#define D8 8
#endif

#ifndef D2
#define D2 2
#endif

// People Counting defines
#define NOBODY                    0
#define SOMEONE                   1
#define LEFT                      0
#define RIGHT                     1

#define DOOR_JAM_2400             1
#define DOOR_JAM_2000             2
#define ON_SIDE                   3

#define DISTANCE_MODE_LONG        2


// define here the profile for your case.
#define PPC_PROFILE DOOR_JAM_2400

#define TRACE_PPC				     0

// PPC_PROFILE DOOR_JAM_2400
#if PPC_PROFILE == DOOR_JAM_2400
#define PROFILE_STRING                               "DOOR_JAM_2400"
#define DISTANCES_ARRAY_SIZE                         10   // nb of samples
#define MAX_DISTANCE                                 2400 // mm
#define MIN_DISTANCE                                 0   // mm
#define DIST_THRESHOLD                               1600  // mm
#define ROWS_OF_SPADS                                8 // 8x16 SPADs ROI
#define TIMING_BUDGET                                33  // was 20 ms, I found 33 ms has better succes rate with lower reflectance target
#define DISTANCE_MODE                                DISTANCE_MODE_LONG
#endif

#if ROWS_OF_SPADS == 4
#define FRONT_ZONE_CENTER                            151
#define BACK_ZONE_CENTER                             247
#elif ROWS_OF_SPADS == 6
#define FRONT_ZONE_CENTER                            159
#define BACK_ZONE_CENTER                             239
#elif ROWS_OF_SPADS == 8
#define FRONT_ZONE_CENTER                            175 // was 167, see UM2555 on st.com, centre = 175 has better return signal rate for the ROI #1
#define BACK_ZONE_CENTER                             231 
#endif

int PplCounter = 0;
int center[2] = {FRONT_ZONE_CENTER, BACK_ZONE_CENTER}; /* these are the spad center of the 2 4*16 zones */
int Zone = 0;

// Components.
STMPE1600DigiOut xshutdown_top(&DEV_I2C, GPIO_15, (0x42 * 2));
STMPE1600DigiOut xshutdown_left(&DEV_I2C, GPIO_14, (0x43 * 2));
STMPE1600DigiOut xshutdown_right(&DEV_I2C, GPIO_15, (0x43 * 2));
VL53L1X_X_NUCLEO_53L1A1 sensor_vl53l1x_top(&DEV_I2C, &xshutdown_top);
VL53L1X_X_NUCLEO_53L1A1 sensor_vl53l1x_left(&DEV_I2C, &xshutdown_left);
VL53L1X_X_NUCLEO_53L1A1 sensor_vl53l1x_right(&DEV_I2C, &xshutdown_right);

int ProcessPeopleCountingData(int16_t Distance, uint8_t zone, uint8_t RangeStatus);

int ProcessPeopleCountingData(int16_t Distance, uint8_t zone, uint8_t RangeStatus) {
  static int PathTrack[] = {0,0,0,0};
  static int PathTrackFillingSize = 1; // init this to 1 as we start from state where nobody is any of the zones
  static int LeftPreviousStatus = NOBODY;
  static int RightPreviousStatus = NOBODY;
  static int PeopleCount = 0;
  static uint16_t Distances[2][DISTANCES_ARRAY_SIZE];
  static uint8_t DistancesTableSize[2] = {0,0};
  uint16_t MinDistance;
  uint8_t i;

  (void)RangeStatus;

#ifdef TRACE_PPC
#define TIMES_WITH_NO_EVENT 10// was 40
  static uint32_t trace_count = TIMES_WITH_NO_EVENT;  // replace by 0 if you want to trace the first TIMES_WITH_NO_EVENT values
#endif

  int CurrentZoneStatus = NOBODY;
  int AllZonesCurrentStatus = 0;
  int AnEventHasOccured = 0;

  // Add just picked distance to the table of the corresponding zone
  if (DistancesTableSize[zone] < DISTANCES_ARRAY_SIZE)
  {
    Distances[zone][DistancesTableSize[zone]] = Distance;
    DistancesTableSize[zone] ++;
  } else
  {
    for (i=1; i<DISTANCES_ARRAY_SIZE; i++)
      Distances[zone][i-1] = Distances[zone][i];
    Distances[zone][DISTANCES_ARRAY_SIZE-1] = Distance;
  }

  // pick up the min distance
  MinDistance = Distances[zone][0];
  if (DistancesTableSize[zone] >= 2)
  {
    for (i=1; i<DistancesTableSize[zone]; i++)
    {
      if (Distances[zone][i] < MinDistance)
        MinDistance = Distances[zone][i];
    }
  }

  if (MinDistance < DIST_THRESHOLD)
  {
    // Someone is in !
    CurrentZoneStatus = SOMEONE;
  }

  if (zone == LEFT) // left zone
  {
    if (CurrentZoneStatus != LeftPreviousStatus)
    {
      // event in left zone has occured
      AnEventHasOccured = 1;

      if (CurrentZoneStatus == SOMEONE)
      {
        AllZonesCurrentStatus += 1;
      }
      // need to check right zone as well ...
      if (RightPreviousStatus == SOMEONE)
      {
        // event in left zone has occured
        AllZonesCurrentStatus += 2;
      }
      // remember for next time
      LeftPreviousStatus = CurrentZoneStatus;
    }
  } else // right zone
  {
    
    if (CurrentZoneStatus != RightPreviousStatus)
    {
      // event in left zone has occured
      AnEventHasOccured = 1;
      if (CurrentZoneStatus == SOMEONE)
      {
        AllZonesCurrentStatus += 2;
      }
      // need to left right zone as well ...
      if (LeftPreviousStatus == SOMEONE)
      {
        // event in left zone has occured
        AllZonesCurrentStatus += 1;
      }
      // remember for next time
      RightPreviousStatus = CurrentZoneStatus;
    }
  }
  
#ifdef TRACE_PPC
  // print debug data only when someone is within the field of view
  trace_count++;
  if ((CurrentZoneStatus == SOMEONE) || (LeftPreviousStatus == SOMEONE) || (RightPreviousStatus == SOMEONE))
    trace_count = 0;

  if (trace_count < TIMES_WITH_NO_EVENT)
  {
    SerialPort.print(zone);
    SerialPort.print(",");
    SerialPort.print(Distance);
    SerialPort.print(",");
    SerialPort.print(MinDistance);
    SerialPort.print(",");
    SerialPort.print(RangeStatus);
    SerialPort.print(",");
    SerialPort.println(PeopleCount);
  }
#endif

  // if an event has occured
  if (AnEventHasOccured)
  {
    if (PathTrackFillingSize < 4)
    {
      PathTrackFillingSize ++;
    }

    // if nobody anywhere lets check if an exit or entry has happened
    if ((LeftPreviousStatus == NOBODY) && (RightPreviousStatus == NOBODY))
    {
      // check exit or entry only if PathTrackFillingSize is 4 (for example 0 1 3 2) and last event is 0 (nobobdy anywhere)
      if (PathTrackFillingSize == 4)
      {
        // check exit or entry. no need to check PathTrack[0] == 0 , it is always the case
        if ((PathTrack[1] == 1)  && (PathTrack[2] == 3) && (PathTrack[3] == 2))
        {
          // This an entry
          PeopleCount ++;
          // reset the table filling size in case an entry or exit just found
          DistancesTableSize[0] = 0;
          DistancesTableSize[1] = 0;
          SerialPort.print("Walk In, People Count=");
          SerialPort.println(PeopleCount);
        } else if ((PathTrack[1] == 2)  && (PathTrack[2] == 3) && (PathTrack[3] == 1))
        {
          // This an exit
          PeopleCount --;
          // reset the table filling size in case an entry or exit just found
          DistancesTableSize[0] = 0;
          DistancesTableSize[1] = 0;
          SerialPort.print("Walk Out, People Count=");
          SerialPort.println(PeopleCount);
        } else
        {
          // reset the table filling size also in case of unexpected path
          DistancesTableSize[0] = 0;
          DistancesTableSize[1] = 0;
          SerialPort.println("Wrong path");
        }
      }

      PathTrackFillingSize = 1;
    } else
    {
      // update PathTrack
      // example of PathTrack update
      // 0
      // 0 1
      // 0 1 3
      // 0 1 3 1
      // 0 1 3 3
      // 0 1 3 2 ==> if next is 0 : check if exit
      PathTrack[PathTrackFillingSize-1] = AllZonesCurrentStatus;
    }
  }

  // output debug data to main host machine
  return(PeopleCount);
}

/* Setup ---------------------------------------------------------------------*/

void setup()
{
  // Initialize serial for output.
  SerialPort.begin(460800);
  SerialPort.println("Starting...");

//NOTE: workaround in order to unblock the I2C bus on the Arduino Due
#ifdef ARDUINO_SAM_DUE
  pinMode(71, OUTPUT);
  pinMode(70, OUTPUT);

  for (int i = 0; i<10; i++) {
    digitalWrite(70, LOW);
    delay(3);
    digitalWrite(71, HIGH);
    delay(3);
    digitalWrite(70, HIGH);
    delay(3);
    digitalWrite(71, LOW);
    delay(3);
  }
  pinMode(70, INPUT);
  pinMode(71, INPUT);
#endif
//End of workaround

  // Initialize I2C bus.
  DEV_I2C.begin();

  // Configure VL53L1X top component.
  sensor_vl53l1x_top.begin();

  // Switch off VL53L1X top component.
  sensor_vl53l1x_top.VL53L1X_Off();

  // Configure (if present) VL53L1X left component.
  sensor_vl53l1x_left.begin();

  // Switch off (if present) VL53L1X left component.
  sensor_vl53l1x_left.VL53L1X_Off();

  // Configure (if present) VL53L1X right component.
  sensor_vl53l1x_right.begin();

  // Switch off (if present) VL53L1X right component.
  sensor_vl53l1x_right.VL53L1X_Off();

  //Initialize all the sensors
  sensor_vl53l1x_top.InitSensor(0x10);

  sensor_vl53l1x_top.VL53L1X_SetDistanceMode(DISTANCE_MODE); /* 1=short, 2=long */
  sensor_vl53l1x_top.VL53L1X_SetTimingBudgetInMs(TIMING_BUDGET); /* in ms possible values [15, 20, 50, 100, 200, 500] */
  sensor_vl53l1x_top.VL53L1X_SetInterMeasurementInMs(TIMING_BUDGET);
  sensor_vl53l1x_top.VL53L1X_SetROI(ROWS_OF_SPADS, 16); /* minimum ROI 4,4 */

  SerialPort.print("Start counting people with profile : ");
  SerialPort.print(PROFILE_STRING);
  SerialPort.println("...");
  sensor_vl53l1x_top.VL53L1X_StartRanging();   /* This function has to be called to enable the ranging */
}

void loop()
{
  int status;
  uint16_t Distance, Signal;
  uint8_t RangeStatus;
  uint8_t dataReady = 0;

  //Poll for measurament completion top sensor
  while (dataReady == 0)
  {
    status = sensor_vl53l1x_top.VL53L1X_CheckForDataReady(&dataReady);
    delay(1);
  }

  status += sensor_vl53l1x_top.VL53L1X_GetRangeStatus(&RangeStatus);
  status += sensor_vl53l1x_top.VL53L1X_GetDistance(&Distance);
  status += sensor_vl53l1x_top.VL53L1X_GetSignalPerSpad(&Signal);
  status += sensor_vl53l1x_top.VL53L1X_ClearInterrupt(); /* clear interrupt has to be called to enable next interrupt*/

  if (status != 0)
  {
    SerialPort.println("Error in operating the device");
  }

  status = sensor_vl53l1x_top.VL53L1X_SetROICenter(center[Zone]);
  if (status != 0)
  {
    SerialPort.println("Error in chaning the center of the ROI");
    while(1)
    {
    }
  }

  // check the status of the ranging. In case of error, lets assume the distance is the max of the use case
  // Value RangeStatus string Comment
  // 0 VL53L1X_RANGESTATUS_RANGE_VALID Ranging measurement is valid
  // 1 VL53L1X_RANGESTATUS_SIGMA_FAIL Raised if sigma estimator check is above the internal defined threshold
  // 2 VL53L1X_RANGESTATUS_SIGNAL_FAIL Raised if signal value is below the internal defined threshold
  // 4 VL53L1X_RANGESTATUS_OUTOFBOUNDS_ FAIL Raised when phase is out of bounds
  // 5 VL53L1X_RANGESTATUS_HARDWARE_FAIL Raised in case of HW or VCSEL failure
  // 7 VL53L1X_RANGESTATUS_WRAP_TARGET_ FAIL Wrapped target, not matching phases
  // 8 VL53L1X_RANGESTATUS_PROCESSING_ FAIL Internal algorithm underflow or overflow
  // 14 VL53L1X_RANGESTATUS_RANGE_INVALID The reported range is invalid
  if ((RangeStatus == 0) || (RangeStatus == 4) || (RangeStatus == 7))
  {
    if (Distance <= MIN_DISTANCE) // wraparound case see the explanation at the constants definition place
      Distance = MAX_DISTANCE + MIN_DISTANCE;
  } else // severe error cases
  {
    Distance = MAX_DISTANCE;
  }

  // inject the new ranged distance in the people counting algorithm
  PplCounter = ProcessPeopleCountingData(Distance, Zone, RangeStatus);
  SerialPort.print(Zone);
  SerialPort.print(",");
  SerialPort.print(Distance);
  SerialPort.print(",");
  SerialPort.println(Signal);
  Zone++;
  Zone = Zone%2;
}
