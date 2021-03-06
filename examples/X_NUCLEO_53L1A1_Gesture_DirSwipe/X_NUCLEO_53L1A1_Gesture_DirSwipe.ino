/**
 ******************************************************************************
 * @file    X_NUCLEO_53L1A1_Gesture_DirSwipe.ino
 * @author  AST
 * @version V1.0.0
 * @date    14 December 2018
 * @brief   Arduino test application for the STMicrolectronics X-NUCLEO-53L1A1
 *          proximity sensor expansion board based on FlightSense.
 *          This application makes use of C++ classes obtained from the C
 *          components' drivers.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2018 STMicroelectronics</center></h2>
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
#include <tof_gestures.h>
#include <tof_gestures_DIRSWIPE_1.h>

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


// Components.
STMPE1600DigiOut xshutdown_top(&DEV_I2C, GPIO_15, (0x42 * 2));
STMPE1600DigiOut xshutdown_left(&DEV_I2C, GPIO_14, (0x43 * 2));
STMPE1600DigiOut xshutdown_right(&DEV_I2C, GPIO_15, (0x43 * 2));
VL53L1X_X_NUCLEO_53L1A1 sensor_vl53l1x_top(&DEV_I2C, &xshutdown_top);
VL53L1X_X_NUCLEO_53L1A1 sensor_vl53l1x_left(&DEV_I2C, &xshutdown_left);
VL53L1X_X_NUCLEO_53L1A1 sensor_vl53l1x_right(&DEV_I2C, &xshutdown_right);

// Gesture structure.
Gesture_DIRSWIPE_1_Data_t gestureDirSwipeData;

// Range values
uint16_t distance_left, distance_right;

void SetupSingleShot(VL53L1X_X_NUCLEO_53L1A1 *sensor)
{
   int status;

   //Change distance mode to short range
   status = sensor->VL53L1X_SetDistanceMode(1);
   if( status )
   {
      SerialPort.println("SetDistanceMode failed");
   }

   //Change timing budget again to 20 ms
   status = sensor->VL53L1X_SetTimingBudgetInMs(20);
   if( status )
   {
      SerialPort.println("SetMeasurementTimingBudgetMicroSeconds failed");
   }
   status = sensor->VL53L1X_SetInterMeasurementInMs(20);
   if( status )
   {
      SerialPort.println("SetInterMeasurementPeriodMilliSeconds failed");
   }
}


/* Setup ---------------------------------------------------------------------*/

void setup()
{
   int status;
   // Led.
   pinMode(13, OUTPUT);

   // Initialize serial for output.
   SerialPort.begin(115200);

//NOTE: workaround in order to unblock the I2C bus on the Arduino Due
#ifdef ARDUINO_SAM_DUE
   pinMode(71, OUTPUT);
   pinMode(70, OUTPUT);

   for (int i = 0; i<10; i++){
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

   // Initialize VL53L1X left component.
   status = sensor_vl53l1x_left.InitSensor(0x12);
   if(status)
   {
      SerialPort.println("Init sensor_vl53l1x_left failed...");
   }

   // Initialize VL53L1X right component.
   status = sensor_vl53l1x_right.InitSensor(0x14);
   if(status)
   {
      SerialPort.println("Init sensor_vl53l1x_right failed...");
   }

   // Initialize VL53L1X gesture library.
   tof_gestures_initDIRSWIPE_1(400, 0, 500, &gestureDirSwipeData);

   //Change Distance mode and timings
   SetupSingleShot(&sensor_vl53l1x_left);
   SetupSingleShot(&sensor_vl53l1x_right);

   //Start measurement
   sensor_vl53l1x_left.VL53L1X_StartRanging();
   sensor_vl53l1x_right.VL53L1X_StartRanging();
}


/* Loop ----------------------------------------------------------------------*/

void loop()
{
   int gesture_code;
   int left_done = 0;
   int right_done = 0;
   uint8_t NewDataReady=0;
   uint8_t RangeStatus;

   //wait for data ready
   do
   {
      //if left not done
      if(left_done == 0)
      {
         NewDataReady = 0;
         //check measurement data ready
         int status = sensor_vl53l1x_left.VL53L1X_CheckForDataReady(&NewDataReady);

         if( status )
         {
            SerialPort.println("GetMeasurementDataReady left sensor failed");
         }
         //if ready
         if(NewDataReady)
         {
            //get status
            status = sensor_vl53l1x_left.VL53L1X_GetRangeStatus(&RangeStatus);
            if( status )
            {
               SerialPort.println("GetRangeStatus left sensor failed");
            }

            //if distance < 1.3 m
            if (RangeStatus == 0)
            {
               // we have a valid range.
               status = sensor_vl53l1x_left.VL53L1X_GetDistance(&distance_left);
               if( status )
               {
                  SerialPort.println("GetDistance left sensor failed");
               }
            }
            else
            {
               distance_left = 1400;   //default distance
            }

            //restart measurement
            status = sensor_vl53l1x_left.VL53L1X_ClearInterrupt();
            if( status )
            {
               SerialPort.println("Restart left sensor failed");
            }

            left_done = 1 ;
         }
      }

      //if right not done
      if(right_done == 0)
      {
         NewDataReady = 0;
         //check measurement data ready
         int status = sensor_vl53l1x_right.VL53L1X_CheckForDataReady(&NewDataReady);

         if( status )
         {
            SerialPort.println("GetMeasurementDataReady right sensor failed");
         }
         //if ready
         if(NewDataReady)
         {
            //get status
            status = sensor_vl53l1x_right.VL53L1X_GetRangeStatus(&RangeStatus);
            if( status )
            {
               SerialPort.println("GetRangeStatus right sensor failed");
            }
            //if distance < 1.3 m
            if (RangeStatus == 0)
            {
               // we have a valid range.
               status = sensor_vl53l1x_right.VL53L1X_GetDistance(&distance_right);
               if( status )
               {
                  SerialPort.println("GetDistance right sensor failed");
               }
            }
            else
            {
               distance_right = 1400;   //default distance
            }

            //restart measurement
            status = sensor_vl53l1x_right.VL53L1X_ClearInterrupt();
            if( status )
            {
               SerialPort.println("Restart right sensor failed");
            }

            right_done = 1 ;
         }
      }
   }
   while(left_done == 0 || right_done == 0);

#ifdef DEBUG_MODE
   Serial.println("Distance left: " + String(distance_left) + " Distance right: " + String(distance_right));
#endif

   // Launch gesture detection algorithm.
   gesture_code = tof_gestures_detectDIRSWIPE_1(distance_left, distance_right, &gestureDirSwipeData);

   // Check the result of the gesture detection algorithm.
   switch(gesture_code)
   {
   case GESTURES_SWIPE_LEFT_RIGHT:
      SerialPort.println("From LEFT to RIGHT --->");
      break;
   case GESTURES_SWIPE_RIGHT_LEFT:
      SerialPort.println("From RIGHT to LEFT <---");
      break;
   default:
      // Do nothing
      break;
   }
}
