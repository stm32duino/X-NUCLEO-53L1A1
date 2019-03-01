/**
 ******************************************************************************
 * @file    X_NUCLEO_53L1A1_HelloWorld.ino
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

// Components.
STMPE1600DigiOut *xshutdown_top;
STMPE1600DigiOut *xshutdown_left;
STMPE1600DigiOut *xshutdown_right;
VL53L1_X_NUCLEO_53L1A1 *sensor_vl53l1_top;
VL53L1_X_NUCLEO_53L1A1 *sensor_vl53l1_left;
VL53L1_X_NUCLEO_53L1A1 *sensor_vl53l1_right;

/* Setup ---------------------------------------------------------------------*/

void setup()
{
   // Led.
   pinMode(13, OUTPUT);

   // Initialize serial for output.
   SerialPort.begin(115200);
   SerialPort.println("Starting...");

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

   // Create VL53L1X top component.
   xshutdown_top = new STMPE1600DigiOut(&DEV_I2C, GPIO_15, (0x42 * 2));
   sensor_vl53l1_top = new VL53L1_X_NUCLEO_53L1A1(&DEV_I2C, xshutdown_top, A2);

   // Switch off VL53L1X top component.
   sensor_vl53l1_top->VL53L1_Off();

   // Create (if present) VL53L1X left component.
   xshutdown_left = new STMPE1600DigiOut(&DEV_I2C, GPIO_14, (0x43 * 2));
   sensor_vl53l1_left = new VL53L1_X_NUCLEO_53L1A1(&DEV_I2C, xshutdown_left, D8);

   //Switch off (if present) VL53L1X left component.
   sensor_vl53l1_left->VL53L1_Off();

   // Create (if present) VL53L1X right component.
   xshutdown_right = new STMPE1600DigiOut(&DEV_I2C, GPIO_15, (0x43 * 2));
   sensor_vl53l1_right = new VL53L1_X_NUCLEO_53L1A1(&DEV_I2C, xshutdown_right, D2);

   // Switch off (if present) VL53L1X right component.
   sensor_vl53l1_right->VL53L1_Off();

   //Initialize all the sensors
   sensor_vl53l1_top->InitSensor(0x10);
   sensor_vl53l1_left->InitSensor(0x12);
   sensor_vl53l1_right->InitSensor(0x14);
}

void loop()
{
   int status;
   uint8_t ready = 0;
   uint16_t distance;

   //Led off
   digitalWrite(13, LOW);

   //Start measurament
   sensor_vl53l1_top->VL53L1X_StartRanging();
   sensor_vl53l1_left->VL53L1X_StartRanging();
   sensor_vl53l1_right->VL53L1X_StartRanging();

   //Poll for measurament completion top sensor
   do
   {
      sensor_vl53l1_top->VL53L1X_CheckForDataReady(&ready);
   }
   while (!ready);

   //Led on
   digitalWrite(13, HIGH);

   //Get distance top
   status = sensor_vl53l1_top->VL53L1X_GetDistance(&distance);

   if (status == VL53L1_ERROR_NONE)
   {
      // Output data.
      char report[64];
      snprintf(report, sizeof(report), "| Distance top [mm]: %d |", distance);
      SerialPort.println(report);
   }

   //Clear interrupt
   status = sensor_vl53l1_top->VL53L1X_ClearInterrupt();

   //Poll for measurament completion left sensor
   do
   {
      sensor_vl53l1_left->VL53L1X_CheckForDataReady(&ready);
   }
   while (!ready);

   //Get distance left
   status = sensor_vl53l1_left->VL53L1X_GetDistance(&distance);

   if (status == VL53L1_ERROR_NONE)
   {
      // Output data.
      char report[64];
      snprintf(report, sizeof(report), "| Distance left [mm]: %d |", distance);
      SerialPort.println(report);
   }

   //Clear interrupt
   status = sensor_vl53l1_left->VL53L1X_ClearInterrupt();

   //Poll for measurament completion right sensor
   do
   {
      sensor_vl53l1_right->VL53L1X_CheckForDataReady(&ready);
   }
   while (!ready);

   //Get distance right
   status = sensor_vl53l1_right->VL53L1X_GetDistance(&distance);

   if (status == VL53L1_ERROR_NONE)
   {
      // Output data.
      char report[64];
      snprintf(report, sizeof(report), "| Distance right [mm]: %d |", distance);
      SerialPort.println(report);
   }

   //Clear interrupt
   status = sensor_vl53l1_right->VL53L1X_ClearInterrupt();

   //Stop measurament on all sensors
   sensor_vl53l1_top->VL53L1X_StopRanging();
   sensor_vl53l1_left->VL53L1X_StopRanging();
   sensor_vl53l1_right->VL53L1X_StopRanging();
}
