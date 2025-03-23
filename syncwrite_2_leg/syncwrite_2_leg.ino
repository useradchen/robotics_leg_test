/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Taehun Lim (Darby) */

#include <DynamixelWorkbench.h>

#if defined(__OPENCM904__)
  #define DEVICE_NAME "3" //Dynamixel on Serial3(USART3)  <-OpenCM 485EXP
#elif defined(__OPENCR__)
  #define DEVICE_NAME ""
#endif   

#define BAUDRATE  1000000
#define DXL_ID_1  2
#define DXL_ID_2  3

DynamixelWorkbench dxl_wb;

uint8_t dxl_id[2] = {DXL_ID_1,DXL_ID_2};
int32_t goal_position[3] = {2148, 1948}; // goal
const uint8_t handler_index = 0;
const int MAX_RETRIES = 3;

void setup() 
{
  Serial.begin(57600);
  delay(2000); // Wait for Serial

  const char *log;
  bool result = false;

  // Init Dynamixel
  result = dxl_wb.init(DEVICE_NAME, BAUDRATE, &log);
  if (result == false)
  {
    Serial.println(log);
    Serial.println("Failed to init");
    while(1);
  }
  
  // Init each motor
  for (int i = 0; i < 2; i++)
  {
    uint16_t model_number = 0;
    result = dxl_wb.ping(dxl_id[i], &model_number, &log);
    if (result == false)
    {
      Serial.println(log);
      Serial.print("Failed to ping ID: ");
      Serial.println(dxl_id[i]);
      while(1);
    }
    Serial.print("Succeeded to ping ID: ");
    Serial.println(dxl_id[i]);

    result = dxl_wb.jointMode(dxl_id[i], 0, 0, &log);
    if (result == false)
    {
      Serial.println(log);
      Serial.println("Failed to change joint mode");
      while(1);
    }

    result = dxl_wb.torqueOn(dxl_id[i], &log);
    if (result == false)
    {
      Serial.println(log);
      Serial.println("Failed to enable torque");
      while(1);
    }
  }

  // Add sync write/read handlers
  result = dxl_wb.addSyncWriteHandler(dxl_id[0], "Goal_Position", &log);
  if (result == false)
  {
    Serial.println(log);
    Serial.println("Failed to add sync write handler");
    while(1);
  }

  result = dxl_wb.addSyncReadHandler(dxl_id[0], "Present_Position", &log);
  if (result == false)
  {
    Serial.println(log);
    Serial.println("Failed to add sync read handler");
    while(1);
  }

  Serial.println("Setup completed successfully");
}

void loop() 
{
  const char *log;
  bool result = false;
  int32_t present_position[2] = {2048 , 2048}; // init pos

  // Write goal position
  result = dxl_wb.syncWrite(handler_index, goal_position, &log);
  // result = dxl_wb.syncWrite(1, goal_position, &log);
  // result = dxl_wb.syncWrite(2, goal_position, &log);

  if (result == false)
  {
    Serial.println(log);
    Serial.println("Failed to sync write");
    delay(100);
    return;
  }

  delay(50); // Give time for motors to start moving

  // Read positions with retry
  bool read_success = false;
  for(int retry = 0; retry < MAX_RETRIES; retry++) 
  {
    bool all_positions_read = true;
    
    // Read each motor position individually
    for(int i = 0; i < 2; i++) 
    {
      int32_t position = 0;
      result = dxl_wb.itemRead(dxl_id[i], "Present_Position", &position, &log);
      if (result == false) 
      {
        all_positions_read = false;
        Serial.print("Failed to read position from ID ");
        Serial.println(dxl_id[i]);
        break;
      }
      present_position[i] = position;
    }

    if (all_positions_read) 
    {
      read_success = true;
      break;
    }
    
    delay(20);
  }

  if (!read_success) 
  {
    Serial.println("Failed to read positions after all retries");
    return;
  }

  // Display position info
  for (int i = 0; i < 2; i++)
  {
    Serial.print("[ID ");
    Serial.print(dxl_id[i]);
    Serial.print("] Goal: ");
    Serial.print(goal_position[i]);
    Serial.print(" Present: ");
    Serial.println(present_position[i]);
  }

  // Check if target position reached
  bool position_reached = true;
  for (int i = 0; i < 2; i++)
  {
    if (abs(goal_position[i] - present_position[i]) > 15)
    {
      position_reached = false;
      break;
    }
  }

  // If reached target position, swap goals
  if (position_reached)
  {
    int32_t tmp = goal_position[0];
    goal_position[0] = goal_position[1];
    goal_position[1] = tmp;
    delay(1000);
  }

  delay(100);
}