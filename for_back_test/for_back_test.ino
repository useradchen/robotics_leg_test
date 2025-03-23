#include <DynamixelWorkbench.h>

#if defined(__OPENCM904__)
  #define DEVICE_NAME "3"
#elif defined(__OPENCR__)
  #define DEVICE_NAME ""
#endif   

#define BAUDRATE  1000000
#define DXL_ID_1  2
#define DXL_ID_2  3

DynamixelWorkbench dxl_wb;

uint8_t dxl_id[2] = {DXL_ID_1, DXL_ID_2};
const uint8_t handler_index = 0;
const int MAX_RETRIES = 3;

// 定義各個動作的位置
const int POSITIONS_COUNT = 3;
int32_t positions[POSITIONS_COUNT][2] = {
    {2348, 1748},  // 初始動作(once)
    {2048, 1648},  // "直立姿勢
    {2048, 2048}   // 
};

int current_position_index = 0;
bool forward_direction = true;

void setup() 
{
  Serial.begin(57600);
  delay(2000);

  const char *log;
  bool result = false;

  // 初始化 Dynamixel
  result = dxl_wb.init(DEVICE_NAME, BAUDRATE, &log);
  if (result == false)
  {
    Serial.println(log);
    Serial.println("Failed to init");
    while(1);
  }
  
  // 初始化每個馬達
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

    // 設定為關節模式
    result = dxl_wb.jointMode(dxl_id[i], 0, 0, &log);
    if (result == false)
    {
      Serial.println(log);
      Serial.println("Failed to change joint mode");
      while(1);
    }

    // 設定速度（Moving_Speed 範圍為 0-1023）
    result = dxl_wb.itemWrite(dxl_id[i], "Moving_Speed", 100, &log);
    if (result == false)
    {
      Serial.println(log);
      Serial.println("Failed to set Moving_Speed");
      while(1);
    }

    // 設定扭矩限制（Torque_Limit 範圍為 0-1023）
    result = dxl_wb.itemWrite(dxl_id[i], "Torque_Limit", 512, &log);
    if (result == false)
    {
      Serial.println(log);
      Serial.println("Failed to set Torque_Limit");
      while(1);
    }
  }

  // 添加同步讀寫處理器
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

  // 開啟扭矩
  for (int i = 0; i < 2; i++)
  {
    result = dxl_wb.torqueOn(dxl_id[i], &log);
    if (result == false)
    {
      Serial.println(log);
      Serial.println("Failed to enable torque");
      while(1);
    }
  }
  
  // 移動到初始位置
  int32_t initial_position[2] = {positions[0][0], positions[0][1]};
  result = dxl_wb.syncWrite(handler_index, initial_position, &log);
  if (result == false)
  {
    Serial.println(log);
    Serial.println("Failed to set initial position");
    while(1);
  }

  Serial.println("Setup completed successfully");
  delay(2000);
}

void loop() 
{
  const char *log;
  bool result = false;
  int32_t present_position[2] = {0, 0};
  
  // 獲取當前目標位置
  int32_t current_goals[2] = {
    positions[current_position_index][0],
    positions[current_position_index][1]
  };

  // 寫入目標位置
  result = dxl_wb.syncWrite(handler_index, current_goals, &log);
  if (result == false)
  {
    Serial.println(log);
    Serial.println("Failed to sync write");
    delay(100);
    return;
  }

  delay(50);

  // 讀取當前位置
  bool read_success = false;
  for(int retry = 0; retry < MAX_RETRIES; retry++) 
  {
    bool all_positions_read = true;
    for(int i = 0; i < 2; i++) 
    {
      result = dxl_wb.itemRead(dxl_id[i], "Present_Position", &present_position[i], &log);
      if (result == false) 
      {
        all_positions_read = false;
        break;
      }
    }

    if (all_positions_read) 
    {
      read_success = true;
      break;
    }
    delay(20);
  }

  // 檢查是否到達目標位置
  bool position_reached = true;
  for (int i = 0; i < 2; i++)
  {
    if (abs(current_goals[i] - present_position[i]) > 15)
    {
      position_reached = false;
      delay(1000); // add
      break;
    }
  }

  // 如果到達目標位置，更新下一個目標
  if (position_reached)
  {
    if (forward_direction)
    {
      current_position_index++;
      if (current_position_index >= POSITIONS_COUNT)
      {
        current_position_index = POSITIONS_COUNT - 2;
        forward_direction = false;
      }
    }
    else
    {
      current_position_index--;
      if (current_position_index < 1)
      {
        current_position_index = 1;
        forward_direction = true;
      }
    }
    delay(500);
  }

  delay(50);
}