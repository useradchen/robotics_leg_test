#include <DynamixelWorkbench.h>

#if defined(__OPENCM904__)
  #define DEVICE_NAME "3"
#elif defined(__OPENCR__)
  #define DEVICE_NAME ""
#endif   

#define BAUDRATE  1000000
#define DXL_ID_1  3    //(上)
#define DXL_ID_2  2    // (下)
#define DXL_ID_3  7    // (上)
#define DXL_ID_4  6  // (下)

DynamixelWorkbench dxl_wb;

uint8_t dxl_id[4] = {DXL_ID_1, DXL_ID_2, DXL_ID_3, DXL_ID_4};  // 更新為4個馬達
const uint8_t handler_index = 0;
const int MAX_RETRIES = 3;

// def & init pos 
const int POSITIONS_COUNT = 3;
// {id2(上) , id3(下) , id4(上) , id6(下)} (束)
int32_t positions[POSITIONS_COUNT][4] = {
    {1748, 2348, 1748, 2348},  // init_pos(once) "ㄑ"
    {1548, 2248, 2148, 2248},  // 
    {2148, 2248, 1548, 2248}   // 
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
  
  // 初始化每個馬達（現在是4個）
  for (int i = 0; i < 4; i++)
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

    // 設定速度
    result = dxl_wb.itemWrite(dxl_id[i], "Moving_Speed", 100, &log); // (0-1023)
    if (result == false)
    {
      Serial.println(log);
      Serial.println("Failed to set Moving_Speed");
      while(1);
    }

    // 設定扭矩限制
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

  // 開啟扭矩（現在是4個馬達）
  for (int i = 0; i < 4; i++)
  {
    result = dxl_wb.torqueOn(dxl_id[i], &log);
    if (result == false)
    {
      Serial.println(log);
      Serial.println("Failed to enable torque");
      while(1);
    }
  }
  
  // 移動到初始位置（現在是4個馬達）
  int32_t initial_position[4] = {positions[0][0], positions[0][1], positions[0][2], positions[0][3]};
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
  int32_t present_position[4] = {0, 0, 0, 0};  // 更新為4個馬達
  
  // 獲取當前目標位置（現在是4個馬達）
  int32_t current_goals[4] = {
    positions[current_position_index][0],
    positions[current_position_index][1],
    positions[current_position_index][2],
    positions[current_position_index][3]
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

  // 讀取當前位置（現在是4個馬達）
  bool read_success = false;
  for(int retry = 0; retry < MAX_RETRIES; retry++) 
  {
    bool all_positions_read = true;
    for(int i = 0; i < 4; i++) 
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

  // 檢查是否到達目標位置（現在是4個馬達）
  bool position_reached = true;
  for (int i = 0; i < 4; i++)
  {
    if (abs(current_goals[i] - present_position[i]) > 15)
    {
      position_reached = false;
      delay(50);
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
    delay(50);
  }

  delay(5);
}