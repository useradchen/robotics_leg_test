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
#define DXL_ID_4  6    // (下)

DynamixelWorkbench dxl_wb;

uint8_t dxl_id[4] = {DXL_ID_1, DXL_ID_2, DXL_ID_3, DXL_ID_4};
const uint8_t handler_index = 0;
const int MAX_RETRIES = 3;

// 定義序列通訊緩衝區
const int BUFFER_SIZE = 32;
byte serialBuffer[BUFFER_SIZE];
int bufferIndex = 0;

// 定義馬達位置變數
int32_t current_positions[4] = {2048, 2048, 2048, 2048}; // 中心位置
bool position_updated = false;

void setup() 
{
  Serial.begin(57600);    // Debug serial
  Serial1.begin(57600);   // Communication with Python
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

    result = dxl_wb.jointMode(dxl_id[i], 0, 0, &log);
    if (result == false)
    {
      Serial.println(log);
      Serial.println("Failed to change joint mode");
      while(1);
    }
  }

  // 添加同步寫入處理器
  result = dxl_wb.addSyncWriteHandler(dxl_id[0], "Goal_Position", &log);
  if (result == false)
  {
    Serial.println(log);
    Serial.println("Failed to add sync write handler");
    while(1);
  }

  // 開啟扭矩
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

  Serial.println("Setup completed successfully");
  delay(2000);
}

void processSerialData() {
  while (Serial1.available() > 0) {
    byte inByte = Serial1.read();
    
    if (inByte == 'P') {
      bufferIndex = 0;
      serialBuffer[bufferIndex++] = inByte;
    }
    else if (bufferIndex > 0 && bufferIndex < 9) {
      serialBuffer[bufferIndex++] = inByte;
      
      if (bufferIndex == 9) {
        // 解析位置資料
        for (int i = 0; i < 4; i++) {
          current_positions[i] = (serialBuffer[1 + i*2] << 8) | serialBuffer[2 + i*2];
        }
        
        position_updated = true;
        bufferIndex = 0;
        
        // 顯示接收到的位置值
        Serial.println("Received positions:");
        for (int i = 0; i < 4; i++) {
          Serial.print("Motor ");
          Serial.print(i+1);
          Serial.print(": ");
          Serial.println(current_positions[i]);
        }
      }
    }
  }
}

void loop() 
{
  // 處理序列資料
  processSerialData();
  
  // 如果收到新的位置資料，更新馬達位置
  if (position_updated) {
    const char *log;
    bool result = dxl_wb.syncWrite(handler_index, current_positions, &log);
    if (result == false) {
      Serial.println("Failed to sync write positions");
    }
    position_updated = false;
  }
  
  delay(5);
}