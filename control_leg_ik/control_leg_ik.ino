#include <DynamixelWorkbench.h>
#include <math.h>

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
const float LENGTH = 210.0; // leg

// Coordinates(x,y,z)
struct Coordinates {
    float x;
    float y;
    float z;
};

// use IK calculate angles
void calculateAngles(float x, float y, float z, float& theta_hp, float& theta_ap, float& theta_ar) {
    float l = LENGTH;
    
    // calculate θhp1
    theta_hp = atan2(x, (2*l - z)) + acos(sqrt(pow((2*l - z), 2) + pow(x, 2)) / (2*l));
    
    // calculate θap M_PI = 3.14....
    // theta_ap = acos(abs(x) / sqrt(pow((2*l - z), 2) + pow(x, 2))) + 
    //            acos(sqrt(pow((2*l - z), 2) + pow(x, 2)) / (2*l)) - M_PI/2;
    
    theta_ap = acos(x / sqrt(pow((2*l - z), 2) + pow(x, 2))) + 
               acos(sqrt(pow((2*l - z), 2) + pow(x, 2)) / (2*l)) - M_PI/2;
    
    // calculate θar
    theta_ar = atan2(y, (2*l - z));
}

// 將弧度轉換為MX-106位置值
int32_t convertRadianToPosition(float degree) {
    // MX-106 4095->360
    return (int32_t)((4096.0/360.0) * degree);
}

// 初始化馬達
bool initializeDynamixels() {
    const char *log;
    bool result = false;

    result = dxl_wb.init(DEVICE_NAME, BAUDRATE, &log);
    if (result == false) {
        Serial.println(log);
        Serial.println("初始化失敗");
        return false;
    }

    for (int i = 0; i < 4; i++) {
        // Ping 
        uint16_t model_number = 0;
        result = dxl_wb.ping(dxl_id[i], &model_number, &log);
        if (result == false) {
            Serial.println(log);
            return false;
        }

        // 設定關節模式
        result = dxl_wb.jointMode(dxl_id[i], 0, 0, &log);
        if (result == false) {
            Serial.println(log);
            return false;
        }

        // 開啟扭矩
        result = dxl_wb.torqueOn(dxl_id[i], &log);
        if (result == false) {
            Serial.println(log);
            return false;
        }
    }

    // 設定同步寫入處理器
    result = dxl_wb.addSyncWriteHandler(dxl_id[0], "Goal_Position", &log);
    if (result == false) {
        Serial.println(log);
        return false;
    }

    return true;
}

// 移動所有馬達到初始位置
bool moveToInitialPosition() {
    const char *log;
    
    // 設定所有馬達到相同初始位置
    int32_t initialPositions[4] = {
        2048,  // 馬達1 (DXL_ID_1 = 3)
        2048,  // 馬達2 (DXL_ID_2 = 2)
        2048,  // 馬達3 (DXL_ID_3 = 7)
        2048   // 馬達4 (DXL_ID_4 = 6)
    };
    
    Serial.println("移動到初始位置...");
    bool result = dxl_wb.syncWrite(handler_index, initialPositions, &log);
    
    if (result) {
        Serial.println("初始位置設定成功！");
    } else {
        Serial.println(log);
        Serial.println("初始位置設定失敗！");
    }
    
    // 等待馬達移動完成
    delay(1000);
    
    return result;
}

// 移動到指定座標
bool moveToCoordinate(float x, float y, float z) {
    const char *log;
    float theta_hp, theta_ap, theta_ar;
    
    // IK calculate 
    calculateAngles(x, y, z, theta_hp, theta_ap, theta_ar);
    
    // 轉換為馬達位置值
    int32_t positions[4] = {
        convertRadianToPosition(theta_hp),  // 馬達1
        convertRadianToPosition(theta_ap),  // 馬達2
        convertRadianToPosition(theta_hp),  // 馬達3
        convertRadianToPosition(theta_ap)   // 馬達4
    };
    
    // 顯示計算結果
    Serial.println("計算的角度（弧度）：");
    Serial.print("theta_hp: "); Serial.println(theta_hp);
    Serial.print("theta_ap: "); Serial.println(theta_ap);
    Serial.print("theta_ar: "); Serial.println(theta_ar);
    
    Serial.println("轉換後的馬達位置值：");
    for(int i = 0; i < 4; i++) {
        Serial.print("馬達 "); Serial.print(i+1); 
        Serial.print(": "); Serial.println(positions[i]);
    }
    
    // 執行同步移動
    bool result = dxl_wb.syncWrite(handler_index, positions, &log);
    return result;
}

void setup() {
    Serial.begin(57600);
    delay(2000);

    if (!initializeDynamixels()) {
        Serial.println("馬達初始化失敗！");
        while(1);
    }

    Serial.println("初始化完成！");
    
    // 移動到初始位置
    if (!moveToInitialPosition()) {
        Serial.println("移動到初始位置失敗！");
        // 初始位置失敗不影響整體程式運作，可以繼續執行
    }
    
    Serial.println("請輸入目標座標 (格式: x,y,z)：");
}

void loop() {
    if (Serial.available() > 0) {
        String input = Serial.readStringUntil('\n');
        input.trim();
        
        // 解析輸入的座標
        int firstComma = input.indexOf(',');
        int secondComma = input.indexOf(',', firstComma + 1);
        
        if (firstComma != -1 && secondComma != -1) {
            float x = input.substring(0, firstComma).toFloat();
            float y = input.substring(firstComma + 1, secondComma).toFloat();
            float z = input.substring(secondComma + 1).toFloat();
            
            Serial.println("\n收到座標：");
            Serial.print("X: "); Serial.println(x);
            Serial.print("Y: "); Serial.println(y);
            Serial.print("Z: "); Serial.println(z);
            
            if (moveToCoordinate(x, y, z)) {
                Serial.println("移動執行成功！");
            } else {
                Serial.println("移動執行失敗！");
            }
            
            Serial.println("\n請輸入新的座標 (格式: x,y,z)：");
        } else {
            Serial.println("格式錯誤！請使用正確格式：x,y,z");
            Serial.println("例如：100,50,150");
        }
    }
    delay(10);
}