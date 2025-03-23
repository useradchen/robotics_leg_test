#include <DynamixelWorkbench.h>
#include <math.h>

#if defined(__OPENCM904__)
  #define DEVICE_NAME "3"
#elif defined(__OPENCR__)
  #define DEVICE_NAME ""
#endif   

#define BAUDRATE  1000000
#define DXL_ID_3  3    // Upper 
#define DXL_ID_2  2    // Lower 
#define DXL_ID_7  7    // Upper 
#define DXL_ID_6  6    // Lower 

DynamixelWorkbench dxl_wb;

uint8_t dxl_id[4] = {DXL_ID_3, DXL_ID_2, DXL_ID_7, DXL_ID_6};
const uint8_t handler_index = 0;
const float LENGTH = 210.0; // leg length

// Coordinates(x,y,z)
struct Coordinates {
    float x;
    float y;
    float z;
};

// 結構體用於存儲計算出的角度
struct IKAngles {
    float theta_hp;
    float theta_ap;
    float theta_ar;
};

// 步行參數結構體
struct WalkingParameters {
    float stride_length;     // 步長
    float step_height;       // 抬腿高度
    float walking_speed;     // 行走速度
    float cycle_time;        // 完整步態週期時間
};

// 步態階段枚舉
enum GaitPhase {
    SWING_PHASE,    // 抬腿相
    STANCE_PHASE    // 支撐相
};

// 全局位置結構體
struct Position {
    float x;
    float y;
    float z;
};

// 全局變數
Position currentPos = {0, 0, 0};  // 當前位置
Position targetPos = {0, 0, 0};   // 目標位置
bool isRelativeMove = true;      // 預設為相對移動

// Calculate angles using inverse kinematics for parallel mechanism
IKAngles calculateInverseKinematics(float x, float y, float z) {
    float l = LENGTH;  // 單根連桿長度
    IKAngles angles;
    
    Serial.println("\n--- 位置計算 ---");
    Serial.print("目標位置: (");
    Serial.print(x); Serial.print(", ");
    Serial.print(y); Serial.print(", ");
    Serial.print(z); Serial.println(")");
    
    // 計算 θhp1 (hip pitch 第一部分)
    float theta_hp1 = atan2(x, (2*l - z));
    
    // 計算 θhp2 (hip pitch 第二部分)
    float d = sqrt(pow(2*l - z, 2) + pow(x, 2));
    float theta_hp2 = acos(d / (2*l));
    
    // 計算完整的 θhp (hip pitch)
    angles.theta_hp = theta_hp1 + theta_hp2;
    
    // 計算 θap (ankle pitch)
    angles.theta_ap = acos(fabs(x) / d) + theta_hp2 - M_PI/2;
    
    // 計算 θhr (hip roll，與ankle roll相同)
    angles.theta_ar = atan2(y, (2*l - z));

    // 添加調試輸出
    Serial.println("\n--- 平行機構逆運動學計算 ---");
    Serial.print("θhp1: "); Serial.print(theta_hp1 * 180/M_PI);
    Serial.print("°, θhp2: "); Serial.print(theta_hp2 * 180/M_PI);
    Serial.print("°, θhp: "); Serial.print(angles.theta_hp * 180/M_PI); Serial.println("°");
    Serial.print("θap: "); Serial.print(angles.theta_ap * 180/M_PI);
    Serial.print("°, θhr: "); Serial.print(angles.theta_ar * 180/M_PI); Serial.println("°");
    
    return angles;
}

int32_t convertRadianToPosition(float radian) {
    float degree = radian * (180.0/M_PI);
    // 將角度映射到0-360度範圍
    while (degree < 0) degree += 360;
    while (degree >= 360) degree -= 360;
    return (int32_t)((4096.0/360.0) * degree);
}

// Initialize motors
bool initializeDynamixels() {
    const char *log;
    bool result = false;

    // Initialize communication
    result = dxl_wb.init(DEVICE_NAME, BAUDRATE, &log);
    if (result == false) {
        Serial.println(log);
        Serial.println("Initialization failed");
        return false;
    }

    for (int i = 0; i < 4; i++) {
        // Ping each motor
        uint16_t model_number = 0;
        result = dxl_wb.ping(dxl_id[i], &model_number, &log);
        if (result == false) {
            Serial.println(log);
            return false;
        }

        // Set joint mode
        result = dxl_wb.jointMode(dxl_id[i], 0, 0, &log);
        if (result == false) {
            Serial.println(log);
            return false;
        }

        // Enable torque
        result = dxl_wb.torqueOn(dxl_id[i], &log);
        if (result == false) {
            Serial.println(log);
            return false;
        }
    }

    // Set up synchronous write handler
    result = dxl_wb.addSyncWriteHandler(dxl_id[0], "Goal_Position", &log);
    if (result == false) {
        Serial.println(log);
        return false;
    }

    return true;
}

// Move all motors to initial position
bool moveToInitialPosition() {
    const char *log;
    
    // 計算初始位置的角度
    IKAngles initialAngles = calculateInverseKinematics(0, 0, 0);
    
    // 轉換為電機位置值，移除多餘的PI偏移
    int32_t initialPositions[4] = {
        convertRadianToPosition(-initialAngles.theta_hp),    // 左腿 Hip pitch
        convertRadianToPosition(initialAngles.theta_ap),     // 左腿 Ankle pitch
        convertRadianToPosition(-initialAngles.theta_hp),    // 右腿 Hip pitch
        convertRadianToPosition(initialAngles.theta_ap)      // 右腿 Ankle pitch
    };
    
    Serial.println("Moving to initial position (0,0,0)...");
    bool result = dxl_wb.syncWrite(handler_index, initialPositions, &log);
    
    if (result) {
        Serial.println("Initial position set successfully!");
        currentPos = {0, 0, 0};  // 重置當前位置
    } else {
        Serial.println(log);
        Serial.println("Failed to set initial position!");
    }
    
    // Wait for motors to complete movement
    delay(1000);
    
    return result;
}

// Get current position of all motors
bool getCurrentPositions(int32_t* currentPositions) {
    const char *log;
    bool result = true;
    
    for (int i = 0; i < 4; i++) {
        result = dxl_wb.itemRead(dxl_id[i], "Present_Position", &currentPositions[i], &log);
        if (result == false) {
            Serial.println(log);
            return false;
        }
    }
    
    return result;
}

// S型曲線插值函數
float sCurveInterpolation(float t) {
    // 使用改進的S型曲線，提供更平滑的加速和減速
    return t * t * (3 - 2 * t);
}

// 修改後的自適應速度控制函數
bool moveWithAdaptiveSpeed(int32_t targetPositions[4], int32_t currentPositions[4]) {
    const char *log;
    int32_t intermediatePositions[4];
    
    // 計算最大位置差異
    int32_t maxDiff = 0;
    for (int i = 0; i < 4; i++) {
        int32_t diff = abs(targetPositions[i] - currentPositions[i]);
        if (diff > maxDiff) maxDiff = diff;
    }
    
    // 根據最大差異自動計算分段數
    // 每1度角度（約11.37個位置值）至少分4段
    int totalSteps = (maxDiff / 10) * 4;
    if (totalSteps < 100) totalSteps = 100;
    if (totalSteps > 500) totalSteps = 500; // 限制最大分段數
    
    // 計算基礎延遲時間（根據最大差異調整）
    // 差異越大，基礎延遲越小，確保總運動時間合理
    int baseDelay = map(maxDiff, 0, 4095, 20, 5);
    
    Serial.print("\n移動開始，總步數: "); 
    Serial.println(totalSteps);
    Serial.print("最大位置差異: ");
    Serial.println(maxDiff);
    
    // 顯示起始位置
    Serial.println("\n起始位置 (0-4095):");
    for (int i = 0; i < 4; i++) {
        Serial.print("馬達 ");
        Serial.print(dxl_id[i]);
        Serial.print(": ");
        Serial.println(currentPositions[i]);
    }

    // 使用正弦加速度曲線進行運動
    for (int step = 0; step < totalSteps; step++) {
        float progress = (float)step / (totalSteps - 1);
        
        // 使用正弦加速度曲線
        float easedProgress = (1 - cos(progress * M_PI)) / 2.0;
        
        // 計算每個馬達的中間位置
        for (int i = 0; i < 4; i++) {
            intermediatePositions[i] = currentPositions[i] + 
                (int32_t)((targetPositions[i] - currentPositions[i]) * easedProgress);
        }

        // 每隔一定步數顯示當前位置
        if (step % (totalSteps/10) == 0) {
            Serial.print("\n當前進度: ");
            Serial.print(progress * 100);
            Serial.println("%");
            Serial.println("當前位置 (0-4095):");
            for (int i = 0; i < 4; i++) {
                Serial.print("馬達 ");
                Serial.print(dxl_id[i]);
                Serial.print(": ");
                Serial.println(intermediatePositions[i]);
            }
        }

        // 執行同步寫入
        bool result = dxl_wb.syncWrite(handler_index, intermediatePositions, &log);
        if (!result) {
            Serial.println(log);
            return false;
        }

        // 動態調整延遲時間
        int delayTime;
        if (progress < 0.2) {
            // 加速階段
            delayTime = baseDelay * (1.0 - progress/0.2);
        } else if (progress > 0.8) {
            // 減速階段
            delayTime = baseDelay * ((progress - 0.8)/0.2);
        } else {
            // 勻速階段
            delayTime = baseDelay * 0.5;
        }
        
        delay(delayTime);
    }

    // 顯示最終位置
    Serial.println("\n最終位置 (0-4095):");
    for (int i = 0; i < 4; i++) {
        Serial.print("馬達 ");
        Serial.print(dxl_id[i]);
        Serial.print(": ");
        Serial.println(targetPositions[i]);
    }

    // 確保到達最終位置
    return dxl_wb.syncWrite(handler_index, targetPositions, &log);
}

// 修改 moveToCoordinate 函數
bool moveToCoordinate(float x, float y, float z) {
    const char *log;
    
    // 計算目標位置
    targetPos.x = isRelativeMove ? currentPos.x + x : x;
    targetPos.y = isRelativeMove ? currentPos.y + y : y;
    targetPos.z = isRelativeMove ? currentPos.z + z : z;
    
    Serial.println("\n=== 移動資訊 ===");
    Serial.print("當前位置: (");
    Serial.print(currentPos.x); Serial.print(", ");
    Serial.print(currentPos.y); Serial.print(", ");
    Serial.print(currentPos.z); Serial.println(")");
    Serial.print("目標位置: (");
    Serial.print(targetPos.x); Serial.print(", ");
    Serial.print(targetPos.y); Serial.print(", ");
    Serial.print(targetPos.z); Serial.println(")");
    
    // 計算左右腿的反向運動學
    IKAngles leftLegAngles = calculateInverseKinematics(targetPos.x, targetPos.y, targetPos.z);
    IKAngles rightLegAngles = calculateInverseKinematics(targetPos.x, -targetPos.y, targetPos.z);
    
    // 轉換為電機位置值，移除多餘的PI偏移
    int32_t targetPositions[4] = {
        convertRadianToPosition(-leftLegAngles.theta_hp),     // 左腿 Hip pitch
        convertRadianToPosition(leftLegAngles.theta_ap),      // 左腿 Ankle pitch
        convertRadianToPosition(-rightLegAngles.theta_hp),    // 右腿 Hip pitch
        convertRadianToPosition(rightLegAngles.theta_ap)      // 右腿 Ankle pitch
    };
    
    // 顯示角度資訊
    Serial.println("\n=== 角度資訊（弧度） ===");
    Serial.print("左腿 Hip: "); Serial.print(-leftLegAngles.theta_hp);
    Serial.print(", Ankle: "); Serial.println(leftLegAngles.theta_ap);
    Serial.print("右腿 Hip: "); Serial.print(-rightLegAngles.theta_hp);
    Serial.print(", Ankle: "); Serial.println(rightLegAngles.theta_ap);
    
    // 顯示目標位置值
    Serial.println("\n=== 目標位置值 ===");
    for (int i = 0; i < 4; i++) {
        Serial.print("馬達 "); Serial.print(dxl_id[i]);
        Serial.print(": "); Serial.println(targetPositions[i]);
    }
    
    // 獲取當前位置
    int32_t currentPositions[4];
    if (!getCurrentPositions(currentPositions)) {
        Serial.println("無法讀取當前位置！");
        return false;
    }
    
    // 使用自適應速度控制移動
    bool result = moveWithAdaptiveSpeed(targetPositions, currentPositions);
    
    // 如果移動成功，更新當前位置
    if (result) {
        currentPos = targetPos;
        Serial.println("位置更新成功！");
    }
    
    return result;
}

// 生成步行軌跡點
Coordinates calculateWalkingTrajectory(WalkingParameters params, float phase, GaitPhase gait_phase) {
    Coordinates point;
    
    if (gait_phase == SWING_PHASE) {
        // 抬腿相：生成半橢圓軌跡
        point.x = params.stride_length * (phase - 0.5);  // 前後運動
        point.y = 0;  // 保持在矢狀面內
        point.z = params.step_height * sin(M_PI * phase);  // 抬腿高度
        
        Serial.print("\nSWING PHASE - Progress: "); Serial.print(phase * 100); Serial.println("%");
    } else {
        // 支撐相：直線運動
        point.x = params.stride_length * (0.5 - phase);  // 反向運動
        point.y = 0;
        point.z = 0;  // 保持接觸地面
        
        Serial.print("\nSTANCE PHASE - Progress: "); Serial.print(phase * 100); Serial.println("%");
    }
    
    Serial.print("Generated point (x,y,z): ");
    Serial.print(point.x); Serial.print(", ");
    Serial.print(point.y); Serial.print(", ");
    Serial.println(point.z);
    
    return point;
}

// 執行一個完整的步行週期
bool executeWalkingCycle(WalkingParameters params) {
    const int TRAJECTORY_POINTS = 50;  // 軌跡點數量
    
    // 執行抬腿相
    for (int i = 0; i < TRAJECTORY_POINTS; i++) {
        float phase = (float)i / TRAJECTORY_POINTS;
        Coordinates point = calculateWalkingTrajectory(params, phase, SWING_PHASE);
        
        if (!moveToCoordinate(point.x, point.y, point.z)) {
            Serial.println("Walking cycle failed during swing phase!");
            return false;
        }
        delay(params.cycle_time * 1000 / (2 * TRAJECTORY_POINTS));  // 半週期用於抬腿相
    }
    
    // 執行支撐相
    for (int i = 0; i < TRAJECTORY_POINTS; i++) {
        float phase = (float)i / TRAJECTORY_POINTS;
        Coordinates point = calculateWalkingTrajectory(params, phase, STANCE_PHASE);
        
        if (!moveToCoordinate(point.x, point.y, point.z)) {
            Serial.println("Walking cycle failed during stance phase!");
            return false;
        }
        delay(params.cycle_time * 1000 / (2 * TRAJECTORY_POINTS));  // 半週期用於支撐相
    }
    
    return true;
}

// 添加測試用函式
void debugTesting(float x, float y, float z) {
    // 暫時關閉相對移動模式
    bool tempRelativeMode = isRelativeMove;
    isRelativeMove = false;
    
    Serial.println("\n=== Debug Testing Mode ===");
    Serial.print("測試絕對座標移動到: (");
    Serial.print(x); Serial.print(", ");
    Serial.print(y); Serial.print(", ");
    Serial.print(z); Serial.println(")");
    
    if (moveToCoordinate(x, y, z)) {
        Serial.println("測試移動成功！");
    } else {
        Serial.println("測試移動失敗！");
    }
    
    // 恢復原本的移動模式
    isRelativeMove = tempRelativeMode;
}

void setup() {
    Serial.begin(57600);
    delay(2000);

    if (!initializeDynamixels()) {
        Serial.println("馬達初始化失敗！");
        while(1);
    }

    Serial.println("初始化完成！");
    
    if (!moveToInitialPosition()) {
        Serial.println("移動到初始位置失敗！");
    }
    
    // 預設為相對移動模式
    isRelativeMove = true;
    
    Serial.println("\n=== 可用命令 ===");
    Serial.println("1. 移動命令: x,y,z (預設為相對移動)");
    Serial.println("2. 絕對移動: a,x,y,z");
    Serial.println("3. 步行命令: walk,stride_length,step_height,cycle_time");
    Serial.println("4. 測試命令: debug,x,y,z");
    Serial.println("5. 回到原點: home 或 h 或 0,0,0");
    Serial.println("\n準備接收命令:");
}

void loop() {
    if (Serial.available() > 0) {
        String input = Serial.readStringUntil('\n');
        input.trim();
        
        if (input == "home" || input == "h") {  // 回到原點命令
            Serial.println("\n=== 回到原點 ===");
            isRelativeMove = false;  // 暫時切換到絕對移動模式
            if (moveToCoordinate(0, 0, 0)) {
                Serial.println("已回到原點！");
            } else {
                Serial.println("回到原點失敗！");
            }
            isRelativeMove = true;  // 恢復相對移動模式
            return;
        }
        
        if (input.startsWith("a,")) {  // 絕對移動命令
            isRelativeMove = false;
            input = input.substring(2);
        } else if (input.startsWith("debug,")) {  // 測試命令
            input = input.substring(6);
            int firstComma = input.indexOf(',');
            int secondComma = input.indexOf(',', firstComma + 1);
            
            if (firstComma != -1 && secondComma != -1) {
                float x = input.substring(0, firstComma).toFloat();
                float y = input.substring(firstComma + 1, secondComma).toFloat();
                float z = input.substring(secondComma + 1).toFloat();
                debugTesting(x, y, z);
            } else {
                Serial.println("無效的測試格式！正確格式: debug,x,y,z");
            }
            return;
        }
        
        if (input.startsWith("walk,")) {
            // 解析步行參數
            input = input.substring(5); // 移除 "walk,"
            int firstComma = input.indexOf(',');
            int secondComma = input.indexOf(',', firstComma + 1);
            
            if (firstComma != -1 && secondComma != -1) {
                WalkingParameters params;
                params.stride_length = input.substring(0, firstComma).toFloat();
                params.step_height = input.substring(firstComma + 1, secondComma).toFloat();
                params.cycle_time = input.substring(secondComma + 1).toFloat();
                params.walking_speed = params.stride_length / params.cycle_time;
                
                Serial.println("\nStarting walking cycle with parameters:");
                Serial.print("Stride length: "); Serial.println(params.stride_length);
                Serial.print("Step height: "); Serial.println(params.step_height);
                Serial.print("Cycle time: "); Serial.println(params.cycle_time);
                
                if (executeWalkingCycle(params)) {
                    Serial.println("Walking cycle completed successfully!");
                } else {
                    Serial.println("Walking cycle failed!");
                }
            } else {
                Serial.println("Invalid walking parameters! Format: walk,stride_length,step_height,cycle_time");
            }
        } else {
            // 解析座標
            int firstComma = input.indexOf(',');
            int secondComma = input.indexOf(',', firstComma + 1);
            
            if (firstComma != -1 && secondComma != -1) {
                float x = input.substring(0, firstComma).toFloat();
                float y = input.substring(firstComma + 1, secondComma).toFloat();
                float z = input.substring(secondComma + 1).toFloat();
                
                // 檢查是否為回到原點的特殊情況
                if (isRelativeMove && x == 0 && y == 0 && z == 0) {
                    Serial.println("\n=== 回到原點 ===");
                    isRelativeMove = false;  // 暫時切換到絕對移動模式
                    if (moveToCoordinate(0, 0, 0)) {
                        Serial.println("已回到原點！");
                    } else {
                        Serial.println("回到原點失敗！");
                    }
                    isRelativeMove = true;  // 恢復相對移動模式
                } else {
                    if (moveToCoordinate(x, y, z)) {
                        Serial.println("移動執行成功！");
                        if (!isRelativeMove) {
                            isRelativeMove = true;  // 執行完絕對移動後恢復相對移動模式
                        }
                    } else {
                        Serial.println("移動執行失敗！");
                    }
                }
            } else {
                Serial.println("無效的格式！請使用以下格式：");
                Serial.println("1. 移動命令: x,y,z (預設為相對移動)");
                Serial.println("2. 絕對移動: a,x,y,z");
                Serial.println("3. 步行命令: walk,stride_length,step_height,cycle_time");
                Serial.println("4. 測試命令: debug,x,y,z");
                Serial.println("5. 回到原點: home 或 h 或 0,0,0");
            }
        }
        
        Serial.println("\n準備接收下一個命令:");
    }
    delay(10);
}
