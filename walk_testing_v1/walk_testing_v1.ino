#include <DynamixelWorkbench.h>
#include <math.h>
#include <map>

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

IKAngles calculateInverseKinematics(float x, float y, float z) {
    float l = LENGTH;
    IKAngles angles;
    // 先計算 θhp2
    float theta_hp2 = acos(sqrt(pow((2*l - z), 2) + pow(x, 2)) / (2*l));

    // 然後計算 θhp
    angles.theta_hp = atan2(x, (2*l - z)) + theta_hp2;

    // 再計算 θap，使用之前計算的 θhp2
    angles.theta_ap = acos(fabs(x) / sqrt(pow((2*l - z), 2) + pow(x, 2))) + theta_hp2 - M_PI/2;

    // θar 的計算沒有問題
    angles.theta_ar = atan2(y, (2*l - z));

    return angles;
}

int32_t convertRadianToPosition(float radian) {
    // 將弧度轉換為角度
    printf("... in ...");
    float degree = radian * (180.0/M_PI);
    float test;
    // 將角度映射到0-360度範圍
    while (degree < 0) degree += 360;
    while (degree >= 360) degree -= 360;
    test = (int32_t)((4096.0/360.0) * degree);
    printf("degree = %f\n" , degree);
    printf("test = %f\n" , test);
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
    
    // Set all motors to the same initial position (center)
    int32_t initialPositions[4] = {
        2048,  // DXL_ID_3 = 3
        2048,  // DXL_ID_2 = 2
        2048,  // DXL_ID_7 = 7
        2048   // DXL_ID_6 = 6
    };
    
    Serial.println("Moving to initial position...");
    bool result = dxl_wb.syncWrite(handler_index, initialPositions, &log);
    
    if (result) {
        Serial.println("Initial position set successfully!");
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
//////////////////////////////////////////////////////////////////
#include <vector>
using namespace std;  // 加入這行來使用整個std命名空間

// 步態階段定義
enum WalkPhase {
    READY,      // 準備姿勢
    LEFT_LIFT,  // 左腳抬起
    LEFT_MOVE,  // 左腳向前
    LEFT_DOWN,  // 左腳落下
    RIGHT_LIFT, // 右腳抬起
    RIGHT_MOVE, // 右腳向前
    RIGHT_DOWN  // 右腳落下
};

// 步態位置結構
struct StepPosition {
    float x;    // 前後方向 (正值為前進)
    float y;    // 左右方向 (正值為左側)
    float z;    // 上下方向 (值越小，腿越伸直)
};

// 定義一個完整的步行序列
vector<StepPosition> leftLegSequence;
vector<StepPosition> rightLegSequence;
int currentStepIndex = 0;
bool isWalking = false;

// 初始化步行序列
void initializeWalkingSequence(float stepLength = 40.0, float liftHeight = 30.0) {
    leftLegSequence.clear();
    rightLegSequence.clear();
    
    // 初始站立姿勢 - 雙腳平行 (0,0,40)
    leftLegSequence.push_back({0, 20, 40});
    rightLegSequence.push_back({0, -20, 40});
    
    // 預備姿勢 - 稍微蹲下 (0,0,50)
    leftLegSequence.push_back({0, 20, 50});
    rightLegSequence.push_back({0, -20, 50});
    
    // 1. 左腳抬起
    leftLegSequence.push_back({0, 20, 50 - liftHeight});  // 左腳抬高
    rightLegSequence.push_back({0, -20, 50 + 5});         // 右腳稍微加壓，承重
    
    // 2. 左腳前移
    leftLegSequence.push_back({stepLength, 20, 50 - liftHeight}); // 左腳前移且保持抬起
    rightLegSequence.push_back({0, -20, 50 + 5});                 // 右腳保持承重
    
    // 3. 左腳落下
    leftLegSequence.push_back({stepLength, 20, 50});      // 左腳落下
    rightLegSequence.push_back({0, -20, 50});             // 右腳回正
    
    // 4. 右腳抬起
    leftLegSequence.push_back({stepLength, 20, 50 + 5});          // 左腳承重
    rightLegSequence.push_back({0, -20, 50 - liftHeight});        // 右腳抬高
    
    // 5. 右腳前移
    leftLegSequence.push_back({stepLength, 20, 50 + 5});                   // 左腳保持承重
    rightLegSequence.push_back({stepLength * 2, -20, 50 - liftHeight});    // 右腳前移且保持抬起
    
    // 6. 右腳落下
    leftLegSequence.push_back({stepLength, 20, 50});              // 左腳回正
    rightLegSequence.push_back({stepLength * 2, -20, 50});        // 右腳落下
    
    // 循環繼續...可以添加更多步態
    
    Serial.println("步行序列已初始化");
    Serial.print("總步驟數: ");
    Serial.println(leftLegSequence.size());
}

// 執行下一個步行階段
bool executeNextWalkStep() {
    if (leftLegSequence.empty() || currentStepIndex >= leftLegSequence.size()) {
        Serial.println("步行序列為空或已完成");
        return false;
    }
    
    StepPosition leftPos = leftLegSequence[currentStepIndex];
    StepPosition rightPos = rightLegSequence[currentStepIndex];
    
    Serial.print("執行步行階段 ");
    Serial.print(currentStepIndex);
    Serial.print(": L(");
    Serial.print(leftPos.x);
    Serial.print(",");
    Serial.print(leftPos.y);
    Serial.print(",");
    Serial.print(leftPos.z);
    Serial.print(") R(");
    Serial.print(rightPos.x);
    Serial.print(",");
    Serial.print(rightPos.y);
    Serial.print(",");
    Serial.print(rightPos.z);
    Serial.println(")");
    
    // 單獨移動左腳
    bool leftResult = moveToCoordinate(leftPos.x, leftPos.y, leftPos.z);
    delay(300); // 移動間的短暫延遲
    
    // 單獨移動右腳
    bool rightResult = moveToCoordinate(rightPos.x, -rightPos.y, rightPos.z);
    
    currentStepIndex++;
    
    // 如果完成了一個完整的循環，重置索引以繼續走路
    if (currentStepIndex >= leftLegSequence.size()) {
        // 可選：可以在此重置索引並繼續循環
        // currentStepIndex = 2; // 從第一個真正的步態開始重新開始，跳過初始姿勢
    }
    
    return leftResult && rightResult;
}

// 修改moveToCoordinate函數，增加只移動單腿的功能
bool moveToCoordinate(float x, float y, float z, bool leftLegOnly = false, bool rightLegOnly = false) {
    const char *log;
    
    // 分別計算左右腿的反向運動學
    IKAngles leftLegAngles = calculateInverseKinematics(x, y, z);
    IKAngles rightLegAngles = calculateInverseKinematics(x, -y, z);  // 使用-y處理右腿鏡像效應

    // 清晰標記左右腿的角度輸出，便於調試
    Serial.println("\n--- Left Leg Angles (radians) ---");
    Serial.print("Hip Pitch (θhp): "); Serial.println(leftLegAngles.theta_hp);
    Serial.print("Ankle Pitch (θap): "); Serial.println(leftLegAngles.theta_ap);
    Serial.print("Ankle Roll (θar): "); Serial.println(leftLegAngles.theta_ar);

    Serial.println("\n--- Right Leg Angles (radians) ---");
    Serial.print("Hip Pitch (θhp): "); Serial.println(rightLegAngles.theta_hp);
    Serial.print("Ankle Pitch (θap): "); Serial.println(rightLegAngles.theta_ap);
    Serial.print("Ankle Roll (θar): "); Serial.println(rightLegAngles.theta_ar);
    
    // 獲取當前位置
    int32_t currentPositions[4];
    if (!getCurrentPositions(currentPositions)) {
        Serial.println("Failed to read current positions!");
        return false;
    }
    
    // 將角度轉換為電機位置值，並清晰標記每個電機的用途
    int32_t targetPositions[4];
    
    // 如果只移動左腿，保持右腿不動
    if (leftLegOnly) {
        targetPositions[0] = convertRadianToPosition(-(leftLegAngles.theta_hp+M_PI));  // Motor 3 - 左腿 Hip pitch
        targetPositions[1] = convertRadianToPosition(leftLegAngles.theta_ap-M_PI);     // Motor 2 - 左腿 Ankle pitch 
        targetPositions[2] = currentPositions[2];  // 保持右腿 Hip 不動
        targetPositions[3] = currentPositions[3];  // 保持右腿 Ankle 不動
    }
    // 如果只移動右腿，保持左腿不動
    else if (rightLegOnly) {
        targetPositions[0] = currentPositions[0];  // 保持左腿 Hip 不動
        targetPositions[1] = currentPositions[1];  // 保持左腿 Ankle 不動
        targetPositions[2] = convertRadianToPosition(-(rightLegAngles.theta_hp+M_PI)); // Motor 7 - 右腿 Hip pitch
        targetPositions[3] = convertRadianToPosition(rightLegAngles.theta_ap-M_PI);    // Motor 6 - 右腿 Ankle pitch
    }
    // 否則移動兩條腿
    else {
        targetPositions[0] = convertRadianToPosition(-(leftLegAngles.theta_hp+M_PI));  // Motor 3 - 左腿 Hip pitch
        targetPositions[1] = convertRadianToPosition(leftLegAngles.theta_ap-M_PI);     // Motor 2 - 左腿 Ankle pitch 
        targetPositions[2] = convertRadianToPosition(-(rightLegAngles.theta_hp+M_PI)); // Motor 7 - 右腿 Hip pitch
        targetPositions[3] = convertRadianToPosition(rightLegAngles.theta_ap-M_PI);    // Motor 6 - 右腿 Ankle pitch
    }
    
    // 輸出轉換後的電機位置值，便於調試
    Serial.println("\n--- Target Motor Positions ---");
    Serial.print("Motor 3 (Left Hip): "); Serial.println(targetPositions[0]);
    Serial.print("Motor 2 (Left Ankle): "); Serial.println(targetPositions[1]);
    Serial.print("Motor 7 (Right Hip): "); Serial.println(targetPositions[2]);
    Serial.print("Motor 6 (Right Ankle): "); Serial.println(targetPositions[3]);
    
    // 檢查位置值是否在有效範圍內 (0-4095)
    for (int i = 0; i < 4; i++) {
        if (targetPositions[i] < 0 || targetPositions[i] > 4095) {
            Serial.print("Warning: Motor ");
            Serial.print(dxl_id[i]);
            Serial.print(" position value ");
            Serial.print(targetPositions[i]);
            Serial.println(" is out of range (0-4095)!");
            
            // 將值限制在有效範圍內
            targetPositions[i] = constrain(targetPositions[i], 0, 4095);
            Serial.print("Adjusted to: ");
            Serial.println(targetPositions[i]);
        }
    }
}
///////////////////////////////////////////////////////////////////

// Calculate angles using inverse kinematics
// 修正返回值，使用結構體返回多個角度


// Move to specified coordinates with gradual motion using for loop
bool moveToCoordinate(float x, float y, float z) {
    const char *log;
    
    // 分別計算左右腿的反向運動學
    IKAngles leftLegAngles = calculateInverseKinematics(x, y, z);
    IKAngles rightLegAngles = calculateInverseKinematics(x, -y, z);  // 使用-y處理右腿鏡像效應

    // 清晰標記左右腿的角度輸出，便於調試
    Serial.println("\n--- Left Leg Angles (radians) ---");
    Serial.print("Hip Pitch (θhp): "); Serial.println(leftLegAngles.theta_hp);
    Serial.print("Ankle Pitch (θap): "); Serial.println(leftLegAngles.theta_ap);
    Serial.print("Ankle Roll (θar): "); Serial.println(leftLegAngles.theta_ar);

    Serial.println("\n--- Right Leg Angles (radians) ---");
    Serial.print("Hip Pitch (θhp): "); Serial.println(rightLegAngles.theta_hp);
    Serial.print("Ankle Pitch (θap): "); Serial.println(rightLegAngles.theta_ap);
    Serial.print("Ankle Roll (θar): "); Serial.println(rightLegAngles.theta_ar);
    
    // 將角度轉換為電機位置值，並清晰標記每個電機的用途
    int32_t targetPositions[4] = {
        convertRadianToPosition(-(leftLegAngles.theta_hp+M_PI)),     // Motor 3 - 左腿 Hip pitch
        convertRadianToPosition(leftLegAngles.theta_ap-M_PI),        // Motor 2 - 左腿 Ankle pitch 
        convertRadianToPosition(-(rightLegAngles.theta_hp+M_PI)),    // Motor 7 - 右腿 Hip pitch
        convertRadianToPosition(rightLegAngles.theta_ap-M_PI)        // Motor 6 - 右腿 Ankle pitch
    };
    
    // 輸出轉換後的電機位置值，便於調試
    Serial.println("\n--- Target Motor Positions ---");
    Serial.print("Motor 3 (Left Hip): "); Serial.println(targetPositions[0]);
    Serial.print("Motor 2 (Left Ankle): "); Serial.println(targetPositions[1]);
    Serial.print("Motor 7 (Right Hip): "); Serial.println(targetPositions[2]);
    Serial.print("Motor 6 (Right Ankle): "); Serial.println(targetPositions[3]);
    
    // 檢查位置值是否在有效範圍內 (0-4095)
    for (int i = 0; i < 4; i++) {
        if (targetPositions[i] < 0 || targetPositions[i] > 4095) {
            Serial.print("Warning: Motor ");
            Serial.print(dxl_id[i]);
            Serial.print(" position value ");
            Serial.print(targetPositions[i]);
            Serial.println(" is out of range (0-4095)!");
            
            // 將值限制在有效範圍內
            targetPositions[i] = constrain(targetPositions[i], 0, 4095);
            Serial.print("Adjusted to: ");
            Serial.println(targetPositions[i]);
        }
    }
    
    // Debug output for target positions
    Serial.println("Target position values:");
    for(int i = 0; i < 4; i++) {
        Serial.print("Motor "); Serial.print(dxl_id[i]); 
        Serial.print(": "); Serial.println(targetPositions[i]);
    }
    
    // Get current positions
    int32_t currentPositions[4];
    if (!getCurrentPositions(currentPositions)) {
        Serial.println("Failed to read current positions!");
        return false;
    }
    
    // Debug output for current positions
    Serial.println("Current position values:");
    for(int i = 0; i < 4; i++) {
        Serial.print("Motor "); Serial.print(dxl_id[i]); 
        Serial.print(": "); Serial.println(currentPositions[i]);
    }
    
    // Calculate the maximum position difference to determine steps
    int32_t maxDifference = 0;
    for (int i = 0; i < 4; i++) {
        int32_t diff = abs(targetPositions[i] - currentPositions[i]);
        if (diff > maxDifference) {
            maxDifference = diff;
        }
    }
    
    // Skip very small movements
    if (maxDifference < 10) {
        Serial.println("Movement too small, skipping");
        return true;
    }
    
    int totalSteps = 0;
    int ranges[] = {100, 200, 300, 400, 500, 600, 700, 800, 900}; // 每個區間的上限
    int steps[] = {5, 10, 15, 20, 25, 30, 35, 40, 45, 50}; // 對應的步數

    for (int i = 0; i < 9; i++) {
        if (maxDifference < ranges[i]) {
            totalSteps = steps[i];
            break;
        }
    }

    
    Serial.print("Maximum position difference: ");
    Serial.println(maxDifference);
    Serial.print("Moving in ");
    Serial.print(totalSteps);
    Serial.println(" steps");
    
    // Use sine-based easing function for smoother acceleration/deceleration
    bool result = true;
    int32_t intermediatePositions[4];
    
    for (int step = 0; step < totalSteps; step++) {
        // Calculate progress ratio using sine-based easing
        // This creates a smooth S-curve acceleration/deceleration profile
        float progress = (float)step / (totalSteps - 1);
        float easedProgress = (sin((progress - 0.5) * M_PI) + 1) / 2;
        
        // Calculate intermediate positions for this step using eased progress
        for (int i = 0; i < 4; i++) {
            intermediatePositions[i] = currentPositions[i] + 
                (int32_t)((targetPositions[i] - currentPositions[i]) * easedProgress);
        }
        
        // Debug output for some steps
        if (step % 10 == 0 || step == totalSteps - 1) {
            Serial.print("Step ");
            Serial.print(step + 1);
            Serial.print("/");
            Serial.print(totalSteps);
            Serial.print(" (Progress: ");
            Serial.print(easedProgress * 100);
            Serial.println("%)");
        }
        
        // Execute synchronized movement to intermediate position
        result = dxl_wb.syncWrite(handler_index, intermediatePositions, &log);
        if (result == false) {
            Serial.println(log);
            Serial.println("Movement failed at step: " + String(step + 1));
            return false;
        }
        
        // Dynamic delay based on the movement phase
        // Shorter delays in the middle for higher speed, longer at start/end for smooth transitions
        int delayTime;
        
        if (progress < 0.2 || progress > 0.8) {
            // Slower at the beginning and end (20% of movement)
            delayTime = ::map(maxDifference, 0, 2000, 30, 15); 
        } else {
            // Faster in the middle
            delayTime = ::map(maxDifference, 0, 2000, 20, 5);
        }
        
        delay(delayTime);
    }
    
    // Ensure we reach the exact target position
    result = dxl_wb.syncWrite(handler_index, targetPositions, &log);
    if (result) {
        Serial.println("Movement completed successfully!");
    } else {
        Serial.println(log);
        Serial.println("Final position adjustment failed!");
    }
    
    return result;
}





























void setup() {
    Serial.begin(57600);
    delay(2000);

    if (!initializeDynamixels()) {
        Serial.println("Motor initialization failed!");
        while(1);
    }

    Serial.println("Initialization complete!");
    
    // Move to initial position
    if (!moveToInitialPosition()) {
        Serial.println("Failed to move to initial position!");
        // Failure to reach initial position doesn't affect overall program operation
    }
    
    Serial.println("Enter target coordinates (format: x,y,z):");

    initializeWalkingSequence();
    Serial.println("初始化完成。步行命令說明：");
    Serial.println("walk - 開始步行序列");
    Serial.println("next - 執行下一個步行階段");
    Serial.println("stand - 恢復站立姿勢");
    Serial.println("x,y,z - 直接移動到指定坐標");
}

void loop() {
    if (Serial.available() > 0) {
        String input = Serial.readStringUntil('\n');
        input.trim();
        
        // 步行命令
        if (input == "walk") {
            Serial.println("開始步行序列");
            currentStepIndex = 0;
            isWalking = true;
            executeNextWalkStep();
        }
        // 執行下一步
        else if (input == "next") {
            if (executeNextWalkStep()) {
                Serial.println("步行階段執行成功");
            } else {
                Serial.println("步行階段執行失敗或已完成");
                isWalking = false;
            }
        }
        // 恢復站立姿勢
        else if (input == "stand") {
            Serial.println("恢復站立姿勢");
            moveToCoordinate(0, 0, 40);
            isWalking = false;
        }
        // 解析為坐標
        else if (input.indexOf(',') != -1) {
            int firstComma = input.indexOf(',');
            int secondComma = input.indexOf(',', firstComma + 1);
            
            if (firstComma != -1 && secondComma != -1) {
                float x = input.substring(0, firstComma).toFloat();
                float y = input.substring(firstComma + 1, secondComma).toFloat();
                float z = input.substring(secondComma + 1).toFloat();
                
                Serial.println("\n接收到的坐標:");
                Serial.print("X: "); Serial.println(x);
                Serial.print("Y: "); Serial.println(y);
                Serial.print("Z: "); Serial.println(z);
                
                // if (moveToCoordinate(x, y, z)) {
                if (moveToCoordinate(x + 0.0f, y + 0.0f, z + 0.0f)) {
                    Serial.println("移動執行成功!");
                } else {
                    Serial.println("移動執行失敗!");
                }
            } else {
                Serial.println("格式無效! 請使用正確格式: x,y,z");
                Serial.println("例如: 100,50,150");
            }
        }
        else {
            Serial.println("未知命令");
        }
        
        Serial.println("\n輸入命令 (walk/next/stand) 或坐標 (x,y,z):");
    }
    delay(10);
}
