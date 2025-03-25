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
#define DXL_ID_6  6    //  Lower
#define DXL_ID_7  7    // Upper 

DynamixelWorkbench dxl_wb;

uint8_t dxl_id[4] = {DXL_ID_3, DXL_ID_2, DXL_ID_7 , DXL_ID_6};
const uint8_t handler_index = 0;
const float LENGTH = 210.0; // leg length
// 在所有函數實現之前添加函數原型

struct IKAngles {
    float theta_hp;
    float theta_ap;
    float theta_ar;
};

struct Position {
    float x;
    float y;
    float z;
};

Position currentPos = {0, 0, 0};
Position targetPos = {0, 0, 0};
bool isRelativeMove = true;

// 簡化步行參數結構體
struct WalkingParams {
    float step_length;      // 步長
    float squat_height;     // 下蹲高度
    float cycle_time;       // 一個步態週期的時間
};

// 貝塞爾曲線控制點結構體
struct BezierPoint {
    float x;
    float y;
    float z;
};

// 全局步行參數
WalkingParams walkParams = {
    .step_length = 20.0,    // 默認步長20mm
    .squat_height = 5.0,    // 下蹲高度5mm
    .cycle_time = 2.0       // 一個步態週期2秒
};

// 添加全局变量来存储每条腿的初始位置
int32_t leftLegInitialPos[2] = {2048, 2048};  // 左腿的初始位置
int32_t rightLegInitialPos[2] = {2048, 2048}; // 右腿的初始位置

// 添加新的全局變量來跟踪實際下蹲高度
float currentSquatHeight = 20.0;  // 初始微蹲高度

// 計算所需的下蹲高度
float calculateRequiredSquatHeight(float stepLength) {
    float squatIncrement = stepLength * 0.5;  // 每10mm步長增加5mm下蹲
    if (squatIncrement > 20.0) {  // 如果計算出的增量大於初始下蹲高度
        return squatIncrement;  // 則使用計算值作為新的下蹲高度
    }
    return 20.0;  // 否則保持20mm的初始下蹲高度
}

// 計算逆运动学
IKAngles calculateInverseKinematics(float x, float y, float z) {
    float l = LENGTH;
    IKAngles angles;
    
    float theta_hp1 = atan2(x, (2*l - z));
    float d = sqrt(pow(2*l - z, 2) + pow(x, 2));
    float theta_hp2 = acos(d / (2*l));
    angles.theta_hp = theta_hp1 + theta_hp2;
    angles.theta_ap = acos(fabs(x) / d) + theta_hp2 - M_PI/2;
    angles.theta_ar = atan2(y, (2*l - z));
    
    return angles;
}

// 角度转换为电机位置
int32_t convertRadianToPosition(float radian) {
    float degree = radian * (180.0/M_PI);
    while (degree < 0) degree += 360;
    while (degree >= 360) degree -= 360;
    return (int32_t)((4096.0/360.0) * degree);
}

// 移动到指定坐标
bool moveToCoordinate(float x, float y, float z, bool isLeftLeg = true) {
    targetPos.x = isRelativeMove ? currentPos.x + x : x;
    targetPos.y = isRelativeMove ? currentPos.y + y : y;
    targetPos.z = isRelativeMove ? currentPos.z + z : z;
    
    IKAngles legAngles;
    
    if(isLeftLeg) {
        legAngles = calculateInverseKinematics(targetPos.x, targetPos.y, targetPos.z);
    } else {
        legAngles = calculateInverseKinematics(targetPos.x, -targetPos.y, targetPos.z);
    }
    
    int32_t targetPositions[4];
    if(isLeftLeg) {
        targetPositions[0] = convertRadianToPosition(-(legAngles.theta_hp+M_PI));
        targetPositions[1] = convertRadianToPosition(legAngles.theta_ap-M_PI);
        targetPositions[2] = rightLegInitialPos[0];
        targetPositions[3] = rightLegInitialPos[1];
    } else {
        targetPositions[0] = leftLegInitialPos[0];
        targetPositions[1] = leftLegInitialPos[1];
        targetPositions[2] = convertRadianToPosition(-(legAngles.theta_hp+M_PI));
        targetPositions[3] = convertRadianToPosition(legAngles.theta_ap-M_PI);
    }
    
    bool result = moveWithAdaptiveSpeed(targetPositions);
    
    if (result) {
        currentPos = targetPos;
    }
    
    return result;
}

// 初始化位置
bool initializePosition() {
    const char *log;
    int32_t middlePositions[4] = {2048, 2048, 2048, 2048};
    
    bool result = dxl_wb.syncWrite(handler_index, middlePositions, &log);
    if (result) {
        leftLegInitialPos[0] = middlePositions[0];
        leftLegInitialPos[1] = middlePositions[1];
        rightLegInitialPos[0] = middlePositions[2];
        rightLegInitialPos[1] = middlePositions[3];
        delay(1000);
        
        isRelativeMove = false;
        if(!moveToCoordinate(0, 0, 20.0, true) || 
           !moveToCoordinate(0, 0, 20.0, false)) {
            Serial.println("Failed to enter initial squat position!");
            return false;
        }
        currentSquatHeight = 20.0;
        delay(1000);
    } else {
        Serial.println("Failed to set initial position");
    }
    return result;
}

// 初始化Dynamixel电机
bool initializeDynamixels() {
    const char *log;
    bool result = false;

    result = dxl_wb.init(DEVICE_NAME, BAUDRATE, &log);
    if (!result) {
        Serial.println("Initialization failed");
        return false;
    }

    for (int i = 0; i < 4; i++) {
        uint16_t model_number = 0;
        if (!dxl_wb.ping(dxl_id[i], &model_number, &log) ||
            !dxl_wb.jointMode(dxl_id[i], 0, 0, &log) ||
            !dxl_wb.torqueOn(dxl_id[i], &log)) {
            return false;
        }

        dxl_wb.itemWrite(dxl_id[i], "Position_P_Gain", 2000);
        dxl_wb.itemWrite(dxl_id[i], "Position_I_Gain", 100);
        dxl_wb.itemWrite(dxl_id[i], "Position_D_Gain", 100);
        dxl_wb.itemWrite(dxl_id[i], "Moving_Speed", 200);
        dxl_wb.itemWrite(dxl_id[i], "Moving_Threshold", 10);
    }

    return dxl_wb.addSyncWriteHandler(dxl_id[0], "Goal_Position", &log);
}

// 读取所有电机位置
bool readAllMotorPositions(int32_t* positions) {
    const char *log;
    
    for (int i = 0; i < 4; i++) {
        int retryCount = 0;
        while (retryCount < 3) {
            if (dxl_wb.itemRead(dxl_id[i], "Present_Position", &positions[i], &log)) {
                break;
            }
            retryCount++;
            delay(20);
        }
        if (retryCount >= 3) {
            return false;
        }
    }
    return true;
}

// 自适应速度移动
bool moveWithAdaptiveSpeed(int32_t targetPositions[4]) {
    const char *log;
    int32_t currentPositions[4];
    
    if (!readAllMotorPositions(currentPositions)) {
        return false;
    }
    
    int32_t maxDiff = 0;
    for (int i = 0; i < 4; i++) {
        int32_t diff = abs(targetPositions[i] - currentPositions[i]);
        if (diff > maxDiff) maxDiff = diff;
    }
    
    if (maxDiff < 5) {
        return true;
    }
    
    int totalSteps = maxDiff / 30;
    if (totalSteps < 10) totalSteps = 10;
    if (totalSteps > 30) totalSteps = 30;
    
    int32_t intermediatePositions[4];
    
    for (int step = 0; step < totalSteps; step++) {
        float t = (float)step / (totalSteps - 1);
        float progress = (1 - cos(t * M_PI)) / 2;
        
        for (int i = 0; i < 4; i++) {
            intermediatePositions[i] = currentPositions[i] + 
                (int32_t)((targetPositions[i] - currentPositions[i]) * progress);
        }

        int writeRetryCount = 0;
        while (writeRetryCount < 3) {
            if (dxl_wb.syncWrite(handler_index, intermediatePositions, &log)) {
                break;
            }
            writeRetryCount++;
            delay(5);
        }
        
        if (writeRetryCount >= 3) {
            return false;
        }

        delay(5);
    }

    return dxl_wb.syncWrite(handler_index, targetPositions, &log);
}

// 計算貝塞爾曲線點
Position calculateBezierPoint(BezierPoint p0, BezierPoint p1, BezierPoint p2, BezierPoint p3, float t) {
    float mt = 1 - t;
    float mt2 = mt * mt;
    float mt3 = mt2 * mt;
    float t2 = t * t;
    float t3 = t2 * t;
    
    Position result;
    result.x = mt3 * p0.x + 3 * mt2 * t * p1.x + 3 * mt * t2 * p2.x + t3 * p3.x;
    result.y = mt3 * p0.y + 3 * mt2 * t * p1.y + 3 * mt * t2 * p2.y + t3 * p3.y;
    result.z = mt3 * p0.z + 3 * mt2 * t * p1.z + 3 * mt * t2 * p2.z + t3 * p3.z;
    
    return result;
}

// 執行一個步態週期
bool executeWalkingCycle() {
    float currentX = 0.0f;
    
    float requiredSquatHeight = calculateRequiredSquatHeight(walkParams.step_length);
    if (requiredSquatHeight > currentSquatHeight) {
        isRelativeMove = false;
        if(!moveToCoordinate(0, 0, requiredSquatHeight, true) || 
           !moveToCoordinate(0, 0, requiredSquatHeight, false)) {
            Serial.println("Failed to adjust squat height!");
            return false;
        }
        currentSquatHeight = requiredSquatHeight;
        delay(500);
    }
    
    // 計算抬腳高度（步長的一半）
    float stepHeight = walkParams.step_length * 0.5;
    
    // 左腿動作序列
    if(!moveToCoordinate(0, 0, currentSquatHeight + stepHeight, true)) {
        Serial.println("Failed to lift left leg!");
        return false;
    }
    delay(300);
    
    if(!moveToCoordinate(walkParams.step_length, 0, currentSquatHeight + stepHeight, true)) {
        Serial.println("Failed to extend left leg!");
        return false;
    }
    delay(300);
    
    if(!moveToCoordinate(walkParams.step_length, 0, currentSquatHeight, true)) {
        Serial.println("Failed to lower left leg!");
        return false;
    }
    delay(500);
    
    // 右腿動作序列
    if(!moveToCoordinate(walkParams.step_length, 0, currentSquatHeight + stepHeight, false)) {
        Serial.println("Failed to lift right leg!");
        return false;
    }
    delay(300);
    
    if(!moveToCoordinate(walkParams.step_length * 2, 0, currentSquatHeight + stepHeight, false)) {
        Serial.println("Failed to extend right leg!");
        return false;
    }
    delay(300);
    
    if(!moveToCoordinate(walkParams.step_length * 2, 0, currentSquatHeight, false)) {
        Serial.println("Failed to lower right leg!");
        return false;
    }
    delay(500);
    
    currentX = walkParams.step_length * 2;
    
    if(Serial.available() > 0) {
        char cmd = Serial.read();
        if(cmd == 'q' || cmd == 'Q') return false;
    }
    
    return true;
}

// 執行連續步行
bool startWalking(float step_length) {
    walkParams.step_length = step_length;
    
    float requiredSquatHeight = calculateRequiredSquatHeight(step_length);
    
    if (requiredSquatHeight > currentSquatHeight) {
        isRelativeMove = false;
        if(!moveToCoordinate(0, 0, requiredSquatHeight, true) || 
           !moveToCoordinate(0, 0, requiredSquatHeight, false)) {
            Serial.println("Failed to adjust squat height!");
            return false;
        }
        currentSquatHeight = requiredSquatHeight;
        delay(500);
    }
    
    Serial.println("Starting to walk...");
    Serial.println("Press 'q' to stop");
    
    while(true) {
        if(!executeWalkingCycle()) {
            break;
        }
    }
    
    isRelativeMove = false;
    if(!moveToCoordinate(0, 0, 20.0, true) || 
       !moveToCoordinate(0, 0, 20.0, false)) {
        Serial.println("Failed to return to initial squat position!");
        return false;
    }
    currentSquatHeight = 20.0;
    
    return true;
}

void setup() {
    Serial.begin(57600);
    delay(2000);

    Serial.println("=== Program Start ===");
    
    if (!initializeDynamixels()) {
        Serial.println("Motor initialization failed!");
        while(1);
    }
    
    if (!initializePosition()) {
        Serial.println("Initial position setting failed!");
        while(1);
    }
    
    isRelativeMove = true;
    Serial.println("Ready for commands:");
}

void loop() {
    if (Serial.available() > 0) {
        String input = Serial.readStringUntil('\n');
        input.trim();
        
        if (input.startsWith("move,")) {
            String coords = input.substring(5);
            int firstComma = coords.indexOf(',');
            int secondComma = coords.indexOf(',', firstComma + 1);
            
            if (firstComma != -1 && secondComma != -1) {
                float x = coords.substring(0, firstComma).toFloat();
                float y = coords.substring(firstComma + 1, secondComma).toFloat();
                float z = coords.substring(secondComma + 1).toFloat();
                
                if (moveToCoordinate(x, y, z)) {
                    Serial.println("Movement completed!");
                } else {
                    Serial.println("Movement failed!");
                }
            }
        }
        else if (input.startsWith("run,")) {
            float distance = input.substring(4).toFloat();
            if (distance > 0) {
                if (startWalking(distance)) {
                    Serial.println("Walking completed!");
                } else {
                    Serial.println("Walking stopped!");
                }
            } else {
                Serial.println("Invalid distance!");
            }
        }
    }
    delay(10);
}
//tt1// 