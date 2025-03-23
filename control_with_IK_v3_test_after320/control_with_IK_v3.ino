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

// 步行參數結構體
struct WalkingParams {
    float step_length;      // 步長
    float step_height;      // 抬腳高度
    float squat_height;     // 下蹲高度
    float cycle_time;       // 一個步態週期的時間
    bool is_left_support;   // 是否左腳支撐
};

// 貝塞爾曲線控制點結構體
struct BezierPoint {
    float x;
    float y;
    float z;
};

// 全局步行參數
WalkingParams walkParams = {
    .step_length = 30.0,    // 默認步長30mm
    .step_height = 30.0,    // 默認抬腳高度30mm
    .squat_height = 20.0,   // 默認下蹲高度20mm
    .cycle_time = 1.0,      // 一個步態週期1秒
    .is_left_support = true // 默認從左腳支撐開始
};

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

// 初始化位置
bool initializePosition() {
    const char *log;
    int32_t middlePositions[4] = {2048, 2048, 2048, 2048};
    
    bool result = dxl_wb.syncWrite(handler_index, middlePositions, &log);
    if (result) {
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

// 移动到初始位置
bool moveToInitialPosition() {
    if (!initializePosition()) {
        return false;
    }
    
    isRelativeMove = false;
    if (!moveToCoordinate(0, 0, 20)) {
        Serial.println("Initial micro squat movement failed!");
        return false;
    }
    
    return true;
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

// 移动到指定坐标
bool moveToCoordinate(float x, float y, float z) {
    targetPos.x = isRelativeMove ? currentPos.x + x : x;
    targetPos.y = isRelativeMove ? currentPos.y + y : y;
    targetPos.z = isRelativeMove ? currentPos.z + z : z;
    
    IKAngles leftLegAngles = calculateInverseKinematics(targetPos.x, targetPos.y, targetPos.z);
    IKAngles rightLegAngles = calculateInverseKinematics(targetPos.x, -targetPos.y, targetPos.z);
    
    int32_t targetPositions[4] = {
        convertRadianToPosition(-(leftLegAngles.theta_hp+M_PI)),
        convertRadianToPosition(leftLegAngles.theta_ap-M_PI),
        convertRadianToPosition(-(rightLegAngles.theta_hp+M_PI)),
        convertRadianToPosition(rightLegAngles.theta_ap-M_PI)
    };
    
    bool result = moveWithAdaptiveSpeed(targetPositions);
    
    if (result) {
        currentPos = targetPos;
    }
    
    return result;
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
    const int STEPS = 50;  // 一個週期的採樣點數
    float dt = walkParams.cycle_time / STEPS;
    
    // 設置支撐腳和擺動腳的起始位置（使用浮點數）
    Position supportLeg = {0.0f, walkParams.is_left_support ? 5.0f : -5.0f, walkParams.squat_height};
    Position swingLeg = {0.0f, walkParams.is_left_support ? -5.0f : 5.0f, walkParams.squat_height};
    
    // 定義擺動腳的貝塞爾曲線控制點
    BezierPoint p0 = {swingLeg.x, swingLeg.y, swingLeg.z};  // 起始點
    BezierPoint p1 = {swingLeg.x + walkParams.step_length/3.0f, swingLeg.y, swingLeg.z + walkParams.step_height}; // 第一控制點
    BezierPoint p2 = {swingLeg.x + 2.0f*walkParams.step_length/3.0f, swingLeg.y, swingLeg.z + walkParams.step_height}; // 第二控制點
    BezierPoint p3 = {swingLeg.x + walkParams.step_length, swingLeg.y, swingLeg.z}; // 終點
    
    // 執行擺動腳的運動
    for(int i = 0; i < STEPS; i++) {
        float t = (float)i / (STEPS - 1);
        Position swingPos = calculateBezierPoint(p0, p1, p2, p3, t);
        
        // 移動雙腿
        if(walkParams.is_left_support) {
            if(!moveToCoordinate(supportLeg.x, supportLeg.y, supportLeg.z)) return false;
            if(!moveToCoordinate(swingPos.x, swingPos.y, swingPos.z)) return false;
        } else {
            if(!moveToCoordinate(swingPos.x, swingPos.y, swingPos.z)) return false;
            if(!moveToCoordinate(supportLeg.x, supportLeg.y, supportLeg.z)) return false;
        }
        
        delay(dt * 1000);  // 轉換為毫秒
        
        // 檢查是否收到停止命令
        if(Serial.available() > 0) {
            char cmd = Serial.read();
            if(cmd == 'q' || cmd == 'Q') return false;
        }
    }
    
    // 切換支撐腳
    walkParams.is_left_support = !walkParams.is_left_support;
    return true;
}

// 執行連續步行
bool startWalking(float step_length) {
    // 更新步長
    walkParams.step_length = step_length;
    
    // 先進入初始下蹲姿態
    isRelativeMove = false;
    if(!moveToCoordinate(0, 0, walkParams.squat_height)) {
        Serial.println("Failed to enter initial squat position!");
        return false;
    }
    
    Serial.println("Starting to walk...");
    Serial.println("Press 'q' to stop");
    
    // 開始連續步行
    while(true) {
        if(!executeWalkingCycle()) {
            break;
        }
    }
    
    // 返回安全姿態
    isRelativeMove = false;
    if(!moveToCoordinate(0, 0, walkParams.squat_height)) {
        Serial.println("Failed to return to safe position!");
        return false;
    }
    
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

    if (!moveToInitialPosition()) {
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
//Att// 