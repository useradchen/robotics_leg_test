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
#define DXL_ID_6  6    // Upper
#define DXL_ID_7  7    // Lower

DynamixelWorkbench dxl_wb;

uint8_t dxl_id[4] = {DXL_ID_3, DXL_ID_2, DXL_ID_6 , DXL_ID_7};
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

int32_t leftLegInitialPos[2] = {2048, 2048};  // left initial position
int32_t rightLegInitialPos[2] = {2048, 2048}; // right initial position

float currentSquatHeight = 20.0;  // Initial Squat Height = 20
float step_length = 0.0f; // 

// calculate required squatHeight
float calculateRequiredSquatHeight(float stepLength) {
    // Serial.print("distance in calculateRequiredSquatHeight = ");
    // Serial.println(stepLength);
    float squatIncrement = (stepLength * 0.5);  // stepLength/2
      Serial.println(" -------- send -------- ");
      Serial.println(squatIncrement);
      return squatIncrement; 
}

// IK
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

// conver
int32_t convertRadianToPosition(float radian) {
    float degree = radian * (180.0/M_PI);
    while (degree < 0) degree += 360;
    while (degree >= 360) degree -= 360;
    return (int32_t)((4096.0/360.0) * degree);
}

// move to coordinate
bool moveToCoordinate(float x, float y, float z, bool isLeftLeg = true) {
    isRelativeMove = true;
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
    int32_t currentPositions[4];  // 加入這行來讀取當前位置
    
    // 先讀取當前位置
    if (!readAllMotorPositions(currentPositions)) {
        Serial.println("Failed to read current motor positions!");
        return false;
    }
    
    if(isLeftLeg) {
        targetPositions[0] = convertRadianToPosition(-(legAngles.theta_hp+M_PI));
        targetPositions[1] = convertRadianToPosition(legAngles.theta_ap-M_PI);
        targetPositions[2] = currentPositions[2];
        targetPositions[3] = currentPositions[3];
    } else {
        targetPositions[0] = currentPositions[0];
        targetPositions[1] = currentPositions[1];
        targetPositions[2] = convertRadianToPosition(legAngles.theta_hp+M_PI);
        targetPositions[3] = convertRadianToPosition(-(legAngles.theta_ap-M_PI));
    }
    
    bool result = moveWithAdaptiveSpeed(targetPositions);
    
    if (result) {
        currentPos = targetPos;
    }
    
    return result;
}

// init pos
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
        
        // 計算初始下蹲位置的馬達角度
        IKAngles legAngles = calculateInverseKinematics(0, 0, 20.0);
        int32_t squatPositions[4] = {
            convertRadianToPosition(-(legAngles.theta_hp+M_PI)),  // 左腿上部
            convertRadianToPosition(legAngles.theta_ap-M_PI),     // 左腿下部 (because of 4096)
            convertRadianToPosition(legAngles.theta_hp+M_PI),  // 右腿上部
            convertRadianToPosition(-(legAngles.theta_ap-M_PI))      // 右腿下部
        };
        
        // 同時移動到初始下蹲位置
        result = dxl_wb.syncWrite(handler_index, squatPositions, &log);
        if (result) {
            currentSquatHeight = 20.0;
            delay(1000);
        } else {
            Serial.println("Failed to enter initial squat position!");
            return false;
        }
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

// 執行一個步態週期
float executeWalkingCycle(float step_length) {
  /***
    currentSquatHeight def in line  52 (float currentSquatHeight = 20.0)
  ***/
    step_length = step_length;
    
    // Serial.print("dis in executeWalkingCycle = ");
    // Serial.println(step_length);
    float currentX = 0.0f;
    float requiredSquatHeight = fabs(currentSquatHeight + calculateRequiredSquatHeight(step_length)); // 計算具體抬高多少距離(z)
    float requiredSquatLow = -fabs(requiredSquatHeight - calculateRequiredSquatHeight(step_length));

    // 計算抬腳高度（）
    Serial.print("currentSquatHeight = ");
    Serial.println(currentSquatHeight);

    Serial.println();
    Serial.print("requiredSquatHeight = ");
    Serial.println(requiredSquatHeight);

    Serial.println();
    Serial.print("requiredSquatLow = ");
    Serial.println(requiredSquatLow);

    delay(2000); // 避免動作切換過快 (by just test)
    // 左腿動(0,0,z)抬腳
    if(!moveToCoordinate(step_length, 0, requiredSquatHeight, true)) {
        Serial.println("Failed to lift left leg!");
        return 0.0; // false
    }
    delay(3000);
    Serial.println("\nz moved !");
    
    // just test z
    delay(2000);
    if(!moveToCoordinate(-step_length, 0, requiredSquatLow, true)) {
        Serial.println("Failed to lift left leg!");
        return 0.0; // false
    }
// =========================================================================================
    // // (x,0,0)前移
    // Serial.println("step_length x = ");
    // Serial.println(step_length);
    // // step_length
    // if(!moveToCoordinate(10, 0, 0 , true)) {
    //   Serial.println("Failed to extend left leg!");
    //   return 0.0; // false
    // }
    // delay(10000);
    // Serial.println("\nx moved !");
  
  // // // (0,0,-z)放下
    // if(!moveToCoordinate(0, 0, requiredSquatLow, true)) {
    //     Serial.println("Failed to lift left leg!");
    //     return 0.0; // false
    // }
    // delay(5000);
    // Serial.println("\nleft moved ok!");
//=========================================================================================
    // // 右腿動作序列 (0,0,z)
    // if(!moveToCoordinate(0 , 0 , currentSquatHeight + stepHeight , false)) {
    //     Serial.println("Failed to lift right leg!");
    //     return false;
    // }
    // delay(3000);
    
    // // (x,0,0)
    // if(!moveToCoordinate(walkParams.step_length , 0 , 0 , false)) {
    //     Serial.println("Failed to extend right leg!");
    //     return false;
    // }
    // delay(3000);
    
    // if(!moveToCoordinate(walkParams.step_length, 0, currentSquatHeight, false)) {
    //     Serial.println("Failed to lower right leg!");
    //     return false;
    // }
    // delay(1000);
    
    // currentX = walkParams.step_length * 2;
    
    // if(Serial.available() > 0) {
    //     char cmd = Serial.read();
    //     if(cmd == 'q' || cmd == 'Q') return false;
    // }
    
    return 1.0; //true
}

// 執行連續步行
bool startWalking(float step_length) {
    // Serial.print("distance in startWalking = ");
    // Serial.println(step_length);
    step_length = step_length;
    float currentSquatHeight = 20.0;
    // 計算下蹲z值
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
        if(!executeWalkingCycle(step_length)) {
            break;
        }
    }
    
    isRelativeMove = false;
    if(!moveToCoordinate(0, 0, 20.0, true) || 
       !moveToCoordinate(0, 0, 20.0, false)) {
        Serial.println("Failed to return to initial squat position!");
        return false;
    }
    // currentSquatHeight = 20.0;
    
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
            Serial.print("distance = ");
            Serial.println(distance);
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