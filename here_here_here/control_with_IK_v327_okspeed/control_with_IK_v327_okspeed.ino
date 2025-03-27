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

#ifndef BEZIER_LEG_TRAJECTORY_H
#define BEZIER_LEG_TRAJECTORY_H
class BezierLegTrajectory {
private:
    // 3D點的結構
    struct Point3D {
        float x, y, z;
    };

    // 計算貝茲曲線上的點
    static Point3D calculateBezierPoint(float t, const Point3D controlPoints[4]) {
        Point3D result = {0, 0, 0};
        float b0 = pow(1 - t, 3);
        float b1 = 3 * t * pow(1 - t, 2);
        float b2 = 3 * pow(t, 2) * (1 - t);
        float b3 = pow(t, 3);

        result.x = b0 * controlPoints[0].x +
                   b1 * controlPoints[1].x +
                   b2 * controlPoints[2].x +
                   b3 * controlPoints[3].x;

        result.y = b0 * controlPoints[0].y +
                   b1 * controlPoints[1].y +
                   b2 * controlPoints[2].y +
                   b3 * controlPoints[3].y;

        result.z = b0 * controlPoints[0].z +
                   b1 * controlPoints[1].z +
                   b2 * controlPoints[2].z +
                   b3 * controlPoints[3].z;

        return result;
    }

public:
    // 生成平滑軌跡的靜態方法
    static bool generateSmoothTrajectory(
        Position currentPos, 
        Position targetPos, 
        bool isLeftLeg, 
        void (*moveCallback)(float, float, float, bool)
    ) {
        // 定義4個控制點
        Point3D controlPoints[4];
        
        // 起點
        controlPoints[0] = {currentPos.x, currentPos.y, currentPos.z};
        
        // 第一個控制點：稍微偏離起點
        controlPoints[1] = {
            currentPos.x + (targetPos.x - currentPos.x) * 0.3f,
            currentPos.y + (targetPos.y - currentPos.y) * 0.3f,
            currentPos.z + (targetPos.z - currentPos.z) * 0.3f
        };
        
        // 第二個控制點：接近終點
        controlPoints[2] = {
            targetPos.x - (targetPos.x - currentPos.x) * 0.3f,
            targetPos.y - (targetPos.y - currentPos.y) * 0.3f,
            targetPos.z - (targetPos.z - currentPos.z) * 0.3f
        };
        
        // 終點
        controlPoints[3] = {targetPos.x, targetPos.y, targetPos.z};

        // 插值點數量
        const int numInterpolationPoints = 20;

        // 生成軌跡點並回調
        for (int i = 0; i <= numInterpolationPoints; ++i) {
            float t = (float)i / numInterpolationPoints;
            Point3D interpolatedPoint = calculateBezierPoint(t, controlPoints);
            
            // 使用回調函數移動
            moveCallback(
                interpolatedPoint.x, 
                interpolatedPoint.y, 
                interpolatedPoint.z, 
                isLeftLeg
            );

            // 可以添加一個小延遲來控制移動速度
            delay(10);
        }

        return true;
    }
};

// 在 moveToCoordinate 中修改使用
bool moveToCoordinate(float x, float y, float z, bool isLeftLeg = true) {
    // 保留原有的邏輯
    isRelativeMove = true;
    targetPos.x = isRelativeMove ? currentPos.x + x : x;
    targetPos.y = isRelativeMove ? currentPos.y + y : y;
    targetPos.z = isRelativeMove ? currentPos.z + z : z;
    
    // 定義回調函數類型
    using MoveCallback = void (*)(float, float, float, bool);
    
    // 創建一個靜態的內部移動函數作為回調
    static MoveCallback internalMoveCallback = [](float x, float y, float z, bool isLeftLeg) {
        // 這裡直接調用原來的 IK 和同步寫入邏輯
        IKAngles legAngles = calculateInverseKinematics(x, isLeftLeg ? y : -y, z);
        
        int32_t targetPositions[4];
        int32_t currentPositions[4];  
        
        if (!readAllMotorPositions(currentPositions)) {
            Serial.println("Failed to read current motor positions!");
            return;
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
        
        const char *log;
        dxl_wb.syncWrite(handler_index, targetPositions, &log);
    };

    // 使用貝茲曲線生成平滑軌跡
    Position currentPosition = {currentPos.x, currentPos.y, currentPos.z};
    Position targetPosition = {targetPos.x, targetPos.y, targetPos.z};
    
    bool result = BezierLegTrajectory::generateSmoothTrajectory(
        currentPosition, 
        targetPosition, 
        isLeftLeg, 
        internalMoveCallback
    );
    
    if (result) {
        currentPos = targetPos;
    }
    
    return result;
}

#endif // BEZIER_LEG_TRAJECTORY_H

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
            convertRadianToPosition(legAngles.theta_ap-M_PI),     // 左腿下部 (because of )
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

// 初始化Dynamixel
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
    
    if(Serial.available() > 0) {
        char cmd = Serial.read();
        if(cmd == 'q' || cmd == 'Q') 
        return false;
    }
    
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