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

// 基本结构体定义
struct Coordinates {
    float x;
    float y;
    float z;
};

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

struct MovementParameters {
    float squat_height;    // 下蹲高度
    float lift_height;     // 抬腿高度
    float cycle_time;      // 周期时间
};

struct TrajectoryPoint {
    float x;
    float z;
};

// 全局变量
Position currentPos = {0, 0, 0};
Position targetPos = {0, 0, 0};
bool isRelativeMove = true;

// 基本运动学计算函数
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

int32_t convertRadianToPosition(float radian) {
    float degree = radian * (180.0/M_PI);
    while (degree < 0) degree += 360;
    while (degree >= 360) degree -= 360;
    return (int32_t)((4096.0/360.0) * degree);
}

// 初始化相关函数
bool setMiddlePosition() {
    const char *log;
    int32_t middlePositions[4] = {2048, 2048, 2048, 2048};
    
    Serial.println("设置所有电机到中间位置(2048)...");
    bool result = dxl_wb.syncWrite(handler_index, middlePositions, &log);
    
    if (result) {
        Serial.println("已设置到中间位置！");
        delay(1000);
    } else {
        Serial.println(log);
        Serial.println("设置中间位置失败！");
    }
    
    return result;
}

bool initializeDynamixels() {
    const char *log;
    bool result = false;

    result = dxl_wb.init(DEVICE_NAME, BAUDRATE, &log);
    if (result == false) {
        Serial.println(log);
        Serial.println("初始化失败");
        return false;
    }

    for (int i = 0; i < 4; i++) {
        uint16_t model_number = 0;
        result = dxl_wb.ping(dxl_id[i], &model_number, &log);
        if (result == false) {
            Serial.println(log);
            return false;
        }

        result = dxl_wb.jointMode(dxl_id[i], 0, 0, &log);
        if (result == false) {
            Serial.println(log);
            return false;
        }

        result = dxl_wb.torqueOn(dxl_id[i], &log);
        if (result == false) {
            Serial.println(log);
            return false;
        }
    }

    result = dxl_wb.addSyncWriteHandler(dxl_id[0], "Goal_Position", &log);
    if (result == false) {
        Serial.println(log);
        return false;
    }

    return true;
}

bool moveToInitialPosition() {
    if (!setMiddlePosition()) {
        return false;
    }
    delay(1000);
    
    Serial.println("执行初始微蹲动作...");
    isRelativeMove = false;
    if (!moveToCoordinate(0, 0, 20)) {
        Serial.println("初始微蹲动作失败！");
        return false;
    }
    delay(500);
    
    Serial.println("初始化位置设置完成！");
    return true;
}

// 运动控制函数
bool readAllMotorPositions(int32_t* positions) {
    const char *log;
    bool result = true;
    
    for (int i = 0; i < 4; i++) {
        // 添加重试机制
        int retryCount = 0;
        const int maxRetries = 3;
        
        while (retryCount < maxRetries) {
            // 清空串口缓冲区
            while(Serial.available()) {
                Serial.read();
            }
            
            result = dxl_wb.itemRead(dxl_id[i], "Present_Position", &positions[i], &log);
            if (result) {
                break;
            }
            retryCount++;
            delay(20);  // 增加重试延迟到20ms
        }
        
        if (!result) {
            Serial.print("读取电机 ");
            Serial.print(dxl_id[i]);
            Serial.println(" 位置失败！");
            return false;
        }
        delay(10);  // 每个电机读取之间添加10ms延迟
    }
    
    return true;
}

bool moveWithAdaptiveSpeed(int32_t targetPositions[4]) {
    const char *log;
    int32_t currentPositions[4];
    
    // 添加重试机制读取当前位置
    int positionRetryCount = 0;
    const int maxPositionRetries = 3;
    bool readSuccess = false;
    
    while (positionRetryCount < maxPositionRetries) {
        if (readAllMotorPositions(currentPositions)) {
            readSuccess = true;
            break;
        }
        positionRetryCount++;
        delay(10);
    }
    
    if (!readSuccess) {
        Serial.println("多次尝试读取位置均失败！");
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

        // 添加写入重试机制
        int writeRetryCount = 0;
        const int maxWriteRetries = 3;
        bool writeSuccess = false;
        
        while (writeRetryCount < maxWriteRetries) {
            bool result = dxl_wb.syncWrite(handler_index, intermediatePositions, &log);
            if (result) {
                writeSuccess = true;
                break;
            }
            writeRetryCount++;
            delay(5);
        }
        
        if (!writeSuccess) {
            Serial.println("多次尝试写入位置均失败！");
            return false;
        }

        delay(2);  // 增加延迟，给电机更多响应时间
    }

    // 最后一次位置写入也添加重试机制
    int finalWriteRetryCount = 0;
    const int maxFinalWriteRetries = 3;
    bool finalWriteSuccess = false;
    
    while (finalWriteRetryCount < maxFinalWriteRetries) {
        bool result = dxl_wb.syncWrite(handler_index, targetPositions, &log);
        if (result) {
            finalWriteSuccess = true;
            break;
        }
        finalWriteRetryCount++;
        delay(5);
    }
    
    if (!finalWriteSuccess) {
        Serial.println("最终位置写入失败！");
        return false;
    }

    delay(5);
    return true;
}

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

// 运动参数计算
MovementParameters calculateParameters(float distance) {
    MovementParameters params;
    
    // 基础值
    const float BASE_SQUAT = 15.0;
    const float BASE_LIFT = 20.0;
    const float BASE_TIME = 150.0;
    
    // 计算下蹲高度 (使用线性插值而不是对数)
    float squat_factor = distance / 100.0;  // 将距离标准化到0-1范围
    params.squat_height = BASE_SQUAT + (25.0 * squat_factor);
    
    // 计算抬腿高度 (基于下蹲高度)
    params.lift_height = params.squat_height * 1.2f + (distance * 0.3f);
    
    // 计算周期时间 (线性关系)
    params.cycle_time = BASE_TIME + (distance * 5.0f);
    
    // 限制范围
    if (params.squat_height < 15.0f) params.squat_height = 15.0f;
    if (params.squat_height > 40.0f) params.squat_height = 40.0f;
    
    if (params.lift_height < 20.0f) params.lift_height = 20.0f;
    if (params.lift_height > 60.0f) params.lift_height = 60.0f;
    
    if (params.cycle_time < 150.0f) params.cycle_time = 150.0f;
    if (params.cycle_time > 500.0f) params.cycle_time = 500.0f;
    
    return params;
}

// 添加椭圆轨迹计算函数
TrajectoryPoint calculateEllipticalTrajectory(float phase, float a, float b, bool isPushLeg) {
    TrajectoryPoint point;
    
    if (isPushLeg) {
        // 支撑腿（后推腿）的轨迹
        if (phase < M_PI) {
            float t = phase / M_PI;
            float pushForce = pow(sin(t * M_PI), 1.5);  // 使用1.5次方使后推力更强
            point.x = -a * 1.2 * pushForce;  // 增加后推距离
            point.z = -b * 0.1 * pushForce;  // 轻微下蹲以增加推力
        } else {
            float t = (phase - M_PI) / M_PI;
            float recovery = 1 - pow(t, 2);
            point.x = -a * 1.2 * recovery;
            point.z = 0;
        }
    } else {
        // 摆动腿（抬起前移的腿）的轨迹
        if (phase < M_PI) {
            float t = phase / M_PI;
            // 更明显的抬腿动作
            float liftCurve = pow(sin(t * M_PI), 1.2);  // 使用1.2次方使抬腿更明显 
            point.x = a * sin(t * M_PI);
            point.z = b * 1.2 * liftCurve;  // 增加抬腿高度
        } else {
            float t = (phase - M_PI) / M_PI;
            // 更自然的落地过程
            float landingCurve = 1 - pow(t, 1.5);  // 使用1.5次方使落地更自然
            point.x = a * landingCurve;
            point.z = b * 0.2 * landingCurve;  // 保持一定高度直到最后
        }
    }
    
    return point;
}

// 修改原地跑动功能
bool runningInPlace(float distance) {
    Serial.println("\n=== 开始前进步行 ===");
    
    MovementParameters params = calculateParameters(distance);
    
    // 设置运动参数
    float a = distance * 0.5;  // 从0.6改为0.5或更小的值
    float b = params.lift_height * 1.2;  // 从1.5改为1.2或更小的值
    
    isRelativeMove = false;
    // 添加重试机制
    int initRetryCount = 0;
    const int maxInitRetries = 3;
    bool initSuccess = false;
    
    while (initRetryCount < maxInitRetries) {
        if (moveToCoordinate(0, 0, params.squat_height)) {
            initSuccess = true;
            break;
        }
        initRetryCount++;
        delay(100);
    }
    
    if (!initSuccess) {
        Serial.println("无法完成初始下蹲动作！");
        return false;
    }
    
    delay(200);
    
    bool running = true;
    float phase = 0.0;
    const float phaseIncrement = 0.04;  // 从0.02改为0.04，加快一倍速度 (0.04,0.06)
    bool isLeftLegMoving = true;  // 标记当前是左腿还是右腿在移动
    
    while (running) {
        // 修改检测停止命令的方式
        if (Serial.available() > 0) {
            char input = Serial.read();
            if (input == 'q' || input == 'Q') {
                Serial.println("收到停止命令！");
                running = false;
                break;
            }
        }
        
        // 计算当前移动腿的轨迹点
        TrajectoryPoint movingLegPoint;
        TrajectoryPoint supportLegPoint;
        
        if (isLeftLegMoving) {
            // 左腿移动，右腿支撑
            movingLegPoint = calculateEllipticalTrajectory(phase, a, b, false);  // 左腿（摆动腿）
            supportLegPoint = calculateEllipticalTrajectory(phase, a, b, true);  // 右腿（支撑腿）
        } else {
            // 右腿移动，左腿支撑
            movingLegPoint = calculateEllipticalTrajectory(phase, a, b, false);  // 右腿（摆动腿）
            supportLegPoint = calculateEllipticalTrajectory(phase, a, b, true);  // 左腿（支撑腿）
        }
        
        // 计算实际的腿部位置
        float movingLegHeight = params.squat_height + movingLegPoint.z;
        float supportLegHeight = params.squat_height + supportLegPoint.z;
        
        // 计算左右腿的逆运动学
        IKAngles leftAngles, rightAngles;
        if (isLeftLegMoving) {
            leftAngles = calculateInverseKinematics(movingLegPoint.x, 0, movingLegHeight);
            rightAngles = calculateInverseKinematics(supportLegPoint.x, 0, supportLegHeight);
        } else {
            leftAngles = calculateInverseKinematics(supportLegPoint.x, 0, supportLegHeight);
            rightAngles = calculateInverseKinematics(movingLegPoint.x, 0, movingLegHeight);
        }
        
        // 设置电机位置
        int32_t targetPositions[4] = {
            convertRadianToPosition(-(leftAngles.theta_hp+M_PI)),
            convertRadianToPosition(leftAngles.theta_ap-M_PI),
            convertRadianToPosition(-(rightAngles.theta_hp+M_PI)),
            convertRadianToPosition(rightAngles.theta_ap-M_PI)
        };
        
        if (!moveWithAdaptiveSpeed(targetPositions)) {
            Serial.println("运动执行失败！");
            running = false;
            break;
        }
        
        phase += phaseIncrement;
        if (phase >= 2 * M_PI) {
            phase = 0.0;  // 重置相位
            isLeftLegMoving = !isLeftLegMoving;  // 切换移动腿
        }
        
        delay(1);  // 减少延迟时间，使检测更灵敏
    }
    
    // 确保安全停止
    isRelativeMove = false;
    if (!moveToCoordinate(0, 0, 20)) {
        Serial.println("无法回到微蹲状态！");
        return false;
    }
    delay(200);
    
    Serial.println("\n=== 结束前进步行 ===");
    return true;
}

void setup() {
    Serial.begin(57600);
    delay(2000);

    Serial.println("开始初始化...");
    if (!initializeDynamixels()) {
        Serial.println("电机初始化失败！");
        while(1);
    }

    Serial.println("电机初始化完成！");
    
    if (!moveToInitialPosition()) {
        Serial.println("初始位置设置失败！");
        while(1);
    }
    
    isRelativeMove = true;
    
    Serial.println("\n=== 可用命令 ===");
    Serial.println("原地跑动: run,distance (按Q停止)");
    Serial.println("\n准备接收命令:");
}

void loop() {
    if (Serial.available() > 0) {
        String input = Serial.readStringUntil('\n');
        input.trim();
        
        if (input.startsWith("run,")) {
            input = input.substring(4);
            float distance = input.toFloat();
            
            if (distance > 0) {
                Serial.print("\n开始原地跑动，参考距离: ");
                Serial.println(distance);
                
                if (runningInPlace(distance)) {
                    Serial.println("原地跑动完成！");
                } else {
                    Serial.println("原地跑动失败！");
                }
            } else {
                Serial.println("无效的距离参数！");
            }
            
            Serial.println("\n准备接收下一个命令:");
        } else if (input == "q" || input == "Q") {
            Serial.println("收到停止命令！");
        } else {
            Serial.println("无效的命令！请使用: run,distance");
        }
    }
    delay(10);
}
//AAAAtt// 