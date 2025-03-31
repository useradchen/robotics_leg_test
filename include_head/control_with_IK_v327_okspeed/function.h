#include <math.h>
#include <DynamixelWorkbench.h>

float LENGTH = 210.0; // leg length

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

#define BAUDRATE  1000000
#define DXL_ID_3  3    // Upper 
#define DXL_ID_2  2    // Lower 
#define DXL_ID_6  6    // Upper
#define DXL_ID_7  7    // Lower

DynamixelWorkbench dxl_wb;

uint8_t dxl_id[4] = {DXL_ID_3, DXL_ID_2, DXL_ID_6 , DXL_ID_7};
const uint8_t handlerIndex = 0;

Position currentPos = {0, 0, 0}; // current position
Position targetPos = {0, 0, 0};  // target position

IKAngles calculateInverseKinematics(float x, float y, float z) {
    /*
    	use x(front and back) y(left and right) z(up and down) calculate inverse kinematics
    */
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
	/*
		Map the input to 4096 
	*/
    float degree = radian * (180.0/M_PI);
    while (degree < 0) degree += 360;
    while (degree >= 360) degree -= 360;
    return (int32_t)((4096.0/360.0) * degree);
}

bool readAllActuatorPositions(int32_t* positions) {
    /*
		
    */
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

#ifndef BEZIER_LEG_TRAJECTORY_H
#define BEZIER_LEG_TRAJECTORY_H
class BezierLegTrajectory {
private:
    struct Point3D {
        float x, y, z;
    };

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
static bool generateSmoothTrajectory(
        Position currentPos, 
        Position targetPos, 
        bool isLeftLeg, 
        void (*moveCallback)(float, float, float, bool)
    ) {
      
        Point3D controlPoints[4];
        

        controlPoints[0] = {currentPos.x, currentPos.y, currentPos.z};
        
        controlPoints[1] = {
            currentPos.x + (targetPos.x - currentPos.x) * 0.3f,
            currentPos.y + (targetPos.y - currentPos.y) * 0.3f,
            currentPos.z + (targetPos.z - currentPos.z) * 0.3f
        };
        
        controlPoints[2] = {
            targetPos.x - (targetPos.x - currentPos.x) * 0.3f,
            targetPos.y - (targetPos.y - currentPos.y) * 0.3f,
            targetPos.z - (targetPos.z - currentPos.z) * 0.3f
        };
        
        controlPoints[3] = {targetPos.x, targetPos.y, targetPos.z};

        const int numInterpolationPoints = 20;

        for (int i = 0; i <= numInterpolationPoints; ++i) {
            float t = (float)i / numInterpolationPoints;
            Point3D interpolatedPoint = calculateBezierPoint(t, controlPoints);
            
            moveCallback(
                interpolatedPoint.x, 
                interpolatedPoint.y, 
                interpolatedPoint.z, 
                isLeftLeg
            );

            delay(10);
        }

        return true;
    }
};

bool moveToCoordinate(float x, float y, float z, bool isLeftLeg = true) {

    // isRelativeMove = false;
    const float baseX = 0.0f;
    const float baseY = 0.0f;
    const float baseZ = 20.0f;
    
    // targetPos.x = isRelativeMove ? currentPos.x + (baseX + x) : (baseX + x);
    // targetPos.y = isRelativeMove ? currentPos.y + (baseY + y) : (baseY + y);
    // targetPos.z = isRelativeMove ? currentPos.z + (baseZ + z) : (baseZ + z);
    
    using MoveCallback = void (*)(float, float, float, bool);
    
    static MoveCallback internalMoveCallback = [](float x, float y, float z, bool isLeftLeg) {
        IKAngles legAngles = calculateInverseKinematics(x, isLeftLeg ? y : -y, z);
        
        int32_t targetPositions[4];
        int32_t currentPositions[4];  
        
        if (!readAllActuatorPositions(currentPositions)) {
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
        dxl_wb.syncWrite(handlerIndex, targetPositions, &log);
    };

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

int32_t leftLegInitialPos[2] = {2048, 2048};  // left initial position
int32_t rightLegInitialPos[2] = {2048, 2048}; // right initial position
float initialHeight = 20.0;  // Initial Squat Height = 20

bool initializePosition() {
  /* 
    move to the middle position first
    then move to the real initial position
  */
    const char *log;
    int32_t middlePositions[4] = {2048, 2048, 2048, 2048};
    
    bool result = dxl_wb.syncWrite(handlerIndex, middlePositions, &log);
    if (result) {
        leftLegInitialPos[0] = middlePositions[0];
        leftLegInitialPos[1] = middlePositions[1];
        rightLegInitialPos[0] = middlePositions[2];
        rightLegInitialPos[1] = middlePositions[3];
        delay(1000);
        
        // move to the initial position (0,0,20)
        // IKAngles legAngles = calculateInverseKinematics(0, 0, 20.0);
        // int32_t realInitialPositions[4] = {
        //     convertRadianToPosition(-(legAngles.theta_hp+M_PI)),  // left thigh
        //     convertRadianToPosition(legAngles.theta_ap-M_PI),     // left calf
        //     convertRadianToPosition(legAngles.theta_hp+M_PI),     // right thigh
        //     convertRadianToPosition(-(legAngles.theta_ap-M_PI))   // right calf
        // };
        
    if(!moveToCoordinate(0, 0, 20.0, true) || 
        !moveToCoordinate(0, 0, 20.0, false)) {
          Serial.println("Failed to return to initial squat position!");
          return false;
      }
        // result = dxl_wb.syncWrite(handlerIndex, realInitialPositions, &log);
        // if (!result) {
        //     Serial.println("Failed to enter real initial position!");
        //     return false;
        }
    // else {
    //     Serial.println("Failed to set initial position");
    // }
    return result;
}