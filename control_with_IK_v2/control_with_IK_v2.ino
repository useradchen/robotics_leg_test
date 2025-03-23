#include <DynamixelWorkbench.h>
#include <math.h>

#if defined(__OPENCM904__)
  #define DEVICE_NAME "3"
#elif defined(__OPENCR__)
  #define DEVICE_NAME ""
#endif   

#define BAUDRATE  1000000
#define DXL_ID_1  3    // Upper 
#define DXL_ID_2  2    // Lower 
#define DXL_ID_3  7    // Upper 
#define DXL_ID_4  6    // Lower 

// Define speed parameters
#define PROFILE_VELOCITY 50    // Lower value means faster movement for MX-106 (range: 0-1023)
                              // 0 means maximum speed, 1-1023 sets the speed

DynamixelWorkbench dxl_wb;

uint8_t dxl_id[4] = {DXL_ID_1, DXL_ID_2, DXL_ID_3, DXL_ID_4};
const uint8_t handler_index = 0;
const float LENGTH = 210.0; // leg length

// Coordinates(x,y,z)
struct Coordinates {
    float x;
    float y;
    float z;
};

// Calculate angles using inverse kinematics
//calculateInverseKinematics
double calculateInverseKinematics(float x, float y, float z) {
    float l = LENGTH;
    float theta_hp , theta_ap , theta_ar;
    // Calculate θhp1
    theta_hp = atan2(x, (2*l - z)) + acos(sqrt(pow((2*l - z), 2) + pow(x, 2)) / (2*l));
    
    // Calculate θap
    theta_ap = acos(abs(x) / sqrt(pow((2*l - z), 2) + pow(x, 2))) + 
               acos(sqrt(pow((2*l - z), 2) + pow(x, 2)) / (2*l)) - M_PI/2;
    
    // Calculate θar
    theta_ar = atan2(y, (2*l - z));

    return theta_hp,theta_ap,theta_ar;
}

// Convert radians to MX-106 position value
// int32_t convertRadianToPosition(float degree) {
//     // MX-106 has 4096 positions per 360 degrees
//     return (int32_t)((4096.0/360.0) * degree);
// }
int32_t convertRadianToPosition(float radian) {
    // 將弧度轉換為角度
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

        // Set velocity for smoother and slower movement
        result = dxl_wb.writeRegister(dxl_id[i], "Moving_Speed", PROFILE_VELOCITY, &log);
        if (result == false) {
            Serial.println(log);
            Serial.println("Failed to set speed - continuing anyway");
            // Continue anyway - this isn't a critical failure
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

// Set velocity for all motors
bool setMotorVelocity(uint32_t velocity) {
    const char *log;
    bool result = true;
    
    for (int i = 0; i < 4; i++) {
        result = dxl_wb.writeRegister(dxl_id[i], "Moving_Speed", velocity, &log);
        if (result == false) {
            Serial.println(log);
            return false;
        }
    }
    
    return result;
}

// Move all motors to initial position
bool moveToInitialPosition() {
    const char *log;
    
    // Set all motors to the same initial position (center)
    int32_t initialPositions[4] = {
        2048,  // Motor 1 (DXL_ID_1 = 3)
        2048,  // Motor 2 (DXL_ID_2 = 2)
        2048,  // Motor 3 (DXL_ID_3 = 7)
        2048   // Motor 4 (DXL_ID_4 = 6)
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

// Move to specified coordinates
bool moveToCoordinate(float x, float y, float z) {
    const char *log;
    float theta_hp, theta_ap, theta_ar;
    
    // Calculate angles using inverse kinematics
    calculateInverseKinematics(x, y, z);
    // calculateInverseKinematics(x, y, z, theta_hp, theta_ap, theta_ar);

    
    // Convert to motor position values
    int32_t positions[4] = {
        convertRadianToPosition(-(theta_hp+M_PI)),  // Motor 1 (start from 180度)
        convertRadianToPosition(theta_ap-M_PI),  // Motor 2
        convertRadianToPosition(-(theta_hp+M_PI)),  // Motor 3
        convertRadianToPosition(theta_ap-M_PI)   // Motor 4
    };
    
    // Display calculation results
    Serial.println("Calculated angles (radians):");
    Serial.print("theta_hp: "); Serial.println(theta_hp);
    Serial.print("theta_ap: "); Serial.println(theta_ap);
    Serial.print("theta_ar: "); Serial.println(theta_ar);
    
    Serial.println("Converted motor position values:");
    for(int i = 0; i < 4; i++) {
        Serial.print("Motor "); Serial.print(i+1); 
        Serial.print(": "); Serial.println(positions[i]);
    }
    
    // Execute synchronized movement
    bool result = dxl_wb.syncWrite(handler_index, positions, &log);
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
    Serial.println("You can also set motor speed by typing 'speed:value' (range: 1-1023, lower is faster)");
}

void loop() {
    if (Serial.available() > 0) {
        String input = Serial.readStringUntil('\n');
        input.trim();
        
        // Check if it's a speed command
        if (input.startsWith("speed:")) {
            String speedValue = input.substring(6);
            uint32_t velocity = speedValue.toInt();
            
            // Ensure velocity is within valid range
            if (velocity > 1023) velocity = 1023;
            if (velocity < 1) velocity = 1;  // 0 is maximum speed, 1 is slowest
            
            if (setMotorVelocity(velocity)) {
                Serial.print("Motor speed set to: ");
                Serial.println(velocity);
                Serial.println("(Lower values = faster movement, higher values = slower movement)");
            } else {
                Serial.println("Failed to set motor speed!");
            }
        }
        // Otherwise parse as coordinates
        else {
            // Parse input coordinates
            int firstComma = input.indexOf(',');
            int secondComma = input.indexOf(',', firstComma + 1);
            
            if (firstComma != -1 && secondComma != -1) {
                float x = input.substring(0, firstComma).toFloat();
                float y = input.substring(firstComma + 1, secondComma).toFloat();
                float z = input.substring(secondComma + 1).toFloat();
                
                Serial.println("\nReceived coordinates:");
                Serial.print("X: "); Serial.println(x);
                Serial.print("Y: "); Serial.println(y);
                Serial.print("Z: "); Serial.println(z);
                
                if (moveToCoordinate(x, y, z)) {
                    Serial.println("Movement executed successfully!");
                } else {
                    Serial.println("Movement execution failed!");
                }
            } else {
                Serial.println("Invalid format! Please use the correct format: x,y,z");
                Serial.println("Example: 100,50,150");
                Serial.println("Or to change speed: speed:50");
            }
        }
        
        Serial.println("\nEnter new coordinates (format: x,y,z) or 'speed:value':");
    }
    delay(10);
}