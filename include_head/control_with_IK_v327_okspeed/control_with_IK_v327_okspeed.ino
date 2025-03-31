#include <DynamixelWorkbench.h>
#include <math.h>
#include "function.h"

#if defined(__OPENCM904__)
  #define DEVICE_NAME "3"
#elif defined(__OPENCR__)
  #define DEVICE_NAME ""
#endif   
/********************************************************************
  BAUDRATE  1000000
  DXL_ID_3  3 Upper  
  DXL_ID_2  2 Lower 
  DXL_ID_6  6 Upper
  DXL_ID_7  7 Lower
  uint8_t dxl_id[4] = {DXL_ID_3, DXL_ID_2, DXL_ID_6 , DXL_ID_7}
  handlerIndex = 0
  leg LENGTH = 210.0
  initialHeight = 20.0 (float)
*******************************************************************/

float forward(float stepLength) {

    stepLength = stepLength;
    float requiredSquatHeight = 0.0;
    float forward = stepLength/2;
    float back = -(stepLength/2);
  
    if (stepLength >= initialHeight){
      requiredSquatHeight = stepLength - initialHeight;
    }
    else{
      requiredSquatHeight = stepLength;
    }

    // left (0,0,z)
    if(!moveToCoordinate(0, 0, requiredSquatHeight, true)) {
        Serial.println("Left leg !!!z ERROR!!!");
        return 0.0; // false
    }
    delay(100);
    
    // left (x,0,0) forward
    if(!moveToCoordinate(forward, 0, 0, true)) {
        Serial.println("Left leg !!!x ERROR!!!");
        return 0.0; // false
    }
    // delay(100);
    
    // right(-x,0,0) back
    if(!moveToCoordinate(back, 0, 0, false)) {
          Serial.println("Right leg !!!-x ERROR!!!");
          return 0.0; // false
    }
    delay(100);

    // if(!moveToCoordinate(0, 0, 0, true)) {
    //     Serial.println("Failed to lift left leg!");
    //     return 0.0; // false
    // }

    // right (0,0,z)
    if(!moveToCoordinate(0, 0, requiredSquatHeight, false)) {
        Serial.println("Right leg !!!z ERROR!!!");
        return 0.0; // false
    }
    delay(100);
    
    // right (x,0,0) forward
    if(!moveToCoordinate(forward, 0, 0, false)) {
        Serial.println("Failed to lift left leg!");
        return 0.0; // false
    }
    // delay(100);

    // left(-x,0,0) back
    if(!moveToCoordinate(back, 0, 0, true)) {
        Serial.println("Failed to lift left leg!");
        return 0.0; // false
    }
    // delay(100);

    if(Serial.available() > 0) {
        char cmd = Serial.read();
        if(cmd == 'q' || cmd == 'Q') 
        return false;
    }
    
    return 1.0; //true
}

bool startWalking(float stepLength) {
    stepLength = stepLength;
    // if(!moveToCoordinate(0, 0, 20, true) || 
    //     !moveToCoordinate(0, 0, 20, false)) {
    //     Serial.println("Failed to adjust squat height!");
    //     return false;
    // }
    delay(500);

    Serial.println("Starting to walk...");
    Serial.println("Press 'q' to stop");
    
    while(true) {
        if(!forward(stepLength)) {
            break;
        }
    }
    // if(!moveToCoordinate(0, 0, 20.0, true) || 
    //    !moveToCoordinate(0, 0, 20.0, false)) {
    //     Serial.println("Failed to return to initial squat position!");
    //     return false;
    // }
    return true;
}

void setup() {
    Serial.begin(57600);
    delay(2000);

    Serial.println("=== Program Start ===");
    // if (!initializeDynamixels()) {
    //     Serial.println("Motor initialization failed!");
    //     while(1);
    // }
    if (!initializePosition()) {
        Serial.println("Initial position setting failed!");
        while(1);
    }
    
    Serial.println("Ready for commands:");
}

void loop() {
  if (Serial.available() > 0) {
    
    float stepLength = 0.0f;
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input.startsWith("run,")) {
      float stepLength = input.substring(4).toFloat();

      Serial.print("stepLength = ");
      Serial.println(stepLength);
      
      if (stepLength > 0) {
          if (startWalking(stepLength)) {
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