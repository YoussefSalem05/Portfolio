#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>

// ========================================================
// HARDWARE PARAMETERS 
// ========================================================
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#define SERVO_MIN_US 500  
#define SERVO_MAX_US 2500 
#define NUM_SERVOS 6 // 0 to 4 for arm, 5 for gripper

// --- ADDED CHANNEL MAP ---
// Maps logic (0-5) to your physical pins: 0, 3, 4, 7, 8, 15
const int channelMap[6] = {0, 3, 4, 7, 8, 15};

const float L1 = 13;      
const float L2 = 9.5;      
const float L3 = 2.8;       
const float Lg = 9.0;       
const float Zoffset = 8.5;  
const float L_eff = L2 + L3 + Lg; 

float servoOffsets[6]     = {90.0, 35.0, 10.0, 70.0, 180.0, 0.0}; 
float HomeServoOffsets[6] = {90.0, 130.0, 140.0, 70.0, 60.0, 100.0};
float servoDirections[6]  = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};

// State tracker for the Gripper toggle
bool isGripperClosed = false; 

// Function prototypes
bool inverseKinematics(float X, float Y, float Z, float Roll_deg, float mathAngles[5]);
void moveServo(int servoNum, float angle);

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n=============================================");
  Serial.println(" ESP32 IK CONTROLLER: CLEARANCE LIFT ACTIVE");
  Serial.println("=============================================");
  Serial.println("-> AUTO MODE   (IK):  A, X, Y, Z, Roll");
  Serial.println("-> MANUAL MODE (Arm): M, ServoNum, Angle");
  Serial.println("-> GRIPPER          : G (Toggle) or G, Angle");
  Serial.println("-> HOME MODE   (All): H");
  Serial.println("-> PICK & PLACE     : P, X, Y, Z, Roll");
  Serial.println("=============================================\n");

  Wire.begin(21, 22);
  
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(50);  
  delay(10);
  
  // Move to Home Position on Boot
  Serial.println("Moving to Home Position...");
  for(int i = 0; i < NUM_SERVOS; i++) {
    moveServo(i, HomeServoOffsets[i]);
  }
  // Ensure tracker matches Home array (Home sets it to 180, which is OPEN)
  isGripperClosed = false; 
}

void loop() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    input.toUpperCase(); 

    if (input.length() > 0) {
      char mode = input.charAt(0);

      // --- AUTO MODE (Inverse Kinematics) ---
      if (mode == 'A') {
        float X, Y, Z, Roll;
        int parsed = sscanf(input.c_str(), "A,%f,%f,%f,%f", &Y, &X, &Z, &Roll);
        
        if (parsed == 4) {
          // Z-SQUASH SAFETY FUNCTION
          float r = sqrt(X*X + Y*Y);
          if (Z < 12.0) {
            Serial.printf("\n[SAFETY] Z input %.1f squashed to prevent floor collision.\n", Z);
            Z = 0.1*r + 4;
          }

          Serial.printf("\n[IK TARGET] X: %.1f, Y: %.1f, Z: %.1f, Roll: %.1f\n", Y, X, Z, Roll);
          
          float mathAngles[5];
          if (inverseKinematics(X, Y, Z, Roll, mathAngles)) {
            bool outOfBounds = false;
            float targetPhysical[5];

            // PRE-CHECK PASS: Calculate all physical angles for the 5 arm joints
            for (int i = 0; i < 5; i++) {
              targetPhysical[i] = servoOffsets[i] + (mathAngles[i] * servoDirections[i]);
              
              if (targetPhysical[i] < 0.0 || targetPhysical[i] > 180.0) {
                outOfBounds = true;
                Serial.printf("  [LIMIT ERROR] Servo %d requires %.1f deg (Out of 0-180 range!)\n", i, targetPhysical[i]);
              }
            }

            // DECISION: Move or Abort
            if (outOfBounds) {
              Serial.println("  [ABORT] Movement refused. Target requires angles outside physical limits.");
            } else {
              Serial.println("  [SUCCESS] All angles valid. Executing Safe Sweep:");
              
         

              Serial.printf("      Servo 2 (Elbow)   : %.1f deg\n", targetPhysical[2]);
              moveServo(2, targetPhysical[2]);
              delay(1500);
              
              Serial.printf("      Servo 0 (Base)    : %.1f deg\n", targetPhysical[0]);
              moveServo(0, targetPhysical[0]);
              delay(500);

              Serial.println("    2. Sweeping arm to target X, Y, Z...");
              Serial.printf("      Servo 1 (Shoulder): %.1f deg\n", targetPhysical[1]);
              moveServo(1, targetPhysical[1]);
              delay(300); 
              
              Serial.printf("      Servo 3 (Roll)    : %.1f deg\n", targetPhysical[3]);
              moveServo(3, targetPhysical[3]);
              

            }
          } else {
            Serial.println("  [IK ERROR] Coordinates mathematically out of physical reach!");
          }
        } else {
          Serial.println("  [FORMAT ERROR] Use: A, X, Y, Z, Roll");
        }
      } 
      
      // ========================================================
      // --- PICK AND PLACE MODE ---
      // ========================================================
      else if (mode == 'P') {
        float X, Y, Z, Roll;
        // Parsed identically to 'A' mode
        int parsed = sscanf(input.c_str(), "P,%f,%f,%f,%f", &Y, &X, &Z, &Roll);
        
        if (parsed == 4) {
          Serial.printf("\n[PICK & PLACE] Starting sequence for X:%.1f, Y:%.1f, Z:%.1f, Roll:%.1f\n", Y, X, Z, Roll);

          Serial.println("  [STEP 5] Opening gripper...");
          moveServo(5, 100.0);
          isGripperClosed = false;

          // -----------------------------------------------------------------
          // STEP 1: Robot goes to the point normally (Exactly like 'A' command)
          // -----------------------------------------------------------------
          float mathAngles[5];
          float targetPhysical[5];
          bool outOfBounds = false;
          
          float r = sqrt(X*X + Y*Y);
          float safeZ = Z;
          if (safeZ < 10.0) safeZ = 0.12*r + 5.4;

          if (inverseKinematics(X, Y, safeZ, Roll, mathAngles)) {
            for (int i = 0; i < 5; i++) {
              targetPhysical[i] = servoOffsets[i] + (mathAngles[i] * servoDirections[i]);
              if (targetPhysical[i] < 0.0 || targetPhysical[i] > 180.0) outOfBounds = true;
            }
            
            if (!outOfBounds) {
              Serial.println("  [STEP 1] Moving to pick point normally...");
              Serial.println("  [SUCCESS] All angles valid. Executing Safe Sweep:");
               

              moveServo(2, targetPhysical[2]);
              delay(1000);    //elbow delay
 

              moveServo(0, targetPhysical[0]);
              delay(500);
              
              moveServo(1, targetPhysical[1]);
              delay(300);
              
              
              moveServo(3, targetPhysical[3]);
              

              
              delay(1000); // 500ms delay between steps

              // -----------------------------------------------------------------
              // STEP 2: Gripper closes
              // -----------------------------------------------------------------
              Serial.println("  [STEP 2] Closing gripper...");
              moveServo(5, 30.0);
              isGripperClosed = true;
              
              delay(500); // 500ms delay between steps

              // -----------------------------------------------------------------
              // STEP 3: Shoulder (motor 1) moves to 117 degrees
              // -----------------------------------------------------------------
              Serial.println("  [STEP 3] Moving shoulder (Motor 1) to 117 degrees...");
              moveServo(1, 128.4);
              
              delay(500); // 500ms delay between steps

              // -----------------------------------------------------------------
              // -----------------------------------------------------------------
              // STEP 4: Use Inverse Kinematics to move to drop point
              // -----------------------------------------------------------------
              Serial.println("  [STEP 4] Moving to drop point...");
              
              // FLIPPED X AND Y HERE to match how your 'P'/'A' commands parse from Serial
              float dropX = 7.0, dropY = 19.0, dropZ = 18.0, dropRoll = 0.0;
              float mathAnglesDrop[5];
              float targetPhysicalDrop[5];
              bool outOfBoundsDrop = false;
              
              if (inverseKinematics(dropX, dropY, dropZ, dropRoll, mathAnglesDrop)) {
                for (int i = 0; i < 5; i++) {
                  targetPhysicalDrop[i] = servoOffsets[i] + (mathAnglesDrop[i] * servoDirections[i]);
                  if (targetPhysicalDrop[i] < 0.0 || targetPhysicalDrop[i] > 180.0) outOfBoundsDrop = true;
                }
                
                if (!outOfBoundsDrop) {
                  // Execute safe sweep to drop coordinates
                  moveServo(2, targetPhysicalDrop[2]);
                  delay(1000);
                  
                  moveServo(0, targetPhysicalDrop[0]);
                  delay(500);
    
                  moveServo(1, targetPhysicalDrop[1]);
                  delay(500);
                  
                  // Moved Motor 3 safely inside the bounds check
                  moveServo(3, targetPhysicalDrop[3]);
                  
                } else {
                  Serial.println("    [ERROR] Drop coordinates out of physical bounds. Aborting remainder.");
                }
              } else {
                Serial.println("    [ERROR] Drop coordinates mathematically unreachable. Aborting remainder.");
              }

              delay(500); // 500ms delay between steps

              // -----------------------------------------------------------------
              // STEP 5: Open Gripper
              // -----------------------------------------------------------------
              Serial.println("  [STEP 5] Opening gripper...");
              moveServo(5, 100.0);
              isGripperClosed = false;
              
              delay(1500); // 500ms delay between steps

              // -----------------------------------------------------------------
              // STEP 6: Move to Home
              // -----------------------------------------------------------------
              Serial.println("  [STEP 6] Returning to Home position...");
              for (int i = 0; i < NUM_SERVOS; i++) {
                moveServo(i, HomeServoOffsets[i]);
              }
              isGripperClosed = false;
              
              delay(500); // Final 500ms delay
              Serial.println("[PICK & PLACE] Sequence complete.");

            } else {
              Serial.println("  [ABORT] Pick coordinates result in angles outside physical limits.");
            }
          } else {
            Serial.println("  [ABORT] Pick coordinates mathematically unreachable.");
          }
        } else {
          Serial.println("  [FORMAT ERROR] Use: P, X, Y, Z, Roll");
        }
      }

      // --- MANUAL MODE (Direct Joint Control) ---
      else if (mode == 'M') {
        int servoNum;
        float angle;
        int parsed = sscanf(input.c_str(), "M,%d,%f", &servoNum, &angle);
        
        if (parsed == 2 && servoNum >= 0 && servoNum < NUM_SERVOS) {
          if (angle < 0.0 || angle > 180.0) {
            Serial.printf("\n[LIMIT ERROR] Servo %d cannot move to %.1f deg (Must be 0-180). Movement refused.\n", servoNum, angle);
          } else {
            Serial.printf("\n[MANUAL] Moving Servo %d to %.1f degrees\n", servoNum, angle);
            moveServo(servoNum, angle);
          }
        } else {
          Serial.println("  [FORMAT ERROR] Use: M, ServoNum, Angle");
        }
      } 

      // --- GRIPPER MODE (Toggle On/Off OR specific Angle) ---
      else if (mode == 'G') {
        float angle;
        int parsed = sscanf(input.c_str(), "G,%f", &angle);
        
        // If the user provided an angle (e.g., "G, 120")
        if (parsed == 1) {
          if (angle < 0.0 || angle > 180.0) {
            Serial.printf("\n[LIMIT ERROR] Gripper cannot move to %.1f deg (Must be 0-180).\n", angle);
          } else {
            Serial.printf("\n[GRIPPER] Moving manually to %.1f degrees\n", angle);
            moveServo(5, angle);
            
            // Sync the toggle state based on an arbitrary midpoint so toggle still works normally later
            isGripperClosed = (angle <= 160.0); 
          }
        } 
        // If the user just sent "G"
        else if (input.length() == 1) {
          // Toggle the boolean state
          isGripperClosed = !isGripperClosed; 
          
          // 30 for Closed, 120 for Open
          float targetAngle = isGripperClosed ? 30 : 100.0; 
          
          Serial.printf("\n[GRIPPER] Toggled %s (Moving Servo 5 to %.1f deg)\n", isGripperClosed ? "CLOSED" : "OPEN", targetAngle);
          moveServo(5, targetAngle);
        } 
        else {
          Serial.println("  [FORMAT ERROR] Use: G (to toggle) OR G, Angle (to set specific angle)");
        }
      }
      
      // --- HOME MODE ---
      else if (mode == 'H') {
        Serial.println("\n[HOME] Returning to Home Coordinates");
        
        // Optional: Lift pitch before going home as well!

        for (int i = 0; i < NUM_SERVOS; i++) {
          Serial.printf("    Servo %d: %.1f deg\n", i, HomeServoOffsets[i]);
          moveServo(i, HomeServoOffsets[i]);
        }
        // Reset the gripper toggle tracker since Home opens the gripper to 180
        isGripperClosed = false; 
      }
      
      // --- UNKNOWN COMMAND ---
      else {
        Serial.println("\n[ERROR] Unknown command. Start with A, M, P, G, or H.");
      }
    }
  }
}

// ========================================================
// INVERSE KINEMATICS SOLVER 
// ========================================================
bool inverseKinematics(float X, float Y, float Z, float Roll_deg, float mathAngles[5]) {
  float q1 = atan2(Y, X);
  float R = sqrt(X*X + Y*Y);
  float Z_rel = Z - Zoffset;
  float D2 = R*R + Z_rel*Z_rel;
  float D = sqrt(D2);
  
  if (D > (L1 + L_eff) || D < abs(L1 - L_eff)) return false;
  
  float cos_q3 = (D2 - L1*L1 - L_eff*L_eff) / (2 * L1 * L_eff);
  cos_q3 = constrain(cos_q3, -1.0, 1.0);
  float q3 = acos(cos_q3); 
  
  float alpha = atan2(Z_rel, R);
  float beta = atan2(L_eff * sin(abs(q3)), L1 + L_eff * cos(abs(q3)));
  
  float q2 = (q3 > 0) ? (alpha + beta) : (alpha - beta);
  
  mathAngles[0] = q1 * (180.0 / PI);
  mathAngles[1] = q2 * (180.0 / PI);
  mathAngles[2] = q3 * (180.0 / PI);
  mathAngles[3] = Roll_deg;  

  // ==========================================
  // VERTICAL AUTO-LEVELING LOGIC
  // ==========================================
  // 1. Calculate the angle needed to point straight down
  float verticalPitch = -(mathAngles[1] + mathAngles[2]) - 90.0; 

  // 2. Apply your safety range constraint 
  mathAngles[4] = constrain(verticalPitch, -90.0, 90.0);        
  // ==========================================
  
  return true;
}

// ========================================================
// HARDWARE COMMAND WRITER 
// ========================================================
void moveServo(int servoNum, float angle) {
  // --- CHANGED LOGIC TO ROUTE THROUGH channelMap ---
  int physicalChannel = channelMap[servoNum];
  int pulseLength = map(angle, 0, 180, SERVO_MIN_US, SERVO_MAX_US);
  pwm.writeMicroseconds(physicalChannel, pulseLength);
}
