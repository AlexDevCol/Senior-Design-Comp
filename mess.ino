/*
 * CQI 2026 Robot Software - Line Following with RemoteXY Control
 * ESP8266 NodeMCU with RemoteXY and line following capabilities
 */

// RemoteXY include library
#define REMOTEXY_MODE__WIFI_POINT

// you can enable debug logging to Serial at 115200
//#define REMOTEXY__DEBUGLOG    

#include <ESP8266WiFi.h>
#include <RemoteXY.h>
#include <SPI.h>
#include <MFRC522.h>
#include <Servo.h>

// RemoteXY connection settings
#define REMOTEXY_WIFI_SSID "NEM_CONNECT"
#define REMOTEXY_WIFI_PASSWORD "1234"
#define REMOTEXY_SERVER_PORT 6377

// RemoteXY GUI configuration  
#pragma pack(push, 1)  
uint8_t RemoteXY_CONF[] =   // 153 bytes
  { 255,4,0,1,0,146,0,19,0,0,0,0,31,2,106,200,200,84,1,1,
  5,0,4,8,62,7,86,26,23,10,46,48,6,26,10,9,48,57,57,51,
  5,18,18,48,4,26,31,65,117,116,111,0,31,77,97,110,117,97,108,0,
  4,15,59,5,110,171,24,10,46,48,6,26,4,34,33,7,86,76,32,59,
  11,128,2,26,74,67,17,21,24,130,8,40,10,92,2,30,37,64,87,111,
  111,100,0,37,64,77,101,116,97,108,0,37,64,87,97,115,116,101,0,37,
  64,72,32,87,97,115,116,101,0,37,64,72,32,67,111,110,116,97,105,110,
  101,114,115,0,37,64,69,108,101,99,116,114,0 };
 
// this structure defines all the variables and events of your control interface
struct {
    // input variables
  int8_t slider_01; // from -100 to 100
  uint8_t pushSwitch_01; // =1 if state is ON, else =0, from 0 to 1
  int8_t slider_02; // from -100 to 100
  int8_t slider_03; // from 0 to 100

    // output variables
  uint8_t strings_01; // from 0 to 6

    // other variable
  uint8_t connect_flag;  // =1 if wire connected, else =0
} RemoteXY;  
#pragma pack(pop)

// IR sensor pins
#define IR_L 13  // D7
#define IR_R 15  // D8

// NFC Reader pins
#define RST_PIN 16  // D0
#define SS_PIN 1    // D10 (TX)

// NFC Dictionary structure
struct NFCEntry {
  const char* serialNumber;
  const char* materialType;
};

// NFC Dictionary - mapping serial numbers to material types
const NFCEntry nfcDictionary[] = {
  {"04:D3:B6:C5:35:02:89", "Wood"},
  {"04:73:83:BA:35:02:89", "Wood"},
  {"04:93:CF:BF:35:02:89", "Wood"},
  {"04:A3:F3:26:34:02:89", "Wood"},
  {"04:B3:9C:81:36:02:89", "Wood"},
  {"04:23:A4:26:34:02:89", "Metal"},
  {"04:93:85:BA:35:02:89", "Metal"},
  {"04:43:78:49:36:02:89", "Metal"},
  {"04:53:C5:BC:35:02:89", "Metal"},
  {"04:E3:BC:BD:35:02:89", "Metal"},
  {"04:B3:6D:56:36:02:89", "Waste"},
  {"04:B3:0F:C2:35:02:89", "Waste"},
  {"04:33:D5:2B:34:02:89", "Waste"},
  {"04:03:7F:9F:35:02:89", "Waste"},
  {"04:A3:8D:0A:35:02:89", "Waste"},
  {"04:B3:95:C8:35:02:89", "Electronics"},
  {"04:73:32:BE:35:02:89", "Electronics"},
  {"04:F3:69:2E:34:02:89", "Electronics"},
  {"04:03:F9:26:34:02:89", "Electronics"},
  {"04:73:78:B8:35:02:89", "Electronics"},
  {"04:93:EF:C6:35:02:89", "Hazardous Waste"},
  {"04:13:F7:B4:35:02:89", "Hazardous Waste"},
  {"04:93:1F:3B:34:02:89", "Hazardous Waste"},
  {"04:53:AC:29:34:02:89", "Hazardous Waste"},
  {"04:A3:8D:29:34:02:89", "Hazardous Waste"},
  {"04:53:F2:33:34:02:89", "Hazardous Containers"},
  {"04:03:50:2F:34:02:89", "Hazardous Containers"}
};

const int nfcDictionarySize = sizeof(nfcDictionary) / sizeof(nfcDictionary[0]);

// MFRC522 instance
MFRC522 mfrc522(SS_PIN, RST_PIN);

// System states
enum RobotState {
  AUTONOMOUS,
  REMOTE_CONTROL
};

// Global variables
RobotState currentState = AUTONOMOUS;
unsigned long lastStateChange = 0;
bool manualOverride = false;

// NFC variables
unsigned long lastNFCCheck = 0;
const unsigned long NFC_CHECK_INTERVAL = 500; // Check every 500ms
String lastDetectedMaterial = "";
String lastDetectedSerial = "";

// Motor control structure
struct Motor {
  const char* name;
  uint8_t speedPin;
  uint8_t dir1Pin;
  uint8_t dir2Pin;
  int duty;
  bool dirState1;
  bool dirState2;

  // Initialize
  void begin() {
    pinMode(speedPin, OUTPUT);
    pinMode(dir1Pin, OUTPUT);
    pinMode(dir2Pin, OUTPUT);
  }

  // Update PWM and direction
  void update() {
    if (duty < 0) {
      dirState1 = LOW;
      dirState2 = !dirState1;
    }
    if (duty > 0) {
      dirState1 = HIGH;
      dirState2 = !dirState1;
    }
    if (abs(duty) < 10) {
      dirState1 = LOW;
      dirState2 = LOW;
    }

    digitalWrite(dir1Pin, dirState1);
    digitalWrite(dir2Pin, dirState2);
    analogWrite(speedPin, abs(duty));
    Serial.printf("%s Duty: %d\n", name, duty);
  }
};

// Motor objects - using your pin configuration
Motor motorL = {"L", 12, 14, 2};  // speedPin, dir1Pin, dir2Pin
Motor motorR = {"R", 5, 4, 0};   // speedPin, dir1Pin, dir2Pin

// Servo objects
Servo servo1;
Servo servo2;
int servo1Pin = 13;
int servo2Pin = 10;

void setup() {
  Serial.begin(115200);
  
  // Initialize RemoteXY
  RemoteXY_Init();
  
  // Initialize motors
  motorL.begin();
  motorR.begin();
  
  // Initialize servos
  servo1.attach(servo1Pin);
  servo2.attach(servo2Pin);
  
  // Initialize IR sensors
  pinMode(IR_L, INPUT);
  pinMode(IR_R, INPUT);
  
  // Initialize NFC reader
  SPI.begin();
  mfrc522.PCD_Init();
  
  Serial.println("CQI 2026 Robot Started");
  Serial.println("WiFi: RemoteXY, Password: 12345678");
  Serial.println("NFC Reader initialized");
  Serial.println("Servos initialized");
  
  // Test motors on startup
  Serial.println("Testing motors...");
  testMotors();
}

void loop() {
  RemoteXY_Handler();
  
  // Check for NFC tags periodically
  checkForNFCTag();
  
  // Check for autonomous/manual mode based on pushSwitch_01
  // pushSwitch_01 == 0: Manual mode, pushSwitch_01 == 1: Auto mode
  if (RemoteXY.pushSwitch_01 == 0) {
    // Push switch OFF = Manual mode
    currentState = REMOTE_CONTROL;
  } else {
    // Push switch ON = Autonomous mode
    currentState = AUTONOMOUS;
  }
  
  if (currentState == AUTONOMOUS) {
    runAutonomousMode();
  } else {
    runRemoteControlMode();
  }
  
  delay(10);
}

void runAutonomousMode() {
  uint16_t ir_l = digitalRead(IR_L);
  uint16_t ir_r = digitalRead(IR_R);
  
  // Line following logic - sensors output LOW on black, HIGH on white
  if ((ir_l == HIGH) && (ir_r == HIGH)) {
    // Both sensors on white - go forward
    motorL.duty = 150;
    motorR.duty = 150;
    Serial.println("Forward");
    
  } else if ((ir_l == LOW) && (ir_r == LOW)) {
    // Both sensors on black - stop
    motorL.duty = 0;
    motorR.duty = 0;
    Serial.println("Stop");
    
  } else if (ir_l == HIGH) {
    // Left sensor on white, right on line - turn right
    motorL.duty = 150;
    motorR.duty = -100;
    Serial.println("Turn Right");
    
  } else if (ir_r == HIGH) {
    // Right sensor on white, left on line - turn left
    motorL.duty = -100;
    motorR.duty = 150;
    Serial.println("Turn Left");
  }
  
  // Update motors
  motorL.update();
  motorR.update();
  
  // Update material display
  updateMaterialDisplay();
}

void runRemoteControlMode() {
  int slider_llimit = -100;
  int slider_ulimit = 100;
  int speedLimit = 255;
  
  // Map slider values to motor duty (matching other.ino logic)
  motorL.duty = map(RemoteXY.slider_01, slider_llimit, slider_ulimit, -speedLimit, speedLimit);
  motorR.duty = map(RemoteXY.slider_02, slider_llimit, slider_ulimit, -speedLimit, speedLimit);
  
  // Update motors
  motorL.update();
  motorR.update();
  
  // Servo control (0-100 range for servo1 only)
  int servo1Angle = map(RemoteXY.slider_03, 0, 100, 0, 180);
  servo1.write(servo1Angle);
  
  // Update material display
  updateMaterialDisplay();
}

// Function to check for NFC tags and identify materials
void checkForNFCTag() {
  // Only check every NFC_CHECK_INTERVAL milliseconds
  if (millis() - lastNFCCheck < NFC_CHECK_INTERVAL) {
    return;
  }
  lastNFCCheck = millis();
  
  // Look for new cards
  if (!mfrc522.PICC_IsNewCardPresent()) {
    return;
  }
  
  // Select one of the cards
  if (!mfrc522.PICC_ReadCardSerial()) {
    return;
  }
  
  // Convert UID to string format
  String serialNumber = "";
  for (byte i = 0; i < mfrc522.uid.size; i++) {
    if (mfrc522.uid.uidByte[i] < 0x10) {
      serialNumber += "0";
    }
    serialNumber += String(mfrc522.uid.uidByte[i], HEX);
    if (i < mfrc522.uid.size - 1) {
      serialNumber += ":";
    }
  }
  serialNumber.toUpperCase();
  
  // Check if this is a new detection
  if (serialNumber != lastDetectedSerial) {
    lastDetectedSerial = serialNumber;
    
    // Find material type in dictionary
    String materialType = findMaterialType(serialNumber);
    
    if (materialType != "") {
      lastDetectedMaterial = materialType;
      Serial.println("=== NFC TAG DETECTED ===");
      Serial.print("Serial Number: ");
      Serial.println(serialNumber);
      Serial.print("Material Type: ");
      Serial.println(materialType);
      Serial.println("========================");
    } else {
      Serial.println("=== UNKNOWN NFC TAG ===");
      Serial.print("Serial Number: ");
      Serial.println(serialNumber);
      Serial.println("Material not recognized");
      Serial.println("========================");
    }
  }
  
  // Halt PICC
  mfrc522.PICC_HaltA();
  // Stop encryption on PCD
  mfrc522.PCD_StopCrypto1();
}

// Function to find material type from serial number
String findMaterialType(String serialNumber) {
  for (int i = 0; i < nfcDictionarySize; i++) {
    if (serialNumber.equals(nfcDictionary[i].serialNumber)) {
      return String(nfcDictionary[i].materialType);
    }
  }
  return ""; // Not found
}

// Function to test motors
void testMotors() {
  Serial.println("=== MOTOR TEST ===");
  
  // Test left motor forward
  Serial.println("Testing Left Motor Forward...");
  motorL.duty = 100;
  motorL.update();
  delay(1000);
  
  // Test left motor backward
  Serial.println("Testing Left Motor Backward...");
  motorL.duty = -100;
  motorL.update();
  delay(1000);
  
  // Stop left motor
  Serial.println("Stopping Left Motor...");
  motorL.duty = 0;
  motorL.update();
  delay(500);
  
  // Test right motor forward
  Serial.println("Testing Right Motor Forward...");
  motorR.duty = 100;
  motorR.update();
  delay(1000);
  
  // Test right motor backward
  Serial.println("Testing Right Motor Backward...");
  motorR.duty = -100;
  motorR.update();
  delay(1000);
  
  // Stop both motors
  Serial.println("Stopping both motors...");
  motorL.duty = 0;
  motorR.duty = 0;
  motorL.update();
  motorR.update();
  
  Serial.println("=== MOTOR TEST COMPLETE ===");
}

// Function to update material display based on detected material
void updateMaterialDisplay() {
  // Map material types to strings_01 values (0-6)
  if (lastDetectedMaterial == "Wood") {
    RemoteXY.strings_01 = 1;
  } else if (lastDetectedMaterial == "Metal") {
    RemoteXY.strings_01 = 2;
  } else if (lastDetectedMaterial == "Waste") {
    RemoteXY.strings_01 = 3;
  } else if (lastDetectedMaterial == "Hazardous Waste") {
    RemoteXY.strings_01 = 4;
  } else if (lastDetectedMaterial == "Hazardous Containers") {
    RemoteXY.strings_01 = 5;
  } else if (lastDetectedMaterial == "Electronics") {
    RemoteXY.strings_01 = 6;
  } else {
    RemoteXY.strings_01 = 0; // No material detected
  }
}

