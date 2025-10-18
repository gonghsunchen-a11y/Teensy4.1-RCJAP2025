#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Arduino.h>
#include <math.h> // Added for sin, cos, and fabs
#include <stdbool.h> // Added for clarity

//ROBOT STATES
enum robotState {BALL_SEARCH, OFFENSE, DEFENSE, AVOID_LINE, IDLE};

//BALL SEARCHING THRESHOLD
#define BALL_Threshold 5
#define TOTAL_BALL_SENSORS 10

//ROBOT MAX SPEED
#define MAX_V 30

// --- MATH CONSTANTS & CONTROL PARAMETERS ---
#define DtoR_const (M_PI / 180.0) // Degrees to Radians conversion factor

// --- PIN DEFINITIONS ---
#define BUTTON_LEFT  31
#define BUTTON_RIGHT 30
// ------------------ OLED ------------------
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1

// Motor 1 Pins
#define pwmPin1 10    // PWM 控制腳
#define DIRA_1 11   // 方向控制腳1
#define DIRB_1 12

// Motor 2 Pins
#define pwmPin2 2    // PWM 控制腳
#define DIRA_2 3   // 方向控制腳1
#define DIRB_2 4

// Motor 3 Pins
#define pwmPin3 23    // PWM 控制腳
#define DIRA_3 36   // 方向控制腳1
#define DIRB_3 37

// Motor 4 Pins
#define pwmPin4 5   // PWM 控制腳
#define DIRA_4 6    // 方向控制腳1
#define DIRB_4 9

// --- GLOBAL OBJECTS & STRUCTS ---
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

struct GyroData {float heading = 0.0; bool valid = false;} gyroData;
struct LineData {uint32_t state = 0xFFFF; bool valid = false;} lineData;
struct BallData {uint8_t dis = 255; uint8_t dir = 255; bool valid = false;} ballData;
struct PosData {int8_t x = 0; int8_t y = 0;} position;

// --- ROBOT CONTROL STRUCT (New: For P-control state) ---
struct RobotControl {
    float robot_heading = 90.0;        // Target heading
    float P_factor = 0.7;             // Proportional gain
    float heading_threshold = 10.0;     // Deadband (degrees)
    int8_t vx = 0;
    int8_t vy = 0;
} control;


// --- FUNCTION PROTOTYPES ---
// Including prototypes for the new functions and existing ones
void Robot_Init();
void readBNO085Yaw();
void ballsensor();
void linesensor();
void positionEst();
void showStart();
void showRunScreen();
void showSensors(float gyro, int ball, int light);
void SetMotorSpeed(uint8_t port, int8_t speed);
void MotorStop();
void RobotIKControl(int8_t vx, int8_t vy, float omega);

void Vector_Motion(float Vx, float Vy);
void Degree_Motion(float moving_degree, int8_t speed);


// ******************************************************
// --- FUNCTION IMPLEMENTATIONS (Existing & New) ---
// ******************************************************

void Robot_Init(){
  Serial.begin(9600);
  Serial2.begin(115200);
  Serial3.begin(115200);
  Serial4.begin(115200);
  Serial5.begin(115200);
  Serial6.begin(115200);
  Serial7.begin(115200);
  Serial8.begin(115200);
  
  pinMode(pwmPin1,OUTPUT);
  pinMode(DIRA_1,OUTPUT);
  pinMode(DIRB_1,OUTPUT);

  pinMode(pwmPin2,OUTPUT);
  pinMode(DIRA_2,OUTPUT);
  pinMode(DIRB_2,OUTPUT);

  pinMode(pwmPin3,OUTPUT);
  pinMode(DIRA_3,OUTPUT);
  pinMode(DIRB_3,OUTPUT);

  pinMode(pwmPin4,OUTPUT);
  pinMode(DIRA_4,OUTPUT);
  pinMode(DIRB_4,OUTPUT);

  Wire.begin();
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) while(1);
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  showStart();
}

void readBNO085Yaw() {
  const int PACKET_SIZE = 19;
  uint8_t buffer[PACKET_SIZE];
  gyroData.valid = false; // Reset flag before read attempt

  while (Serial2.available() >= PACKET_SIZE) {
    buffer[0] = Serial2.read();
    if (buffer[0] != 0xAA) continue;
    buffer[1] = Serial2.read();
    if (buffer[1] != 0xAA) continue;

    // Read remaining 17 bytes
    for (int i = 2; i < PACKET_SIZE; i++) {
      buffer[i] = Serial2.read();
    }

    // --- Checksum: sum of bytes [2..16], mod 256 ---
    uint8_t esti_checksum = 0;
    for (int i = 2; i <= 16; i++) {
      esti_checksum += buffer[i];
    }
    esti_checksum %= 256;

    // Compare with buffer[18]
    if (esti_checksum != buffer[18]) {
      Serial.println("Checksum error");
      continue;
    }

    // --- Extract yaw (Little Endian) ---
    int16_t yaw_raw = (int16_t)((buffer[4] << 8) | buffer[3]);

    Serial.print("yaw_raw: ");
    Serial.println(yaw_raw);

    // Convert to degrees if within range
    if (abs(yaw_raw) <= 18000) {
      gyroData.heading = yaw_raw * 0.01f;
      gyroData.valid = true;
    }

    break; // Process one packet per call
  }
}


void ballsensor(){
  uint8_t buffer_index = 0;
  uint8_t buffer[3];
  ballData.valid = false;
  while (Serial4.available()){
    uint8_t b = Serial4.read();
    if(b == 0xFF){
      ballData.valid = false;
      ballData.dir = 255;
      ballData.dis = 255;
      break;
    }
    if ((buffer_index == 0 && b != 0xAA)) continue; // wait for start
    buffer[buffer_index++] = b;
    if (buffer_index == 3) {
      buffer_index = 0;
      if (buffer[0] == 0xAA && buffer[2] == 0xEE) {
        uint8_t temp = buffer[1];
        ballData.valid = true;
        ballData.dir = (temp & 0x0F);
        ballData.dis = (temp & 0xF0) >> 4;
      }
    }
  }
}

void linesensor(){
  uint8_t buffer_index = 0;
  uint8_t buffer[7];
  lineData.valid = false;
  while (Serial5.available()) {
    uint8_t b = Serial5.read();
    if (buffer_index == 0 && b != 0xAA) return; // wait for start
    buffer[buffer_index++] = b;
    if (buffer_index == 7) {
      buffer_index = 0;
      // Corrected original logic for safety (removed redundant buffer[0] check)
      if (buffer[0] == 0xAA && buffer[6] == 0xEE) { 
        uint8_t checksum = (buffer[2] + buffer[3] + buffer[4]) & 0xFF;
        if (checksum == buffer[5]) {
            lineData.valid = true;
            lineData.state = buffer[2] | (buffer[3] << 8) | (buffer[4] << 16);
        }
      }
    }
  }
}

void positionEst(){
  uint8_t buffer_index = 0;
  uint8_t buffer[4];
  ballData.valid = false;
  while (Serial3.available()){
    uint8_t b = Serial3.read();
    if ((buffer_index == 0 && b != 0xAA)) return; // wait for start
    buffer[buffer_index++] = b;
    if (buffer_index == 4) {
      buffer_index = 0;
      if (buffer[0] == 0xAA && buffer[3] == 0xEE) {
        position.x = buffer[1] - 127;
        position.y = buffer[2] - 127;
      }
    }
  }
}

void showStart() {
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(0, 20);
  display.println("Start");
  display.display();
}

void showRunScreen() {
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(0, 20);
  display.println("Run");
  display.display();
}

void showSensors(float gyro, int ball, int light) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);  display.print("Gyro: ");  display.println(gyro);
  display.setCursor(0, 15); display.print("Ball: ");  display.println(ball);
  display.setCursor(0, 30); display.print("Light: "); display.println(light);
  display.display();
}

/*Actuators Part*/
void SetMotorSpeed(uint8_t port, int8_t speed){
  speed = constrain(speed,-1.5 * MAX_V, 1.5 * MAX_V);
  int pwmVal = abs(speed) * 255 / 100;
  switch (port){
    case 4:
      analogWrite(pwmPin1, pwmVal);
      if(speed>0){
        digitalWrite(DIRA_1,HIGH);
        digitalWrite(DIRB_1,LOW);
      } else if(speed<0){
        digitalWrite(DIRA_1,LOW);
        digitalWrite(DIRB_1,HIGH);
      } else{
        digitalWrite(DIRA_1,LOW);
        digitalWrite(DIRB_1,LOW);
      }
      break;
    case 3:
      analogWrite(pwmPin2, pwmVal);
      if(speed>0){
        digitalWrite(DIRA_2,HIGH);
        digitalWrite(DIRB_2,LOW);
      } else if(speed<0){
        digitalWrite(DIRA_2,LOW);
        digitalWrite(DIRB_2,HIGH);
      } else{
        digitalWrite(DIRA_2,LOW);
        digitalWrite(DIRB_2,LOW);
      }
      break;
    case 2:
      analogWrite(pwmPin3, pwmVal);
      if(speed>0){
        digitalWrite(DIRA_3,HIGH);
        digitalWrite(DIRB_3,LOW);
      } else if(speed<0){
        digitalWrite(DIRA_3,LOW);
        digitalWrite(DIRB_3,HIGH);
      } else{
        digitalWrite(DIRA_3,LOW);
        digitalWrite(DIRB_3,LOW);
      }
      break;
    case 1:
      analogWrite(pwmPin4, pwmVal);
      if(speed>0){
        digitalWrite(DIRA_4,HIGH);
        digitalWrite(DIRB_4,LOW);
      } else if(speed<0){
        digitalWrite(DIRA_4,LOW);
        digitalWrite(DIRB_4,HIGH);
      } else{
        digitalWrite(DIRA_4,LOW);
        digitalWrite(DIRB_4,LOW);
      }
      break;
  }
}

void MotorStop(){
  digitalWrite(DIRA_1,LOW);
  digitalWrite(DIRB_1,LOW);
  digitalWrite(DIRA_2,LOW);
  digitalWrite(DIRB_2,LOW);
  digitalWrite(DIRA_3,LOW);
  digitalWrite(DIRB_3,LOW);
  digitalWrite(DIRA_4,LOW);
  digitalWrite(DIRB_4,LOW);
  analogWrite(pwmPin1, 0);
  analogWrite(pwmPin2, 0);
  analogWrite(pwmPin3, 0);
  analogWrite(pwmPin4, 0);
}

void RobotIKControl(int8_t vx, int8_t vy, float omega){
  // Note: Cast omega to int8_t for consistent data types in the IK control matrix
  int8_t p1 = -vx + vy + (int8_t)omega;
  int8_t p2 = -vx - vy + (int8_t)omega;
  int8_t p3 = vx - vy + (int8_t)omega;
  int8_t p4 = vx + vy + (int8_t)omega;
  SetMotorSpeed(1, p1);
  SetMotorSpeed(2, p2);
  SetMotorSpeed(3, p3);
  SetMotorSpeed(4, p4);
}

// ---------------------------------------------
// --- INTEGRATED MOTION CONTROL FUNCTIONS ---
// ---------------------------------------------

/**
 * @brief Handles proportional heading control (P-control) and calculates angular velocity (omega).
 * @param Vx Desired X-axis velocity (speed)
 * @param Vy Desired Y-axis velocity (speed)
 */
void Vector_Motion(float Vx, float Vy) {
  
      float omega = 0.0;
      // 2. Current heading calculation
      float current_gyro_heading = gyroData.heading;
      float sensor_heading = 90.0 - current_gyro_heading;

      // 3. Calculate the heading error (e = Target - Current)
      float e = control.robot_heading - sensor_heading;

      // 4. Calculate the angular velocity (omega) using P-Control
      if (fabs(e) > control.heading_threshold) {
          omega = e * control.P_factor;
      }
      RobotIKControl((int8_t)Vx, (int8_t)Vy, omega);
}

/**
 * @brief Moves the robot at a specified angle and speed using P-control for heading.
 * @param moving_degree The desired direction of travel (0-360 degrees).
 * @param speed The desired linear speed magnitude (-100 to 100).
 * @return true if motion control was initiated, false if degree was out of range.
 */
void Degree_Motion(float moving_degree, int8_t speed) {
    // 1. Check for valid degree range (0 to 360)
    if (moving_degree > 360.0 || moving_degree < 0.0) {
        MotorStop();
    }

    // 2. Convert angle from degrees to radians
    float moving_degree_rad = moving_degree * DtoR_const;

    // 3. Calculate velocity components (Vx, Vy)
    float Vx = cos(moving_degree_rad) * speed;
    float Vy = sin(moving_degree_rad) * speed;

    // 4. Call the Vector_Motion function to handle control
    Vector_Motion(Vx, Vy);
}

// NOTE: To make this code run on an Arduino, you must add a setup() and loop() function
// which will call Robot_Init() and the sensor/motion functions respectively.


void update_robot_heading(){
    ;
}

void kick() {
    ;
}