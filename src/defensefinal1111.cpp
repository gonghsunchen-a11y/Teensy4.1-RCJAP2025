#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Arduino.h>
#include <Robot.h>
#include <math.h>

#define SENSE(i) (!((lineData.state >> (i)) & 1))

#define possession_Threshold 205

//SPEED
float ballVx = 0;
float ballVy = 0;
float lineVx = 0;
float lineVy = 0;

int count = 0;
float x = 2;
float init_lineDegree = -1;
float diff = 0;
bool emergency = false;
bool start = false;
bool overhalf = false;
bool first_detect = false;

int left_limit = 110;
int right_limit = 228;
int middle = (left_limit+right_limit) * 0.5;

//CAMERA
unsigned long lastCameraUpdate = 0;  // 記錄上次執行時間
const unsigned long interval = 100;  // 10Hz = 每100毫秒一次

int lastLeftState = HIGH;
int lastRightState = HIGH;
unsigned long lastPress = 0;
unsigned long lastUpdate = 0;
bool showData = false; // false = Start/Run, true = 顯示數據
bool showRun = false;
// #define DEBUG
void robot_offense();
bool Debug();
bool ball_search();
void offense();
void defense();
void line_processing();
void attack();
void white_line_processing();

void setup(){
  Robot_Init();
  showMessage("Start");
}

void loop(){
  //Debug();
  while(Debug());
  while(1){
    defense();
  }  
}

bool Debug(){
  int leftState = digitalRead(BUTTON_LEFT);
  int rightState = digitalRead(BUTTON_RIGHT);

  // ------------------ 左鍵切換 Start / Data ------------------
  if(leftState == LOW && lastLeftState == HIGH &&(millis() - lastPress) > 100){
    showData = !showData;
    showRun = false; // 切回 Data 時取消 Run
    lastPress = millis();
    if(!showData){
      showMessage("Start");
    }
  }
  lastLeftState = leftState;

  // ------------------ 右鍵在 Start 顯示 Run ------------------
  if(rightState == LOW && lastRightState == HIGH &&(millis() - lastPress) > 100){
    if(!showData){ // 只有 Start 畫面生效
      showRun = !showRun;
      lastPress = millis();
      if(showRun){
        showMessage("Run");
        return false;
      }
    }
    else{
      showMessage("Ready");
    }
  }
  lastRightState = rightState;

  // ------------------ Sensor ------------------
  readBNO085Yaw();
  ballsensor();
  //readussensor();
  if(showData &&(millis() - lastUpdate > 200)){ 
    display.clearDisplay();
    showSensors(gyroData.heading, ballData.dir, lineData.valid);
    showUS(usData.dist_l, usData.dist_r, usData.dist_b);
    if(rightState == LOW){
      showMessage("scanning...");
      Serial5.write(0xAA);
      while(digitalRead(BUTTON_LEFT));
      Serial5.write(0xEE);
    }
    // Serial.print("ball_dir="); Serial.println(ballData.dir);
    lastUpdate = millis();
  }
  Serial.println("debug");
  return true;
}


void defense(){
    readBNO085Yaw();
    readussensor();
    ballsensor();
    linesensor();
    readCameraData();
    Serial.printf( "targetData.x=%d\n", targetData.x);
    Serial.printf( "targetData.h=%d\n", targetData.h);
    bool online = lineData.state != 0x3FFFF;
    Serial.printf("online%d\n", online);
    //Serial.printf("ballData.dir%d\n", ballData.dir);
    Serial.printf("usData.dist_b=%d\n", usData.dist_b);
    Serial.printf("usData.dist_l=%d\n", usData.dist_l);
    Serial.printf("usData.dist_r=%d\n", usData.dist_r);
    //bool at_left_corner = (usData.dist_b <= Back_safe && usData.dist_b >= Back_limit) && ((usData.dist_r > Side_limit - 10) && (usData.dist_l <= Side_limit));
    //bool at_right_corner = (usData.dist_b <= Back_safe && usData.dist_b >= Back_limit) && ((usData.dist_r <= Side_limit) && (usData.dist_l > Side_limit - 10));
    //bool far_away = (usData.dist_b >= Back_safe);
    //bool vertical_out_of_bound = (usData.dist_b <= Back_limit);
    //bool back_panelty = at_left_corner | at_right_corner | far_away | vertical_out_of_bound;
    bool at_left_side = false;
    bool at_right_side = false;
    bool far_away = false;
    bool too_close = false;
    if(!targetData.valid){
        at_left_side = ((usData.dist_r > Side_limit - 10) && (usData.dist_l <= Side_limit));
        at_right_side = ((usData.dist_r <= Side_limit) && (usData.dist_l > Side_limit - 10));
    }
    else{
        at_left_side = targetData.x < 100;
        at_right_side = targetData.x > 220;
    }
    far_away = (usData.dist_b >= Back_safe) | targetData.h < 40;
    too_close = (usData.dist_b <= Back_limit) | (targetData.h > 45);
    Serial.printf("far_away%d\n",far_away);
    Serial.printf("too_close%d\n",too_close);
    bool back_panelty = at_left_side | at_right_side | far_away | too_close;
    bool unknown_ver = (far_away & too_close);
    bool unknown_hor = (at_left_side & at_right_side);

    control.vx = 0;
    control.vy = 0;
    if(back_panelty&&!online){
        if(ballData.dir == 255){//out of panelty area and no ball, need to move back
            //float move_back_degree = -1;
            if(at_left_side){//left corner
                control.vx = (MAX_VX * 0.5);
                Serial.println("left_corner");
            }
            else if(at_right_side){//right corner
                control.vx = -(MAX_VX * 0.5);
                Serial.println("right_corner");
            }
            //close to the back wall
            if(too_close){
                //control.vy = (Back_limit - usData.dist_b) * (MAX_V);
                control.vy = 25;
                control.vy = constrain(control.vy, -MAX_V, MAX_V);
                Serial.println("close to the back wall");
            }
            if(far_away){
                Serial.println("far_away");
                if(usData.dist_l - usData.dist_r > 80){//left side
                    //move_back_degree = 315;
                    control.vx = MAX_VX;
                }
                else if(usData.dist_r - usData.dist_l > 80){
                    control.vx = -MAX_VX;
                    //move_back_degree = 225;
                }
                control.vy = -30;
                Serial.printf("back_speed:%d\n", control.vy);
                control.vy = constrain(control.vy, -MAX_V, MAX_V);
            }
        }
        else{
            float back_degree = -1;
            float ballDegree = ballDegreelist[ballData.dir];
            //float ballspeed = map(ballData.dis, 0, 12, 20, MAX_V);

            //close to the back wall
            /*
            if(vertical_out_of_bound){
                back_degree = 90;
                Serial.println("close to the back wall");
            }*/
            float speed_ratio = 0.8;
            if(far_away){
                Serial.println("far_away");
                if(at_left_side){//left corner
                    back_degree = 315;
                    if(ballDegree > 270){//right side of the robot
                        back_degree = 270; 
                    }
                    Serial.println("left_corner");
                }
                else if(at_right_side){//right corner
                    back_degree = 225;
                    if(ballDegree < 270 && ballDegree >180){//left side of the robot
                        back_degree = 270;
                    }
                    Serial.println("right_corner");
                }
                else{
                    back_degree = 270;
                    if(ballDegree > 270){//right side of the robot
                        back_degree = 225; 
                    }
                    else if(ballDegree < 270 && ballDegree >180){//left side of the robot
                        back_degree = 315;
                    }
                }
            }
            else{
                if(at_left_side){//left corner
                    back_degree = 0;
                    if(ballDegree > 270 || back_degree < 90){//right side of the robot
                        speed_ratio = 0.25;
                        back_degree = 45;
                    }
                    Serial.println("left_corner");
                }
                else if(at_right_side){//right corner
                    back_degree = 180;
                    if(ballDegree > 180 && ballDegree < 270){//left side of the robot
                        speed_ratio = 0.25;
                        back_degree = 135;
                    }
                    Serial.println("right_corner");
                }
            }
            Serial.printf("back_degree1=%f\n", back_degree);
            if(back_degree != -1){
                Serial.printf("back_degree=%f\n", back_degree);
                float temp = back_degree * DtoR_const;            
                control.vx = speed_ratio * MAX_V * cos(temp);
                control.vy = speed_ratio * MAX_V * sin(temp);
            }
        }
        //white_line_processing();
    }
    else if(far_away && (at_left_side || at_right_side) && online){
        white_line_processing();
    }
    else{
        Serial.println("defense");
        float leftSumX = 0, leftSumY = 0;
        float rightSumX = 0, rightSumY = 0;
        bool leftDetected = false;
        bool rightDetected = false;
        bool vertical_line = false;
        vertical_line = (SENSE(2) | SENSE(3) | SENSE(4) | SENSE(5) | SENSE(6)) & (SENSE(11) | SENSE(12) | SENSE(13) | SENSE(14) | SENSE(15));
        for (int i = 0; i < 18; i++) {
            Serial.print((lineData.state >> i) & 1,BIN);
        }
        Serial.println("");
        for (int i = 0; i < 18; i++) {
            if (!((lineData.state >> i) & 1)) { // line detected on sensor i
                Serial.println(i);
                float deg = linesensorDegreelist[i];
                float cx = cos(deg * DtoR_const);
                float cy = sin(deg * DtoR_const);
                if (cx < 0) {
                    leftSumX += cx;
                    leftSumY += cy;
                    leftDetected = true;
                }
                if(cx > 0){
                    rightSumX += cx;
                    rightSumY += cy;
                    rightDetected = true;
                }
            }
        }
        
        float leftDeg = -1, rightDeg = -1;
        if(leftDetected){
            leftDeg = atan2(leftSumY, leftSumX) * RtoD_const;
            if (leftDeg < 0) leftDeg += 360;
        }
        if(rightDetected){
            rightDeg = atan2(rightSumY, rightSumX) * RtoD_const;
            if (rightDeg < 0) rightDeg += 360;
        }
        //calibrate
        Serial.printf("%d,%d", leftDetected, rightDetected);
        if(leftDetected | rightDetected){
            float angle = abs(leftDeg - rightDeg);
            angle = (angle > 180) ? 360 - angle : angle;
            
            float calibrate_ratio = (angle > 90) ? 0 : (1 - angle / 180.0);
            float calibrate_speed = MAX_V * calibrate_ratio;
            
            float calibrate_angle = -1;
            if(leftDetected){
                calibrate_angle = leftDeg;
            }
            else if(rightDetected){
                calibrate_angle = rightDeg;
            }
            else if(rightDetected&&leftDetected){
                calibrate_angle = (leftDeg+rightDeg) / 2;
            }

            if(rightDeg < 90 && (leftDeg > 180 && leftDeg < 270)){
                calibrate_angle = 315;
            }
            if(rightDeg > 270 && (leftDeg < 180 && leftDeg > 270)){
                calibrate_angle = 225;
            }
            float calibrate_vx = 0;
            float calibrate_vy = 0; 
            if(calibrate_angle != -1){
                float temp = 3.14*(calibrate_angle / 180.0);
                calibrate_vx = calibrate_speed * cos(temp);
                calibrate_vy = calibrate_speed * sin(temp);
            }
            
            //moving
            int move_dir = 0;
            float ballDegree = ballDegreelist[ballData.dir];
            if(ballData.dir == 255){
                move_dir = 0;
            }
            else{
                if(ballDegree > 95 && ballDegree < 270){
                    move_dir = -1;
                }
                else if (ballDegree < 85 || ballDegree > 270){
                    move_dir = 1;
                }
            }
            Serial.printf("move_dir%d\n", move_dir);
            float moving_degree = -1;
            if(move_dir == -1){
                moving_degree = leftDeg;
            }
            else if(move_dir == 1){
                moving_degree = rightDeg;
            }
            //moving_degree = (abs(moving_degree - 270)<10) ? -1 : moving_degree;
            if(calibrate_ratio > 0.65){//調大減少晃動，調少跟絲滑
                control.vx = calibrate_vx;
                control.vy = calibrate_vy;
            }
            else{
                if(moving_degree != -1){
                    float temp = moving_degree * DtoR_const;
                    control.vx = MAX_V * cos(temp);
                    control.vy = MAX_V * sin(temp);
                    Serial.printf("vx = %d, vy = %d\n", control.vx, control.vy);
                }
                else{
                    control.vx = 0;
                    control.vy = 0;
                }
            }
            if(vertical_line){//on the side line, line pattern = vertical
                Serial.println("ver");
                control.vy = (control.vy < 0) ? control.vy * 0.5: control.vy;
                if(ballDegree < 180){
                    control.vy = MAX_V; 
                }
            }
            
            if(usData.dist_b > Back_limit + 5){//close to the back wall
                Serial.println("ver1");
                //control.vx = 0;
                /*
                control.vy = ((usData.dist_b-Back_limit) / 3) * control.vy;
                if(ballDegree < 180){
                    control.vy = MAX_V; 
                }*/
            }
            if(usData.dist_b <= Back_limit){//nearly touch the back wall
                Serial.println("ver2");
                //control.vy = MAX_VY * ((Back_limit - usData.dist_b) / Back_limit);
                control.vy = 20;
            }
            //float left_slow_ratio = (usData.dist_l - Side_safe) > 0 ? (usData.dist_l - Side_safe)/Side_safe: 0.5;
            //float right_slow_ratio =(usData.dist_r - Side_safe) > 0 ? (usData.dist_r - Side_safe)/Side_safe: 0.5;
            int left_limit = 110;
            int right_limit = 228;
            int middle = (left_limit+right_limit) * 0.5;
            float left_slow_ratio = targetData.valid && targetData.x < middle ? 1 - abs(targetData.x - middle) / 160.0: 1;
            float right_slow_ratio = targetData.valid && targetData.x > middle ? 1 - abs(targetData.x - middle) / 160.0 : 1;

            if(control.vx > 0){
                control.vx *= right_slow_ratio * 0.9;
                Serial.printf("right_slow_ratio = %f\n", right_slow_ratio);
            }
            if(control.vx < 0){
                control.vx *= left_slow_ratio * 0.9;
                Serial.printf("left_slow_ratio = %f\n", left_slow_ratio);
            }
            if(usData.dist_l <= Back_limit){//nearly touch the back wall
                Serial.println("ver2");
                //control.vy = MAX_VY * ((Back_limit - usData.dist_b) / Back_limit);
                control.vy = 20;
            } 
            if(usData.dist_l <= Back_limit){//nearly touch the back wall
                Serial.println("ver2");
                //control.vy = MAX_VY * ((Back_limit - usData.dist_b) / Back_limit);
                control.vy = 20;
            }
            Serial.printf("left_degree = %f\n", leftDeg);
            Serial.printf("right_degree = %f\n", rightDeg);
            Serial.printf("angle = %f\n", angle); 
            Serial.printf("calibrate_ratio = %f\n", calibrate_ratio);
            Serial.printf("calibrate_speed = %f\n", calibrate_speed);
            Serial.printf("calibrate_angle = %f\n", calibrate_angle);
            Serial.printf("moving_degree = %f\n", moving_degree);
        }
    }    
    Serial.printf("vx = %d, vy = %d\n", control.vx, control.vy);
    control.vx = constrain(control.vx, -MAX_VX, MAX_VX);
    control.vy = constrain(control.vy, -MAX_VY, MAX_VY);
    Serial.printf("vx = %d, vy = %d\n", control.vx, control.vy);
    Vector_Motion(control.vx, control.vy);
}

void white_line_processing(){
    float sumX = 0, sumY = 0;
    for(int i = 0; i < 18; i++){
        bool detected =((lineData.state &(1UL << i)) == 0); // 0 表有線
        // Serial.print("Sensor "); Serial.print(i);
        // Serial.print(": "); Serial.println(detected ? "ON line" : "OFF line");
        if(detected){
        float deg = linesensorDegreelist[i];
        sumX += cos(deg * DtoR_const);
        sumY += sin(deg * DtoR_const);
        count++;
        }
    }
    count = 0;
    if(lineData.state == 0b111111111111111111 && !overhalf && count > 0){ // no line
        count = 0;
        lineVx = 0;
        lineVy = 0;
        first_detect = false;
        init_lineDegree = -1;
    }
    if(control.picked_up){
        count = 0;
        lineVx = 0;
        lineVy = 0;
        first_detect = !first_detect;
        init_lineDegree = -1;
        overhalf = !overhalf;
    }
    if(count > 0 || overhalf){
        float lineDegree = atan2(sumY, sumX) * RtoD_const;
        if(lineDegree < 0){
        lineDegree += 360;
        }
        if(start){
        showMessage("Line");
        start = false;
        }
        // Serial.print("sumX="); Serial.print(sumX);
        // Serial.print(", sumY="); Serial.print(sumY);
        // Serial.print(", average lineDegree="); Serial.println(lineDegree);
        if(first_detect == false){
        init_lineDegree = lineDegree;
        first_detect = true;
        }

        diff = fabs(fmod((lineDegree - init_lineDegree), 360));
        float finalDegree;
        if(diff > EMERGENCY_THRESHOLD){
        overhalf = true;
        finalDegree = lineDegree;
        }
        else{
        overhalf = false;
        finalDegree = fmod(lineDegree + 180, 360);
        // delay(1000);
        }
        /*
        Serial.print("lineDegree=");
        Serial.println(lineDegree);
        Serial.print("finalDegree=");
        Serial.println(finalDegree);
        Serial.print("first_detect");
        Serial.println(first_detect);
        */
        float speed = 50;
        lineVx = speed * cos(finalDegree * DtoR_const);
        lineVy = speed * sin(finalDegree * DtoR_const);
        Vector_Motion(int(lineVx), int(lineVy));
        //Serial.print("lineVx="); Serial.print(lineVx);
        //Serial.print("lineVy="); Serial.println(lineVy);
    }
}