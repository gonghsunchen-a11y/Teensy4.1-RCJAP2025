#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Arduino.h>
#include <Robot.h>
int vx=0;
int vy=0;
float ball[16]={
22.5,45,67.5,90,90,112.5,135,157.5,202.5,225,247.5,270,292.5,315,337.5};

bool showData = false;  // false = Start/Run, true = 顯示數據
bool showRun  = false;


// ------------------ 模擬數據 ------------------
int lightSensor = 999;
int ballSensor  = 999;
void defense();
void setup(){
    Robot_Init();
    showStart();
}

int lastLeftState  = HIGH;
int lastRightState = HIGH;
unsigned long lastPress = 0;
unsigned long lastUpdate = 0;


void loop(){
  int leftState  = digitalRead(BUTTON_LEFT);
  int rightState = digitalRead(BUTTON_RIGHT);


  // ------------------ 左鍵切換 Start / Data ------------------
  if (leftState == LOW && lastLeftState == HIGH && (millis() - lastPress) > 100) {
    showData = !showData;
    showRun = false; // 切回 Data 時取消 Run
    lastPress = millis();
    if (!showData) showStart();
  }
  lastLeftState = leftState;

  // ------------------ 右鍵在 Start 顯示 Run ------------------
  if (rightState == LOW && lastRightState == HIGH && (millis() - lastPress) > 100) {
    if (!showData) {  // 只有 Start 畫面生效
      showRun = !showRun;
      lastPress = millis();
      if (showRun){
        showRunScreen();
        while (1)
        {
          defense();
        }
        
      }
    }
  }
  lastRightState = rightState;


  // ------------------ Sensor ------------------
  readBNO085Yaw();
  ballsensor();

  // ------------------ 顯示 Data 畫面 ------------------
  if (showData && (millis() - lastUpdate > 200)) {
    showSensors(gyroData.heading, ballData.dir, ballSensor);
    if(rightState==LOW){
      Serial5.write(0XAA);
      while(digitalRead(BUTTON_LEFT));
      Serial5.write(0XEE);
    }
    lastUpdate = millis();
  }
}


void defense(){
  readBNO085Yaw();
ballsensor();
linesensor();
if(ballData.dis!=255){
int speed=0;
speed=map(ballData.dis,0,12,20,MAX_V);
float ball_degree = ball[ballData.dir];
if(ball_degree<90 || ball_degree>270){
    vx=speed;
}
if(ball_degree>90 && ball_degree<270){
    vx=-speed;
}
if(ball_degree==90){
    vx=0;
}
}
else{vx=0;
}
/*
if(analogRead(Pin)>500){
    if(vx<0){
    vy=0;
}
}
if(analogRead(Pin1)<650){
    if(vx<0){
    vx=0;
}
}
if(analogRead(Pin2)<380){
    if(vx>0){
    vx=0;
}
}
//if(analogRead(Pin1)<650&&analogRead(Pin2)<380)
Serial.println(vx);
Serial.println(vy);
Serial.println(analogRead(Pin));



if(!((lineData.state >> 3) & 1) ){
    vy = MAX_V*0.8;
}



if(!((lineData.state >> 3) & 1) ||!((lineData.state >> 5) & 1)){
    vy = MAX_V*0.8;
}

else if(!((lineData.state >> 2) & 1 )|| !((lineData.state >> 6) & 1)){
    vy = MAX_V*0.6;
}
else if(!((lineData.state >> 1) & 1 )|| !((lineData.state >> 7) & 1)){
    vy = MAX_V*0.4;
}
else if(!((lineData.state >> 0) & 1 )|| !((lineData.state >> 8) & 1)){
    vy = MAX_V*0.2;
}
else if(!((lineData.state >> 4) & 1)){
    vy = MAX_V;
}
else if(!((lineData.state >> 9) & 1 )|| !((lineData.state >> 17) & 1)){
    vy = -MAX_V*0.2;
}
else if(!((lineData.state >> 10) & 1 )|| !((lineData.state >> 16) & 1)){
    vy = -MAX_V*0.4;
}
else if(!((lineData.state >> 11) & 1) || !((lineData.state >> 15) & 1)){
    vy = -MAX_V*0.6;
}
else if(!(lineData.state >> 12) & 1 || !(lineData.state >> 14) & 1){
    vy = -MAX_V*0.8;
}
else if(!((lineData.state >> 13) & 1)){
    vy = -MAX_V;
}
Serial.printf("y=%d\n",vy);
*/
Vector_Motion(vx,vy);
Serial.println(vy);
Serial.println(lineData.state,BIN);

}
