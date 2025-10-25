#include <Arduino.h>
#include <Robot.h>
int vx=0;
float ball[16]={
22.5,45,67.5,90,90,112.5,135,157.5,202.5,225,247.5,270,292.5,315,337.5};
float ball_degree = ball[ballData.dir];

void setup(){
Robot_Init();
}
void loop(){
Serial.println(ballData.dis);
Serial.println(ballData.dir);
readBNO085Yaw();
ballsensor();
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

Vector_Motion(vx,0);
}