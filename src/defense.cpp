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
readBNO085Yaw();
ballsensor();
float ball_degree = ball[ballData.dir];
if(ball_degree<90){
    vx=20;
}
if(ball_degree>90){
    vx=-20;
}
if(ball_degree==90){
    vx=0;
}
Vector_Motion(vx,0);
}