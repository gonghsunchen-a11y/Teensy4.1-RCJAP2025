#include <Arduino.h>
#include <Robot.h>
int vx=0;
int vy=0;
float ball[16]={
22.5,45,67.5,90,90,112.5,135,157.5,202.5,225,247.5,270,292.5,315,337.5};
float ball_degree = ball[ballData.dir];

void setup(){
Robot_Init();
}
void loop(){
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


if(!lineData.state&(0b01<<3) || !lineData.state&(0b01<<5)){
    vy = MAX_V*0.8;
}
else if(!lineData.state&(0b01<<2) || !lineData.state&(0b01<<6)){
    vy = MAX_V*0.6;
}
else if(!lineData.state&(0b01<<1) || !lineData.state&(0b01<<7)){
    vy = MAX_V*0.4;
}
else if(!lineData.state&(0b01<<0) || !lineData.state&(0b01<<8)){
    vy = MAX_V*0.2;
}
else if(!lineData.state&(0b01<<4)){
    vy = MAX_V;
}
else if(!lineData.state&(0b01<<9) || !lineData.state&(0b01<<17)){
    vy = -MAX_V*0.2;
}
else if(!lineData.state&(0b01<<10) || !lineData.state&(0b01<<16)){
    vy = -MAX_V*0.4;
}
else if(!lineData.state&(0b01<<11) || !lineData.state&(0b01<<15)){
    vy = -MAX_V*0.6;
}
else if(!lineData.state&(0b01<<12) || !lineData.state&(0b01<<14)){
    vy = -MAX_V*0.8;
}
else if(!lineData.state&(0b01<<13)){
    vy = -MAX_V;
}


Vector_Motion(vx,vy);
Serial.println(vy);
Serial.println(lineData.state,BIN);
} 
