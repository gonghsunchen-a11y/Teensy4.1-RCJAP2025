#include <Arduino.h>
#include <Robot.h>
int vx=0;
int vy=0;
float ball[16]={
22.5,45,67.5,90,90,112.5,135,157.5,202.5,225,247.5,270,292.5,315,337.5};
float ball_degree = ball[ballData.dir];
const int Pin = 41;     
const int Pin1 = 40;    
const int Pin2 = 39; 
void stop();
void stop1();
void stop2();
volatile bool centerTouch = false;
volatile bool leftUnTouch   = false;
volatile bool rightUnTouch  = false;

int front = A13;
int back  = A8;
int left  = A12;
int right = A14;
#define MAX_RANG 520.0       
#define ADC_SOLUTION 1023.0 
#define N 5
float front_buf[N]={0}, back_buf[N]={0}, left_buf[N]={0}, right_buf[N]={0};
int ultrasonic_count=0;
float dist_f=0, dist_b=0, dist_l=0, dist_r=0;

float back_safe = 30;   
float side_safe = 45;  

void setup(){
Robot_Init();
    attachInterrupt(digitalPinToInterrupt(Pin), stop, RISING);
    attachInterrupt(digitalPinToInterrupt(Pin1), stop1, FALLING);
    attachInterrupt(digitalPinToInterrupt(Pin2), stop2, FALLING);
    pinMode(front, INPUT);
    pinMode(back, INPUT);
    pinMode(left, INPUT);
    pinMode(right, INPUT);
}
void loop(){
readBNO085Yaw();
ballsensor();
linesensor();
float f = analogRead(front)*MAX_RANG/ADC_SOLUTION;
float b = analogRead(back) *MAX_RANG/ADC_SOLUTION;
float l = analogRead(left) *MAX_RANG/ADC_SOLUTION;
float r = analogRead(right)*MAX_RANG/ADC_SOLUTION;
front_buf[ultrasonic_count] = f;
back_buf[ultrasonic_count]  = b;
left_buf[ultrasonic_count]  = l;
right_buf[ultrasonic_count] = r;
ultrasonic_count++;

if(ultrasonic_count >= N){
    dist_f = dist_b = dist_l = dist_r = 0;
    for(int i=0;i<N;i++){
        dist_f += front_buf[i];
        dist_b += back_buf[i];
        dist_l += left_buf[i];
        dist_r += right_buf[i];
    }
    dist_f /= N;
    dist_b /= N;
    dist_l /= N;
    dist_r /= N;
    ultrasonic_count=0;
}


if(ballData.dis!=255){
int speed=0;
speed=map(ballData.dis,0,12,20,MAX_VX);
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
/*if(analogRead(Pin)>500){
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
}*/
//if(analogRead(Pin1)<650&&analogRead(Pin2)<380)
float sumX = 0, sumY = 0;
uint8_t count = 0;
for (int i = 0; i < 18; i++) {
    bool detected = ((lineData.state & (1UL << i)) == 0); // 0 表有線
    //Serial.print("Sensor "); Serial.print(i);
    //Serial.print(": "); Serial.println(detected ? "ON line" : "OFF line");
    if (detected) {
        float deg = linesensorDegreelist[i];
        if (deg >= 360) deg -= 360;

        //Serial.print("  deg +180 = "); Serial.println(deg);

        sumX += cos(deg * DtoR_const);
        sumY += sin(deg * DtoR_const);
        count++;
    }
}
float lineDegree = atan2(sumY, sumX) * RtoD_const;
if (lineDegree < 0) {
    lineDegree += 360; 
}
if(lineDegree<45||lineDegree>315){
    vx = vx * 0.5;
}
if(lineDegree<135&&lineDegree>225){
    vx = vx * 0.5;
}

bool lefttouch= false;
bool righttouch= false;
bool backtouch= false;

if(analogRead(Pin)>800){//back
    backtouch = true;
}
else{
    backtouch= false;
}
if(analogRead(Pin1)>700){//left
    lefttouch = true;
}
else{
    lefttouch = false;
}
if(analogRead(Pin2)>740){//right
    righttouch = true;
}
else{
    righttouch = false;
}


/*
Serial.println(vx);
Serial.println(vy);
Serial.println(analogRead(Pin));
Serial.println(analogRead(Pin1));
Serial.println(analogRead(Pin2));
*/

if(!((lineData.state >> 4) & 1)){
    vy = MAX_VY;
}
else if(!((lineData.state >> 3) & 1) ||!((lineData.state >> 5) & 1)){
    vy = MAX_VY*0.8;
}
else if(!((lineData.state >> 2) & 1 )|| !((lineData.state >> 6) & 1)){
    vy = MAX_VY*0.7;
}
else if(!((lineData.state >> 1) & 1 )|| !((lineData.state >> 7) & 1)){
    vy = MAX_VY*0.55;
}
else if(!((lineData.state >> 0) & 1 )|| !((lineData.state >> 8) & 1)){
    vy = MAX_VY*0.25;
}

if(!((lineData.state >> 13) & 1)){
    vy = -MAX_VY;
}
else if(!(lineData.state >> 12) & 1 || !(lineData.state >> 14) & 1){
    vy = -MAX_VY*0.8;
}
else if(!((lineData.state >> 11) & 1) || !((lineData.state >> 15) & 1)){
    vy = -MAX_VY*0.7;
}
else if(!((lineData.state >> 10) & 1 )|| !((lineData.state >> 16) & 1)){
    vy = -MAX_VY*0.55;
}
else if(!((lineData.state >> 9) & 1 )|| !((lineData.state >> 17) & 1)){
    vy = -MAX_VY*0.25;
}

if(!backtouch && lefttouch && righttouch){
    Serial.println("a1");
}
else if(!backtouch && !lefttouch && righttouch){
    if(vx<0){
        vx=0;
    }
    if(vy<0){
        if(fabs(lineDegree-270) >= 10){
           Serial.println("lock");
            vy=0;
        }
    }
    Serial.println("b1");
}
else if(backtouch && !lefttouch && righttouch){
    vx=50;
    if(vy<0){
        //if(fabs(lineDegree-270) >= 20){
            //Serial.println("lock");
            vy=0;
        //}
    }
    Serial.println("c1");
}

if(!backtouch && lefttouch && righttouch){
    Serial.println("a2");
}
else if(!backtouch && lefttouch && !righttouch){
    if(vx>0){
        vx=0;
    }
    if(vy<0){
        if(fabs(lineDegree-270) >= 10){
            Serial.println("lock");
            vy=0;
        }
    }
    Serial.println("b2");
}
else if(backtouch && lefttouch && !righttouch){
    vx=-50;
    if(vy<0){
        //if(fabs(lineDegree-270) >= 20){
            //Serial.println("lock");
            vy=0;
        //}
    }
    Serial.println("c2");
}

if(dist_b < back_safe && vy < 0){
    vy = 0;
}


if(dist_l < side_safe && vx < 0){
    vx = 0;
}


if(dist_r < side_safe && vx > 0){
    vx = 0;
}

// 左右超出保護
if(dist_l < side_safe){       // 左側太靠近邊界
    vx = fabs(vx);             // 向右回到安全範圍
}

if(dist_r < side_safe){       // 右側太靠近邊界
    vx = -fabs(vx);            // 向左回到安全範圍
}


Vector_Motion(vx,vy);
Serial.println(lineDegree);
Serial.printf("y=%d\n",vy);
Serial.println(vy);
Serial.println(lineData.state,BIN);
} 
void stop(){
    centerTouch = true;
}
void stop1(){
    leftUnTouch = true;
}
void stop2(){
    rightUnTouch = true;
}