#include <Arduino.h>
#include <Robot.h>
#define  MAX_RANG      (520.0)//the max measurement value of the module is 520cm(a little bit longer than  effective max range)
#define  ADC_SOLUTION      (1023.0)//ADC accuracy of Arduino UNO is 10bit
int front= A13;
int left= A12;
int right= A14;
int back= A8;
float dist_f;
float dist_b;   
float dist_l;
float dist_r;
float vx=0;
float vy=0;
/*float dist_f;
float dist_b;
float dist_l;
float dist_r;
float vx=0;
float vy=0;
int front= A13;
int left= A12;
int right= A14;
int back= A8;
#define N 5  // 每5組數據平均一次

float front_buf[N] = {0};
float back_buf[N]  = {0};
float left_buf[N]  = {0};
float right_buf[N] = {0};
int count = 0;
void setup(){
Robot_Init();
pinMode(front,INPUT);
pinMode(back,INPUT);
pinMode(left,INPUT);
pinMode(right,INPUT);
}
void loop() {
    readBNO085Yaw();

    // 讀取原始距離
    float f = analogRead(front)*MAX_RANG/ADC_SOLUTION;
    float b = analogRead(back) *MAX_RANG/ADC_SOLUTION;
    float l = analogRead(left) *MAX_RANG/ADC_SOLUTION;
    float r = analogRead(right)*MAX_RANG/ADC_SOLUTION;

    // 存入陣列
    front_buf[count] = f;
    back_buf[count]  = b;
    left_buf[count]  = l;
    right_buf[count] = r;
    count++;

    if(count >= N){  // 收集滿5組數據
        // 計算平均
        dist_f = 0;
        dist_b = 0;
        dist_l = 0;
        dist_r = 0;
        for(int i=0; i<N; i++){
            dist_f += front_buf[i];
            dist_b += back_buf[i];
            dist_l += left_buf[i];
            dist_r += right_buf[i];
        }
        dist_f /= N;
        dist_b /= N;
        dist_l /= N;
        dist_r /= N;

        // 計算速度
        if(abs(dist_r-dist_l)<5 ){ 
            vx=0; 
        }
        else{ 
            vx=(dist_r-dist_l)*0.8; 
            if(vx<15 && vx>0){
                vx=15;
            }
            else if(vx>-15&&vx<0){
                vx=-15;
            }
        }

        if(abs(dist_f-dist_b)<5 ){ 
            vy=0; 
        }
        else{ 
            vy=(dist_f-dist_b)*0.8;
            if(vy<15&& vy>0){
                vy=15;
            }
            else if(vy>-15 && vy<0){
                vy=-15;
            }
        }

        Vector_Motion(vx, vy);

        // 印出結果
        Serial.println("---");
        Serial.println(dist_f);
        Serial.println(dist_b);
        Serial.println(dist_l);
        Serial.println(dist_r);

        count = 0; // 重置計數
    }
}*/

void setup(){
Robot_Init();
pinMode(front,INPUT);
pinMode(back,INPUT);
pinMode(left,INPUT);
pinMode(right,INPUT);
}
void loop(){
readBNO085Yaw();
dist_f = analogRead(front)*MAX_RANG/ADC_SOLUTION;
dist_b = analogRead(back)*MAX_RANG/ADC_SOLUTION;
dist_l = analogRead(left)*MAX_RANG/ADC_SOLUTION;
dist_r = analogRead(right)*MAX_RANG/ADC_SOLUTION;
if(abs(dist_r-dist_l)<10 ){
    vx=0;
}
else{
    vx=(dist_r-dist_l)*0.5;
}
if(abs(dist_f-dist_b)<10 ){
    vy=0;
}
else{
    vy=(dist_f-dist_b)*0.5;
}



Serial.println("---");
Serial.println(dist_f);
Serial.println(dist_b);
Serial.println(dist_l);
Serial.println(dist_r);
delay(500);
}
