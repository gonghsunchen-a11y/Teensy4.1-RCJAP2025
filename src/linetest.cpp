#include <Arduino.h>
#include <Robot.h>

const int Pin = 41;     
const int Pin1 = 40;    
const int Pin2 = 39;   

volatile bool centerTouch = false;
volatile bool leftTouch   = false;
volatile bool rightTouch  = false;
void stop();
void stop1();
void stop2();

int val = 0;  // var
int val1 = 0;  // var
int val2 = 0;  // var
void setup(){
    Robot_Init();
    pinMode(Pin, INPUT_PULLUP);
    pinMode(Pin1, INPUT_PULLUP);
    pinMode(Pin2, INPUT_PULLUP);

    Serial.begin(9600);
    attachInterrupt(digitalPinToInterrupt(Pin), stop, RISING);
    attachInterrupt(digitalPinToInterrupt(Pin1), stop1, RISING);
    attachInterrupt(digitalPinToInterrupt(Pin2), stop2, RISING);

//analogReadResolution(12);
    
}

void loop(){
    /*val = analogRead(Pin);
    val1 = analogRead(Pin1);
    val2 = analogRead(Pin2);
    // read the input pin
    Serial.println("Pin");
    Serial.println(val); 
    Serial.println("Pin1");
    Serial.println(val1);
    Serial.println("Pin2");         // debug value
    Serial.println(val2); 
    delay(1000);*/

    /*digitalRead(Pin);
    digitalRead(Pin1);
    digitalRead(Pin2);
    Serial.println("Pin");
    Serial.println(digitalRead(Pin)); 
    Serial.println("Pin1");
    Serial.println(digitalRead(Pin1)); 
    Serial.println("Pin2");         // debug value
    Serial.println(digitalRead(Pin2)); 
    delay(1000);*/
    readBNO085Yaw();
    if(centerTouch==true){
        Serial.println("back");
        Vector_Motion(0,30);  
        if(digitalRead(Pin) == 0){
        Vector_Motion(0,0);          
            centerTouch = false;    
        }
            
    }
    else if(leftTouch==true){
        Serial.println("right");
        Vector_Motion(30,0);  
        if(digitalRead(Pin1) == 0){
            Vector_Motion(0,0);        
            leftTouch = false;
    }
    }
    else if(rightTouch==true){
        Serial.println("left");
        Vector_Motion(-30,0); 
        if(digitalRead(Pin2) == 0){
        Vector_Motion(0,0);       
            rightTouch = false;   
        }
            
    }
    else{
        Serial.println("RUNNING");
        Vector_Motion(0,0);   
    }    
    
}   
void stop(){
    centerTouch = true;
}
void stop1(){
    leftTouch = true;
}
void stop2(){
    rightTouch = true;
}