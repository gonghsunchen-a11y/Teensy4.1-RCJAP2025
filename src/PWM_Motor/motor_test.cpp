#include <Arduino.h>
#include <Robot.h>


void setup() {
Robot_Init();
}

void loop(){
readBNO085Yaw();
Vector_Motion(0,10);
}