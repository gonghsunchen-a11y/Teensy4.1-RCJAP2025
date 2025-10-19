#include <Arduino.h>
#include <Robot.h>
void setup(){
    Robot_Init();
}
void loop(){
    linesensor();
    Serial.println(lineData.state,BIN);
    delay(500);
}
/*void loop(){
    readBNO085Yaw();
    ballsensor();
    Serial.print("dir");
    Serial.println(ballData.dir);
    Serial.print("dis");
    Serial.println(ballData.dis);

       if (ballData.dir == 3 || ballData.dir == 4) {
        if (ballData.dis <= 2) {
            // 太近 → 停下
            Vector_Motion(0, 0);
           // Serial.println("Ball close - stop");
        } else {
            // 根據距離減速靠近
            int speed = map(ballData.dis, 2, 8, 0, 30);  // 距離越近速度越慢
            speed = constrain(speed, 0, 30);    
            Vector_Motion(0, speed);  // 往前
            Serial.print("Moving forward, speed = ");
        }
    } else {
        // 球不在前方 → 停止
        Vector_Motion(0, 0);
      //  Serial.println("Ball not in front - stop");
        
    }

}*/