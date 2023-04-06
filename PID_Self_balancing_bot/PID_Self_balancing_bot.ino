#include "Wire.h"
 #include <MPU6050_light.h>
 
 MPU6050 mpu(Wire);
int enA = 5;
int in1 = 6;
int in2 = 7;
// Motor B connections
int enB = 10;
int in3 = 8;
int in4 = 9;
 float roll;
 float d, p, i, i_temp, error, prev_error=0, PID, throttle;
 float kp = 40;
 float kd = 9;
 float ki = 10;

 unsigned long timer = 0;
 void setup() {
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  
  // Turn off motors - Initial state
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
   Serial.begin(9600);
   Wire.begin();
 byte status = mpu.begin();
   Serial.print(F("MPU6050 status: "));
   Serial.println(status);
   while (status != 0) { } // stop everything if could not connect to MPU6050
 Serial.println(F("Calculating offsets, do not move MPU6050"));
   delay(1000);
   mpu.calcOffsets(); // gyro and accelero
   Serial.println("Done!\n");
   
 }
 void loop() {
   mpu.update();
 if ((millis() - timer) > 10) { // print data every 10ms
     Serial.print("Y: ");
     roll = mpu.getAngleY();
     Serial.println(roll);
     error = roll;
     p = kp*error;
     d = kd*(error - prev_error);
     i_temp = i_temp + kp*error;
     if (i_temp>90) {
      i_temp = 90;
     }
     else if (i_temp<-90) {
      i_temp = -90;
     }
     i = ki * i_temp;
     PID = p+i+d;
     if (PID>90) {
      PID = 90;
     }
     else if (PID<-90) {
      PID = -90;
     }
     prev_error = error;
     timer = millis();
     if (PID>=0){
      throttle = map(PID, 0, 90,  0, 255);
      Serial.print("throttle: ");
      Serial.println(throttle);
      analogWrite(enA, throttle);
      analogWrite(enB, throttle);
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
      digitalWrite(in3, HIGH);
      digitalWrite(in4, LOW);
     }
     else {
      throttle = map(PID, -90, 0, 255, 0);
      Serial.print("throttle: ");
      Serial.println(throttle);
      analogWrite(enA, throttle);
      analogWrite(enB, throttle);
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
      digitalWrite(in3, LOW);
      digitalWrite(in4, HIGH);
     }
     
  }
  delay(100);
  
 }
