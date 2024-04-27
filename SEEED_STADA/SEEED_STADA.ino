#include <Servo.h>

#include <Arduino.h>
#include <LSM6DS3.h>
#include "LSM6DS3.h"
#include "Wire.h"
#include <Wire.h>

//Create an instance of class LSM6DS3
LSM6DS3 myIMU(I2C_MODE, 0x6A);    //I2C device address 0x6A

Servo servo_1; // servo controller (multiple can exist)
Servo servo_2; // servo controller (multiple can exist)

int servo_pin1 = 8; // PWM pin for servo control
int servo_pin2 = 9; // PWM pin for servo control
int pos = 0;    // servo starting position


void setup() {
  Serial.begin(9600); //starting usb com
  Serial1.begin(9600); //start uart com (in/tx is pin 6, out/rx is 7)
  servo_1.attach(servo_pin1); // start servo control
  servo_2.attach(servo_pin2); // start servo control
  if (myIMU.begin() != 0) {
        Serial.println("Device error");
    } else {
        Serial.println("Device OK!");
    }
}

int get_number(int start_num, int end_num, String str){
    str = str.substring(0);
    String s_number = "";
    for (int i = start_num; i < end_num; i++){
      s_number+= str[i];
    }
    return s_number.toInt();
}
void loop() {
  //imu
     //Serial.print("Time: ");
    unsigned long myTime = millis();
    //Serial.println(myTime); // prints time since program started
  
    //Accelerometer
    float accelx = myIMU.readFloatAccelX();  // Read IMU value from A0
    //Serial.print(" accel x = ");
    //Serial.println(imuValue);
    float accely = myIMU.readFloatAccelY();  // Read IMU value from A0
    //Serial.print(" accel y = ");
    //Serial.println(imuValue);
    float accelz = myIMU.readFloatAccelZ();  // Read IMU value from A0
    //Serial.print(" accel z = ");
    //Serial.println(imuValue);

//GYRO
    float gyrox = myIMU.readFloatGyroX();  // Read IMU value from A0
    //Serial.print(" gyro x = ");
    //Serial.println(imuValue);

    float gyroy = myIMU.readFloatGyroY();  // Read IMU value from A0
    //Serial.print(" gyro y = ");

    float gyroz = myIMU.readFloatGyroZ();  // Read IMU value from A0
    //Serial.print(" gyro z = ");
  
  //fsr
    int read_val0 = analogRead(A0);  // Generate PWM signal
    int read_val1 = analogRead(A1);  // Generate PWM signal
    int read_val2 = analogRead(A2);  // Generate PWM signal
    int read_val3 = analogRead(A3);  // Generate PWM signal

  //XBEE
    String xbee_write = String(accelx)+" "+ String(accely)+" "+ String(accelz)+" "+ String(gyrox)+" "+ String(gyroy)+" "+ String(gyroz)+" "+ String(read_val0)+" "+ String(read_val1)+" "+ String(read_val2)+" "+ String(read_val3);
    Serial1.println(xbee_write); //sending message through xbee
    String xbee_read = Serial1.readString(); //reading message through xbee
    Serial.println(xbee_read); 
    int xbee_read_length = xbee_read.length();

    //is it motor?
    if (xbee_read[0]=='m'){
      int xbee_read_number =  get_number(3, xbee_read_length, xbee_read); //get angle
      //1st motor
      if (xbee_read[1]=='1'){
        servo_1.write(xbee_read_number);
      }
      if (xbee_read[1]=='2'){
        servo_2.write(xbee_read_number);
      }
      delay(500); 
    }
   

}
