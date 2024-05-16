//ARDUINO 3 --> GYRO TEMPARATURE ESC

#include <MPU9255.h>
#include <Wire.h>
#include "DFRobot_SHT20.h"
#include <Servo.h>

#define g 9.81 // 1g ~ 9.81 m/s^2

//String incomingString = "";  // variable to hold incoming string

Servo esc;
int throttlePin = 9;
MPU9255 mpu;
DFRobot_SHT20 sht20;

double process_angular_velocity(int16_t input, scales sensor_scale )
{
   if(sensor_scale == scale_250dps)
  {
    return input/131;
  }

  //for +- 500 dps
  if(sensor_scale == scale_500dps)
  {
    return input/65.5;
  }

  //for +- 1000 dps
  if(sensor_scale == scale_1000dps)
  {
    return input/32.8;
  }

  //for +- 2000 dps
  if(sensor_scale == scale_2000dps)
  {
    return input/16.4;
  }

  return 0;
}

double process_acceleration(int input, scales sensor_scale )
{
   double output = 1;

  //for +- 2g

  if(sensor_scale == scale_2g)
  {
    output = input;
    output = output/16384;
    output = output*g;
  }

  //for +- 4g
  if(sensor_scale == scale_4g)
  {
    output = input;
    output = output/8192;
    output = output*g;
  }

  //for +- 8g
  if(sensor_scale == scale_8g)
  {
    output = input;
    output = output/4096;
    output = output*g;
  }

  //for +-16g
  if(sensor_scale == scale_16g)
  {
    output = input;
    output = output/2048;
    output = output*g;
  }

  return output;
}

void setup() {
  Serial.begin(115200);// initialize Serial port for computer communication
  Serial1.begin(9600);// initialize Serial1 port for sensor communication
  Serial2.begin(9600);
  esc.attach(throttlePin);
  
  if(mpu.init())
  {
    Serial.println("MPU initialization failed");
  }
  else
  {
    Serial.println("MPU initialization successful!");
  }

  sht20.initSHT20();   // Init SHT20 Sensor
  delay(100);
  sht20.checkSHT20();  // Check SHT20 Sensor  
}

void loop() {

  if (Serial.available()) {  // check if data is available to read
  
    char c = Serial.read();  // read a character
    incomingString += c;  // append character to incoming string
    if (c == '\n') {  // check for end of line character/
      if (incomingString == "1\n") {
         //esc çalış komutu
         esc.writeMicroseconds(2000); 
      }else{
        esc.writeMicroseconds(1000); // Motoru durdurma komutu
      }
      incomingString = "";  // clear incoming string
    }
  }
  
  //take readings
  mpu.read_acc();
  mpu.read_gyro();

  float temp = sht20.readTemperature();   // Read Temperature

  ////process and print acceleration data////
  //X axis
  String message = "";
  message += String(process_acceleration(mpu.ax,scale_2g));

  //Y axis
  message += ",";
  message += String(process_acceleration(mpu.ay,scale_2g));

  //Z axis
  message += ",";
  message += String(process_acceleration(mpu.az,scale_2g));

  ////process and print gyroscope data////
  //X axis
  message += ",";
  message += String(process_angular_velocity(mpu.gx,scale_250dps));

  //Y axis
  message += ",";
  message += String(process_angular_velocity(mpu.gy,scale_250dps));

  //Z axis
  message += ",";
  message += String(process_angular_velocity(mpu.gz,scale_250dps));

  //temparature
  message += ",";
  message += String(temp, 1);
  
  Serial.println(message);

  delay(100);