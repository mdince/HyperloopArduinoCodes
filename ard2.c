//ARD-2 PHOTOELECTRİC-BRAKE-ESC-TEMPARATURE

#include <Wire.h>
#include "DFRobot_SHT20.h"
#include <Servo.h>

String incomingString = "";  // variable to hold incoming string

const int RELAY_PIN = 12;  //FOR BRAKE SYSTEM
const int BRAKING_TIME = 500;

DFRobot_SHT20 sht20; //FOT TEMPARATURE SENSOR

Servo esc; //FOR ESC
int throttlePin = 9;

const int sensorPin = 2; // FOR PHOTOELECTRİC SENSOR
int counter = 0;
int previous_value = 0; 
int sensorValue = 0;

void setup() {
  
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(sensorPin, INPUT); //temp
  digitalWrite(RELAY_PIN, HIGH); // Set the relay to normally closed
  Serial.begin(9600);  // Start serial communication
  Serial1.begin(9600);// initialize Serial1 port for temp sensor communication

  sht20.initSHT20();   // Init SHT20 Sensor
  delay(100);
  sht20.checkSHT20();  // Check SHT20 Sensor

  esc.attach(throttlePin);

}

void loop() {

  float temp = sht20.readTemperature();   // Read Temperature
  int sensorValue = digitalRead(sensorPin); // Sensörün durumunu oku
  String message = "";

  if (Serial.available()) {  // check if data is available to read
  
    char c = Serial.read();  // read a character
    incomingString += c;  // append character to incoming string
    if (c == '\n') {  // check for end of line character
      if (incomingString == "00\n") {  
        //fren, esc kapalı
        esc.writeMicroseconds(1000); // Motoru durdurma komutu.
        digitalWrite(RELAY_PIN, HIGH);
      }else if (incomingString == "01\n"){
        //fren kapalı esc açık
        esc.writeMicroseconds(2000); // Motoru başlatma komutu.
        digitalWrite(RELAY_PIN, HIGH);
      }else if (incomingString == "10\n"){// check if incoming string is "1" for brake
        //fren açık
        // Engage the brake by deactivating the relay
        digitalWrite(RELAY_PIN, LOW);
        delay(BRAKING_TIME);
        // Disengage the brake by activating the relay
      }
      incomingString = "";  // clear incoming string
    }
  }

  message += String(temp, 1);
  message += ",";

  if (sensorValue == HIGH) {
    if(previous value == 0){
     counter = counter + 1; 
    }
  }

  previous_value = sensorValue;

  message += counter;

  Serial.println(message);
}