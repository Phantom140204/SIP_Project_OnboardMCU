#include <Arduino.h>
#include <string.h>
#include <SPI.h>
#include <Wire.h>
#include "MotorControl.h"
#include <LoRa.h>
#include <FXAS21002C_Basic.h>
#include <FXOS8700Q_Basic.h>
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include <MadgwickAHRS.h>

//Connections on Arduino Mega - 
// VCC - 3.3V, GND - GND
// CSN - CSN_PIN, CE - CE_PIN
// SCK - 52, MOSI - 51, MISO - 50

// #define MOTOR1_PINS PA0,PA1
// #define MOTOR2_PINS PA2,PA3
#define RXPin PB8
#define TxPin PB9

// void setMotors(String keypress);
void LoRa_Init();

// String previous_keypress = "";
long prev_time = millis();
// MotorControl motor1(MOTOR1_PINS), motor2(MOTOR2_PINS);
TinyGPSPlus gps;
SoftwareSerial gpsSerial(RXPin, TxPin);
TwoWire Wire1(PB7, PB6);
SPIClass spi1(PB5, PB4, PB3);

float Acc[3]  ;
float Mag[3]  ;
float Gyr[3]  ;

//Class Declarations
FXAS21002CBasic Gyro(0x21 , &Wire1);      // Gyroscope sensor class declaration
FXOS8700QBasic AcMg(1 , 0x1F , &Wire1);   // Magnetometer and Accelerometer sensors classes declarations
Madgwick Filter;
// highpass R(0.98);
// highpass P(0.98);
// highpass Y(0.98);
float present ;

// float pres, prev;

void setup() {
  Wire1.begin();
  Serial.begin(9600);
  gpsSerial.begin(9600);
  LoRa_Init();
}

void loop() {
  
  if(millis() - prev_time > 10){
    LoRa.beginPacket();
    LoRa.print("Orientation: ");
    LoRa.print(Filter.getYaw());
    LoRa.print(" ");
    LoRa.print(Filter.getPitch());
    LoRa.print(" ");
    LoRa.print(Filter.getRoll());
    LoRa.print(" ");
    LoRa.print(gps.location.lat());
    LoRa.print(" ");
    LoRa.print(gps.location.lng());
    LoRa.endPacket();
    prev_time = millis();
    // Serial.println("Data Transmitted: ");
      Serial.print("Orientation: ");
    Serial.print(Filter.getYaw());
    Serial.print(" ");
    Serial.print(Filter.getPitch());
    Serial.print(" ");
    Serial.print(Filter.getRoll());
    Serial.print(" ");
    Serial.print(gps.location.lat());
    Serial.print(" ");
    Serial.println(gps.location.lng());

  }
  Gyro.updateGyroData(&Gyr[0]);
  AcMg.updateAccelMagData(&Acc[0] , &Mag[0]);
  Filter.update(Gyr[0] , Gyr[1] , Gyr[2] , Acc[0] , Acc[1] , Acc[2] , Mag[0] , Mag[1] , Mag[2] );
  while (gpsSerial.available() > 0){
   gps.encode(gpsSerial.read());
  }
  
}


void LoRa_Init(){
  LoRa.setSPI(spi1);
  LoRa.setPins(PB12,PB13,PB14);
  if(LoRa.begin(433E6)){
    Serial.println("LoRa Initialization Successful");
  }
  else{
    Serial.println("LoRa Initialization Failed");
    while(1);
  }
  LoRa.setTxPower(20);
}
