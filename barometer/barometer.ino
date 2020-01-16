/*********
  i2c BAROMETER_ADDRESS 0x76
*********/

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>

/*#include <SPI.h>
#define BMP_SCK 18
#define BMP_MISO 19
#define BMP_MOSI 23
#define BMP_CS 5*/

#define SEALEVELPRESSURE_HPA (1013.25)
#define DEBUG 0

Adafruit_BMP280 bmp; // I2C
//Adafruit_BMP280 bmp(BMP_CS); // hardware SPI
//Adafruit_BMP280 bmp(BMP_CS, BMP_MOSI, BMP_MISO, BMP_SCK); // software SPI

unsigned long delayTime;

void setup() {
  Serial.begin(9600);
  if(DEBUG){
    Serial.println("BMP280 test");
  }

  bool status;

  // default settings
  // (you can also pass in a Wire library object like &Wire2)
  status = bmp.begin();
  if (!status) {
    Serial.println("Could not find a valid BMP280 sensor, check wiring!\n");
    while (1);
  }

  if(DEBUG){
    Serial.println("-- Default Test --\n");
  }
  delayTime = 500;

}

void loop() { 
  printValues();
  delay(delayTime);
}

void printValues() {
  // Serial.print("Temperature = ");
  // Serial.print("[");
  Serial.print(bmp.readTemperature());
  // Serial.println(" *C");
  
  // Convert temperature to Fahrenheit
  /*Serial.print("Temperature = ");
  Serial.print(1.8 * bmp.readTemperature() + 32);
  Serial.println(" *F");*/
  
  // Serial.print("Pressure = ");
  Serial.print(",");
  Serial.print(bmp.readPressure());
  // Serial.print(bmp.readPressure() / 100.0F);
  // Serial.println(" hPa");

  // Serial.print("Approx. Altitude = ");
  Serial.print(",");
  Serial.println(bmp.readAltitude(SEALEVELPRESSURE_HPA));
  // Serial.println("]");
  // Serial.println(" m");
}