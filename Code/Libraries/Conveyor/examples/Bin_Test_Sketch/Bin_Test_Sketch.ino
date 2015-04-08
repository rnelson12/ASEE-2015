#include <SPI.h>
#include <Pixy.h>
#include <Conveyor.h>
#include <Servo.h>
/************************************
 * Test Sketch for Conveyor Library *
 ************************************
*/
Conveyor *bins;

void setup() {
  Serial.begin(9600);
  bins = new Conveyor();
}

void loop() {
  // put your main code here, to run repeatedly:
  unsigned long timer = millis();
  bool isDumped = false;
  
  while(!isDumped){
   isDumped = bins->dumpBins(timer);
  }
  while(!isDumped){
   isDumped = bins->dumpBins(timer);
  }
  while(!isDumped){
   isDumped = bins->dumpBins(timer);
  }
  while(!isDumped){
   isDumped = bins->dumpBins(timer);
  }
}
