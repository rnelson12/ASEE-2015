#include <SPI.h>
#include <Pixy.h>
#include <Wire.h>
#include <Sensors.h>
Compass* compass;

void setup() {
  Serial.begin(9600);
  compass = new Compass(0.069522276053);
}

void loop() {
  Serial.print("Init degrees: "); 
  Serial.println(compass->getInitDegrees());
  Serial.println(compass->getDegrees());
}
