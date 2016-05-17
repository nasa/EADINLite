#include "eadin.h"

EADIN TTL(0x00);

uint8_t tmp[8] = {0xa1,0xb2,0xc3,0xd4,0xe5,0xf6,0xe7,0xd8};

unsigned long startTime,endTime;

void setup() {
  TTL.begin(&Serial1,115200,4);
  Serial.begin(115200);
  delay(2000);
  Serial.println("EADIN Master Test Script.");
  Serial.println("Defaults: qCRC=loaded, qCRC=ON, baud=115200, comPort=Serial1, debug=OFF"); 
}

void loop() {
  uint8_t tmp2[8] = {0x00};
  startTime = micros();
  TTL.write(tmp,0x01);
  int flag = TTL.read(tmp2);
  endTime = micros() - startTime;
  Serial.print("Flag : ");
  Serial.print(flag);
  Serial.print(", Round Trip Time (micros) : ");
  Serial.println(endTime);
  delay(1000);
}
