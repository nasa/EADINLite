#include "eadin.h" // custom libary included later!

EADIN TTL(0x01);

uint8_t tmp[8] = {0xaa,0xbb,0xcc,0xdd,0xee,0xff,0xee,0xdd};
uint8_t tmp2[8] = {0x00};

//float lastGood;

void setup() {
  TTL.begin(&Serial1,115200,4);
  //Serial.begin(115200);
}

void loop() {
  int flag = TTL.read(tmp2);
  if (flag==0x01){
    TTL.write(tmp);
    digitalWrite(13,HIGH);
    //lastGood = millis();
  }
  //if (millis() > 10000 + lastGood) {digitalWrite(13,LOW);}
  //if (flag!=2){Serial.println(flag,HEX);}
}
