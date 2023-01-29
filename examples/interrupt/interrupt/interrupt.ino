#include "GPIO_CORE.h"

GPIO_CORE BTN(0,LOW,GPIO_IN_DIG_UP);
GPIO_CORE LED(2,LOW,GPIO_OUT_DIG);

void setup() {
  Serial.begin(115200);
  LED.begin();
  BTN.begin();
  BTN.setInterrupt(FALLING);
}

void loop() {
  LED.write(BTN.read());
  Serial.println(BTN.change());
  delay(100);
}