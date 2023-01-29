#include "GPIO_OUT.h"

GPIO_OUT LED(3,HIGH,GPIO_OUT_PWM);

void setup() {
  Serial.begin(115200);
  //LED.ESP32_pwm_channel(0);
  LED.setFreq(1000);
  LED.begin();
}

boolean mode = 0;

void loop() {
  LED.writeRaw(0); delay(1000);
  LED.writeRaw(255); delay(1000);
  LED.writeRaw(1000); delay(1000);
  LED.setResolution( mode ? 8 : 10 );
  mode = !mode;
}
