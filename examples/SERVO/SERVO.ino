#include "GPIO_OUT.h"

GPIO_OUT  SERVO(2,GPIO_SERVO);

void setup() {
  Serial.begin(115200);
  SERVO.begin();
  SERVO.play_cos(3000);
  //SERVO.play_fade(3000);
  //SERVO.play_blink(3000);
}

void loop() {
  SERVO.update();
}
