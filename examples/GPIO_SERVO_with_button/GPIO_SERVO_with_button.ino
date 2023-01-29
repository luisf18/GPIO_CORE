#include "GPIO_OUT.h"
#include "EasySensor.h"

Sensor    BTN(0,LOW,GPIO_IN_DIG_PULLUP);
GPIO_OUT  SERVO(2,GPIO_SERVO);

void setup() {
  Serial.begin(115200);
  SERVO.begin();
  SERVO.play_cos(2000);
  BTN.begin();
}

void loop() {
  if( BTN.read() ){
    delay(200);
  }else{
    if(SERVO.update() > 1){
      Serial.println(SERVO.read());
    }
  }
}
