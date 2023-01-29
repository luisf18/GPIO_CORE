#include "GPIO_CORE.h"

GPIO_CORE V(2,LOW,GPIO_IN_DIG_UP);
GPIO_CORE LED(13,GPIO_OUT_DIG);

void setup() {
  Serial.begin(115200);
  V.begin();
  //V.filter_LowPass(0.8);
  //V.digital_compare(500);
  //V.filter_debounce(10,2);
  V.setInterrupt(FALLING);
  LED.begin();
}

void loop() {

  if( V.change() ){
    LED.write(!LED.read());
    Serial.println(V.change_count());
    V.state_reset();
  }

  //Serial.println( V.read() );

  delay(50);

  //Serial.println( String(V.value()) + " " + String(V.filter_value()) );

}
