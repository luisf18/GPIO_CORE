#include "GPIO_CORE.h"

GPIO_CORE V(A0,GPIO_ADC);

void setup() {
  Serial.begin(115200);
  V.begin();
  //V.filter_LowPass(0.8);
  V.filter_debounce(50,5);
}

void loop() {
  
  V.read();
  Serial.println( V.filter_value() );
  delay(10);

}
