#include "GPIO_CORE.h"

GPIO_CORE V(A0,GPIO_ADC);

void setup() {
  Serial.begin(115200);
  V.begin();
  V.filter_LowPass(0.8);
}

void loop() {
  
  float v_filter = V.voltage();
  float v_real   = V.analog_to_voltage(V.value());

  Serial.println( String( v_filter ) + "V " + String( v_real ) + "V" );

  delay(10);

}
