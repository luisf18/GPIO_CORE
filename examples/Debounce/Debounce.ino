#include "GPIO_CORE.h"

GPIO_CORE BTN(0,LOW,GPIO_IN_DIG_UP);
GPIO_CORE LED(2,LOW,GPIO_OUT_DIG);

void setup() {
  Serial.begin(115200);
  LED.begin();
  BTN.begin();
  BTN.filter_debounce(50,2);
  Serial.println( "[S,R,F,C]" );
}

void loop() {
  
  BTN.update();

  if( BTN.change() ){
    Serial.println( "[" + String( BTN.state() ) + "," + String( BTN.rise() ) + "," + String( BTN.fall() ) + "," + String( BTN.change() ) + "]" );
    if(BTN.rise()){
      LED.write( !LED.read() );
    }
  }

}
