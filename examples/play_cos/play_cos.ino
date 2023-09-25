
#include "GPIO_CORE.h"
GPIO_CORE BTN(0,LOW,GPIO_IN_DIG_UP);
GPIO_CORE LED(2,LOW,GPIO_PWM);

void setup() {
  Serial.begin(115200);
  LED.begin();
  BTN.begin();
  BTN.filter_debounce(50,2);
  Serial.println( "[S,R,F,C]" );
}

boolean ledOn = false;

void loop() {
  
  BTN.update();
  LED.update();

  if( BTN.change() ){
    Serial.println( "[" + String( BTN.state() ) + "," + String( BTN.rise() ) + "," + String( BTN.fall() ) + "," + String( BTN.change() ) + "]" );
    if(BTN.rise()){
      ledOn = !ledOn;
      if( ledOn ){ LED.play_cos(3000,180); }else{ LED.player.stop(); LED.off(); LED.player.reset(); }
    }
  }

}
