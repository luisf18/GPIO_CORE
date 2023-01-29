#include "GPIO_CORE.h"

GPIO_CORE LED(2,LOW,GPIO_PWM);
//GPIO_CORE GND(9,HIGH,GPIO_OUT_DIG);

int i = 0;

void setup() {
  Serial.begin(115200);
  //GND.begin();
  //GND.off();
  LED.begin();

  LED.on();  delay(1000);
  LED.off(); delay(1000);

  LED.setRange(0,800);
  LED.blink(5,200);
  LED.print_conf();
}

void loop() {
  int a = LED.update();
  
  //if( a >= PLAYER_UPDATE ){ Serial.println( LED.player.value() ); }

  switch ( ( a ? -1 : i) ) {
      case 0: LED.off();delay(1000);Serial.println("BLINK");LED.player.repeat(5); LED.play_blink(1000);        i = 1; break;
      case 1: LED.off();delay(1000);Serial.println("FADE"); LED.player.repeat(5); LED.play_triangular(1000,0); i = 2; break;
      case 2: LED.off();delay(1000);Serial.println("SINE"); LED.player.repeat(5); LED.play_sin(1000,0);        i = 0; break;
      default: break;
  }
  delay(2);
}
