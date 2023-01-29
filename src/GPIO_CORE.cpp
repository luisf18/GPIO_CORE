/* =====================================================
 *  GPIO OUT Library
 *    Author: luisf18 (github)
 *    Ver.: 0.0.1
 *    last_update: 04/11/2022
 * =====================================================
 */


#include "GPIO_CORE.h"
//#include "Player.h"

#if defined(__AVR__)
  #include "processors/ATMEGA328/interrupt.h"
  extern void (*GPIO_CORE_ISR[GPIO_CORE_ISR_PIN_COUNT])();
#elif defined(ESP32)
  #include "esp32-hal-ledc.h"
  #include "processors/ESP32/interrupt.h"
  extern void IRAM_ATTR (*GPIO_CORE_ISR[GPIO_CORE_ISR_PIN_COUNT])();
#elif defined(ESP8266)
  #include "core_esp8266_wiring_pwm.cpp"
  #include "processors/ESP8266/interrupt.h"
  extern void ICACHE_RAM_ATTR (*GPIO_CORE_ISR[GPIO_CORE_ISR_PIN_COUNT])();
#endif

// Read functions type
typedef uint16_t (*sensor_read_f_t)(uint8_t,uint16_t);

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////  EXTERNAL POINTERS  //////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

GPIO_CORE *GPIO_PTR[GPIO_CORE_PIN_COUNT];


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////  EXTERNAL READ FUNCTIONS  ///////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
uint16_t GPIO_CORE_CB_DIG_IN_HIGH(uint8_t pin,uint16_t Max){ return    digitalRead(pin); }
uint16_t GPIO_CORE_CB_DIG_IN_LOW (uint8_t pin,uint16_t Max){ return   !digitalRead(pin); }
uint16_t GPIO_CORE_CB_ADC_HIGH(uint8_t pin,uint16_t Max)   { return     analogRead(pin); }
uint16_t GPIO_CORE_CB_ADC_LOW (uint8_t pin,uint16_t Max)   { return Max-analogRead(pin); }

gpio_core_read_t GPIO_CORE_IN_CB[][2] = {
  { GPIO_CORE_CB_DIG_IN_LOW, GPIO_CORE_CB_DIG_IN_HIGH },
  { GPIO_CORE_CB_ADC_LOW, GPIO_CORE_CB_ADC_HIGH }
};

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////  EXTERNAL WRITE FUNCTIONS  ///////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*/
gpio_core_write_t GPIO_CORE_IN_CB[][2] = {
  //...
};
/*/


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////  CONSTRUCTOR  /////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

GPIO_CORE::GPIO_CORE(uint8_t _pin, boolean _state_on, uint8_t _type) {
  pin  = _pin;
  TYPE = _type;
  Tech = GPIO_TECH(TYPE);
  stateOn(_state_on );
}

GPIO_CORE::GPIO_CORE(uint8_t _pin,uint8_t _type){
  pin=_pin;
  TYPE=_type;
  Tech = GPIO_TECH(TYPE);
  stateOn(GPIO_STATE_ON(TYPE));
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////  BEGIN  ///////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
boolean GPIO_CORE::begin(){

  if(!reset()) return false;

  if( GPIO_SIGNAL(TYPE) == GPIO__OUT ){
    
    OUT = true;
    
    pinMode(pin,OUTPUT);
    
    switch (Tech){
      case GPIO_OUT__DIG       :  break;
      case GPIO_OUT__PWM       : OK = pwm_setup(); reset_analog(0,GPIO_CORE_PWM_MAX,GPIO_CORE_PWM_RES,0); break;
      case GPIO_OUT__PWM_MOTOR : OK = pwm_setup(25000,10); break;
      case GPIO_OUT__PWM_SERVO : OK = pwm_setup(100,10); setRange(/*35*/40,258); break;
      case GPIO_OUT__DAC       :  break;
      case GPIO_OUT__RMT       :  break;
      //case GPIO_OUT_DAC  :  break;
      //case GPIO_OUT_SERVO:  break;
      //case GPIO_OUT_PPM  :  break;
      //case GPIO_OUT_IR   :  break;
      //case GPIO_OUT_PIXEL:  break;
      //case GPIO_OUT_MAGIC:  break;
      //case GPIO_OUT_TX   :  break;
      default: OK = false; return false; break;
    }

    off();

  }else if( GPIO_SIGNAL(TYPE) == GPIO__IN  ){
    
    IN  = true;

    switch(Tech){
      case GPIO_IN__DIG       : pinMode(pin,INPUT       ); cb_read = GPIO_CORE_IN_CB[0][state_on]; break;
      case GPIO_IN__DIG_PULLUP: pinMode(pin,INPUT_PULLUP); cb_read = GPIO_CORE_IN_CB[0][state_on]; break;
      case GPIO_IN__ADC       :
        pinMode(pin,INPUT       );
        cb_read = GPIO_CORE_IN_CB[1][state_on];
        reset_analog(0,GPIO_CORE_ADC_MAX,GPIO_CORE_ADC_RES,0);
        voltageRange( 0, GPIO_CORE_VCC);
      break;
      #if defined(ESP32)
      case GPIO_IN__DIG_PULLDOWN: pinMode(pin,INPUT_PULLDOWN); cb_read = GPIO_CORE_IN_CB[0][state_on]; break;
      #endif
      //case CUSTOM: custom_type.setup();                                break;
      default: OK = false; return false; break;
    }

  }

  print_conf();

  return OK;

}

// Reset config. =========================================
boolean GPIO_CORE::reset(){
  
  IN  = false;
  OUT = false;

  cb_read = nullptr;
  cb_write = nullptr;

  An_Filter = 0;
  Dig_Filter = 0;
  wave   = 0;
  //mode   = -1; // << ---

  ISR_mode = -1;
  
  // Analog
  setRange(0,1000);
  reset_analog(0,1,1,0);
  voltageRange(0,GPIO_CORE_VCC);
  
  // Pointer
  if( pin >= GPIO_CORE_PIN_COUNT ) return false;
  GPIO_PTR[pin] = this;
  return true;
}

boolean GPIO_CORE::active(){ return OK; }
uint8_t GPIO_CORE::gpio(){ return pin; }

// IN suport...
void    GPIO_CORE::stateOn(boolean _state_on){ state_on = _state_on; }


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////  READ  ////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

uint16_t GPIO_CORE::read(int _max          ){ val = readRaw(); return map(((Dig_Filter || An_Filter)?filter(val):val),SET_MIN_AN,SET_MAX_AN,0,_max); }
int      GPIO_CORE::read(int _min, int _max){ val = readRaw(); return map(((Dig_Filter || An_Filter)?filter(val):val),SET_MIN_AN,SET_MAX_AN,_min,_max); }
int      GPIO_CORE::read(                  ){ val = readRaw(); return map(((Dig_Filter || An_Filter)?filter(val):val),SET_MIN_AN,SET_MAX_AN,0,1000); }
float    GPIO_CORE::voltage(){ val = readRaw(); return analog_to_voltage( ((Dig_Filter || An_Filter)?filter(val):val) ); }
uint16_t GPIO_CORE::readRaw(){ return ( cb_read != nullptr ? cb_read(pin,AN_MAX) : val ); }
uint16_t GPIO_CORE::value(){ return val; }

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////  WRITE  ///////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// GPIO_CORE Digital ==================================================
void GPIO_CORE::off(){ writeRaw(SET_MIN_AN); }
void GPIO_CORE::on() { writeRaw(SET_MAX_AN); }

// Range 0 to 1000 ====================================================
void GPIO_CORE::write(uint16_t x){
  if(Tech == GPIO_OUT__DIG ){
    writeRaw( x );
  } else {
    writeRaw( (x >= 1000 ? SET_MAX_AN : ( ((uint32_t)x*(SET_MAX_AN-SET_MIN_AN))/1000 + SET_MIN_AN ) ) );
  }
}
void GPIO_CORE::write(int x,int _val_max){ write(x,0,_val_max); }
void GPIO_CORE::write(int x,int _val_min,int _val_max){
  writeRaw(
    map(
      constrain(x,_val_min,_val_max),
      _val_min,
      _val_max,
      SET_MIN_AN,
      SET_MAX_AN
    )
  );
}

void GPIO_CORE::mV(int _mV){    writeRaw(mV_to_analog(_mV));    }
void GPIO_CORE::voltage(int v){ writeRaw(voltage_to_analog(v)); }
//void GPIO_CORE::pos(uint16_t _pos){ write( (100*_pos)/18 ); }

// GPIO_CORE Write ====================================================
void GPIO_CORE::writeRaw(uint16_t _an){
  if( OUT ){
    val = _an;
    if(Tech == GPIO_OUT__DIG ){
      digitalWrite(pin,(_an>0)==state_on);
    }else{
      #if defined(ESP32)
      ledcWrite(PWM_CH, (state_on?_an:AN_MAX-_an) );
      #else
      analogWrite(pin,(state_on?_an:(AN_MAX-_an)));
      #endif
    }
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////  MATH  ///////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Convert ===========================================================================
float    GPIO_CORE::analog_to_voltage(uint16_t _an){ return (a_V*_an+b_V);  }
uint16_t GPIO_CORE::voltage_to_analog(float _V    ){ return (a_V!=0?(_V-b_V)/a_V:0);  }
uint16_t GPIO_CORE::analog_to_mV(uint16_t _an){ return 1000*analog_to_voltage(_an);    }
uint16_t GPIO_CORE::mV_to_analog(uint16_t _mV){ return voltage_to_analog(_mV/1000.0);  }
uint16_t GPIO_CORE::analog_to_range(uint16_t _an){ return map(_an,SET_MIN_AN,SET_MAX_AN,0,1000); }
uint16_t GPIO_CORE::range_to_analog(uint16_t _r ){ return map(_r ,0,1000,SET_MIN_AN,SET_MAX_AN); }

int GPIO_CORE::filter_value(){ return sig; }
int GPIO_CORE::filter( uint16_t an ){
  if( Dig_Filter || An_Filter ){
         if(Dig_Filter == 1){ an = AN_MAX*(an > Filter_high); }
    else if(Dig_Filter == 2){ an = AN_MAX*( sig ? an >= Filter_low : an > Filter_high ); }
    
         if(An_Filter == 1){ sig = Filter_a*sig + (1.0-Filter_a)*an; an = sig; }
    else if(An_Filter == 2){
      if( Dig_Filter == 0 ) an = map(an,AN_MIN,AN_MAX,0,db_states-1);
      Serial.println("AN_MIN: " + String(AN_MIN));
      Serial.println("AN_MAX: " + String(AN_MAX));
      Serial.println("AN: " + String(an));

      Change = false;
      Fall   = false;
      Rise   = false;
      if( an != last_an ){ lastDebounceTime = millis(); }
      else if( ( millis() - lastDebounceTime ) > db_delay ){
        if( an != sig ){
          Fall = an < sig;
          Rise = an > sig;
          Change = true;
          sig = an;
          State = an;
        }
      }
      last_an = an;
      an = map(sig,0,db_states-1,AN_MIN,AN_MAX);
    }else{
      sig = an;
    }
  }
  return an;
}

// Schmitt Trigger
void GPIO_CORE::digital_compare_voltage(float V_trig){ digital_compare( analog_to_voltage(V_trig) ); }
void GPIO_CORE::digital_compare(uint16_t _trig){
  Dig_Filter = 1;
  Filter_high = _trig;
  sig = 0;
}
void GPIO_CORE::digital_SchmittTrigger_voltage(float V_low , float V_high){ digital_SchmittTrigger( analog_to_voltage(V_low), analog_to_voltage(V_high) ); }
void GPIO_CORE::digital_SchmittTrigger(uint16_t _an_low, uint16_t _an_high){
  Dig_Filter = 2;
  Filter_low  = _an_low;
  Filter_high = _an_high;
  sig = 0;
}
void GPIO_CORE::filter_LowPass(float _a){ An_Filter = 1; Filter_a = _a; sig = 0; }
void GPIO_CORE::filter_debounce(uint16_t _db_delay, uint16_t _db_states){ An_Filter = 2; db_delay = _db_delay; db_states = _db_states; sig = 0; }

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////  ANALOG  //////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void GPIO_CORE::reset_analog( uint16_t _min, uint16_t _max, uint16_t _res, uint32_t _HZ ){
  AN_MIN = _min;
  AN_MAX = _max;
  AN_RES = _res;
  AN_HZ  = _HZ;
  setRange( SET_MIN, SET_MAX );
  voltageRange( analog_to_voltage(SET_MIN_AN), analog_to_voltage(SET_MIN_AN));
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////  RANGE  ///////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void GPIO_CORE::setRange(float _min, float _max){
  if( _max > _min ){
    SET_MIN = _min;
    SET_MAX = _max;
    SET_MIN_AN = (SET_MIN/1000.0)*(AN_MAX-AN_MIN)+AN_MIN;
    SET_MAX_AN = (SET_MAX/1000.0)*(AN_MAX-AN_MIN)+AN_MIN;
  }
}
// Pensar sobre AN_MAX ou SET_MAX_AN ...
void  GPIO_CORE::setVoltageRange(float Vmin, float Vmax){
  setRange(
    (1000.0/(float)(AN_MAX-AN_MIN))*((Vmin-b_V)/a_V - AN_MIN),
    (1000.0/(float)(AN_MAX-AN_MIN))*((Vmax-b_V)/a_V - AN_MIN)
  );
}
void  GPIO_CORE::voltageRange(float Vmin, float Vmax){ setVoltageCoef((Vmax-Vmin)/(AN_MAX-AN_MIN),Vmin); }
void  GPIO_CORE::setVoltageCoef(float _a, float _b){ if(_a!=0){ a_V = _a; b_V = _b; } }

//void GPIO_CORE::setBrightness(uint16_t _brightness){ brightness = _brightness; }

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////  PWM  ///////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

uint32_t GPIO_CORE::frequency(  uint32_t F_HZ ){ if(OUT){ pwm_setup( F_HZ,AN_RES); Serial.print("F_HZ: "); Serial.println(AN_HZ ); return AN_HZ;  } }
uint8_t  GPIO_CORE::resolution( uint8_t  RES  ){ if(OUT){ pwm_setup(AN_HZ,   RES); Serial.print("RES: " ); Serial.println(AN_RES); return AN_RES; } }
uint32_t GPIO_CORE::frequency(){  return AN_HZ;  }
uint8_t  GPIO_CORE::resolution(){ return AN_RES; }

boolean GPIO_CORE::pwm_setup(){
  #if defined(ESP32) || defined(ESP8266)
  return pwm_setup(GPIO_CORE_PWM_HZ,GPIO_CORE_PWM_RES);
  #else
  reset_analog( 0, AN_MAX, GPIO_CORE_PWM_RES, GPIO_CORE_PWM_HZ );
  return true;
  #endif
}

// Only for esp32
#if defined(ESP32)
boolean ledc_channel_assigned[GPIO_CORE_PWM_COUNT];

uint8_t gpio_core_ledc_pin_count = 0;

uint8_t gpio_core_new_ledc_channel(){
  if( gpio_core_ledc_pin_count < GPIO_CORE_PWM_COUNT ){ // check if there are pins available
    uint8_t ch = GPIO_CORE_PWM_COUNT - gpio_core_ledc_pin_count - 1;
    while( ch && ledc_channel_assigned[ch] ) ch--;
    gpio_core_ledc_pin_count = GPIO_CORE_PWM_COUNT - ch; // update pin count
    if( ledc_channel_assigned[ch] ) return 0xFF;
    ledc_channel_assigned[ch] = true;
    return ch;
  }
  return 0xFF;
}

void GPIO_CORE::ESP32_pwm_channel(uint8_t ch){ PWM_CH = ch; ledc_channel_assigned[ch] = true; }

#endif

uint8_t GPIO_CORE :: pwm_setup(uint32_t F_HZ, uint8_t RES){
  
  #if defined(ESP32)
  if( RES > GPIO_CORE_PWM_MAX_RES ) return 0;
  // find a channel
  if(PWM_CH == 0xFF) PWM_CH = gpio_core_new_ledc_channel();
  if(PWM_CH == 0xFF) return 0;
  int PWM_HZ  = ledcSetup(PWM_CH,F_HZ,RES);
  ledcAttachPin(pin,PWM_CH);
  reset_analog( 0, (1<<RES)-1, RES, PWM_HZ );
  return 1 | (AN_HZ == F_HZ) << 1 | (AN_RES == RES) << 2 ;

  #elif defined(ESP8266)
  analogWriteFreq(F_HZ);
  analogWriteResolution(RES);
  // AN_MAX -> RES
  AN_RES = 0;
  uint32_t a = (analogScale + 1) >> 1;
  while( a ){ a = a >> 1; AN_RES++; }
  reset_analog( 0, analogScale, AN_RES, analogFreq );
  return 1 | (AN_HZ == F_HZ) << 1 | (AN_RES == RES) << 2 ;
  #else
  
  return 0;
  #endif
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////  PLAYER  /////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Control player =============================================
uint8_t GPIO_CORE::update(){
  if( OUT ){
    uint8_t p = player.update();
    if( p >= PLAYER_UPDATE && wave > 0 ){
           if( wave <= 2 ) write( player.value(),        1    ); // Blink and Blink_n
      else if( wave <= 5 ) write( player.value(),        1000 ); // Triangular, Sawtooth and square
      else if( wave <= 7 ) write( player.value(), -1000, 1000 ); // sin and cos
    }
    return p;
  }else if( IN ){
    //if( p >= PLAYER_UPDATE ){
    return read() > 0;
    //}
  }
  return 0;
}

// Play functions  ==========================================
void GPIO_CORE::play_blink(   uint16_t _T_ms ){ wave = 1; player.play_blink(_T_ms);   }
void GPIO_CORE::play_blink_n( uint8_t n, uint16_t _dt ){ wave = 2; player.play_blink_n(n,_dt); }
void GPIO_CORE::play_square(uint16_t _T_ms, uint16_t dutycicle){ wave = 3; player.play_square(_T_ms, dutycicle); }
void GPIO_CORE::play_triangular( uint16_t _T_ms, uint16_t phase_i ){ wave = 4; player.play_triangular(_T_ms,phase_i); }
void GPIO_CORE::play_sawtooth(   uint16_t _T_ms, uint16_t phase_i ){ wave = 5; player.play_sawtooth(_T_ms,phase_i);   }
void GPIO_CORE::play_sin( uint16_t _T_ms, uint16_t phase_deg ){ wave = 6; player.play_sin(_T_ms, phase_deg); }
void GPIO_CORE::play_cos( uint16_t _T_ms, uint16_t phase_deg ){ wave = 7; player.play_cos(_T_ms, phase_deg); }

// Blink with delay  ========================================
void GPIO_CORE::blink(int n, int dt){
  dt = dt/2;
  for(int i=0;i<n;i++){
    on();  delay(dt);
    off(); delay(dt);
  }
}

// Fade with delay  ========================================
void GPIO_CORE::fade(int n, int _T){
  uint16_t dt = (_T/200);
  for(int j=0;j<n;j++){
    for(int i=0;  i<100;i++){ write(10*i); delay(dt); }
    for(int i=100;i>0;  i--){ write(10*i); delay(dt); }
  }
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////  PRINT  /////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// PRINT ===================================================
void GPIO_CORE::print_conf(){
  Serial.println("\n===========================================" );
  Serial.println( "GPIO: " + String(pin) );
  Serial.println( "|- Flux: "  + String((IN?"IN":"")) + String((OUT?"OUT":"")) );
  Serial.println( "|- Type: "  + String(TYPE) );
  Serial.println( "|- Tech: "  + String(Tech) );
  Serial.println( "|- state ON: "  + String(state_on) );
  Serial.println( "|- Analog Freq [Hz]: "  + String(AN_HZ));
  Serial.println( "|- Analog Res [bits]: "  + String(AN_RES));
  Serial.println( "|- PWM_CH: "  + String(PWM_CH));
  Serial.println( "|- AN MIN: "  + String(AN_MIN));
  Serial.println( "|- AN MAX: "  + String(AN_MAX));
  Serial.println( "|- SET MIN: "  + String(SET_MIN));
  Serial.println( "|- SET MAX: "  + String(SET_MAX));
  Serial.println( "===========================================\n" );
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////  INTERRUPT  ///////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

boolean GPIO_CORE::state(){ return State;  }
//boolean GPIO_CORE::isOn  () { return  state(); }
//boolean GPIO_CORE::isOff () { return !state(); }
boolean GPIO_CORE::fall(){ return Fall;  }
boolean GPIO_CORE::rise(){ return Rise;  }
boolean  GPIO_CORE::change(){ return Change; }
uint16_t GPIO_CORE::change_count(){ return ISR_count; }
void     GPIO_CORE::state_reset() { Change = false; ISR_count=0; }
//void     GPIO_CORE::count_reset() { Change = false; ISR_count=0; }

// Set, add, Stop
void GPIO_CORE::addISR(void (*_ISR_function)(void)){ ISR_f = _ISR_function; }
void GPIO_CORE::stopInterrupt(){
  ISR_mode = -1;
  detachInterrupt( pin + GPIO_CORE_ISR_PIN_OFFSET );
}
void GPIO_CORE::setInterrupt(int _mode){ Serial.println("ISR_PIN: "+String(pin + GPIO_CORE_ISR_PIN_OFFSET)); setInterrupt( _mode, GPIO_CORE_ISR[ pin + GPIO_CORE_ISR_PIN_OFFSET ] ); }
void GPIO_CORE::setInterrupt(int _mode, void (*_ISR_extern_func)(void)){
  ISR_mode = _mode;
  attachInterrupt(
    pin + GPIO_CORE_ISR_PIN_OFFSET,
    _ISR_extern_func,
    _mode
  );
}

// Callback function
#if defined(ESP32)
void IRAM_ATTR GPIO_CORE::HANDLE_ISR()
#elif defined(ESP8266)
void ICACHE_RAM_ATTR GPIO_CORE::HANDLE_ISR()
#elif defined(__AVR__)
void GPIO_CORE::HANDLE_ISR()
#endif
{
  last_change_us = micros();
  Change = true;
  ISR_count++;
  if(ISR_f!=nullptr)ISR_f();
}



/*/ SEND SONY Functions ==============================
void GPIO_CORE::sendIR_NEC(uint32_t code){
  #if defined(ESP32)
  off();
  sendIR_header();
  for (int i=14; i >= 0; i--) IR_state(code & 1<<i);
  off();
  #endif
}

void GPIO_CORE::IR_state(boolean state){
  #if defined(ESP32)
  writeTone(IR_CH, 38000);
  delayMicroseconds(600*(state+1));
  writeTone(IR_CH, 0);
  delayMicroseconds(600);
  #endif
}

void GPIO_CORE::sendIR_header(){
  #if defined(ESP32)
  GPIO_OUTcWriteTone(pin, 38000);
  delayMicroseconds(2400);
  GPIO_OUTcWriteTone(pin, 0);
  delayMicroseconds(600);
  #endif
}

// =================================================
*/