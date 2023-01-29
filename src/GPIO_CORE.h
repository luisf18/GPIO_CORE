/* ====================================================
 *  EasySensor Library
 *    Author: luisf18 (github)
 *    Ver.: 1.0.0
 *    last_update: 04/12/2022
 * ====================================================
 */

#ifndef GPIO_OUT_H
#define GPIO_OUT_H
#include <Arduino.h>
#include "Player.h"

#if defined(__AVR__)
  #include "processors/ATMEGA328/conf.h"
#elif defined(ESP32)
  #include "processors/ESP32/conf.h"
#elif defined(ESP8266)
  #include "processors/ESP8266/conf.h"
//#else
#endif


// TYPES ============================
#ifndef GPIO_DEFINES_H
#define GPIO_DEFINES_H

// in or out
#define GPIO_STATE_ON_LOW   0x80
#define GPIO_STATE_ON_HIGH  0x00
#define GPIO_state_on_LOW   0x80
#define GPIO_state_on_HIGH  0x00

// in or out
#define GPIO__IN   0
#define GPIO__OUT  1

// in
#define GPIO_IN__DIG          0
#define GPIO_IN__DIG_PULLDOWN 1
#define GPIO_IN__DIG_PULLUP   2
#define GPIO_IN__ADC          3
#define GPIO_IN__RMT          4
#define GPIO_IN__TOUCH        5

// out
#define GPIO_OUT__DIG         0
#define GPIO_OUT__PWM         1
#define GPIO_OUT__PWM_MOTOR   2
#define GPIO_OUT__PWM_SERVO   3
#define GPIO_OUT__DAC         4
#define GPIO_OUT__RMT         5

#define GPIO_IN_DIG       GPIO__IN  | (GPIO_IN__DIG          << 1)
#define GPIO_IN_DIG_DOWN  GPIO__IN  | (GPIO_IN__DIG_PULLDOWN << 1)
#define GPIO_IN_DIG_UP    GPIO__IN  | (GPIO_IN__DIG_PULLUP   << 1)
#define GPIO_ADC          GPIO__IN  | (GPIO_IN__ADC          << 1)
#define GPIO_IN_RMT       GPIO__IN  | (GPIO_IN__RMT          << 1)
#define GPIO_TOUCH        GPIO__IN  | (GPIO_IN__TOUCH        << 1)
#define GPIO_OUT_DIG      GPIO__OUT | (GPIO_OUT__DIG         << 1)
#define GPIO_PWM          GPIO__OUT | (GPIO_OUT__PWM         << 1)
#define GPIO_MOTOR        GPIO__OUT | (GPIO_OUT__PWM_MOTOR   << 1)
#define GPIO_SERVO        GPIO__OUT | (GPIO_OUT__PWM_SERVO   << 1)
#define GPIO_DAC          GPIO__OUT | (GPIO_OUT__ADC         << 1)
#define GPIO_OUT_RMT      GPIO__OUT | (GPIO_OUT__RMT         << 1)
#define GPIO_IN_DIG_PULLDOWN  GPIO_IN_DIG_DONN
#define GPIO_IN_DIG_PULLUP    GPIO_IN_DIG_UP

#define GPIO_TECH(type)     ((type&0x0F)>>1)
#define GPIO_SIGNAL(type)   ( type&1       ) // in or out
#define GPIO_STATE_ON(type) !(type>>7      )

#endif

// Normaly HIGH or LOW?
#define GPIO_NORMALLY_LOW  0x00
#define GPIO_NORMALLY_HIGH 0x10
#define GPIO_NORMALLY_HALF 0x20

typedef uint16_t (*gpio_core_read_t )(uint8_t,uint16_t);
typedef void     (*gpio_core_write_t)(uint8_t,uint16_t);

class GPIO_CORE{
   private:
    // Flags
    boolean OK  = false;
    boolean IN  = false;
    boolean OUT = false;

    // Basic
    uint8_t pin      = 0xFF;
    int     TYPE     = 0;
    uint8_t Tech     = 0;
    boolean state_on = HIGH;
    
    // Analog
    int      val     = 0;
    uint32_t AN_HZ   = 0;
    uint16_t AN_RES  = 0;
    uint16_t AN_MIN     = 0, AN_MAX     = 1;
    float    SET_MIN    = 0, SET_MAX    = 1000.0;
    uint16_t SET_MIN_AN = 0, SET_MAX_AN = 1;

    // OUT ==================================================
    gpio_core_write_t cb_write = nullptr;
    // PWM
    uint8_t  PWM_CH  = 0xFF;
    
    // RMT
    //uint8_t RMT_CH = 0;
    
    // LED
    uint16_t brightness = 0;

    // IR
    //void IR_state(boolean state); 
    //void sendIR_header();

    uint8_t wave = 0;
    
    // IN ===================================================
    gpio_core_read_t cb_read = nullptr;
    // INTERRUPT
    void   (*ISR_f)()        = nullptr;
    int      ISR_mode        = -1;
    volatile uint16_t State  = 0;
    volatile boolean  Change = false;
    volatile boolean  Rise   = false;
    volatile boolean  Fall   = false;
    volatile unsigned long last_change_us = 0;
    volatile uint16_t ISR_count;

    // MATH ===================================================
    // Linear Voltage Coef.
    float a_V = GPIO_CORE_VCC, b_V = 0;
    
    // Filters ( Threshold, Schmitt Trigger, ... )
    uint16_t sig = 0;
    uint8_t  Dig_Filter = 0;
    uint8_t  An_Filter  = 0;
    uint16_t Filter_low;
    uint16_t Filter_high;
    float    Filter_a;

    // Debounce
    uint16_t last_an  = 0;
    uint16_t db_delay = 50;
    uint16_t db_states = 0;
    uint32_t lastDebounceTime = 0;


  public:

    Player player;

    GPIO_CORE(uint8_t _pin, boolean _state_on, uint8_t _type);
    GPIO_CORE(uint8_t _pin, uint8_t _type);
    GPIO_CORE(){ pin=0xFF; }

    // Basic commands ============================================
    boolean reset();
    boolean begin();
    uint8_t gpio();
    void    stateOn(boolean _state_on);
    boolean active();

    // Read ======================================================
    uint16_t read(int _max);
    int      read(int _min, int _max);
    int      read();
    uint16_t readRaw();
    float    voltage();
    uint16_t value();

    // Write =====================================================
    void on();
    void off();
    void writeRaw(uint16_t _an);
    void write(int x,int _val_min,int _val_max);
    void write(int x,int _val_max);
    void write(uint16_t x);
    void voltage(int v);
    void mV(int _mV);

    // Analog ====================================================
    void reset_analog( uint16_t _min, uint16_t _max, uint16_t _res, uint32_t _HZ );
    void setRange(float _min, float _max);
    void setVoltageRange(float Vmin, float Vmax);
    void voltageRange(float Vmin, float Vmax);
    void setVoltageCoef(float _a, float _b);
    void setBrightness(uint16_t _brightness);

    // PWM =======================================================
    uint32_t frequency(  uint32_t F_HZ );
    uint8_t  resolution( uint8_t  RES  );
    uint32_t frequency();
    uint8_t  resolution();
    boolean  pwm_setup();
    uint8_t  pwm_setup(uint32_t F_HZ, uint8_t RES);
    #if defined(ESP32)
    void ESP32_pwm_channel(uint8_t ch);
    #endif

    // MATH ===================================================
    float    analog_to_voltage(uint16_t _an);
    uint16_t voltage_to_analog(float _V);
    uint16_t analog_to_mV(uint16_t _an);
    uint16_t mV_to_analog(uint16_t _mV);
    uint16_t analog_to_range(uint16_t _an);
    uint16_t range_to_analog(uint16_t _r );
    
    // Filters ====================================================
    int  filter_value();
    int  filter( uint16_t an );
    void filter_LowPass(float _a);
    void filter_debounce(uint16_t _db_delay, uint16_t _db_states);
    void digital_compare_voltage(float V_trig);
    void digital_compare(uint16_t _trig);
    void digital_SchmittTrigger_voltage(float V_low , float V_high);
    void digital_SchmittTrigger(uint16_t _an_low, uint16_t _an_high);

    // Control player =============================================
    uint8_t update();
    uint16_t handle_player(uint16_t i, uint16_t r);

    // Play functions  ============================================
    void play_blink(   uint16_t _T_ms );
    void play_blink_n( uint8_t n, uint16_t _dt );
    void play_square(uint16_t _T_ms, uint16_t dutycicle);
    void play_triangular( uint16_t _T_ms, uint16_t phase_i );
    void play_sawtooth(   uint16_t _T_ms, uint16_t phase_i );
    void play_sin( uint16_t _T_ms, uint16_t phase_deg );
    void play_cos( uint16_t _T_ms, uint16_t phase_deg );

    // Effects with delay =========================================
    void blink(int n, int dt);
    void fade(int n, int _T);

    // Print ======================================================
    void print_conf();

    // INTERRUPT ==================================================
    boolean  state ();
    //boolean isOn  ();
    //boolean isOff ();
    boolean fall  ();
    boolean rise  ();
    boolean change();
    uint16_t change_count();
    void state_reset();
    void stopInterrupt();
    void setInterrupt(int _mode);
    void setInterrupt(int _mode, void (*_ISR_extern_func)(void));
    void addISR(void (*_ISR_function)(void));
    
    // Callback ISR function
    #if defined(ESP32)
    void IRAM_ATTR HANDLE_ISR();
    #elif defined(ESP8266)
    void ICACHE_RAM_ATTR HANDLE_ISR();
    #elif defined(__AVR__)
    void HANDLE_ISR();
    #endif


    // OUTROS =====================================================
    // Servo
    //void pos(uint16_t _pos);
    
    // LED
    //void setBrightness(uint16_t _brightness);

    // IR
    //void sendIR_NEC(uint32_t code);
};

#endif