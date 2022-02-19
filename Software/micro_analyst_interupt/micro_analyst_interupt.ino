#include <RingBuf.h>
#include <EwmaT.h>
#include <Ewma.h>

/*
  Micro Analyst Pedal Assistance converter.
  Takes torque and cadence as inputs and outputs a throttle signal.
 
  This example code is in the public domain.
 */

//Select arduino version
#define TINY84

// Debug messages
//#define DEBUG // Comment out this line to disable debugging.
#ifdef DEBUG
static char buff[100]={}; 
#define debug_printf(...) sprintf(buff, __VA_ARGS__); Serial.println(buff)
#else
#define debug_printf(...)
#endif

// Pin names:
#ifdef UNO
#define cadence_sensor 2 // This pin must be an interupt pin. For UNO it is INT0 on Pin 2
#define torque_sensor A1 // Any analog pin
#define throttle 11 // This must be a PWM pin on TIMER1 (pin 9 or 10)
#define cadence_output_sim 6
#endif

#ifdef TINY84
#define cadence_sensor 8 // This pin must be an interupt pin. For ATTINY84a it is INT0 on Pin 8
#define torque_sensor 7 // Any analog pin
#define throttle 5 // This must be a PWM pin on TIMER1 (pin 5 or 6)
#define cadence_output_sim 0
#endif

#ifdef TINY85
#define cadence_sensor 2 // This pin must be an interupt pin. For ATTINY85 it is INT0 on Pin 2
#define torque_sensor 3 // Any analog pin
#define throttle 4 // This must be a PWM pin on TIMER1 (pin 4 or 1?)
#define cadence_output_sim 0
#define TCCR1B TCCR1
#endif

// micro constants
#define MICRO_REFERENCE_VOLTAGE 5u
#define MICRO_ADC_RESOLUTION 1024u
#define TOGGLE(x) digitalWrite(x, !bool(digitalRead(x)))

// sensor constants
#define POLES_PER_REV 36u //36.0
#define TORQUE_SENSOR_OFFSET_VOLTAGE 1.75L
#define TORQUE_SENSOR_MAX_VOLTAGE 3u
#define TORQUE_NM_PER_VOLT 64.68L
#define MAX_TORQUE (TORQUE_SENSOR_MAX_VOLTAGE-TORQUE_SENSOR_OFFSET_VOLTAGE)*TORQUE_NM_PER_VOLT

// pedal assist factors
#define MINIMUM_PEDAL_RPMS 8u // absolute minimum pedal speed to activate motor
#define MAXIMUM_PEDAL_RPMS 20u // max pedal speed above which use is on pure torque mode
#define CADENCE_FILTER_MAX_VAL 15u*1000u*POLES_PER_REV // equivalent of 4 RPMs (15 seconds per revolution)
#define PAS_FACTOR 0.30L

// Torque sensor variables
static uint8_t torque_assist_factor = 16;
static float torque_NM = 0;
static Ewma torque_filter(0.5);

//Cadence sensor variables
static RingBuf <unsigned long, 10> cadence_tick_buffer;
static Ewma cadence_filter(0.1, CADENCE_FILTER_MAX_VAL);
static unsigned long cadence_last_tick;
static unsigned long cadence_slow_down_tick;
static uint16_t cadence_RPM;

static Ewma throttle_filter(0.9);

// the setup routine runs once when you press reset:
void setup() {                
  // initialize the digital pin as an output.
  pinMode(cadence_sensor, INPUT_PULLUP);
  pinMode(torque_sensor, INPUT);
  pinMode(throttle, OUTPUT);
  pinMode(cadence_output_sim, OUTPUT);

#ifdef DEBUG
  Serial.begin(115200);
  debug_printf("Pedal Assist Program Starting!");
#endif

  // enable interupt pin for cadence sensor
  attachInterrupt(digitalPinToInterrupt(cadence_sensor), get_cadence_ticks_interupt, CHANGE);

  // set PWM output frequency to max. Comment out this line to leave frequency alone.
  setPwmFrequency(throttle, 1);

  // Start-up blink for a few seconds.
//  while(1){
//    analogWrite(throttle, 0);
//    delay(10);
//    analogWrite(throttle, 128);
//    delay(10);
//    analogWrite(throttle, 255);
//    delay(10);
//  }

}

static inline void FRAMEWORK_TASKRUNNER_1ms(void) {
  
  // Debug led stuff for cadence sensor
//  uint8_t rpm_led = map(cadence_RPM, 0, 120, 0, 255);
//  analogWrite(throttle, rpm_led);

  // Debug stuff for torque sensor
//  uint8_t torque_led = map(torque_filter.average, 0, MAX_TORQUE, 0, 255);
//  analogWrite(throttle, torque_led);
  
}

static inline void FRAMEWORK_TASKRUNNER_10ms(void) {
//  TOGGLE(cadence_output_sim);
  calc_torque();
  get_cadence_ticks();
}

static inline void FRAMEWORK_TASKRUNNER_100ms(void) {
  TOGGLE(cadence_output_sim);
  PAS_algorithm();
}

static inline void FRAMEWORK_TASKRUNNER_1000ms(void) {
  
  
}

// the loop routine runs over and over again forever:
static unsigned long timeLast = 0;
void loop() {
  // 1ms framework loop...
  unsigned long timeNow = micros();
  if ((unsigned long)(timeNow - timeLast >= 1000)) {
    timeLast = timeNow;
    FRAMEWORK_TASKRUNNER_run();
  }
}

static uint16_t timer;
static void FRAMEWORK_TASKRUNNER_run(void) {
  timer++;
  if(timer == 1000){
    timer = 0;
  }
  FRAMEWORK_TASKRUNNER_1ms();
  if (timer % 10 == 0) {
    FRAMEWORK_TASKRUNNER_10ms();
  }
  if (timer % 40 == 0) {
      FRAMEWORK_TASKRUNNER_100ms();
  }
  if (timer % 1000 == 0) {
    FRAMEWORK_TASKRUNNER_1000ms();
  }
}

static void PAS_algorithm(){
  // Convert the cadence assist factor
  uint16_t _cadence_RPM = constrain(cadence_RPM, MINIMUM_PEDAL_RPMS, MAXIMUM_PEDAL_RPMS);
  float pas_factor = float(map(_cadence_RPM, MINIMUM_PEDAL_RPMS, MAXIMUM_PEDAL_RPMS, 0, 100))/100.0;

  float torque_offset = pas_factor*MAX_TORQUE*PAS_FACTOR;

  // Convert the torque factor
  uint8_t torque_factor = float(map(torque_NM+torque_offset, 0, MAX_TORQUE, 0, 100))/100.0;

  // Combine and convert all factors.
  uint8_t throttle_factor = constrain(pas_factor*torque_factor*torque_assist_factor*100, 0, 100);
  uint8_t throttle_duty = map(throttle_factor, 0, 100, 50, 210);
  throttle_filter.filter(throttle_duty);
  
  //Write the duty
  analogWrite(throttle, throttle_filter.output);
//  uint16_t t = floor(torque_NM);
//  uint16_t d = (uint16_t)((torque_NM - float(t))*1000.0);
//  debug_printf("pas_factor %d cadence_RPM %d torque_NM %d.%d", int(pas_factor*100), cadence_RPM, t, d);
}

static void calc_torque(){
  float torque_voltage = 0;
  torque_voltage = float(analogRead(torque_sensor))*float(MICRO_REFERENCE_VOLTAGE)/float(MICRO_ADC_RESOLUTION);
  torque_voltage = constrain(torque_voltage, TORQUE_SENSOR_OFFSET_VOLTAGE, TORQUE_SENSOR_MAX_VOLTAGE) - TORQUE_SENSOR_OFFSET_VOLTAGE;
  torque_filter.filter(torque_voltage*TORQUE_NM_PER_VOLT);
  torque_NM = torque_filter.output;
}

static void get_cadence_ticks_interupt(){
  if(digitalRead(cadence_sensor)){
    cadence_tick_buffer.push(millis());
  }
}

static void get_cadence_ticks(){
  unsigned long this_tick = 0;
  uint16_t delta = 0;
  unsigned long now = millis();
  
  // Get all ticks accumulated in buffer and add their times into the filter.
  while(cadence_tick_buffer.pop(this_tick)){
    delta = uint16_t(this_tick - cadence_last_tick);
    // Debouncing...
    if(delta < 5){
      this_tick = 0;
      continue;
    }
    cadence_filter.filter(min(delta, CADENCE_FILTER_MAX_VAL));
    cadence_last_tick = this_tick;
    cadence_slow_down_tick = now;
    debug_printf("tick");
  }

  // Add in a slow-down factor to ramp down the cadence.
  if((now - cadence_slow_down_tick)/4 > 16.6*MAXIMUM_PEDAL_RPMS/POLES_PER_REV){
    cadence_slow_down_tick = now;
    cadence_filter.filter(min(2 * cadence_filter.output, CADENCE_FILTER_MAX_VAL));
  }

  // Convert the cadence to RPM 
  cadence_RPM = uint16_t(60.0*1000.0/float(cadence_filter.output)/POLES_PER_REV);
}


void setPwmFrequency(int pin, int divisor) {
  byte mode;
  switch(divisor) {
    case 1: mode = 0x01; break;
    case 8: mode = 0x02; break;
    case 64: mode = 0x03; break;
    case 256: mode = 0x04; break;
    case 1024: mode = 0x05; break;
    default: return;
  }
  TCCR1B = TCCR1B & (0b11111000 | mode);
}
