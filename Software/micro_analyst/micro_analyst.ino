#include <EwmaT.h>
#include <Ewma.h>

/*
  Blink
  Turns on an LED on for one second, then off for one second, repeatedly.
 
  This example code is in the public domain.
 */
 
// Pin names:
#define cadence_sensor 8 // This pin must be an interupt pin. For ATTINY84a it is INT0
#define torque_sensor 1
#define throttle 7
#define cadence_output_sim 0

//constants
#define MAX_FILTER_LENGTH 30
#define TORQUE_FILTER_COUNT 25
#define CADENCE_FILTER_COUNT 10

//physical properties
#define POLES_PER_REV 5.0 //36.0
#define REFERENCE_VOLTAGE 1.5
#define TORQUE_NM_PER_VOLT 64.68
#define MAX_TORQUE REFERENCE_VOLTAGE*TORQUE_NM_PER_VOLT

//pedal assist factors
#define MINIMUM_PEDAL_RPMS 15 // absolute minimum pedal speed to activate motor
#define MAXIMUM_PEDAL_RPMS 30 // max pedal speed above which use is on pure torque mode

static uint8_t torque_assist_factor = 2;

float torque_NM = 0;
EwmaT <uint16_t> torque_filter(0.1, 100);

typedef struct _tick_counter {
  bool current_val;
  uint8_t index;
  uint8_t interupt_index;
  unsigned long last_time;
  uint16_t history[CADENCE_FILTER_COUNT];
  uint16_t sum;
  uint16_t rpm;
} tick_counter;
static volatile tick_counter cadence_ticks;

// the setup routine runs once when you press reset:
void setup() {                
  // initialize the digital pin as an output.
  pinMode(cadence_sensor, INPUT_PULLUP);
  pinMode(torque_sensor, INPUT);
  pinMode(throttle, OUTPUT);
  pinMode(cadence_output_sim, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(cadence_sensor), get_cadence_ticks, CHANGE);

  // Start-up blink for a few seconds.
  for(int i=0; i<2; i++){
    digitalWrite(throttle, HIGH);
    delay(500);
    digitalWrite(throttle, LOW);
    delay(500);
  }

}

static inline void FRAMEWORK_TASKRUNNER_1ms(void) {
//  get_cadence_ticks();

  // Debug led stuff for cadence sensor
  uint8_t rpm_led = map(cadence_ticks.rpm, 0, 120, 0, 255);
  analogWrite(throttle, rpm_led);

  // Debug stuff for torque sensor
//  uint8_t torque_led = map(torque_filter.average, 0, MAX_TORQUE, 0, 255);
//  analogWrite(throttle, torque_led);
  
}

static inline void FRAMEWORK_TASKRUNNER_10ms(void) {
  calc_torque();
  //PAS_algorithm();
}

static inline void FRAMEWORK_TASKRUNNER_100ms(void) {
  bool val = digitalRead(cadence_output_sim);
  digitalWrite(cadence_output_sim, !val);
}

static inline void FRAMEWORK_TASKRUNNER_1000ms(void) {

}

// the loop routine runs over and over again forever:
static unsigned long timeLast = 0;
void loop() {
  // 1ms framework loop...
  // some kind of issue with using millis, seems to be off for some reason.... using micros instead.
  unsigned long timeNow = micros();
  if (timeNow - timeLast >= 1000) {
    timeLast = timeNow;
    FRAMEWORK_TASKRUNNER_run();
  }
  // Overflow protection.
  if (timeNow < timeLast){
    timeLast = 0;
  }
}

static uint16_t timer;
static void FRAMEWORK_TASKRUNNER_run(void) {
  /*1ms tasks here*/
  timer++;
  if(timer == 1000){
    timer = 0;
  }
  FRAMEWORK_TASKRUNNER_1ms();

  /*10ms tasks here*/
  if (timer % 10 == 0) {
    FRAMEWORK_TASKRUNNER_10ms();
  }

    /*100ms tasks here*/
  if (timer % 100 == 0) {
      FRAMEWORK_TASKRUNNER_100ms();
  }

  /*1000ms tasks here*/
  if (timer % 1000 == 0) {
    FRAMEWORK_TASKRUNNER_1000ms();
  }
}

static void PAS_algorithm(){
  float pas_factor = float(map(cadence_ticks.rpm, MINIMUM_PEDAL_RPMS, MAXIMUM_PEDAL_RPMS, 0, 100))/100.0;
  uint8_t torque_factor = float(map(torque_NM, 0, MAX_TORQUE, 0, 100))/100.0;
  uint8_t throttle_duty = map(pas_factor*torque_factor*torque_assist_factor*100, 0, 100, 0, 255);
  analogWrite(throttle, throttle_duty);
}

static void calc_torque(){
  uint16_t torque = 0;
  torque = analogRead(torque_sensor) - REFERENCE_VOLTAGE;
  torque_NM = torque_filter.filter(torque)*TORQUE_NM_PER_VOLT;
}

static void get_cadence_ticks_interupt(){
  bool val = digitalRead(cadence_sensor);
  unsigned long now = millis();
  
}

static void get_cadence_ticks(){
  bool val = digitalRead(cadence_sensor);
  unsigned long now = millis();
  if((val != cadence_ticks.current_val) & (val == true)){
    unsigned long delta = now - cadence_ticks.last_time;
    cadence_ticks.last_time = now;
    uint16_t rpm = uint16_t(60.0*1000.0/float(delta)/POLES_PER_REV);
    cadence_ticks.sum -= cadence_ticks.history[cadence_ticks.index];
    cadence_ticks.history[cadence_ticks.index++] = rpm;
    cadence_ticks.sum += rpm;
    if(cadence_ticks.index >= CADENCE_FILTER_COUNT){
      cadence_ticks.index = 0;
    }
    cadence_ticks.rpm = cadence_ticks.sum/CADENCE_FILTER_COUNT;
  }
  if((now - cadence_ticks.last_time) > 1000){
    for(int i=0; i < CADENCE_FILTER_COUNT; i++){
      cadence_ticks.history[i] = 0;
    }
    cadence_ticks.sum = 0;
    cadence_ticks.last_time = 0;
    cadence_ticks.rpm = 0;
  }
  cadence_ticks.current_val = val;
}
