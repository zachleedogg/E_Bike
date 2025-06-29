#include <PID_v1.h>
#include <MovingAverage.h>
#include <RingBuf.h>

/*
  Micro Analyst Pedal Assistance converter.
  Takes torque, cadence, and battery current as inputs and outputs a throttle signal.
 
  This example code is in the public domain.
 */

//Select arduino version
#define TINY85

//Select Program mode
#define PRODUCTION
//#define CURRENT_SENSE_TEST
//#define THROTTLE_TEST
//#define CADENCE_TEST
//#define TORQUE_TEST
//#define PAS_TEST
//# define PID_TEST

// Debug messages
#ifdef UNO
#define DEBUG
#define debug_printf(...) {char buff[200]={}; sprintf(buff, __VA_ARGS__); Serial.println(buff);}
#else
#define debug_printf(...)
#endif

// Pin names:
#ifdef UNO
#define cadence_sensor 2 // This pin must be an interupt pin. For UNO it is INT0 on Pin 2
#define torque_sensor A1 // Any analog pin
#define current_sensor A0 //Any analog pin
#define throttle 11 // This must be a PWM pin on TIMER1 (pin 9 or 10)
#define cadence_out 10
#endif

//For ATTINY best to use the ATTinyCore Universal by SpenceKonde
#ifdef TINY84
#define cadence_sensor 8 // This pin must be an interupt pin. For ATTINY84a it is INT0 on Pin 8
#define torque_sensor 7 // Any analog pin
#define current_sensor 6 //Any analog pin
#define throttle 5 // This must be a PWM pin on TIMER1 (pin 5 or 6)
#endif

//For ATTINY best to use the ATTinyCore Universal by SpenceKonde
#ifdef TINY85
#define cadence_sensor 2 // This pin must be an interupt pin. For ATTINY85 it is INT0 on Pin 2
#define torque_sensor A3 // Any analog pin
#define throttle 1 // This must be a PWM pin on TIMER1 (pin 4 or 1?)
#define current_sensor A2 //Any analog pin
#define TCCR1B TCCR1 // To simplify code, assign the clock register to be the same as other standard arduinos.
#endif

// micro constants
#define MICRO_REFERENCE_VOLTAGE 5.0L // Arduino reference voltage
#define MICRO_ADC_RESOLUTION 1023.0L // Normal ADC precision
#define TOGGLE(x) digitalWrite(x, !bool(digitalRead(x)))

// pedal assist sensor constants
#define POLES_PER_REV 36u //36.0 From Grin Datasheet                            
#define TORQUE_SENSOR_RANGE 1.5L //from Grin Datasheet
#define TORQUE_SENSOR_MAX_VOLTAGE torque_sensor_offset+TORQUE_SENSOR_RANGE
#define TORQUE_NM_PER_VOLT 64.68L //From Grin Datasheet
#define MAX_TORQUE TORQUE_SENSOR_RANGE*TORQUE_NM_PER_VOLT //97NM


// pedal assist sensor user adjustable factors.
#define MINIMUM_PEDAL_RPMS 8u // absolute minimum pedal speed to activate motor
#define MAXIMUM_PEDAL_RPMS 15u // max pedal speed above which use is on pure torque mode
#define TORQUE_SENSOR_CUTTOFF_FREQ 4L // 4Hz filtering
#define ASSIST_FACTOR_MULTIPLIER 1.0L // Multipler for assist level.
#define CADENCE_FILTER_MAX_DELTA_MS 60.0*1000.0/POLES_PER_REV/5.0 // equivalent of 5 RPMs (12 seconds per revolution)
#define CADENCE_DEBOUNCE_TIME_MS 5 // Debounce for cadence signal.

//Current Sensor values
#define CURRENT_SENSOR_AMPS_PER_VOLT 10.0L
#define CURRENT_SENSE_MAX_CURRENT MICRO_REFERENCE_VOLTAGE*CURRENT_SENSOR_AMPS_PER_VOLT

//Throttle values
#define THROTTLE_MIN_VOLTAGE 80u
#define THROTTLE_MAX_VOLTAGE 255u

typedef struct {
    double alpha;
    double y;
} lowPassFilter_S;

// PID variables
#define PID_LOOP_TIME_MS 1
static double Setpoint, Input, Output;
static double Kp = 2.50;
static double Ki = 18.00;
static double Kd = 0.01;
static PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, P_ON_E, DIRECT);

// Torque sensor variables
static double torque_NM = 0;
static uint16_t torque_sensor_offset = 0;
static MovingAverage <uint16_t> torque_filer_adc(20); //Value taken at 1kHz.
static lowPassFilter_S torque_filter_lpf_NM = {
  .alpha = .001/(1.0/TORQUE_SENSOR_CUTTOFF_FREQ), // 1ms time-delta must be used in 1ms loop.
  .y = 0,
};
//static MovingAverage <double> torque_filter_NM(20); // Value taken at 10Hz.

//Cadence sensor variables
static RingBuf <unsigned long, 20> cadence_tick_buffer;
static uint16_t cadence_RPM = 0;

// Current Sensor variables
static double current_sense_A = 0;
static uint16_t current_sense_offset = 0;
static MovingAverage <uint16_t> current_sense_filter_adc(20); //Value taken at 1kHz.
static MovingAverage <double> current_sense_filter_A(5); //Value taken at 100Hz.

#ifdef DEBUG
static unsigned long debug_cadence_rpm = 4;
#endif


/**********************************SETUP************************************************************************/
void setup() {             
  // initialize the pins.
  //pinMode(cadence_sensor, INPUT_PULLUP);
  pinMode(torque_sensor, INPUT);
  pinMode(current_sensor, INPUT);
  pinMode(throttle, OUTPUT);
  pinMode(0, OUTPUT);


  // Delay for torque sensor power-up and zeroing
  delay(20);
  // take initial reading and calculate the offset voltages on the torque sensor and current sensor.
  torque_sensor_offset = 307;//analogRead(torque_sensor);
  
  //must have this delay or motor controller will fault on startup.
  delay(1000);
  current_sense_offset = analogRead(current_sensor);

  // enable interupt pin for cadence sensor
  attachInterrupt(digitalPinToInterrupt(cadence_sensor), get_cadence_ticks_interupt, CHANGE);

  // set PWM output frequency to max (60kHz pwm for better DAC conversion). Comment out this line to leave frequency alone.
  setPwmFrequency(throttle, 1);


  //turn the PID on
  Setpoint = 0;
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(PID_LOOP_TIME_MS);
  myPID.SetOutputLimits(THROTTLE_MIN_VOLTAGE, THROTTLE_MAX_VOLTAGE);

  #ifdef DEBUG
  pinMode(cadence_out, OUTPUT);
  Serial.begin(115200);
  debug_printf("Pedal Assist Program Starting!");
#endif
  
}


/**********************************1ms Loop************************************************************************/
static inline void FRAMEWORK_TASKRUNNER_1ms(void) {

  // read torque raw ADC with moving filter.
  get_torque_value();

  // read current sence raw ADC with moving filter.
  get_current_sense_value();

  // calculate instantaneous cadence.
  get_cadence_ticks();

  // caluculate torque with low pass filter.
  calc_torque();

  //caluculate instantaneous DC battery current.
  calc_current_sense();


  // run the algo to combine cadence, torque sensor, and battery current.
  PAS_algorithm();
  
  //Write the duty from the PI loop.
  analogWrite(throttle, Output);


  
  //Debugging stuff
#ifdef DEBUG
  uart_input();
  debug_cadence_tick();
#endif
}

/**********************************10ms Loop************************************************************************/
static inline void FRAMEWORK_TASKRUNNER_10ms(void) {
//  calc_current_sense();
  __asm__("nop\n\t");

}

/**********************************100ms Loop************************************************************************/
static inline void FRAMEWORK_TASKRUNNER_100ms(void) {
  //calculated the instantaneous torque.
  //calc_torque();
  //debug_printf("cadence: %d torque: %d Setpoint: %d Input: %d Output: %d", int(cadence_RPM), int(torque_NM), uint16_t(Setpoint), uint16_t(Input), uint16_t(Output));
  __asm__("nop\n\t");
}

/**********************************1000ms Loop************************************************************************/
static inline void FRAMEWORK_TASKRUNNER_1000ms(void) {
  __asm__("nop\n\t");
}

/**********************************Task Runner************************************************************************/
static void FRAMEWORK_TASKRUNNER_run(void) {
  static uint16_t timer = 0;
  timer++;
  
  FRAMEWORK_TASKRUNNER_1ms();
  if (timer % 10 == 0) {
    FRAMEWORK_TASKRUNNER_10ms();
  }
  if (timer % 100 == 0) {
      FRAMEWORK_TASKRUNNER_100ms();
  }
  if (timer % 1000 == 0) {
    FRAMEWORK_TASKRUNNER_1000ms();
  }
  if(timer == 1000){
    timer = 0;
  }
}

/**********************************Main Loop************************************************************************/
#ifdef PRODUCTION
void loop() {
  static unsigned long timeLast = 0;

  
  //compute PID outside of scheduler to ensure its own internal timing is met.
  myPID.Compute();

  
  // 1ms framework loop...
  unsigned long timeNow = micros();
  if (timeNow - timeLast >= 1000) {
    timeLast = timeNow;
    FRAMEWORK_TASKRUNNER_run();
  }
}
#endif

#ifdef CURRENT_SENSE_TEST
void loop() {
  analogWrite(throttle, 150);
  uint16_t current_adc_val = analogRead(current_sensor);
  current_adc_val = map(current_adc_val, 0, 1023, 0, 255);
  analogWrite(0, current_adc_val);
  delay(10);
}
#endif

#ifdef THROTTLE_TEST
void loop() {
  analogWrite(throttle, 0);
  delay(5000);
  analogWrite(throttle, 75);
  delay(3000);
  analogWrite(throttle, 128);
  delay(3000);
  analogWrite(throttle, THROTTLE_MAX_VOLTAGE);
  delay(10000);
}
#endif

#ifdef CADENCE_TEST
void loop() {
    get_cadence_ticks();
    delay(1);
    //uint16_t cad = map(cadence_RPM, 0, 120, THROTTLE_MIN_VOLTAGE, THROTTLE_MAX_VOLTAGE);
    constrain(cadence_RPM, 0, 30);
    uint16_t cad = map(cadence_RPM, 0, 30, 0, 255);
    if (cad > 1){
      analogWrite(0, 255);
    } else {
      analogWrite(0, cad);
    }
}
#endif

#ifdef TORQUE_TEST
void loop() {


  // Convert the torque factor tp a percentage (from 0-1).
  get_torque_value();
  calc_torque();
  analogWrite(0, (int)(mapf(torque_NM, 0, MAX_TORQUE, 0, 255)));
  delay(1);

}
#endif

#ifdef PAS_TEST
void loop() {

  get_cadence_ticks();
  get_torque_value();
  calc_torque();
  
  // Convert the cadence assist Factor to a percentage (from 0-1).
  uint16_t _cadence_RPM = constrain(cadence_RPM, MINIMUM_PEDAL_RPMS, MAXIMUM_PEDAL_RPMS);
  double cadence_factor = mapf(_cadence_RPM, MINIMUM_PEDAL_RPMS, MAXIMUM_PEDAL_RPMS, 0.0, 1.0);
  //cadence_factor = pow(cadence_factor, 2);

  // Convert the torque factor tp a percentage (from 0-1).
  uint16_t _torque_NM = constrain(torque_NM, 0, MAX_TORQUE);
  double torque_factor = mapf(_torque_NM, 0, MAX_TORQUE, 0.0, 1.0);

  analogWrite(0, (int)(mapf(torque_factor*cadence_factor, 0,1,0,255)));

  delay(1);
}
#endif

#ifdef PID_TEST
void loop() {

  Setpoint = 6;
  while(1){
    //compute PID outside of scheduler to ensure its own internal timing is met.
    myPID.Compute();
  
    // Input is always the instantaneous current sense value.
    // read current sence raw ADC with moving filter.
  
//    uint16_t current_adc_val = analogRead(current_sensor);
//    Input = mapf(current_adc_val, 0, 1023, 0.0, 50.0);
//    analogWrite(0, current_adc_val);
  //
    get_current_sense_value();
    analogWrite(0, map(current_sense_filter_adc.get(),0,1023,0,255));
    current_sense_A = current_sense_filter_adc.get()*5.0*10/1023.0;

    // Input is always the instantaneous current sense value.
    Input = current_sense_A;
    
    //Write the duty from the PI loop.
    analogWrite(throttle, Output);
    
    delay(1);
  }
}
#endif


/**********************************Assist Algorithm************************************************************************/
static void PAS_algorithm(){
  // Convert the cadence assist Factor to a percentage (from 0-1).
  uint16_t _cadence_RPM = constrain(cadence_RPM, MINIMUM_PEDAL_RPMS, MAXIMUM_PEDAL_RPMS);
  double cadence_factor = mapf(_cadence_RPM, MINIMUM_PEDAL_RPMS, MAXIMUM_PEDAL_RPMS, 0.0, 1.0);

  // Convert the torque factor tp a percentage (from 0-1).
  double _torque_NM = constrain(torque_NM, 0, MAX_TORQUE);
  double torque_factor = mapf(_torque_NM, 0, MAX_TORQUE, 0.0, 1.0);

  double assist_factor = cadence_factor*torque_factor*ASSIST_FACTOR_MULTIPLIER;
  analogWrite(0, (int)(mapf(assist_factor, 0,1,0,255)));


  // Create the setpoint as a proportion of torque to current.
  Setpoint = CURRENT_SENSE_MAX_CURRENT*assist_factor;

  // Input is always the instantaneous current sense value.
  Input = current_sense_A;
}


/**********************************Torque************************************************************************/
static void get_torque_value(){
  uint16_t value = analogRead(torque_sensor);
  //Ensure the offset is not greater than the actual value (perhaps due to some noise...)
  if (torque_sensor_offset < value){
    value -= torque_sensor_offset;
  } else {
    value = 0;
  }
  torque_filer_adc.push(value);
}

static void calc_torque(){
  double torque_voltage = 0;
  torque_voltage = double(torque_filer_adc.get())*MICRO_REFERENCE_VOLTAGE/MICRO_ADC_RESOLUTION;
  torque_voltage = constrain(torque_voltage, 0, TORQUE_SENSOR_RANGE);
  torque_NM = lpf(&torque_filter_lpf_NM, torque_voltage*TORQUE_NM_PER_VOLT);
}

/**********************************Cadence************************************************************************/
static void get_cadence_ticks_interupt(){
  static unsigned long cadence_debounce;
  
  if(digitalRead(cadence_sensor)){
    unsigned long millis_now = millis();
    if((millis_now - cadence_debounce) > CADENCE_DEBOUNCE_TIME_MS){
      cadence_tick_buffer.push(millis_now);
    }
    cadence_debounce = millis_now;
  }
}

static void get_cadence_ticks(){
  static unsigned long cadence_last_tick = 0;
  static uint16_t delta = 1000;
  
  unsigned long now = millis();
  unsigned long this_tick = 0;


  if (cadence_tick_buffer.isEmpty() != true){
    // Get all ticks accumulated in buffer and add their times into the filter.
    while(cadence_tick_buffer.lockedPop(this_tick)){
      delta = (uint16_t)(this_tick - cadence_last_tick);
      cadence_last_tick = this_tick;
      //debug_printf("tick %d", delta);
    }
  }
  else { // No ticks have been detected, so slow down the RPMs each time.
    uint16_t temp_rpm = (uint16_t)(60.0L*1000.0L/POLES_PER_REV/(double)(now - cadence_last_tick));
    if (temp_rpm < cadence_RPM){
      delta = now - cadence_last_tick;
    }
  }
  // Convert the cadence time to RPM 
  cadence_RPM = (uint16_t)(60.0L*1000.0L/POLES_PER_REV/double(delta));

}


/**********************************Current Sense************************************************************************/
static void get_current_sense_value(){
  uint16_t value = analogRead(current_sensor);
  //Ensure the offset is not greater than the actual value (perhaps due to some noise...)
  if (current_sense_offset < value){
    value -= current_sense_offset;
  } else {
    value = 0;
  }
  current_sense_filter_adc.push(value);
}

static void calc_current_sense(){
  double current_sense_voltage = 0;
  current_sense_voltage = (double(current_sense_filter_adc.get())*MICRO_REFERENCE_VOLTAGE/MICRO_ADC_RESOLUTION);
  //current_sense_voltage = constrain(current_sense_voltage, 0, MICRO_REFERENCE_VOLTAGE);
  current_sense_filter_A.push(current_sense_voltage*CURRENT_SENSOR_AMPS_PER_VOLT);
  current_sense_A = current_sense_filter_A.get();
}

/**********************************Misc Helpers************************************************************************/
void setPwmFrequency(int pin, int divisor) {
  if(pin){ //suppress the unused parameter warning
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
}

double mapf(double x, double in_min, double in_max, double out_min, double out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

double lpf(lowPassFilter_S *this_lpf, double x){
  //
// SHORTEST USEFUL ALGORITHM ****EVER****
//

  this_lpf->y += this_lpf->alpha * (x-this_lpf->y);
  return this_lpf->y;
  
  //
  //    # # #   ##    ##   #####  #
  //    # # #  #  #  #  #    #    #
  //    # # #  #  #  #  #    #    
  //     # #    ##    ##     #    #
  //
}

#ifdef DEBUG
static void uart_input(){
  static char buffer[20];
  int i = 0;
  while(Serial.available()){
    buffer[i]=Serial.read();
    if(buffer[i] == '\n'){
      command_input(String(buffer));
      i = 0;
    } else {
      i++;
    }
  }
}

static void command_input(String string){
  debug_printf("%c", string[0]);
  debug_printf("%s", &string[1]);
  switch(string[0]){
    case 'p':
      Kp = atof(&string[1]);
      break;
    case 'i':
      Ki = atof(&string[1]);
      break;
    case 'd':
      Kd = atof(&string[1]);
      break;
    case 'c':
      debug_cadence_rpm = atof(&string[1]);
      break;
    case 't':
      Output = atof(&string[1]);
      break;
    default:
      break;
  }
  myPID.SetTunings(Kp,Ki,Kd); 
}

static void debug_cadence_tick(){
  static unsigned long debug_cadence_pwm_last_time;
  uint16_t period = 1000/((debug_cadence_rpm*POLES_PER_REV)/60.0)/2;
  if(millis() - debug_cadence_pwm_last_time >= period){
    TOGGLE(cadence_out);
    debug_cadence_pwm_last_time = millis();
  }
}
#endif
