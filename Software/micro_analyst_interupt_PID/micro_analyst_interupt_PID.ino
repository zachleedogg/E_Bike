#include <PID_v1.h>
#include <MovingAverage.h>
#include <RingBuf.h>

/*
  Micro Analyst Pedal Assistance converter.
  Takes torque and cadence as inputs and outputs a throttle signal.
 
  This example code is in the public domain.
 */

//Select arduino version
#define TINY85

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
#define throttle 11 // This must be a PWM pin on TIMER1 (pin 9 or 10)
#define cadence_out 10
#endif

#ifdef TINY84
#define cadence_sensor 8 // This pin must be an interupt pin. For ATTINY84a it is INT0 on Pin 8
#define torque_sensor 7 // Any analog pin
#define throttle 5 // This must be a PWM pin on TIMER1 (pin 5 or 6)
#endif

#ifdef TINY85
#define cadence_sensor 2 // This pin must be an interupt pin. For ATTINY85 it is INT0 on Pin 2
#define torque_sensor 3 // Any analog pin
#define throttle 1 // This must be a PWM pin on TIMER1 (pin 4 or 1?)
#define TCCR1B TCCR1 // To simplify code, assign the clock register to be the same as other standard arduinos.
#endif

// micro constants
#define MICRO_REFERENCE_VOLTAGE 5.0L // Arduino reference voltage
#define MICRO_ADC_RESOLUTION 1024u // Normal ADC precision
#define TOGGLE(x) digitalWrite(x, !bool(digitalRead(x)))

// pedal assist sensor constants
#define POLES_PER_REV 36u //36.0 From Grin Datasheet                            
#define TORQUE_SENSOR_RANGE 1.5L //from Grin Datasheet
#define TORQUE_SENSOR_MAX_VOLTAGE torque_sensor_offset_voltage+TORQUE_SENSOR_RANGE
#define TORQUE_NM_PER_VOLT 64.68L //From Grin Datasheet
#define MAX_TORQUE TORQUE_SENSOR_RANGE*TORQUE_NM_PER_VOLT

// pedal assist sensor user adjustable factors.
#define MINIMUM_PEDAL_RPMS 8u // absolute minimum pedal speed to activate motor
#define MAXIMUM_PEDAL_RPMS 28u // max pedal speed above which use is on pure torque mode
#define ASSIST_FACTOR 1.0L // Multipler for assist level.
#define CADENCE_FILTER_MAX_VAL 60u*POLES_PER_REV // equivalent of 1 RPMs (60 seconds per revolution)

// PID variables
static double Setpoint, Input, Output;
static double Kp = .19;
static double Ki = .05;
static double Kd = 0;
static PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, P_ON_M, REVERSE);

// Torque sensor variables
static float torque_NM = 0;
static float torque_sensor_offset_voltage = 0;
//static uint16_t torque_offset_ADC = 0;
static MovingAverage <uint16_t> torque_filer_adc(20);
static MovingAverage <float> torque_filter_NM(20);

//Cadence sensor variables
static RingBuf <unsigned long, 10> cadence_tick_buffer;
static MovingAverage <uint16_t> cadence_filter(2, CADENCE_FILTER_MAX_VAL);
static unsigned long cadence_last_tick;
static unsigned long cadence_debounce;
static unsigned long cadence_slow_down_tick;
static uint16_t cadence_RPM = 0;
static bool cadence_hysterysis = true;

#ifdef DEBUG
static unsigned long debug_cadence_pwm_last_time;
static unsigned long debug_cadence_rpm = 4;
#endif

// the setup routine runs once when you press reset:
void setup() {                
  // initialize the digital pin as an output.
  pinMode(cadence_sensor, INPUT_PULLUP);
  pinMode(torque_sensor, INPUT);
  pinMode(throttle, OUTPUT);

  delay(500);

  torque_sensor_offset_voltage = float(analogRead(torque_sensor))*MICRO_REFERENCE_VOLTAGE/float(MICRO_ADC_RESOLUTION) + 2;

#ifdef DEBUG
  pinMode(cadence_out, OUTPUT);
  Serial.begin(115200);
  debug_printf("Pedal Assist Program Starting!");
#endif

  // enable interupt pin for cadence sensor
  attachInterrupt(digitalPinToInterrupt(cadence_sensor), get_cadence_ticks_interupt, CHANGE);

  // set PWM output frequency to max. Comment out this line to leave frequency alone.
  setPwmFrequency(throttle, 1);

  //turn the PID on
  Setpoint = 0;
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(25);
  myPID.SetOutputLimits(65,205);
  
}

static inline void FRAMEWORK_TASKRUNNER_1ms(void) {
  // read torque ADC.
  get_torque_value();
  get_cadence_ticks();

  PAS_algorithm();
  //Write the duty from the PI loop.
  analogWrite(throttle, Output);
  //Debugging stuff
#ifdef DEBUG
  uart_input();
  debug_cadence_tick();
#endif
}

static inline void FRAMEWORK_TASKRUNNER_10ms(void) {

}

static inline void FRAMEWORK_TASKRUNNER_100ms(void) {
  calc_torque();
  debug_printf("cadence: %d torque: %d Setpoint: %d Input: %d Output: %d", int(cadence_RPM), int(torque_NM), uint16_t(Setpoint), uint16_t(Input), uint16_t(Output));
}

static inline void FRAMEWORK_TASKRUNNER_1000ms(void) {
//  static bool val = false;
//  if (val){
//    analogWrite(throttle, 150);
//    val = false;
//  } else {
//    analogWrite(throttle, 70);
//    val = true;
//  }
  

}

// the loop routine runs over and over again forever:
static unsigned long timeLast = 0;
void loop() {
  //compute PID outside of scheduler to ensure its own internal timing is met.
  myPID.Compute();
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
  if (timer % 100 == 0) {
      FRAMEWORK_TASKRUNNER_100ms();
  }
  if (timer % 1000 == 0) {
    FRAMEWORK_TASKRUNNER_1000ms();
  }
}

//This one sortof works... was on my test ride to mom and dads
//static void PAS_algorithm(){
//  // Convert the cadence assist factor
//  uint16_t _cadence_RPM = constrain(cadence_RPM, MINIMUM_PEDAL_RPMS, MAXIMUM_PEDAL_RPMS);
//  double pas_factor = double(map(_cadence_RPM, MINIMUM_PEDAL_RPMS, MAXIMUM_PEDAL_RPMS, 0, 1000));
//  Setpoint = 1000 - pas_factor;
//  // Convert the torque factor
//  Input = double(map(torque_NM*ASSIST_FACTOR, 0, MAX_TORQUE, 0, 1000));
//}

//Best one so far. rode this to get my hair cut. battery was running low
//static void PAS_algorithm(){
//  // Convert the cadence assist factor as a percentage from 0-1.
//  uint16_t _cadence_RPM = constrain(cadence_RPM, MINIMUM_PEDAL_RPMS, MAXIMUM_PEDAL_RPMS);
//  double pas_factor = mapf(_cadence_RPM, MINIMUM_PEDAL_RPMS, MAXIMUM_PEDAL_RPMS, 0, 1);
//  Setpoint = 100;
//  // Convert the torque factor
//  Input = mapf(torque_NM*pas_factor, 0, MAX_TORQUE, 70, 1000);
//}

//// new one, really good. need to fix constant speeding up.
//static void PAS_algorithm(){
//  // Convert the cadence assist factor as a percentage from 0-1.
//  uint16_t _cadence_RPM = constrain(cadence_RPM, MINIMUM_PEDAL_RPMS, MAXIMUM_PEDAL_RPMS);
//  double pas_factor = mapf(_cadence_RPM, MINIMUM_PEDAL_RPMS, MAXIMUM_PEDAL_RPMS, 0.0, 1.0);
//  pas_factor = pow(pas_factor, 2);
//  Setpoint = 100;
//  // Convert the torque factor
//  if(torque_NM > 0.3){
//    torque_NM = constrain(torque_NM, 0, MAX_TORQUE/8);
//    double torque_target = max(0, torque_NM-4);
//    Input = mapf(torque_NM*pas_factor*ASSIST_FACTOR - torque_target, 0, MAX_TORQUE, 100.0, 1000.0);
//  }
//  if(_cadence_RPM <= MINIMUM_PEDAL_RPMS){
//    Input = 0;
//  }
//}

//// HUGE BREAKTHRU!!! Never go back.
static void PAS_algorithm(){
  // Convert the cadence assist factor as a percentage from 0-1.
  uint16_t _cadence_RPM = constrain(cadence_RPM, MINIMUM_PEDAL_RPMS, MAXIMUM_PEDAL_RPMS);
  double pas_factor = mapf(_cadence_RPM, MINIMUM_PEDAL_RPMS, MAXIMUM_PEDAL_RPMS, 0.0, 1.0);
  pas_factor = pow(pas_factor, 3);
  Setpoint = 500;
  // Convert the torque factor
  torque_NM = constrain(torque_NM, 0, MAX_TORQUE/2);

  if(torque_NM > 6){
    Input = mapf(torque_NM*pas_factor, 0, MAX_TORQUE, 500, 1000.0);
  }
  else {
    Input = mapf(torque_NM*pas_factor, 0, MAX_TORQUE, 500.0, 0);
  }
  if(_cadence_RPM <= MINIMUM_PEDAL_RPMS){
    Input = 0;
    myPID.SetTunings(Kp, 0, Kd);
  } else if(_cadence_RPM >= MAXIMUM_PEDAL_RPMS){
    myPID.SetTunings(Kp, Ki, Kd);
  }
}

//// Untested, going for constant assist.
//static void PAS_algorithm(){
//  // Convert the cadence assist factor as a percentage from 0-1.
//  uint16_t _cadence_RPM = constrain(cadence_RPM, MINIMUM_PEDAL_RPMS, MAXIMUM_PEDAL_RPMS);
//  double pas_factor = mapf(double(_cadence_RPM), MINIMUM_PEDAL_RPMS, MAXIMUM_PEDAL_RPMS, 0.0, 1.0);
//  pas_factor = pow(pas_factor, 2);
//  Setpoint = 500;
//  // Convert the torque factor
//  torque_NM = constrain(torque_NM, 0, MAX_TORQUE/4);
//  double torque_assist_val = 8;
//
//  if(torque_NM > torque_assist_val){
//    Input = mapf(torque_NM*pas_factor, torque_assist_val, MAX_TORQUE, Setpoint, 1000.0);
//  }
//  else {
//    Input = mapf(torque_NM*pas_factor, 0.0, torque_assist_val, Setpoint-50, Setpoint);
//  }
//  if(_cadence_RPM <= MINIMUM_PEDAL_RPMS){
//    Input = Setpoint-100;
//    torque_filter_NM.clear();
//  }
//}

static void get_torque_value(){
  torque_filer_adc.push(analogRead(torque_sensor));
}

static void calc_torque(){
  float torque_voltage = 0;
  torque_voltage = float(torque_filer_adc.get())*MICRO_REFERENCE_VOLTAGE/float(MICRO_ADC_RESOLUTION) - torque_sensor_offset_voltage;;
  
  torque_voltage = constrain(torque_voltage, 0, TORQUE_SENSOR_MAX_VOLTAGE);
  torque_filter_NM.push(torque_voltage*TORQUE_NM_PER_VOLT);
  torque_NM = torque_filter_NM.get();
}

static void get_cadence_ticks_interupt(){
  if(digitalRead(cadence_sensor)){
    if((millis() - cadence_debounce) > 2){
      cadence_tick_buffer.push(millis());
    }
    cadence_debounce = millis();
  }
}

static void get_cadence_ticks(){
  unsigned long now = millis();
  unsigned long this_tick = 0;
  uint16_t delta = 0;

  // Get all ticks accumulated in buffer and add their times into the filter.
  while(cadence_tick_buffer.lockedPop(this_tick)){
    delta = (uint16_t)(this_tick - cadence_last_tick);
    cadence_filter.push(min(delta, CADENCE_FILTER_MAX_VAL));
    cadence_last_tick = this_tick;
    cadence_slow_down_tick = now;
    //debug_printf("tick %d", delta);
  }

//  // Add in a slow-down factor to ramp down the cadence. With hysterysis so that it doesn't affect start-up, only slow-down.
//  if(cadence_RPM > MINIMUM_PEDAL_RPMS){
//    cadence_hysterysis = true;
//  } else if(cadence_RPM == 1){
//    cadence_hysterysis = false;
//  }
  if(((now - cadence_slow_down_tick) > cadence_filter.get()) && cadence_hysterysis){
    cadence_filter.push(min(now - cadence_last_tick, CADENCE_FILTER_MAX_VAL));
    cadence_slow_down_tick = now;
  }

  // Convert the cadence to RPM 
  cadence_RPM = uint16_t(60.0L*1000.0L/POLES_PER_REV/double(cadence_filter.get()));
}


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
  uint16_t period = 1000/((debug_cadence_rpm*POLES_PER_REV)/60.0)/2;
  if(millis() - debug_cadence_pwm_last_time >= period){
    TOGGLE(cadence_out);
    debug_cadence_pwm_last_time = millis();
  }
}
#endif
