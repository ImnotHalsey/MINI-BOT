// Importing Libraries
#include "sbus.h"
#include <ezButton.h>
#include <Robojax_WCS.h>
#include "Filter.h"

// Declarations
#define SIYI 1    // Flight Controller
#define DEBUG 1   // Serial Printer

// depth control settings
#define SET_DEPTH 40
#define DC_AMPS 30     // Depth control amps
#define DC_TIME 500    // Depth control time
#define BUMPER_SENSOR_PIN PA8  // Bumper sensor

// Serial Print Turn ON/OFF
#if DEBUG == 1
  #define debug(x) Serial.print(x)
  #define debugln(x) Serial.println(x)
#else
  #define debug(x)
  #define debugln(x)
#endif

// Transmitter code
#if SIYI == 0
  // Transmitter channels for Flysky
  #define TH_CH 3
  #define ST_CH 1
  #define ACT_CH 7
  #define ENA_CH 5
  #define SPD_CH 6
  #define PTO_CH 10
  #define LIGHT_CH 8
#else
  // Transmitter channels for SIYI
  #define TH_CH 3
  #define ST_CH 1
  #define ACT_CH 6
  #define ENA_CH 8
  #define SPD_CH 5
  #define PTO_CH 10
  #define LIGHT_CH 9
#endif

// Pwm pins
#define PWM_PIN PB4     // Pin 7
#define PWM1_PIN PB3    // Pin 6
#define LIGHT_PIN 8
#define PWM_PTO_PIN 4
#define ACT_RPWM_PIN PA15  // Pin 9
#define ACT_LPWM_PIN PA11  // Pin 10

// Variable Declarations
int max_spd = 60;
bool lane_follow = false;
int throttle = 0, steer = 0, enable_switch = 0, speed = 0, actuator = 0;
int data = -100;
long int last_i2c_data;
int steer_pwm = 0, throttle_pwm = 0;
long int lasttime;
int sbus_fs_count = 0;
double prev = millis();
bool bumper_alarm = false;

// Depth control settings
const int MINI_RANGE = 770;    // Minimum resistance value from the potentiometer
const int MAX_RANGE = 950;     // Maximum resistance value from the potentiometer
const int SOFT_MINI_LIMIT = 5; // Minimum resistance value from the potentiometer
const int SOFT_MAX_LIMIT = 60; // Maximum resistance value from the potentiometer
const int ANG_TOL = 3;


// Define the PWM frequency and resolution
#define PWM_FREQ 490     // PWM frequency in Hz
#define PWM_RESOLUTION 8 // PWM resolution in bits

// depth Control
#define CURRENT_SENSOR_PIN_1 PA0
#define act_potPin PA1 // Analog pin for potentiometer feedback
#define MODEL 11					// see list above
#define SENSOR_VCC_PIN 8			// pin for powring up the senso
#define ZERO_CURRENT_LED_PIN 2		// zero current LED pin
#define ZERO_CURRENT_WAIT_TIME 5000 // wait for 5 seconds to allow zero current measurement
uint8_t CORRECTION_VLALUE=164;		// mA
uint16_t MEASUREMENT_ITERATION=10;
float VOLTAGE_REFERENCE=5050.0; // 5000mv is for 5V
uint8_t BIT_RESOLUTION=10;
#define DEBUT_ONCE true
float _quiescent_Output_voltage=0.5;
float current1 = 0;

// Define missing variables here
bool _zeroCurrentSet = false;
float _zeroCurrentValue = 0;
float sensitivity = 33.0; //  sensitivity=	11.0 for WCS1500(model9) 	33.0 for WCS1700 (model11)

Robojax_WCS currentSensor_1(MODEL, CURRENT_SENSOR_PIN_1, SENSOR_VCC_PIN,
							ZERO_CURRENT_WAIT_TIME, ZERO_CURRENT_LED_PIN,
							CORRECTION_VLALUE, MEASUREMENT_ITERATION, VOLTAGE_REFERENCE,
							BIT_RESOLUTION, DEBUT_ONCE);

// Create a new exponential filter with a weight of 5 and an initial value of 0.
long FilterWeight = 20;
ExponentialFilter<long> ADCFilter(FilterWeight, 0);


ExponentialFilter<long> Pot_Filter(10, 0);
// depth Control

bool act_up_called = false;
unsigned long act_up_start_time = 0;

ezButton BUMPER_SENSOR(BUMPER_SENSOR_PIN);

HardwareSerial Serial5(PD_2, PC_12);

bfs::SbusRx sbus_rx(&Serial5);

bfs::SbusData sbus_data;

// Add a timer to keep track of the duration the current has been above the threshold
static unsigned long currentAboveThresholdDuration = 0;

#if SIYI == 0
// flysky transmitter sbus mapping
int readChannel(byte channelInput, int minLimit, int maxLimit, int defaultValue)
{
  uint16_t ch = sbus_data.ch[channelInput - 1];
  if (ch < 240)
    return defaultValue;
  return map(ch, 240, 1807, minLimit, maxLimit);
}
#else
// SIYI transmitter sbus mapping
int readChannel(byte channelInput, int minLimit, int maxLimit, int defaultValue)
{
  uint16_t ch = sbus_data.ch[channelInput - 1];
  if (ch < 272)
    return defaultValue;
  return map(ch, 272, 1712, minLimit, maxLimit);
}
#endif


void depthControlCurrent() {
  // Get the current reading and filter it
  current1 = abs(currentSensor_1.getCurrent() * 1000);
  ADCFilter.Filter(current1);

  // Check if the current is above the threshold
  if (ADCFilter.Current() > DC_AMPS * 1000) {
    // Increase the duration if the current is above the threshold
    currentAboveThresholdDuration += millis() - lasttime;
  } else {
    // Reset the duration if the current falls below the threshold
    currentAboveThresholdDuration = 0;
  }

  // Check if the duration is more than the specified time and actuator hasn't been triggered
  if (currentAboveThresholdDuration >= DC_TIME && !actUpCalled) {
    // Move the actuator up
    actUpCalled = true;
    actUpStartTime = millis(); // Record the start time
  } else {
    // Check if the actuator has been triggered
    if (actUpCalled) {
      // Move the actuator to a position slightly deeper
      actMoveTo(SET_DEPTH + 20);

      // Calculate the time difference
      unsigned long currentTime = millis();
      unsigned long elapsedTime = currentTime - actUpStartTime;

      // Check if 2 seconds have passed
      if (elapsedTime >= 2000) {
        // Reset the flag and move the actuator to the specified depth
        actUpCalled = false;
        actMoveTo(SET_DEPTH);
      }
    } else {
      // Move the actuator to the specified depth
      actMoveTo(SET_DEPTH);
    }
  }

  lasttime = millis();
}

void setZeroCurrent() {
  // Set the zero current value to the current sensor reading
  zeroCurrentValue = getCurrent();
  zeroCurrentSet = true;
}

float getCurrent() {
  int sensorValue;
  float voltage, current, currentSum = 0;

  for (int i = 0; i < MEASUREMENT_ITERATION; i++) {
    // Read the analog value from the current sensor pin
    sensorValue = analogRead(CURRENT_SENSOR_PIN_1);

    // Wait for a short duration to stabilize readings
    delay(2);

    // Convert sensor value to voltage and then to current
    voltage = (sensorValue) * (VOLTAGE_REFERENCE / (pow(2, BIT_RESOLUTION) - 1)) - (_quiescent_Output_voltage * VOLTAGE_REFERENCE) + CORRECTION_VLALUE;
    current = voltage / sensitivity;

    // Accumulate current readings
    currentSum += current;
  }

  // Calculate the average current over multiple readings
  float averageCurrent = currentSum / MEASUREMENT_ITERATION;

  // If the zero current is set, subtract it from the average current
  if (zeroCurrentSet) {
    averageCurrent -= zeroCurrentValue;
  }

  return averageCurrent;
}


void act_move_to(int target_pos)
{
  // debug("act moving to ");
  // debugln(target_pos);
  potValue = (analogRead(act_potPin) / 10) * 10;
  Pot_Filter.Filter(potValue);
  potValue=Pot_Filter.Current();
  potValue = map(potValue, mini_range, max_range, 100, 0);

  if (potValue >= (target_pos + 2))
  {
    if (abs(potValue - (target_pos + 2)) < 3)
    {
      act_stop();
    }
    else
    {
      act_down();
    }
  }
  else if (potValue <= (target_pos - 2))
  {
    if (abs(potValue - (target_pos - 2)) < 3)
    {
      act_stop();
    }
    else
    {
      act_up();
    }
  }
  else
  {
    act_stop();
  }
}

void depth_control()
{
  int light_on = readChannel(LIGHT_CH, -100, 100, 0);
  if (light_on < 50)
  {
    act_stop();
  }
  else
  {
    depth_control_current();
  }
}

void setup()
{
  pinMode(pwm, OUTPUT);
  pinMode(pwm1, OUTPUT);
  analogWriteFrequency(PWM_FREQ);
  analogWriteResolution(PWM_RESOLUTION);
  analogReadResolution(10);
  pinMode(LIGHT, OUTPUT);
  pinMode(act_lpwm, OUTPUT);
  pinMode(act_rpwm, OUTPUT);
  BUMPER_SENSOR.setDebounceTime(50);
  Serial.begin(115200);
  sbus_rx.Begin();
  stop();
  act_stop();
  currentSensor_1.start();
  delay(20);
}

void stop()
{
  analogWrite(pwm, 0);
  analogWrite(pwm1, 0);
  // act_stop();
}
void act_up()
{
  analogWrite(act_rpwm, 250);
  analogWrite(act_lpwm, 0);
  // debug("act up  ");
}

void act_down()
{
  analogWrite(act_lpwm, 250);
  analogWrite(act_rpwm, 0);
  // debug("act down");
}
void act_stop()
{
  analogWrite(act_lpwm, 0);
  analogWrite(act_rpwm, 0);
  // debug("act stop");
}

void read_actuator()
{
  actuator = readChannel(ACT_CH, -100, 100, 0);
  potValue = (analogRead(act_potPin) / 10) * 10;
  Pot_Filter.Filter(potValue);
  potValue=Pot_Filter.Current();
  potValue = map(potValue, mini_range, max_range, 100, 0);

  if (actuator > 20)
  {
    act_down();
  }

  else if (actuator < -20)
  {
    act_up();
  }
  else
  {
    depth_control();
  }
}

void read_throttle()
{
  throttle = readChannel(TH_CH, -60, 60, 0);
  throttle = constrain(throttle, -max_spd, max_spd);
  if (abs(throttle) < 5)
  {
    throttle = 0;
  }
  throttle_pwm = map(throttle, -60, 60, 250, 125);
}


void read_steer()
{
  steer = readChannel(ST_CH, -60, 60, 0);
  steer = constrain(steer, -30, 30);
  if (abs(steer) < 5)
  {
    steer = 0;
  }
  /* */
  if (abs(throttle) < 5)
  {
    steer = 0;
  }

  if (abs(steer) > abs(throttle))
  {
    if (steer > 0)
    {
      steer = abs(throttle) - 3;
    }
    else if (steer < 0)
    {
      steer = -abs(throttle) + 3;
    }
  }
  steer = constrain(steer, -30, 30);

  steer_pwm = map(steer, -60, 60, 125, 250);
}


void run()
{

  analogWrite(pwm, steer_pwm);
  analogWrite(pwm1, throttle_pwm);
}


void loop() {
  if (sbus_rx.Read()) {
    sbus_data = sbus_rx.data();
    
    // Print SBUS channel data
    for (int8_t i = 0; i < 13; i++) {
      Serial.print(sbus_data.ch[i]);
      Serial.print("\t");
    }
    Serial.println();

    if (sbus_data.failsafe == 0 && sbus_data.lost_frame == 0) {
      readActuator();

      enableSwitch = readChannel(ENA_CH, -100, 100, 0);
      if (enableSwitch > 50) {
        speed = readChannel(SPD_CH, -100, 100, 0);
        if (speed > 0) {
          laneFollow = 0;
          maxSpd = 60;
        } else if (speed < 0) {
          laneFollow = 1;
          maxSpd = 20;
        } else {
          laneFollow = 0;
          maxSpd = 30;
        }

        if (laneFollow == 1) {
          stop();
        } else {
          readThrottle();
          readSteer();
          run();
        }
      } else {
        stop();
      }
    } else {
      stop();
    }
  }
  // current1 = abs(currentSensor_1.getCurrent() * 1000);
  // ADCFilter.Filter(current1);
  // debug(current1);
  // debug("   ");
  // debug(ADCFilter.Current());
  // debug("   ");
  //  //debug(" looptime: ");
  // debug(millis() - prev);
  // prev = millis();
  // debugln();
  // sprint();

  delay(100);
}

void sprint()
{
  debug(" looptime: ");
  debug(millis() - prev);
  prev = millis();
  debug(" Current ");
  debug(ADCFilter.Current()/1000);
  debug(" Angle Y: ");
  debug(filteredAy1);
  debug(" Pot: ");
  debug(potValue);
  debug(" Val: ");
  debug(val);
  debug(" IMU: ");
  debug(estimatedAy);
  debug(" th: ");
  debug(throttle);
  debugln();
}
