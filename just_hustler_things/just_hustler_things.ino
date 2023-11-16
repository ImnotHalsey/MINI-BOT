// Prototype Version 1

// Libraries

#include "sbus.h"
#include <Wire.h>
#include <math.h>

// Debug Printer

#define DEBUG 1 // Set this to 0 to stop printing the LOGS

#if DEBUG == 1
  #define debug(x) Serial.print(x)
  #define debugln(x) Serial.println(x)
#else
  #define debug(x)
  #define debugln(x)
#endif

// Declarations

  // Pwm pins
#define pwm 7
#define pwm1 6
#define LIGHT 8
#define pwm_pto 4
#define act_rpwm 9
#define act_lpwm 10

  // i2c multiplexer channels
#define DAC_CH 7
#define RTC_CH 6
#define IMU_CH 5
#define I2C_STEPPER 4
#define IMU1_CH 2
#define IMU2_CH 3

  // Values Declarations 

int max_spd = 60;
bool lane_follow = 0;
int throttle = 0, steer = 0, enable_switch = 0, speed = 0, actuator = 0;
int data = -100;
long int last_i2c_data;
int steer_pwm = 0, throttle_pwm = 0;
long int lasttime;
int sbus_fs_count = 0;
double prev = millis();

// Transmitter Code

    // For Flysky

#define TH_CH 3
#define ST_CH 1
#define ACT_CH 7
#define ENA_CH 5
#define SPD_CH 6
#define PTO_CH 10
#define LIGHT_CH 8

    // For SIYI

// #define TH_CH 3
// #define ST_CH 1
// #define ACT_CH 6
// #define ENA_CH 8
// #define SPD_CH 5
// #define PTO_CH 10
// #define LIGHT_CH 9

// Transmitter Code 
bfs::SbusRx sbus_rx(&Serial1);
bfs::SbusData sbus_data;

    // flysky
int readChannel(byte channelInput, int minLimit, int maxLimit, int defaultValue)
{
  uint16_t ch = sbus_data.ch[channelInput - 1];
  if (ch < 240)
    return defaultValue;
  return map(ch, 240, 1807, minLimit, maxLimit);
}

  // SIYI
// int readChannel(byte channelInput, int minLimit, int maxLimit, int defaultValue)
// {
//   uint16_t ch = sbus_data.ch[channelInput - 1];
//   if (ch < 272)
//     return defaultValue;
//   return map(ch, 272, 1712, minLimit, maxLimit);
// }

void tcaselect(uint8_t bus)
{
  Wire.beginTransmission(0x70); // TCA9548A address is 0x70
  Wire.write(1 << bus);         // send byte to select bus
  Wire.endTransmission();
  delay(1);
}

void depth_control() {
  if (isStraightMotion()) {
    handleStraightMotion();
  } else {
    handleCurvedMotion();
  }
}

bool isStraightMotion() {
  return estimatedAy >= -ang_tol && estimatedAy <= ang_tol;
}

void handleStraightMotion() {
  if (potValue >= idleUpperLimit && abs(potValue - idleUpperLimit) < 3) {
    act_stop();
  } else if (potValue >= idleUpperLimit) {
    act_down();
  } else if (potValue <= idleLowerLimit && abs(potValue - idleLowerLimit) < 3) {
    act_stop();
  } else if (potValue <= idleLowerLimit) {
    act_up();
  } else {
    act_stop();
  }
}

void handleCurvedMotion() {
  int val = (estimatedAy > 0) ? map(estimatedAy, 4, 10, 12, 0) : map(estimatedAy, -4, -10, 12, 24);

  if (potValue < val - 2 && potValue < 60) {
    act_up();
  } else if (potValue > val + 2 && potValue > 10) {
    act_down();
  } else {
    act_stop();
  }
}

void act_up()
{
  analogWrite(act_rpwm, 250);
  analogWrite(act_lpwm, 0);
  debug("act up  ");
}

void act_down()
{
  analogWrite(act_lpwm, 250);
  analogWrite(act_rpwm, 0);
  debug("act down");
}
void act_stop()
{
  analogWrite(act_lpwm, 0);
  analogWrite(act_rpwm, 0);
  debug("act stop");
}

// Reading Actuator 
void read_actuator()
{
  actuator = readChannel(ACT_CH, -100, 100, 0);

  potValue = (analogRead(act_potPin)/10)*10;

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
    // read_imu();
    estimatedAy=0;
    depth_control();
    //act_stop();
  }
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
  if (!sbus_rx.Read() || sbus_data.failsafe != 0) {
    stop();
    return;
  }

  sbus_data = sbus_rx.data();
  read_actuator();

  int light_on = readChannel(LIGHT_CH, -100, 100, 0);
  digitalWrite(LIGHT, (light_on > 50) ? LOW : HIGH);

  enable_switch = readChannel(ENA_CH, -100, 100, 0);
  if (enable_switch <= 50) {
    stop();
    return;
  }

  speed = readChannel(SPD_CH, -100, 100, 0);
  lane_follow = (speed < 0);
  max_spd = lane_follow ? 20 : (speed > 0 ? 60 : 30);

  if (lane_follow) {
    stop();
  } else {
    read_throttle();
    read_steer();
    run();
  }

  sprint();
}

void sprint()
{
  debug(" looptime: ");
  debug(millis() - prev);
  prev = millis();
  debug(" Angle Y: ");
  debug(estimatedAy);
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
