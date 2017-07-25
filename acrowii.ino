#include <Servo.h>

#include "PID_v1.h"
#include "PinChangeInterrupt.h"

#define RC_NUM_CHANNELS 4

#define RC_THROTTLE_CHANNEL 0
#define RC_ROLL_CHANNEL 1
#define RC_PITCH_CHANNEL 2
#define RC_YAW_CHANNEL 3

// arduino uno rev 3
#define RC_THROTTLE_PIN A0
#define RC_ROLL_PIN A1
#define RC_PITCH_PIN A2
#define RC_YAW_PIN A3

// flyduino nanowii
//#define RC_THROTTLE_PIN D7
//#define RC_ROLL_PIN D16
//#define RC_PITCH_PIN D14
//#define RC_YAW_PIN D15

#define RC_EXPO 0.0

#define RC_HI 2000
#define RC_MID 1500
#define RC_LOW 1000

#define PID_ROLL_P 1.0
#define PID_ROLL_I 0.00
#define PID_ROLL_D 0.0
#define PID_ROLL_RATE 1.0

#define PID_PITCH_P 1.0
#define PID_PITCH_I 0.00
#define PID_PITCH_D 0.0
#define PID_PITCH_RATE 1.0

#define PID_YAW_P 1.0
#define PID_YAW_I 0.00
#define PID_YAW_D 0.0
#define PID_YAW_RATE 1.0

// arduino uno rev 3
#define ESC_OUTPUT_0_PIN 3
#define ESC_OUTPUT_1_PIN 5
#define ESC_OUTPUT_2_PIN 6
#define ESC_OUTPUT_3_PIN 9

// flyduino nanowii
//#define ESC_OUTPUT_0_PIN D6
//#define ESC_OUTPUT_1_PIN D10
//#define ESC_OUTPUT_2_PIN D9
//#define ESC_OUTPUT_3_PIN D5

#define ESC_OUTPUT_MIN 1000
#define ESC_OUTPUT_ARM 1100
#define ESC_OUTPUT_MAX 2000

#define ESC_ARMING_DELAY 250000

boolean armed = false;
volatile int rc_channel[RC_NUM_CHANNELS];
int esc_power[4];
Servo esc[4];

static void interruptRcThrottle()
{ 
  static unsigned long timer;
  uint8_t trigger = getPinChangeInterruptTrigger(digitalPinToPCINT(RC_THROTTLE_PIN));
  timeInterrupt(&timer, &trigger, &rc_channel[RC_THROTTLE_CHANNEL]);
}

static void interruptRcRoll()
{
  static unsigned long timer;
  uint8_t trigger = getPinChangeInterruptTrigger(digitalPinToPCINT(RC_ROLL_PIN));
  timeInterrupt(&timer, &trigger, &rc_channel[RC_ROLL_CHANNEL]);
}

static void interruptRcPitch()
{
  static unsigned long timer;
  uint8_t trigger = getPinChangeInterruptTrigger(digitalPinToPCINT(RC_PITCH_PIN));
  timeInterrupt(&timer, &trigger, &rc_channel[RC_PITCH_CHANNEL]);
}

static void interruptRcYaw()
{
  static unsigned long timer;
  uint8_t trigger = getPinChangeInterruptTrigger(digitalPinToPCINT(RC_YAW_PIN));
  timeInterrupt(&timer, &trigger, &rc_channel[RC_YAW_CHANNEL]);
}

static int timeInterrupt(unsigned long *timer, uint8_t *trigger, volatile int *rc_channel)
{
  if(*trigger == RISING)
  { 
    *timer = micros();
  } 
  else
  { 
    *rc_channel = micros() - *timer;
    *timer = 0;
  } 
}

static void doArmDisarm()
{
  static unsigned long time_start = 0; 

  if (rc_channel[RC_THROTTLE_CHANNEL] < 1050 && rc_channel[RC_YAW_CHANNEL] > 1950 && !armed)
  {
    // arm
    if (time_start == 0)
      time_start = micros();
    
    unsigned long now = micros();

    if (now - time_start > ESC_ARMING_DELAY)
    {
      armed = true;
      time_start = 0;
    }
  } 
  else if (rc_channel[RC_THROTTLE_CHANNEL] < 1050 && rc_channel[RC_YAW_CHANNEL] < 1050 && armed)
  {
    // disarm
    if (time_start == 0)
      time_start = micros();
    
    unsigned long now = micros();

    if (now - time_start > ESC_ARMING_DELAY)
    {
      esc_power[0] = ESC_OUTPUT_MIN;
      esc_power[1] = ESC_OUTPUT_MIN;
      esc_power[2] = ESC_OUTPUT_MIN;
      esc_power[3] = ESC_OUTPUT_MIN;
      armed = false;
      time_start = 0;
    }
  }
  else
  {
    time_start = 0;
  }
}

static void doThrottle()
{
  if (!armed)
    return;

  int throttle = rc_channel[RC_THROTTLE_CHANNEL];

  if (throttle < RC_LOW)
    throttle = RC_LOW;
  else if (throttle > RC_HI)
    throttle = RC_HI;
  
  int throttle_output = map(throttle, RC_LOW, RC_HI, ESC_OUTPUT_ARM, ESC_OUTPUT_MAX);
  esc_power[0] = throttle_output;
  esc_power[1] = throttle_output;
  esc_power[2] = throttle_output;
  esc_power[3] = throttle_output;
}

static void doEngineOutput()
{
  esc[0].writeMicroseconds(esc_power[0]);
  esc[1].writeMicroseconds(esc_power[1]);
  esc[2].writeMicroseconds(esc_power[2]);
  esc[3].writeMicroseconds(esc_power[3]);
}

static void printDebugInfo()
{
  Serial.print("thr: ");
  Serial.print(rc_channel[RC_THROTTLE_CHANNEL]);
  Serial.print(" roll: ");
  Serial.print(rc_channel[RC_ROLL_CHANNEL]);
  Serial.print(" pitch: ");
  Serial.print(rc_channel[RC_PITCH_CHANNEL]);
  Serial.print(" yaw: ");
  Serial.print(rc_channel[RC_YAW_CHANNEL]);
  Serial.print(" armed: ");
  Serial.print(armed);
  Serial.print(" output: ");
  Serial.print(esc_power[0]);
  Serial.print(", ");
  Serial.print(esc_power[1]);
  Serial.print(", ");
  Serial.print(esc_power[2]);
  Serial.print(", ");
  Serial.print(esc_power[3]);
  Serial.println();
}

void setup() 
{
  pinMode(RC_THROTTLE_PIN, INPUT);
  pinMode(RC_ROLL_PIN, INPUT);
  pinMode(RC_PITCH_PIN, INPUT);
  pinMode(RC_YAW_PIN, INPUT);
  
  attachPCINT(digitalPinToPCINT(RC_THROTTLE_PIN), interruptRcThrottle, CHANGE);
  attachPCINT(digitalPinToPCINT(RC_ROLL_PIN), interruptRcRoll, CHANGE);
  attachPCINT(digitalPinToPCINT(RC_PITCH_PIN), interruptRcPitch, CHANGE);
  attachPCINT(digitalPinToPCINT(RC_YAW_PIN), interruptRcYaw, CHANGE);

  esc_power[0] = ESC_OUTPUT_MIN;
  esc_power[1] = ESC_OUTPUT_MIN;
  esc_power[2] = ESC_OUTPUT_MIN;
  esc_power[3] = ESC_OUTPUT_MIN;
  
  esc[0].attach(ESC_OUTPUT_0_PIN);
  esc[1].attach(ESC_OUTPUT_1_PIN);
  esc[2].attach(ESC_OUTPUT_2_PIN);
  esc[3].attach(ESC_OUTPUT_3_PIN);
  
  Serial.begin(9600);
}

void loop() 
{
  doArmDisarm();
  doThrottle();
  doEngineOutput();
  printDebugInfo();
  delay(100);
}
