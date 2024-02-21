/*
  Purpose: Test the PI controller by toggling the target every 5 seconds 
  Authors: Ben Sprik and Blake Billharz
  Sources: N/A
  Set up: 
    Connect the motor driver shield to the arduino and connect power from the battery via the voltage regulator.
    
    Left motor:
      encoderPwr -> 3v3
      encoderGnd -> GND
      encoderA -> 2
      encoderB -> 5
      direction -> 7
      pwm -> 9
    Right motor:
      encoderPwr -> 3v3
      encoderGnd -> GND
      encoderA -> 3
      encoderB -> 6
      direction -> 8
      pwm -> 10
    I2C:
      SCL -> A5
      SDA -> A4
      GND -> GND
*/

# include <Encoder.h>

// pinout to connect motor driver and motor
#define ENC1_A 2
#define ENC2_A 3
#define nD2    4
#define ENC1_B 5
#define ENC2_B 6
#define M1DIR  7
#define M2DIR  8
#define M1PWM  9
#define M2PWM  10

// constants
const double pi = 3.141592;
const double MAX_VOLTAGE = 7.8;
const int ENC_CNT_PER_REV = 3200;

// Controller gains as determined by simulink model
double Kp = 12;
double Ki = 5;

// Globals
unsigned long last_time_ms;   // time for calculating time steps
unsigned int PWM;             // value 0-255 to set duty cycle. 0: low, 255: high
volatile double target_pos_rad[2] = {0};
double out_voltage[2] = {0};
double integral_error[2] = {0};

// class for holding motor pins
struct MotorPins {
  int pwm;
  int dir;
};

// motor variables
Encoder encoders[2] = {Encoder(ENC1_A, ENC1_B), Encoder(ENC2_A, ENC2_B)};
MotorPins motor_pins[2];


////////////////////////// INTERUPT TESTING
// #define TIMER_INTERRUPT_DEBUG         0
// #define _TIMERINTERRUPT_LOGLEVEL_     0
#define USE_TIMER_2 true
#define INTERRUPT_INTERVAL_MS  5000
#include "TimerInterrupt.h"

void recieveTargetISR() {
  // temporary code to toggle target
  for (int i = 0; i < 2; i++) {
    if (target_pos_rad[i] == 0) {
      target_pos_rad[i] = pi;
    }
    else {
      target_pos_rad[i] = 0;
    }
  }

}
//////////////////////////////


void setup() {
  
  // setup motors
  motor_pins[0].pwm = M1PWM;
  motor_pins[0].dir = M1DIR;
  motor_pins[1].pwm = M2PWM;
  motor_pins[1].dir = M2DIR;

  pinMode(nD2, OUTPUT);
  pinMode(M1DIR, OUTPUT);
  pinMode(M1PWM, OUTPUT);
  pinMode(M2DIR, OUTPUT);
  pinMode(M2PWM, OUTPUT);

  digitalWrite(nD2, HIGH);    // enable motor driver outputs

  // setup serial
  Serial.begin(9600);
  while(!Serial);

  // set up isr to recieve target
  ITimer2.init();
  if (!ITimer2.attachInterruptInterval(INTERRUPT_INTERVAL_MS, recieveTargetISR)){
    Serial.println("Couldn't set up interrupt. Quitting.");
    while (1);
  }

  last_time_ms = millis(); // set up sample time variable
}

double target_last = 0;
unsigned long time_lp_ms = 0;

void loop() {

  // Variables for calculating position
  int i;
  unsigned int PWM;
  double pos_error[2] = {0};

  // calculate time step for computing integral error
  unsigned long time_step_ms = millis() - last_time_ms;
  last_time_ms = millis();

  // loop through to control both motors
  for (i=0; i<2; i++) {

    // calculate position error
    pos_error[i] = target_pos_rad[i] - countsToRads( encoders[i].read() );

    // update integral error
    integral_error[i] = integral_error[i] + pos_error[i] * ((float) time_step_ms / 1000);

    // calculate voltage
    out_voltage[i] = Kp*pos_error[i] + Ki*integral_error[i];

    // drive motors direction
    if (out_voltage[i]>0) {
      digitalWrite(motor_pins[i].dir, HIGH);
    } 
    else {
      digitalWrite(motor_pins[i].dir, LOW);
    }

    // drive motors speed
    PWM = 255*abs(out_voltage[i])/MAX_VOLTAGE;
    analogWrite(motor_pins[i].pwm, min(PWM,255));

  }

  if (target_pos_rad[0] != target_last) {
    Serial.println("target: " + (String) target_pos_rad[0]);
    target_last = target_pos_rad[0];
  }

  time_lp_ms += time_step_ms;
  if (time_lp_ms >= 500) {
    Serial.println("Error 0: " + (String) pos_error[0]);
    Serial.println("Error 1: " + (String) pos_error[1]);
    Serial.println();
    time_lp_ms = 0;
  }

}


float countsToRads(long enc_counts) {
  return 2*pi * (float) enc_counts/ ENC_CNT_PER_REV;
}

