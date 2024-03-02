/*
  Purpose: Run an experiment to determine the transfer function for the translational velocity of the system. 
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

#include <Arduino.h>
#include <Serial.h>
#include <Encoder.h>


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
const double WHEEL_RADIUS_M = 0.1;
const double DISTANCE_BETWEEN_WHEELS_M = 0.5;
unsigned long desired_Ts_ms = 10; // desired sample time in milliseconds



// Globals
unsigned long start_time_ms;
unsigned long last_time_ms;   // time for calculating time steps
float current_time;
unsigned int PWM;             // value 0-255 to set duty cycle. 0: low, 255: high
volatile double target_pos_rad[2] = {0};
double out_voltage[2] = {0};
double rotational_velocity[2] = {0};
double prev_pos_rad[2] = {0,0};
double new_pos_rad[2] = {0,0};


// class for holding motor pins
struct MotorPins {
  int pwm;
  int dir;
};

// motor variables
Encoder encoders[2] = {Encoder(ENC1_A, ENC1_B), Encoder(ENC2_A, ENC2_B)};
MotorPins motor_pins[2];



double Va1 = 3.5;
double Va2 = 3.5;
double Va;
double inst_fwd_vel_MpS = 0;

// Va = Va1 + Va2 = 1 (testing)  testing forward velocity
// deltaVa = Va1 - Va2 = 0 (not test) 

void setup() {
  out_voltage[0] = Va1;
  out_voltage[1] = Va2;
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

  Serial.begin(115200);
  Serial.println("Ready!");
  last_time_ms = millis(); // set up sample time variable
  start_time_ms = last_time_ms;
}

int finished = false;

void loop() {

  // Variables for calculating position
  int i;
  unsigned int PWM;
  double pos_rad[2] = {0,0};
  double w_motor_RadpS[2] = {0,0};
  
   // // print out timestamp in seconds
  current_time = (float)(last_time_ms-start_time_ms)/1000;

  if ((current_time >= 1) && (current_time <= 3 )) {
    // loop through to control both motors
    for (i=0; i<2; i++) {

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


      new_pos_rad[i] = countsToRads(encoders[i].read());

      w_motor_RadpS[i] = (new_pos_rad[i] - prev_pos_rad[i]) / ((float)desired_Ts_ms/1000);

      prev_pos_rad[i] = new_pos_rad[i];

    } 

    
    
    Va = Va1 + Va2;

    inst_fwd_vel_MpS = WHEEL_RADIUS_M * (w_motor_RadpS[0] + w_motor_RadpS[1]) / 2;

    Serial.print(current_time);
    Serial.print("\t");
    Serial.print(Va);
    Serial.print("\t");
    Serial.println(inst_fwd_vel_MpS);
  }
  else {
    for (i=0; i<2; i++) {
      analogWrite(motor_pins[i].pwm, 0);
    }
  }


  if (current_time > 3 && !finished) {
    Serial.println("Finished");
    finished = true;
  }

  while (millis()<last_time_ms + desired_Ts_ms) {
  //wait until desired time passes to go top of the loop
  }
  last_time_ms = millis();
}

  



// function to convert cnts to rads
float countsToRads(long enc_counts) {
  return 2*pi * (float) enc_counts/ ENC_CNT_PER_REV;
}


