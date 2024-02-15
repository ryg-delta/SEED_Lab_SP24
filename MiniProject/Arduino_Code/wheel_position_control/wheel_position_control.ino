/*
Intro:
motor_velocity_control turns a motor on and uses a basic control system to reach a set speed 
in rad/s between 1s and 3s of the program running.

Connect battery to voltage monitor through a fuse and switch. Place the Pololu Dual MC33926 Motor Driver Shield
onto the Arduino. Connect the power inputs of the motor driver to the outputs of the voltage monitor. Make sure 
the jumper on the motor driver is disconnected. Connect the motor power pins to the M1PWR outputs on the motor
hat. Connect encoder pins to the motor hat with jumpers: ENCA-> 2, ENCB-> 5, ENCPWR->5V, ENCGND->GND. Power the
Arduino with USB to recieve serial data about time, voltage, and motor speed. The data output is setup to be captured
by ReadFromArduino.mlx. 
*/

# include <Encoder.h>

// pinout to connect motor driver and motor
#define ENC1_A 2
#define ENC2_A 3
#define nD2 4
#define ENC1_B 5
#define ENC2_B 6
#define M1DIR 7
#define M2DIR 8
#define M1PWM 9
#define M2PWM 10

// constants
const double pi = 3.141592;
const double BatteryVoltage = 7.8;

// Controller gains as determined by simulink model
double Kp = 4.5;
double Ki = 1; 

// Globals
unsigned long last_time_ms;   // time for calculating time steps
unsigned int PWM;             // value 0-255 to set duty cycle. 0: low, 255: high
double target_pos_rad[2] = {0};
double Voltage[2] = {0};
double integral_error[2] = {0};

// class for holding motor pins
struct MotorPins {
  int pwm;
  int dir;
};

// motor variables
Encoder encoders[2] = {Encoder(ENC1_A, ENC1_B), Encoder(ENC2_A, ENC2_B)};
MotorPins motorPins[2];
int motor_PWM_pins[2] = {M1PWM, M2PWM};
int motor_DIR_pins[2] = {M1DIR, M2DIR};

void setup() {
  
  // setup motors
  motorPins[0].pwm = M1PWM;
  motorPins[0].dir = M1DIR;
  motorPins[1].pwm = M2PWM;
  motorPins[1].dir = M2DIR;

  pinMode(nD2, OUTPUT);
  pinMode(M1DIR, OUTPUT);
  pinMode(M1PWM, OUTPUT);
  pinMode(M2DIR, OUTPUT);
  pinMode(M2PWM, OUTPUT);

  digitalWrite(nD2, HIGH);    // enable motor driver outputs

  // setup serial
  Serial.begin(115200);
  Serial.println("Ready!");

  last_time_ms = millis(); // set up sample time variable
}

//float prev_pos_ENC[2] = {0};   // holds previous encoder posotion

void loop() {
  // Variables for calculating position
  int i;
  unsigned int PWM;
 
  double pos_error[2] = {0};
  unsigned long time_step_ms = millis() - last_time_ms;
  last_time_ms = millis();

  for (i=0; i<2; i++) {
    // calculate position error
    pos_error[i] = target_pos_rad[i] - counts_to_radians( encoders[i].read() );

    // update integral error
    integral_error[i] = integral_error[i] + pos_error[i]*((float)time_step_ms/1000);

    // calculate voltage
    Voltage[i] = Kp*pos_error[i] + Ki*integral_error[i];

    // drive motors direction
    if (Voltage[i]>0) {
      digitalWrite(motorPins[i].dir, HIGH);
    } 
    else {
      digitalWrite(motorPins[i].dir, LOW);
    }

    // drive motors speed
    PWM = 255*abs(Voltage[i])/BatteryVoltage;
    analogWrite(motorPins[i].pwm, min(PWM,255));

  }

}


float counts_to_radians(long enc_counts) {
  return 2*pi*(float)enc_counts/3200;
}

