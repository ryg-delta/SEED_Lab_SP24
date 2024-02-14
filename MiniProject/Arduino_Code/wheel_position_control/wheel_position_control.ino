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
#define ENC1_A = 2;
#define ENC2_A = 3;
#define nD2 = 4;
#define ENC1_B = 5;
#define ENC2_B = 6;
#define M1DIR = 7;
#define M2DIR = 8;
#define M1PWM = 9;
#define M2PWM = 10;

unsigned long desired_Ts_ms = 10; // desired sample time in milliseconds
unsigned long last_time_ms;
unsigned long start_time_ms;
double current_time;
double actual_speed;
double desired_speed = 4;   // speed for motor to rotate rad/s

double error;
double Voltage;
double Kp = 4.5;           // Gain. Determined using simulink model.
double Ki = 1; 

unsigned int PWM;           // value 0-255 to set duty cycle. 0: low, 255: high

double BatteryVoltage = 7.8; // expected voltage from battery
double pi = 3.14;



double target_pos_rad[2] = {0};

//long M_Enc_Count[2] = {-999,999};

Encoder encoders[2]; 

int motor_PWM_pins[2] = {M1PWM, M2PWM}
int motor_DIR_pins[2] = {M1DIR, M2DIR};

double Voltage[2] = {0};

double integral_error[2] = {0};

void setup() {
  // setup encoder using Encoder.h library
   
  encoders[0] = Encoder(ENC1_A, ENC1_B);
  encoders[1] = Encoder(ENC2_A, ENC2_B);

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
    Voltage[i] = Kp*pos_error[i] + Ki*interal_error[i];

    // drive motors direction
    if (Voltage[i]>0) {
      digitalWrite(motor_DIR_pins[i], HIGH);
    } 
    else {
      digitalWrite(motor_DIR_pins[i], LOW);
    }

    // drive motors speed
    PWM = 255*abs(Voltage[i])/BatteryVoltage;
    analogWrite(motor_PWM_pins[i], min(PWM,255));

  }

}


float counts_to_radians(long enc_counts) {
  return 2*pi*(float)enc_counts/3200;
}
