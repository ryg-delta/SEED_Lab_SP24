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

unsigned long desired_Ts_ms = 10; // desired sample time in milliseconds
unsigned long last_time_ms;
unsigned long start_time_ms;
float current_time;
float actual_speed;
float desired_speed = 4;   // speed for motor to rotate rad/s

float error;
float Voltage;
float Kp = 4.5;           // Gain. Determined using simulink model. 

unsigned int PWM;           // value 0-255 to set duty cycle. 0: low, 255: high

float BatteryVoltage = 7.8; // expected voltage from battery
float pi = 3.14;

// pinout to connect motor driver and motor
int ENC1_A = 2;
int nD2 = 4;
int ENC1_B = 5;
int M1DIR = 7;
int M1PWM = 9;

// setup encoder using Encoder.h library
Encoder M1ENC(ENC1_A, ENC1_B);
long M1_Enc_Count = -999;


void setup() {
  
  pinMode(nD2, OUTPUT);
  pinMode(M1DIR, OUTPUT);
  pinMode(M1PWM, OUTPUT);

  digitalWrite(nD2, HIGH);    // enable motor driver outputs

  Serial.begin(115200);
  Serial.println("Ready!");
  last_time_ms = millis(); // set up sample time variable
  start_time_ms = last_time_ms;
}


float prev_pos1_rad = 0;   // holds previous encoder posotion
int finished = false;     // when 2 seconds have passed, the motor control is finished

void loop() {
  // Variables for calculating position
  float pos1_rad;
  long new_M1_Enc_Count;
  
  // print out timestamp in seconds
  current_time = (float)(last_time_ms-start_time_ms)/1000;

  // run the motor between 1 and 3 seconds
  if ((current_time >= 1) && current_time <= 3) {
    
    // read the encoder
    new_M1_Enc_Count = M1ENC.read();
    // update the M1 encoder count
    if (new_M1_Enc_Count != M1_Enc_Count) {
      M1_Enc_Count = new_M1_Enc_Count;
    }
    
    // determine the curent position of encoder in radians
    pos1_rad= 2*pi*(float)M1_Enc_Count/3200;

    // calculate speed
    actual_speed = (pos1_rad - prev_pos1_rad) / ((float)desired_Ts_ms/1000);

    // save current encoder location
    prev_pos1_rad = pos1_rad;

    // determine error
    error = desired_speed - actual_speed;
    // calculate voltage to apply to motor
    Voltage = Kp*error;

    // check the sign of voltage and set the motor driver sign pin as appropriate
    if (Voltage>0) {
      digitalWrite(M1DIR, HIGH);
    } else {
      digitalWrite(M1DIR, LOW);
    }

    // calculate the PWM value and apply the value to the M1PWM pin
    PWM = 255*abs(Voltage)/BatteryVoltage;
    analogWrite(M1PWM, min(PWM,255));

    // print time, voltage, and speed
    Serial.print(current_time);
    Serial.print("\t");
    Serial.print(Voltage);
    Serial.print("\t");
    Serial.println(actual_speed);
  }
  // time not between 1 and 3 s, motor off
  else {
     analogWrite(M1PWM, 0);
  }
 
  // tell matlab program data done streaming data
  if (current_time > 3 && !finished) {
    Serial.println("Finished");
    finished = true;
  }

  // sample period = desired_Ts_ms
  while (millis()<last_time_ms + desired_Ts_ms) {
  //wait until desired time passes to go top of the loop
  }
  last_time_ms = millis();

}

