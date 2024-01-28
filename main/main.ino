#include <PID_v1.h>

uint8_t speed_robot=150;  // set robot's speed, 0 < speed < 256
int8_t check_out=0;

/// define sensors pinout
#define line_1      A0 // left
#define line_2      A1 // mid-left
#define line_3      A2 // mid
#define line_4      A3 // mid-right
#define line_5      A4 // right

const int ena = 5; //pwm - left
const int enb = 6; //pwm - right
const int in1 = 3; // left motor
const int in2 = 9; // left motor
const int in3 = 10; // right motor
const int in4 = 11; // right motor

//Declare variables to connect
double Setpoint=0, Input, Output;
uint8_t flag_zero=0;

double Kp=20, Ki=0.04555555, Kd=11.898989; 

PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
float sensor;

void setup()
{
  pinMode(ena,1); // pwm pin for left motor
  pinMode(enb,1); // pwm pin for right motor
  pinMode(line_1,INPUT);
  pinMode(line_2,INPUT);
  pinMode(line_3,INPUT);
  pinMode(line_4,INPUT);
  pinMode(line_5,INPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  Input =0; // line_3 == 0 is default, so we must adjust so that line_3 always is 0
  
  myPID.SetSampleTime(1); // sampling time depends on car speed, the faster the sample the better
  myPID.SetMode(AUTOMATIC); 
  myPID.SetOutputLimits(-speed_robot,speed_robot); // the speed value, "-speed" means the left wheel rotates to maximum, the right wheel stops rotating
}

// hashing pulses controls the speed of 2 motors, from there we can control the turning direction, speed... through a single variable
void motorControl(int16_t duty_value)
{
  int16_t speed_a, speed_b;
  int speed_zero;
  speed_zero = speed_robot / 2;

  if (duty_value > 1)
  {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    speed_b = -speed_zero;
    speed_a = duty_value;
  }
  else if (duty_value == 0)
  {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
    speed_a = speed_b = 0;
  }
  else if (duty_value < -1)
  {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    speed_a = -speed_zero;
    speed_b = -duty_value;
  }

  analogWrite(ena, speed_b + speed_zero);  // Bánh trái
  analogWrite(enb, speed_a + speed_zero);  // Bánh phải
}

// light on is 0V, light off (on black line) is 5V, from 5 sensors we have 11 positions -4 -3 -2.5 -2 -1 0 1 2 2.5 3 4
 void scan_sensors()
{
  if (digitalRead(line_1) == 0 && digitalRead(line_2) == 0 && digitalRead(line_3) == 0 && digitalRead(line_4) == 0 && digitalRead(line_5) == 1)
    sensor = 4;
  else if (digitalRead(line_1) == 0 && digitalRead(line_2) == 0 && digitalRead(line_3) == 0 && digitalRead(line_4) == 1 && digitalRead(line_5) == 1)
    sensor = 3;
  else if (digitalRead(line_1) == 0 && digitalRead(line_2) == 0 && digitalRead(line_3) == 1 && digitalRead(line_4) == 1 && digitalRead(line_5) == 1)
    sensor = 2.5;
  else if (digitalRead(line_1) == 0 && digitalRead(line_2) == 0 && digitalRead(line_3) == 0 && digitalRead(line_4) == 1 && digitalRead(line_5) == 0)
    sensor = 2;
  else if (digitalRead(line_1) == 0 && digitalRead(line_2) == 0 && digitalRead(line_3) == 1 && digitalRead(line_4) == 1 && digitalRead(line_5) == 0)
    sensor = 1;
  else if (digitalRead(line_1) == 0 && digitalRead(line_2) == 1 && digitalRead(line_3) == 1 && digitalRead(line_4) == 1 && digitalRead(line_5) == 0)
    sensor = 0;
  else if (digitalRead(line_1) == 0 && digitalRead(line_2) == 0 && digitalRead(line_3) == 1 && digitalRead(line_4) == 0 && digitalRead(line_5) == 0)
    sensor = 0;
  else if (digitalRead(line_1) == 0 && digitalRead(line_2) == 0 && digitalRead(line_3) == 0 && digitalRead(line_4) == 0 && digitalRead(line_5) == 0)
    sensor = 0;
  else if (digitalRead(line_1) == 0 && digitalRead(line_2) == 1 && digitalRead(line_3) == 1 && digitalRead(line_4) == 0 && digitalRead(line_5) == 0)
    sensor = -1;
  else if (digitalRead(line_1) == 0 && digitalRead(line_2) == 1 && digitalRead(line_3) == 0 && digitalRead(line_4) == 0 && digitalRead(line_5) == 0)
    sensor = -2;
  else if (digitalRead(line_1) == 1 && digitalRead(line_2) == 1 && digitalRead(line_3) == 1 && digitalRead(line_4) == 0 && digitalRead(line_5) == 0)
    sensor = -2.5;
  else if (digitalRead(line_1) == 1 && digitalRead(line_2) == 1 && digitalRead(line_3) == 0 && digitalRead(line_4) == 0 && digitalRead(line_5) == 0)
    sensor = -3;
  else if (digitalRead(line_1) == 1 && digitalRead(line_2) == 0 && digitalRead(line_3) == 0 && digitalRead(line_4) == 0 && digitalRead(line_5) == 0)
    sensor = -4;
}

void loop() {
  Setpoint=0;
  scan_sensors(); // read data from sensors
  Input = sensor;
  myPID.Compute(); // calculate
  motorControl(Output); // control the motor to keep the robot on the right path
}
