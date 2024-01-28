/***********************************************************************************************************
   ___  _    _ _     _                  
  / _ \| |__(_) |_  | |_ ___ __ _ _ __  
 | (_) | '_ \ |  _| |  _/ -_) _` | '  \ 
  \___/|_.__/_|\__|  \__\___\__,_|_|_|_|


Robot line follow 
Using Arduino UNO R3 + PID library
-------------------------------------------------------------------------------------------------------------------------------------------
Code by Chu Tiến Thịnh  
Mọi thắc mắc xin liên hệ theo thông tin bên dưới, nếu sự cố phức tạp vui lòng gặp trực tiếp hoặc call video (có thể tính phí tùy trường hợp)
Các bạn có nhu cầu đặt hàng xe cũng vui lòng liên hệ theo thông tin bên dưới
Thanks
--------------------------------------------------------------------------------------------------------------------------------------------
chutienthinh.tuhu@gmail.com
chutienthinh.bk@gmail.com
+84981762826
https://www.facebook.com/thinh.chutien                                   
https://www.facebook.com/obit.team/
https://obitvn.wordpress.com/
Chuc Ban May Man

************************************************************************************************************/
uint8_t speed_robot=150;  // set (tốc độ của robot) speed_robot (pwm value), 0 < speed_robot < 256
int8_t check_out=0;

/// define sensor pinout
#define line_1      A0 // trái (hoặc ngược lại)
#define line_2      A1
#define line_3      A2 // giữa
#define line_4      A3 
#define line_5      A4 // phải

const int ena = 5;
const int enb = 6;
const int in1 = 3;
const int in2 = 9;
const int in3 = 10;
const int in4 = 11;

#include <PID_v1.h> // download tại https://github.com/br3ttb/Arduino-PID-Library


//Define Variables we'll be connecting to
double Setpoint=0, Input, Output;
uint8_t flag_zero=0;

//double Kp=15, Ki=0.045, Kd=12;  
double Kp=20, Ki=0.04555555, Kd=11.898989; 
// giá trị cần được thay đổi cho từng loại xe


PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
float sensor;

void setup()
{
  pinMode(ena,1); // chân băm xung cho động cơ
  pinMode(enb,1); // chân băm xung cho động cơ
  pinMode(line_1,INPUT);
  pinMode(line_2,INPUT);
  pinMode(line_3,INPUT);
  pinMode(line_4,INPUT);
  pinMode(line_5,INPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  Input =0; // mặc định line giữa =0, => luôn luôn điều chỉnh xe để input=0
  
  myPID.SetSampleTime(1); // thời gian lấy mẫu phụ thuộc tốc độ xe, lấy mẫu càng nhanh càng tốt
  myPID.SetMode(AUTOMATIC); 
  myPID.SetOutputLimits(-speed_robot,speed_robot); // giá trị tốc độ, -speed tức bánh bên trái quay max, bên phải ngừng quay
  
}

///// băm xung điều khiển tốc độ 2 bánh xe, từ đó ta điều khiển được hướng rẽ, tốc dộ..... thông qua 1 biến duy nhất
//void motorControl(int16_t duty_value)
//{
//  int16_t speed_a, speed_b;
//  int speed_zero;
//  speed_zero = speed_robot/2;
//  if(duty_value>1) 
//  {
//    speed_b=-speed_zero;
//    speed_a=duty_value;
//  }
//  else if(duty_value==0) 
//  {
//    speed_a=speed_b=0;
//    
//  }
//  else if(duty_value<-1) 
//  {
//    speed_a=-speed_zero;
//    speed_b=-duty_value;
//  } 
//      analogWrite(6,speed_b+speed_zero);  //trai //85
//      analogWrite(5,speed_a+speed_zero); //phai
//}     

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

//Sáng đèn là 0v, đèn tắt (vào line đen) là 5V, từ 5 cảm biến ta có 9 vị trí -4 -3 -2 -1 0 1 2 3 4
 void scan_sensor()
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
    sensor = 0;  // Thêm trường hợp này
  else if (digitalRead(line_1) == 0 && digitalRead(line_2) == 0 && digitalRead(line_3) == 1 && digitalRead(line_4) == 0 && digitalRead(line_5) == 0)
    sensor = 0;  // Nguyên trường hợp vào line giữa
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
  scan_sensor(); // đọc dữ liệu từ cảm biến
  Input = sensor;
  myPID.Compute(); // tính toán
  motorControl(Output); // điều khiển bánh xe để robot đi đúng đường
 
}
