#include <Servo.h>
#include <PinChangeInterrupt.h>  //外部中断
#include <MsTimer2.h>    //定时中断

Servo myservo;      //创建一个舵机控制对象
#define distanceBiasFront 7

/////////TB6612驱动引脚////
#define RightN 3    //  AIN1  //由于定时器2的使用，Pin3和Pin11无法输出PWM，只能作为数字引脚
#define RightP 6    //   AIN2
#define LeftN 11    //   BIN1  //由于定时器2的使用，Pin3和Pin11无法输出PWM，只能作为数字引脚
#define LeftP 5      //   BIN2
#define SERVO 9

// 超声波引脚
#define TrigF 18
#define EchoF 17
#define TrigL 16
#define EchoL 15

/////////编码器引脚////////
#define ENCODER_L 4    //编码器采集引脚 每路2个 共4个
#define DIRECTION_L 8
#define ENCODER_R 2
#define DIRECTION_R 7

#define T 0.156f
#define L 0.1445f
#define pi 3.1415926
#define MAXDIS 11600

volatile long Velocity_L, Velocity_R;  //左右轮编码器数据
int Velocity_Left, Velocity_Right = 0, Velocity, Angle;  //左右轮速度
float Velocity_KP = 1.5, Velocity_KI = 0.5;
float leftP = 5, leftI = 0, leftD = 40;
float serialKp = 0, kp;
unsigned char Flag_Stop = 0;  //停止标志位和上位机相关变量
float TargetR, TargetL;
int Battery_Voltage;    //电池电压采样变量
unsigned char servo;
int MotorR, MotorL;
boolean stringComplete = false;  // 蓝牙串口接收命令完成指示
float distance_Front, Last_distance_Front, limit = 20;
String comdata;
int maxPwm = 250, maxVel = 45;  //maxVel = 45：10ms定时器中断时，最大转速为45
int initAngle = 95;
volatile unsigned int numStop;
unsigned int durationF;
unsigned char newvalueF = 0;
unsigned int durationL;
unsigned char newvalueL = 0;
/**************************************************************************
  函数功能：赋值给PWM寄存器
  入口参数：PWM
**************************************************************************/
void Driver(int motora, int motorb)
{
  if (motora > maxPwm)
    motora = maxPwm;
  if (motora < -maxPwm)
    motora = -maxPwm;
  if (motorb > maxPwm)
    motorb = maxPwm;
  if (motorb < -maxPwm)
    motorb = -maxPwm;

  if (motora > 0)
    analogWrite(RightP, motora), digitalWrite(RightN, LOW);  //赋值给PWM寄存器
  else
    analogWrite(RightP, 255 + motora), digitalWrite(RightN, HIGH);  //赋值给PWM寄存器
  if (motorb > 0)
    analogWrite(LeftP, motorb), digitalWrite(LeftN, LOW);  //赋值给PWM寄存器
  else
    analogWrite(LeftP, 255 + motorb), digitalWrite(LeftN, HIGH);  //赋值给PWM寄存器
}

/**************************************************************************
  函数功能：异常关闭电机
  入口参数：电压
  返回  值：1：异常  0：正常
**************************************************************************/
unsigned char Turn_Off()
{
  byte temp;
  if (Flag_Stop == 1 || (Battery_Voltage < 700))  //Flag_Stop置1或者电压太低关闭电机
  {
    temp = 1;
    digitalWrite(RightP, LOW);  //电机驱动的电平控制
    digitalWrite(RightN, LOW);  //电机驱动的电平控制
    digitalWrite(LeftP, LOW);  //电机驱动的电平控制
    digitalWrite(LeftN, LOW);  //电机驱动的电平控制
  } else
    temp = 0;
  return temp;
}

/**************************************************************************
  函数功能：小车运动数学模型
  入口参数：速度和转角
**************************************************************************/
void Kinematic_Analysis(float velocity, float angle)
{
  char K = 1;
  TargetL = velocity * (1 + T * tan(angle * pi / 180) / 2 / L);
  TargetR = velocity * (1 - T * tan(angle * pi / 180) / 2 / L);  //后轮差速
  servo = initAngle + angle * K;  //舵机转向
  //  if(servo>95)servo=servo*1.15;
  myservo.write(servo);  // 指定舵机转向的角度
}

/**************************************************************************
  函数功能：增量PI控制器
  入口参数：编码器测量值，目标速度
  返回  值：电机PWM
  根据增量式离散PID公式
  pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
  e(k)代表本次偏差
  e(k-1)代表上一次的偏差  以此类推
  pwm代表增量输出
  在我们的速度控制闭环系统里面，只使用PI控制
  pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)
**************************************************************************/
int Incremental_PI_Right(int Encoder, int Target)
{
  static float Bias_Right, Pwm_Right, Last_bias_Right;
  Bias_Right = Encoder - Target;  //计算偏差
  Pwm_Right -= Velocity_KP * (Bias_Right - Last_bias_Right) + Velocity_KI * Bias_Right;  //增量式PI控制器
  Last_bias_Right = Bias_Right;  //保存上一次偏差
  if (Pwm_Right > maxPwm)
    Pwm_Right = maxPwm;
  if (Pwm_Right < -maxPwm)
    Pwm_Right = -maxPwm;
  return Pwm_Right;  //增量输出
}

int Incremental_PI_Left(int Encoder, int Target)
{
  static float Bias_Left, Pwm_Left, Last_bias_Left;
  Bias_Left = Encoder - Target;  //计算偏差
  Pwm_Left -= Velocity_KP * (Bias_Left - Last_bias_Left) + Velocity_KI * Bias_Left;  //增量式PI控制器
  Last_bias_Left = Bias_Left;  //保存上一次偏差
  if (Pwm_Left > maxPwm)
    Pwm_Left = maxPwm;
  if (Pwm_Left < -maxPwm)
    Pwm_Left = -maxPwm;
  return Pwm_Left;  //增量输出
}

/*********函数功能：10ms控制函数 核心代码 *******/
void control()
{
  int Temp;    //临时变量
  static float Voltage_All;  //电压采样相关变量
  static unsigned char Voltage_Count;
  static unsigned char state;
  static float oldleft, left, leftint;
  float tmpdis;
  if (state == 0) {  // 每 50ms 测一次距离
    digitalWrite(TrigF, HIGH);
    digitalWrite(TrigL, HIGH);
    delayMicroseconds(10);
    digitalWrite(TrigF, LOW);
    digitalWrite(TrigL, LOW);
  }
  if (state == 1) Angle = 0;
  if (state < 4)
    state++;
  else
    state = 0;
  tmpdis = (Velocity_L + Velocity_R) * 0.5 / 40 * 5; // 通过编码器计算的距离(*5表示50ms的距离)
  if (state == 4) {
    if (newvalueL) {
      left = durationL * 0.5 / 29;
      Serial.println(left);
      leftint += (20 - left);
      if (left < 10) {
        numStop = 3;
      }
    }
    else
      oldleft = 0;
    newvalueL = 0;
    if (oldleft != 0) {
      Angle = (20 - left) * leftP +(oldleft - left) * leftD + leftint * leftI * 0.1;
    }
    if (Angle > 36) Angle = 36;
    if (Angle < -36) Angle = -36;

    oldleft = left;
  }

  if (newvalueF) {
    float tmpmeasure = durationF * 0.5 / 29 - distanceBiasFront;
    if (tmpmeasure < 20) numStop = 3;
    newvalueF = 0;
  }

  Velocity_Left = Velocity_L;
  Velocity_L = 0;    //读取左轮编码器数据，并清零，这就是通过M法测速（单位时间内的脉冲数）得到速度。
  Velocity_Right = Velocity_R;
  Velocity_R = 0;    //读取右轮编码器数据，并清零
  Kinematic_Analysis(Velocity, Angle);
  MotorR = Incremental_PI_Right(Velocity_Right, TargetR);  //===速度PI控制器
  MotorL = Incremental_PI_Left(Velocity_Left, TargetL);  //===速度PI控制器   //
  if (Turn_Off() == 0)
    Driver(MotorR, MotorL);  //如果不存在异常，使能电机

  Temp = analogRead(0);  //采集一下电池电压      //A0端口 模拟读取就是0端口
  Voltage_Count++;  //平均值计数器
  Voltage_All += Temp;  //多次采样累积
  if (Voltage_Count == 200) {
    Battery_Voltage = Voltage_All * 0.05371 / 2;  //求平均值    单位：10mV
    Voltage_All = 0;
    Voltage_Count = 0;
  }
}

/***************函数功能：遥控**********/
void start(void)
{
  Velocity = 20;    //
  Angle = 0;    //
  Flag_Stop = 0;
  Battery_Voltage = 800;
  kp = serialKp / 100;
  distance_Front = MAXDIS;
  numStop = 0;
}

void measureF()
{
  uint8_t trigger = getPinChangeInterruptTrigger(digitalPinToPCINT(EchoF));
  static unsigned long oldmicro;
  if (trigger == RISING) oldmicro = micros();
  if (trigger == FALLING) {
    durationF = micros() - oldmicro;
    if (durationF > MAXDIS) durationF = MAXDIS; // 2m
    newvalueF = 1;
  }
}

void measureL()
{
  uint8_t trigger = getPinChangeInterruptTrigger(digitalPinToPCINT(EchoL));
  static unsigned long oldmicro;
  if (trigger == RISING) oldmicro = micros();
  if (trigger == FALLING) {
    durationL = micros() - oldmicro;
    if (durationL > MAXDIS) durationL = MAXDIS; // 2m
    newvalueL = 1;
  }
}

/***********函数功能：初始化************/
void setup()
{
  pinMode(RightP, OUTPUT);  //电机控制引脚
  pinMode(RightN, OUTPUT);  //电机控制引脚，
  pinMode(LeftP, OUTPUT);  //电机速度控制引脚
  pinMode(LeftN, OUTPUT);  //电机速度控制引脚
  myservo.attach(SERVO);  //舵机初始化 //D9

  pinMode(ENCODER_L, INPUT);  //编码器引脚
  pinMode(DIRECTION_L, INPUT);  //编码器引脚
  pinMode(ENCODER_R, INPUT);  //编码器引脚
  pinMode(DIRECTION_R, INPUT);  //编码器引脚
  delay(200);    //延时等待初始化完成

  pinMode(TrigF, OUTPUT);
  pinMode(EchoF, INPUT);
  pinMode(TrigL, OUTPUT);
  pinMode(EchoL, INPUT);
  digitalWrite(TrigF, LOW);
  digitalWrite(TrigL, LOW);

  sei();      //全局中断开启
  attachInterrupt(0, READ_ENCODER_R, CHANGE);  //开启外部中断 编码器接口1   //中断通道0,D2引脚；中断通道1,D3引脚
  attachPCINT(20, READ_ENCODER_L, CHANGE);  //开启外部中断 编码器接口2, 4引脚
  attachPCINT(11, measureF, CHANGE);  // A3
  attachPCINT(9, measureL, CHANGE);  // A1
  MsTimer2::set(10, control);  //使用Timer2设置10ms定时中断
  //  MsTimer2::start();                //
  Serial.begin(38400);  //蓝牙控制    //开启串口
}

/******函数功能：主循环程序体*******/
void loop()
{
  long timer;
  if (stringComplete == true) {
    stringComplete = false;
    MsTimer2::start();  //
    start();
    timer = millis();
    Serial.print(comdata);
    Serial.print("KP:");
    Serial.println(kp);
    while (numStop < 3) {
      delay(100);
    }
    Flag_Stop = 1;
    Serial.print("End: ");
    Serial.print(distance_Front);
    Serial.print("cm, ");
    Serial.print(millis() - timer);  //
    Serial.println("ms");
  }
  delay(20);
  MsTimer2::stop();  //
}

/*****函数功能：外部中断读取编码器数据，具有二倍频功能 注意外部中断是跳变沿触发********/
void READ_ENCODER_L()
{
  if (digitalRead(ENCODER_L) == LOW)  //如果是下降沿触发的中断
  {
    if (digitalRead(DIRECTION_L) == LOW)
      Velocity_L--;  //根据另外一相电平判定方向
    else
      Velocity_L++;
  } else      //如果是上升沿触发的中断
  {
    if (digitalRead(DIRECTION_L) == LOW)
      Velocity_L++;  //根据另外一相电平判定方向
    else
      Velocity_L--;
  }
}

/*****函数功能：外部中断读取编码器数据，具有二倍频功能 注意外部中断是跳变沿触发********/
void READ_ENCODER_R()
{
  if (digitalRead(ENCODER_R) == LOW)  //如果是下降沿触发的中断
  {
    if (digitalRead(DIRECTION_R) == LOW)
      Velocity_R++;  //根据另外一相电平判定方向
    else
      Velocity_R--;
  } else      //如果是上升沿触发的中断
  {
    if (digitalRead(DIRECTION_R) == LOW)
      Velocity_R--;  //根据另外一相电平判定方向
    else
      Velocity_R++;
  }
}

/****函数功能：串口接收中断******/
void serialEvent()
{
  int data, k = 0, sign = 1;
  char inChar;
  int SerialStart = 0;
  comdata = "";
  SerialStart = Serial.available();
  while (SerialStart > 0) {
    inChar = Serial.read();  // get the new byte:
    comdata += inChar;
    delay(2);
    data = int (inChar - 48);
    if (data >= 0 && data <= 9)  //数据格式：关键字+数据，例如P60,即serialKp = 60；
    {
      switch (k) {
      case 0:
        break;
      case 1:
        leftP = leftP * 10 + data * sign;
        break;
      case 2:
        leftI = leftI * 10 + data * sign;
        break;
      case 3:
        leftD = leftD * 10 + data * sign;
        break;
      default:
        break;
        //可以任意定义其他传送通道
      }
    }
    switch (inChar) {
    case 'P':
      k = 1;
      leftP = 0;
      sign = 1;
      break;
    case 'I':
      k = 2;
      leftI = 0;
      sign = 1;
      break;
    case 'D':
      k = 3;
      leftD = 0;
      sign = 1;
      break;
    case '-':
      sign = -1;
      break;
    case '\n':
      if (leftP != 0)
        stringComplete = true;
      SerialStart = Serial.available();
      break;
    default:
      break;
      //可以任意定义其他传送通道
    }
  }
}

