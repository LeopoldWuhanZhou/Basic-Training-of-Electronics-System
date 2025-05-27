/*
  转速控制程序
  通过串口发送速度设定值，数据格式：关键字+数据（T40,表示速度设置为40）
  需要注意的细节：定时器2的使用会影响Pin3和Pin11的PWM输出；同时定时器2中断会影响串口接收数据，故在串口接收之前，先暂停定时器2中断。
*/
#include <MsTimer2.h>             //定时中断头文件
#include <PinChangeInterrupt.h>   //外部中断头文件
//#include <PinChangeInt.h>       //讲义中示例的外部中断头文件

//*******TB6612电机驱动引脚*******//
#define RightN  3   //    AIN1  //由于定时器2的使用，Pin3和Pin11无法输出PWM，只能作为数字引脚
#define RightP  6   //    AIN2
#define LeftN   11  //    BIN1  //由于定时器2的使用，Pin3和Pin11无法输出PWM，只能作为数字引脚
#define LeftP   5   //    BIN2

//*****编码器引脚*****//
#define ENCODER_L   4         //左轮编码器采集引脚，外部中断触发源端口
#define DIRECTION_L 8
#define ENCODER_R   2         //右轮编码器采集引脚，外部中断触发源端口
#define DIRECTION_R 7

//*****全局变量*****//
volatile long Counter_Left, Counter_Right ; //左右轮编码器计数器
int Velocity_Left, Velocity_Right;          //左右轮速度
int Velocity_Target;                        //左右轮目标速度
float Velocity_KP = 4, Velocity_KI = 0.5;   //小车速度控制的PI参数
int  maxPwm = 250, maxVel = 47;             //maxVel = 47：10ms定时器中断时，最大转速为47
int MotorR, MotorL;                         //左右轮电机驱动电压

int Battery_Voltage;                  //电池电压采样变量
boolean stringComplete = false;       // 蓝牙串口接收命令完成指示
String comdata;

/**************************************************************************
  函数功能：赋值给PWM寄存器,小车的驱动函数
  入口参数：左右轮电机驱动电压，PWM格式输出
**************************************************************************/
void Driver(int motora, int motorb)   //motora:右轮；motorb:左轮
{
  if (motora > maxPwm) motora = maxPwm;
  if (motora < -maxPwm) motora = -maxPwm;
  if (motorb > maxPwm) motorb = maxPwm;
  if (motorb < -maxPwm) motorb = -maxPwm;

  if (motora > 0) analogWrite(RightP, motora), digitalWrite(RightN, LOW); //赋值给PWM寄存器
  else            analogWrite(RightP, 255 + motora), digitalWrite(RightN, HIGH); //赋值给PWM寄存器
  if (motorb > 0) analogWrite(LeftP, motorb), digitalWrite(LeftN, LOW); //赋值给PWM寄存器
  else            analogWrite(LeftP, 255 + motorb), digitalWrite(LeftN, HIGH); //赋值给PWM寄存器
}
/**************************************************************************
  函数功能：增量PI控制器，根据当前速度和目标速度的差值，算出车轮的合适驱动电压
  入口参数：编码器测量值-Encoder，目标速度-Target
  返回  值：电机驱动电压PWM
**************************************************************************/
int Incremental_PI_Right (int Encoder, int Target)
{
  static float Bias_Right, Pwm_Right, Last_bias_Right;
  Bias_Right = Encoder - Target;                              //计算偏差
  Pwm_Right -= Velocity_KP * (Bias_Right - Last_bias_Right) + Velocity_KI * Bias_Right; //增量式PI控制器
  Last_bias_Right = Bias_Right;                               //保存上一次偏差
  if (Pwm_Right > maxPwm) Pwm_Right = maxPwm;
  if (Pwm_Right < - maxPwm) Pwm_Right = -maxPwm;
  return Pwm_Right;                                           //增量输出
}
int Incremental_PI_Left (int Encoder, int Target)
{
  static float Bias_Left, Pwm_Left, Last_bias_Left;
  Bias_Left = Encoder - Target;                                   //计算偏差
  Pwm_Left -= Velocity_KP * (Bias_Left - Last_bias_Left) + Velocity_KI * Bias_Left; //增量式PI控制器
  Last_bias_Left = Bias_Left;                                     //保存上一次偏差
  if (Pwm_Left > maxPwm) Pwm_Left = maxPwm;
  if (Pwm_Left < - maxPwm) Pwm_Left = -maxPwm;
  return Pwm_Left;                                                //增量输出
}
/*********函数功能：10ms定时器中断服务函数********/
void control()
{
  Velocity_Left = Counter_Left; Counter_Left = 0;                 //读取左轮编码器数据，并清零
  /*需要补充*/    //读取右轮编码器数据，并清零
  MotorR = Incremental_PI_Right(Velocity_Right, Velocity_Target); //速度PI控制器，根据右轮实际速度，和目标速度，算出右轮驱动电压
   /*需要补充*/   //速度PI控制器，根据左轮实际速度，和目标速度，算出左轮驱动电压
  Driver(MotorR, MotorL); //驱动两后轮
  Battery_Detect();       //电池监测
}
/*********函数功能：监测当前电池电压，单位10mV*******/
/******Battery_Voltage<740时，表明电池电压不够*******/
void Battery_Detect()
{
  int Temp;                                     //临时变量
  static float Voltage_All;                     //电压采样相关变量
  static unsigned char Voltage_Count;           //电压采集用的计数器
  Temp = analogRead(0);   //采集一下电池电压    //A0端口 模拟读取就是0端口
  Voltage_Count++;        //平均值计数器
  Voltage_All += Temp;    //多次采样累积
  if (Voltage_Count == 20)
  {
    Battery_Voltage = Voltage_All * 0.5371 / 2; //求平均值，单位：10mV
    Voltage_All = 0;
    Voltage_Count = 0;
  }
}
/*****函数功能：外部中断读取编码器数据，具有二倍频功能 注意外部中断是跳变沿触发********/
void READ_ENCODER_L()
{
  if (digitalRead(ENCODER_L) == LOW)  //如果是下降沿触发的中断
  {
    if (digitalRead(DIRECTION_L) == LOW)      Counter_Left--;  //根据另外一相电平判定方向
    else      Counter_Left++;
  }
  else                                //如果是上升沿触发的中断
  {
    if (digitalRead(DIRECTION_L) == LOW)      Counter_Left++; //根据另外一相电平判定方向
    else     Counter_Left--;
  }
}
/*****函数功能：外部中断读取编码器数据，具有二倍频功能 注意外部中断是跳变沿触发********/
void READ_ENCODER_R()
{
  if (digitalRead(ENCODER_R) == LOW) //如果是下降沿触发的中断
  {
    if (digitalRead(DIRECTION_R) == LOW)      Counter_Right++;//根据另外一相电平判定方向
    else      Counter_Right--;
  }
  else                              //如果是上升沿触发的中断
  {
    if (digitalRead(DIRECTION_R) == LOW)      Counter_Right--; //根据另外一相电平判定方向
    else     Counter_Right++;
  }
}

/***********函数功能：初始化 相当于Main函数************/
void setup()
{
  pinMode(RightP, OUTPUT);          //电机驱动引脚
  pinMode(RightN, OUTPUT);          //电机驱动引脚
  pinMode(LeftP, OUTPUT);           //电机驱动引脚
  pinMode(LeftN, OUTPUT);           //电机驱动引脚

  pinMode(ENCODER_L, INPUT);        //编码器引脚
  pinMode(DIRECTION_L, INPUT);      //编码器引脚
  pinMode(ENCODER_R, INPUT);        //编码器引脚
  pinMode(DIRECTION_R, INPUT);      //编码器引脚
  delay(200);                       //延时等待初始化完成

  sei();                            //全局中断开启
  attachInterrupt(0, READ_ENCODER_R, CHANGE);             //开启外部中断 右轮编码器    //系统库函数，中断通道0，代表D2引脚；中断通道1,代表D3引脚
  attachPCINT(20, READ_ENCODER_L, CHANGE);                //开启外部中断 左轮编码器    //自编库函数，中断源为20，对应D4引脚，
  //attachPinChangeInterrupt(4, READ_ENCODER_L, CHANGE);  //讲义中示例采用的中断函数，中断源为4，对应D4引脚，对应的库函数头文件为<PinChangeInt.h>

  MsTimer2::set(10, control);       //使用Timer2设置10ms定时中断

  Serial.begin(38400);             //开启串口
}
/******函数功能：主循环程序体*******/
void loop()
{
  if (stringComplete == true)
  {
    stringComplete = false;
    Serial.print("Command: " + comdata);
  }
  MsTimer2::start();          //开启定时器中断

  Serial.print("Target: "); Serial.print(Velocity_Target);      //
  Serial.print(";  ");
  Serial.print("Left: "); Serial.print(Velocity_Left);      //
  Serial.print(";  ");
  Serial.print("Right: "); Serial.print(Velocity_Right);    //
  Serial.print(";  ");
  Serial.print("Battery_Voltage: "); Serial.println(Battery_Voltage);    //
  delay(500);

  MsTimer2::stop();           //需关闭定时器中断，因为影响串口接收
}

/****函数功能：串口接收中断******/
void serialEvent()
{
  int data, k = 0, sign = 1;
  char inChar;
  int SerialStart = 0;
  comdata = "";
  SerialStart = Serial.available();
  while (SerialStart > 0)
  {
    inChar = Serial.read();       // get the new byte:
    comdata += inChar;
    delay(2);
    data = int(inChar - 48);
    if (data >= 0 && data <= 9)   //数据格式：关键字+数据，例如T30,表示Velocity_Target = 30；
    {
      switch (k)
      {
        case 0: break;
        case 1: Velocity_Target = Velocity_Target * 10 + data * sign; break;//T: -45~45  
        case 2: Velocity_KP = Velocity_KP * 10 + data * sign; break;        //P: 0~100
        case 3: Velocity_KI = Velocity_KI * 10 + data * sign; break;        //I: 0~100
        //……      //可以任意定义其他传送通道
        default: break;
      }
    }
    switch (inChar)
    {
      case 'T': k = 1; Velocity_Target = 0; sign = 1; break;
      case 'P': k = 2; Velocity_KP = 0; sign = 1; break;
      case 'I': k = 3; Velocity_KI = 0; sign = 1; break;
      //……      //可以任意定义其他传送通道
      case '-': sign = -1; break;
      case '\n': stringComplete = true; SerialStart = Serial.available(); Velocity_KP /= 10; Velocity_KI /= 10; break;
      default: break;
    }
  }
}


