/*
  定点停车程序——不带速度控制
  通过串口发送数据即可启动程序，数据格式：关键字+数据（0~100，默认30），这里的关键字就是P，表示比例系数这个反馈参量，比如发送P30
  小车的离墙距离，通过10ms定时器进行探测，每次探测完，重新设定小车的行进驱动电压
  需要注意的细节：定时器2的使用会影响Pin3和Pin11的PWM输出；同时定时器2中断会影响串口接收数据，故在串口接收之前，先暂停定时器2中断.
*/
#include <MsTimer2.h>          //定时中断
#include <Ultrasonic.h>       //超声波测距

Ultrasonic ultra_Front(18, 17, 11600);  // (Trig PIN, Echo Pin) //5800:100cm
#define distanceBiasFront 5   //前置超声波探测器与小车前端距离

/////////TB6612驱动引脚////
#define RightN  3   //    AIN1  //由于定时器2的使用，Pin3和Pin11无法输出PWM，只能作为数字引脚
#define RightP  6   //    AIN2
#define LeftN   11  //    BIN1  //由于定时器2的使用，Pin3和Pin11无法输出PWM，只能作为数字引脚
#define LeftP   5   //    BIN2

/////////编码器引脚////////
#define ENCODER_L     4  //编码器采集引脚 每路2个 共4个
#define DIRECTION_L   8
#define ENCODER_R     2
#define DIRECTION_R   7

float serialKp = 30, kp;    //反馈比例系数，根据需要使用；串口发送值和实际使用值
float serialKi = 30, ki;    //反馈积分系数，根据需要使用；串口发送值和实际使用值
int Motor, maxPwm = 250;
int distance_Front, Last_distance_Front, limit = 30;

unsigned int numStop;       //用于判定小车是否停止
unsigned char Flag_Stop = 0;//停止标志位

boolean stringComplete = false; // 蓝牙串口接收命令完成指示
String comdata;
int Battery_Voltage;        //电池电压采样变量

/**************************************************************************
  函数功能：赋值给PWM寄存器,小车的驱动函数
  入口参数：PWM
**************************************************************************/
void Driver(int motora, int motorb)
{
  if (motora > maxPwm) motora = maxPwm;
  if (motora < -maxPwm) motora = -maxPwm;
  if (motorb > maxPwm) motorb = maxPwm;
  if (motorb < -maxPwm) motorb = -maxPwm;

  if (motora > 0) analogWrite(RightP, motora), digitalWrite(RightN, LOW); //赋值给PWM寄存器
  else            analogWrite(RightP, 255 + motora), digitalWrite(RightN, HIGH);  //赋值给PWM寄存器
  if (motorb > 0) analogWrite(LeftP, motorb), digitalWrite(LeftN, LOW);   //赋值给PWM寄存器
  else            analogWrite(LeftP, 255 + motorb), digitalWrite(LeftN, HIGH);    //赋值给PWM寄存器
}
/**************************************************************************
  函数功能：异常关闭电机
  入口参数：电压
  返回  值：1：异常  0：正常
  /**************************************************************************/
unsigned char  Turn_Off()
{
  byte temp;
  if (Flag_Stop == 1 || (Battery_Voltage < 700)) //Flag_Stop置1或者电压太低关闭电机
  {
    temp = 1;
    digitalWrite(RightP, LOW);  //电机驱动的电平控制
    digitalWrite(RightN, LOW);  //电机驱动的电平控制
    digitalWrite(LeftP, LOW);   //电机驱动的电平控制
    digitalWrite(LeftN, LOW);   //电机驱动的电平控制
  }
  else temp = 0;
  return temp;
}

/*********函数功能：10ms控制函数 核心代码 *******/
void control()
{
  distance_Front = int(ultra_Front.getDistanceInCM()) - distanceBiasFront - limit;
  if (distance_Front == Last_distance_Front && abs(distance_Front) < 10) numStop++; //判断小车是否停止
  else numStop = 0;                                                                 //
  Last_distance_Front = distance_Front;
  
  wallfinder();  //根据剩余距离-distance_Front，确定当前小车行进动力-Motor
  if (Turn_Off() == 0) Driver(Motor, Motor);    //如果不存在异常，使能电机
  Battery_Detect();
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

/***************函数功能：开始**********/
void start(void)
{
  Motor = maxPwm;           //全速
  Flag_Stop = 0;
  numStop = 0;
  kp = serialKp / 10;       //反馈比例系数，来源于串口指令
}
/***************函数功能：设定小车运行动力**********/
void wallfinder()           //可以自由发挥，下面仅为示例
{
  /*********需补充 ***********/        //简单的比例反馈：根据剩余距离-distance_Front，确定当前小车行进动力-Motor
  if (Motor > maxPwm) Motor = maxPwm;  //不超过最大输出动力
}
/***********函数功能：初始化************/
void setup()
{
  pinMode(RightP, OUTPUT);      //电机控制引脚
  pinMode(RightN, OUTPUT);      //电机控制引脚，
  pinMode(LeftP, OUTPUT);       //电机速度控制引脚
  pinMode(LeftN, OUTPUT);       //电机速度控制引脚

  pinMode(ENCODER_L, INPUT);       //编码器引脚，本程序没用上
  pinMode(DIRECTION_L, INPUT);     //编码器引脚，本程序没用上
  pinMode(ENCODER_R, INPUT);       //编码器引脚，本程序没用上
  pinMode(DIRECTION_R, INPUT);     //编码器引脚，本程序没用上
  delay(200);                      //延时等待初始化完成

  sei();//全局中断开启
  MsTimer2::set(10, control);      //使用Timer2设置10ms定时中断
  Serial.begin(38400);            //蓝牙控制    //开启串口
}
/******函数功能：主循环程序体*******/
void loop()
{
  long timer;
  if (stringComplete == true)
  {
    stringComplete = false;
    MsTimer2::start();      //
    start();
    timer = millis();
    Serial.print(comdata);
    Serial.println("Start: {distance|Motor|Battery}");
    while (numStop < 3)   //
    {
      Serial.print("{");
      Serial.print(distance_Front);
      Serial.print("|");
      Serial.print(Motor);
      Serial.print("|");
      Serial.print(Battery_Voltage);
      Serial.println("}");
      delay(300);
    }
    Flag_Stop = 1;
    Serial.print("End: ");
    Serial.print(distance_Front);   //回传剩余距离
    Serial.print("cm, ");
    Serial.print(millis() - timer); //回传耗时
    Serial.println("ms");
  }
  delay(20);
  MsTimer2::stop();                 //
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
    inChar = Serial.read();    // get the new byte:
    comdata += inChar;
    delay(2);
    data = int(inChar - 48);
    if (data >= 0 && data <= 9)   //数据格式：关键字+数据，例如P60,即serialKp = 60；
    {
      switch (k)
      {
        case 0: break;
        case 1: serialKp = serialKp * 10 + data * sign; break;
        case 2: serialKi = serialKi * 10 + data * sign; break;
        default: break;
          //可以任意定义其他传送通道
      }
    }
    switch (inChar)
    {
      case 'P': k = 1; serialKp = 0; sign = 1; break;
      case 'I': k = 2; serialKi = 0; sign = 1; break;
      case '-': sign = -1; break;
      case '\n': stringComplete = true; SerialStart = Serial.available(); break;
      default: break;
        //可以任意定义其他传送通道
    }
  }
}


