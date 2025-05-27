#include <Ultrasonic.h>
Ultrasonic ultra_Front(18, 17);  // (Trig PIN, Echo Pin)
Ultrasonic ultra_Rear(16, 15);   // (Trig PIN, Echo Pin)
#define distanceBiasFront 10      //前置超声波传感器距离小车前沿的距离，单位cm
#define distanceBiasRear  5       //后置超声波传感器距离小车后沿的距离，单位cm

/////////TB6612驱动引脚////       //小车后轮驱动
#define RightN 3  //    AIN1
#define RightP 6  //    AIN2
#define LeftN 11  //    BIN1 
#define LeftP 5   //    BIN2

int MotorR, MotorL;               //左右轮驱动电压
int  maxPwm = 250;                //左右轮最大驱动电压（不能超过255）
int Battery_Voltage;              //电池电压采样变量
int voltage;                      //电池电压裕度：voltage = Battery_Voltage-740，单位10mV
int distance_Front, distance_Rear, limit = 30; //超声波模块的输出以及避障距离
int forward, turn, gravity;       //遥控器传输过来的三维数据：前进、转弯、重力方向
unsigned char Flag_Stop = 0;      //停止标志位
boolean stringComplete = false;   // 蓝牙串口接收命令完成指示
String comdata;                   //

/**************************************************************************
  函数功能：赋值给PWM寄存器，驱动小车两个后轮
  入口参数：两个PWM值，参数大于0，前进；参数小于0，后退
**************************************************************************/
void Driver(int motora, int motorb)
{
  if (motora > maxPwm) motora = maxPwm;
  else if (motora < -maxPwm) motora = -maxPwm;
  if (motorb > maxPwm) motorb = maxPwm;
  else if (motorb < -maxPwm) motorb = -maxPwm;

  if (motora > 0) analogWrite(RightP, motora), digitalWrite(RightN, LOW);   //右轮前进
  else            analogWrite(RightP, 255 + motora), digitalWrite(RightN, HIGH);  //右轮后退
  ///*
  if (motorb > 0) analogWrite(LeftP, motorb), digitalWrite(LeftN, LOW);     //左轮前进
  else            analogWrite(LeftP, 255 + motorb), digitalWrite(LeftN, HIGH);    //左轮后退
  //*/
}
/**************************************************************************
  函数功能：异常关闭电机
  入口参数：电压
  返回  值：1：异常  0：正常
 **************************************************************************/
unsigned char Turn_Off()
{
  byte temp;
  if (Flag_Stop == 1 || (Battery_Voltage < 740)) //Flag_Stop置1或者电压太低关闭电机
  {
    temp = 1;
    digitalWrite(RightP, LOW);  //电机驱动的电平控制
    digitalWrite(RightN, LOW);  //电机驱动的电平控制
    digitalWrite(LeftP, LOW);   //电机驱动的电平控制
    digitalWrite(LeftN, LOW);   //电机驱动的电平控制
  }
  else  temp = 0;
  return temp;
}

/************************************************
  函数功能：电池电量检测
  电池采样比例：1/11，采样输出值0~1023，对应0~5V
**************************************************/
void Battery_Detect()
{
  int Voltage_Temp;
  static float Voltage_All;
  static unsigned char Voltage_Count;
  Voltage_Temp = analogRead(0);   //采集一次电池电压    //A0端口 模拟读取就是0端口
  Voltage_Count++;                //计数器
  Voltage_All += Voltage_Temp;    //多次采样累积
  if (Voltage_Count == 10)        //累计采样10次
  {                               //电池采样比例：1/11，（0~1023—>0~5V）
    Battery_Voltage = Voltage_All * 0.5371;  //求平均值，单位：10mV
    Voltage_All = 0;
    Voltage_Count = 0;
    voltage = Battery_Voltage - 740;  //单位10mV
    if (voltage > 100) voltage = 100;
    if (voltage < 0) voltage = 0;
  }
}
/***************函数功能：设定速度与转弯**********/
void setAcc(void)
{
  MotorR = forward * 60;  //根据遥控器程序协议，forward取值为-5~5
  MotorL = forward * 60;  //当MotorR和MotorL相等时，小车为直行状态
                          //小车的转弯通过设置后驱两轮不同速度实现
  if (turn > 0) MotorL = MotorL /turn/5;       //左转  //根据遥控器程序协议，turn取值为-5~5
  else  if (turn < 0) MotorR = MotorR /turn/5; //右转  //即两后轮最大差速为250-125=125
}
/************************函数功能：避障设置***********************
  根据前/后两个超声波测距模块，进行避障设置，遇到障碍，停止行进
*****************************************************************/
void avoidObstacle()
{
  Flag_Stop = 0;
  distance_Front = int(ultra_Front.getDistanceInCM()) - distanceBiasFront;
  distance_Rear = int(ultra_Rear.getDistanceInCM()) - distanceBiasRear;
  if (forward >= 0 && distance_Front < limit) 
  {
    Flag_Stop = 1;    //前方路障，小车停止
    Serial.print("front");
  }
  if (forward <= 0 && distance_Rear < limit)  
  {
    Flag_Stop = 1;    //后方路障，小车停止
    Serial.print("back");
  }
}

void setup()
{
  pinMode(RightP, OUTPUT);         //右轮电机控制引脚
  pinMode(RightN, OUTPUT);         //右轮电机控制引脚
  pinMode(LeftP, OUTPUT);          //左轮电机控制引脚
  pinMode(LeftN, OUTPUT);          //左轮电机控制引脚
  delay(200);                      //延时等待初始化完成
  Serial.begin(38400);            //蓝牙控制    //开启串口
}
/******函数功能：主循环程序体*******/
void loop()
{
  Battery_Detect();   //检测电池电压是否充足
  avoidObstacle();    //避障措施
  setAcc();           //设定后两轮的速度
  if (Turn_Off() == 0) Driver(MotorR, MotorL); //如果不存在异常，使能电机
  if (stringComplete == true)//回传数据内容可以自行定义
  {
    stringComplete = false;
    Serial.print(comdata);  //回传接收的数据

    Serial.print("{");      //回传数据格式：{voltage|distance_Front-distance_Rear|forward-turn}
    Serial.print(voltage);  //电池电压裕度
    Serial.print("|");
    Serial.print(distance_Front); Serial.print("-"); Serial.print(distance_Rear);
    Serial.print("|");
    Serial.print(forward); Serial.print("-"); Serial.print(turn);    //直行指令 //转弯指令
    Serial.println("}");
  }
}
/**************函数功能：串口接收中断***************
  程序执行完loop()之后，自动执行serialEvent()
  然后继续回到loop()， 如此循环
****************************************************/
void serialEvent()
{
  int data, k = 0;
  char inChar;
  boolean start = 0;
  comdata = "";
  if (Serial.available()) start = 1;  //当串口接收缓存里有数据时，进入循环
  while (start)
  {
    while (!Serial.available());      //
    inChar = char(Serial.read());     //读取一个字节数据，并把这个字节从串口接收缓存里删除
    comdata += inChar;                //
    delay(2);
    data = int(inChar - '0');         //根据遥控器程序的协议，把传送的字符数据转换成整型数据
    switch (k)                        //可以理解为通信信道，目前只定义了三路，分别对应三维加速度数据（前进、转弯、重力方向）
    {
      case 0: break;
      case 1: forward = data; break;
      case 2: turn = data; break;
      case 3: gravity = data; break;
    }
    k = 0;
    switch (inChar)                   //三路信道的识别关键字
    {
      case 'X': k = 1; break;
      case 'Y': k = 2; break;
      case 'Z': k = 3; break;
      case '\n': start = 0;  break;   //每一条信息传送结束标识符
    }
    stringComplete = true;     
  }
}
