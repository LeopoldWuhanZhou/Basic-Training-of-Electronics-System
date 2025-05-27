#include <SoftwareSerial.h>
#include <MPU6050_tockn.h>
#include <Wire.h>

//实例化软串口
#define SS_RX 4
#define SS_TX 5
SoftwareSerial BT_Serial(SS_RX, SS_TX); // RX, TX
String comdata = "";

MPU6050 mpu6050(Wire);
int AccX, AccY, AccZ;

void setup()
{
  pinMode(SS_RX, INPUT);
  pinMode(SS_TX, OUTPUT);

  Serial.begin(38400);
  while (!Serial);

  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);

  Serial.println("");
  Serial.println("Bluetooth-remoted car starting...");

  BT_Serial.begin(38400);
  while (!BT_Serial);
}
void loop()
{
  mpu6050.update();

  //小车速度分为5档，大约倾斜45°时，达到满档；
  //再倾斜，档位保持满档；
  //设置倾斜90度时，加速度值为7，45°时，sin45°≈5/7；
  AccX = int(mpu6050.getAccX() * 5);
  AccY = int(mpu6050.getAccY() * 5);
  AccZ = int(mpu6050.getAccZ() * 5);
  if (AccX > 5) AccX = 5;   //5档
  else if (AccX < -5) AccX = -5;
  if (AccY > 5) AccY = 5;
  else if (AccY < -5) AccY = -5;
  if (AccZ > 5) AccZ = 5;
  else if (AccZ < -5) AccZ = -5;

  //把三维加速度信息组合成一条字符串，方便蓝牙串口传送;
  //格式: X/AccX/Y/AccY/Z/AccZ;
  comdata = "X";
  comdata += char('0' + AccX);  //转换成字符之后，不管AccX是正是负，都只占一个字节
  comdata += "Y";
  comdata += char('0' + AccY);  //
  comdata += "Z";
  comdata += char('0' + AccZ);  //


  //发送给电脑串口，用于显示
  Serial.print("accX : "); Serial.print(AccX);
  Serial.print("\taccY : "); Serial.print(AccY);
  Serial.print("\taccZ : "); Serial.println(AccZ);
  Serial.println(comdata);

  //发送给蓝牙串口，用于遥控
  BT_Serial.println(comdata);
  delay(100);

  //通过蓝牙接收小车的回传信息
  comdata = "return data: ";  //
  while (BT_Serial.available())
  {
    comdata += char(BT_Serial.read());
    delay(2);
  }
  //通过电脑串口显示小车的回传信息
  if (comdata.length()>13)  //"return data: "为13字节
  {
    Serial.println(comdata);
    comdata = "";
    delay(10);
  }
}
