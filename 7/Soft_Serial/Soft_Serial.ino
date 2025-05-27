#include <SoftwareSerial.h>
#define SS_RX 4
#define SS_TX 5
//+ADDR:98d3:31:fd72a1
//实例化软串口
SoftwareSerial BT_Serial(SS_RX, SS_TX); // RX, TX
String comdata = "";

void setup()
{
  pinMode(SS_RX, INPUT);
  pinMode(SS_TX, OUTPUT);
  Serial.begin(38400);
  while (!Serial);
  Serial.println("Hardware serial port begins @ 0, 1 (RX, TX), connecting USB serial.");

  BT_Serial.begin(38400);
  Serial.println("Software serial port begins @ " + String(SS_RX) + ", " + \
  String(SS_TX) + " (RX, TX), connecting your device."); 
}

void loop()
{
  while (Serial.available()){
    comdata += char(Serial.read());
    delay(2);
  }
  if (comdata.length()){
    Serial.print(comdata); //回显
    BT_Serial.print(comdata);
    comdata = "";
  }
  
  while (BT_Serial.available()){
    comdata += char(BT_Serial.read());
    delay(2);
  }
  if (comdata.length()){
    Serial.print(comdata);
    comdata = "";
  }
}
