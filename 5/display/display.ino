#include "TM1637.h"     //添加数码管模块头文件（预先把TM1637库函数文件拷贝进arduino安装目录下的libraries文件夹里）
#include "waveData2.h"

#define CLK 3           //数码管模块接口
#define DIO 2           //数码管模块接口
TM1637 tm1637(CLK,DIO); //定义数码管对象

unsigned int signalOutPWM = 9;                  //信号输出端口
unsigned int switchFrequency = 4;
unsigned int switchAmplitude = 5;
unsigned int switchWave = 6;

unsigned int delaytime = 1950;                  //每个数据由970Hz的PWM波形滤波产生，约1.9ms延迟+其他语句延迟 约等于 2ms，足以产生两个完整PWM波形
unsigned int wholePeriodNum = 512;
unsigned int i, freqStep, Amplitude, Wave;

unsigned int waveflag;
unsigned int amplitudeflag;
unsigned int freqflag;

int8_t displayData[] = {Wave, Amplitude, freqStep / 10, freqStep % 10};  //四个数码管待显示的数据

void setup() {
  tm1637.set();       //数码管对象设置
  tm1637.init();      //数码管对象初始化
  pinMode(signalOutPWM, OUTPUT);
  pinMode(switchFrequency, INPUT_PULLUP);
  pinMode(switchAmplitude, INPUT_PULLUP);
  pinMode(switchWave, INPUT_PULLUP);
  freqStep = 5;          
  Amplitude = 1;
  Wave = 1;
}

void loop() {
  waveflag = digitalRead(switchWave);
  amplitudeflag = digitalRead(switchAmplitude);
  freqflag = digitalRead(switchFrequency);
  if(freqflag == LOW)
  {
    if(freqStep == 20)
      freqStep = 1;
    else
      freqStep ++;
  }
  if(waveflag == LOW)
  {
    if(Wave == 3)
      Wave = 1;
    else
      Wave++;
  }
  if(amplitudeflag == LOW)
  {
    if(Amplitude == 4)
      Amplitude = 1;
    else
      Amplitude ++;
  }
  displayData[0] = Wave;
  displayData[1] = Amplitude;
  displayData[2] = freqStep / 10;
  displayData[3] = freqStep % 10;
  tm1637.display(displayData);  //根据displayData数组的值，更新数码管显示
  i %= wholePeriodNum;
  if (Wave == 1) {
    analogWrite(signalOutPWM, sinData[i] * Amplitude);
  } else if (Wave == 2) {
    analogWrite(signalOutPWM, squareData[i / 256] * Amplitude);
  } else if (Wave == 3) {
    analogWrite(signalOutPWM, triangularData[i] * Amplitude);
  }
  i += freqStep;
  delayMicroseconds(delaytime);
}
