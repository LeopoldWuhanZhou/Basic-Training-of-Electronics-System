//1Hz正弦波发生器
//采用DDS方法产生波形：将一个完整正弦波离散化为512个数据，存储为数组，以一个固定频率顺序读取数组的值，即可复制波形，通过改变读取的步进量，改变输出波形频率
//采用PWM进行模拟输出，约2ms的模拟转换时间，当采用频率步长为1时，一个完整波形需要512个数据，刚好是1s左右，即1Hz
#include <TM1637.h>
#include "waveData2.h"

unsigned int signalOutPWM = 9;                  //信号输出端口
unsigned int switchFrequency = 3;
unsigned int switchAmplitude = 5;

unsigned int delaytime = 1950;                  //每个数据由970Hz的PWM波形滤波产生，约1.9ms延迟+其他语句延迟 约等于 2ms，足以产生两个完整PWM波形
unsigned int wholePeriodNum = 512;
unsigned int i, freqStep, amplitude;



void setup() {  
  pinMode(signalOutPWM, OUTPUT);
  pinMode(switchFrequency, INPUT_PULLUP);
  pinMode(switchAmplitude, INPUT_PULLUP);
  freqStep = 5;          //  
}

void loop() {
  if(switchFrequency == 0)
  {
    if(freqStep == 20)
    {
      freqStep = 0;
    }
    else
    {
      freqStep += 1;
    }
  }
  i %= wholePeriodNum;
  analogWrite(signalOutPWM, sinData[i]);
  i += freqStep;
  delayMicroseconds(delaytime);
}
