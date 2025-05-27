#include <MPU6050_tockn.h>
#include <Wire.h>

MPU6050 mpu6050(Wire);
long timer = 0;

int AccX, AccY, AccZ;
void setup()
{
  Serial.begin(38400);
  while (!Serial);
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
  Serial.println("");
  timer = millis();
}
void loop()
{
  mpu6050.update();
  AccX = int(mpu6050.getAccX() * 100.0);
  AccY = int(mpu6050.getAccY() * 100.0);
  AccZ = int(mpu6050.getAccZ() * 100.0);

  if ((millis() - timer) > 1000)
  {
    Serial.print("accX : "); Serial.print(AccX);
    Serial.print("\taccY : "); Serial.print(AccY);
    Serial.print("\taccZ : "); Serial.println(AccZ);

    timer = millis();
  }
  delay(10);
}
