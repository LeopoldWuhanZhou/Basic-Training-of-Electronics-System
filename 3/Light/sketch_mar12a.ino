int ledPins[3] = {11, 9, 3};
int ledCycles[3] = {50, 100, 210};
void setup() {
  // put your setup code here, to run once:
   pinMode(11,OUTPUT);
   pinMode(9,OUTPUT);
   pinMode(3,OUTPUT);   
}

void loop() {
  // put your main code here, to run repeatedly:
  while(1){
      analogWrite(ledPins[0],ledCycles[0]);
      delay(1000);
      analogWrite(ledPins[1],ledCycles[1]);
      delay(1000);
      analogWrite(ledPins[3],ledCycles[2]);
      delay(1000);

  }
  delay(2000);
}
