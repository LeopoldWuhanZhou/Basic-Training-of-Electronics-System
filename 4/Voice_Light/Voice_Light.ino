int digitalPin = 8;
int value;
void setup() {
  // put your setup code here, to run once:
  pinMode(digitalPin,INPUT);
  pinMode(4, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  value =  digitalRead(digitalPin);
  if (value == HIGH)
  {
    digitalWrite (4, HIGH);
  }
  if (value == LOW)
  {
    digitalWrite (4, LOW);
  }
  delay(1000);
}
