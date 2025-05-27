int analogPin = A0;
int value;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  value =  analogRead(analogPin);
  Serial.println(value);
  if(value > 500 )
  {
    digitalWrite(4, HIGH);
  }
  else
  {
    digitalWrite(4, LOW);
  }
}
