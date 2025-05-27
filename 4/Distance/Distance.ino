int analogPin = A0;
int value, pause;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(8, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  value =  analogRead(analogPin);
  pause = value/2;
  Serial.println(value);
  tone(8, 262, pause);
  delay(1.3*pause);
  noTone(8);
}
