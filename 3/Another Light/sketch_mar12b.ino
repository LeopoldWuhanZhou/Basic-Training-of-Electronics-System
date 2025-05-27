int led1=11;
int led2=9;
int led3=3;
int b1=1,b2=1,b3=1;
void setup() {
  // put your setup code here, to run once:
   pinMode(11,OUTPUT);
   pinMode(9,OUTPUT);
   pinMode(3,OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
   int control=1;
   while(1){
    b1=b1*7;
    b2=b2*9;
    b3=b3*13;
    analogWrite(led1,10);
    delay(1000);
    analogWrite(led2,100);
    delay(1000);
    analogWrite(led3,200);
    delay(1000);
   }
   delay(30);
}
