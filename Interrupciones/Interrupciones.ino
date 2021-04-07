#include <DueTimer.h>
int led= 13;
int interrupt2= 3;
volatile byte val;


void setup() {
  // put your setup code here, to run once:
  //pinMode(5,OUTPUT);
  pinMode(led,OUTPUT);
  pinMode(interrupt2,INPUT);
  attachInterrupt(interrupt2,function,CHANGE);
  val=LOW;
  Timer3.attachInterrupt(handler).start(1000);
}

void loop() {
  // put your main code here, to run repeatedly:
  //digitalWrite(5,HIGH);
  //delay(1000);
  //digitalWrite(5,LOW);
  //delay(1000);
} 

void function(){
  digitalWrite(led,val);
  val=!val;
}

void handler(){
  Timer3.stop();  

}
