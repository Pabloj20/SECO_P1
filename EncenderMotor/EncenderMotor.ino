#include "Arduino.h"
#define MCK 84000000

int motor1 = 4;
int motor2 = 5;
int en= 2;
//const int freq1 = (1 << 23);
//const int freq2 = 31250;
const int pin = 35;
const int factdiv = 4200;//84MHz/20Khz
const int dutycycle=0;//factdiv-(50%(dutycycle)*factdiv/100);
const int clock_samp=42000000;
int encoderA = 3;
int encoderB = 7;
int contA=0;
int contB=0;
int cont=0;
int posA=0;
int posB=0;
int pulses[];





void setup() {
  //pinMode(motor1, OUTPUT);
  Serial.begin(115200);
  pinMode(motor2, OUTPUT);  
  pinMode(en,OUTPUT);
  pinMode(35,OUTPUT);
  analogWriteResolution(12);
  pmc_enable_periph_clk(PWM_INTERFACE_ID);
  //Configure Clocks
  PWMC_ConfigureClocks(clock_samp,0,MCK);
  //Configure Pin
  PIO_Configure(g_APinDescription[pin].pPort,
  PIO_PERIPH_B,
  g_APinDescription[pin].ulPin,
  g_APinDescription[pin].ulPinConfiguration);
  //Set Pin to count off CLKA set at freq1
  int chan = g_APinDescription[pin].ulPWMChannel;
  PWMC_ConfigureChannel(PWM_INTERFACE,0,0,0,0);
  PWMC_SetPeriod(PWM_INTERFACE,0,factdiv);
  PWMC_SetDutyCycle(PWM_INTERFACE,0,dutycycle);
  PWMC_EnableChannel(PWM_INTERFACE,0);
 
  attachInterrupt(encoderA,handler,CHANGE);
  attachInterrupt(encoderB,handler,CHANGE);  
}

void loop() {
  digitalWrite(en,LOW);
  //digitalWrite(motor1,LOW);
  digitalWrite(motor2,LOW);
  
  
}

void handler(){
posA= digitalRead(encoderA);
posB= digitalRead(encoderB);

switch(posA) {
  case 1 :
    switch(posB){
      case 1:
        if(posA==contA){
          cont++; 
        }else{
          cont--;
        }
        break;
      case 0:
        if(posA!=contA){
          cont++;
        }else{
          cont--;
        }
        break;     
    }
    break;

   case 0 :
    switch(posB){
      case 1:
        if(posA!=contA){
          cont++; 
        }else{
          cont--;
        }
        break;
      case 0:
        if(posA==contA){
          cont++;
        }else{
          cont--;
        }
        break;     
    }
    break;
    }

    contA=posA;
    contB=posB;
   
     
  
}
  
  
