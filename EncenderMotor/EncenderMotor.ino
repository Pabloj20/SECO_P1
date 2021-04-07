#include "Arduino.h"
#define MCK 84000000
#include <DueTimer.h>
#define PI 3.1415926535897932384626433832795
//int pin= 35;
//int pin= 37;
int motor1 = 4;
int motor2 = 5;
int en= 2;
//const int freq1 = (1 << 23);
//const int freq2 = 31250;
const int pin1 = 35;
const int pin2 = 37;
const int factdiv = 4200;//84MHz/20Khz
const int pulsos_vuelta= 3590;// Apartado 3
const float posicion_final=PI;
const double Kp= 12;
float dutycycle1=0;//factdiv-(50%(dutycycle)*factdiv/100);
float dutycycle2=4200;
float dutycycle=0;
const int clock_samp=42000000;
int encoderA = 3;
int encoderB = 7;
int contA=0;
int contB=0;
int cont=0;
int cont_aux=0;
int posA=0;
int posB=0;
int pulses[1200];
int muestras=0;
float error=0;
float error_aux=0;
float radianes=0;
float voltaje=0;
float porcent=0;






void setup() {
  //pinMode(motor1, OUTPUT);
  Serial.begin(115200);
  Serial.println("Inicio de programa");
 // pinMode(motor2, OUTPUT);  
  pinMode(en,OUTPUT);
  //pinMode(35,OUTPUT);
  analogWriteResolution(12); 
  //setvoltage(); 
  attachInterrupt(encoderA,handler,CHANGE);
  attachInterrupt(encoderB,handler,CHANGE);
  //digitalWrite(motor2,LOW); 
  digitalWrite(en,HIGH);
  Timer3.attachInterrupt(controlador).setPeriod(1000).start();
}

void loop() {
  
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
    //Serial.println(cont);
    cont_aux=abs(cont);
    rad_pulses(cont_aux); 
}


void setvoltage(){
  analogWriteResolution(12);
  pmc_enable_periph_clk(PWM_INTERFACE_ID);
  //Configure Clocks
  PWMC_ConfigureClocks(clock_samp,0,MCK);
  //Configure Pin
  PIO_Configure(g_APinDescription[pin2].pPort,
  PIO_PERIPH_B,
  g_APinDescription[pin2].ulPin,
  g_APinDescription[pin2].ulPinConfiguration);
  //Set Pin to count off CLKA set at freq1
  int chanA = g_APinDescription[pin2].ulPWMChannel;
  PWMC_ConfigureChannel(PWM_INTERFACE,1,0,0,0);
  PWMC_SetPeriod(PWM_INTERFACE,1,factdiv);
  PWMC_SetDutyCycle(PWM_INTERFACE,1,dutycycle2);
  PWMC_EnableChannel(PWM_INTERFACE,1);


  PIO_Configure(g_APinDescription[pin1].pPort,
  PIO_PERIPH_B,
  g_APinDescription[pin1].ulPin,
  g_APinDescription[pin1].ulPinConfiguration);
  //Set Pin to count off CLKA set at freq1
  int chanB = g_APinDescription[pin1].ulPWMChannel;
  PWMC_ConfigureChannel(PWM_INTERFACE,0,0,0,0);
  PWMC_SetPeriod(PWM_INTERFACE,0,factdiv);
  PWMC_SetDutyCycle(PWM_INTERFACE,0,dutycycle1);
  PWMC_EnableChannel(PWM_INTERFACE,0);
  
}

void controlador(){
   /* muestras++;
    pulses[muestras-1]=cont;
    //Serial.print(muestras);
    //Serial.print("\t");    
    Serial.println(pulses[muestras-1]);
    if(muestras==600){
      //digitalWrite(en,LOW);
      dutycycle2=4200;
      setvoltage();
    }else if(muestras==1200){     
      Timer3.stop();
      //Serial.println(pulses[1199]);
      Serial.println("Fin de programa\n");
    }*/
    error= posicion_final-radianes;
    error_aux=abs(error);
    voltaje= error_aux*Kp;
    porcentaje(voltaje);
    duty(porcent);
    setvoltage();
    
    
}

float rad_pulses(int contador){
  
  radianes= (2*PI*contador)/pulsos_vuelta;  
  return radianes;
  
}

float porcentaje(float voltaje){
   porcent= (voltaje/12)*100;
  return porcent;
}

void duty(float porcent){
  dutycycle=factdiv-((porcent*factdiv)/100);
  if(error<0){
    dutycycle1=4200;
    dutycycle2=dutycycle;
  }else{
    dutycycle2=4200;
    dutycycle1=dutycycle;
  }
    
 
}
  
  
