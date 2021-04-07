#include "Arduino.h"
#define MCK 84000000

//const int freq1 = (1 << 23);
//const int freq2 = 31250;
const int pin = 35;
const int factdiv = 4200;//84MHz/20Khz
const int dutycycle=2100;//factdiv-(50%(dutycycle)*factdiv/100);
const int clock_samp=42000000;

void setup()
{
  // put your setup code here, to run once:
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
 
  
}

void loop()
{
  // put your main code here, to run repeatedly:

}
