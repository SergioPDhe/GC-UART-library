#include "GC_COMM.h"

GCComm gcComm;

uint8_t test = 128;
uint8_t test2 = 0b00000001;
void setup() 
{
  gcComm.InitDataLine();  
  DDRB = 0b00111100;
  PORTB = 0b00111100;
  delay(500);
  PORTB = 0;
}



void loop() 
{
  gcComm.ReceiveCommand();
  //gcComm.SendPollResponse();
  //delay(2000);
  gcComm.ControlX = test;
  gcComm.ControlY = test+64;
  gcComm.StYXBA = test2;
  test += 1;
  test2 = test2<<1;
  if (test2==16) test2 = 1;

  if (gcComm.rumble) PORTB = (1<<3);
  else PORTB = 0;
}
