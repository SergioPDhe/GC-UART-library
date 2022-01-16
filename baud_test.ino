#include "GC_COMM.h"

GCComm gcComm;

uint8_t test = 128;
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
  test++;
}
