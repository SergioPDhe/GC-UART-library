#include "GC_COMM.h"

GCComm gcComm;

uint8_t test = 128;
void setup() 
{
  gcComm.InitDataLine();  
  DDRB = (1<<5);
}



void loop() 
{
  gcComm.ReceiveCommand();
  gcComm.ControlX = test;
  gcComm.ControlY = test+64;
  test++;
}
