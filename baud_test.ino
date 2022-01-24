#include "GC_COMM.h"

GCComm gcComm;

uint8_t test = 128;
uint8_t test2 = 0b00000001;
int count = 0;
void setup() 
{
  gcComm.InitDataLine();
    
  DDRB = 0b00111100;
  DDRF = 0;
  DDRA = 0;
  PORTA = 0b11111111;
  PORTF = 0b11111111;
  //delay(500);
  PORTB = 0;
  Serial.begin(115200);
  Serial.println(123);
}



void loop() 
{
  /*
  gcComm.ReceiveCommand();
  //gcComm.SendPollResponse();
  //delay(2000);
  gcComm.ControlX = test;
  gcComm.ControlY = test+64;
  gcComm.StYXBA = test2;
  test += 1;
  test2 = test2<<1;
  if (test2==16) test2 = 1;

  //if (gcComm.rumble) PORTB = (1<<3);
  //else PORTB = 0;*/

  //delay(1000);
  gcComm.StYXBA = ~PINA & 0b00011111;
  gcComm.LRZDpad = ((~PINA & 0b11100000)>>1)| 0b10000000;

  if (~PINF & 0b00000001) gcComm.ControlX = 0xFF;
  else if (~PINF & 0b00000010) gcComm.ControlX = 0;
  else gcComm.ControlX = 0x80; 

  if (~PINF & 0b00000100) gcComm.ControlY = 0xFF;
  else if (~PINF & 0b00001000) gcComm.ControlY = 0;
  else gcComm.ControlY = 0x80; 

  count++;
}

ISR(INT0_vect)
{

  //PORTF = (1<<1);
  //Serial.println(333);

  gcComm.ReceiveCommand();
  //gcComm.SendPollResponse();
  //delay(2000);
  //gcComm.ControlX = test;
  //gcComm.ControlY = test+64;
  //gcComm.StYXBA = test2;
  //test += 1;
  //test2 = test2<<1;
  //if (test2==16) test2 = 1;

  //if (gcComm.rumble) PORTB = (1<<5);
  //else PORTB = 0;

  if(gcComm.rumble != 0) Serial.println(gcComm.rumble);
  count=0;
}
