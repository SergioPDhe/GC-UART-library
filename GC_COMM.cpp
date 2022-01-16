/*
GCN protocol guides:
  https://simplecontrollers.bigcartel.com/gamecube-protocol
  http://www.int03.co.uk/crema/hardware/gamecube/gc-control.html
 */
#include "GC_COMM.h"


GCComm::GCComm()
{
  UBRR0H =  (uint8_t)0; // 
  UBRR0L =  (uint8_t)1; //// Sets baud rate to 1M (see datasheet)
  UCSR0A |= (1<<U2X0);  //
  
  UCSR0C =  (1<<UCSZ00)|(1<<UCSZ01); // change frame size to 8 bits, no parity, 1 stop bit (default value for reading GC commands)
}


void GCComm::InitDataLine() // pin activation must happen during setup
{
  UCSR0B = (1<<RXEN0)|(1<<TXEN0); // activate RX and TX pins
}



void GCComm::SendPollResponse() // responds to console poll to check if a controller is plugged in (See GCN communication protocol guides)
{
  SetFrameSize6(); // sets UART frame size to 6 (see function for details)
  
  SendByte(0x09);   // send expected poll response bytes
  SendByte(0x00);
  SendByte(0x03);
  SendStopBit();    // sends stop bit (duh)

  SetFrameSize8(); // flushes buffer and sets UART frame back to 8 (see function for details)
}


void GCComm::SendOrigin() // responds to console poll for origin (See GCN communication protocol guides)
{
  SetFrameSize6();  // sets UART frame size to 6 (see function for details)
  
  SendByte((uint8_t)0x00);     // Sends Start, Y, X, B, A and some overhead stuff, format is usually [0,0,0,St,Y,X,B,A]
  SendByte((uint8_t)0x80);     // Sends L, R, Z and Dpad inputs, format is usually [0,0,0,St,Y,X,B,A]
  
  SendByte((uint8_t)128);      // control stick inputs
  SendByte((uint8_t)128);
  
  SendByte((uint8_t)128);      // c-stick inputs
  SendByte((uint8_t)128);
  
  SendByte((uint8_t)0x00);     // analog L and R inputs
  SendByte((uint8_t)0x00);
  
  SendByte((uint8_t)0x00);     // null bytes (expected by console)
  SendByte((uint8_t)0x00);
  
  SendStopBit();               // stop bit


  SetFrameSize8(); // sets UART frame back to 8 (see function for details)
}


void GCComm::SendInputs() // sends inputs to console
{
  SetFrameSize6();
  
  SendByte(StYXBA);   // Sends Start, Y, X, B, A and some overhead stuff, format is usually [0,0,0,St,Y,X,B,A]
  SendByte(LRZDpad);  // Sends L, R, Z and Dpad inputs, format is usually [1,L,R,Z,Dup,Ddown,Dright,Dleft]
  
  SendByte(ControlX); // control stick inputs
  SendByte(ControlY);

  SendByte(CstickX);  // c-stick inputs
  SendByte(CstickY);
  
  SendByte(AnalogL);  // analog L and R inputs
  SendByte(AnalogR);
  SendStopBit();      // stop bit

  FlushReceiveBuffer(); // empties RX buffer
  
  SetFrameSize8(); // sets UART frame back to 8 (see function for details)
}

void GCComm::ReceiveCommand()
{
  uint8_t command = ReceiveByte();

  switch (command)
  {
    case 0x40:
      ReceiveByte();                      // get second command out of the way
      SetRumble(ReceiveByte()&0b00000011); // get last 2 bits of 3rd command to check for rumble
      FlushReceiveBuffer();
      SendInputs();
      PORTB |= (1<<2);
      break;
      
    case 0x00:
      FlushReceiveBuffer();
      SendPollResponse();
      PORTB |= (1<<4);
      break;
      
    case 0x41:
      FlushReceiveBuffer();
      SendOrigin();
      PORTB |= (1<<3);
      break;
      
  }
  FlushReceiveBuffer();
}

inline void GCComm::SendByte(uint8_t dataByte) // Sends a full GC byte to the console in pairs of GC bits using 4 UART bytes (REQUIRES 6-BIT FRAME)
{
  uint8_t first, second, third, last;     // split the byte into 4 pairs of bits (possible values are 00=0, 01=1, 10=2, 11=3)
  first =   (dataByte & 0b11000000) >> 6;
  second =  (dataByte & 0b00110000) >> 4;
  third =   (dataByte & 0b00001100) >> 2;
  last =    (dataByte & 0b00000011);

  first = Byte2GC(first);     // convert each pair to a corresponding byte sent by UART
  second = Byte2GC(second);
  third = Byte2GC(third);
  last = Byte2GC(last);

  SendPair(first);      // send data
  SendPair(second);
  SendPair(third);
  SendPair(last);
}

inline uint8_t GCComm::ReceiveByte() // Receives 1 full GC byte as 4 UART bytes (REQUIRES 8-BIT FRAME)
{
  uint8_t first, second, third, last;
  
  first = ReceivePair();    //receive each pair of bytes
  second = ReceivePair();
  third = ReceivePair();
  last = ReceivePair();

  first = GC2Byte(first);   //convert UART bytes into GC bits
  second = GC2Byte(second);
  third = GC2Byte(third);
  last = GC2Byte(last);

  return (first << 6) | (second << 4) | (third << 2) | (last); // return GC byte
}

inline void GCComm::SendPair(uint8_t sent)  // sends UART Byte via TX (SHOULD USE 6-BIT FRAME)
{
  while (!(UCSR0A & (1<<UDRE0))); //wait for TX buffer to be ready to send data
  UDR0 = sent;  // send data
  UCSR0A |= (1 << TXC0);          // write 1 to TXC0 to clear it
}

inline uint8_t GCComm::ReceivePair()  // receives UART byte via RX (SHOULD USE 8-BIT FRAME)
{
  while (!(UCSR0A & (1<<RXC0)));  // wait for data to be received
  return UDR0;  // read data
}

inline uint8_t GCComm::Byte2GC(uint8_t dataByte) // translates 2 GC bits into one UART byte (SHOULD USE 6-BIT FRAME)
{
  switch (dataByte) // UART sends bits LSB first (unlike GC which sends them MSB first) so these have to be flipped
  {                 
                          // start & stop bits in []
                          // GC bits      desired output            sent UART       UART input
    case 0: return 0x04;  // 0b00   ->    [0]001000[1] -> (flip) -> [0]000100[1]  ->  0x04 
    case 1: return 0x34;  // 0b01   ->    [0]001011[1] -> (flip) -> [0]110100[1]  ->  0x34
    case 2: return 0x07;  // 0b10   ->    [0]111000[1] -> (flip) -> [0]000111[1]  ->  0x07
    case 3: return 0x37;  // 0b11   ->    [0]111011[1] -> (flip) -> [0]110111[1]  ->  0x37
  }
}

inline uint8_t GCComm::GC2Byte(uint8_t dataByte)  // translates 1 UART byte (8-bit frame) to 2 GC bits
{
  dataByte &= 0b01010010; // we only care about checking the 2nd and 2nd-last in the UART since they correspond with the GC bits (possible values )
                          // the middle bit is checked to see if its a stop bit (1 if stop bit)
  
  switch (dataByte)
  {
    case 0x00: return 0;
    case 0x40: return 1;
    case 0x02: return 2;
    case 0x42: return 3;
    case 0x52: return 255;  // STOP bit if input is 0b01010010
    default:   return 0;    // default to 00 if something goes wrong
  }
}

inline void GCComm::SendStopBit()
{
  SendPair(0x3F); // send (01111111) stop bit to tell console that your message is done
}

inline void GCComm::FlushReceiveBuffer() // clear receiver buffer before changing frame size or awaiting new messages from the console
{
  while (!(UCSR0A & (1<<TXC0)));
  uint8_t dummy;
  while(UCSR0A & (1<<RXC0)) dummy = UDR0; // read data until no more data is in the buffer
}

inline void GCComm::SetFrameSize6() // changes UART frame size to 6 (after receiving a console command and before sending a controller message)
{
   //while (UCSR0A & (1<<RXC0)); // wait for receive buffer to be clear (i.e. wait for the console command to be read)
   //FlushReceiveBuffer();
   UCSR0C =  (1<<UCSZ00);
}

inline void GCComm::SetFrameSize8() // changes UART frame size to 8 (after receiving sending a controller message - this is the default frame size for receiving data from the console)
{
   FlushReceiveBuffer();
   while (!(UCSR0A & (1<<TXC0))); // wait for transmit buffer to be clear (i.e. wait for the controller message to be sent)
   UCSR0C =  (1<<UCSZ00) | (1<<UCSZ01);
}

inline void GCComm::SetRumble(uint8_t command)
{
  rumble = false;
  if (command = 4) rumble = true;
}
