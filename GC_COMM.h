#ifndef GC_COMM
#define GC_COMM

#include "Arduino.h"

class GCComm
{
  public:
    GCComm(); // sets up serial port
    void SendPollResponse();
    void SendOrigin();
    void SendInputs(); // sends
    void InitDataLine();
    void ReceiveCommand();

    volatile uint8_t  StYXBA =    0x00;
    volatile uint8_t  LRZDpad =   0x80;
    volatile uint8_t  ControlX =  128;
    volatile uint8_t  ControlY =  128;
    volatile uint8_t  CstickX =   128;
    volatile uint8_t  CstickY =   128; 
    volatile uint8_t  AnalogL =   0;
    volatile uint8_t  AnalogR =   0;
    
    volatile bool     rumble =    false;
    
  private:
    inline void SendByte(uint8_t dataByte);
    inline uint8_t ReceiveByte();
    inline void SendPair(uint8_t sent);
    inline uint8_t ReceivePair();
    inline uint8_t Byte2GC(uint8_t dataByte);
    inline uint8_t GC2Byte(uint8_t dataByte);
    inline void SendStopBit();
    inline void FlushReceiveBuffer();
    inline void SetFrameSize6();
    inline void SetFrameSize8();
    inline void SetRumble(uint8_t command);
};

#endif
