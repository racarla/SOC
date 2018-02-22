

#ifndef FMU_HXX_
#define FMU_HXX_

#include "global-defs.hxx"
#include "hardware-defs.hxx"
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <stdint.h>
#include <iostream>
#include <exception>
#include <stdexcept>

struct ErrStatus {
  uint16_t cntSuccess = 0;
  uint16_t cntUnavailErr = 0;
  uint16_t cntReadErr = 0;
  uint16_t cntPayloadErr = 0;
  uint16_t cntMessageErr = 0;
  uint16_t cntSizeErr = 0;
  uint16_t cntParseErr = 0;
  uint16_t cntHeaderErr = 0;
  uint16_t cntChecksumErr = 0;
};

class Fmu {
  public:
    static const uint8_t BfsHeaderSize = 7;
    const uint8_t BfsHeader[2]={0x42,0x46};
    Fmu();
    void WriteMessage(BfsMessage MessageId,uint16_t PayloadSize,uint8_t *Payload);
    bool ReadMessage(BfsMessage *MessageId,uint16_t *PayloadSize,uint8_t *Payload);
    bool GetSensorData(FmuData *FmuDataPtr);
    ErrStatus errStatus_;
    
  private:
    int FmuFileDesc_;
    void OpenPort();
    void WritePort(size_t BufferSize,uint8_t* Buffer);    
    void CalcChecksum(size_t ArraySize, uint8_t *ByteArray, uint8_t *Checksum);
    void ChecksumIteration(uint8_t Data, uint8_t *Checksum);
    void BuildBfsMessage(BfsMessage MessageId,uint16_t PayloadSize,uint8_t *Payload,uint16_t *TxBufferSize,uint8_t *TxBuffer);
    bool ParseBfsMessage(uint8_t RxBuffer,BfsMessage *MessageId,uint16_t *PayloadSize,uint8_t *Payload);

};

#endif
