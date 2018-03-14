#ifndef DATALOGGER_HXX_
#define DATALOGGER_HXX_

#include "hdf5class.hxx"
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <stdint.h>
#include <fstream>
#include <iostream>
#include <vector>
#include <exception>
#include <stdexcept>
#include <cstring>

class Datalogger {
  public:
    enum DataSources {
      SensorData
    };
    Datalogger();
    void LogSensorData(std::vector<uint8_t> &Buffer);
    bool ReadBinaryData(uint8_t ReadByte,DataSources *Source,std::vector<uint8_t> *Payload);
  private:
    FILE *LogFile_;
    std::vector<uint8_t> Buffer_;
    const uint8_t header_[2] = {0x42,0x46};
    const uint8_t headerLength_ = 5;
    const uint8_t checksumLength_ = 2;
    uint8_t LengthBuffer_[2];
    uint16_t Length_;
    uint8_t Checksum_[2];
    uint16_t ParserState_;
    bool FileExists(const std::string &FileName);
    void LogData(DataSources source,std::vector<uint8_t> &Buffer);
    void CalcChecksum(size_t ArraySize, uint8_t *ByteArray, uint8_t *Checksum);
};
#endif
