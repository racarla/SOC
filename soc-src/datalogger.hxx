#ifndef DATALOGGER_HXX_
#define DATALOGGER_HXX_

#include "global-defs.hxx"
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
#include <H5Cpp.h>

class Datalogger {
  public:
    Datalogger();
    void RegisterGlobalData(DefinitionTree &DefinitionTreeRef);
    void LogBinaryData();
    void CreateHdfLog();
    void End();
  private:
    enum DataType {
      Uint64Key,
      Uint32Key,
      Uint16Key,
      Uint8Key,
      Int64Key,
      Int32Key,
      Int16Key,
      Int8Key,
      FloatKey,
      DoubleKey,
      Uint64Desc,
      Uint32Desc,
      Uint16Desc,
      Uint8Desc,
      Int64Desc,
      Int32Desc,
      Int16Desc,
      Int8Desc,
      FloatDesc,
      DoubleDesc,
      Data
    };
    std::string DataLogName;
    FILE *LogFile_;
    std::vector<std::string> SaveAsUint64Keys_;
    std::vector<std::string> SaveAsUint64Description_;
    std::vector<uint64_t*> SaveAsUint64Values_;
    std::vector<std::string> SaveAsUint32Keys_;
    std::vector<std::string> SaveAsUint32Description_;
    std::vector<uint32_t*> SaveAsUint32Values_;
    std::vector<std::string> SaveAsUint16Keys_;
    std::vector<std::string> SaveAsUint16Description_;
    std::vector<uint16_t*> SaveAsUint16Values_;
    std::vector<std::string> SaveAsUint8Keys_;
    std::vector<std::string> SaveAsUint8Description_;
    std::vector<uint8_t*> SaveAsUint8Values_;
    std::vector<std::string> SaveAsInt64Keys_;
    std::vector<std::string> SaveAsInt64Description_;
    std::vector<int64_t*> SaveAsInt64Values_;
    std::vector<std::string> SaveAsInt32Keys_;
    std::vector<std::string> SaveAsInt32Description_;
    std::vector<int32_t*> SaveAsInt32Values_;
    std::vector<std::string> SaveAsInt16Keys_;
    std::vector<std::string> SaveAsInt16Description_;
    std::vector<int16_t*> SaveAsInt16Values_;
    std::vector<std::string> SaveAsInt8Keys_;
    std::vector<std::string> SaveAsInt8Description_;
    std::vector<int8_t*> SaveAsInt8Values_;
    std::vector<std::string> SaveAsFloatKeys_;
    std::vector<std::string> SaveAsFloatDescription_;
    std::vector<float*> SaveAsFloatValues_;
    std::vector<std::string> SaveAsDoubleKeys_;
    std::vector<std::string> SaveAsDoubleDescription_;
    std::vector<double*> SaveAsDoubleValues_;
    std::vector<uint8_t> LogDataBuffer_;
    std::vector<uint8_t> Buffer_;
    const uint8_t header_[2] = {0x42,0x46};
    const uint8_t headerLength_ = 5;
    const uint8_t checksumLength_ = 2;
    uint8_t LengthBuffer_[2];
    uint16_t Length_ = 0;
    uint8_t Checksum_[2];
    uint16_t ParserState_ = 0;
    bool FileExists(const std::string &FileName);
    void LogBinary(DataType Type, std::vector<uint8_t> &Buffer);
    bool ReadBinaryData(uint8_t ReadByte,DataType *Type,std::vector<uint8_t> *Payload);
    void CalcChecksum(size_t ArraySize, uint8_t *ByteArray, uint8_t *Checksum);
};

class Hdf5Utils {
  public:
    Hdf5Utils(std::string FileName);
    void CreateGroup(std::string GroupName);
    void WriteData(std::string GroupName,std::string Name,uint8_t *data,std::string Attr,size_t rows,size_t columns);
    void WriteData(std::string GroupName,std::string Name,uint16_t *data,std::string Attr,size_t rows,size_t columns);
    void WriteData(std::string GroupName,std::string Name,uint32_t *data,std::string Attr,size_t rows,size_t columns);
    void WriteData(std::string GroupName,std::string Name,uint64_t *data,std::string Attr,size_t rows,size_t columns);
    void WriteData(std::string GroupName,std::string Name,float *data,std::string Attr,size_t rows,size_t columns);
    void WriteData(std::string GroupName,std::string Name,double *data,std::string Attr,size_t rows,size_t columns);
  private:
    H5::H5File *file_;
};

#endif
