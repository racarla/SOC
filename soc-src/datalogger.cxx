#include "datalogger.hxx"

/* Initializes the datalogger states and creates a binary file for flight data */
Datalogger::Datalogger() {
  size_t FileNameCounter = 0;
  std::string DataLogBaseName = "data";
  std::string DataLogType = ".bin";
  std::string DataLogName = DataLogBaseName + std::to_string(FileNameCounter) + DataLogType;

  while(FileExists(DataLogName)) {
    FileNameCounter++;
    DataLogName = DataLogBaseName + std::to_string(FileNameCounter) + DataLogType;
  }

  if ((LogFile_ = fopen(DataLogName.c_str(),"wb"))<0) {
    throw std::runtime_error("Datalog failed to open.");
  }
}

/* Logs the FMU sensor data into the binary file that was created */
void Datalogger::LogSensorData(std::vector<uint8_t> &Buffer) {
  LogData(DataSources::SensorData,Buffer);
}

/* Reads data packets from a byte */
bool Datalogger::ReadBinaryData(uint8_t ReadByte,DataSources *Source,std::vector<uint8_t> *Payload) {
  int count;
  // header
  if (ParserState_ < 2) {
    if (ReadByte == header_[ParserState_]) {
      Buffer_.push_back(ReadByte);
      ParserState_++;
    }
  } else if (ParserState_ == 3) {
    LengthBuffer_[0] = ReadByte;
    Buffer_.push_back(ReadByte);
    ParserState_++;
  } else if (ParserState_ == 4) {
    LengthBuffer_[1] = ReadByte;
    Length_ = ((uint16_t)LengthBuffer_[1] << 8) | LengthBuffer_[0];
    Buffer_.push_back(ReadByte);
    ParserState_++;
  } else if (ParserState_ < (Length_ + headerLength_)) {
    Buffer_.push_back(ReadByte);
    ParserState_++;
  } else if (ParserState_ == (Length_ + headerLength_)) {
    CalcChecksum(Length_ + headerLength_,Buffer_.data(),Checksum_);
    if (ReadByte == Checksum_[0]) {
      ParserState_++;
    } else {
      ParserState_ = 0;
      LengthBuffer_[0] = 0;
      LengthBuffer_[1] = 0;
      Length_ = 0;
      Checksum_[0] = 0;
      Checksum_[1] = 0;
      return false;
    }
  // checksum 1
  } else if (ParserState_ == (Length_ + headerLength_ + 1)) {
    if (ReadByte == Checksum_[1]) {
      // data source
      *Source = (DataSources) Buffer_[2];
      // payload size
      Payload->resize(Length_);
      // payload
      std::memcpy(Payload->data(),Buffer_.data()+headerLength_,Length_);
      ParserState_ = 0;
      LengthBuffer_[0] = 0;
      LengthBuffer_[1] = 0;
      Length_ = 0;
      Checksum_[0] = 0;
      Checksum_[1] = 0;
      return true;
    } else {
      ParserState_ = 0;
      LengthBuffer_[0] = 0;
      LengthBuffer_[1] = 0;
      Length_ = 0;
      Checksum_[0] = 0;
      Checksum_[1] = 0;
      return false;
    }
  }
  return false;
}

/* Checks to see if a file exists, returns true if it does and false if it does not */
bool Datalogger::FileExists(const std::string &FileName) {
  if (FILE *file = fopen(FileName.c_str(),"r")) {
    fclose(file);
    return true;
  } else {
    return false;
  }  
}

/* Logs data into the binary file that was created */
void Datalogger::LogData(DataSources source,std::vector<uint8_t> &Buffer) {
  std::vector<uint8_t> LogBuffer;
  LogBuffer.resize(headerLength_ + Buffer.size() + checksumLength_);
  // header
  LogBuffer[0] = header_[0];
  LogBuffer[1] = header_[1];
  // data source
  LogBuffer[2] = (uint8_t) source;
  // data length
  LogBuffer[3] = Buffer.size() & 0xff;
  LogBuffer[4] = Buffer.size() >> 8;
  // payload
  std::memcpy(LogBuffer.data()+headerLength_,Buffer.data(),Buffer.size());
  // checksum
  CalcChecksum((size_t)(Buffer.size()+headerLength_),LogBuffer.data(),Checksum_);
  LogBuffer[Buffer.size()+headerLength_] = Checksum_[0];
  LogBuffer[Buffer.size()+headerLength_+1] = Checksum_[1];
  // write to disk
  fwrite(LogBuffer.data(),LogBuffer.size(),1,LogFile_);
  fflush(LogFile_);
}

/* Computes a two byte checksum. */
void Datalogger::CalcChecksum(size_t ArraySize, uint8_t *ByteArray, uint8_t *Checksum) {
  Checksum[0] = 0;
  Checksum[1] = 0;
  for (size_t i = 0; i < ArraySize; i++) {
    Checksum[0] += ByteArray[i];
    Checksum[1] += Checksum[0];
  }
}
