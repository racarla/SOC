#include "datalogger.hxx"

/* Initializes the datalogger states and creates a binary file for flight data */
Datalogger::Datalogger() {
  size_t FileNameCounter = 0;
  std::string DataLogBaseName = "data";
  std::string DataLogType = ".bin";
  DataLogName = DataLogBaseName + std::to_string(FileNameCounter) + DataLogType;
  while(FileExists(DataLogName)) {
    FileNameCounter++;
    DataLogName = DataLogBaseName + std::to_string(FileNameCounter) + DataLogType;
  }
  if ((LogFile_ = fopen(DataLogName.c_str(),"wb"))<0) {
    throw std::runtime_error("Datalog failed to open.");
  }
}

/* Registers global data with the datalogger */
void Datalogger::RegisterGlobalData(DefinitionTree &DefinitionTreeRef) {
  // Get all keys
  std::vector<std::string> Keys;
  DefinitionTreeRef.GetKeys("/",&Keys);
  // Find keys that are marked to be datalogged
  for (auto const & elem: Keys) {
    if (DefinitionTreeRef.GetDatalog(elem)) {
      // store keys, description, and value pointers
      if (DefinitionTreeRef.GetValue<uint64_t>(elem)) {
        SaveAsUint64Keys_.push_back(elem);
        SaveAsUint64Description_.push_back(DefinitionTreeRef.GetDescription(elem));
        SaveAsUint64Values_.push_back(DefinitionTreeRef.GetValue<uint64_t>(elem));
      }
      if (DefinitionTreeRef.GetValue<uint32_t>(elem)) {
        SaveAsUint32Keys_.push_back(elem);
        SaveAsUint32Description_.push_back(DefinitionTreeRef.GetDescription(elem));
        SaveAsUint32Values_.push_back(DefinitionTreeRef.GetValue<uint32_t>(elem));
      }
      if (DefinitionTreeRef.GetValue<uint16_t>(elem)) {
        SaveAsUint16Keys_.push_back(elem);
        SaveAsUint16Description_.push_back(DefinitionTreeRef.GetDescription(elem));
        SaveAsUint16Values_.push_back(DefinitionTreeRef.GetValue<uint16_t>(elem));
      }
      if (DefinitionTreeRef.GetValue<uint8_t>(elem)) {
        SaveAsUint8Keys_.push_back(elem);
        SaveAsUint8Description_.push_back(DefinitionTreeRef.GetDescription(elem));
        SaveAsUint8Values_.push_back(DefinitionTreeRef.GetValue<uint8_t>(elem));
      }
      if (DefinitionTreeRef.GetValue<int64_t>(elem)) {
        SaveAsInt64Keys_.push_back(elem);
        SaveAsInt64Description_.push_back(DefinitionTreeRef.GetDescription(elem));
        SaveAsInt64Values_.push_back(DefinitionTreeRef.GetValue<int64_t>(elem));
      }
      if (DefinitionTreeRef.GetValue<int32_t>(elem)) {
        SaveAsInt32Keys_.push_back(elem);
        SaveAsInt32Description_.push_back(DefinitionTreeRef.GetDescription(elem));
        SaveAsInt32Values_.push_back(DefinitionTreeRef.GetValue<int32_t>(elem));
      }
      if (DefinitionTreeRef.GetValue<int16_t>(elem)) {
        SaveAsInt16Keys_.push_back(elem);
        SaveAsInt16Description_.push_back(DefinitionTreeRef.GetDescription(elem));
        SaveAsInt16Values_.push_back(DefinitionTreeRef.GetValue<int16_t>(elem));
      }
      if (DefinitionTreeRef.GetValue<int8_t>(elem)) {
        SaveAsInt8Keys_.push_back(elem);
        SaveAsInt8Description_.push_back(DefinitionTreeRef.GetDescription(elem));
        SaveAsInt8Values_.push_back(DefinitionTreeRef.GetValue<int8_t>(elem));
      }
      if (DefinitionTreeRef.GetValue<float>(elem)) {
        SaveAsFloatKeys_.push_back(elem);
        SaveAsFloatDescription_.push_back(DefinitionTreeRef.GetDescription(elem));
        SaveAsFloatValues_.push_back(DefinitionTreeRef.GetValue<float>(elem));
      }
      if (DefinitionTreeRef.GetValue<double>(elem)) {
        SaveAsDoubleKeys_.push_back(elem);
        SaveAsDoubleDescription_.push_back(DefinitionTreeRef.GetDescription(elem));
        SaveAsDoubleValues_.push_back(DefinitionTreeRef.GetValue<double>(elem));
      }
    }
  }
  // define the data buffer size
  LogDataBuffer_.resize(
    SaveAsUint64Values_.size()*sizeof(uint64_t) +
    SaveAsUint32Values_.size()*sizeof(uint32_t) +
    SaveAsUint16Values_.size()*sizeof(uint16_t) +
    SaveAsUint8Values_.size()*sizeof(uint8_t) +
    SaveAsInt64Values_.size()*sizeof(int64_t) +
    SaveAsInt32Values_.size()*sizeof(int32_t) +
    SaveAsInt16Values_.size()*sizeof(int16_t) +
    SaveAsInt8Values_.size()*sizeof(int8_t) +
    SaveAsFloatValues_.size()*sizeof(float) +
    SaveAsDoubleValues_.size()*sizeof(double)
  );
  // save meta data to disk
  for (size_t i=0; i < SaveAsUint64Keys_.size();i++) {
    std::vector<uint8_t> Buffer;
    for (size_t j=0; j < SaveAsUint64Keys_[i].size(); j++) {
      Buffer.push_back((uint8_t)SaveAsUint64Keys_[i][j]);
    }
    LogBinary(DataType::Uint64Key,Buffer);
    Buffer.clear();
    for (size_t j=0; j < SaveAsUint64Description_[i].size(); j++) {
      Buffer.push_back((uint8_t)SaveAsUint64Description_[i][j]);
    }
    LogBinary(DataType::Uint64Desc,Buffer);
  }
  for (size_t i=0; i < SaveAsUint32Keys_.size();i++) {
    std::vector<uint8_t> Buffer;
    for (size_t j=0; j < SaveAsUint32Keys_[i].size(); j++) {
      Buffer.push_back((uint8_t)SaveAsUint32Keys_[i][j]);
    }
    LogBinary(DataType::Uint32Key,Buffer);
    Buffer.clear();
    for (size_t j=0; j < SaveAsUint32Description_[i].size(); j++) {
      Buffer.push_back((uint8_t)SaveAsUint32Description_[i][j]);
    }
    LogBinary(DataType::Uint32Desc,Buffer);
  }
  for (size_t i=0; i < SaveAsUint16Keys_.size();i++) {
    std::vector<uint8_t> Buffer;
    for (size_t j=0; j < SaveAsUint16Keys_[i].size(); j++) {
      Buffer.push_back((uint8_t)SaveAsUint16Keys_[i][j]);
    }
    LogBinary(DataType::Uint16Key,Buffer);
    Buffer.clear();
    for (size_t j=0; j < SaveAsUint16Description_[i].size(); j++) {
      Buffer.push_back((uint8_t)SaveAsUint16Description_[i][j]);
    }
    LogBinary(DataType::Uint16Desc,Buffer);
  }
  for (size_t i=0; i < SaveAsUint8Keys_.size();i++) {
    std::vector<uint8_t> Buffer;
    for (size_t j=0; j < SaveAsUint8Keys_[i].size(); j++) {
      Buffer.push_back((uint8_t)SaveAsUint8Keys_[i][j]);
    }
    LogBinary(DataType::Uint8Key,Buffer);
    Buffer.clear();
    for (size_t j=0; j < SaveAsUint8Description_[i].size(); j++) {
      Buffer.push_back((uint8_t)SaveAsUint8Description_[i][j]);
    }
    LogBinary(DataType::Uint8Desc,Buffer);
  }
  for (size_t i=0; i < SaveAsInt64Keys_.size();i++) {
    std::vector<uint8_t> Buffer;
    for (size_t j=0; j < SaveAsInt64Keys_[i].size(); j++) {
      Buffer.push_back((uint8_t)SaveAsInt64Keys_[i][j]);
    }
    LogBinary(DataType::Int64Key,Buffer);
    Buffer.clear();
    for (size_t j=0; j < SaveAsInt64Description_[i].size(); j++) {
      Buffer.push_back((uint8_t)SaveAsInt64Description_[i][j]);
    }
    LogBinary(DataType::Int64Desc,Buffer);
  }
  for (size_t i=0; i < SaveAsInt32Keys_.size();i++) {
    std::vector<uint8_t> Buffer;
    for (size_t j=0; j < SaveAsInt32Keys_[i].size(); j++) {
      Buffer.push_back((uint8_t)SaveAsInt32Keys_[i][j]);
    }
    LogBinary(DataType::Int32Key,Buffer);
    Buffer.clear();
    for (size_t j=0; j < SaveAsInt32Description_[i].size(); j++) {
      Buffer.push_back((uint8_t)SaveAsInt32Description_[i][j]);
    }
    LogBinary(DataType::Int32Desc,Buffer);
  }
  for (size_t i=0; i < SaveAsInt16Keys_.size();i++) {
    std::vector<uint8_t> Buffer;
    for (size_t j=0; j < SaveAsInt16Keys_[i].size(); j++) {
      Buffer.push_back((uint8_t)SaveAsInt16Keys_[i][j]);
    }
    LogBinary(DataType::Int16Key,Buffer);
    Buffer.clear();
    for (size_t j=0; j < SaveAsInt16Description_[i].size(); j++) {
      Buffer.push_back((uint8_t)SaveAsInt16Description_[i][j]);
    }
    LogBinary(DataType::Int16Desc,Buffer);
  }
  for (size_t i=0; i < SaveAsInt8Keys_.size();i++) {
    std::vector<uint8_t> Buffer;
    for (size_t j=0; j < SaveAsInt8Keys_[i].size(); j++) {
      Buffer.push_back((uint8_t)SaveAsInt8Keys_[i][j]);
    }
    LogBinary(DataType::Int8Key,Buffer);
    Buffer.clear();
    for (size_t j=0; j < SaveAsInt8Description_[i].size(); j++) {
      Buffer.push_back((uint8_t)SaveAsInt8Description_[i][j]);
    }
    LogBinary(DataType::Int8Desc,Buffer);
  }
  for (size_t i=0; i < SaveAsFloatKeys_.size();i++) {
    std::vector<uint8_t> Buffer;
    for (size_t j=0; j < SaveAsFloatKeys_[i].size(); j++) {
      Buffer.push_back((uint8_t)SaveAsFloatKeys_[i][j]);
    }
    LogBinary(DataType::FloatKey,Buffer);
    Buffer.clear();
    for (size_t j=0; j < SaveAsFloatDescription_[i].size(); j++) {
      Buffer.push_back((uint8_t)SaveAsFloatDescription_[i][j]);
    }
    LogBinary(DataType::FloatDesc,Buffer);
  }
  for (size_t i=0; i < SaveAsDoubleKeys_.size();i++) {
    std::vector<uint8_t> Buffer;
    for (size_t j=0; j < SaveAsDoubleKeys_[i].size(); j++) {
      Buffer.push_back((uint8_t)SaveAsDoubleKeys_[i][j]);
    }
    LogBinary(DataType::DoubleKey,Buffer);
    Buffer.clear();
    for (size_t j=0; j < SaveAsDoubleDescription_[i].size(); j++) {
      Buffer.push_back((uint8_t)SaveAsDoubleDescription_[i][j]);
    }
    LogBinary(DataType::DoubleDesc,Buffer);
  }
}

/* Logs data into the binary file that was created */
void Datalogger::LogBinaryData() {
  size_t BufferLocation = 0;
  // payload
  memcpy(LogDataBuffer_.data()+BufferLocation,&(*SaveAsUint64Values_[0]),SaveAsUint64Values_.size()*sizeof(uint64_t));
  BufferLocation += SaveAsUint64Values_.size()*sizeof(uint64_t);
  memcpy(LogDataBuffer_.data()+BufferLocation,&(*SaveAsUint32Values_[0]),SaveAsUint32Values_.size()*sizeof(uint32_t));
  BufferLocation += SaveAsUint32Values_.size()*sizeof(uint32_t);
  memcpy(LogDataBuffer_.data()+BufferLocation,&(*SaveAsUint16Values_[0]),SaveAsUint16Values_.size()*sizeof(uint16_t));
  BufferLocation += SaveAsUint16Values_.size()*sizeof(uint16_t);
  memcpy(LogDataBuffer_.data()+BufferLocation,&(*SaveAsUint8Values_[0]),SaveAsUint8Values_.size()*sizeof(uint8_t));
  BufferLocation += SaveAsUint8Values_.size()*sizeof(uint8_t);
  memcpy(LogDataBuffer_.data()+BufferLocation,&(*SaveAsInt64Values_[0]),SaveAsInt64Values_.size()*sizeof(int64_t));
  BufferLocation += SaveAsInt64Values_.size()*sizeof(int64_t);
  memcpy(LogDataBuffer_.data()+BufferLocation,&(*SaveAsInt32Values_[0]),SaveAsInt32Values_.size()*sizeof(int32_t));
  BufferLocation += SaveAsInt32Values_.size()*sizeof(int32_t);
  memcpy(LogDataBuffer_.data()+BufferLocation,&(*SaveAsInt16Values_[0]),SaveAsInt16Values_.size()*sizeof(int16_t));
  BufferLocation += SaveAsInt16Values_.size()*sizeof(int16_t);
  memcpy(LogDataBuffer_.data()+BufferLocation,&(*SaveAsInt8Values_[0]),SaveAsInt8Values_.size()*sizeof(int8_t));
  BufferLocation += SaveAsInt8Values_.size()*sizeof(int8_t);
  memcpy(LogDataBuffer_.data()+BufferLocation,&(*SaveAsFloatValues_[0]),SaveAsFloatValues_.size()*sizeof(float));
  BufferLocation += SaveAsFloatValues_.size()*sizeof(float);
  memcpy(LogDataBuffer_.data()+BufferLocation,&(*SaveAsDoubleValues_[0]),SaveAsDoubleValues_.size()*sizeof(double));
  BufferLocation += SaveAsDoubleValues_.size()*sizeof(double);
  // write to disk
  LogBinary(DataType::Data,LogDataBuffer_);
}

void Datalogger::End() {
  fclose(LogFile_);
  SaveAsUint64Keys_.clear();
  SaveAsUint64Description_.clear();
  SaveAsUint64Values_.clear();
  SaveAsUint32Keys_.clear();
  SaveAsUint32Description_.clear();
  SaveAsUint32Values_.clear();
  SaveAsUint16Keys_.clear();
  SaveAsUint16Description_.clear();
  SaveAsUint16Values_.clear();
  SaveAsUint8Keys_.clear();
  SaveAsUint8Description_.clear();
  SaveAsUint8Values_.clear();
  SaveAsInt64Keys_.clear();
  SaveAsInt64Description_.clear();
  SaveAsInt64Values_.clear();
  SaveAsInt32Keys_.clear();
  SaveAsInt32Description_.clear();
  SaveAsInt32Values_.clear();
  SaveAsInt16Keys_.clear();
  SaveAsInt16Description_.clear();
  SaveAsInt16Values_.clear();
  SaveAsInt8Keys_.clear();
  SaveAsInt8Description_.clear();
  SaveAsInt8Values_.clear();
  SaveAsFloatKeys_.clear();
  SaveAsFloatDescription_.clear();
  SaveAsFloatValues_.clear();
  SaveAsDoubleKeys_.clear();
  SaveAsDoubleDescription_.clear();
  SaveAsDoubleValues_.clear();
}

void Datalogger::CreateHdfLog() {
  // create HDF datalog file
  size_t FileNameCounter = 0;
  std::string DataLogBaseName = "data";
  std::string DataLogType = ".h5";
  std::string h5DataLogName = DataLogBaseName + std::to_string(FileNameCounter) + DataLogType;
  while(FileExists(h5DataLogName)) {
    FileNameCounter++;
    h5DataLogName = DataLogBaseName + std::to_string(FileNameCounter) + DataLogType;
  }
  Hdf5Utils Logger(h5DataLogName);
  // open binary log file
  LogFile_ = fopen(DataLogName.c_str(),"rb");
  /* figure out how big the file is */
  fseek(LogFile_,0,SEEK_END);
  size_t size = ftell(LogFile_);
  rewind(LogFile_);
  std::cout << "File size: " << size << " bytes"<< std::endl;
  /* read binary file into memory */
  std::vector<uint8_t> MemBuffer;
  MemBuffer.resize(size);
  fread(MemBuffer.data(),1,size,LogFile_);
  fclose(LogFile_);
  // find all the meta data
  DataType Type;
  std::vector<uint8_t> Payload;
  for (size_t i=0; i < MemBuffer.size(); i++) {
    if (ReadBinaryData(MemBuffer[i],&Type,&Payload)) {
      switch (Type) {
        case DataType::Uint64Key: {
          std::string KeyName;
          KeyName.assign((char *)Payload.data(),Payload.size());
          SaveAsUint64Keys_.push_back(KeyName);
        }
        case DataType::Uint32Key: {
          std::string KeyName;
          KeyName.assign((char *)Payload.data(),Payload.size());
          SaveAsUint32Keys_.push_back(KeyName);
        }
        case DataType::Uint16Key: {
          std::string KeyName;
          KeyName.assign((char *)Payload.data(),Payload.size());
          SaveAsUint16Keys_.push_back(KeyName);
        }
        case DataType::Uint8Key: {
          std::string KeyName;
          KeyName.assign((char *)Payload.data(),Payload.size());
          SaveAsUint8Keys_.push_back(KeyName);
        }
        case DataType::Int64Key: {
          std::string KeyName;
          KeyName.assign((char *)Payload.data(),Payload.size());
          SaveAsInt64Keys_.push_back(KeyName);
        }
        case DataType::Int32Key: {
          std::string KeyName;
          KeyName.assign((char *)Payload.data(),Payload.size());
          SaveAsInt32Keys_.push_back(KeyName);
        }
        case DataType::Int16Key: {
          std::string KeyName;
          KeyName.assign((char *)Payload.data(),Payload.size());
          SaveAsInt16Keys_.push_back(KeyName);
        }
        case DataType::Int8Key: {
          std::string KeyName;
          KeyName.assign((char *)Payload.data(),Payload.size());
          SaveAsInt8Keys_.push_back(KeyName);
        }
        case DataType::FloatKey: {
          std::string KeyName;
          KeyName.assign((char *)Payload.data(),Payload.size());
          SaveAsFloatKeys_.push_back(KeyName);
        }
        case DataType::DoubleKey: {
          std::string KeyName;
          KeyName.assign((char *)Payload.data(),Payload.size());
          SaveAsDoubleKeys_.push_back(KeyName);
        }
        case DataType::Uint64Desc: {
          std::string Desc;
          Desc.assign((char *)Payload.data(),Payload.size());
          SaveAsUint64Description_.push_back(Desc);
        }
        case DataType::Uint32Desc: {
          std::string Desc;
          Desc.assign((char *)Payload.data(),Payload.size());
          SaveAsUint32Description_.push_back(Desc);
        }
        case DataType::Uint16Desc: {
          std::string Desc;
          Desc.assign((char *)Payload.data(),Payload.size());
          SaveAsUint16Description_.push_back(Desc);
        }
        case DataType::Uint8Desc: {
          std::string Desc;
          Desc.assign((char *)Payload.data(),Payload.size());
          SaveAsUint8Description_.push_back(Desc);
        }
        case DataType::Int64Desc: {
          std::string Desc;
          Desc.assign((char *)Payload.data(),Payload.size());
          SaveAsInt64Description_.push_back(Desc);
        }
        case DataType::Int32Desc: {
          std::string Desc;
          Desc.assign((char *)Payload.data(),Payload.size());
          SaveAsInt32Description_.push_back(Desc);
        }
        case DataType::Int16Desc: {
          std::string Desc;
          Desc.assign((char *)Payload.data(),Payload.size());
          SaveAsInt16Description_.push_back(Desc);
        }
        case DataType::Int8Desc: {
          std::string Desc;
          Desc.assign((char *)Payload.data(),Payload.size());
          SaveAsInt8Description_.push_back(Desc);
        }
        case DataType::FloatDesc: {
          std::string Desc;
          Desc.assign((char *)Payload.data(),Payload.size());
          SaveAsFloatDescription_.push_back(Desc);
        }
        case DataType::DoubleDesc: {
          std::string Desc;
          Desc.assign((char *)Payload.data(),Payload.size());
          SaveAsDoubleDescription_.push_back(Desc);
        }
      }
    }
  }

  // // find all the flight data
  // std::vector<std::vector<uint64_t>> Uint64Data;
  // Uint64Data.resize(SaveAsUint64Keys_.size());
  //
  // for (size_t i=0; i < Buffer.size(); i++) {
  //   if (ReadBinaryData(Buffer[i],&metaData,&Payload)) {
  //     if (metaData == MetaData::Data) {
  //       size_t BufferLocation = 0;
  //       uint64_t uint64d[SaveAsUint64Keys_.size()];
  //       memcpy(&uint64d,Payload.data(),SaveAsUint64Keys_.size()*sizeof(uint64_t));
  //       BufferLocation += SaveAsUint64Keys_.size()*sizeof(uint64_t);
  //       for (size_t i=0; i < Uint64Data.size(); i++) {
  //         Uint64Data[i].push_back(uint64d[i]);
  //       }
  //     }
  //   }
  // }
  //
  // uint64_t val[1];
  // val[0] = 12345;
  // Logger.WriteData("/Fmu","Time_us",val,"Time, us",1,1);
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

/* Logs byte buffer given meta data */
void Datalogger::LogBinary(DataType Type, std::vector<uint8_t> &Buffer) {
  std::vector<uint8_t> LogBuffer;
  LogBuffer.resize(headerLength_ + Buffer.size() + checksumLength_);
  // header
  LogBuffer[0] = header_[0];
  LogBuffer[1] = header_[1];
  // data source
  LogBuffer[2] = (uint8_t) Type;
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

/* Reads data packets from binary*/
bool Datalogger::ReadBinaryData(uint8_t ReadByte,DataType *Type,std::vector<uint8_t> *Payload) {
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
      Buffer_.clear();
      return false;
    }
  // checksum 1
  } else if (ParserState_ == (Length_ + headerLength_ + 1)) {
    if (ReadByte == Checksum_[1]) {
      // data source
      *Type = (DataType) Buffer_[2];
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
      Buffer_.clear();
      return true;
    } else {
      ParserState_ = 0;
      LengthBuffer_[0] = 0;
      LengthBuffer_[1] = 0;
      Length_ = 0;
      Checksum_[0] = 0;
      Checksum_[1] = 0;
      Buffer_.clear();
      return false;
    }
  }
  return false;
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

Hdf5Utils::Hdf5Utils(std::string FileName) {
  file_ = new H5::H5File(FileName.c_str(), H5F_ACC_EXCL);
}

void Hdf5Utils::CreateGroup(std::string GroupName) {
  H5::Exception::dontPrint();

  // open group if exists, otherwise create it
  try {
    H5::Group LogGroup = file_->openGroup(GroupName.c_str());
  } catch (...) {
    H5::Group LogGroup = file_->createGroup(GroupName.c_str());
  }
  H5::Group LogGroup = file_->openGroup(GroupName.c_str());
}

void Hdf5Utils::WriteData(std::string GroupName,std::string Name,uint8_t *data,std::string Attr,size_t rows,size_t columns) {
  H5::Exception::dontPrint();

  // open group if exists, otherwise create it
  try {
    H5::Group LogGroup = file_->openGroup(GroupName.c_str());
  } catch (...) {
    H5::Group LogGroup = file_->createGroup(GroupName.c_str());
  }
  H5::Group LogGroup = file_->openGroup(GroupName.c_str());

  // data space
  hsize_t DataDims[2];
  DataDims[0] = rows;
  DataDims[1] = columns;
  H5::DataSpace LogDataSpace = H5::DataSpace(2,DataDims);

  // data set
  H5::DataSet LogDataSet = H5::DataSet(LogGroup.createDataSet(Name.c_str(),H5::PredType::NATIVE_UINT8,LogDataSpace));

  // attribute
  H5::StrType str_type(H5::PredType::C_S1, H5T_VARIABLE);
  hsize_t AttrDims[1];
  AttrDims[0] = 1;
  H5::DataSpace AttrDataSpace = H5::DataSpace(1,AttrDims);
  H5::Attribute LogAttribute = H5::Attribute(LogDataSet.createAttribute("Desc",str_type,AttrDataSpace));
  LogAttribute.write(str_type,Attr);

  // write data
  LogDataSet.write(data,H5::PredType::NATIVE_UINT8);
}

void Hdf5Utils::WriteData(std::string GroupName,std::string Name,uint16_t *data,std::string Attr,size_t rows,size_t columns) {
  H5::Exception::dontPrint();

  // open group if exists, otherwise create it
  try {
    H5::Group LogGroup = file_->openGroup(GroupName.c_str());
  } catch (...) {
    H5::Group LogGroup = file_->createGroup(GroupName.c_str());
  }
  H5::Group LogGroup = file_->openGroup(GroupName.c_str());

  // data space
  hsize_t DataDims[2];
  DataDims[0] = rows;
  DataDims[1] = columns;
  H5::DataSpace LogDataSpace = H5::DataSpace(2,DataDims);

  // data set
  H5::DataSet LogDataSet = H5::DataSet(LogGroup.createDataSet(Name.c_str(),H5::PredType::NATIVE_UINT16,LogDataSpace));

  // attribute
  H5::StrType str_type(H5::PredType::C_S1, H5T_VARIABLE);
  hsize_t AttrDims[1];
  AttrDims[0] = 1;
  H5::DataSpace AttrDataSpace = H5::DataSpace(1,AttrDims);
  H5::Attribute LogAttribute = H5::Attribute(LogDataSet.createAttribute("Desc",str_type,AttrDataSpace));
  LogAttribute.write(str_type,Attr);

  // write data
  LogDataSet.write(data,H5::PredType::NATIVE_UINT16);
}

void Hdf5Utils::WriteData(std::string GroupName,std::string Name,uint32_t *data,std::string Attr,size_t rows,size_t columns) {
  H5::Exception::dontPrint();

  // open group if exists, otherwise create it
  try {
    H5::Group LogGroup = file_->openGroup(GroupName.c_str());
  } catch (...) {
    H5::Group LogGroup = file_->createGroup(GroupName.c_str());
  }
  H5::Group LogGroup = file_->openGroup(GroupName.c_str());

  // data space
  hsize_t DataDims[2];
  DataDims[0] = rows;
  DataDims[1] = columns;
  H5::DataSpace LogDataSpace = H5::DataSpace(2,DataDims);

  // data set
  H5::DataSet LogDataSet = H5::DataSet(LogGroup.createDataSet(Name.c_str(),H5::PredType::NATIVE_UINT32,LogDataSpace));

  // attribute
  H5::StrType str_type(H5::PredType::C_S1, H5T_VARIABLE);
  hsize_t AttrDims[1];
  AttrDims[0] = 1;
  H5::DataSpace AttrDataSpace = H5::DataSpace(1,AttrDims);
  H5::Attribute LogAttribute = H5::Attribute(LogDataSet.createAttribute("Desc",str_type,AttrDataSpace));
  LogAttribute.write(str_type,Attr);

  // write data
  LogDataSet.write(data,H5::PredType::NATIVE_UINT32);
}

void Hdf5Utils::WriteData(std::string GroupName,std::string Name,uint64_t *data,std::string Attr,size_t rows,size_t columns) {
  H5::Exception::dontPrint();

  // open group if exists, otherwise create it
  try {
    H5::Group LogGroup = file_->openGroup(GroupName.c_str());
  } catch (...) {
    H5::Group LogGroup = file_->createGroup(GroupName.c_str());
  }
  H5::Group LogGroup = file_->openGroup(GroupName.c_str());

  // data space
  hsize_t DataDims[2];
  DataDims[0] = rows;
  DataDims[1] = columns;
  H5::DataSpace LogDataSpace = H5::DataSpace(2,DataDims);

  // data set
  H5::DataSet LogDataSet = H5::DataSet(LogGroup.createDataSet(Name.c_str(),H5::PredType::NATIVE_UINT64,LogDataSpace));

  // attribute
  H5::StrType str_type(H5::PredType::C_S1, H5T_VARIABLE);
  hsize_t AttrDims[1];
  AttrDims[0] = 1;
  H5::DataSpace AttrDataSpace = H5::DataSpace(1,AttrDims);
  H5::Attribute LogAttribute = H5::Attribute(LogDataSet.createAttribute("Desc",str_type,AttrDataSpace));
  LogAttribute.write(str_type,Attr);

  // write data
  LogDataSet.write(data,H5::PredType::NATIVE_UINT64);
}

void Hdf5Utils::WriteData(std::string GroupName,std::string Name,float *data,std::string Attr,size_t rows,size_t columns) {
  H5::Exception::dontPrint();

  // open group if exists, otherwise create it
  try {
    H5::Group LogGroup = file_->openGroup(GroupName.c_str());
  } catch (...) {
    H5::Group LogGroup = file_->createGroup(GroupName.c_str());
  }
  H5::Group LogGroup = file_->openGroup(GroupName.c_str());

  // data space
  hsize_t DataDims[2];
  DataDims[0] = rows;
  DataDims[1] = columns;
  H5::DataSpace LogDataSpace = H5::DataSpace(2,DataDims);

  // data set
  H5::DataSet LogDataSet = H5::DataSet(LogGroup.createDataSet(Name.c_str(),H5::PredType::NATIVE_FLOAT,LogDataSpace));

  // attribute
  H5::StrType str_type(H5::PredType::C_S1, H5T_VARIABLE);
  hsize_t AttrDims[1];
  AttrDims[0] = 1;
  H5::DataSpace AttrDataSpace = H5::DataSpace(1,AttrDims);
  H5::Attribute LogAttribute = H5::Attribute(LogDataSet.createAttribute("Desc",str_type,AttrDataSpace));
  LogAttribute.write(str_type,Attr);

  // write data
  LogDataSet.write(data,H5::PredType::NATIVE_FLOAT);
}

void Hdf5Utils::WriteData(std::string GroupName,std::string Name,double *data,std::string Attr,size_t rows,size_t columns) {
  H5::Exception::dontPrint();

  // open group if exists, otherwise create it
  try {
    H5::Group LogGroup = file_->openGroup(GroupName.c_str());
  } catch (...) {
    H5::Group LogGroup = file_->createGroup(GroupName.c_str());
  }
  H5::Group LogGroup = file_->openGroup(GroupName.c_str());

  // data space
  hsize_t DataDims[2];
  DataDims[0] = rows;
  DataDims[1] = columns;
  H5::DataSpace LogDataSpace = H5::DataSpace(2,DataDims);

  // data set
  H5::DataSet LogDataSet = H5::DataSet(LogGroup.createDataSet(Name.c_str(),H5::PredType::NATIVE_DOUBLE,LogDataSpace));

  // attribute
  H5::StrType str_type(H5::PredType::C_S1, H5T_VARIABLE);
  hsize_t AttrDims[1];
  AttrDims[0] = 1;
  H5::DataSpace AttrDataSpace = H5::DataSpace(1,AttrDims);
  H5::Attribute LogAttribute = H5::Attribute(LogDataSet.createAttribute("Desc",str_type,AttrDataSpace));
  LogAttribute.write(str_type,Attr);

  // write data
  LogDataSet.write(data,H5::PredType::NATIVE_DOUBLE);
}
