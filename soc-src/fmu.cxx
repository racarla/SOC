
#include "fmu.hxx"

/* Opens port to communicate with FMU. */
void FlightManagementUnit::Begin() {
  std::cout << "Opening UART port with FMU...";
  if ((FmuFileDesc_=open(Port_.c_str(),O_RDWR|O_NOCTTY|O_NONBLOCK))<0) {
    throw std::runtime_error("UART failed to open.");
  }
  struct termios Options;
  tcgetattr(FmuFileDesc_,&Options);
  Options.c_cflag = Baud_ | CS8 | CREAD | CLOCAL;
  Options.c_iflag = IGNPAR;
  Options.c_oflag = 0;
  Options.c_lflag = 0;
  Options.c_cc[VTIME] = 0;
  Options.c_cc[VMIN] = 0;
  tcflush(FmuFileDesc_,TCIFLUSH);
  tcsetattr(FmuFileDesc_,TCSANOW,&Options);
  fcntl(FmuFileDesc_,F_SETFL,O_NONBLOCK);
  std::cout << "done!" << std::endl;
}

/* Registers data with global defs */
void FlightManagementUnit::RegisterGlobalData(DefinitionTree *DefinitionTreePtr) {
  DefinitionTreePtr->InitMember("/Sensors/Fmu/Time_us");
  DefinitionTreePtr->SetValue("/Sensors/Fmu/Time_us",&SensorData_.Time_us);
  DefinitionTreePtr->SetDescription("/Sensors/Fmu/Time_us","Flight management unit time, us");
  DefinitionTreePtr->SetDatalog("/Sensors/Fmu/Time_us",true);
  DefinitionTreePtr->SetTelemetry("/Sensors/Fmu/Time_us",true);

  // DefinitionTreePtr->InitMember("/Sensors/Fmu/Mpu9250/AccelX");
  // DefinitionTreePtr->SetValue("/Sensors/Fmu/Mpu9250/AccelX",&SensorData_.InternalMpu9250.Accel_mss(0,0));
  // DefinitionTreePtr->SetDescription("/Sensors/Fmu/Mpu9250/AccelX","X accelerometer, m/s/s");
  // DefinitionTreePtr->SetDatalog("/Sensors/Fmu/Mpu9250/AccelX",true);
  // DefinitionTreePtr->SetTelemetry("/Sensors/Fmu/Mpu9250/AccelX",true);
  //
  // DefinitionTreePtr->InitMember("/Sensors/Fmu/Mpu9250/AccelY");
  // DefinitionTreePtr->SetValue("/Sensors/Fmu/Mpu9250/AccelY",&SensorData_.InternalMpu9250.Accel_mss(1,0));
  // DefinitionTreePtr->SetDescription("/Sensors/Fmu/Mpu9250/AccelY","Y accelerometer, m/s/s");
  // DefinitionTreePtr->SetDatalog("/Sensors/Fmu/Mpu9250/AccelY",true);
  // DefinitionTreePtr->SetTelemetry("/Sensors/Fmu/Mpu9250/AccelY",true);
  //
  // DefinitionTreePtr->InitMember("/Sensors/Fmu/Mpu9250/AccelZ");
  // DefinitionTreePtr->SetValue("/Sensors/Fmu/Mpu9250/AccelZ",&SensorData_.InternalMpu9250.Accel_mss(2,0));
  // DefinitionTreePtr->SetDescription("/Sensors/Fmu/Mpu9250/AccelZ","Z accelerometer, m/s/s");
  // DefinitionTreePtr->SetDatalog("/Sensors/Fmu/Mpu9250/AccelZ",true);
  // DefinitionTreePtr->SetTelemetry("/Sensors/Fmu/Mpu9250/AccelZ",true);

}

/* Receive sensor data from FMU */
bool FlightManagementUnit::ReceiveSensorData() {
  Message message;
  std::vector<uint8_t> Payload;
  size_t PayloadLocation = 0;
  if (ReceiveMessage(&message,&Payload)) {
    if (message == SensorData) {
      // meta data
      uint8_t NumberMpu9250Sensor,NumberBme280Sensor,NumberuBloxSensor,NumberSwiftSensor,NumberAms5915Sensor,NumberSbusSensor,NumberAnalogSensor;
      memcpy(&NumberMpu9250Sensor,Payload.data()+PayloadLocation,sizeof(NumberMpu9250Sensor));
      PayloadLocation += sizeof(NumberMpu9250Sensor);
      memcpy(&NumberBme280Sensor,Payload.data()+PayloadLocation,sizeof(NumberBme280Sensor));
      PayloadLocation += sizeof(NumberBme280Sensor);
      memcpy(&NumberuBloxSensor,Payload.data()+PayloadLocation,sizeof(NumberuBloxSensor));
      PayloadLocation += sizeof(NumberuBloxSensor);
      memcpy(&NumberSwiftSensor,Payload.data()+PayloadLocation,sizeof(NumberSwiftSensor));
      PayloadLocation += sizeof(NumberSwiftSensor);
      memcpy(&NumberAms5915Sensor,Payload.data()+PayloadLocation,sizeof(NumberAms5915Sensor));
      PayloadLocation += sizeof(NumberAms5915Sensor);
      memcpy(&NumberSbusSensor,Payload.data()+PayloadLocation,sizeof(NumberSbusSensor));
      PayloadLocation += sizeof(NumberSbusSensor);
      memcpy(&NumberAnalogSensor,Payload.data()+PayloadLocation,sizeof(NumberAnalogSensor));
      PayloadLocation += sizeof(NumberAnalogSensor);
      // resize data buffers
      SensorData_.Mpu9250.resize(NumberMpu9250Sensor);
      SensorData_.Bme280.resize(NumberBme280Sensor);
      SensorData_.uBlox.resize(NumberuBloxSensor);
      SensorData_.Swift.resize(NumberSwiftSensor);
      SensorData_.Ams5915.resize(NumberAms5915Sensor);
      SensorData_.Sbus.resize(NumberSbusSensor);
      SensorData_.Analog.resize(NumberAnalogSensor);
      // sensor data
      memcpy(&SensorData_.Time_us,Payload.data()+PayloadLocation,sizeof(SensorData_.Time_us));
      PayloadLocation += sizeof(SensorData_.Time_us);
      memcpy(&SensorData_.InternalMpu9250,Payload.data()+PayloadLocation,sizeof(Mpu9250SensorData));
      PayloadLocation += sizeof(SensorData_.InternalMpu9250);
      memcpy(&SensorData_.InternalBme280,Payload.data()+PayloadLocation,sizeof(Bme280SensorData));
      PayloadLocation += sizeof(SensorData_.InternalBme280);
      memcpy(&SensorData_.InternalVoltage,Payload.data()+PayloadLocation,sizeof(VoltageSensorsData));
      PayloadLocation += sizeof(SensorData_.InternalVoltage);
      memcpy(&SensorData_.Mpu9250,Payload.data()+PayloadLocation,SensorData_.Mpu9250.size()*sizeof(Mpu9250SensorData));
      PayloadLocation += SensorData_.Mpu9250.size()*sizeof(Mpu9250SensorData);
      memcpy(&SensorData_.Bme280,Payload.data()+PayloadLocation,SensorData_.Bme280.size()*sizeof(Bme280SensorData));
      PayloadLocation += SensorData_.Bme280.size()*sizeof(Bme280SensorData);
      memcpy(&SensorData_.uBlox,Payload.data()+PayloadLocation,SensorData_.uBlox.size()*sizeof(uBloxSensorData));
      PayloadLocation += SensorData_.uBlox.size()*sizeof(uBloxSensorData);
      memcpy(&SensorData_.Swift,Payload.data()+PayloadLocation,SensorData_.Swift.size()*sizeof(SwiftSensorData));
      PayloadLocation += SensorData_.Swift.size()*sizeof(SwiftSensorData);
      memcpy(&SensorData_.Ams5915,Payload.data()+PayloadLocation,SensorData_.Ams5915.size()*sizeof(Ams5915SensorData));
      PayloadLocation += SensorData_.Ams5915.size()*sizeof(Ams5915SensorData);
      memcpy(&SensorData_.Sbus,Payload.data()+PayloadLocation,SensorData_.Sbus.size()*sizeof(SbusSensorData));
      PayloadLocation += SensorData_.Sbus.size()*sizeof(SbusSensorData);
      memcpy(&SensorData_.Analog,Payload.data()+PayloadLocation,SensorData_.Analog.size()*sizeof(AnalogSensorData));
      PayloadLocation += SensorData_.Analog.size()*sizeof(AnalogSensorData);
      return true;
    } else {
      return false;
    }
  } else {
    return false;
  }
}

/* Send a BFS Bus message. */
void FlightManagementUnit::SendMessage(Message message,std::vector<uint8_t> &Payload) {
  // check that the payload length is within the maximum buffer size
  if (Payload.size() < (kUartBufferMaxSize-headerLength_-checksumLength_)) {
    // header
    Buffer_[0] = header_[0];
    Buffer_[1] = header_[1];
    // message ID
    Buffer_[2] = (uint8_t)message;
    // payload length
    Buffer_[3] = Payload.size() & 0xff;
    Buffer_[4] = Payload.size() >> 8;
    // payload
    std::memcpy(Buffer_+headerLength_,Payload.data(),Payload.size());
    // checksum
    CalcChecksum((size_t)(Payload.size()+headerLength_),Buffer_,Checksum_);
    Buffer_[Payload.size()+headerLength_] = Checksum_[0];
    Buffer_[Payload.size()+headerLength_+1] = Checksum_[1];
    // transmit
    WritePort(Buffer_,Payload.size()+headerLength_+checksumLength_);
  }
}

/* Receive a BFS Bus message. */
bool FlightManagementUnit::ReceiveMessage(Message *message,std::vector<uint8_t> *Payload) {
  int count;
  while ((count=read(FmuFileDesc_,&RxByte_,sizeof(RxByte_)))>0) {
    // header
    if (ParserState_ < 2) {
      if (RxByte_ == header_[ParserState_]) {
        Buffer_[ParserState_] = RxByte_;
        ParserState_++;
      }
    // length
    } else if (ParserState_ == 3) {
      LengthBuffer_[0] = RxByte_;
      Buffer_[ParserState_] = RxByte_;
      ParserState_++;
    } else if (ParserState_ == 4) {
      LengthBuffer_[1] = RxByte_;
      Length_ = ((uint16_t)LengthBuffer_[1] << 8) | LengthBuffer_[0];
      if (Length_ > (kUartBufferMaxSize-headerLength_-checksumLength_)) {
        ParserState_ = 0;
        LengthBuffer_[0] = 0;
        LengthBuffer_[1] = 0;
        Length_ = 0;
        Checksum_[0] = 0;
        Checksum_[1] = 0;
        return false;
      }
      Buffer_[ParserState_] = RxByte_;
      ParserState_++;
    // message ID and payload
    } else if (ParserState_ < (Length_ + headerLength_)) {
      Buffer_[ParserState_] = RxByte_;
      ParserState_++;
    // checksum 0
    } else if (ParserState_ == (Length_ + headerLength_)) {
      CalcChecksum(Length_ + headerLength_,Buffer_,Checksum_);
      if (RxByte_ == Checksum_[0]) {
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
      if (RxByte_ == Checksum_[1]) {
        // message ID
        *message = (Message) Buffer_[2];
        // payload size
        Payload->resize(Length_);
        // payload
        std::memcpy(Payload->data(),Buffer_+headerLength_,Length_);
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
  }
  return false;
}

/* Writes data to serial port. */
void FlightManagementUnit::WritePort(uint8_t* Buffer,size_t BufferSize) {
  int count;
  if ((count=write(FmuFileDesc_,Buffer,BufferSize))<0) {
    throw std::runtime_error("UART failed to write.");
  }
}

/* Computes a two byte checksum. */
void FlightManagementUnit::CalcChecksum(size_t ArraySize, uint8_t *ByteArray, uint8_t *Checksum) {
  Checksum[0] = 0;
  Checksum[1] = 0;
  for (size_t i = 0; i < ArraySize; i++) {
    Checksum[0] += ByteArray[i];
    Checksum[1] += Checksum[0];
  }
}
