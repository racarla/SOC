
#include "fmu.hxx"

Fmu::Fmu() {
  OpenPort();
}

/* Opens port to communicate with FMU. */
void Fmu::OpenPort() {
  std::cout << "Opening UART port with FMU...";
  if ((FmuFileDesc_=open(FmuPort,O_RDWR|O_NOCTTY|O_NONBLOCK))<0) {
    throw std::runtime_error("UART failed to open.");
  }
  struct termios Options;
  tcgetattr(FmuFileDesc_,&Options);
  Options.c_cflag = FmuBaud | CS8 | CREAD | CLOCAL;
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

/* Writes data to serial port. */
void Fmu::WritePort(size_t BufferSize,uint8_t* Buffer) {
  int count;
  if ((count=write(FmuFileDesc_,Buffer,BufferSize))<0) {
    throw std::runtime_error("UART failed to write.");
  }
}

/* Get sensor data from FMU. */
bool Fmu::GetSensorData(FmuData *FmuDataPtr) {
  BfsMessage MessageId;
  uint16_t PayloadSize;
  size_t PayloadLocation = 0;
  uint8_t Payload[sizeof(FmuDataPtr->Time_us)+2*sizeof(Voltage)+sizeof(Mpu9250Data)+sizeof(Bme280Data)+FmuDataPtr->Mpu9250Ext.size()*sizeof(Mpu9250Data)+FmuDataPtr->Bme280Ext.size()*sizeof(Bme280Data)+FmuDataPtr->SbusRx.size()*sizeof(SbusRxData)+FmuDataPtr->Gps.size()*sizeof(GpsData)+FmuDataPtr->Pitot.size()*sizeof(PitotData)+FmuDataPtr->PressureTransducer.size()*sizeof(PressureData)+FmuDataPtr->Analog.size()*sizeof(AnalogData)+FmuDataPtr->SbusVoltage.size()*sizeof(Voltage)+FmuDataPtr->PwmVoltage.size()*sizeof(Voltage)];
  if (ReadMessage(&MessageId,&PayloadSize,Payload)) {
    if ((MessageId==kData)&&(PayloadSize==sizeof(Payload))) {
errStatus_.cntSuccess++;
      memcpy(&FmuDataPtr->Time_us,Payload,sizeof(FmuDataPtr->Time_us));
      PayloadLocation += sizeof(FmuDataPtr->Time_us);
      memcpy(&FmuDataPtr->InputVoltage,Payload+PayloadLocation,sizeof(FmuDataPtr->InputVoltage));
      PayloadLocation += sizeof(FmuDataPtr->InputVoltage);
      memcpy(&FmuDataPtr->RegulatedVoltage,Payload+PayloadLocation,sizeof(FmuDataPtr->RegulatedVoltage));
      PayloadLocation += sizeof(FmuDataPtr->RegulatedVoltage);
      memcpy(&FmuDataPtr->Mpu9250,Payload+PayloadLocation,sizeof(Mpu9250Data));
      PayloadLocation += sizeof(Mpu9250Data);
      memcpy(&FmuDataPtr->Bme280,Payload+PayloadLocation,sizeof(Bme280Data));
      PayloadLocation += sizeof(Bme280Data);
      memcpy(&FmuDataPtr->Mpu9250Ext[0],Payload+PayloadLocation,FmuDataPtr->Mpu9250Ext.size()*sizeof(Mpu9250Data));
      PayloadLocation += FmuDataPtr->Mpu9250Ext.size()*sizeof(Mpu9250Data);
      memcpy(&FmuDataPtr->Bme280Ext[0],Payload+PayloadLocation,FmuDataPtr->Bme280Ext.size()*sizeof(Bme280Data));
      PayloadLocation += FmuDataPtr->Bme280Ext.size()*sizeof(Bme280Data);
      memcpy(&FmuDataPtr->SbusRx[0],Payload+PayloadLocation,FmuDataPtr->SbusRx.size()*sizeof(SbusRxData));
      PayloadLocation += FmuDataPtr->SbusRx.size()*sizeof(SbusRxData);
      memcpy(&FmuDataPtr->Gps[0],Payload+PayloadLocation,FmuDataPtr->Gps.size()*sizeof(GpsData));
      PayloadLocation += FmuDataPtr->Gps.size()*sizeof(GpsData);
      memcpy(&FmuDataPtr->Pitot[0],Payload+PayloadLocation,FmuDataPtr->Pitot.size()*sizeof(PitotData));
      PayloadLocation += FmuDataPtr->Pitot.size()*sizeof(PitotData);
      memcpy(&FmuDataPtr->PressureTransducer[0],Payload+PayloadLocation,FmuDataPtr->PressureTransducer.size()*sizeof(PressureData));
      PayloadLocation += FmuDataPtr->PressureTransducer.size()*sizeof(PressureData);
      memcpy(&FmuDataPtr->Analog[0],Payload+PayloadLocation,FmuDataPtr->Analog.size()*sizeof(AnalogData));
      PayloadLocation += FmuDataPtr->Analog.size()*sizeof(AnalogData);
      memcpy(&FmuDataPtr->SbusVoltage[0],Payload+PayloadLocation,FmuDataPtr->SbusVoltage.size()*sizeof(Voltage));
      PayloadLocation += FmuDataPtr->SbusVoltage.size()*sizeof(Voltage);
      memcpy(&FmuDataPtr->PwmVoltage[0],Payload+PayloadLocation,FmuDataPtr->PwmVoltage.size()*sizeof(Voltage));
      PayloadLocation += FmuDataPtr->PwmVoltage.size()*sizeof(Voltage);
            
      return true;
    } else {
      errStatus_.cntReadErr++;
    }
  } else {
    errStatus_.cntPayloadErr++;
    return false;
  }
  return false;
}

/* Writes a Bfs Bus message. */
void Fmu::WriteMessage(BfsMessage MessageId,uint16_t PayloadSize,uint8_t *Payload) {
  uint8_t Buffer[PayloadSize + BfsHeaderSize];
  uint16_t BufferSize;
  BuildBfsMessage(MessageId,PayloadSize,Payload,&BufferSize,Buffer);
  WritePort(BufferSize,Buffer);
}

/* Read BFS Bus messages. */
bool Fmu::ReadMessage(BfsMessage *MessageId,uint16_t *PayloadSize,uint8_t *Payload) {
  int count;
  uint8_t buffer[1];
  if ((count=read(FmuFileDesc_,buffer,sizeof(buffer)))>0) {
    if (ParseBfsMessage(buffer[0],MessageId,PayloadSize,Payload)) {
      return true;
    } else {
      errStatus_.cntParseErr++;
    }
  } else {
    errStatus_.cntUnavailErr++;
  }
  return false;
}


/* Calculate a 2 byte checksum given a byte array. */
void Fmu::CalcChecksum(size_t ArraySize, uint8_t *ByteArray, uint8_t *Checksum) {
  Checksum[0] = 0;
  Checksum[1] = 0;
  for (size_t i = 0; i < ArraySize; i++) {
    ChecksumIteration(ByteArray[i], Checksum);
  }
}

/* Iterates the checksum state. */
void Fmu::ChecksumIteration(uint8_t Data, uint8_t *Checksum) {
  Checksum[0] += Data;
  Checksum[1] += Checksum[0];
}

/* Build a BFS Bus message to send. */
void Fmu::BuildBfsMessage(BfsMessage MessageId,uint16_t PayloadSize,uint8_t *Payload,uint16_t *TxBufferSize,uint8_t *TxBuffer) {
  uint8_t Checksum[2];
  // header
  for (size_t i=0; i < sizeof(BfsHeader); i++) {
    TxBuffer[i] = BfsHeader[i];
  }
  // message ID
  TxBuffer[2] = (uint8_t) MessageId;
  // payload length
  TxBuffer[3] = PayloadSize & 0xff;
  TxBuffer[4] = PayloadSize >> 8;
  // payload
  for (size_t i=0; i < PayloadSize; i++) {
    TxBuffer[5+i] = Payload[i];
  }
  // checksum
  CalcChecksum(PayloadSize+5,TxBuffer,Checksum);
  TxBuffer[PayloadSize+5] = Checksum[0];
  TxBuffer[PayloadSize+6] = Checksum[1];
  // transmit buffer size
  *TxBufferSize = PayloadSize + 7;
}

/* Parse a BFS Bus message. */
bool Fmu::ParseBfsMessage(uint8_t RxBuffer,BfsMessage *MessageId,uint16_t *PayloadSize,uint8_t *Payload) {
  static uint16_t ParserState = 0;
  static uint8_t Checksum[2] = {0,0};
  static uint8_t PayloadSizeBuffer[2] = {0,0};
  static uint16_t pSize = 0;
  static BfsMessage Message = kConfig;
  static uint8_t Buffer[4096] = {0}; // Reduced to 512 from 4096;
  if (ParserState < 2) { // header
    if (RxBuffer == BfsHeader[ParserState]) {
      ChecksumIteration(RxBuffer, Checksum);
      ParserState++;
    } else {
      Checksum[0] = 0;
      Checksum[1] = 0;
      PayloadSizeBuffer[0] = 0;
      PayloadSizeBuffer[1] = 0;
      ParserState = 0;      
      errStatus_.cntHeaderErr++;
      return false;
    }
  } else if (ParserState == 2) { // message ID
    Message = (BfsMessage)RxBuffer;
    if (Message==kData) {
      ChecksumIteration(RxBuffer, Checksum);
      ParserState++;
    } else {
      Checksum[0] = 0;
      Checksum[1] = 0;
      PayloadSizeBuffer[0] = 0;
      PayloadSizeBuffer[1] = 0;
      ParserState = 0;      
      errStatus_.cntMessageErr++;
      return false;
    }
  } else if (ParserState == 3) { // payload length
    PayloadSizeBuffer[0] = RxBuffer;
    ChecksumIteration(RxBuffer, Checksum);
    ParserState++;
  } else if (ParserState == 4) { // payload length
    PayloadSizeBuffer[1] = RxBuffer;
    pSize = ((uint16_t)PayloadSizeBuffer[1] << 8) | PayloadSizeBuffer[0];
  
    if (pSize < 400) {
      ChecksumIteration(RxBuffer, Checksum);
      ParserState++;
    } else {
      Checksum[0] = 0;
      Checksum[1] = 0;
      PayloadSizeBuffer[0] = 0;
      PayloadSizeBuffer[1] = 0;
      ParserState = 0;
      errStatus_.cntSizeErr++;
      return false;
    }
    
  } else if ((ParserState>=5)&&(ParserState<(pSize+5))) { // payload
    Buffer[ParserState-5] = RxBuffer;
    ChecksumIteration(RxBuffer, Checksum);
    ParserState++;
    
  } else if (ParserState==(pSize+5)) {
    if (RxBuffer == Checksum[0]) {
      ParserState++;
    } else {
      Checksum[0] = 0;
      Checksum[1] = 0;
      PayloadSizeBuffer[0] = 0;
      PayloadSizeBuffer[1] = 0;
      ParserState = 0;
      errStatus_.cntChecksumErr++;
      return false;
    }
  } else if (ParserState==(pSize+6)) {
    if (RxBuffer == Checksum[1]) {
      *MessageId = Message;
      *PayloadSize = pSize;
      memcpy(Payload,Buffer,pSize);
      Checksum[0] = 0;
      Checksum[1] = 0;
      PayloadSizeBuffer[0] = 0;
      PayloadSizeBuffer[1] = 0;
      ParserState = 0;
      return true;
    } else {
      Checksum[0] = 0;
      Checksum[1] = 0;
      PayloadSizeBuffer[0] = 0;
      PayloadSizeBuffer[1] = 0;
      ParserState = 0;
      errStatus_.cntChecksumErr++;
      return false;
    }
  }
  return false;
}
