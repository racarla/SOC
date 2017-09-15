
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
  bool ReadOperation;
  BfsMessage MessageId;
  uint16_t PayloadSize;
  uint8_t Payload[sizeof(FmuDataPtr->Time_s)+2*sizeof(Voltage)+sizeof(Mpu9250Data)+sizeof(Bme280Data)+FmuDataPtr->Mpu9250Ext.size()*sizeof(Mpu9250Data)+FmuDataPtr->Bme280Ext.size()*sizeof(Bme280Data)+FmuDataPtr->SbusRx.size()*sizeof(SbusRxData)+FmuDataPtr->Gps.size()*sizeof(GpsData)+FmuDataPtr->Pitot.size()*sizeof(PitotData)+FmuDataPtr->Analog.size()*sizeof(AnalogData)+FmuDataPtr->SbusVoltage.size()*sizeof(Voltage)+FmuDataPtr->PwmVoltage.size()*sizeof(Voltage)];
  if (ReadMessage(&ReadOperation,&MessageId,&PayloadSize,Payload)) {
    if ((!ReadOperation)&&(MessageId==kData)&&(PayloadSize==sizeof(Payload))) {
      size_t RxPayloadOffset = 0;
      memcpy(&FmuDataPtr->Time_s,Payload,sizeof(FmuDataPtr->Time_s));
      RxPayloadOffset += sizeof(FmuDataPtr->Time_s);
      memcpy(&FmuDataPtr->InputVoltage,Payload+RxPayloadOffset,sizeof(FmuDataPtr->InputVoltage));
      RxPayloadOffset += sizeof(Voltage);
      memcpy(&FmuDataPtr->RegulatedVoltage,Payload+RxPayloadOffset,sizeof(FmuDataPtr->RegulatedVoltage));
      RxPayloadOffset += sizeof(Voltage);
      memcpy(&FmuDataPtr->Mpu9250,Payload+RxPayloadOffset,sizeof(Mpu9250Data));
      RxPayloadOffset += sizeof(Mpu9250Data);
      memcpy(&FmuDataPtr->Bme280,Payload+RxPayloadOffset,sizeof(Bme280Data));
      RxPayloadOffset += sizeof(Bme280Data);

      for (size_t i=0; i < FmuDataPtr->Vn100.size(); i++) {
        memcpy(&FmuDataPtr->Vn100[i],Payload+RxPayloadOffset,sizeof(Vn100Data));
        RxPayloadOffset += sizeof(Vn100Data);
      }
      for (size_t i=0; i < FmuDataPtr->Vn200.size(); i++) {
        memcpy(&FmuDataPtr->Vn200[i],Payload+RxPayloadOffset,sizeof(Vn200Data));
        RxPayloadOffset += sizeof(Vn200Data);
      }
      for (size_t i=0; i < FmuDataPtr->Mpu9250Ext.size(); i++) {
        memcpy(&FmuDataPtr->Mpu9250Ext[i],Payload+RxPayloadOffset,sizeof(Mpu9250Data));
        RxPayloadOffset += sizeof(Mpu9250Data);
      }
      for (size_t i=0; i < FmuDataPtr->Bme280Ext.size(); i++) {
        memcpy(&FmuDataPtr->Bme280Ext[i],Payload+RxPayloadOffset,sizeof(Bme280Data));
        RxPayloadOffset += sizeof(Bme280Data);
      }
      for (size_t i=0; i < FmuDataPtr->SbusRx.size(); i++) {
        memcpy(&FmuDataPtr->SbusRx[i],Payload+RxPayloadOffset,sizeof(SbusRxData));
        RxPayloadOffset += sizeof(SbusRxData);
      }
      for (size_t i=0; i < FmuDataPtr->Gps.size(); i++) {
        memcpy(&FmuDataPtr->Gps[i],Payload+RxPayloadOffset,sizeof(GpsData));
        RxPayloadOffset += sizeof(GpsData);
      }
      for (size_t i=0; i < FmuDataPtr->Pitot.size(); i++) {
        memcpy(&FmuDataPtr->Pitot[i],Payload+RxPayloadOffset,sizeof(PitotData));
        RxPayloadOffset += sizeof(PitotData);
      }
      for (size_t i=0; i < FmuDataPtr->Pressure.size(); i++) {
        memcpy(&FmuDataPtr->Pressure[i],Payload+RxPayloadOffset,sizeof(PressureData));
        RxPayloadOffset += sizeof(PressureData);
      }
      for (size_t i=0; i < FmuDataPtr->Analog.size(); i++) {
        memcpy(&FmuDataPtr->Analog[i],Payload+RxPayloadOffset,sizeof(AnalogData));
        RxPayloadOffset += sizeof(AnalogData);
      }
      for (size_t i=0; i < FmuDataPtr->SbusVoltage.size(); i++) {
        memcpy(&FmuDataPtr->SbusVoltage[i],Payload+RxPayloadOffset,sizeof(Voltage));
        RxPayloadOffset += sizeof(Voltage);
      }
      for (size_t i=0; i < FmuDataPtr->PwmVoltage.size(); i++) {
        memcpy(&FmuDataPtr->PwmVoltage[i],Payload+RxPayloadOffset,sizeof(Voltage));
        RxPayloadOffset += sizeof(Voltage);
      }
      return true;
    }
  } else {
    return false;
  }
  return false;
}

/* Writes a Bfs Bus message. */
void Fmu::WriteMessage(BfsMessage MessageId,uint16_t PayloadSize,uint8_t *Payload) {
  uint8_t Buffer[PayloadSize + BfsHeaderSize];
  BuildBfsMessage(false,MessageId,PayloadSize,Payload,Buffer);
  WritePort(PayloadSize + BfsHeaderSize,Buffer);
}

/* Read BFS Bus messages. */
bool Fmu::ReadMessage(bool *ReadOperation,BfsMessage *MessageId,uint16_t *PayloadSize,uint8_t *Payload) {
  int count;
  uint8_t buffer[1];
  uint8_t SrcAddr, DestAddr;
  if ((count=read(FmuFileDesc_,buffer,sizeof(buffer)))>0) {
    if (ParseBfsMessage(buffer[0],ReadOperation,MessageId,PayloadSize,Payload)) {
      return true;
    }
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
void Fmu::BuildBfsMessage(bool ReadOperation,BfsMessage MessageId,uint16_t PayloadSize,uint8_t *Payload,uint8_t *TxBuffer) {
  // header
  TxBuffer[0] = BfsHeader[0];
  TxBuffer[1] = BfsHeader[1];

  // ReadOperation
  TxBuffer[2] = ReadOperation;

  // message ID
  uint16_t message = (uint16_t) MessageId;
  TxBuffer[3] = message & 0xff;
  TxBuffer[4] = message >> 8;
  // length
  TxBuffer[5] = PayloadSize & 0xff;
  TxBuffer[6] = PayloadSize >> 8;
  // payload
  for (uint16_t i = 0; i < PayloadSize; i++) {
    TxBuffer[i + 7] = Payload[i];
  }
  // checksum
  uint8_t Checksum[2];
  CalcChecksum(PayloadSize + 7,TxBuffer,Checksum);
  TxBuffer[PayloadSize + 7] = Checksum[0];
  TxBuffer[PayloadSize + 8] = Checksum[1];
}

/* Parse a BFS Bus message. */
bool Fmu::ParseBfsMessage(uint8_t RxBuffer,bool *ReadOperation,BfsMessage *MessageId,uint16_t *PayloadSize,uint8_t *Payload) {
  static uint16_t ParserState = 0;
  static uint8_t Checksum[2] = {0,0};
  static uint8_t MessageIdBuffer[2] = {0,0};
  static uint8_t PayloadSizeBuffer[2] = {0,0};
  static bool ReadOp = false;
  static uint16_t pSize = 0;
  static BfsMessage Message = kConfig;
  static uint8_t Buffer[4096] = {0};
  if (ParserState < 2) { // header
    if (RxBuffer == BfsHeader[ParserState]) {
      ChecksumIteration(RxBuffer, Checksum);
      ParserState++;
    } else {
      Checksum[0] = 0;
      Checksum[1] = 0;
      MessageIdBuffer[0] = 0;
      MessageIdBuffer[1] = 0;
      PayloadSizeBuffer[0] = 0;
      PayloadSizeBuffer[1] = 0;
      ParserState = 0;
    }
  } else if (ParserState == 2) { // read operation
    ReadOp = RxBuffer;
    ChecksumIteration(RxBuffer, Checksum);
    ParserState++;
  } else if (ParserState == 3) { // message ID
    MessageIdBuffer[0] = RxBuffer;
    ChecksumIteration(RxBuffer, Checksum);
    ParserState++;
  } else if (ParserState == 4) { // message ID
    MessageIdBuffer[1] = RxBuffer;
    Message = (BfsMessage)(((uint16_t)MessageIdBuffer[1] << 8) | MessageIdBuffer[0]);
    ChecksumIteration(RxBuffer, Checksum);
    ParserState++;
  } else if (ParserState == 5) { // payload length
    PayloadSizeBuffer[0] = RxBuffer;
    ChecksumIteration(RxBuffer, Checksum);
    ParserState++;
  } else if (ParserState == 6) { // payload length
    PayloadSizeBuffer[1] = RxBuffer;
    pSize = ((uint16_t)PayloadSizeBuffer[1] << 8) | PayloadSizeBuffer[0];
    ChecksumIteration(RxBuffer, Checksum);
    ParserState++;
  } else if ((ParserState>=7)&&(ParserState<(pSize+7))) { // payload
    Buffer[ParserState-7] = RxBuffer;
    ChecksumIteration(RxBuffer, Checksum);
    ParserState++;
  } else if (ParserState==(pSize+7)) {
    if (RxBuffer == Checksum[0]) {
      ParserState++;
    } else {
      Checksum[0] = 0;
      Checksum[1] = 0;
      MessageIdBuffer[0] = 0;
      MessageIdBuffer[1] = 0;
      PayloadSizeBuffer[0] = 0;
      PayloadSizeBuffer[1] = 0;
      ParserState = 0;
      return false;
    }
  } else if (ParserState==(pSize+8)) {
    if (RxBuffer == Checksum[1]) {
      *ReadOperation = ReadOp;
      *MessageId = Message;
      *PayloadSize = pSize;
      memcpy(Payload,Buffer,pSize);
      Checksum[0] = 0;
      Checksum[1] = 0;
      MessageIdBuffer[0] = 0;
      MessageIdBuffer[1] = 0;
      PayloadSizeBuffer[0] = 0;
      PayloadSizeBuffer[1] = 0;
      ParserState = 0;
      return true;
    } else {
      Checksum[0] = 0;
      Checksum[1] = 0;
      MessageIdBuffer[0] = 0;
      MessageIdBuffer[1] = 0;
      PayloadSizeBuffer[0] = 0;
      PayloadSizeBuffer[1] = 0;
      ParserState = 0;
      return false;
    }
  }
  return false;
}
