#include "telemetry.hxx"

/* Opens a socket for telemetry */
TelemetryClient::TelemetryClient() {
  TelemetrySocket_ = socket(AF_INET, SOCK_DGRAM, 0);
  TelemetryServer_.sin_family = AF_INET;
  TelemetryServer_.sin_port = htons(TelemetryPort_);
  TelemetryServer_.sin_addr.s_addr = inet_addr("127.0.0.1");
}

void TelemetryClient::Configure(const rapidjson::Value& Config,DefinitionTree *DefinitionTreePtr) {
  std::vector<uint8_t> Buffer;
  if (Config.HasMember("Uart")) {
    Uart = Config["Uart"].GetString();
    Buffer.resize(Uart.size());
    memcpy(Buffer.data(),Uart.data(),Buffer.size());
    SendPacket(UartPacket,Buffer);
  } else {
    throw std::runtime_error(std::string("ERROR")+_RootPath+std::string(": Uart not specified in configuration."));
  }
  if (Config.HasMember("Baud")) {
    Baud = Config["Baud"].GetUint64();
    Buffer.resize(sizeof(Baud));
    memcpy(Buffer.data(),&Baud,Buffer.size());
    SendPacket(BaudPacket,Buffer);
  } else {
    throw std::runtime_error(std::string("ERROR")+_RootPath+std::string(": Baud not specified in configuration."));
  }
  if (Config.HasMember("Time")) {
    DataPtr_.Time.Time_us = DefinitionTreePtr->GetValuePtr<uint64_t*>(Config["Time"].GetString());
    useTime = true;
  }
  if (Config.HasMember("Static-Pressure")) {
    std::string Sensor = Config["Static-Pressure"].GetString();
    DataPtr_.StaticPress.Pressure_Pa = DefinitionTreePtr->GetValuePtr<float*>(Sensor+"/Pressure_Pa");
    DataPtr_.StaticPress.Temperature_C = DefinitionTreePtr->GetValuePtr<float*>(Sensor+"/Temperature_C");
    useStaticPressure = true;
  }
  if (Config.HasMember("Airspeed")) {
    DataPtr_.Airspeed.Airspeed_ms = DefinitionTreePtr->GetValuePtr<float*>(Config["Airspeed"].GetString());
    useAirspeed = true;
  }
  if (Config.HasMember("Altitude")) {
    DataPtr_.Alt.Alt_m = DefinitionTreePtr->GetValuePtr<float*>(Config["Altitude"].GetString());
    useAlt = true;
  }
  if (Config.HasMember("Filter")) {
    std::string Sensor = Config["Filter"].GetString();
    DataPtr_.Attitude.Ax = DefinitionTreePtr->GetValuePtr<float*>(Sensor+"/AccelX_mss");
    DataPtr_.Attitude.Axb = DefinitionTreePtr->GetValuePtr<float*>(Sensor+"/AccelXBias_mss");
    DataPtr_.Attitude.Ay = DefinitionTreePtr->GetValuePtr<float*>(Sensor+"/AccelY_mss");
    DataPtr_.Attitude.Ayb = DefinitionTreePtr->GetValuePtr<float*>(Sensor+"/AccelYBias_mss");
    DataPtr_.Attitude.Az = DefinitionTreePtr->GetValuePtr<float*>(Sensor+"/AccelZ_mss");
    DataPtr_.Attitude.Azb = DefinitionTreePtr->GetValuePtr<float*>(Sensor+"/AccelZBias_mss");
    DataPtr_.Attitude.Gx = DefinitionTreePtr->GetValuePtr<float*>(Sensor+"/GyroX_rads");
    DataPtr_.Attitude.Gxb = DefinitionTreePtr->GetValuePtr<float*>(Sensor+"/GyroXBias_rads");
    DataPtr_.Attitude.Gy = DefinitionTreePtr->GetValuePtr<float*>(Sensor+"/GyroY_rads");
    DataPtr_.Attitude.Gyb = DefinitionTreePtr->GetValuePtr<float*>(Sensor+"/GyroYBias_rads");
    DataPtr_.Attitude.Gz = DefinitionTreePtr->GetValuePtr<float*>(Sensor+"/GyroZ_rads");
    DataPtr_.Attitude.Gzb = DefinitionTreePtr->GetValuePtr<float*>(Sensor+"/GyroZBias_rads");
    DataPtr_.Attitude.Pitch = DefinitionTreePtr->GetValuePtr<float*>(Sensor+"/Pitch_rad");
    DataPtr_.Attitude.Roll = DefinitionTreePtr->GetValuePtr<float*>(Sensor+"/Roll_rad");
    DataPtr_.Attitude.Yaw = DefinitionTreePtr->GetValuePtr<float*>(Sensor+"/Yaw_rad");
    DataPtr_.Attitude.Heading = DefinitionTreePtr->GetValuePtr<float*>(Sensor+"/Heading_rad");
    DataPtr_.Attitude.Track = DefinitionTreePtr->GetValuePtr<float*>(Sensor+"/Track_rad");
    DataPtr_.Attitude.Lon = DefinitionTreePtr->GetValuePtr<double*>(Sensor+"/Longitude_rad");
    DataPtr_.Attitude.Lat = DefinitionTreePtr->GetValuePtr<double*>(Sensor+"/Latitude_rad");
    DataPtr_.Attitude.Alt = DefinitionTreePtr->GetValuePtr<double*>(Sensor+"/Altitude_m");
    DataPtr_.Attitude.Vn= DefinitionTreePtr->GetValuePtr<double*>(Sensor+"/NorthVelocity_ms");
    DataPtr_.Attitude.Ve = DefinitionTreePtr->GetValuePtr<double*>(Sensor+"/EastVelocity_ms");
    DataPtr_.Attitude.Vd = DefinitionTreePtr->GetValuePtr<double*>(Sensor+"/DownVelocity_ms");
    useAttitude = true;
  }
  if (Config.HasMember("Gps")) {
    std::string Sensor = Config["Gps"].GetString();
    DataPtr_.Gps.Fix = (bool*)DefinitionTreePtr->GetValuePtr<uint8_t*>(Sensor+"/Fix");
    DataPtr_.Gps.NumberSatellites = DefinitionTreePtr->GetValuePtr<uint8_t*>(Sensor+"/NumberSatellites");
    DataPtr_.Gps.TOW = DefinitionTreePtr->GetValuePtr<uint32_t*>(Sensor+"/TOW");
    DataPtr_.Gps.Year = DefinitionTreePtr->GetValuePtr<uint16_t*>(Sensor+"/Year");
    DataPtr_.Gps.Month = DefinitionTreePtr->GetValuePtr<uint8_t*>(Sensor+"/Month");
    DataPtr_.Gps.Day = DefinitionTreePtr->GetValuePtr<uint8_t*>(Sensor+"/Day");
    DataPtr_.Gps.Hour = DefinitionTreePtr->GetValuePtr<uint8_t*>(Sensor+"/Hour");
    DataPtr_.Gps.Min = DefinitionTreePtr->GetValuePtr<uint8_t*>(Sensor+"/Minute");
    DataPtr_.Gps.Sec = DefinitionTreePtr->GetValuePtr<uint8_t*>(Sensor+"/Second");
    DataPtr_.Gps.Lat = DefinitionTreePtr->GetValuePtr<double*>(Sensor+"/Latitude_rad");
    DataPtr_.Gps.Lon = DefinitionTreePtr->GetValuePtr<double*>(Sensor+"/Longitude_rad");
    DataPtr_.Gps.Alt = DefinitionTreePtr->GetValuePtr<double*>(Sensor+"/Altitude_m");
    DataPtr_.Gps.Vn = DefinitionTreePtr->GetValuePtr<double*>(Sensor+"/NorthVelocity_ms");
    DataPtr_.Gps.Ve = DefinitionTreePtr->GetValuePtr<double*>(Sensor+"/EastVelocity_ms");
    DataPtr_.Gps.Vd = DefinitionTreePtr->GetValuePtr<double*>(Sensor+"/DownVelocity_ms");
    DataPtr_.Gps.HAcc = DefinitionTreePtr->GetValuePtr<double*>(Sensor+"/HorizontalAccuracy_m");
    DataPtr_.Gps.VAcc = DefinitionTreePtr->GetValuePtr<double*>(Sensor+"/VerticalAccuracy_m");
    DataPtr_.Gps.SAcc = DefinitionTreePtr->GetValuePtr<double*>(Sensor+"/VelocityAccuracy_ms");
    DataPtr_.Gps.pDOP = DefinitionTreePtr->GetValuePtr<double*>(Sensor+"/pDOP");
    useGps = true;
  }
  if (Config.HasMember("Imu")) {
    std::string Sensor = Config["Imu"].GetString();
    DataPtr_.Imu.Ax = DefinitionTreePtr->GetValuePtr<float*>(Sensor+"/AccelX_mss");
    DataPtr_.Imu.Ay = DefinitionTreePtr->GetValuePtr<float*>(Sensor+"/AccelY_mss");
    DataPtr_.Imu.Az = DefinitionTreePtr->GetValuePtr<float*>(Sensor+"/AccelZ_mss");
    DataPtr_.Imu.Gx = DefinitionTreePtr->GetValuePtr<float*>(Sensor+"/GyroX_rads");
    DataPtr_.Imu.Gy = DefinitionTreePtr->GetValuePtr<float*>(Sensor+"/GyroY_rads");
    DataPtr_.Imu.Gz = DefinitionTreePtr->GetValuePtr<float*>(Sensor+"/GyroZ_rads");
    DataPtr_.Imu.Temperature_C = DefinitionTreePtr->GetValuePtr<float*>(Sensor+"/Temperature_C");
    useImu = true;
  }
  if (Config.HasMember("Sbus")) {
    std::string Sensor = Config["Sbus"].GetString();
    DataPtr_.Sbus.FailSafe = (bool*) DefinitionTreePtr->GetValuePtr<uint8_t*>(Sensor+"/FailSafe");
    DataPtr_.Sbus.LostFrames = DefinitionTreePtr->GetValuePtr<uint64_t*>(Sensor+"/LostFrames");
    for (size_t j=0; j < 16; j++) {
      DataPtr_.Sbus.Channels[j] = DefinitionTreePtr->GetValuePtr<float*>(Sensor+"/Channels/"+std::to_string(j));
    }
    useSbus = true;
  }
}

void TelemetryClient::Send() {
  std::vector<uint8_t> DataPayload;
  if (useTime) {
    Data_.Time.Time_us = *DataPtr_.Time.Time_us;
  }
  if (useStaticPressure) {
    Data_.StaticPress.Pressure_Pa = *DataPtr_.StaticPress.Pressure_Pa;
    Data_.StaticPress.Temperature_C = *DataPtr_.StaticPress.Temperature_C;
  }
  if (useAirspeed) {
    Data_.Airspeed.Airspeed_ms = *DataPtr_.Airspeed.Airspeed_ms;
  }
  if (useAlt) {
    Data_.Alt.Alt_m = *DataPtr_.Alt.Alt_m;
  }
  if (useAttitude) {
    Data_.Attitude.Ax = *DataPtr_.Attitude.Ax;
    Data_.Attitude.Axb = *DataPtr_.Attitude.Axb;
    Data_.Attitude.Ay = *DataPtr_.Attitude.Ay;
    Data_.Attitude.Ayb = *DataPtr_.Attitude.Ayb;
    Data_.Attitude.Az = *DataPtr_.Attitude.Az;
    Data_.Attitude.Azb = *DataPtr_.Attitude.Azb;
    Data_.Attitude.Gx = *DataPtr_.Attitude.Gx;
    Data_.Attitude.Gxb = *DataPtr_.Attitude.Gxb;
    Data_.Attitude.Gy = *DataPtr_.Attitude.Gy;
    Data_.Attitude.Gyb = *DataPtr_.Attitude.Gyb;
    Data_.Attitude.Gz = *DataPtr_.Attitude.Gz;
    Data_.Attitude.Gzb = *DataPtr_.Attitude.Gzb;
    Data_.Attitude.Pitch = *DataPtr_.Attitude.Pitch;
    Data_.Attitude.Roll = *DataPtr_.Attitude.Roll;
    Data_.Attitude.Yaw = *DataPtr_.Attitude.Yaw;
    Data_.Attitude.Heading = *DataPtr_.Attitude.Heading;
    Data_.Attitude.Track = *DataPtr_.Attitude.Track;
    Data_.Attitude.Lon = *DataPtr_.Attitude.Lon;
    Data_.Attitude.Lat = *DataPtr_.Attitude.Lat;
    Data_.Attitude.Alt = *DataPtr_.Attitude.Alt;
    Data_.Attitude.Vn= *DataPtr_.Attitude.Vn;
    Data_.Attitude.Ve = *DataPtr_.Attitude.Ve;
    Data_.Attitude.Vd = *DataPtr_.Attitude.Vd;
  }
  if (useGps) {
    Data_.Gps.Fix = *DataPtr_.Gps.Fix;
    Data_.Gps.NumberSatellites = *DataPtr_.Gps.NumberSatellites;
    Data_.Gps.TOW = *DataPtr_.Gps.TOW;
    Data_.Gps.Year = *DataPtr_.Gps.Year;
    Data_.Gps.Month = *DataPtr_.Gps.Month;
    Data_.Gps.Day = *DataPtr_.Gps.Day;
    Data_.Gps.Hour = *DataPtr_.Gps.Hour;
    Data_.Gps.Min = *DataPtr_.Gps.Min;
    Data_.Gps.Sec = *DataPtr_.Gps.Sec;
    Data_.Gps.Lat = *DataPtr_.Gps.Lat;
    Data_.Gps.Lon = *DataPtr_.Gps.Lon;
    Data_.Gps.Alt = *DataPtr_.Gps.Alt;
    Data_.Gps.Vn = *DataPtr_.Gps.Vn;
    Data_.Gps.Ve = *DataPtr_.Gps.Ve;
    Data_.Gps.Vd = *DataPtr_.Gps.Vd;
    Data_.Gps.HAcc = *DataPtr_.Gps.HAcc;
    Data_.Gps.VAcc = *DataPtr_.Gps.VAcc;
    Data_.Gps.SAcc = *DataPtr_.Gps.SAcc;
    Data_.Gps.pDOP = *DataPtr_.Gps.pDOP;  
  }
  if (useImu) {
    Data_.Imu.Ax = *DataPtr_.Imu.Ax;
    Data_.Imu.Ay = *DataPtr_.Imu.Ay;
    Data_.Imu.Az = *DataPtr_.Imu.Az;
    Data_.Imu.Gx = *DataPtr_.Imu.Gx;
    Data_.Imu.Gy = *DataPtr_.Imu.Gy;
    Data_.Imu.Gz = *DataPtr_.Imu.Gz;
    Data_.Imu.Temperature_C = *DataPtr_.Imu.Temperature_C;
  }
  if (useSbus) {
    Data_.Sbus.FailSafe = *DataPtr_.Sbus.FailSafe;
    Data_.Sbus.LostFrames = *DataPtr_.Sbus.LostFrames;
    for (size_t j=0; j < 16; j++) {
      Data_.Sbus.Channels[j] = *DataPtr_.Sbus.Channels[j];
    }
  }
  DataPayload.resize(sizeof(Data));
  memcpy(DataPayload.data(),&Data_,DataPayload.size());
  SendPacket(DataPacket,DataPayload);
}

/* Sends byte buffer given meta data */
void TelemetryClient::SendPacket(PacketType_ Type, std::vector<uint8_t> &Buffer) {
  std::vector<uint8_t> TelemBuffer;
  TelemBuffer.resize(headerLength_ + Buffer.size() + checksumLength_);
  // header
  TelemBuffer[0] = header_[0];
  TelemBuffer[1] = header_[1];
  // data source
  TelemBuffer[2] = (uint8_t) Type;
  // data length
  TelemBuffer[3] = Buffer.size() & 0xff;
  TelemBuffer[4] = Buffer.size() >> 8;
  // payload
  std::memcpy(TelemBuffer.data()+headerLength_,Buffer.data(),Buffer.size());
  // checksum
  CalcChecksum((size_t)(Buffer.size()+headerLength_),TelemBuffer.data(),Checksum_);
  TelemBuffer[Buffer.size()+headerLength_] = Checksum_[0];
  TelemBuffer[Buffer.size()+headerLength_+1] = Checksum_[1];
  // write to UDP
  sendto(TelemetrySocket_,TelemBuffer.data(),TelemBuffer.size(),0,(struct sockaddr *)&TelemetryServer_,sizeof(TelemetryServer_));
}

/* Computes a two byte checksum. */
void TelemetryClient::CalcChecksum(size_t ArraySize, uint8_t *ByteArray, uint8_t *Checksum) {
  Checksum[0] = 0;
  Checksum[1] = 0;
  for (size_t i = 0; i < ArraySize; i++) {
    Checksum[0] += ByteArray[i];
    Checksum[1] += Checksum[0];
  }
}

TelemetryServer::TelemetryServer() {
  TelemetrySocket_ = socket(AF_INET, SOCK_DGRAM, 0);
  TelemetryServer_.sin_family = AF_INET;
  TelemetryServer_.sin_port = htons(TelemetryPort_);
  TelemetryServer_.sin_addr.s_addr = inet_addr("127.0.0.1");
  if (bind(TelemetrySocket_, (struct sockaddr *) &TelemetryServer_,sizeof(TelemetryServer_)) < 0) {
    throw std::runtime_error("Error binding to UDP port.");
  }
  Buffer.resize(kUartBufferMaxSize);
}

void TelemetryServer::ReceivePacket() {
  PacketType_ Type;
  std::vector<uint8_t> Payload;
  ssize_t MessageSize = recv(TelemetrySocket_,Buffer.data(),Buffer.size(),0);
  if (MessageSize > 0) {
    for (size_t i=0; i < MessageSize; i++) {
      if (ParseMessage(Buffer[i],&Type,&Payload)) { 
        if (Type == UartPacket) {
          Uart.resize(Payload.size());
          for (size_t j=0; j < Payload.size(); j++) {
            Uart[j] = (char) Payload[j];
          }
          rxUart = true;
        }
        if (Type == BaudPacket) {
          memcpy(&Baud,Payload.data(),sizeof(Baud));
          rxBaud = true;
        }
        if (Type == DataPacket) {
          memcpy(&Data_,Payload.data(),sizeof(Data_));
          update(Data_);
        }
      }
    }
  }
  if ((rxUart)&&(rxBaud)&&(!uartLatch)) {
    if ((FileDesc_=open(Uart.c_str(),O_RDWR|O_NOCTTY))<0) {
      throw std::runtime_error(std::string("ERROR")+std::string(": UART failed to open."));
    } else {
      std::cout << "Starting telemetry UART" << std::endl;
      std::cout << "UART: " << Uart << std::endl;
      std::cout << "Baud: " << Baud << std::endl;
    }
    int baud_bits = B115200;
    if ( Baud == 115200 ) {
	baud_bits = B115200;
    } else if ( Baud == 230400 ) {
	baud_bits = B230400;
    } else if ( Baud == 500000 ) {
	baud_bits = B500000;
     } else {
	printf("unsupported baud rate = %d\n", Baud);
    }
    struct termios Options;
    tcgetattr(FileDesc_,&Options);
    Options.c_cflag = baud_bits | CS8 | CREAD | CLOCAL;
    Options.c_iflag = IGNPAR;
    Options.c_oflag = 0;
    Options.c_lflag = 0;
    Options.c_cc[VTIME] = 0;
    Options.c_cc[VMIN] = 0;
    tcflush(FileDesc_,TCIFLUSH);
    tcsetattr(FileDesc_,TCSANOW,&Options);
    fcntl(FileDesc_,F_SETFL,O_NONBLOCK);
    uartLatch = true;
  }
}

bool TelemetryServer::ParseMessage(uint8_t byte,PacketType_ *message,std::vector<uint8_t> *Payload) {
    RxByte_ = byte;
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
        *message = (PacketType_) Buffer_[2];
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
  return false;
}

/* Computes a two byte checksum. */
void TelemetryServer::CalcChecksum(size_t ArraySize, uint8_t *ByteArray, uint8_t *Checksum) {
  Checksum[0] = 0;
  Checksum[1] = 0;
  for (size_t i = 0; i < ArraySize; i++) {
    Checksum[0] += ByteArray[i];
    Checksum[1] += Checksum[0];
  }
}

void TelemetryServer :: generate_cksum(uint8_t id, uint8_t size, uint8_t * buf, uint8_t & cksum0, uint8_t & cksum1)
{
   cksum0 = 0;
   cksum1 = 0;

   cksum0 += id;
   cksum1 += cksum0;

   cksum0 += size;
   cksum1 += cksum0;

   for (int i = 0; i < size; i++)
   {
      cksum0 += buf[i];
      cksum1 += cksum0;
   }

}
void TelemetryServer :: sending_packs(uint8_t * package, uint8_t IDnum, uint8_t size)
{
   uint8_t buf[4];
   uint8_t checksum0;
   uint8_t checksum1;

   generate_cksum(IDnum, size, package, checksum0, checksum1);

   buf[0] = 147;
   buf[1] = 224;
   buf[2] = IDnum;
   buf[3] = size;

   write(FileDesc_, buf, 4);
   write(FileDesc_, package, size);
   
   buf[0] = checksum0;
   buf[1] = checksum1;
   
   write(FileDesc_, buf, 2);
}

   void TelemetryServer :: update(const Data &DataRef)
   {

   count = count+1;

   if (count<=5){
      return;
   }
 
   count = 0;

   gpsPack gps1; 
   gps1.index = 0; 
   gps1.timestamp = DataRef.Time.Time_us/1000000.0;
   gps1.lat_deg = DataRef.Gps.Lat*(180/M_PI);//pi?
   gps1.long_deg = DataRef.Gps.Lon*(180/M_PI);
   gps1.alt_m = DataRef.Gps.Alt;
   gps1.vn_ms = DataRef.Gps.Vn*100;
   gps1.ve_ms = DataRef.Gps.Ve*100;
   gps1.vd_ms = DataRef.Gps.Vd*100;
   gps1.unix_time_sec = DataRef.Time.Time_us/1000000.0; 
   gps1.satellites = DataRef.Gps.NumberSatellites;
   gps1.horiz_accuracy_m = DataRef.Gps.HAcc*100;
   gps1.vert_accuracy_m = DataRef.Gps.VAcc*100;
   gps1.pdop = DataRef.Gps.pDOP*100;
   gps1.fixType = DataRef.Gps.Fix;
   
   
   airPack air1;
   air1.index = 0;
   air1.timestamp = DataRef.Time.Time_us/1000000.0;
   air1.pressure_mbar = DataRef.StaticPress.Pressure_Pa/10.0;
   air1.temp_degC = DataRef.StaticPress.Temperature_C*100;
   const double mps2kt = 1.9438444924406046432;
   air1.airspeed_smoothed_kt = DataRef.Airspeed.Airspeed_ms * mps2kt * 100;
   std::cout << "airspeed m/s = " << DataRef.Airspeed.Airspeed_ms << std::endl;
   air1.altitude_smoothed_m = DataRef.Alt.Alt_m;
   air1.altitude_true_m = DataRef.Alt.Alt_m;
   air1.pressure_vertical_speed_fps=1;//maybe find?
   air1.wind_dir_deg=1;//no
   air1.wind_speed_kt=1;//no
   air1.pitot_scale_factor=1;//no
   air1.status = 0;
   
   pilotPack pilot1;
   pilot1.index = 0;
   pilot1.time = DataRef.Time.Time_us/1000000.0;
   pilot1.chan[0] = DataRef.Sbus.Channels[0]*20000;
   pilot1.chan[1] = DataRef.Sbus.Channels[1]*20000;
   pilot1.chan[2] = DataRef.Sbus.Channels[2]*20000;
   pilot1.chan[3] = DataRef.Sbus.Channels[3]*20000;
   pilot1.chan[4] = DataRef.Sbus.Channels[4]*20000;
   pilot1.chan[5] = DataRef.Sbus.Channels[5]*20000;
   pilot1.chan[6] = DataRef.Sbus.Channels[6]*20000;
   pilot1.chan[7] = 0;
   pilot1.status = 0;
  

   ImunodePack Imunode1;
   Imunode1.index = 0;
   Imunode1.imu_timestamp = DataRef.Time.Time_us/1000000.0;
   Imunode1.p_rad_sec = DataRef.Imu.Gx;
   Imunode1.q_rad_sec = DataRef.Imu.Gy;
   Imunode1.r_rad_sec = DataRef.Imu.Gz;
   Imunode1.ax_mps_sec = DataRef.Imu.Ax;
   Imunode1.ay_mps_sec = DataRef.Imu.Ay;
   Imunode1.az_mps_sec = DataRef.Imu.Az;
   Imunode1.hx = DataRef.Imu.Hx;
   Imunode1.hy = DataRef.Imu.Hy;
   Imunode1.hz = DataRef.Imu.Hz;
   Imunode1.temp_C = DataRef.Imu.Temperature_C;
   Imunode1.status = 0;
   

   filterPack fill1;
   fill1.index = 0;
   fill1.timestamp = DataRef.Time.Time_us/1000000.0;
   fill1.latitude_deg = DataRef.Attitude.Lat*(180/M_PI);
   fill1.longitude_deg = DataRef.Attitude.Lon*(180/M_PI);
   fill1.altitude_m = DataRef.Attitude.Alt;
   fill1.vn_ms = DataRef.Attitude.Vn*100;
   fill1.ve_ms = DataRef.Attitude.Ve*100;
   fill1.vd_ms = DataRef.Attitude.Vd*100;
   fill1.roll_deg = DataRef.Attitude.Roll*10*(180/M_PI);
   fill1.pitch_deg = DataRef.Attitude.Pitch*10*(180/M_PI);
   fill1.heading_deg = DataRef.Attitude.Heading*10*(180/M_PI);
   fill1.p_bias = DataRef.Attitude.Gxb*1000;
   fill1.q_bias = DataRef.Attitude.Gyb*1000;
   fill1.r_bias = DataRef.Attitude.Gzb*1000;
   fill1.ax_bias = DataRef.Attitude.Axb*1000;
   fill1.ay_bias = DataRef.Attitude.Ayb*1000;
   fill1.az_bias = DataRef.Attitude.Azb*1000;
   fill1.sequence_num = 1;
   fill1.status = 0;


   ap_status ap1;
   ap1.index;
   ap1.frame_time;
   ap1.flags; //?
   ap1.groundtrack_deg;
   ap1.roll_deg;
   ap1.Target_msl_ft;
   ap1.ground_m;  
   ap1.pitch_deg;
   ap1.airspeed_kt;
   ap1.flight_timer;
   ap1.target_waypoint_idx;
if( sizeof(fill1.latitude_deg) != 0 && num2 !=0) {
   ap1.wp_lon = DataRef.Attitude.Lon*(180/M_PI);
   ap1.wp_lat = DataRef.Attitude.Lat*(180/M_PI);;
   num2 = 0;
}
   ap1.wp_index;
   ap1.routesize;
   ap1.sequence_num;




 /*  ap1.timestamp = fmuData.Time_us/1000000.0;
   ap1.master_switch;
   ap1.pilot_pass_through;
   ap1.groundtrack_deg;
   ap1.roll_deg;
   ap1.altitude_msl_ft;
   ap1.altitude_ground_m;
   ap1.pitch_deg;
   ap1.airspeed_kt;
   ap1.flight_timer;
   ap1.target_waypoint_idx;
if (fill1.latitude_deg != 0 && num2 != 0) {
   ap1.longitude_deg = navOut.LLA[0]*(180/M_PI);
   ap1.latitude_deg = navOut.LLA[0]*(180/M_PI);
   num2 = 0;
}
   ap1.route_size;
   ap1.sequence_num;
   ap1.index = 65535;
   
   
   numactPack numact1;
   numact1.index; 
   numact1.timestamp = fmuData.Time_us;
   numact1.aileron;
   numact1.elevator;
   numact1.throttle;
   numact1.rudder;
   numact1.channel5;
   numact1.flaps;
   numact1.channel7;
   numact1.channel8;
   numact1.status;

   healthPack health1;
   health1.index;
   health1.frame_time = fmuData.Time_us;
   health1.system_load_avg;
   health1.board_vct;
   health1.extern_volts;
   health1.extern_cell_volts;
   health1.extern_amps;
   health1.dekamah;
   */

   uint8_t IDnum;
   uint8_t size;
//ap 
   IDnum = 32;
   size = sizeof(ap1);
   sending_packs((uint8_t *)(&ap1), IDnum, size);
//GPS BdddfhhhdBHHHB
   IDnum = 26;
   size = sizeof(gps1);
   sending_packs((uint8_t *)(&gps1), IDnum, size);
//airdata BdHhhffhHBBB
   IDnum = 18;
   size = sizeof(air1);
   sending_packs((uint8_t *)(&air1), IDnum, size);
//pilotcontrol BdhhhhhhhhB
   IDnum = 20; 
   size = sizeof(pilot1);
   sending_packs((uint8_t *)(&pilot1), IDnum, size);
//imudata BdfffffffffhB
   IDnum = 17;
   size = sizeof(Imunode1);
   sending_packs((uint8_t *)(&Imunode1), IDnum, size);
//filterdata BdddfhhhhhhhhhhhhBB
   IDnum = 31; 
   size = sizeof(fill1);
   sending_packs((uint8_t *)(&fill1), IDnum, size);
/*
//actdata BdhhHhhhhhB
   size = sizeof(numact1);
   IDnum = 21; 
   sending_packs((uint8_t *)(&numact1), IDnum, size);
//health BdHHHHHH
   size = sizeof(health1);
   IDnum = 19; //change
   sending_packs((uint8_t *)(&health1), IDnum, size);
*/ 
   };
