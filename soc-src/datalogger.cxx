
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

/* Logs the FMU data into the binary file that was created */
void Datalogger::LogFmuData(FmuData FmuDataRef) {
  fwrite(&FmuDataRef.Time_s,sizeof(FmuDataRef.Time_s),1,LogFile_);
  fwrite(&FmuDataRef.InputVoltage,sizeof(Voltage),1,LogFile_);
  fwrite(&FmuDataRef.RegulatedVoltage,sizeof(Voltage),1,LogFile_);
  fwrite(&FmuDataRef.Mpu9250,sizeof(Mpu9250Data),1,LogFile_);
  fwrite(&FmuDataRef.Bme280,sizeof(Bme280Data),1,LogFile_);
  for (size_t i=0; i < FmuDataRef.Mpu9250Ext.size(); i++) {
    fwrite(&FmuDataRef.Mpu9250Ext[i],sizeof(Mpu9250Data),1,LogFile_);
  }
  for (size_t i=0; i < FmuDataRef.Bme280Ext.size(); i++) {
    fwrite(&FmuDataRef.Bme280Ext[i],sizeof(Bme280Data),1,LogFile_);
  }
  for (size_t i=0; i < FmuDataRef.SbusRx.size(); i++) {
    fwrite(&FmuDataRef.SbusRx[i],sizeof(SbusRxData),1,LogFile_);
  }
  for (size_t i=0; i < FmuDataRef.Gps.size(); i++) {
    fwrite(&FmuDataRef.Gps[i],sizeof(GpsData),1,LogFile_);
  }
  for (size_t i=0; i < FmuDataRef.Pitot.size(); i++) {
    fwrite(&FmuDataRef.Pitot[i],sizeof(PitotData),1,LogFile_);
  }  
  for (size_t i=0; i < FmuDataRef.Analog.size(); i++) {
    fwrite(&FmuDataRef.Analog[i],sizeof(AnalogData),1,LogFile_);
  }
  for (size_t i=0; i < FmuDataRef.SbusVoltage.size(); i++) {
    fwrite(&FmuDataRef.SbusVoltage[i],sizeof(Voltage),1,LogFile_);
  }
  for (size_t i=0; i < FmuDataRef.PwmVoltage.size(); i++) {
    fwrite(&FmuDataRef.PwmVoltage[i],sizeof(Voltage),1,LogFile_);
  }
  fflush(LogFile_);
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


