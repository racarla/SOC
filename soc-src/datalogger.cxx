
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
void Datalogger::LogData(FmuData fmuData, AirdataLog airdataLog, NavLog navLog, MissMgrLog missMgrLog, ExciteMgrLog exciteMgrLog, CntrlMgrLog cntrlMgrLog) {
  fwrite(&fmuData.Time_us,sizeof(fmuData.Time_us),1,LogFile_);
  fwrite(&fmuData.InputVoltage,sizeof(Voltage),1,LogFile_);
  fwrite(&fmuData.RegulatedVoltage,sizeof(Voltage),1,LogFile_);
  fwrite(&fmuData.Mpu9250,sizeof(Mpu9250Data),1,LogFile_);
  fwrite(&fmuData.Bme280,sizeof(Bme280Data),1,LogFile_);
  for (size_t i=0; i < fmuData.Mpu9250Ext.size(); i++) {
    fwrite(&fmuData.Mpu9250Ext[i],sizeof(Mpu9250Data),1,LogFile_);
  }
  for (size_t i=0; i < fmuData.Bme280Ext.size(); i++) {
    fwrite(&fmuData.Bme280Ext[i],sizeof(Bme280Data),1,LogFile_);
  }
  for (size_t i=0; i < fmuData.SbusRx.size(); i++) {
    fwrite(&fmuData.SbusRx[i],sizeof(SbusRxData),1,LogFile_);
  }
  for (size_t i=0; i < fmuData.Gps.size(); i++) {
    fwrite(&fmuData.Gps[i],sizeof(GpsData),1,LogFile_);
  }
  for (size_t i=0; i < fmuData.Pitot.size(); i++) {
    fwrite(&fmuData.Pitot[i],sizeof(PitotData),1,LogFile_);
  }  
  for (size_t i=0; i < fmuData.PressureTransducer.size(); i++) {
    fwrite(&fmuData.PressureTransducer[i],sizeof(PressureData),1,LogFile_);
  }
  for (size_t i=0; i < fmuData.Analog.size(); i++) {
    fwrite(&fmuData.Analog[i],sizeof(AnalogData),1,LogFile_);
  }
  for (size_t i=0; i < fmuData.SbusVoltage.size(); i++) {
    fwrite(&fmuData.SbusVoltage[i],sizeof(Voltage),1,LogFile_);
  }
  for (size_t i=0; i < fmuData.PwmVoltage.size(); i++) {
    fwrite(&fmuData.PwmVoltage[i],sizeof(Voltage),1,LogFile_);
  }

  fwrite(&airdataLog,sizeof(AirdataLog),1,LogFile_);
  fwrite(&navLog,sizeof(NavLog),1,LogFile_);
  fwrite(&missMgrLog,sizeof(MissMgrLog),1,LogFile_);
  fwrite(&exciteMgrLog,sizeof(ExciteMgrLog),1,LogFile_);
  fwrite(&cntrlMgrLog,sizeof(CntrlMgrLog),1,LogFile_);

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


