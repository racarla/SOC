/*
main.cxx
Brian R Taylor
brian.taylor@bolderflight.com
2017-04-18
Copyright (c) 2017 Bolder Flight Systems
Permission is hereby granted, free of charge, to any person obtaining a copy of this software 
and associated documentation files (the "Software"), to deal in the Software without restriction, 
including without limitation the rights to use, copy, modify, merge, publish, distribute, 
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is 
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all copies or 
substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING 
BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND 
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, 
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/


#define kMaxAllocObj 3 // Allocator Objectives
#define kMaxAllocEff 6 // Allocator Effectors
#define kMaxCntrlCmd 4 // Controller Dimension
#define kMaxExciteChan 4 // Excitation Channels
#define kMaxExciteElem 46 // Excitation Elements (Multisine components)

#include "global-defs.hxx"
#include "hdf5class.hxx"
#include "config.hxx"

#include "../soc-src/airdata.hxx"
#include "../soc-src/navigation.hxx"
#include "../soc-src/structs.hxx"
#include "../soc-src/missionMgr.hxx"
#include "../soc-src/cntrlMgr.hxx"
#include "../soc-src/exciteMgr.hxx"

#include <H5Cpp.h>
#include <iostream>
#include <string>
#include <vector>

#define EIGEN_INITIALIZE_MATRICES_BY_NAN 1

//#define EIGEN_MAX_ALIGN_BYTES 0
//#define EIGEN_MAX_STATIC_ALIGN_BYTES 0

using namespace std;
using namespace H5;

int main(int argc, char* argv[]) {
  if (argc!=4) {
    std::cerr << "ERROR: Incorrect number of input arguments." << std::endl;
    return -1;
  }

  /* initialize structures */
  FmuData fmuData;
  FmuConfig fmuConfig;
  //AircraftConfig configData;


  /* load configuration file */
  LoadConfigFile(argv[1],&fmuData,&fmuConfig);

  /* load the flight data file*/
  FILE *BinaryFile = fopen(argv[2],"rb");

  /* figure out how many data records we have */
  fseek(BinaryFile,0,SEEK_END);
  size_t size = ftell(BinaryFile);
  rewind(BinaryFile);
  size_t bytes = 0;
  size_t NumberRecords;

  // Count the number of bytes in each record
  bytes += sizeof(fmuData.Time_us);
  bytes += sizeof(Voltage);
  bytes += sizeof(Voltage);
  bytes += sizeof(Mpu9250Data);
  bytes += sizeof(Bme280Data);

  bytes += fmuData.Mpu9250Ext.size() * sizeof(Mpu9250Data);
  bytes += fmuData.Bme280Ext.size() * sizeof(Bme280Data);
  bytes += fmuData.SbusRx.size() * sizeof(SbusRxData);
  bytes += fmuData.Gps.size() * sizeof(GpsData);
  bytes += fmuData.Pitot.size() * sizeof(PitotData);
  bytes += fmuData.PressureTransducer.size() * sizeof(PressureData);
  bytes += fmuData.Analog.size() * sizeof(AnalogData);
  bytes += fmuData.SbusVoltage.size() * sizeof(Voltage);
  bytes += fmuData.PwmVoltage.size() * sizeof(Voltage);
  cout << "Data packet size: " << bytes << " bytes" << endl;

  bytes += sizeof(AirdataLog);
  cout << "Data packet size: " << bytes << " bytes" << endl;
  bytes += sizeof(NavLog);
  cout << "Data packet size: " << bytes << " bytes" << endl;
  bytes += sizeof(MissMgrLog);
  cout << "Data packet size: " << bytes << " bytes" << endl;
  bytes += sizeof(ExciteMgrLog);
  cout << "Data packet size: " << bytes << " bytes" << endl;
  bytes += sizeof(CntrlMgrLog);
  cout << endl;

  // Compute the number of records
  NumberRecords = size/bytes;

  cout << "File size: " << size << " bytes"<< endl;
  cout << "Data packet size: " << bytes << " bytes" << endl;
  cout << "Number of records: " << NumberRecords << endl;

  /* allocate memory to store data file contents */
  uint16_t** FileBuffer = new uint16_t* [NumberRecords];
  for (size_t i=0; i < NumberRecords; i++) {
    FileBuffer[i] = new uint16_t [bytes];
  }

  /* read binary file into memory */
  for (size_t i=0; i < NumberRecords; i++) {
    fread(FileBuffer[i],bytes,1,BinaryFile);
  }
  fclose(BinaryFile);

  /* create an HDF5 file */
  hdf5class Logger(argv[3]);

  /* parse binary file into data structures */
  uint64_t* Time_us = new uint64_t [NumberRecords];
  Voltage* InputVoltage = new Voltage [NumberRecords];
  Voltage* RegulatedVoltage = new Voltage [NumberRecords];
  Mpu9250Data* Mpu9250 = new Mpu9250Data [NumberRecords];
  Bme280Data* Bme280 = new Bme280Data [NumberRecords];
  Mpu9250Data** Mpu9250Ext = new Mpu9250Data* [fmuData.Mpu9250Ext.size()];
  for (size_t i=0; i < fmuData.Mpu9250Ext.size(); i++) {
    Mpu9250Ext[i] = new Mpu9250Data [NumberRecords];
  }
  Bme280Data** Bme280Ext = new Bme280Data* [fmuData.Bme280Ext.size()];
  for (size_t i=0; i < fmuData.Bme280Ext.size(); i++) {
    Bme280Ext[i] = new Bme280Data [NumberRecords];
  }
  SbusRxData** SbusRx = new SbusRxData* [fmuData.SbusRx.size()];
  for (size_t i=0; i < fmuData.SbusRx.size(); i++) {
    SbusRx[i] = new SbusRxData [NumberRecords];
  }
  GpsData** Gps = new GpsData* [fmuData.Gps.size()];
  for (size_t i=0; i < fmuData.Gps.size(); i++) {
    Gps[i] = new GpsData [NumberRecords];
  }
  PitotData** Pitot = new PitotData* [fmuData.Pitot.size()];
  for (size_t i=0; i < fmuData.Pitot.size(); i++) {
    Pitot[i] = new PitotData [NumberRecords];
  }
  PressureData** PressureTransducer = new PressureData* [fmuData.PressureTransducer.size()];
  for (size_t i=0; i < fmuData.PressureTransducer.size(); i++) {
    PressureTransducer[i] = new PressureData [NumberRecords];
  }
  AnalogData** Analog = new AnalogData* [fmuData.Analog.size()];
  for (size_t i=0; i < fmuData.Analog.size(); i++) {
    Analog[i] = new AnalogData [NumberRecords];
  }
  Voltage** SbusVoltage = new Voltage* [fmuData.SbusVoltage.size()];
  for (size_t i=0; i < fmuData.SbusVoltage.size(); i++) {
    SbusVoltage[i] = new Voltage [NumberRecords];
  }
  Voltage** PwmVoltage = new Voltage* [fmuData.PwmVoltage.size()];
  for (size_t i=0; i < fmuData.PwmVoltage.size(); i++) {
    PwmVoltage[i] = new Voltage [NumberRecords];
  }

  AirdataLog* airdataLog = new AirdataLog [NumberRecords];
  NavLog* navLog = new NavLog [NumberRecords];
  MissMgrLog* missMgrLog = new MissMgrLog [NumberRecords];
  ExciteMgrLog* exciteMgrLog = new ExciteMgrLog [NumberRecords];
  CntrlMgrLog* cntrlMgrLog = new CntrlMgrLog [NumberRecords];

  uint8_t* startByte;
  size_t lenByte;

  // Read each line of the binary file and copy into the structure pointers
  for (size_t i=0; i < NumberRecords; i++) {
    startByte = (uint8_t*) (FileBuffer[i]); // Initialize the start location

    Time_us[i] = *(uint64_t *)startByte;
    startByte += sizeof(uint64_t);

    InputVoltage[i] = *(Voltage *)startByte;
    startByte += sizeof(Voltage);

    RegulatedVoltage[i] = *(Voltage *)startByte;
    startByte += sizeof(Voltage);

    Mpu9250[i] = *(Mpu9250Data *)startByte;
    startByte += sizeof(Mpu9250Data);

    Bme280[i] = *(Bme280Data *)startByte;
    startByte += sizeof(Bme280Data);

    for (size_t j=0; j < fmuData.Mpu9250Ext.size(); j++) {
      Mpu9250Ext[j][i] = *(Mpu9250Data *)startByte;
      startByte += sizeof(Mpu9250Data);
    }

    for (size_t j=0; j < fmuData.Bme280Ext.size(); j++) {
      Bme280Ext[j][i] = *(Bme280Data *)startByte;
      startByte += sizeof(Bme280Data);
    }

    for (size_t j=0; j < fmuData.SbusRx.size(); j++) {
      SbusRx[j][i] = *(SbusRxData *)startByte;
      startByte += sizeof(SbusRxData);
    }

    for (size_t j=0; j < fmuData.Gps.size(); j++) {
      Gps[j][i] = *(GpsData *)startByte;
      startByte += sizeof(GpsData);
    }

    for (size_t j=0; j < fmuData.Pitot.size(); j++) {
      Pitot[j][i] = *(PitotData *)startByte;
      startByte += sizeof(PitotData);
    }

    for (size_t j=0; j < fmuData.PressureTransducer.size(); j++) {
      PressureTransducer[j][i] = *(PressureData *)startByte;
      startByte += sizeof(PressureData);
    }

    for (size_t j=0; j < fmuData.Analog.size(); j++) {
      Analog[j][i] = *(AnalogData *)startByte;
      startByte += sizeof(AnalogData);
    }

    for (size_t j=0; j < fmuData.SbusVoltage.size(); j++) {
      SbusVoltage[j][i] = *(Voltage *)startByte;
      startByte += sizeof(Voltage);
    }

    for (size_t j=0; j < fmuData.PwmVoltage.size(); j++) {
      PwmVoltage[j][i] = *(Voltage *)startByte;
      startByte += sizeof(Voltage);
    }

    airdataLog[i] = *(AirdataLog *)startByte;
    startByte += sizeof(AirdataLog);

    navLog[i] = *(NavLog *)startByte;
    startByte += sizeof(NavLog);

    missMgrLog[i] = *(MissMgrLog *)startByte;
    startByte += sizeof(MissMgrLog);

    exciteMgrLog[i] = *(ExciteMgrLog *)startByte;
    startByte += sizeof(ExciteMgrLog);

    cntrlMgrLog[i] = *(CntrlMgrLog *)startByte;
    startByte += sizeof(CntrlMgrLog);
  }


  /* Save data into HDF5 */
  short* data3Ds = new short [3*NumberRecords];

  float* data1D = new float [NumberRecords];
  float* data3D = new float [3*NumberRecords];
  float* data4D = new float [4*NumberRecords];
  float* data5D = new float [5*NumberRecords];
  float* data6D = new float [6*NumberRecords];
  float* data7D = new float [7*NumberRecords];

  double* data1Dd = new double [NumberRecords];
  double* data3Dd = new double [3*NumberRecords];
  double* data4Dd = new double [4*NumberRecords];

  uint8_t* data1Du8 = new uint8_t [NumberRecords];
  uint16_t* data1Du16 = new uint16_t [NumberRecords];
  uint32_t* data1Du32 = new uint32_t [NumberRecords];
  int8_t* data1Di8 = new int8_t [NumberRecords];
  int16_t* data1Di16 = new int16_t [NumberRecords];
  int32_t* data1Di32 = new int32_t [NumberRecords];



  string GroupName = "/Fmu";

  /* FMU data: time, input voltage, and regulated voltage */
  Logger.WriteData(GroupName,"Time_us",Time_us,"Time, us",NumberRecords,1);
  for (size_t k=0; k < NumberRecords; k++) {
    data1D[k] = InputVoltage[k].Voltage_V;
  }
  Logger.WriteData(GroupName,"InputVoltage_V",data1D,"Input voltage, V",NumberRecords,1);
  for (size_t k=0; k < NumberRecords; k++) {
    data1D[k] = RegulatedVoltage[k].Voltage_V;
  }
  Logger.WriteData(GroupName,"RegulatedVoltage_V",data1D,"Regulated voltage, V",NumberRecords,1);


  /* MPU-9250 data: accel, gyro, mag, and temperature */
  GroupName = "/Mpu9250";

  size_t m = 0;
  for (size_t k=0; k < NumberRecords; k++) {
    data3D[m] = Mpu9250[k].Accel_mss(0,0);
    data3D[m+1] = Mpu9250[k].Accel_mss(1,0);
    data3D[m+2] = Mpu9250[k].Accel_mss(2,0);
    m = m + 3;
  }
  Logger.WriteData(GroupName,"Accel_mss",data3D,"X, Y, Z accelerometer translated to aircraft body axis system, m/s/s",NumberRecords,3);
  m = 0;
  for (size_t k=0; k < NumberRecords; k++) {
    data3D[m] = Mpu9250[k].Gyro_rads(0,0);
    data3D[m+1] = Mpu9250[k].Gyro_rads(1,0);
    data3D[m+2] = Mpu9250[k].Gyro_rads(2,0);
    m = m + 3;
  }
  Logger.WriteData(GroupName,"Gyro_rads",data3D,"X, Y, Z gyro translated to aircraft body axis system, rad/s",NumberRecords,3);
  m = 0;
  for (size_t k=0; k < NumberRecords; k++) {
    data3D[m] = Mpu9250[k].Mag_uT(0,0);
    data3D[m+1] = Mpu9250[k].Mag_uT(1,0);
    data3D[m+2] = Mpu9250[k].Mag_uT(2,0);
    m = m + 3;
  }
  Logger.WriteData(GroupName,"Mag_uT",data3D,"X, Y, Z magnetometer translated to aircraft body axis system, uT",NumberRecords,3);
  for (size_t k=0; k < NumberRecords; k++) {
    data1D[k] = Mpu9250[k].Temp_C;
  }
  Logger.WriteData(GroupName,"Temp_C",data1D,"Temperature, C",NumberRecords,1);

  /* BME-280 data: pressure, temperature, humidity */
  GroupName = "/Bme280";

  for (size_t k=0; k < NumberRecords; k++) {
    data1D[k] = Bme280[k].Pressure_Pa;
  }
  Logger.WriteData(GroupName,"Pressure_Pa",data1D,"Static pressure, Pa",NumberRecords,1);
  for (size_t k=0; k < NumberRecords; k++) {
    data1D[k] = Bme280[k].Temp_C;
  }
  Logger.WriteData(GroupName,"Temp_C",data1D,"Temperature, C",NumberRecords,1);
  for (size_t k=0; k < NumberRecords; k++) {
    data1D[k] = Bme280[k].Humidity_RH;
  }
  Logger.WriteData(GroupName,"Humidity_RH",data1D,"Percent relative humidity",NumberRecords,1);


  /* External MPU-9250 */
  for (size_t l=0; l < fmuData.Mpu9250Ext.size(); l++) {
    GroupName = fmuConfig.Mpu9250Names[l];
    m = 0;
    for (size_t k=0; k < NumberRecords; k++) {
      data3D[m] = Mpu9250Ext[l][k].Accel_mss(0,0);
      data3D[m+1] = Mpu9250Ext[l][k].Accel_mss(1,0);
      data3D[m+2] = Mpu9250Ext[l][k].Accel_mss(2,0);
      m = m + 3;
    }
    Logger.WriteData(GroupName,"Accel_mss",data3D,"X, Y, Z accelerometer translated to aircraft body axis system, m/s/s",NumberRecords,3);
    m = 0;
    for (size_t k=0; k < NumberRecords; k++) {
      data3D[m] = Mpu9250Ext[l][k].Gyro_rads(0,0);
      data3D[m+1] = Mpu9250Ext[l][k].Gyro_rads(1,0);
      data3D[m+2] = Mpu9250Ext[l][k].Gyro_rads(2,0);
      m = m + 3;
    }
    Logger.WriteData(GroupName,"Gyro_rads",data3D,"X, Y, Z gyro translated to aircraft body axis system, rad/s",NumberRecords,3);
    m = 0;
    for (size_t k=0; k < NumberRecords; k++) {
      data3D[m] = Mpu9250Ext[l][k].Mag_uT(0,0);
      data3D[m+1] = Mpu9250Ext[l][k].Mag_uT(1,0);
      data3D[m+2] = Mpu9250Ext[l][k].Mag_uT(2,0);
      m = m + 3;
    }
    Logger.WriteData(GroupName,"Mag_uT",data3D,"X, Y, Z magnetometer translated to aircraft body axis system, uT",NumberRecords,3);
    for (size_t k=0; k < NumberRecords; k++) {
      data1D[k] = Mpu9250Ext[l][k].Temp_C;
    }
    Logger.WriteData(GroupName,"Temp_C",data1D,"Temperature, C",NumberRecords,1);
  }


  /* External BME-280 */
  for (size_t l=0; l < fmuData.Bme280Ext.size(); l++) {
    GroupName = fmuConfig.Bme280Names[l];
    for (size_t k=0; k < NumberRecords; k++) {
      data1D[k] = Bme280Ext[l][k].Pressure_Pa;
    }
    Logger.WriteData(GroupName,"Pressure_Pa",data1D,"Static pressure, Pa",NumberRecords,1);
    for (size_t k=0; k < NumberRecords; k++) {
      data1D[k] = Bme280Ext[l][k].Temp_C;
    }
    Logger.WriteData(GroupName,"Temp_C",data1D,"Temperature, C",NumberRecords,1);
    for (size_t k=0; k < NumberRecords; k++) {
      data1D[k] = Bme280Ext[l][k].Humidity_RH;
    }
    Logger.WriteData(GroupName,"Humidity_RH",data1D,"Percent relative humidity",NumberRecords,1);
  }


  /* Sbus RX */
  for (size_t l=0; l < fmuData.SbusRx.size(); l++) {
    GroupName = fmuConfig.SbusRxNames[l];
    for (size_t k=0; k < NumberRecords; k++) {
      data1Du8[k] = SbusRx[l][k].Failsafe;
    }
    Logger.WriteData(GroupName,"Failsafe",data1Du8,"True when failsafe active",NumberRecords,1);    
    for (size_t k=0; k < NumberRecords; k++) {
      data1Du16[k] = SbusRx[l][k].LostFrames;
    }
    Logger.WriteData(GroupName,"LostFrames",data1Du16,"Number of lost frames",NumberRecords,1); 
    for (size_t k=0; k < NumberRecords; k++) {
      data1Du8[k] = SbusRx[l][k].AutoEnabled;
    }
    Logger.WriteData(GroupName,"AutoEnabled",data1Du8,"True when autopilot enabled",NumberRecords,1); 
    for (size_t k=0; k < NumberRecords; k++) {
      data1Du8[k] = SbusRx[l][k].ThrottleEnabled;
    }
    Logger.WriteData(GroupName,"ThrottleEnabled",data1Du8,"True when throttle enabled",NumberRecords,1);
     for (size_t k=0; k < NumberRecords; k++) {
      data1D[k] = SbusRx[l][k].RSSI;
    }
    Logger.WriteData(GroupName,"RSSI",data1D,"RSSI value",NumberRecords,1);
    m = 0;
    for (size_t k=0; k < NumberRecords; k++) {
      data5D[m] = SbusRx[l][k].Inceptors(0,0);
      data5D[m+1] = SbusRx[l][k].Inceptors(1,0);
      data5D[m+2] = SbusRx[l][k].Inceptors(2,0);
      data5D[m+3] = SbusRx[l][k].Inceptors(3,0);
      data5D[m+4] = SbusRx[l][k].Inceptors(4,0);
      m = m + 5;
    }
    Logger.WriteData(GroupName,"Inceptors",data5D,"Aerodynamic inceptors, roll pitch yaw lift thrust, normalized to +/- 1",NumberRecords,5);
    m = 0;
    for (size_t k=0; k < NumberRecords; k++) {
      data5D[m] = SbusRx[l][k].AuxInputs(0,0);
      data5D[m+1] = SbusRx[l][k].AuxInputs(1,0);
      data5D[m+2] = SbusRx[l][k].AuxInputs(2,0);
      data5D[m+3] = SbusRx[l][k].AuxInputs(3,0);
      data5D[m+4] = SbusRx[l][k].AuxInputs(4,0);
      m = m + 5;
    }
    Logger.WriteData(GroupName,"AuxInputs",data5D,"Auxiliary inputs, normalized to +/- 1",NumberRecords,5);
  } 


  /* Gps */
  for (size_t l=0; l < fmuData.Gps.size(); l++) {
    GroupName = fmuConfig.GpsNames[l];

    for (size_t k=0; k < NumberRecords; k++) {
      data1Du8[k] = Gps[l][k].Fix;
    }
    Logger.WriteData(GroupName,"Fix",data1Du8,"True for 3D fix only",NumberRecords,1); 

    for (size_t k=0; k < NumberRecords; k++) {
      data1Du8[k] = Gps[l][k].NumberSatellites;
    }
    Logger.WriteData(GroupName,"NumberSatellites",data1Du8,"Number of satellites used in solution",NumberRecords,1); 

    for (size_t k=0; k < NumberRecords; k++) {
      data1Du32[k] = Gps[l][k].TOW;
    }
    Logger.WriteData(GroupName,"TOW",data1Du32,"GPS time of the navigation epoch",NumberRecords,1); 

    for (size_t k=0; k < NumberRecords; k++) {
      data1Du16[k] = Gps[l][k].Year;
    }
    Logger.WriteData(GroupName,"Year",data1Du16,"UTC year",NumberRecords,1);

    for (size_t k=0; k < NumberRecords; k++) {
      data1Du8[k] = Gps[l][k].Month;
    }
    Logger.WriteData(GroupName,"Month",data1Du8,"UTC month",NumberRecords,1); 

    for (size_t k=0; k < NumberRecords; k++) {
      data1Du8[k] = Gps[l][k].Day;
    }
    Logger.WriteData(GroupName,"Day",data1Du8,"UTC day",NumberRecords,1); 

    for (size_t k=0; k < NumberRecords; k++) {
      data1Du8[k] = Gps[l][k].Hour;
    }
    Logger.WriteData(GroupName,"Hour",data1Du8,"UTC hour",NumberRecords,1); 

    for (size_t k=0; k < NumberRecords; k++) {
      data1Du8[k] = Gps[l][k].Min;
    }
    Logger.WriteData(GroupName,"Min",data1Du8,"UTC minute",NumberRecords,1); 

    for (size_t k=0; k < NumberRecords; k++) {
      data1Du8[k] = Gps[l][k].Sec;
    }
    Logger.WriteData(GroupName,"Sec",data1Du8,"UTC second",NumberRecords,1); 

    m = 0;
    for (size_t k=0; k < NumberRecords; k++) {
      data3Dd[m] = Gps[l][k].LLA(0,0);
      data3Dd[m+1] = Gps[l][k].LLA(1,0);
      data3Dd[m+2] = Gps[l][k].LLA(2,0);
      m = m + 3;
    }
    Logger.WriteData(GroupName,"LLA",data3Dd,"Latitude (rad), Longitude (rad), Altitude (m)",NumberRecords,3);

    m = 0;
    for (size_t k=0; k < NumberRecords; k++) {
      data3Dd[m] = Gps[l][k].NEDVelocity_ms(0,0);
      data3Dd[m+1] = Gps[l][k].NEDVelocity_ms(1,0);
      data3Dd[m+2] = Gps[l][k].NEDVelocity_ms(2,0);
      m = m + 3;
    }
    Logger.WriteData(GroupName,"NEDVelocity_ms",data3Dd,"North, East, Down Velocity, m/s",NumberRecords,3);

    m = 0;
    for (size_t k=0; k < NumberRecords; k++) {
      data3Dd[m] = Gps[l][k].Accuracy(0,0);
      data3Dd[m+1] = Gps[l][k].Accuracy(1,0);
      data3Dd[m+2] = Gps[l][k].Accuracy(2,0);
      m = m + 3;
    }
    Logger.WriteData(GroupName,"Accuracy",data3Dd,"Horizontal (m), vertical (m), and speed (m/s) accuracy estimates",NumberRecords,3);

    for (size_t k=0; k < NumberRecords; k++) {
      data1Dd[k] = Gps[l][k].pDOP;
    }
    Logger.WriteData(GroupName,"pDOP",data1Dd,"Position degree of precision",NumberRecords,1); 
  }


  /* Pitot */
  for (size_t l=0; l < fmuData.Pitot.size(); l++) {
    GroupName = fmuConfig.PitotNames[l];

    Logger.CreateGroup(GroupName);

    for (size_t k=0; k < NumberRecords; k++) {
      data1D[k] = Pitot[l][k].Static.Pressure_Pa;
    }
    Logger.WriteData(GroupName + "/Static","Pressure_Pa",data1D,"Static pressure, Pa",NumberRecords,1);

    for (size_t k=0; k < NumberRecords; k++) {
      data1D[k] = Pitot[l][k].Static.Temp_C;
    }
    Logger.WriteData(GroupName + "/Static","Temp_C",data1D,"Temperature, C",NumberRecords,1);

    for (size_t k=0; k < NumberRecords; k++) {
      data1D[k] = Pitot[l][k].Diff.Pressure_Pa;
    }
    Logger.WriteData(GroupName + "/Diff","Pressure_Pa",data1D,"Differential pressure, Pa",NumberRecords,1);

    for (size_t k=0; k < NumberRecords; k++) {
      data1D[k] = Pitot[l][k].Diff.Temp_C;
    }
    Logger.WriteData(GroupName + "/Diff","Temp_C",data1D,"Temperature, C",NumberRecords,1);
  }


  /* Pressure */
  for (size_t l=0; l < fmuData.PressureTransducer.size(); l++) {
    GroupName = fmuConfig.PressureTransducerNames[l];

    for (size_t k=0; k < NumberRecords; k++) {
      data1D[k] = PressureTransducer[l][k].Pressure_Pa;
    }
    Logger.WriteData(GroupName,"Pressure_Pa",data1D,"Pressure, Pa",NumberRecords,1);

    for (size_t k=0; k < NumberRecords; k++) {
      data1D[k] = PressureTransducer[l][k].Temp_C;
    }
    Logger.WriteData(GroupName,"Temp_C",data1D,"Temperature, C",NumberRecords,1);
  }


  /* Analog */
  for (size_t l=0; l < fmuData.Analog.size(); l++) {
    GroupName = fmuConfig.AnalogNames[l];

    for (size_t k=0; k < NumberRecords; k++) {
      data1D[k] = Analog[l][k].Voltage_V;
    }
    Logger.WriteData(GroupName,"Voltage_V",data1D,"Measured voltage, V",NumberRecords,1);

    for (size_t k=0; k < NumberRecords; k++) {
      data1D[k] = Analog[l][k].CalValue;
    }
    Logger.WriteData(GroupName,"CalValue",data1D,"Value from applying calibration to voltage",NumberRecords,1);
  }


  /* Sbus Voltage */
  for (size_t l=0; l < fmuData.SbusVoltage.size(); l++) {
    GroupName = "SbusVoltage_" + to_string(l);

    for (size_t k=0; k < NumberRecords; k++) {
      data1D[k] = SbusVoltage[l][k].Voltage_V;
    }
    Logger.WriteData(GroupName,"Voltage_V",data1D,"Measured voltage, V",NumberRecords,1);
  }


  /* Pwm Voltage */ 
  for (size_t l=0; l < fmuData.PwmVoltage.size(); l++) {
    GroupName = "PwmVoltage_" + to_string(l);

    for (size_t k=0; k < NumberRecords; k++) {
      data1D[k] = PwmVoltage[l][k].Voltage_V;
    }
    Logger.WriteData(GroupName,"Voltage_V",data1D,"Measured voltage, V",NumberRecords,1);
  }


  // AirdataLog
  GroupName = "/Airdata";

  Logger.CreateGroup(GroupName);

  for (size_t k=0; k < NumberRecords; k++) {
    data1D[k] = airdataLog[k].temp_C;
  }
  Logger.WriteData(GroupName,"temp_C",data1D,"Temperature, C",NumberRecords,1);

  for (size_t k=0; k < NumberRecords; k++) {
    data1D[k] = airdataLog[k].alt_m;
  }
  Logger.WriteData(GroupName,"alt_m",data1D,"Altitude, m",NumberRecords,1);

  for (size_t k=0; k < NumberRecords; k++) {
    data1D[k] = airdataLog[k].altFilt_m;
  }
  Logger.WriteData(GroupName,"altFilt_m",data1D,"Altitude Filterd, m",NumberRecords,1);

  for (size_t k=0; k < NumberRecords; k++) {
    data1D[k] = airdataLog[k].vIas_mps;
  }
  Logger.WriteData(GroupName,"vIas_mps",data1D,"Airspeed, mps",NumberRecords,1);

  for (size_t k=0; k < NumberRecords; k++) {
    data1D[k] = airdataLog[k].vIasFilt_mps;
  }
  Logger.WriteData(GroupName,"vIasFilt_mps",data1D,"Airspeed Filtered, mps",NumberRecords,1);


  // Navigation
  GroupName = "/NavFilter";

  for (size_t k=0; k < NumberRecords; k++) {
    data1Dd[k] = navLog[k].Time_s;
  }
  Logger.WriteData(GroupName,"Time_s",data1D,"Time, s",NumberRecords,1);

  m = 0;
  for (size_t k=0; k < NumberRecords; k++) {
    data3Dd[m] = navLog[k].LLA[0];
    data3Dd[m+1] = navLog[k].LLA[1];
    data3Dd[m+2] = navLog[k].LLA[2];
    m = m + 3;
  }
  Logger.WriteData(GroupName,"LLA",data3Dd,"Latitude (rad), Longitude (rad), Altitude (m)",NumberRecords,3);

  m = 0;
  for (size_t k=0; k < NumberRecords; k++) {
    data3Dd[m] = navLog[k].NEDVelocity_ms[0];
    data3Dd[m+1] = navLog[k].NEDVelocity_ms[1];
    data3Dd[m+2] = navLog[k].NEDVelocity_ms[2];
    m = m + 3;
  }
  Logger.WriteData(GroupName,"NEDVelocity_ms",data3Dd,"North, East, Down Velocity, m/s",NumberRecords,3);

  m = 0;
  for (size_t k=0; k < NumberRecords; k++) {
    data3D[m] = navLog[k].Euler_rad[0];
    data3D[m+1] = navLog[k].Euler_rad[1];
    data3D[m+2] = navLog[k].Euler_rad[2];
    m = m + 3;
  }
  Logger.WriteData(GroupName,"Euler_rad",data3Dd,"Euler Roll, Pitch, Yaw Orientation, rad",NumberRecords,3);

  m = 0;
  for (size_t k=0; k < NumberRecords; k++) {
    data3D[m] = navLog[k].Quaternion[0];
    data3D[m+1] = navLog[k].Quaternion[1];
    data3D[m+2] = navLog[k].Quaternion[2];
    data3D[m+3] = navLog[k].Quaternion[3];
    m = m + 4;
  }
  Logger.WriteData(GroupName,"Quaternion",data4Dd,"Quaternion Orientation, nd",NumberRecords,4);

  m = 0;
  for (size_t k=0; k < NumberRecords; k++) {
    data3D[m] = navLog[k].AccelBias_mss[0];
    data3D[m+1] = navLog[k].AccelBias_mss[1];
    data3D[m+2] = navLog[k].AccelBias_mss[2];
    m = m + 3;
  }
  Logger.WriteData(GroupName,"AccelBias_mss",data3Dd,"Accel Bias X, Y, Z, mps2",NumberRecords,3);

  m = 0;
  for (size_t k=0; k < NumberRecords; k++) {
    data3D[m] = navLog[k].GyroBias_rads[0];
    data3D[m+1] = navLog[k].GyroBias_rads[1];
    data3D[m+2] = navLog[k].GyroBias_rads[2];
    m = m + 3;
  }
  Logger.WriteData(GroupName,"GyroBias_rads",data3Dd,"Gyro Bias X, Y, Z, rps",NumberRecords,3);

  m = 0;
  for (size_t k=0; k < NumberRecords; k++) {
    data3Ds[m] = navLog[k].Pp[0];
    data3Ds[m+1] = navLog[k].Pp[1];
    data3Ds[m+2] = navLog[k].Pp[2];
    m = m + 3;
  }
  Logger.WriteData(GroupName,"Pp",data3Dd,"Position Covariance, rad",NumberRecords,3);

  m = 0;
  for (size_t k=0; k < NumberRecords; k++) {
    data3Ds[m] = navLog[k].Pv[0];
    data3Ds[m+1] = navLog[k].Pv[1];
    data3Ds[m+2] = navLog[k].Pv[2];
    m = m + 3;
  }
  Logger.WriteData(GroupName,"Pv",data3Dd,"Velocity Covariance, mps",NumberRecords,3);

  m = 0;
  for (size_t k=0; k < NumberRecords; k++) {
    data3Ds[m] = navLog[k].Pa[0];
    data3Ds[m+1] = navLog[k].Pa[1];
    data3Ds[m+2] = navLog[k].Pa[2];
    m = m + 3;
  }
  Logger.WriteData(GroupName,"Pa",data3Dd,"Angle Covariance, rad",NumberRecords,3);

  m = 0;
  for (size_t k=0; k < NumberRecords; k++) {
    data3Ds[m] = navLog[k].Pab[0];
    data3Ds[m+1] = navLog[k].Pab[1];
    data3Ds[m+2] = navLog[k].Pab[2];
    m = m + 3;
  }
  Logger.WriteData(GroupName,"Pab",data3Dd,"Accel Bias Covariance, mps2",NumberRecords,3);

  m = 0;
  for (size_t k=0; k < NumberRecords; k++) {
    data3Ds[m] = navLog[k].Pgb[0];
    data3Ds[m+1] = navLog[k].Pgb[1];
    data3Ds[m+2] = navLog[k].Pgb[2];
    m = m + 3;
  }
  Logger.WriteData(GroupName,"Pgb",data3Dd,"Gyro Bias Covariance, rps",NumberRecords,3);


  // Mission Manager
  GroupName = "/Mission";

  Logger.CreateGroup(GroupName);

  for (size_t k=0; k < NumberRecords; k++) {
    data1D[k] = missMgrLog[k].time_s;
  }
  Logger.WriteData(GroupName,"time_s",data1D,"Mission Time, s",NumberRecords,1);

  for (size_t k=0; k < NumberRecords; k++) {
    data1Du32[k] = missMgrLog[k].frame_cnt;
  }
  Logger.WriteData(GroupName,"frame_cnt",data1Du32,"Frame Count, cnt",NumberRecords,1);

  for (size_t k=0; k < NumberRecords; k++) {
    data1Du8[k] = missMgrLog[k].autoEngage;
  }
  Logger.WriteData(GroupName,"autoEngage",data1Du8,"Auto Engaged, bool",NumberRecords,1);

  for (size_t k=0; k < NumberRecords; k++) {
    data1Di8[k] = missMgrLog[k].cntrlMode;
  }
  Logger.WriteData(GroupName,"cntrlMode",data1Di8,"Control System Mode, int",NumberRecords,1);

  for (size_t k=0; k < NumberRecords; k++) {
    data1Du8[k] = missMgrLog[k].testArm;
  }
  Logger.WriteData(GroupName,"testArm",data1Du8,"Tester Armed, bool",NumberRecords,1);

  for (size_t k=0; k < NumberRecords; k++) {
    data1Du8[k] = missMgrLog[k].testEngage;
  }
  Logger.WriteData(GroupName,"testEngage",data1Du8,"Tester Engaged, bool",NumberRecords,1);

  for (size_t k=0; k < NumberRecords; k++) {
    data1Du8[k] = missMgrLog[k].indxTest;
  }
  Logger.WriteData(GroupName,"indxTest",data1Du8,"Tester Index, cnt",NumberRecords,1);



  for (size_t k=0; k < NumberRecords; k++) {
    data1Dd[k] = missMgrLog[k].tDurSens_ms;
  }
  Logger.WriteData(GroupName,"tDurSens_ms",data1Dd,"Sensor Processing Duration, ms",NumberRecords,1);

  for (size_t k=0; k < NumberRecords; k++) {
    data1Dd[k] = missMgrLog[k].tDurNav_ms;
  }
  Logger.WriteData(GroupName,"tDurNav_ms",data1Dd,"Nav Filter Duration, ms",NumberRecords,1);

  for (size_t k=0; k < NumberRecords; k++) {
    data1Dd[k] = missMgrLog[k].tDurExcite_ms;
  }
  Logger.WriteData(GroupName,"tDurExcite_ms",data1Dd,"Excitation Generation Duration, ms",NumberRecords,1);

  for (size_t k=0; k < NumberRecords; k++) {
    data1Dd[k] = missMgrLog[k].tDurCntrl_ms;
  }
  Logger.WriteData(GroupName,"tDurCntrl_ms",data1Dd,"Control Law Duration, ms",NumberRecords,1);

  for (size_t k=0; k < NumberRecords; k++) {
    data1Dd[k] = missMgrLog[k].tCmd_ms;
  }
  Logger.WriteData(GroupName,"tCmd_ms",data1Dd,"Command Message Sent Time, ms",NumberRecords,1);
  
  for (size_t k=0; k < NumberRecords; k++) {
    data1Dd[k] = missMgrLog[k].tFrame_ms;
  }
  Logger.WriteData(GroupName,"tFrame_ms",data1Dd,"Total Frame Duration, ms",NumberRecords,1);


  // Excitation Manager
  GroupName = "/Excite";

  Logger.CreateGroup(GroupName);

  for (size_t k=0; k < NumberRecords; k++) {
    data1Du8[k] = exciteMgrLog[k].exciteMode;
  }
  Logger.WriteData(GroupName,"exciteMode",data1Du8,"Excitation Mode, bool",NumberRecords,1);

  for (size_t k=0; k < NumberRecords; k++) {
    data1Du8[k] = exciteMgrLog[k].indxTest;
  }
  Logger.WriteData(GroupName,"indxTest",data1Du8,"Excitation Index Number, int",NumberRecords,1);

  for (size_t k=0; k < NumberRecords; k++) {
    data1D[k] = exciteMgrLog[k].timeExcite_s;
  }
  Logger.WriteData(GroupName,"timeExcite_s",data1D,"Time, s",NumberRecords,1);

  m = 0;
  for (size_t k=0; k < NumberRecords; k++) {
    data4D[m] = exciteMgrLog[k].cmdExcite[0];
    data4D[m+1] = exciteMgrLog[k].cmdExcite[1];
    data4D[m+2] = exciteMgrLog[k].cmdExcite[2];
    data4D[m+3] = exciteMgrLog[k].cmdExcite[3];
    m = m + 4;
  }
  Logger.WriteData(GroupName,"cmdExcite",data4D,"Excitation Commands, mag",NumberRecords,4);


  // Control System Manager
  GroupName = "/Cntrl";

  Logger.CreateGroup(GroupName);

  m = 0;
  for (size_t k=0; k < NumberRecords; k++) {
    data4D[m] = cntrlMgrLog[k].cmdCntrlBase[0];
    data4D[m+1] = cntrlMgrLog[k].cmdCntrlBase[1];
    data4D[m+2] = cntrlMgrLog[k].cmdCntrlBase[2];
    data4D[m+3] = cntrlMgrLog[k].cmdCntrlBase[3];
    m = m + 4;
  }
  Logger.WriteData(GroupName,"cmdCntrlBase",data4D,"Controller Baseline Commands, Roll [rps], Pitch [rps], Yaw [rps], Throttle [nd]",NumberRecords,4);
  
  m = 0;
  for (size_t k=0; k < NumberRecords; k++) {
    data4D[m] = cntrlMgrLog[k].cmdCntrlRes[0];
    data4D[m+1] = cntrlMgrLog[k].cmdCntrlRes[1];
    data4D[m+2] = cntrlMgrLog[k].cmdCntrlRes[2];
    data4D[m+3] = cntrlMgrLog[k].cmdCntrlRes[3];
    m = m + 4;
  }
  Logger.WriteData(GroupName,"cmdCntrlRes",data4D,"Controller Research Commands, Roll [rps], Pitch [rps], Yaw [rps], Throttle [nd]",NumberRecords,4);

  m = 0;
  for (size_t k=0; k < NumberRecords; k++) {
    data4D[m] = cntrlMgrLog[k].cmdCntrl[0];
    data4D[m+1] = cntrlMgrLog[k].cmdCntrl[1];
    data4D[m+2] = cntrlMgrLog[k].cmdCntrl[2];
    data4D[m+3] = cntrlMgrLog[k].cmdCntrl[3];
    m = m + 4;
  }
  Logger.WriteData(GroupName,"cmdCntrl",data4D,"Controller Commands, Roll [rps], Pitch [rps], Yaw [rps], Throttle [nd]",NumberRecords,4);

  for (size_t k=0; k < NumberRecords; k++) {
    data1Di8[k] = cntrlMgrLog[k].mode;
  }
  Logger.WriteData(GroupName,"mode",data1Di8,"Mode, int",NumberRecords,1);

  m = 0;
  for (size_t k=0; k < NumberRecords; k++) {
    data3D[m] = cntrlMgrLog[k].vObj[0];
    data3D[m+1] = cntrlMgrLog[k].vObj[1];
    data3D[m+2] = cntrlMgrLog[k].vObj[2];
    m = m + 3;
  }
  Logger.WriteData(GroupName,"vObj",data3D,"Control Allocation Objective",NumberRecords,3);

  m = 0;
  for (size_t k=0; k < NumberRecords; k++) {
    data6D[m] = cntrlMgrLog[k].cmdAlloc[0];
    data6D[m+1] = cntrlMgrLog[k].cmdAlloc[1];
    data6D[m+2] = cntrlMgrLog[k].cmdAlloc[2];
    data6D[m+3] = cntrlMgrLog[k].cmdAlloc[3];
    data6D[m+4] = cntrlMgrLog[k].cmdAlloc[4];
    data6D[m+5] = cntrlMgrLog[k].cmdAlloc[5];
    m = m + 6;
  }
  Logger.WriteData(GroupName,"cmdAlloc",data6D,"Control Allocation Commands, Elev, Rud, AilR, FlapR, FlapL, AilL",NumberRecords,6);

  m = 0;
  for (size_t k=0; k < NumberRecords; k++) {
    data7D[m] = cntrlMgrLog[k].cmdEff[0];
    data7D[m+1] = cntrlMgrLog[k].cmdEff[1];
    data7D[m+2] = cntrlMgrLog[k].cmdEff[2];
    data7D[m+3] = cntrlMgrLog[k].cmdEff[3];
    data7D[m+4] = cntrlMgrLog[k].cmdEff[4];
    data7D[m+5] = cntrlMgrLog[k].cmdEff[5];
    data7D[m+6] = cntrlMgrLog[k].cmdEff[6];
    m = m + 7;
  }
  Logger.WriteData(GroupName,"cmdEff",data7D,"Control Effector Commands, Throt (nd), Elev, Rud, AilR, FlapR, FlapL, AilL",NumberRecords,7);

	return 0;
}
