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

#include "hdf5class.hxx"
#include "config.hxx"
#include "global-defs.hxx"

#include "../soc-src/airdata.hxx"
#include "../soc-src/structs.hxx"
#include "../soc-src/missionMgr.hxx"
#include "../soc-src/cntrlMgr.hxx"
#include "../soc-src/exciteMgr.hxx"
#include "../soc-src/cntrlAllocMgr.hxx"

#include <H5Cpp.h>
#include <iostream>
#include <string>
#include <vector>

#define EIGEN_INITIALIZE_MATRICES_BY_NAN 1

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
  cout << "Bytes: " << bytes << endl;

  bytes += sizeof(fmuData.Time_us);
  cout << "Bytes: " << bytes << endl;

  bytes += sizeof(Voltage);
  cout << "Bytes: " << bytes << endl;

  bytes += sizeof(Voltage);
  cout << "Bytes: " << bytes << endl;

  bytes += sizeof(Mpu9250Data);
  cout << "Bytes: " << bytes << endl;

  bytes += sizeof(Bme280Data);
  cout << "Bytes: " << bytes << endl;

  bytes += fmuData.Mpu9250Ext.size() * sizeof(Mpu9250Data);
  bytes += fmuData.Bme280Ext.size() * sizeof(Bme280Data);
  bytes += fmuData.SbusRx.size() * sizeof(SbusRxData);
  bytes += fmuData.Gps.size() * sizeof(GpsData);
  bytes += fmuData.Pitot.size() * sizeof(PitotData);
  bytes += fmuData.PressureTransducer.size() * sizeof(PressureData);
  bytes += fmuData.Analog.size() * sizeof(AnalogData);
  bytes += fmuData.SbusVoltage.size() * sizeof(Voltage);
  bytes += fmuData.PwmVoltage.size() * sizeof(Voltage);
  cout << "Bytes: " << bytes << endl;

  bytes += sizeof(AirdataStruct);
  cout << "Bytes: " << bytes << endl;

  bytes += sizeof(NavigationData);
  cout << "Bytes: " << bytes << endl;

  bytes += sizeof(MissMgrStruct);
  cout << "Bytes: " << bytes << endl;

  bytes += sizeof(ExciteMgrStruct);
  cout << "Bytes: " << bytes << endl;

  bytes += sizeof(CntrlMgrStruct);
  cout << "Bytes: " << bytes << endl;

  bytes += sizeof(CntrlAllocStruct);
  cout << "Bytes: " << bytes << endl;
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

  AirdataStruct* airdataData = new AirdataStruct [NumberRecords];
  NavigationData* navData = new NavigationData [NumberRecords];
  MissMgrStruct* missMgrData = new MissMgrStruct [NumberRecords];
  ExciteMgrStruct* exciteMgrData = new ExciteMgrStruct [NumberRecords];
  CntrlMgrStruct* cntrlMgrData = new CntrlMgrStruct [NumberRecords];
  CntrlAllocStruct* cntrlAllocData = new CntrlAllocStruct [NumberRecords];


  size_t* startByte;
  size_t lenByte;

  // Read each line of the binary file and copy into the structure pointers
  for (size_t i=0; i < NumberRecords; i++) {
    lenByte = (size_t) FileBuffer[i]; // Initialize the start location

    startByte = startByte + lenByte; lenByte = sizeof(Time_us[i]);
    memcpy(&Time_us[i], startByte, lenByte);

    startByte = startByte + lenByte; lenByte = sizeof(InputVoltage[i]);
    memcpy(&InputVoltage[i], startByte, lenByte);

    startByte = startByte + lenByte; lenByte = sizeof(RegulatedVoltage[i]);
    memcpy(&RegulatedVoltage[i], startByte, lenByte);

    startByte = startByte + lenByte; lenByte = sizeof(Mpu9250[i]);
    memcpy(&Mpu9250[i], startByte, lenByte);

    startByte = startByte + lenByte; lenByte = sizeof(Bme280[i]);
    memcpy(&Bme280[i], startByte, lenByte);

    for (size_t j=0; j < fmuData.Mpu9250Ext.size(); j++) {
      startByte = startByte + lenByte; lenByte = sizeof(Mpu9250Ext[j][i]);
      memcpy(&Mpu9250Ext[j][i], startByte, lenByte);
    }

    for (size_t j=0; j < fmuData.Mpu9250Ext.size(); j++) {
      startByte = startByte + lenByte; lenByte = sizeof(Mpu9250Ext[j][i]);
      memcpy(&Mpu9250Ext[j][i], startByte, lenByte);
    }

    for (size_t j=0; j < fmuData.Bme280Ext.size(); j++) {
      startByte = startByte + lenByte; lenByte = sizeof(Bme280Ext[j][i]);
      memcpy(&Bme280Ext[j][i], startByte, lenByte);
    }

    for (size_t j=0; j < fmuData.SbusRx.size(); j++) {
      startByte = startByte + lenByte; lenByte = sizeof(SbusRx[j][i]);
      memcpy(&SbusRx[j][i], startByte, lenByte);
    }

    for (size_t j=0; j < fmuData.Gps.size(); j++) {
      startByte = startByte + lenByte; lenByte = sizeof(Gps[j][i]);
      memcpy(&Gps[j][i], startByte, lenByte);
    }

    for (size_t j=0; j < fmuData.Pitot.size(); j++) {
      startByte = startByte + lenByte; lenByte = sizeof(Pitot[j][i]);
      memcpy(&Pitot[j][i], startByte, lenByte);
    }

    for (size_t j=0; j < fmuData.PressureTransducer.size(); j++) {
      startByte = startByte + lenByte; lenByte = sizeof(PressureTransducer[j][i]);
      memcpy(&PressureTransducer[j][i], startByte, lenByte);
    }

    for (size_t j=0; j < fmuData.Analog.size(); j++) {
      startByte = startByte + lenByte; lenByte = sizeof(Analog[j][i]);
      memcpy(&Analog[j][i], startByte, lenByte);
    }

    for (size_t j=0; j < fmuData.SbusVoltage.size(); j++) {
      startByte = startByte + lenByte; lenByte = sizeof(SbusVoltage[j][i]);
      memcpy(&SbusVoltage[j][i], startByte, lenByte);
    }

    for (size_t j=0; j < fmuData.PressureTransducer.size(); j++) {
      startByte = startByte + lenByte; lenByte = sizeof(PressureTransducer[j][i]);
      memcpy(&PressureTransducer[j][i], startByte, lenByte);
    }

    for (size_t j=0; j < fmuData.PwmVoltage.size(); j++) {
      startByte = startByte + lenByte; lenByte = sizeof(PwmVoltage[j][i]);
      memcpy(&PwmVoltage[j][i], startByte, lenByte);
    }

    startByte = startByte + lenByte; lenByte = sizeof(airdataData[i]);
    memcpy(&airdataData[i], startByte, lenByte);

    startByte = startByte + lenByte; lenByte = sizeof(navData[i]);
    memcpy(&navData[i], startByte, lenByte);

    startByte = startByte + lenByte; lenByte = sizeof(missMgrData[i]);
    memcpy(&missMgrData[i], startByte, lenByte);

    startByte = startByte + lenByte; lenByte = sizeof(cntrlMgrData[i]);
    memcpy(&cntrlMgrData[i], startByte, lenByte);

    startByte = startByte + lenByte; lenByte = sizeof(cntrlAllocData[i]);
    memcpy(&cntrlAllocData[i], startByte, lenByte);
  }


  /* Save data into HDF5 */
  float* data1D = new float [NumberRecords];
  float* data3D = new float [3*NumberRecords];
  float* data4D = new float [4*NumberRecords];
  float* data5D = new float [5*NumberRecords];
  float* data6D = new float [6*NumberRecords];

  double* data1Dd = new double [NumberRecords];
  double* data3Dd = new double [3*NumberRecords];
  double* data4Dd = new double [4*NumberRecords];

  uint8_t* data1Du8 = new uint8_t [NumberRecords];
  uint16_t* data1Du16 = new uint16_t [NumberRecords];
  uint32_t* data1Du32 = new uint32_t [NumberRecords];
  int8_t* data1Di8 = new int8_t [NumberRecords];
  int16_t* data1Di16 = new int16_t [NumberRecords];
  int32_t* data1Di32 = new int32_t [NumberRecords];


  /* FMU data: time, input voltage, and regulated voltage */
  Logger.WriteData("/Fmu","Time_us",Time_us,"Time, us",NumberRecords,1);
  for (size_t k=0; k < NumberRecords; k++) {
    data1D[k] = InputVoltage[k].Voltage_V;
  }
  Logger.WriteData("/Fmu","InputVoltage_V",data1D,"Input voltage, V",NumberRecords,1);
  for (size_t k=0; k < NumberRecords; k++) {
    data1D[k] = RegulatedVoltage[k].Voltage_V;
  }
  Logger.WriteData("/Fmu","RegulatedVoltage_V",data1D,"Regulated voltage, V",NumberRecords,1);


  /* MPU-9250 data: accel, gyro, mag, and temperature */
  size_t m = 0;
  for (size_t k=0; k < NumberRecords; k++) {
    data3D[m] = Mpu9250[k].Accel_mss(0,0);
    data3D[m+1] = Mpu9250[k].Accel_mss(1,0);
    data3D[m+2] = Mpu9250[k].Accel_mss(2,0);
    m = m + 3;
  }
  Logger.WriteData("/Fmu/Mpu9250","Accel_mss",data3D,"X, Y, Z accelerometer translated to aircraft body axis system, m/s/s",NumberRecords,3);
  m = 0;
  for (size_t k=0; k < NumberRecords; k++) {
    data3D[m] = Mpu9250[k].Gyro_rads(0,0);
    data3D[m+1] = Mpu9250[k].Gyro_rads(1,0);
    data3D[m+2] = Mpu9250[k].Gyro_rads(2,0);
    m = m + 3;
  }
  Logger.WriteData("/Fmu/Mpu9250","Gyro_rads",data3D,"X, Y, Z gyro translated to aircraft body axis system, rad/s",NumberRecords,3);
  m = 0;
  for (size_t k=0; k < NumberRecords; k++) {
    data3D[m] = Mpu9250[k].Mag_uT(0,0);
    data3D[m+1] = Mpu9250[k].Mag_uT(1,0);
    data3D[m+2] = Mpu9250[k].Mag_uT(2,0);
    m = m + 3;
  }
  Logger.WriteData("/Fmu/Mpu9250","Mag_uT",data3D,"X, Y, Z magnetometer translated to aircraft body axis system, uT",NumberRecords,3);
  for (size_t k=0; k < NumberRecords; k++) {
    data1D[k] = Mpu9250[k].Temp_C;
  }
  Logger.WriteData("/Fmu/Mpu9250","Temp_C",data1D,"Temperature, C",NumberRecords,1);

  /* BME-280 data: pressure, temperature, humidity */
  for (size_t k=0; k < NumberRecords; k++) {
    data1D[k] = Bme280[k].Pressure_Pa;
  }
  Logger.WriteData("/Fmu/Bme280","Pressure_Pa",data1D,"Static pressure, Pa",NumberRecords,1);
  for (size_t k=0; k < NumberRecords; k++) {
    data1D[k] = Bme280[k].Temp_C;
  }
  Logger.WriteData("/Fmu/Bme280","Temp_C",data1D,"Temperature, C",NumberRecords,1);
  for (size_t k=0; k < NumberRecords; k++) {
    data1D[k] = Bme280[k].Humidity_RH;
  }
  Logger.WriteData("/Fmu/Bme280","Humidity_RH",data1D,"Percent relative humidity",NumberRecords,1);


string GroupName;

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


  /* AirdataStruct */
  GroupName = "/Soc/Airdata";

  Logger.CreateGroup(GroupName);

  for (size_t k=0; k < NumberRecords; k++) {
    data1D[k] = airdataData[k].temp_C;
  }
  Logger.WriteData(GroupName,"temp_C",data1D,"Temperature, C",NumberRecords,1);

  for (size_t k=0; k < NumberRecords; k++) {
    data1D[k] = airdataData[k].alt_m;
  }
  Logger.WriteData(GroupName,"alt_m",data1D,"Altitude, m",NumberRecords,1);

  for (size_t k=0; k < NumberRecords; k++) {
    data1D[k] = airdataData[k].altFilt_m;
  }
  Logger.WriteData(GroupName,"altFilt_m",data1D,"Altitude Filterd, m",NumberRecords,1);

  for (size_t k=0; k < NumberRecords; k++) {
    data1D[k] = airdataData[k].vIas_mps;
  }
  Logger.WriteData(GroupName,"vIas_mps",data1D,"Airspeed, mps",NumberRecords,1);

  for (size_t k=0; k < NumberRecords; k++) {
    data1D[k] = airdataData[k].vIasFilt_mps;
  }
  Logger.WriteData(GroupName,"vIasFilt_mps",data1D,"Airspeed Filtered, mps",NumberRecords,1);


  /* Navigation */
  GroupName = "/Soc/NavFilter";

  Logger.CreateGroup(GroupName);

  for (size_t k=0; k < NumberRecords; k++) {
    data1Dd[k] = navData[k].Time_s;
  }
  Logger.WriteData(GroupName,"Time_s",data1D,"Time, s",NumberRecords,1);

  m = 0;
  for (size_t k=0; k < NumberRecords; k++) {
    data3Dd[m] = navData[k].LLA(0,0);
    data3Dd[m+1] = navData[k].LLA(1,0);
    data3Dd[m+2] = navData[k].LLA(2,0);
    m = m + 3;
  }
  Logger.WriteData(GroupName,"LLA",data3Dd,"Latitude (rad), Longitude (rad), Altitude (m)",NumberRecords,3);

  m = 0;
  for (size_t k=0; k < NumberRecords; k++) {
    data3Dd[m] = navData[k].NEDVelocity_ms(0,0);
    data3Dd[m+1] = navData[k].NEDVelocity_ms(1,0);
    data3Dd[m+2] = navData[k].NEDVelocity_ms(2,0);
    m = m + 3;
  }
  Logger.WriteData(GroupName,"NEDVelocity_ms",data3Dd,"North, East, Down Velocity, m/s",NumberRecords,3);

  m = 0;
  for (size_t k=0; k < NumberRecords; k++) {
    data3Dd[m] = navData[k].Euler_rad(0,0);
    data3Dd[m+1] = navData[k].Euler_rad(1,0);
    data3Dd[m+2] = navData[k].Euler_rad(2,0);
    m = m + 3;
  }
  Logger.WriteData(GroupName,"Euler_rad",data3Dd,"Euler Roll, Pitch, Yaw Orientation, rad",NumberRecords,3);

  m = 0;
  for (size_t k=0; k < NumberRecords; k++) {
    data4Dd[m] = navData[k].Quaternion(0,0);
    data4Dd[m+1] = navData[k].Quaternion(1,0);
    data4Dd[m+2] = navData[k].Quaternion(2,0);
    data4Dd[m+3] = navData[k].Quaternion(3,0);
    m = m + 4;
  }
  Logger.WriteData(GroupName,"Quaternion",data4Dd,"Quaternion Orientation, nd",NumberRecords,4);

  m = 0;
  for (size_t k=0; k < NumberRecords; k++) {
    data3Dd[m] = navData[k].AccelBias_mss(0,0);
    data3Dd[m+1] = navData[k].AccelBias_mss(1,0);
    data3Dd[m+2] = navData[k].AccelBias_mss(2,0);
    m = m + 3;
  }
  Logger.WriteData(GroupName,"AccelBias_mss",data3Dd,"Accel Bias X, Y, Z, mps2",NumberRecords,3);

  m = 0;
  for (size_t k=0; k < NumberRecords; k++) {
    data3Dd[m] = navData[k].GyroBias_rads(0,0);
    data3Dd[m+1] = navData[k].GyroBias_rads(1,0);
    data3Dd[m+2] = navData[k].GyroBias_rads(2,0);
    m = m + 3;
  }
  Logger.WriteData(GroupName,"GyroBias_rads",data3Dd,"Gyro Bias X, Y, Z, rps",NumberRecords,3);

  m = 0;
  for (size_t k=0; k < NumberRecords; k++) {
    data3Dd[m] = navData[k].Pp(0,0);
    data3Dd[m+1] = navData[k].Pp(1,0);
    data3Dd[m+2] = navData[k].Pp(2,0);
    m = m + 3;
  }
  Logger.WriteData(GroupName,"Pp",data3Dd,"Position Covariance, rad",NumberRecords,3);

  m = 0;
  for (size_t k=0; k < NumberRecords; k++) {
    data3Dd[m] = navData[k].Pv(0,0);
    data3Dd[m+1] = navData[k].Pv(1,0);
    data3Dd[m+2] = navData[k].Pv(2,0);
    m = m + 3;
  }
  Logger.WriteData(GroupName,"Pv",data3Dd,"Velocity Covariance, mps",NumberRecords,3);

  m = 0;
  for (size_t k=0; k < NumberRecords; k++) {
    data3Dd[m] = navData[k].Pa(0,0);
    data3Dd[m+1] = navData[k].Pa(1,0);
    data3Dd[m+2] = navData[k].Pa(2,0);
    m = m + 3;
  }
  Logger.WriteData(GroupName,"Pa",data3Dd,"Angle Covariance, rad",NumberRecords,3);

  m = 0;
  for (size_t k=0; k < NumberRecords; k++) {
    data3Dd[m] = navData[k].Pab(0,0);
    data3Dd[m+1] = navData[k].Pab(1,0);
    data3Dd[m+2] = navData[k].Pab(2,0);
    m = m + 3;
  }
  Logger.WriteData(GroupName,"Pab",data3Dd,"Accel Bias Covariance, mps2",NumberRecords,3);

  m = 0;
  for (size_t k=0; k < NumberRecords; k++) {
    data3Dd[m] = navData[k].Pgb(0,0);
    data3Dd[m+1] = navData[k].Pgb(1,0);
    data3Dd[m+2] = navData[k].Pgb(2,0);
    m = m + 3;
  }
  Logger.WriteData(GroupName,"Pgb",data3Dd,"Gyro Bias Covariance, rps",NumberRecords,3);


  /* Mission Manager */
  GroupName = "/Soc/MissionMgr";

  Logger.CreateGroup(GroupName);

  for (size_t k=0; k < NumberRecords; k++) {
    data1D[k] = missMgrData[k].time_s;
  }
  Logger.WriteData(GroupName,"time_s",data1D,"Mission Time, s",NumberRecords,1);

  for (size_t k=0; k < NumberRecords; k++) {
    data1Du32[k] = missMgrData[k].frame_cnt;
  }
  Logger.WriteData(GroupName,"frame_cnt",data1Du32,"Frame Count, cnt",NumberRecords,1);

  for (size_t k=0; k < NumberRecords; k++) {
    data1Du8[k] = missMgrData[k].autoEngage;
  }
  Logger.WriteData(GroupName,"autoEngage",data1Du8,"Auto Engaged, bool",NumberRecords,1);

  for (size_t k=0; k < NumberRecords; k++) {
    data1Di8[k] = missMgrData[k].cntrlMode;
  }
  Logger.WriteData(GroupName,"cntrlMode",data1Di8,"Control System Mode, int",NumberRecords,1);

  for (size_t k=0; k < NumberRecords; k++) {
    data1Du8[k] = missMgrData[k].trigArm;
  }
  Logger.WriteData(GroupName,"trigArm",data1Du8,"Trigger Armed, bool",NumberRecords,1);

  for (size_t k=0; k < NumberRecords; k++) {
    data1Du8[k] = missMgrData[k].trigEngage;
  }
  Logger.WriteData(GroupName,"trigEngage",data1Du8,"Trigger Engaged, bool",NumberRecords,1);

  for (size_t k=0; k < NumberRecords; k++) {
    data1Du8[k] = missMgrData[k].testArm;
  }
  Logger.WriteData(GroupName,"testArm",data1Du8,"Tester Armed, bool",NumberRecords,1);

  for (size_t k=0; k < NumberRecords; k++) {
    data1Du8[k] = missMgrData[k].testEngage;
  }
  Logger.WriteData(GroupName,"testEngage",data1Du8,"Tester Engaged, bool",NumberRecords,1);

  for (size_t k=0; k < NumberRecords; k++) {
    data1Du8[k] = missMgrData[k].indxTest;
  }
  Logger.WriteData(GroupName,"indxTest",data1Du8,"Tester Index, cnt",NumberRecords,1);

  for (size_t k=0; k < NumberRecords; k++) {
    data1Du8[k] = missMgrData[k].numTest;
  }
  Logger.WriteData(GroupName,"numTest",data1Du8,"Tester Total, cnt",NumberRecords,1);


  /* Excitation Manager */
  GroupName = "/Soc/ExcitMgr";

  Logger.CreateGroup(GroupName);

  for (size_t k=0; k < NumberRecords; k++) {
    data1Du8[k] = exciteMgrData[k].exciteMode;
  }
  Logger.WriteData(GroupName,"exciteMode",data1Du8,"Excitation Mode, bool",NumberRecords,1);

  for (size_t k=0; k < NumberRecords; k++) {
    data1Du8[k] = exciteMgrData[k].indxTest;
  }
  Logger.WriteData(GroupName,"indxTest",data1Du8,"Excitation Index Number, int",NumberRecords,1);

  for (size_t k=0; k < NumberRecords; k++) {
    data1D[k] = exciteMgrData[k].timeExcite_s;
  }
  Logger.WriteData(GroupName,"timeExcite_s",data1D,"Time, s",NumberRecords,1);

  m = 0;
  for (size_t k=0; k < NumberRecords; k++) {
    data4D[m] = exciteMgrData[k].cmdExcite(0,0);
    data4D[m+1] = exciteMgrData[k].cmdExcite(1,0);
    data4D[m+2] = exciteMgrData[k].cmdExcite(2,0);
    data4D[m+3] = exciteMgrData[k].cmdExcite(3,0);
    m = m + 4;
  }
  Logger.WriteData(GroupName,"cmdExcite",data4D,"Excitation Commands, mag",NumberRecords,4);


  /* Control System Manager */
  GroupName = "/Soc/CntrlMgr";

  Logger.CreateGroup(GroupName);

  m = 0;
  for (size_t k=0; k < NumberRecords; k++) {
    data4D[m] = cntrlMgrData[k].cmdBase(0,0);
    data4D[m+1] = cntrlMgrData[k].cmdBase(1,0);
    data4D[m+2] = cntrlMgrData[k].cmdBase(2,0);
    data4D[m+3] = cntrlMgrData[k].cmdBase(3,0);
    m = m + 4;
  }
  Logger.WriteData(GroupName,"cmdBase",data4D,"Controller Baseline Commands, Roll [rps], Pitch [rps], Yaw [rps], Throttle [nd]",NumberRecords,4);
  
  m = 0;
  for (size_t k=0; k < NumberRecords; k++) {
    data4D[m] = cntrlMgrData[k].cmdRes(0,0);
    data4D[m+1] = cntrlMgrData[k].cmdRes(1,0);
    data4D[m+2] = cntrlMgrData[k].cmdRes(2,0);
    data4D[m+3] = cntrlMgrData[k].cmdRes(3,0);
    m = m + 4;
  }
  Logger.WriteData(GroupName,"cmdRes",data4D,"Controller Research Commands, Roll [rps], Pitch [rps], Yaw [rps], Throttle [nd]",NumberRecords,4);

  m = 0;
  for (size_t k=0; k < NumberRecords; k++) {
    data4D[m] = cntrlMgrData[k].cmd(0,0);
    data4D[m+1] = cntrlMgrData[k].cmd(1,0);
    data4D[m+2] = cntrlMgrData[k].cmd(2,0);
    data4D[m+3] = cntrlMgrData[k].cmd(3,0);
    m = m + 4;
  }
  Logger.WriteData(GroupName,"cmd",data4D,"Controller Commands, Roll [rps], Pitch [rps], Yaw [rps], Throttle [nd]",NumberRecords,4);

  for (size_t k=0; k < NumberRecords; k++) {
    data1Di8[k] = cntrlMgrData[k].mode;
  }
  Logger.WriteData(GroupName,"mode",data1Di8,"Mode, int",NumberRecords,1);


  /* Control Allocation Manager */
  GroupName = "/Soc/CntrlAlloc";

  Logger.CreateGroup(GroupName);

  m = 0;
  for (size_t k=0; k < NumberRecords; k++) {
    data6D[m] = cntrlAllocData[k].cmdAlloc(0,0);
    data6D[m+1] = cntrlAllocData[k].cmdAlloc(1,0);
    data6D[m+2] = cntrlAllocData[k].cmdAlloc(2,0);
    data6D[m+3] = cntrlAllocData[k].cmdAlloc(3,0);
    data6D[m+4] = cntrlAllocData[k].cmdAlloc(4,0);
    data6D[m+5] = cntrlAllocData[k].cmdAlloc(5,0);
    m = m + 6;
  }
  Logger.WriteData(GroupName,"cmdAlloc",data6D,"Control Allocation Commands, Elev, Rud, AilR, FlapR, FlapL, AilL",NumberRecords,6);
  

	return 0;
}
