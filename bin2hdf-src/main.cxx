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
#include "../soc-src/hardware-defs.hxx"
#include "../soc-src/fmu.hxx"
#include "../soc-src/datalogger.hxx"
#include <H5Cpp.h>
#include <iostream>
#include <string>
#include <vector>

using namespace std;
using namespace H5;

int main(int argc, char* argv[]) {
  if (argc!=3) {
    cerr << "ERROR: Incorrect number of input arguments." << endl;
    return -1;
  }

  /* declare classes */
  FlightManagementUnit Fmu(FmuPort,FmuBaud);
  Datalogger Datalog;

  /* load the flight data file*/
  FILE *BinaryFile = fopen(argv[1],"rb");

  cout << "Opening: " << argv[1] << endl;

  /* create an HDF5 file */
  hdf5class Logger(argv[2]);

  /* figure out how big the file is */
  fseek(BinaryFile,0,SEEK_END);
  size_t size = ftell(BinaryFile);
  rewind(BinaryFile);

  cout << "File size: " << size << " bytes"<< endl;

  /* read binary file into memory */
  vector<uint8_t> Buffer;
  Buffer.resize(size);
  fread(Buffer.data(),1,size,BinaryFile);
  fclose(BinaryFile);

  // source and payload for parsing
  Datalogger::DataSources Source;
  vector<uint8_t> Payload;

  // Vector of parsed sensor data
  vector<struct FlightManagementUnit::SensorData> SensorData;

  /* parse binary files into data structures */
  for (size_t i=0; i < Buffer.size(); i++) {
    // datalog payload found
    if (Datalog.ReadData(Buffer[i],&Source,&Payload)) {
      // payload is a sensor payload
      if (Source == Datalogger::SensorData) {
        struct FlightManagementUnit::SensorData TempData;
        Fmu.DeserializeSensorData(Payload);
        Fmu.GetSensorData(&TempData);
        SensorData.push_back(TempData);
      }
    }
  }

  cout << SensorData.size() << endl;

  // std::vector<uint64_t> Time_us;

  // for (size_t i=0; i < size; i++) {
  //   Time_us
  // }



  // // /* parse binary files into data structures */
  // uint64_t* Time_us = new uint64_t [NumberRecords];
  // Voltage* InputVoltage = new Voltage [NumberRecords];
  // Voltage* RegulatedVoltage = new Voltage [NumberRecords];
  // Mpu9250Data* Mpu9250 = new Mpu9250Data [NumberRecords];
  // Bme280Data* Bme280 = new Bme280Data [NumberRecords];
  // Mpu9250Data** Mpu9250Ext = new Mpu9250Data* [Data.Mpu9250Ext.size()];
  // for (size_t i=0; i < Data.Mpu9250Ext.size(); i++) {
  //   Mpu9250Ext[i] = new Mpu9250Data [NumberRecords];
  // }
  // Bme280Data** Bme280Ext = new Bme280Data* [Data.Bme280Ext.size()];
  // for (size_t i=0; i < Data.Bme280Ext.size(); i++) {
  //   Bme280Ext[i] = new Bme280Data [NumberRecords];
  // }
  // SbusRxData** SbusRx = new SbusRxData* [Data.SbusRx.size()];
  // for (size_t i=0; i < Data.SbusRx.size(); i++) {
  //   SbusRx[i] = new SbusRxData [NumberRecords];
  // }
  // GpsData** Gps = new GpsData* [Data.Gps.size()];
  // for (size_t i=0; i < Data.Gps.size(); i++) {
  //   Gps[i] = new GpsData [NumberRecords];
  // }
  // PitotData** Pitot = new PitotData* [Data.Pitot.size()];
  // for (size_t i=0; i < Data.Pitot.size(); i++) {
  //   Pitot[i] = new PitotData [NumberRecords];
  // }
  // PressureData** PressureTransducer = new PressureData* [Data.PressureTransducer.size()];
  // for (size_t i=0; i < Data.PressureTransducer.size(); i++) {
  //   PressureTransducer[i] = new PressureData [NumberRecords];
  // }
  // AnalogData** Analog = new AnalogData* [Data.Analog.size()];
  // for (size_t i=0; i < Data.Analog.size(); i++) {
  //   Analog[i] = new AnalogData [NumberRecords];
  // }
  // Voltage** SbusVoltage = new Voltage* [Data.SbusVoltage.size()];
  // for (size_t i=0; i < Data.SbusVoltage.size(); i++) {
  //   SbusVoltage[i] = new Voltage [NumberRecords];
  // }
  // Voltage** PwmVoltage = new Voltage* [Data.PwmVoltage.size()];
  // for (size_t i=0; i < Data.PwmVoltage.size(); i++) {
  //   PwmVoltage[i] = new Voltage [NumberRecords];
  // }

  // for (size_t i=0; i < NumberRecords; i++) {
  //   memcpy(&Time_us[i],FileBuffer[i],sizeof(Time_us[i]));
  //   memcpy(&InputVoltage[i],FileBuffer[i]+sizeof(Time_us[i]),sizeof(InputVoltage[i]));
  //   memcpy(&RegulatedVoltage[i],FileBuffer[i]+sizeof(Time_us[i])+sizeof(InputVoltage[i]),sizeof(RegulatedVoltage[i]));
  //   memcpy(&Mpu9250[i],FileBuffer[i]+sizeof(Time_us[i])+sizeof(InputVoltage[i])+sizeof(RegulatedVoltage[i]),sizeof(Mpu9250[i]));
  //   memcpy(&Bme280[i],FileBuffer[i]+sizeof(Time_us[i])+sizeof(InputVoltage[i])+sizeof(RegulatedVoltage[i])+sizeof(Mpu9250[i]),sizeof(Bme280[i]));
  //   for (size_t j=0; j < Data.Mpu9250Ext.size(); j++) {
  //     memcpy(&Mpu9250Ext[j][i],FileBuffer[i]+sizeof(Time_us[i])+sizeof(InputVoltage[i])+sizeof(RegulatedVoltage[i])+sizeof(Mpu9250[i])+sizeof(Bme280[i])+j*sizeof(Mpu9250Ext[j][i]),sizeof(Mpu9250Ext[j][i]));
  //   }
  //   for (size_t j=0; j < Data.Bme280Ext.size(); j++) {
  //     memcpy(&Bme280Ext[j][i],FileBuffer[i]+sizeof(Time_us[i])+sizeof(InputVoltage[i])+sizeof(RegulatedVoltage[i])+sizeof(Mpu9250[i])+sizeof(Bme280[i])+
  //       Data.Mpu9250Ext.size()*sizeof(Mpu9250Data)+j*sizeof(Bme280Ext[j][i]),sizeof(Bme280Ext[j][i]));
  //   }
  //   for (size_t j=0; j < Data.SbusRx.size(); j++) {
  //     memcpy(&SbusRx[j][i],FileBuffer[i]+sizeof(Time_us[i])+sizeof(InputVoltage[i])+sizeof(RegulatedVoltage[i])+sizeof(Mpu9250[i])+sizeof(Bme280[i])+
  //       Data.Mpu9250Ext.size()*sizeof(Mpu9250Data)+Data.Bme280Ext.size()*sizeof(Bme280Data)+j*sizeof(SbusRx[j][i]),sizeof(SbusRx[j][i]));
  //   }
  //   for (size_t j=0; j < Data.Gps.size(); j++) {
  //     memcpy(&Gps[j][i],FileBuffer[i]+sizeof(Time_us[i])+sizeof(InputVoltage[i])+sizeof(RegulatedVoltage[i])+sizeof(Mpu9250[i])+sizeof(Bme280[i])+
  //       Data.Mpu9250Ext.size()*sizeof(Mpu9250Data)+Data.Bme280Ext.size()*sizeof(Bme280Data)+Data.SbusRx.size()*sizeof(SbusRxData)+j*sizeof(Gps[j][i]),sizeof(Gps[j][i]));
  //   }
  //   for (size_t j=0; j < Data.Pitot.size(); j++) {
  //     memcpy(&Pitot[j][i],FileBuffer[i]+sizeof(Time_us[i])+sizeof(InputVoltage[i])+sizeof(RegulatedVoltage[i])+sizeof(Mpu9250[i])+sizeof(Bme280[i])+
  //       Data.Mpu9250Ext.size()*sizeof(Mpu9250Data)+Data.Bme280Ext.size()*sizeof(Bme280Data)+Data.SbusRx.size()*sizeof(SbusRxData)+Data.Gps.size()*sizeof(GpsData)+
  //       j*sizeof(Pitot[j][i]),sizeof(Pitot[j][i]));
  //   }
  //   for (size_t j=0; j < Data.PressureTransducer.size(); j++) {
  //     memcpy(&PressureTransducer[j][i],FileBuffer[i]+sizeof(Time_us[i])+sizeof(InputVoltage[i])+sizeof(RegulatedVoltage[i])+sizeof(Mpu9250[i])+sizeof(Bme280[i])+
  //       Data.Mpu9250Ext.size()*sizeof(Mpu9250Data)+Data.Bme280Ext.size()*sizeof(Bme280Data)+Data.SbusRx.size()*sizeof(SbusRxData)+Data.Gps.size()*sizeof(GpsData)+
  //       Data.Pitot.size()*sizeof(PitotData)+j*sizeof(PressureTransducer[j][i]),sizeof(PressureTransducer[j][i]));
  //   }
  //   for (size_t j=0; j < Data.Analog.size(); j++) {
  //     memcpy(&Analog[j][i],FileBuffer[i]+sizeof(Time_us[i])+sizeof(InputVoltage[i])+sizeof(RegulatedVoltage[i])+sizeof(Mpu9250[i])+sizeof(Bme280[i])+
  //       Data.Mpu9250Ext.size()*sizeof(Mpu9250Data)+Data.Bme280Ext.size()*sizeof(Bme280Data)+Data.SbusRx.size()*sizeof(SbusRxData)+Data.Gps.size()*sizeof(GpsData)+
  //       Data.Pitot.size()*sizeof(PitotData)+Data.PressureTransducer.size()*sizeof(PressureData)+j*sizeof(Analog[j][i]),sizeof(Analog[j][i]));
  //   }
  //   for (size_t j=0; j < Data.SbusVoltage.size(); j++) {
  //     memcpy(&SbusVoltage[j][i],FileBuffer[i]+sizeof(Time_us[i])+sizeof(InputVoltage[i])+sizeof(RegulatedVoltage[i])+sizeof(Mpu9250[i])+sizeof(Bme280[i])+
  //       Data.Mpu9250Ext.size()*sizeof(Mpu9250Data)+Data.Bme280Ext.size()*sizeof(Bme280Data)+Data.SbusRx.size()*sizeof(SbusRxData)+Data.Gps.size()*sizeof(GpsData)+
  //       Data.Pitot.size()*sizeof(PitotData)+Data.PressureTransducer.size()*sizeof(PressureData)+Data.Analog.size()*sizeof(AnalogData)+j*sizeof(SbusVoltage[j][i]),sizeof(SbusVoltage[j][i]));
  //   }
  //   for (size_t j=0; j < Data.PwmVoltage.size(); j++) {
  //     memcpy(&PwmVoltage[j][i],FileBuffer[i]+sizeof(Time_us[i])+sizeof(InputVoltage[i])+sizeof(RegulatedVoltage[i])+sizeof(Mpu9250[i])+sizeof(Bme280[i])+
  //       Data.Mpu9250Ext.size()*sizeof(Mpu9250Data)+Data.Bme280Ext.size()*sizeof(Bme280Data)+Data.SbusRx.size()*sizeof(SbusRxData)+Data.Gps.size()*sizeof(GpsData)+
  //       Data.Pitot.size()*sizeof(PitotData)+Data.PressureTransducer.size()*sizeof(PressureData)+Data.Analog.size()*sizeof(AnalogData)+Data.SbusVoltage.size()*sizeof(Voltage)+
  //       j*sizeof(PwmVoltage[j][i]),sizeof(PwmVoltage[j][i]));
  //   }
  // }

  // /* save data into HDF5 */
  // float* data1D = new float [NumberRecords];
  // float* data3D = new float [3*NumberRecords];
  // float* data5D = new float [5*NumberRecords];
  // double* data1Dd = new double [NumberRecords];
  // double* data3Dd = new double [3*NumberRecords];
  // uint8_t* data1Du8 = new uint8_t [NumberRecords];
  // uint16_t* data1Du16 = new uint16_t [NumberRecords];
  // uint32_t* data1Du32 = new uint32_t [NumberRecords];

  // /* FMU data: time, input voltage, and regulated voltage */
  // Logger.WriteData("/Fmu","Time_us",Time_us,"Time, us",NumberRecords,1);
  // for (size_t k=0; k < NumberRecords; k++) {
  //   data1D[k] = InputVoltage[k].Voltage_V;
  // }
  // Logger.WriteData("/Fmu","InputVoltage_V",data1D,"Input voltage, V",NumberRecords,1);
  // for (size_t k=0; k < NumberRecords; k++) {
  //   data1D[k] = RegulatedVoltage[k].Voltage_V;
  // }
  // Logger.WriteData("/Fmu","RegulatedVoltage_V",data1D,"Regulated voltage, V",NumberRecords,1);

  // /* MPU-9250 data: accel, gyro, mag, and temperature */
  // size_t m = 0;
  // for (size_t k=0; k < NumberRecords; k++) {
  //   data3D[m] = Mpu9250[k].Accel_mss(0,0);
  //   data3D[m+1] = Mpu9250[k].Accel_mss(1,0);
  //   data3D[m+2] = Mpu9250[k].Accel_mss(2,0);
  //   m = m + 3;
  // }
  // Logger.WriteData("/Fmu/Mpu9250","Accel_mss",data3D,"X, Y, Z accelerometer translated to aircraft body axis system, m/s/s",NumberRecords,3);
  // m = 0;
  // for (size_t k=0; k < NumberRecords; k++) {
  //   data3D[m] = Mpu9250[k].Gyro_rads(0,0);
  //   data3D[m+1] = Mpu9250[k].Gyro_rads(1,0);
  //   data3D[m+2] = Mpu9250[k].Gyro_rads(2,0);
  //   m = m + 3;
  // }
  // Logger.WriteData("/Fmu/Mpu9250","Gyro_rads",data3D,"X, Y, Z gyro translated to aircraft body axis system, rad/s",NumberRecords,3);
  // m = 0;
  // for (size_t k=0; k < NumberRecords; k++) {
  //   data3D[m] = Mpu9250[k].Mag_uT(0,0);
  //   data3D[m+1] = Mpu9250[k].Mag_uT(1,0);
  //   data3D[m+2] = Mpu9250[k].Mag_uT(2,0);
  //   m = m + 3;
  // }
  // Logger.WriteData("/Fmu/Mpu9250","Mag_uT",data3D,"X, Y, Z magnetometer translated to aircraft body axis system, uT",NumberRecords,3);
  // for (size_t k=0; k < NumberRecords; k++) {
  //   data1D[k] = Mpu9250[k].Temp_C;
  // }
  // Logger.WriteData("/Fmu/Mpu9250","Temp_C",data1D,"Temperature, C",NumberRecords,1);

  // /* BME-280 data: pressure, temperature, humidity */
  // for (size_t k=0; k < NumberRecords; k++) {
  //   data1D[k] = Bme280[k].Pressure_Pa;
  // }
  // Logger.WriteData("/Fmu/Bme280","Pressure_Pa",data1D,"Static pressure, Pa",NumberRecords,1);
  // for (size_t k=0; k < NumberRecords; k++) {
  //   data1D[k] = Bme280[k].Temp_C;
  // }
  // Logger.WriteData("/Fmu/Bme280","Temp_C",data1D,"Temperature, C",NumberRecords,1);
  // for (size_t k=0; k < NumberRecords; k++) {
  //   data1D[k] = Bme280[k].Humidity_RH;
  // }
  // Logger.WriteData("/Fmu/Bme280","Humidity_RH",data1D,"Percent relative humidity",NumberRecords,1);

  // /* External MPU-9250 */
  // for (size_t l=0; l < Data.Mpu9250Ext.size(); l++) {
  //   string GroupName = Config.Mpu9250Names[l];
  //   m = 0;
  //   for (size_t k=0; k < NumberRecords; k++) {
  //     data3D[m] = Mpu9250Ext[l][k].Accel_mss(0,0);
  //     data3D[m+1] = Mpu9250Ext[l][k].Accel_mss(1,0);
  //     data3D[m+2] = Mpu9250Ext[l][k].Accel_mss(2,0);
  //     m = m + 3;
  //   }
  //   Logger.WriteData(GroupName,"Accel_mss",data3D,"X, Y, Z accelerometer translated to aircraft body axis system, m/s/s",NumberRecords,3);
  //   m = 0;
  //   for (size_t k=0; k < NumberRecords; k++) {
  //     data3D[m] = Mpu9250Ext[l][k].Gyro_rads(0,0);
  //     data3D[m+1] = Mpu9250Ext[l][k].Gyro_rads(1,0);
  //     data3D[m+2] = Mpu9250Ext[l][k].Gyro_rads(2,0);
  //     m = m + 3;
  //   }
  //   Logger.WriteData(GroupName,"Gyro_rads",data3D,"X, Y, Z gyro translated to aircraft body axis system, rad/s",NumberRecords,3);
  //   m = 0;
  //   for (size_t k=0; k < NumberRecords; k++) {
  //     data3D[m] = Mpu9250Ext[l][k].Mag_uT(0,0);
  //     data3D[m+1] = Mpu9250Ext[l][k].Mag_uT(1,0);
  //     data3D[m+2] = Mpu9250Ext[l][k].Mag_uT(2,0);
  //     m = m + 3;
  //   }
  //   Logger.WriteData(GroupName,"Mag_uT",data3D,"X, Y, Z magnetometer translated to aircraft body axis system, uT",NumberRecords,3);
  //   for (size_t k=0; k < NumberRecords; k++) {
  //     data1D[k] = Mpu9250Ext[l][k].Temp_C;
  //   }
  //   Logger.WriteData(GroupName,"Temp_C",data1D,"Temperature, C",NumberRecords,1);
  // }

  // /* External BME-280 */
  // for (size_t l=0; l < Data.Bme280Ext.size(); l++) {
  //   string GroupName = Config.Bme280Names[l];
  //   for (size_t k=0; k < NumberRecords; k++) {
  //     data1D[k] = Bme280Ext[l][k].Pressure_Pa;
  //   }
  //   Logger.WriteData(GroupName,"Pressure_Pa",data1D,"Static pressure, Pa",NumberRecords,1);
  //   for (size_t k=0; k < NumberRecords; k++) {
  //     data1D[k] = Bme280Ext[l][k].Temp_C;
  //   }
  //   Logger.WriteData(GroupName,"Temp_C",data1D,"Temperature, C",NumberRecords,1);
  //   for (size_t k=0; k < NumberRecords; k++) {
  //     data1D[k] = Bme280Ext[l][k].Humidity_RH;
  //   }
  //   Logger.WriteData(GroupName,"Humidity_RH",data1D,"Percent relative humidity",NumberRecords,1);
  // }

  // /* Sbus RX */
  // for (size_t l=0; l < Data.SbusRx.size(); l++) {
  //   string GroupName = Config.SbusRxNames[l];
  //   for (size_t k=0; k < NumberRecords; k++) {
  //     data1Du8[k] = SbusRx[l][k].Failsafe;
  //   }
  //   Logger.WriteData(GroupName,"Failsafe",data1Du8,"True when failsafe active",NumberRecords,1);    
  //   for (size_t k=0; k < NumberRecords; k++) {
  //     data1Du16[k] = SbusRx[l][k].LostFrames;
  //   }
  //   Logger.WriteData(GroupName,"LostFrames",data1Du16,"Number of lost frames",NumberRecords,1); 
  //   for (size_t k=0; k < NumberRecords; k++) {
  //     data1Du8[k] = SbusRx[l][k].AutoEnabled;
  //   }
  //   Logger.WriteData(GroupName,"AutoEnabled",data1Du8,"True when autopilot enabled",NumberRecords,1); 
  //   for (size_t k=0; k < NumberRecords; k++) {
  //     data1Du8[k] = SbusRx[l][k].ThrottleEnabled;
  //   }
  //   Logger.WriteData(GroupName,"ThrottleEnabled",data1Du8,"True when throttle enabled",NumberRecords,1);
  //    for (size_t k=0; k < NumberRecords; k++) {
  //     data1D[k] = SbusRx[l][k].RSSI;
  //   }
  //   Logger.WriteData(GroupName,"RSSI",data1D,"RSSI value",NumberRecords,1);
  //   m = 0;
  //   for (size_t k=0; k < NumberRecords; k++) {
  //     data5D[m] = SbusRx[l][k].Inceptors(0,0);
  //     data5D[m+1] = SbusRx[l][k].Inceptors(1,0);
  //     data5D[m+2] = SbusRx[l][k].Inceptors(2,0);
  //     data5D[m+3] = SbusRx[l][k].Inceptors(3,0);
  //     data5D[m+4] = SbusRx[l][k].Inceptors(4,0);
  //     m = m + 5;
  //   }
  //   Logger.WriteData(GroupName,"Inceptors",data5D,"Aerodynamic inceptors, roll pitch yaw lift thrust, normalized to +/- 1",NumberRecords,5);
  //   m = 0;
  //   for (size_t k=0; k < NumberRecords; k++) {
  //     data5D[m] = SbusRx[l][k].AuxInputs(0,0);
  //     data5D[m+1] = SbusRx[l][k].AuxInputs(1,0);
  //     data5D[m+2] = SbusRx[l][k].AuxInputs(2,0);
  //     data5D[m+3] = SbusRx[l][k].AuxInputs(3,0);
  //     data5D[m+4] = SbusRx[l][k].AuxInputs(4,0);
  //     m = m + 5;
  //   }
  //   Logger.WriteData(GroupName,"AuxInputs",data5D,"Auxiliary inputs, normalized to +/- 1",NumberRecords,5);
  // } 

  // /* Gps */
  // for (size_t l=0; l < Data.Gps.size(); l++) {
  //   string GroupName = Config.GpsNames[l];
  //   for (size_t k=0; k < NumberRecords; k++) {
  //     data1Du8[k] = Gps[l][k].Fix;
  //   }
  //   Logger.WriteData(GroupName,"Fix",data1Du8,"True for 3D fix only",NumberRecords,1); 
  //   for (size_t k=0; k < NumberRecords; k++) {
  //     data1Du8[k] = Gps[l][k].NumberSatellites;
  //   }
  //   Logger.WriteData(GroupName,"NumberSatellites",data1Du8,"Number of satellites used in solution",NumberRecords,1); 
  //   for (size_t k=0; k < NumberRecords; k++) {
  //     data1Du32[k] = Gps[l][k].TOW;
  //   }
  //   Logger.WriteData(GroupName,"TOW",data1Du32,"GPS time of the navigation epoch",NumberRecords,1); 
  //   for (size_t k=0; k < NumberRecords; k++) {
  //     data1Du16[k] = Gps[l][k].Year;
  //   }
  //   Logger.WriteData(GroupName,"Year",data1Du16,"UTC year",NumberRecords,1);
  //   for (size_t k=0; k < NumberRecords; k++) {
  //     data1Du8[k] = Gps[l][k].Month;
  //   }
  //   Logger.WriteData(GroupName,"Month",data1Du8,"UTC month",NumberRecords,1); 
  //   for (size_t k=0; k < NumberRecords; k++) {
  //     data1Du8[k] = Gps[l][k].Day;
  //   }
  //   Logger.WriteData(GroupName,"Day",data1Du8,"UTC day",NumberRecords,1); 
  //   for (size_t k=0; k < NumberRecords; k++) {
  //     data1Du8[k] = Gps[l][k].Hour;
  //   }
  //   Logger.WriteData(GroupName,"Hour",data1Du8,"UTC hour",NumberRecords,1); 
  //   for (size_t k=0; k < NumberRecords; k++) {
  //     data1Du8[k] = Gps[l][k].Min;
  //   }
  //   Logger.WriteData(GroupName,"Min",data1Du8,"UTC minute",NumberRecords,1); 
  //   for (size_t k=0; k < NumberRecords; k++) {
  //     data1Du8[k] = Gps[l][k].Sec;
  //   }
  //   Logger.WriteData(GroupName,"Sec",data1Du8,"UTC second",NumberRecords,1); 
  //   m = 0;
  //   for (size_t k=0; k < NumberRecords; k++) {
  //     data3Dd[m] = Gps[l][k].LLA(0,0);
  //     data3Dd[m+1] = Gps[l][k].LLA(1,0);
  //     data3Dd[m+2] = Gps[l][k].LLA(2,0);
  //     m = m + 3;
  //   }
  //   Logger.WriteData(GroupName,"LLA",data3Dd,"Latitude (rad), Longitude (rad), Altitude (m)",NumberRecords,3);
  //   m = 0;
  //   for (size_t k=0; k < NumberRecords; k++) {
  //     data3Dd[m] = Gps[l][k].NEDVelocity_ms(0,0);
  //     data3Dd[m+1] = Gps[l][k].NEDVelocity_ms(1,0);
  //     data3Dd[m+2] = Gps[l][k].NEDVelocity_ms(2,0);
  //     m = m + 3;
  //   }
  //   Logger.WriteData(GroupName,"NEDVelocity_ms",data3Dd,"North, East, Down Velocity, m/s",NumberRecords,3);
  //   m = 0;
  //   for (size_t k=0; k < NumberRecords; k++) {
  //     data3Dd[m] = Gps[l][k].Accuracy(0,0);
  //     data3Dd[m+1] = Gps[l][k].Accuracy(1,0);
  //     data3Dd[m+2] = Gps[l][k].Accuracy(2,0);
  //     m = m + 3;
  //   }
  //   Logger.WriteData(GroupName,"Accuracy",data3Dd,"Horizontal (m), vertical (m), and speed (m/s) accuracy estimates",NumberRecords,3);
  //   for (size_t k=0; k < NumberRecords; k++) {
  //     data1Dd[k] = Gps[l][k].pDOP;
  //   }
  //   Logger.WriteData(GroupName,"pDOP",data1Dd,"Position degree of precision",NumberRecords,1); 
  // }

  // /* Pitot */
  // for (size_t l=0; l < Data.Pitot.size(); l++) {
  //   string GroupName = Config.PitotNames[l];
  //   Logger.CreateGroup(GroupName);
  //   for (size_t k=0; k < NumberRecords; k++) {
  //     data1D[k] = Pitot[l][k].Static.Pressure_Pa;
  //   }
  //   Logger.WriteData(GroupName + "/Static","Pressure_Pa",data1D,"Static pressure, Pa",NumberRecords,1);
  //   for (size_t k=0; k < NumberRecords; k++) {
  //     data1D[k] = Pitot[l][k].Static.Temp_C;
  //   }
  //   Logger.WriteData(GroupName + "/Static","Temp_C",data1D,"Temperature, C",NumberRecords,1);
  //   for (size_t k=0; k < NumberRecords; k++) {
  //     data1D[k] = Pitot[l][k].Diff.Pressure_Pa;
  //   }
  //   Logger.WriteData(GroupName + "/Diff","Pressure_Pa",data1D,"Differential pressure, Pa",NumberRecords,1);
  //   for (size_t k=0; k < NumberRecords; k++) {
  //     data1D[k] = Pitot[l][k].Diff.Temp_C;
  //   }
  //   Logger.WriteData(GroupName + "/Diff","Temp_C",data1D,"Temperature, C",NumberRecords,1);
  // }

  // /* Pressure */
  // for (size_t l=0; l < Data.PressureTransducer.size(); l++) {
  //   string GroupName = Config.PressureTransducerNames[l];
  //   for (size_t k=0; k < NumberRecords; k++) {
  //     data1D[k] = PressureTransducer[l][k].Pressure_Pa;
  //   }
  //   Logger.WriteData(GroupName,"Pressure_Pa",data1D,"Pressure, Pa",NumberRecords,1);
  //   for (size_t k=0; k < NumberRecords; k++) {
  //     data1D[k] = PressureTransducer[l][k].Temp_C;
  //   }
  //   Logger.WriteData(GroupName,"Temp_C",data1D,"Temperature, C",NumberRecords,1);
  // }

  // /* Analog */
  // for (size_t l=0; l < Data.Analog.size(); l++) {
  //   string GroupName = Config.AnalogNames[l];
  //   for (size_t k=0; k < NumberRecords; k++) {
  //     data1D[k] = Analog[l][k].Voltage_V;
  //   }
  //   Logger.WriteData(GroupName,"Voltage_V",data1D,"Measured voltage, V",NumberRecords,1);
  //   for (size_t k=0; k < NumberRecords; k++) {
  //     data1D[k] = Analog[l][k].CalValue;
  //   }
  //   Logger.WriteData(GroupName,"CalValue",data1D,"Value from applying calibration to voltage",NumberRecords,1);
  // }

  // /* Sbus Voltage */
  // for (size_t l=0; l < Data.SbusVoltage.size(); l++) {
  //   string GroupName = "SbusVoltage_" + to_string(l);
  //   for (size_t k=0; k < NumberRecords; k++) {
  //     data1D[k] = SbusVoltage[l][k].Voltage_V;
  //   }
  //   Logger.WriteData(GroupName,"Voltage_V",data1D,"Measured voltage, V",NumberRecords,1);
  // }

  // /* Pwm Voltage */ 
  // for (size_t l=0; l < Data.PwmVoltage.size(); l++) {
  //   string GroupName = "PwmVoltage_" + to_string(l);
  //   for (size_t k=0; k < NumberRecords; k++) {
  //     data1D[k] = PwmVoltage[l][k].Voltage_V;
  //   }
  //   Logger.WriteData(GroupName,"Voltage_V",data1D,"Measured voltage, V",NumberRecords,1);
  // }

	return 0;
}
