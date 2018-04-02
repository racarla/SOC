/*
fmu.hxx
Brian R Taylor
brian.taylor@bolderflight.com

Copyright (c) 2018 Bolder Flight Systems
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

#ifndef FMU_HXX_
#define FMU_HXX_

#include "hardware-defs.hxx"
#include "definition-tree.hxx"
#include "../includes/rapidjson/document.h"
#include "../includes/rapidjson/stringbuffer.h"
#include "../includes/rapidjson/writer.h"
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <stdint.h>
#include <iostream>
#include <exception>
#include <stdexcept>
#include <vector>
#include <cstring>
#include <Eigen/Dense>

class FlightManagementUnit {
  public:
    enum Message {
      ModeCommand,
      ConfigMesg,
      SensorData,
      EffectorCommand
    };
    enum Mode {
      ConfigMode,
      RunMode
    };
    void Begin();
    void RegisterGlobalData(DefinitionTree *DefinitionTreePtr);
    bool ReceiveSensorData();
    void UpdateConfiguration(const rapidjson::Value& SensorConfig);
  private:
    struct Mpu9250SensorData {
      Eigen::Matrix<float,3,1>Accel_mss;        // x,y,z accelerometers, m/s/s
      Eigen::Matrix<float,3,1>Gyro_rads;        // x,y,z gyros, rad/s
      Eigen::Matrix<float,3,1>Mag_uT;           // x,y,z magnetometers, uT
      float Temperature_C;                      // Temperature, C
    };
    struct Bme280SensorData {
      float Pressure_Pa;                        // Pressure, Pa
      float Temperature_C;                      // Temperature, C
      float Humidity_RH;                        // Relative humidity
    };
    struct VoltageSensorsData {
      float InputVoltage_V;
      float RegulatedVoltage_V;
      float PwmServoVoltage_V;
      float SbusServoVoltage_V;
    };
    struct uBloxSensorData {
      bool Fix;                                 // True for 3D fix only
      uint8_t NumberSatellites;                 // Number of satellites used in solution
      uint32_t TOW;                             // GPS time of the navigation epoch
      uint16_t Year;                            // UTC year
      uint8_t Month;                            // UTC month
      uint8_t Day;                              // UTC day
      uint8_t Hour;                             // UTC hour
      uint8_t Min;                              // UTC minute
      uint8_t Sec;                              // UTC second
      Eigen::Matrix<double,3,1>LLA;             // Latitude (rad), Longitude (rad), Altitude (m)
      Eigen::Matrix<double,3,1>NEDVelocity_ms;  // NED Velocity, m/s
      Eigen::Matrix<double,3,1>Accuracy;        // Horizontal (m), vertical (m), and speed (m/s) accuracy estimates
      double pDOP;                              // Position DOP
    };
    struct Ams5915SensorData {
      float Pressure_Pa;                        // Pressure, Pa
      float Temperature_C;                      // Temperature, C
    };
    struct SwiftSensorData {
      Ams5915SensorData Static;
      Ams5915SensorData Differential;
    };
    struct SbusSensorData {
      float Channels[16];
      bool FailSafe;
      uint16_t LostFrames;
    };
    struct AnalogSensorData {
      float Voltage_V;
      float CalibratedValue;
    };
    struct SensorData {
      uint64_t Time_us;
      Mpu9250SensorData InternalMpu9250;
      Bme280SensorData InternalBme280;
      VoltageSensorsData InternalVoltage;
      std::vector<Mpu9250SensorData> Mpu9250;
      std::vector<Bme280SensorData> Bme280;
      std::vector<uBloxSensorData> uBlox;
      std::vector<SwiftSensorData> Swift;
      std::vector<Ams5915SensorData> Ams5915;
      std::vector<SbusSensorData> Sbus;
      std::vector<AnalogSensorData> Analog;
    };
    const std::string Port_ = FmuPort;
    const speed_t Baud_ = FmuBaud;
    int FmuFileDesc_;
    uint8_t Buffer_[kUartBufferMaxSize];
    const uint8_t header_[2] = {0x42,0x46};
    const uint8_t headerLength_ = 5;
    const uint8_t checksumLength_ = 2;
    uint8_t RxByte_;
    uint16_t ParserState_ = 0;
    uint8_t LengthBuffer_[2];
    uint16_t Length_ = 0;
    uint8_t Checksum_[2];
    struct SensorData SensorData_;
    size_t SerializedDataMetadataSize = 7;
    void SendMessage(Message message,std::vector<uint8_t> &Payload);
    bool ReceiveMessage(Message *message,std::vector<uint8_t> *Payload);
    void WritePort(uint8_t* Buffer,size_t BufferSize);
    void CalcChecksum(size_t ArraySize, uint8_t *ByteArray, uint8_t *Checksum);
};

#endif