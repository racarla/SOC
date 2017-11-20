/*
global-defs.hxx
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

#ifndef GLOBAL_DEFS_HXX_
#define GLOBAL_DEFS_HXX_

#include <stdint.h>
#include <vector>
#include <Eigen/Dense>

enum BfsMessage {
  kMode,
  kConfig,
  kEffectorAngleCmd,
  kEffectorDirectCmd,
  kData
};

enum BfsMode {
  kStandby,
  kRun
};

/* Config */
struct AircraftConfig {
  size_t NumberEffectors;
};

/* Data */
struct Mpu9250Data {
  Eigen::Matrix<float,3,1>Accel_mss;        // x,y,z accelerometers, m/s/s
  Eigen::Matrix<float,3,1>Gyro_rads;        // x,y,z gyros, rad/s
  Eigen::Matrix<float,3,1>Mag_uT;           // x,y,z magnetometers, uT
  float Temp_C;                             // Temperature, C  
};

struct Bme280Data {
  float Pressure_Pa;                        // Pressure, Pa
  float Temp_C;                             // Temperature, C
  float Humidity_RH;                        // Relative humidity
};

struct SbusRxData {
  bool Failsafe;                            // True when failsafe active
  uint16_t LostFrames;                      // Number of lost frames
  bool AutoEnabled;                         // True for autopilot mode
  bool ThrottleEnabled;                     // True for throttle enabled
  float RSSI;                               // RSSI value
  Eigen::Matrix<float,5,1> Inceptors;       // Aerodynamic inceptors, roll pitch yaw lift thrust, normalized to +/- 1
  Eigen::Matrix<float,5,1> AuxInputs;       // Auxiliary inputs, normalized to +/- 1
};

struct GpsData {
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

struct PressureData {
  float Pressure_Pa;                        // Pressure, Pa
  float Temp_C;                             // Temperature, C
};

struct PitotData {
  PressureData Static;                      // Static pressure data
  PressureData Diff;                        // Differential pressure data
};

struct AnalogData {
  float Voltage_V;                          // Measured voltage, V
  float CalValue;                           // Value from applying calibration to voltage
};

struct Voltage {
  float Voltage_V;                          // Measured voltage, V
};

struct FmuData {
  uint64_t Time_us;
  Voltage InputVoltage;
  Voltage RegulatedVoltage;
  Mpu9250Data Mpu9250;
  Bme280Data Bme280;
  std::vector<Mpu9250Data> Mpu9250Ext;
  std::vector<Bme280Data> Bme280Ext;
  std::vector<SbusRxData> SbusRx;
  std::vector<GpsData> Gps;
  std::vector<PitotData> Pitot;
  std::vector<PressureData> PressureTransducer;
  std::vector<AnalogData> Analog; 
  std::vector<Voltage> SbusVoltage;
  std::vector<Voltage> PwmVoltage;
};

struct NavigationData {
  double Time_s;                            // [sec], timestamp of NAV filter
  Eigen::Matrix<double,3,1>LLA;             // Latitude (rad), Longitude (rad), Altitude (m)
  Eigen::Matrix<double,3,1>NEDVelocity_ms;  // NED Velocity, m/s
  Eigen::Matrix<double,3,1>Euler_rad;       // Euler angles, rad
  Eigen::Matrix<double,4,1>Quaternion;      // Quaternion estimate
  Eigen::Matrix<double,3,1>AccelBias_mss;   // x,y,z accelerometer bias, m/s/s
  Eigen::Matrix<double,3,1>GyroBias_rads;   // x,y,z gyro bias, rad/s
  Eigen::Matrix<double,3,1>Pp;              // [rad], covariance estimate for position
  Eigen::Matrix<double,3,1>Pv;              // [rad], covariance estimate for velocity
  Eigen::Matrix<double,3,1>Pa;              // [rad], covariance estimate for angles
  Eigen::Matrix<double,3,1>Pab;             // [rad], covariance estimate for accelerometer bias
  Eigen::Matrix<double,3,1>Pgb;             // [rad], covariance estimate for rate gyro bias
};

#endif // GLOBAL_DEFS_HXX_
