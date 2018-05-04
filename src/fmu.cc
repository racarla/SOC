/*
fmu.cxx
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

#include "fmu.hxx"

/* Opens port to communicate with FMU. */
void FlightManagementUnit::Begin() {
  std::cout << "\t\tOpening UART port with FMU..." << std::flush;
  if ((FmuFileDesc_=open(Port_.c_str(),O_RDWR|O_NOCTTY|O_NONBLOCK))<0) {
    throw std::runtime_error(std::string("ERROR")+RootPath+std::string(": UART failed to open."));
  }
  struct termios Options;
  tcgetattr(FmuFileDesc_,&Options);
  Options.c_cflag = Baud_ | CS8 | CREAD | CLOCAL;
  Options.c_iflag = IGNPAR;
  Options.c_oflag = 0;
  Options.c_lflag = 0;
  Options.c_cc[VTIME] = 0;
  Options.c_cc[VMIN] = 0;
  tcflush(FmuFileDesc_,TCIFLUSH);
  tcsetattr(FmuFileDesc_,TCSANOW,&Options);
  fcntl(FmuFileDesc_,F_SETFL,O_NONBLOCK);
  std::cout <<  "done!" << std::endl;
}

/* Updates FMU configuration given a JSON value and registers data with global defs */
void FlightManagementUnit::Configure(const rapidjson::Value& Config, DefinitionTree *DefinitionTreePtr) {
  std::vector<uint8_t> Payload;
  // switch FMU to configuration mode
  Payload.push_back((uint8_t)Mode::kConfigMode);
  SendMessage(Message::kModeCommand,Payload);
  // clear the serial buffer
  while ((read(FmuFileDesc_,&RxByte_,sizeof(RxByte_)))>0) {}
  // parse and send sensor configuration messages
  if (Config.HasMember("Sensors")) {
    const rapidjson::Value& Sensors = Config["Sensors"];
    assert(Sensors.IsArray());
    for (size_t i=0; i < Sensors.Size(); i++) {
      const rapidjson::Value& Sensor = Sensors[i];
      if (Sensor.HasMember("Type")) {
        Payload.clear();
        rapidjson::StringBuffer StringBuff;
        rapidjson::Writer<rapidjson::StringBuffer> Writer(StringBuff);
        Sensor.Accept(Writer);
        std::string OutputString = StringBuff.GetString();
        std::string ConfigString = std::string("{\"Sensors\":[") + OutputString + std::string("]}");
        for (size_t j=0; j < ConfigString.size(); j++) {
          Payload.push_back((uint8_t)ConfigString[j]);
        }
        SendMessage(Message::kConfigMesg,Payload);
        // create list of sensor output names
        if (Sensor["Type"] == "Time") {
          SensorNames_.Time_us.push_back(RootPath + "/" + Sensor["Output"].GetString());
        }
        if (Sensor["Type"] == "InternalMpu9250") {
          SensorNames_.InternalMpu9250.push_back(RootPath + "/" + Sensor["Output"].GetString());
        }
        if (Sensor["Type"] == "InternalBme280") {
          SensorNames_.InternalBme280.push_back(RootPath + "/" + Sensor["Output"].GetString());
        }
        if (Sensor["Type"] == "InputVoltage") {
          SensorNames_.InputVoltage_V.push_back(RootPath + "/" + Sensor["Output"].GetString());
        }
        if (Sensor["Type"] == "RegulatedVoltage") {
          SensorNames_.RegulatedVoltage_V.push_back(RootPath + "/" + Sensor["Output"].GetString());
        }
        if (Sensor["Type"] == "PwmVoltage") {
          SensorNames_.PwmVoltage_V.push_back(RootPath + "/" + Sensor["Output"].GetString());
        }
        if (Sensor["Type"] == "SbusVoltage") {
          SensorNames_.SbusVoltage_V.push_back(RootPath + "/" + Sensor["Output"].GetString());
        }
        if (Sensor["Type"] == "Mpu9250") {
          SensorNames_.Mpu9250.push_back(RootPath + "/" + Sensor["Output"].GetString());
        }
        if (Sensor["Type"] == "Bme280") {
          SensorNames_.Bme280.push_back(RootPath + "/" + Sensor["Output"].GetString());
        }
        if (Sensor["Type"] == "uBlox") {
          SensorNames_.uBlox.push_back(RootPath + "/" + Sensor["Output"].GetString());
        }
        if (Sensor["Type"] == "Swift") {
          SensorNames_.Swift.push_back(RootPath + "/" + Sensor["Output"].GetString());
        }
        if (Sensor["Type"] == "Ams5915") {
          SensorNames_.Ams5915.push_back(RootPath + "/" + Sensor["Output"].GetString());
        }
        if (Sensor["Type"] == "Sbus") {
          SensorNames_.Sbus.push_back(RootPath + "/" + Sensor["Output"].GetString());
        }
        if (Sensor["Type"] == "Analog") {
          SensorNames_.Analog.push_back(RootPath + "/" + Sensor["Output"].GetString());
        }
        if (Sensor["Type"] == "Node") {
          const rapidjson::Value& Node = Sensor;
          if (Node.HasMember("Sensors")) {
            const rapidjson::Value& NodeSensors = Node["Sensors"];
            assert(NodeSensors.IsArray());
            for (size_t j=0; j < NodeSensors.Size(); j++) {
              const rapidjson::Value& NodeSensor = NodeSensors[j];
              if (NodeSensor.HasMember("Type")) {
                if (NodeSensor["Type"] == "PwmVoltage") {
                  SensorNames_.PwmVoltage_V.push_back(RootPath + "/" + NodeSensor["Output"].GetString());
                }
                if (NodeSensor["Type"] == "SbusVoltage") {
                  SensorNames_.SbusVoltage_V.push_back(RootPath + "/" + NodeSensor["Output"].GetString());
                }
                if (NodeSensor["Type"] == "Mpu9250") {
                  SensorNames_.Mpu9250.push_back(RootPath + "/" + NodeSensor["Output"].GetString());
                }
                if (NodeSensor["Type"] == "Bme280") {
                  SensorNames_.Bme280.push_back(RootPath + "/" + NodeSensor["Output"].GetString());
                }
                if (NodeSensor["Type"] == "uBlox") {
                  SensorNames_.uBlox.push_back(RootPath + "/" + NodeSensor["Output"].GetString());
                }
                if (NodeSensor["Type"] == "Swift") {
                  SensorNames_.Swift.push_back(RootPath + "/" + NodeSensor["Output"].GetString());
                }
                if (NodeSensor["Type"] == "Ams5915") {
                  SensorNames_.Ams5915.push_back(RootPath + "/" + NodeSensor["Output"].GetString());
                }
                if (NodeSensor["Type"] == "Sbus") {
                  SensorNames_.Sbus.push_back(RootPath + "/" + NodeSensor["Output"].GetString());
                }
                if (NodeSensor["Type"] == "Analog") {
                  SensorNames_.Analog.push_back(RootPath + "/" + NodeSensor["Output"].GetString());
                }
              } else {
                throw std::runtime_error(std::string("ERROR")+RootPath+std::string(": Node sensor type not specified."));
              }
            }
          }
        }
      } else {
        throw std::runtime_error(std::string("ERROR")+RootPath+std::string(": Sensor type not specified."));
      }
    }
  }
  // switch FMU to run mode
  Payload.clear();
  Payload.push_back((uint8_t)Mode::kRunMode);
  SendMessage(Message::kModeCommand,Payload);
  // get the updated configuration from the sensor meta data
  std::cout << "\t\tGetting FMU configuration..." << std::flush;

  size_t i=0;
  while(i < 100) {
    if (ReceiveSensorData()) {
      i++;
    }
  }
  std::cout <<  "done!" << std::endl;
  std::cout << "\t\tFMU configuration:" << std::endl;
  std::cout << "\t\t\tSensors:" << std::endl;
  std::cout << "\t\t\t\tTime:" << SensorData_.Time_us.size() << std::endl;
  std::cout << "\t\t\t\tInternal Mpu9250:" << SensorData_.InternalMpu9250.size() << std::endl;
  std::cout << "\t\t\t\tInternal Bme280:" << SensorData_.InternalBme280.size() << std::endl;
  std::cout << "\t\t\t\tInput voltage:" << SensorData_.InputVoltage_V.size() << std::endl;
  std::cout << "\t\t\t\tRegulated voltage:" << SensorData_.RegulatedVoltage_V.size() << std::endl;
  std::cout << "\t\t\t\tPwm voltage:" << SensorData_.PwmVoltage_V.size() << std::endl;
  std::cout << "\t\t\t\tSbus voltage:" << SensorData_.SbusVoltage_V.size() << std::endl;
  std::cout << "\t\t\t\tMpu9250:" << SensorData_.Mpu9250.size() << std::endl;
  std::cout << "\t\t\t\tBme280:" << SensorData_.Bme280.size() << std::endl;
  std::cout << "\t\t\t\tuBlox:" << SensorData_.uBlox.size() << std::endl;
  std::cout << "\t\t\t\tSwift:" << SensorData_.Swift.size() << std::endl;
  std::cout << "\t\t\t\tAms5915:" << SensorData_.Ams5915.size() << std::endl;
  std::cout << "\t\t\t\tSbus:" << SensorData_.Sbus.size() << std::endl;
  std::cout << "\t\t\t\tAnalog:" << SensorData_.Analog.size() << std::endl;
  // register sensor data with global definition tree
  std::cout << "\t\tRegistering FMU data with global definition tree..." << std::flush;
  for (size_t i=0; i < SensorData_.Time_us.size(); i++) {
    DefinitionTreePtr->InitMember(SensorNames_.Time_us[i],&SensorData_.Time_us[i],"Flight management unit time, us",true,false);
  }
  for (size_t i=0; i < SensorData_.InternalMpu9250.size(); i++) {
    DefinitionTreePtr->InitMember(SensorNames_.InternalMpu9250[i]+"/AccelX_mss",&SensorData_.InternalMpu9250[i].Accel_mss(0,0),"Flight management unit MPU-9250 X accelerometer, corrected for installation rotation, m/s/s",true,false);
    DefinitionTreePtr->InitMember(SensorNames_.InternalMpu9250[i]+"/AccelY_mss",&SensorData_.InternalMpu9250[i].Accel_mss(1,0),"Flight management unit MPU-9250 Y accelerometer, corrected for installation rotation, m/s/s",true,false);
    DefinitionTreePtr->InitMember(SensorNames_.InternalMpu9250[i]+"/AccelZ_mss",&SensorData_.InternalMpu9250[i].Accel_mss(2,0),"Flight management unit MPU-9250 Z accelerometer, corrected for installation rotation, m/s/s",true,false);
    DefinitionTreePtr->InitMember(SensorNames_.InternalMpu9250[i]+"/GyroX_rads",&SensorData_.InternalMpu9250[i].Gyro_rads(0,0),"Flight management unit MPU-9250 X gyro, corrected for installation rotation, rad/s",true,false);
    DefinitionTreePtr->InitMember(SensorNames_.InternalMpu9250[i]+"/GyroY_rads",&SensorData_.InternalMpu9250[i].Gyro_rads(1,0),"Flight management unit MPU-9250 Y gyro, corrected for installation rotation, rad/s",true,false);
    DefinitionTreePtr->InitMember(SensorNames_.InternalMpu9250[i]+"/GyroZ_rads",&SensorData_.InternalMpu9250[i].Gyro_rads(2,0),"Flight management unit MPU-9250 Z gyro, corrected for installation rotation, rad/s",true,false);
    DefinitionTreePtr->InitMember(SensorNames_.InternalMpu9250[i]+"/MagX_uT",&SensorData_.InternalMpu9250[i].Mag_uT(0,0),"Flight management unit MPU-9250 X magnetometer, corrected for installation rotation, uT",true,false);
    DefinitionTreePtr->InitMember(SensorNames_.InternalMpu9250[i]+"/MagY_uT",&SensorData_.InternalMpu9250[i].Mag_uT(1,0),"Flight management unit MPU-9250 Y magnetometer, corrected for installation rotation, uT",true,false);
    DefinitionTreePtr->InitMember(SensorNames_.InternalMpu9250[i]+"/MagZ_uT",&SensorData_.InternalMpu9250[i].Mag_uT(2,0),"Flight management unit MPU-9250 Z magnetometer, corrected for installation rotation, uT",true,false);
    DefinitionTreePtr->InitMember(SensorNames_.InternalMpu9250[i]+"/Temperature_C",&SensorData_.InternalMpu9250[i].Temperature_C,"Flight management unit MPU-9250 temperature, C",true,false);
  }
  for (size_t i=0; i < SensorData_.InternalBme280.size(); i++) {
    DefinitionTreePtr->InitMember(SensorNames_.InternalBme280[i]+"/Pressure_Pa",&SensorData_.InternalBme280[i].Pressure_Pa,"Flight management unit BME-280 static pressure, Pa",true,false);
    DefinitionTreePtr->InitMember(SensorNames_.InternalBme280[i]+"/Temperature_C",&SensorData_.InternalBme280[i].Temperature_C,"Flight management unit BME-280 temperature, C",true,false);
    DefinitionTreePtr->InitMember(SensorNames_.InternalBme280[i]+"/Humidity_RH",&SensorData_.InternalBme280[i].Humidity_RH,"Flight management unit BME-280 percent relative humidity",true,false);
  }
  for (size_t i=0; i < SensorData_.InputVoltage_V.size(); i++) {
    DefinitionTreePtr->InitMember(SensorNames_.InputVoltage_V[i],&SensorData_.InputVoltage_V[i],"Flight management unit input voltage, V",true,false);
  }
  for (size_t i=0; i < SensorData_.RegulatedVoltage_V.size(); i++) {
    DefinitionTreePtr->InitMember(SensorNames_.RegulatedVoltage_V[i],&SensorData_.RegulatedVoltage_V[i],"Flight management unit regulated voltage, V",true,false);
  }
  for (size_t i=0; i < SensorData_.PwmVoltage_V.size(); i++) {
    DefinitionTreePtr->InitMember(SensorNames_.PwmVoltage_V[i],&SensorData_.PwmVoltage_V[i],"Flight management unit PWM servo voltage, V",true,false);
  }
  for (size_t i=0; i < SensorData_.SbusVoltage_V.size(); i++) {
    DefinitionTreePtr->InitMember(SensorNames_.SbusVoltage_V[i],&SensorData_.SbusVoltage_V[i],"Flight management unit SBUS servo voltage, V",true,false);
  }
  for (size_t i=0; i < SensorData_.Mpu9250.size(); i++) {
    DefinitionTreePtr->InitMember(SensorNames_.Mpu9250[i] + "/Status",&SensorData_.Mpu9250[i].status,"MPU-9250_" + std::to_string(i) + " read status, positive if a good sensor read",true,false);
    DefinitionTreePtr->InitMember(SensorNames_.Mpu9250[i] + "/AccelX_mss",&SensorData_.Mpu9250[i].Accel_mss(0,0),"MPU-9250_" + std::to_string(i) + " X accelerometer, corrected for installation rotation, m/s/s",true,false);
    DefinitionTreePtr->InitMember(SensorNames_.Mpu9250[i] + "/AccelY_mss",&SensorData_.Mpu9250[i].Accel_mss(1,0),"MPU-9250_" + std::to_string(i) + " Y accelerometer, corrected for installation rotation, m/s/s",true,false);
    DefinitionTreePtr->InitMember(SensorNames_.Mpu9250[i] + "/AccelZ_mss",&SensorData_.Mpu9250[i].Accel_mss(2,0),"MPU-9250_" + std::to_string(i) + " Z accelerometer, corrected for installation rotation, m/s/s",true,false);
    DefinitionTreePtr->InitMember(SensorNames_.Mpu9250[i] + "/GyroX_rads",&SensorData_.Mpu9250[i].Gyro_rads(0,0),"MPU-9250_" + std::to_string(i) + " X gyro, corrected for installation rotation, rad/s",true,false);
    DefinitionTreePtr->InitMember(SensorNames_.Mpu9250[i] + "/GyroY_rads",&SensorData_.Mpu9250[i].Gyro_rads(1,0),"MPU-9250_" + std::to_string(i) + " Y gyro, corrected for installation rotation, rad/s",true,false);
    DefinitionTreePtr->InitMember(SensorNames_.Mpu9250[i] + "/GyroZ_rads",&SensorData_.Mpu9250[i].Gyro_rads(2,0),"MPU-9250_" + std::to_string(i) + " Z gyro, corrected for installation rotation, rad/s",true,false);
    DefinitionTreePtr->InitMember(SensorNames_.Mpu9250[i] + "/MagX_uT",&SensorData_.Mpu9250[i].Mag_uT(0,0),"MPU-9250_" + std::to_string(i) + " X magnetometer, corrected for installation rotation, uT",true,false);
    DefinitionTreePtr->InitMember(SensorNames_.Mpu9250[i] + "/MagY_uT",&SensorData_.Mpu9250[i].Mag_uT(1,0),"MPU-9250_" + std::to_string(i) + " Y magnetometer, corrected for installation rotation, uT",true,false);
    DefinitionTreePtr->InitMember(SensorNames_.Mpu9250[i] + "/MagZ_uT",&SensorData_.Mpu9250[i].Mag_uT(2,0),"MPU-9250_" + std::to_string(i) + " Z magnetometer, corrected for installation rotation, uT",true,false);
    DefinitionTreePtr->InitMember(SensorNames_.Mpu9250[i] + "/Temperature_C",&SensorData_.Mpu9250[i].Temperature_C,"MPU-9250_" + std::to_string(i) + " temperature, C",true,false);
  }
  for (size_t i=0; i < SensorData_.Bme280.size(); i++) {
    DefinitionTreePtr->InitMember(SensorNames_.Bme280[i] + "/Status",&SensorData_.Bme280[i].status,"BME-280_" + std::to_string(i) + " read status, positive if a good sensor read",true,false);
    DefinitionTreePtr->InitMember(SensorNames_.Bme280[i] + "/Pressure_Pa",&SensorData_.Bme280[i].Pressure_Pa,"BME-280_" + std::to_string(i) + " static pressure, Pa",true,false);
    DefinitionTreePtr->InitMember(SensorNames_.Bme280[i] + "/Temperature_C",&SensorData_.Bme280[i].Temperature_C,"BME-280_" + std::to_string(i) + " temperature, C",true,false);
    DefinitionTreePtr->InitMember(SensorNames_.Bme280[i] + "/Humidity_RH",&SensorData_.Bme280[i].Humidity_RH,"BME-280_" + std::to_string(i) + " percent relative humidity",true,false);
  }
  for (size_t i=0; i < SensorData_.uBlox.size(); i++) {
    DefinitionTreePtr->InitMember(SensorNames_.uBlox[i] + "/Fix",(uint8_t*)&SensorData_.uBlox[i].Fix,"uBlox_" + std::to_string(i) + " fix status, true for 3D fix only",true,false);
    DefinitionTreePtr->InitMember(SensorNames_.uBlox[i] + "/NumberSatellites",&SensorData_.uBlox[i].NumberSatellites,"uBlox_" + std::to_string(i) + " number of satellites used in solution",true,false);
    DefinitionTreePtr->InitMember(SensorNames_.uBlox[i] + "/TOW",&SensorData_.uBlox[i].TOW,"uBlox_" + std::to_string(i) + " GPS time of the navigation epoch",true,false);
    DefinitionTreePtr->InitMember(SensorNames_.uBlox[i] + "/Year",&SensorData_.uBlox[i].Year,"uBlox_" + std::to_string(i) + " UTC year",true,false);
    DefinitionTreePtr->InitMember(SensorNames_.uBlox[i] + "/Month",&SensorData_.uBlox[i].Month,"uBlox_" + std::to_string(i) + " UTC month",true,false);
    DefinitionTreePtr->InitMember(SensorNames_.uBlox[i] + "/Day",&SensorData_.uBlox[i].Day,"uBlox_" + std::to_string(i) + " UTC day",true,false);
    DefinitionTreePtr->InitMember(SensorNames_.uBlox[i] + "/Hour",&SensorData_.uBlox[i].Hour,"uBlox_" + std::to_string(i) + " UTC hour",true,false);
    DefinitionTreePtr->InitMember(SensorNames_.uBlox[i] + "/Minute",&SensorData_.uBlox[i].Min,"uBlox_" + std::to_string(i) + " UTC minute",true,false);
    DefinitionTreePtr->InitMember(SensorNames_.uBlox[i] + "/Second",&SensorData_.uBlox[i].Sec,"uBlox_" + std::to_string(i) + " UTC second",true,false);
    DefinitionTreePtr->InitMember(SensorNames_.uBlox[i] + "/Latitude_rad",&SensorData_.uBlox[i].LLA(0,0),"uBlox_" + std::to_string(i) + " latitude, rad",true,false);
    DefinitionTreePtr->InitMember(SensorNames_.uBlox[i] + "/Longitude_rad",&SensorData_.uBlox[i].LLA(1,0),"uBlox_" + std::to_string(i) + " longitude, rad",true,false);
    DefinitionTreePtr->InitMember(SensorNames_.uBlox[i] + "/Altitude_m",&SensorData_.uBlox[i].LLA(2,0),"uBlox_" + std::to_string(i) + " altitude above mean sea level, m",true,false);
    DefinitionTreePtr->InitMember(SensorNames_.uBlox[i] + "/NorthVelocity_ms",&SensorData_.uBlox[i].NEDVelocity_ms(0,0),"uBlox_" + std::to_string(i) + " north velocity, m/s",true,false);
    DefinitionTreePtr->InitMember(SensorNames_.uBlox[i] + "/EastVelocity_ms",&SensorData_.uBlox[i].NEDVelocity_ms(1,0),"uBlox_" + std::to_string(i) + " east velocity, m/s",true,false);
    DefinitionTreePtr->InitMember(SensorNames_.uBlox[i] + "/DownVelocity_ms",&SensorData_.uBlox[i].NEDVelocity_ms(2,0),"uBlox_" + std::to_string(i) + " down velocity, m/s",true,false);
    DefinitionTreePtr->InitMember(SensorNames_.uBlox[i] + "/HorizontalAccuracy_m",&SensorData_.uBlox[i].Accuracy(0,0),"uBlox_" + std::to_string(i) + " horizontal accuracy estimate, m",true,false);
    DefinitionTreePtr->InitMember(SensorNames_.uBlox[i] + "/VerticalAccuracy_m",&SensorData_.uBlox[i].Accuracy(1,0),"uBlox_" + std::to_string(i) + " vertical accuracy estimate, m",true,false);
    DefinitionTreePtr->InitMember(SensorNames_.uBlox[i] + "/VelocityAccuracy_ms",&SensorData_.uBlox[i].Accuracy(2,0),"uBlox_" + std::to_string(i) + " velocity accuracy estimate, m/s",true,false);
    DefinitionTreePtr->InitMember(SensorNames_.uBlox[i] + "/pDOP",&SensorData_.uBlox[i].pDOP,"uBlox_" + std::to_string(i) + " position dilution of precision",true,false);
  }
  for (size_t i=0; i < SensorData_.Swift.size(); i++) {
    DefinitionTreePtr->InitMember(SensorNames_.Swift[i] + "/Static/Status",&SensorData_.Swift[i].Static.status,"Swift_" + std::to_string(i) + " static pressure read status, positive if a good sensor read",true,false);
    DefinitionTreePtr->InitMember(SensorNames_.Swift[i] + "/Static/Pressure_Pa",&SensorData_.Swift[i].Static.Pressure_Pa,"Swift_" + std::to_string(i) + " static pressure, Pa",true,false);
    DefinitionTreePtr->InitMember(SensorNames_.Swift[i] + "/Static/Temperature_C",&SensorData_.Swift[i].Static.Temperature_C,"Swift_" + std::to_string(i) + " static pressure transducer temperature, C",true,false);
    DefinitionTreePtr->InitMember(SensorNames_.Swift[i] + "/Differential/Status",&SensorData_.Swift[i].Differential.status,"Swift_" + std::to_string(i) + " differential pressure read status, positive if a good sensor read",true,false);
    DefinitionTreePtr->InitMember(SensorNames_.Swift[i] + "/Differential/Pressure_Pa",&SensorData_.Swift[i].Differential.Pressure_Pa,"Swift_" + std::to_string(i) + " differential pressure, Pa",true,false);
    DefinitionTreePtr->InitMember(SensorNames_.Swift[i] + "/Differential/Temperature_C",&SensorData_.Swift[i].Differential.Temperature_C,"Swift_" + std::to_string(i) + " differential pressure transducer temperature, C",true,false);
  }
  for (size_t i=0; i < SensorData_.Ams5915.size(); i++) {
    DefinitionTreePtr->InitMember(SensorNames_.Ams5915[i] + "/Status",&SensorData_.Ams5915[i].status,"AMS-5915_" + std::to_string(i) + " read status, positive if a good sensor read",true,false);
    DefinitionTreePtr->InitMember(SensorNames_.Ams5915[i] + "/Pressure_Pa",&SensorData_.Ams5915[i].Pressure_Pa,"AMS-5915_" + std::to_string(i) + " pressure, Pa",true,false);
    DefinitionTreePtr->InitMember(SensorNames_.Ams5915[i] + "/Temperature_C",&SensorData_.Ams5915[i].Temperature_C,"AMS-5915_" + std::to_string(i) + " pressure transducer temperature, C",true,false);
  }
  for (size_t i=0; i < SensorData_.Sbus.size(); i++) {
    DefinitionTreePtr->InitMember(SensorNames_.Sbus[i] + "/FailSafe",(uint8_t*)&SensorData_.Sbus[i].FailSafe,"SBUS_" + std::to_string(i) + " fail safe status",true,false);
    DefinitionTreePtr->InitMember(SensorNames_.Sbus[i] + "/LostFrames",&SensorData_.Sbus[i].LostFrames,"SBUS_" + std::to_string(i) + " number of lost frames",true,false);
    for (size_t j=0; j < 16; j++) {
      DefinitionTreePtr->InitMember(SensorNames_.Sbus[i] + "/Channels/" + std::to_string(j),&SensorData_.Sbus[i].Channels[j],"SBUS_" + std::to_string(i) + " channel" + std::to_string(j) + " normalized value",true,false);
    }
  }
  for (size_t i=0; i < SensorData_.Analog.size(); i++) {
    DefinitionTreePtr->InitMember(SensorNames_.Analog[i] + "/Voltage_V",&SensorData_.Analog[i].Voltage_V,"Analog_" + std::to_string(i) + " measured voltage, V",true,false);
    DefinitionTreePtr->InitMember(SensorNames_.Analog[i] + "/CalibratedValue",&SensorData_.Analog[i].CalibratedValue,"Analog_" + std::to_string(i) + " calibrated value",true,false);
  }
  std::cout <<  "done!" << std::endl;
}

/* Receive sensor data from FMU */
bool FlightManagementUnit::ReceiveSensorData() {
  Message message;
  std::vector<uint8_t> Payload;
  size_t PayloadLocation = 0;
  if (ReceiveMessage(&message,&Payload)) {
    if (message == kSensorData) {
      // meta data
      uint8_t AcquireInternalData,NumberPwmVoltageSensor,NumberSbusVoltageSensor,NumberMpu9250Sensor,NumberBme280Sensor,NumberuBloxSensor,NumberSwiftSensor,NumberAms5915Sensor,NumberSbusSensor,NumberAnalogSensor;
      memcpy(&AcquireInternalData,Payload.data()+PayloadLocation,sizeof(AcquireInternalData));
      PayloadLocation += sizeof(AcquireInternalData);
      memcpy(&NumberPwmVoltageSensor,Payload.data()+PayloadLocation,sizeof(NumberPwmVoltageSensor));
      PayloadLocation += sizeof(NumberPwmVoltageSensor);
      memcpy(&NumberSbusVoltageSensor,Payload.data()+PayloadLocation,sizeof(NumberSbusVoltageSensor));
      PayloadLocation += sizeof(NumberSbusVoltageSensor);
      memcpy(&NumberMpu9250Sensor,Payload.data()+PayloadLocation,sizeof(NumberMpu9250Sensor));
      PayloadLocation += sizeof(NumberMpu9250Sensor);
      memcpy(&NumberBme280Sensor,Payload.data()+PayloadLocation,sizeof(NumberBme280Sensor));
      PayloadLocation += sizeof(NumberBme280Sensor);
      memcpy(&NumberuBloxSensor,Payload.data()+PayloadLocation,sizeof(NumberuBloxSensor));
      PayloadLocation += sizeof(NumberuBloxSensor);
      memcpy(&NumberSwiftSensor,Payload.data()+PayloadLocation,sizeof(NumberSwiftSensor));
      PayloadLocation += sizeof(NumberSwiftSensor);
      memcpy(&NumberAms5915Sensor,Payload.data()+PayloadLocation,sizeof(NumberAms5915Sensor));
      PayloadLocation += sizeof(NumberAms5915Sensor);
      memcpy(&NumberSbusSensor,Payload.data()+PayloadLocation,sizeof(NumberSbusSensor));
      PayloadLocation += sizeof(NumberSbusSensor);
      memcpy(&NumberAnalogSensor,Payload.data()+PayloadLocation,sizeof(NumberAnalogSensor));
      PayloadLocation += sizeof(NumberAnalogSensor);
      // resize data buffers
      if (AcquireInternalData & 0x01) {
        SensorData_.Time_us.resize(1);
      }
      if (AcquireInternalData & 0x02) {
        SensorData_.InternalMpu9250.resize(1);
      }
      if (AcquireInternalData & 0x04) {
        SensorData_.InternalBme280.resize(1);
      }
      if (AcquireInternalData & 0x08) {
        SensorData_.InputVoltage_V.resize(1);
      }
      if (AcquireInternalData & 0x10) {
        SensorData_.RegulatedVoltage_V.resize(1);
      }
      SensorData_.PwmVoltage_V.resize(NumberPwmVoltageSensor);
      SensorData_.SbusVoltage_V.resize(NumberSbusVoltageSensor);
      SensorData_.Mpu9250.resize(NumberMpu9250Sensor);
      SensorData_.Bme280.resize(NumberBme280Sensor);
      SensorData_.uBlox.resize(NumberuBloxSensor);
      SensorData_.Swift.resize(NumberSwiftSensor);
      SensorData_.Ams5915.resize(NumberAms5915Sensor);
      SensorData_.Sbus.resize(NumberSbusSensor);
      SensorData_.Analog.resize(NumberAnalogSensor);
      // sensor data
      memcpy(SensorData_.Time_us.data(),Payload.data()+PayloadLocation,SensorData_.Time_us.size()*sizeof(SensorData_.Time_us[0]));
      PayloadLocation += SensorData_.Time_us.size()*sizeof(SensorData_.Time_us[0]);
      memcpy(SensorData_.InternalMpu9250.data(),Payload.data()+PayloadLocation,SensorData_.InternalMpu9250.size()*sizeof(InternalMpu9250SensorData));
      PayloadLocation += SensorData_.InternalMpu9250.size()*sizeof(InternalMpu9250SensorData);
      memcpy(SensorData_.InternalBme280.data(),Payload.data()+PayloadLocation,SensorData_.InternalBme280.size()*sizeof(InternalBme280SensorData));
      PayloadLocation += SensorData_.InternalBme280.size()*sizeof(InternalBme280SensorData);
      memcpy(SensorData_.InputVoltage_V.data(),Payload.data()+PayloadLocation,SensorData_.InputVoltage_V.size()*sizeof(SensorData_.InputVoltage_V[0]));
      PayloadLocation += SensorData_.InputVoltage_V.size()*sizeof(SensorData_.InputVoltage_V[0]);
      memcpy(SensorData_.RegulatedVoltage_V.data(),Payload.data()+PayloadLocation,SensorData_.RegulatedVoltage_V.size()*sizeof(SensorData_.RegulatedVoltage_V[0]));
      PayloadLocation += SensorData_.RegulatedVoltage_V.size()*sizeof(SensorData_.RegulatedVoltage_V[0]);
      memcpy(SensorData_.PwmVoltage_V.data(),Payload.data()+PayloadLocation,SensorData_.PwmVoltage_V.size()*sizeof(SensorData_.PwmVoltage_V[0]));
      PayloadLocation += SensorData_.PwmVoltage_V.size()*sizeof(SensorData_.PwmVoltage_V[0]);
      memcpy(SensorData_.SbusVoltage_V.data(),Payload.data()+PayloadLocation,SensorData_.SbusVoltage_V.size()*sizeof(SensorData_.SbusVoltage_V[0]));
      PayloadLocation += SensorData_.SbusVoltage_V.size()*sizeof(SensorData_.SbusVoltage_V[0]);
      memcpy(SensorData_.Mpu9250.data(),Payload.data()+PayloadLocation,SensorData_.Mpu9250.size()*sizeof(Mpu9250SensorData));
      PayloadLocation += SensorData_.Mpu9250.size()*sizeof(Mpu9250SensorData);
      memcpy(SensorData_.Bme280.data(),Payload.data()+PayloadLocation,SensorData_.Bme280.size()*sizeof(Bme280SensorData));
      PayloadLocation += SensorData_.Bme280.size()*sizeof(Bme280SensorData);
      memcpy(SensorData_.uBlox.data(),Payload.data()+PayloadLocation,SensorData_.uBlox.size()*sizeof(uBloxSensorData));
      PayloadLocation += SensorData_.uBlox.size()*sizeof(uBloxSensorData);
      memcpy(SensorData_.Swift.data(),Payload.data()+PayloadLocation,SensorData_.Swift.size()*sizeof(SwiftSensorData));
      PayloadLocation += SensorData_.Swift.size()*sizeof(SwiftSensorData);
      memcpy(SensorData_.Ams5915.data(),Payload.data()+PayloadLocation,SensorData_.Ams5915.size()*sizeof(Ams5915SensorData));
      PayloadLocation += SensorData_.Ams5915.size()*sizeof(Ams5915SensorData);
      memcpy(SensorData_.Sbus.data(),Payload.data()+PayloadLocation,SensorData_.Sbus.size()*sizeof(SbusSensorData));
      PayloadLocation += SensorData_.Sbus.size()*sizeof(SbusSensorData);
      memcpy(SensorData_.Analog.data(),Payload.data()+PayloadLocation,SensorData_.Analog.size()*sizeof(AnalogSensorData));
      PayloadLocation += SensorData_.Analog.size()*sizeof(AnalogSensorData);
      return true;
    } else {
      return false;
    }
  } else {
    return false;
  }
}

/* Send a BFS Bus message. */
void FlightManagementUnit::SendMessage(Message message,std::vector<uint8_t> &Payload) {
  // check that the payload length is within the maximum buffer size
  if (Payload.size() < (kUartBufferMaxSize-headerLength_-checksumLength_)) {
    // header
    Buffer_[0] = header_[0];
    Buffer_[1] = header_[1];
    // message ID
    Buffer_[2] = (uint8_t)message;
    // payload length
    Buffer_[3] = Payload.size() & 0xff;
    Buffer_[4] = Payload.size() >> 8;
    // payload
    std::memcpy(Buffer_+headerLength_,Payload.data(),Payload.size());
    // checksum
    CalcChecksum((size_t)(Payload.size()+headerLength_),Buffer_,Checksum_);
    Buffer_[Payload.size()+headerLength_] = Checksum_[0];
    Buffer_[Payload.size()+headerLength_+1] = Checksum_[1];
    // transmit
    WritePort(Buffer_,Payload.size()+headerLength_+checksumLength_);
  }
}

/* Receive a BFS Bus message. */
bool FlightManagementUnit::ReceiveMessage(Message *message,std::vector<uint8_t> *Payload) {
  int count;
  while ((count=read(FmuFileDesc_,&RxByte_,sizeof(RxByte_)))>0) {
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
        *message = (Message) Buffer_[2];
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
  }
  return false;
}

/* Writes data to serial port. */
void FlightManagementUnit::WritePort(uint8_t* Buffer,size_t BufferSize) {
  int count;
  if ((count=write(FmuFileDesc_,Buffer,BufferSize))<0) {
    do {
      count=write(FmuFileDesc_,Buffer,BufferSize);
      usleep(10);
    } while (count < 0);
  }
}

/* Computes a two byte checksum. */
void FlightManagementUnit::CalcChecksum(size_t ArraySize, uint8_t *ByteArray, uint8_t *Checksum) {
  Checksum[0] = 0;
  Checksum[1] = 0;
  for (size_t i = 0; i < ArraySize; i++) {
    Checksum[0] += ByteArray[i];
    Checksum[1] += Checksum[0];
  }
}
