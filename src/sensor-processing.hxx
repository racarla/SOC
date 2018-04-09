/*
sensor-processing.hxx
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

#ifndef SENSOR_PROCESSING_HXX_
#define SENSOR_PROCESSING_HXX_

#include "AirData.h"
#include "uNavINS.h"
#include "hardware-defs.hxx"
#include "definition-tree.hxx"
#include "rapidjson/document.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/writer.h"
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

class BaselineAirData {
  public:
    void UpdateConfiguration(const rapidjson::Value& BaselineAirDataConfig);
    void RegisterGlobalData(DefinitionTree *DefinitionTreePtr);
    void Run();
  private:
    struct Config {
      std::vector<std::string> StaticPressureSourceName;
      std::vector<std::string> DifferentialPressureSourceName;
      std::vector<std::string> TemperatureSourceName;
      std::vector<std::string> MslAltSourceName;
      std::vector<float*> StaticPressureSourcePtr;
      std::vector<float*> DifferentialPressureSourcePtr;
      std::vector<float> DifferentialPressureBiases;
      std::vector<float*> TemperatureSourcePtr;
      std::vector<float*> MslAltSourcePtr;
      float InitialTemperature_C;
      float InitialPressureAlt_m;
      float InitialMSLAlt_m;
    };
    struct Data {
      float StaticPressure_Pa;
      float DifferentialPressure_Pa;
      float Temperature_C;
      float Density_kgm3;
      float IAS_ms;
      float EAS_ms;
      float TAS_ms;
      float PressureAltitude_m;
      float AGL_m;
      float MSL_m;
      float DensityAltitude_m;
    };
    AirData *airdata_;
    Config config_;
    Data data_;
};

class SensorProcessing {
  public:
    void Begin();
    void UpdateConfiguration(const rapidjson::Value& SensorConfig);
    void RegisterGlobalData(DefinitionTree *DefinitionTreePtr);
    bool Initialized();
    void Run();
  private:
    struct Classes {
      std::vector<BaselineAirData> Baselineairdata;
    };
    Classes classes_;
};

#endif