/*
mission.hxx
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

#ifndef MISSION_HXX_
#define MISSION_HXX_

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

class MissionManager {
  public:
    struct TestPointDefinition {
      std::string SensorProcessing;
      std::string Control;
      std::string Excitation;
    };
    void Configure(const rapidjson::Value& Config, DefinitionTree *DefinitionTreePtr);
    bool Configured();
    void Run();
    std::string GetEnagagedSensorProcessing();
    std::string GetEnagagedController();
    std::string GetArmedController();
    std::string GetEnagagedExcitation();
  private:
    struct Configuration {
      struct {
        float *SourcePtr;
        float Threshold = 0.5;
        float Gain = 1.0;
      } EngageSwitch;
    };
    Configuration config_;
    std::string RootPath_ = "/Mission-Manager";
    bool Configured_ = false;
    bool InitializedLatch_ = false;
    size_t PersistenceCounter_ = 0;
    const size_t PersistenceThreshold_ = 5;
    size_t NumberOfTestPoints_;
    size_t TestPointIndex_ = 0;
    bool TestPointIndexLatch_ = false;
    std::string EngagedSensorProcessing_ = "Baseline";
    std::string EnagagedController_ = "Baseline";
    std::string ArmedController_;
    std::string EnagagedExcitation_ = "None";
    std::map<std::string,TestPointDefinition> TestPoints_;
};

#endif