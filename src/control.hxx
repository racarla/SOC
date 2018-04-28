/*
control.hxx
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

#ifndef CONTROL_HXX_
#define CONTROL_HXX_

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

class ControlFunctionClass {
  public:
    enum Mode {
      kReset,
      kInitialize,
      kStandby,
      kHold,
      kEngage
    };
    void Configure(const rapidjson::Value& Config,std::string RootPath,DefinitionTree *DefinitionTreePtr);
    bool Initialized();
    void Run(Mode mode);
};

class ControlEmptyClass: public ControlFunctionClass {
  public:
    enum Mode {
      kReset,
      kInitialize,
      kStandby,
      kHold,
      kEngage
    };
    void Configure(const rapidjson::Value& Config,std::string RootPath,DefinitionTree *DefinitionTreePtr);
    bool Initialized();
    void Run(Mode mode);
};

class ControlConstantClass: public ControlFunctionClass {
  public:
    enum Mode {
      kReset,
      kInitialize,
      kStandby,
      kHold,
      kEngage
    };
    void Configure(const rapidjson::Value& Config,std::string RootPath,DefinitionTree *DefinitionTreePtr);
    bool Initialized();
    void Run(Mode mode);
  private:
    struct Config {
      float Constant;
    };
    struct Data {
      uint8_t Mode;
      float Command;
    };
    Config config_;
    Data data_;
};

class ControlGainClass: public ControlFunctionClass {
  public:
    enum Mode {
      kReset,
      kInitialize,
      kStandby,
      kHold,
      kEngage
    };
    void Configure(const rapidjson::Value& Config,std::string RootPath,DefinitionTree *DefinitionTreePtr);
    bool Initialized();
    void Run(Mode mode);
  private:
    struct Config {
      float *Reference;
      float Gain;
    };
    Config config_;
};

class ControlPIDClass: public ControlFunctionClass {
  public:
    enum Mode {
      kReset,
      kInitialize,
      kStandby,
      kHold,
      kEngage
    };
    void Configure(const rapidjson::Value& Config,std::string RootPath,DefinitionTree *DefinitionTreePtr);
    bool Initialized();
    void Run(Mode mode);
};

class ControlPID2Class: public ControlFunctionClass {
  public:
    enum Mode {
      kReset,
      kInitialize,
      kStandby,
      kHold,
      kEngage
    };
    void Configure(const rapidjson::Value& Config,std::string RootPath,DefinitionTree *DefinitionTreePtr);
    bool Initialized();
    void Run(Mode mode);
};

class ControlStateSpaceClass: public ControlFunctionClass {
  public:
    enum Mode {
      kReset,
      kInitialize,
      kStandby,
      kHold,
      kEngage
    };
    void Configure(const rapidjson::Value& Config,std::string RootPath,DefinitionTree *DefinitionTreePtr);
    bool Initialized();
    void Run(Mode mode);
};

class ControlLaws {
  public:
    void Configure(const rapidjson::Value& Config, DefinitionTree *DefinitionTreePtr);
    bool Initialized();
    void SetEngagedController(std::string ControlGroupName);
    void SetArmedController(std::string ControlGroupName);
    size_t ActiveControlLevels();
    void Run(size_t ControlLevel);
  private:
    std::string RootPath_ = "/Control";
    bool InitializedLatch_ = false;
    std::string EngagedGroup_;
    std::string ArmedGroup_;
    std::vector<std::string> ResearchGroupKeys_;
    std::map<std::string,std::string> OutputKeys_;
    std::vector<std::vector<ControlFunctionClass>> BaselineControlGroup_;
    std::map<std::string,std::vector<std::vector<ControlFunctionClass>>> ResearchControlGroups_;
    std::vector<std::variant<uint64_t,uint32_t,uint16_t,uint8_t,int64_t,int32_t,int16_t,int8_t,float, double>> OutputData_;
    std::vector<std::variant<uint64_t*,uint32_t*,uint16_t*,uint8_t*,int64_t*,int32_t*,int16_t*,int8_t*,float*,double*>> BaselineDataPtr_;
    std::map<std::string,std::vector<std::variant<uint64_t*,uint32_t*,uint16_t*,uint8_t*,int64_t*,int32_t*,int16_t*,int8_t*,float*,double*>>> ResearchDataPtr_;
};

#endif
