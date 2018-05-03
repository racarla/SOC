/*
excitation.hxx
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

#ifndef EXCITATION_HXX_
#define EXCITATION_HXX_

#include "hardware-defs.hxx"
#include "definition-tree.hxx"
#include "utils.hxx"
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
#include <memory>

class ExcitationFunctionClass {
  public:
    enum Mode {
      kArm,
      kEngage
    };
    virtual void Configure(const rapidjson::Value& Config,std::string RootPath,DefinitionTree *DefinitionTreePtr);
    virtual void Run(Mode mode);
};

class Pulse: public ExcitationFunctionClass {
public:
  void Configure(const rapidjson::Value& Config,std::string RootPath,DefinitionTree *DefinitionTreePtr);
  void Run(Mode mode);
private:
  struct Config {
    float *Signal;
    float Amplitude;
    float StartTime_s;
    float Duration_s;
  };
  struct Data {
    uint8_t Mode;
    float Excitation;
  };
  Config config_;
  Data data_;
  elapsedMicros Time_us = 0;
  bool TimeLatch = false;
};

class Doublet: public ExcitationFunctionClass {
public:
  void Configure(const rapidjson::Value& Config,std::string RootPath,DefinitionTree *DefinitionTreePtr);
  void Run(Mode mode);
private:
  struct Config {
    float *Signal;
    float Amplitude;
    float StartTime_s;
    float Duration_s;
  };
  struct Data {
    uint8_t Mode;
    float Excitation;
  };
  Config config_;
  Data data_;
  elapsedMicros Time_us = 0;
  bool TimeLatch = false;
};

class Doublet121: public ExcitationFunctionClass {
public:
  void Configure(const rapidjson::Value& Config,std::string RootPath,DefinitionTree *DefinitionTreePtr);
  void Run(Mode mode);
private:
  struct Config {
    float *Signal;
    float Amplitude;
    float StartTime_s;
    float Duration_s;
  };
  struct Data {
    uint8_t Mode;
    float Excitation;
  };
  Config config_;
  Data data_;
  elapsedMicros Time_us = 0;
  bool TimeLatch = false;
};

class Doublet3211: public ExcitationFunctionClass {
public:
  void Configure(const rapidjson::Value& Config,std::string RootPath,DefinitionTree *DefinitionTreePtr);
  void Run(Mode mode);
private:
  struct Config {
    float *Signal;
    float Amplitude;
    float StartTime_s;
    float Duration_s;
  };
  struct Data {
    uint8_t Mode;
    float Excitation;
  };
  Config config_;
  Data data_;
  elapsedMicros Time_us = 0;
  bool TimeLatch = false;
};

class LinearChirp: public ExcitationFunctionClass {
public:
  void Configure(const rapidjson::Value& Config,std::string RootPath,DefinitionTree *DefinitionTreePtr);
  void Run(Mode mode);
};

class ExcitationSystem {
  public:
    void Configure(const rapidjson::Value& Config, DefinitionTree *DefinitionTreePtr);
    bool Configured();
    void SetEngagedExcitation(std::string ExcitationGroupName);
    void Run(std::string ControlLevel);
  private:
    std::string RootPath_ = "/Excitation";
    bool Configured_ = false;
    std::string EngagedGroup_ = "None";
    std::vector<std::string> ExcitationGroupKeys_;
    std::vector<std::string> ExcitationGroupLevels_;
    std::vector<std::vector<std::vector<std::shared_ptr<ExcitationFunctionClass>>>> ExcitationGroups_;
};

#endif
