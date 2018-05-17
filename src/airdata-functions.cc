/*
airdata-functions.cc
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

#include "airdata-functions.hxx"

void IndicatedAirspeed::Configure(const rapidjson::Value& Config,std::string RootPath,DefinitionTree *DefinitionTreePtr) {
  // get output name
  std::string OutputName;
  if (Config.HasMember("Output")) {
    OutputName = RootPath + "/" + Config["Output"].GetString();
  } else {
    throw std::runtime_error(std::string("ERROR")+RootPath+std::string(": Output not specified in configuration."));
  }
  // get pressure sources
  if (Config.HasMember("Differential-Pressure")) {
    const rapidjson::Value& PressureSources = Config["Differential-Pressure"];
    for (auto &PressureSource : PressureSources.GetArray()) {
      DifferentialPressureKeys_.push_back(PressureSource.GetString());
      if (DefinitionTreePtr->GetValuePtr<float*>(DifferentialPressureKeys_.back())) {
        config_.DifferentialPressure.push_back(DefinitionTreePtr->GetValuePtr<float*>(DifferentialPressureKeys_.back()));
      } else {
        throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Differential pressure source ")+DifferentialPressureKeys_.back()+std::string(" not found in global data."));
      }
    }
  } else {
    throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Differential pressure sources not specified in configuration."));
  }
  // resize bias vector
  data_.DifferentialPressureBias.resize(config_.DifferentialPressure.size());
  // get initialization time
  if (Config.HasMember("Initialization-Time")) {
    config_.InitTime = Config["Initialization-Time"].GetFloat();
  } else {
    throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Initialization time not specified in configuration."));
  }
  // pointer to log run mode data
  ModeKey_ = OutputName+"/Mode";
  DefinitionTreePtr->InitMember(ModeKey_,&data_.Mode,"Run mode",true,false);
  // pointer to log ias data
  OutputKey_ = OutputName+"/"+Config["Output"].GetString();
  DefinitionTreePtr->InitMember(OutputKey_,&data_.Ias_ms,"Indicated airspeed, m/s",true,false);
}

void IndicatedAirspeed::Initialize() {
  // grab the starting time
  if (!TimeLatch_) {
    T0_us_ = micros();
    TimeLatch_ = true;
  }
  // compute the elapsed time
  float ElapsedTime = ((float)(micros()-T0_us_))/1e6;
  // if less than init time, compute bias
  if (ElapsedTime < config_.InitTime) {
    for (size_t i=0; i < config_.DifferentialPressure.size(); i++) {
      data_.DifferentialPressureBias[i] = data_.DifferentialPressureBias[i] + (*config_.DifferentialPressure[i]-data_.DifferentialPressureBias[i])/((float)NumberSamples_);
    }
    NumberSamples_++;
  } else {
    Initialized_ = true;
  }
}

bool IndicatedAirspeed::Initialized() {
  return Initialized_;
}

void IndicatedAirspeed::Run(Mode mode) {
  data_.Mode = (uint8_t) mode;
  if (mode!=kStandby) {
    // compute average differential pressure
    data_.AvgDifferentialPressure = 0.0f;
    for (size_t i=0; i < config_.DifferentialPressure.size(); i++) {
      data_.AvgDifferentialPressure += (*config_.DifferentialPressure[i]-data_.DifferentialPressureBias[i])/config_.DifferentialPressure.size();
    }
    // compute indicated airspeed
    data_.Ias_ms = AirData_.getIAS(data_.AvgDifferentialPressure);
  }
}

void IndicatedAirspeed::Clear(DefinitionTree *DefinitionTreePtr) {
  config_.DifferentialPressure.clear();
  config_.InitTime = 0.0f;
  data_.DifferentialPressureBias.clear();
  data_.Mode = kStandby;
  data_.Ias_ms = 0.0f;
  data_.AvgDifferentialPressure = 0.0f;
  bool TimeLatch_ = false;
  bool Initialized_ = false;
  uint64_t T0_us_ = 0;
  size_t NumberSamples_ = 1;
  DefinitionTreePtr->Erase(ModeKey_);
  DefinitionTreePtr->Erase(OutputKey_);
  DifferentialPressureKeys_.clear();
  ModeKey_.clear();
  OutputKey_.clear();
}

uint64_t IndicatedAirspeed::micros() {
  struct timeval tv;
  gettimeofday(&tv,NULL);
  return tv.tv_sec*(uint64_t)1000000+tv.tv_usec;
}

void AglAltitude::Configure(const rapidjson::Value& Config,std::string RootPath,DefinitionTree *DefinitionTreePtr) {
  // get output name
  std::string OutputName;
  if (Config.HasMember("Output")) {
    OutputName = RootPath + "/" + Config["Output"].GetString();
  } else {
    throw std::runtime_error(std::string("ERROR")+RootPath+std::string(": Output not specified in configuration."));
  }
  // get pressure sources
  if (Config.HasMember("Static-Pressure")) {
    const rapidjson::Value& PressureSources = Config["Static-Pressure"];
    for (auto &PressureSource : PressureSources.GetArray()) {
      StaticPressureKeys_.push_back(PressureSource.GetString());
      if (DefinitionTreePtr->GetValuePtr<float*>(StaticPressureKeys_.back())) {
        config_.StaticPressure.push_back(DefinitionTreePtr->GetValuePtr<float*>(StaticPressureKeys_.back()));
      } else {
        throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Static pressure source ")+StaticPressureKeys_.back()+std::string(" not found in global data."));
      }
    }
  } else {
    throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Static pressure sources not specified in configuration."));
  }
  // get initialization time
  if (Config.HasMember("Initialization-Time")) {
    config_.InitTime = Config["Initialization-Time"].GetFloat();
  } else {
    throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Initialization time not specified in configuration."));
  }
  // pointer to log run mode data
  ModeKey_ = OutputName+"/Mode";
  DefinitionTreePtr->InitMember(ModeKey_,&data_.Mode,"Run mode",true,false);
  // pointer to log ias data
  OutputKey_ = OutputName+"/"+Config["Output"].GetString();
  DefinitionTreePtr->InitMember(OutputKey_,&data_.Agl_m,"Altitude above ground, m",true,false);
}

void AglAltitude::Initialize() {
  // grab the starting time
  if (!TimeLatch_) {
    T0_us_ = micros();
    TimeLatch_ = true;
  }
  // compute the elapsed time
  float ElapsedTime = ((float)(micros()-T0_us_))/1e6;
  // if less than init time, compute initial pressure altitude
  if (ElapsedTime < config_.InitTime) {
    // average static pressure sources
    data_.AvgStaticPressure = 0.0f;
    for (size_t i=0; i < config_.StaticPressure.size(); i++) {
      data_.AvgStaticPressure += (*config_.StaticPressure[i])/config_.StaticPressure.size();
    }
    // compute pressure altitude
    data_.PressAlt0 = data_.PressAlt0 + (AirData_.getPressureAltitude(data_.AvgStaticPressure)-data_.PressAlt0)/((float)NumberSamples_);
    NumberSamples_++;
  } else {
    Initialized_ = true;
  }
}

bool AglAltitude::Initialized() {
  return Initialized_;
}

void AglAltitude::Run(Mode mode) {
  data_.Mode = (uint8_t) mode;
  if (mode!=kStandby) {
    // compute average static pressure
    data_.AvgStaticPressure = 0.0f;
    for (size_t i=0; i < config_.StaticPressure.size(); i++) {
      data_.AvgStaticPressure += (*config_.StaticPressure[i])/config_.StaticPressure.size();
    }
    // compute altitude above ground level
    data_.Agl_m = AirData_.getAGL(data_.AvgStaticPressure,data_.PressAlt0);
  }
}

void AglAltitude::Clear(DefinitionTree *DefinitionTreePtr) {
  config_.StaticPressure.clear();
  config_.InitTime = 0.0f;
  data_.Mode = kStandby;
  data_.Agl_m = 0.0f;
  data_.PressAlt0 = 0.0f;
  data_.AvgStaticPressure = 0.0f;
  bool TimeLatch_ = false;
  bool Initialized_ = false;
  uint64_t T0_us_ = 0;
  size_t NumberSamples_ = 1;
  DefinitionTreePtr->Erase(ModeKey_);
  DefinitionTreePtr->Erase(OutputKey_);
  StaticPressureKeys_.clear();
  ModeKey_.clear();
  OutputKey_.clear();
}

uint64_t AglAltitude::micros() {
  struct timeval tv;
  gettimeofday(&tv,NULL);
  return tv.tv_sec*(uint64_t)1000000+tv.tv_usec;
}
