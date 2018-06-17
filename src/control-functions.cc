/*
control-functions.cc
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

#include "control-functions.hxx"

/* PID class methods, see control-functions.hxx for more information */
void PIDClass::Configure(const rapidjson::Value& Config,std::string RootPath,DefinitionTree *DefinitionTreePtr) {
  float Kp = 0;
  float Ki = 0;
  float Kd = 0;
  float b = 1;
  float c = 1;
  float Tf = 0;
  float UpperLimit = 0;
  float LowerLimit = 0;
  bool SaturateOutput = false;
  std::string OutputName;
  std::string SysName;

  if (Config.HasMember("Output")) {
    OutputName = Config["Output"].GetString();
    SysName = RootPath + "/" + OutputName;

    // pointer to log run mode data
    ModeKey_ = SysName + "/Mode";
    DefinitionTreePtr->InitMember(ModeKey_,&data_.Mode,"Run mode",true,false);

    // pointer to log command data
    OutputKey_ = SysName;
    DefinitionTreePtr->InitMember(OutputKey_,&data_.Output,"Control law output",true,false);

  } else {
    throw std::runtime_error(std::string("ERROR")+RootPath+std::string(": Output not specified in configuration."));
  }

  if (Config.HasMember("Reference")) {
    ReferenceKey_ = Config["Reference"].GetString();
    if (DefinitionTreePtr->GetValuePtr<float*>(ReferenceKey_)) {
      config_.Reference = DefinitionTreePtr->GetValuePtr<float*>(ReferenceKey_);
    } else {
      throw std::runtime_error(std::string("ERROR")+SysName+std::string(": Reference ")+ReferenceKey_+std::string(" not found in global data."));
    }
  } else {
    throw std::runtime_error(std::string("ERROR")+SysName+std::string(": Reference not specified in configuration."));
  }
  if (Config.HasMember("Feedback")) {
    FeedbackKey_ = Config["Feedback"].GetString();
    if (DefinitionTreePtr->GetValuePtr<float*>(FeedbackKey_)) {
      config_.Feedback = DefinitionTreePtr->GetValuePtr<float*>(FeedbackKey_);
    } else {
      throw std::runtime_error(std::string("ERROR")+SysName+std::string(": Feedback ")+FeedbackKey_+std::string(" not found in global data."));
    }
  } else {
    throw std::runtime_error(std::string("ERROR")+SysName+std::string(": Feedback not specified in configuration."));
  }
  if (Config.HasMember("Gains")) {
    const rapidjson::Value& Gains = Config["Gains"];
    if (Gains.HasMember("Proportional")) {
      Kp = Gains["Proportional"].GetFloat();
    }
    if (Gains.HasMember("Derivative")) {
      Kd = Gains["Derivative"].GetFloat();
    }
    if (Gains.HasMember("Integral")) {
      Ki = Gains["Integral"].GetFloat();
    }
  } else {
    throw std::runtime_error(std::string("ERROR")+SysName+std::string(": Gains not specified in configuration."));
  }

  if (Config.HasMember("Sample-Time")) {
    if (Config["Sample-Time"].IsString()) {
      SampleTimeKey_ = Config["Sample-Time"].GetString();
      if (DefinitionTreePtr->GetValuePtr<float*>(SampleTimeKey_)) {
        config_.dt = DefinitionTreePtr->GetValuePtr<float*>(SampleTimeKey_);
      } else {
        throw std::runtime_error(std::string("ERROR")+SysName+std::string(": Sample time ")+SampleTimeKey_+std::string(" not found in global data."));
      }
    } else {
      config_.UseSampleTime = true;
      config_.SampleTime = Config["Sample-Time"].GetFloat();
    }
  } else {
    throw std::runtime_error(std::string("ERROR")+SysName+std::string(": Sample time not specified in configuration."));
  }

  if (Config.HasMember("Setpoint-Weights")) {
    const rapidjson::Value& Weights = Config["Setpoint-Weights"];
    if (Weights.HasMember("Proportional")) {
      b = Weights["Proportional"].GetFloat();
    }
    if (Weights.HasMember("Derivative")) {
      c = Weights["Derivative"].GetFloat();
    }
  }

  if (Config.HasMember("Time-Constant")) {
    Tf = Config["Time-Constant"].GetFloat();
  }

  if (Config.HasMember("Limits")) {
    SaturateOutput = true;
    // pointer to log saturation data
    SaturatedKey_ = SysName+"/Saturated";
    DefinitionTreePtr->InitMember(SaturatedKey_,&data_.Saturated,"Control law saturation, 0 if not saturated, 1 if saturated on the upper limit, and -1 if saturated on the lower limit",true,false);
    if (Config["Limits"].HasMember("Lower")&&Config["Limits"].HasMember("Upper")) {
      UpperLimit = Config["Limits"]["Upper"].GetFloat();
      LowerLimit = Config["Limits"]["Lower"].GetFloat();
    } else {
      throw std::runtime_error(std::string("ERROR")+SysName+std::string(": Either upper or lower limit not specified in configuration."));
    }
  }

  // configure PID Class
  PIDClass_.Configure(Kp,Ki,Kd,b,c,Tf,SaturateOutput,UpperLimit,LowerLimit);
}

void PIDClass::Initialize() {}
bool PIDClass::Initialized() {return true;}

void PIDClass::Run(Mode mode) {
  // mode
  data_.Mode = (uint8_t) mode;
  // sample time
  if(!config_.UseSampleTime) {
    config_.SampleTime = *config_.dt;
  }
  PIDClass_.Run(mode,*config_.Reference,*config_.Feedback,config_.SampleTime,&data_.Output,&data_.Saturated);
}

void PIDClass::Clear(DefinitionTree *DefinitionTreePtr) {
  config_.UseSampleTime = false;
  data_.Mode = kStandby;
  data_.Saturated = 0;
  data_.Output = 0.0f;
  DefinitionTreePtr->Erase(ModeKey_);
  DefinitionTreePtr->Erase(SaturatedKey_);
  DefinitionTreePtr->Erase(OutputKey_);
  ReferenceKey_.clear();
  FeedbackKey_.clear();
  ModeKey_.clear();
  SaturatedKey_.clear();
  OutputKey_.clear();
  PIDClass_.Clear();
}

/* SS class methods, see control-functions.hxx for more information */
void SSClass::Configure(const rapidjson::Value& Config,std::string RootPath,DefinitionTree *DefinitionTreePtr) {
  bool SatFlag = false;
  std::string OutputName;
  std::string SysName;

  if (Config.HasMember("Name")) {
    SysName = RootPath + "/" + Config["Name"].GetString();

    // pointer to log run mode data
    ModeKey_ = SysName + "/Mode";
    DefinitionTreePtr->InitMember(ModeKey_,&data_.Mode,"Run mode",true,false);
  } else {
    throw std::runtime_error(std::string("ERROR")+RootPath+std::string(": Name not specified in configuration."));
  }

  // grab inputs
  if (Config.HasMember("Inputs")) {
    for (size_t i=0; i < Config["Inputs"].Size(); i++) {
      const rapidjson::Value& Input = Config["Inputs"][i];
      InputKeys_.push_back(Input.GetString());
      if (DefinitionTreePtr->GetValuePtr<float*>(InputKeys_.back())) {
        config_.Inputs.push_back(DefinitionTreePtr->GetValuePtr<float*>(InputKeys_.back()));
      } else {
        throw std::runtime_error(std::string("ERROR")+SysName+std::string(": Input ")+InputKeys_.back()+std::string(" not found in global data."));
      }
    }
  } else {
    throw std::runtime_error(std::string("ERROR")+SysName+std::string(": Inputs not specified in configuration."));
  }

  // grab outputs
  if (Config.HasMember("Outputs")) {
    // resize output matrix
    data_.y.resize(Config["Outputs"].Size());
    data_.ySat.resize(Config["Outputs"].Size());
    for (size_t i=0; i < Config["Outputs"].Size(); i++) {
      const rapidjson::Value& Output = Config["Outputs"][i];
      std::string OutputName = Output.GetString();

      // pointer to log output
      OutputKeys_.push_back(SysName +"/"+ OutputName);
      DefinitionTreePtr->InitMember(OutputKeys_.back(),&data_.y(i),"SS output",true,false);

      // pointer to log saturation data
      SaturatedKeys_.push_back(SysName +"/"+ OutputName + "/Saturated");
      DefinitionTreePtr->InitMember(SaturatedKeys_.back(),&data_.ySat(i),"Allocation saturation, 0 if not saturated, 1 if saturated on the upper limit, and -1 if saturated on the lower limit",true,false);
    }

  } else {
    throw std::runtime_error(std::string("ERROR")+SysName+std::string(": Outputs not specified in configuration."));
  }

  // grab A
  if (Config.HasMember("A")) {
    // resize A matrix
    config_.A.resize(Config["A"][0].Size(),Config["A"].Size());
    for (size_t m=0; m < Config["A"].Size(); m++) {
      for (size_t n=0; n < Config["A"][m].Size(); n++) {
        config_.A(m,n) = Config["A"][m][n].GetFloat();
      }
    }
  } else {
    throw std::runtime_error(std::string("ERROR")+SysName+std::string(": A not specified in configuration."));
  }

  // resize state vector
  config_.x.resize(config_.A.size());

  // grab B
  if (Config.HasMember("B")) {
    // resize B matrix
    config_.B.resize(Config["B"][0].Size(),Config["B"].Size());
    for (size_t m=0; m < Config["B"].Size(); m++) {
      for (size_t n=0; n < Config["B"][m].Size(); n++) {
        config_.B(m,n) = Config["B"][m][n].GetFloat();
      }
    }
  } else {
    throw std::runtime_error(std::string("ERROR")+SysName+std::string(": B not specified in configuration."));
  }

  // grab C
  if (Config.HasMember("C")) {
    // resize C matrix
    config_.C.resize(Config["C"][0].Size(),Config["C"].Size());
    for (size_t m=0; m < Config["C"].Size(); m++) {
      for (size_t n=0; n < Config["C"][m].Size(); n++) {
        config_.C(m,n) = Config["C"][m][n].GetFloat();
      }
    }
  } else {
    throw std::runtime_error(std::string("ERROR")+SysName+std::string(": C not specified in configuration."));
  }

  // grab D
  if (Config.HasMember("D")) {
    // resize D matrix
    config_.D.resize(Config["D"][0].Size(),Config["D"].Size());
    for (size_t m=0; m < Config["D"].Size(); m++) {
      for (size_t n=0; n < Config["D"][m].Size(); n++) {
        config_.D(m,n) = Config["D"][m][n].GetFloat();
      }
    }
  } else {
    throw std::runtime_error(std::string("ERROR")+SysName+std::string(": D not specified in configuration."));
  }

  // grab stample time
  if (Config.HasMember("Sample-Time")) {
    if (Config["Sample-Time"].IsString()) {
      SampleTimeKey_ = Config["Sample-Time"].GetString();
      if (DefinitionTreePtr->GetValuePtr<float*>(SampleTimeKey_)) {
        config_.dt = DefinitionTreePtr->GetValuePtr<float*>(SampleTimeKey_);
      } else {
        throw std::runtime_error(std::string("ERROR")+SysName+std::string(": Sample time ")+SampleTimeKey_+std::string(" not found in global data."));
      }
    } else {
      config_.UseSampleTime = true;
      config_.SampleTime = Config["Sample-Time"].GetFloat();
    }
  } else {
    throw std::runtime_error(std::string("ERROR")+SysName+std::string(": Sample time not specified in configuration."));
  }

  // grab limits
  if (Config.HasMember("Limits")) {
    if (Config["Limits"].HasMember("Lower")&&Config["Limits"].HasMember("Upper")) {
      SatFlag = true;

      // resize limit vectors
      config_.yMin.resize(Config["Limits"]["Lower"].Size());
      config_.yMax.resize(Config["Limits"]["Upper"].Size());
      for (size_t i=0; i < Config["Limits"]["Lower"].Size(); i++) {
        config_.yMin(i) = Config["Limits"]["Lower"][i].GetFloat();
      }
      for (size_t i=0; i < Config["Limits"]["Upper"].Size(); i++) {
        config_.yMax(i) = Config["Limits"]["Upper"][i].GetFloat();
      }
    } else {
      throw std::runtime_error(std::string("ERROR")+SysName+std::string(": Either upper or lower limit not specified in configuration."));
    }
  } else {
    throw std::runtime_error(std::string("ERROR")+SysName+std::string(": Limits not specified in configuration."));
  }


  // configure SS Class
  SSClass_.Configure(config_.A, config_.B, config_.C, config_.D, SatFlag, config_.yMax, config_.yMin);
}

void SSClass::Initialize() {}
bool SSClass::Initialized() {return true;}

void SSClass::Run(Mode mode) {
  // mode
  data_.Mode = (uint8_t) mode;

  // sample time
  if(!config_.UseSampleTime) {
    config_.SampleTime = *config_.dt;
  }

  // inputs to Eigen3 vector
  for (size_t i=0; i < config_.Inputs.size(); i++) {
    config_.u(i) = *config_.Inputs[i];
  }

  // Call Algorithm
  SSClass_.Run(mode, config_.u, config_.SampleTime, &data_.y, &data_.ySat);
}

void SSClass::Clear(DefinitionTree *DefinitionTreePtr) {
  config_.UseSampleTime = false;
  data_.Mode = kStandby;
  config_.A.resize(0,0);
  config_.B.resize(0,0);
  config_.C.resize(0,0);
  config_.D.resize(0,0);
  DefinitionTreePtr->Erase(ModeKey_);
  DefinitionTreePtr->Erase(SampleTimeKey_);
  ModeKey_.clear();
  SampleTimeKey_.clear();
  InputKeys_.clear();
  OutputKeys_.clear();
  SaturatedKeys_.clear();
  SSClass_.Clear();
}
