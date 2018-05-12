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
  std::string OutputName;
  if (Config.HasMember("Output")) {
    OutputName = RootPath + "/" + Config["Output"].GetString();
  } else {
    throw std::runtime_error(std::string("ERROR")+RootPath+std::string(": Output not specified in configuration."));
  }
  if (Config.HasMember("Reference")) {
    ReferenceKey_ = Config["Reference"].GetString();
    if (DefinitionTreePtr->GetValuePtr<float*>(ReferenceKey_)) {
      config_.Reference = DefinitionTreePtr->GetValuePtr<float*>(ReferenceKey_);
    } else {
      throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Reference ")+ReferenceKey_+std::string(" not found in global data."));
    }
  } else {
    throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Reference not specified in configuration."));
  }
  if (Config.HasMember("Feedback")) {
    FeedbackKey_ = Config["Feedback"].GetString();
    if (DefinitionTreePtr->GetValuePtr<float*>(FeedbackKey_)) {
      config_.Reference = DefinitionTreePtr->GetValuePtr<float*>(FeedbackKey_);
    } else {
      throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Feedback ")+FeedbackKey_+std::string(" not found in global data."));
    }
  } else {
    throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Feedback not specified in configuration."));
  }
  if (Config.HasMember("Gains")) {
    const rapidjson::Value& Gains = Config["Gains"];
    if (Gains.HasMember("Proportional")) {
      config_.Kp = Gains["Proportional"].GetFloat();
    }
    if (Gains.HasMember("Derivative")) {
      config_.Kd = Gains["Derivative"].GetFloat();
    }
    if (Gains.HasMember("Integral")) {
      config_.Ki = Gains["Integral"].GetFloat();
    }
  } else {
    throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Gains not specified in configuration."));
  }
  if (Config.HasMember("Sample-Time")) {
    if (Config["Sample-Time"].IsString()) {
      SampleTimeKey_ = Config["Sample-Time"].GetString();
      if (DefinitionTreePtr->GetValuePtr<float*>(SampleTimeKey_)) {
        config_.dt = DefinitionTreePtr->GetValuePtr<float*>(SampleTimeKey_);
      } else {
        throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Sample time ")+SampleTimeKey_+std::string(" not found in global data."));
      }
    } else {
      config_.UseSampleTime = true;
      config_.SampleTime = Config["Sample-Time"].GetFloat();
    }
  } else {
    throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Sample time not specified in configuration."));
  }
  if (Config.HasMember("Setpoint-Weights")) {
    const rapidjson::Value& Weights = Config["Setpoint-Weights"];
    if (Weights.HasMember("Proportional")) {
      config_.b = Weights["Proportional"].GetFloat();
    }
    if (Weights.HasMember("Derivative")) {
      config_.c = Weights["Derivative"].GetFloat();
    }
  }
  if (Config.HasMember("Limits")) {
    config_.SaturateOutput = true;
    // pointer to log saturation data
    SaturatedKey_ = OutputName+"/Saturated";
    DefinitionTreePtr->InitMember(SaturatedKey_,&data_.Saturated,"Control law saturation, 0 if not saturated, 1 if saturated on the upper limit, and -1 if saturated on the lower limit",true,false);
    if (Config["Limits"].HasMember("Lower")&&Config["Limits"].HasMember("Upper")) {
      config_.UpperLimit = Config["Limits"]["Upper"].GetFloat();
      config_.LowerLimit = Config["Limits"]["Lower"].GetFloat();
    } else {
      throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Either upper or lower limit not specified in configuration."));
    }
  }
  // pointer to log run mode data
  ModeKey_ = OutputName+"/Mode";
  DefinitionTreePtr->InitMember(ModeKey_,&data_.Mode,"Run mode",true,false);
  // pointer to log command data
  OutputKey_ = OutputName+"/Output";
  DefinitionTreePtr->InitMember(OutputKey_,&data_.Output,"Control law output",true,false);
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
  // error for proportional
  states_.ProportionalError = (config_.b*(*config_.Reference))-*config_.Feedback;
  // error for integral
  states_.IntegralError = *config_.Reference-*config_.Feedback;
  // error for derivative
  states_.DerivativeError = (config_.c*(*config_.Reference))-*config_.Feedback;
  // derivative of Error
  if (config_.SampleTime > 0.0f) {
    states_.DerivativeErrorState = 1.0f/(config_.Tf+config_.SampleTime/(states_.DerivativeError-states_.PreviousDerivativeError));
  }
  states_.PreviousDerivativeError = states_.DerivativeError;
  switch(mode) {
    case kStandby: {
      break;
    }
    case kArm: {
      InitializeState(0.0f);
      CalculateCommand();
      break;
    }
    case kHold: {
      CalculateCommand();
      break;
    }
    case kEngage: {
      UpdateState();
      CalculateCommand();
      break;
    }
  }
}

void PIDClass::InitializeState(float Command) {
  // Protect for Ki == 0
  if (config_.Ki != 0.0f) {
    states_.IntegralErrorState = (Command-(config_.Kp*states_.ProportionalError+config_.Kd*states_.DerivativeErrorState))/config_.Ki;
  } else {
    states_.IntegralErrorState = 0.0f;
  }
}

void PIDClass::UpdateState() {
  // Protect for unlimited windup when Ki == 0
  if (config_.Ki != 0.0f) {
    states_.IntegralErrorState += (config_.SampleTime*states_.IntegralError);
  } else {
    states_.IntegralErrorState = 0.0;
  }
}

void PIDClass::CalculateCommand() {
  float ProportionalCommand = config_.Kp*states_.ProportionalError;
  float IntegralCommand = config_.Ki*states_.IntegralErrorState;
  float DerivativeCommand = config_.Kd*states_.DerivativeErrorState;
  data_.Output = ProportionalCommand+IntegralCommand+DerivativeCommand;
  // saturate cmd, set iErr to limit that produces saturated cmd
  // saturate command
  if (config_.SaturateOutput) {
    if (data_.Output <= config_.LowerLimit) {
      data_.Output = config_.LowerLimit;
      data_.Saturated = -1;
      // Re-compute the integrator state
      InitializeState(data_.Output);
    } else if (data_.Output >= config_.UpperLimit) {
      data_.Output = config_.UpperLimit;
      data_.Saturated = 1;
      // Re-compute the integrator state
      InitializeState(data_.Output);
    } else {
      data_.Saturated = 0;
    }
  }
}

void PIDClass::Clear(DefinitionTree *DefinitionTreePtr) {
  config_.UseSampleTime = false;
  config_.Kp = 0.0f;
  config_.Ki = 0.0f;
  config_.Kd = 0.0f;
  config_.Tf = 0.0f;
  config_.b = 1.0f;
  config_.c = 1.0f;
  config_.SaturateOutput = false;
  config_.UpperLimit = 0.0f;
  config_.LowerLimit = 0.0f;
  states_.ProportionalError = 0.0f;
  states_.DerivativeError = 0.0f;
  states_.PreviousDerivativeError = 0.0f;
  states_.IntegralError = 0.0f;
  states_.DerivativeErrorState = 0.0f;
  states_.IntegralErrorState = 0.0f;
  data_.Mode = kStandby;
  data_.Saturated = 0;
  data_.Output = 0.0f;
  DefinitionTreePtr->Erase(ReferenceKey_);
  DefinitionTreePtr->Erase(FeedbackKey_);
  DefinitionTreePtr->Erase(ModeKey_);
  DefinitionTreePtr->Erase(SaturatedKey_);
  DefinitionTreePtr->Erase(OutputKey_);
  ReferenceKey_.clear();
  FeedbackKey_.clear();
  ModeKey_.clear();
  SaturatedKey_.clear();
  OutputKey_.clear();
}
