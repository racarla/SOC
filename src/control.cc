/*
control.cc
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

#include "control.hxx"

/* base function class methods */
void ControlFunctionClass::Configure(const rapidjson::Value& Config,std::string RootPath,DefinitionTree *DefinitionTreePtr) {}
void ControlFunctionClass::SetPreviousOutput(std::string RootPath,DefinitionTree *DefinitionTreePtr) {}
void ControlFunctionClass::Run(Mode mode) {}

/* control constant class methods */

/* method for configuring the constant block */
/* example JSON configuration:
{
  "Output": "OutputName",
  "Constant": X
}
Where OutputName gives a convenient name for the block (i.e. SpeedControl).
Constant is the value of the constant output. Data type for output is float.
*/
void ControlConstantClass::Configure(const rapidjson::Value& Config,std::string RootPath,DefinitionTree *DefinitionTreePtr) {
  std::string OutputName;
  if (Config.HasMember("Output")) {
    OutputName = RootPath + "/" + Config["Output"].GetString();
  } else {
    throw std::runtime_error(std::string("ERROR")+RootPath+std::string(": Output not specified in configuration."));
  }
  if (Config.HasMember("Constant")) {
    config_.Constant = Config["Constant"].GetFloat();
  } else {
    throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Constant value not specified in configuration."));
  }
  // pointer to log run mode data
  DefinitionTreePtr->InitMember(OutputName+"/Mode",&data_.Mode,"Control law mode",true,false);
  // pointer to log command data
  DefinitionTreePtr->InitMember(OutputName+"/Output",&data_.Command,"Control law output",true,false);
}

/* constant block run method, outputs the mode and constant value */
void ControlConstantClass::Run(Mode mode) {
  data_.Mode = (uint8_t) mode;
  data_.Command = config_.Constant;
}

/* control gain class methods */

/* method for configuring the gain block */
/* example JSON configuration:
{
  "Output": "OutputName",
  "Input": "InputName",
  "Gain": X,
  "Limits": {
    "Upper": X,
    "Lower": X
  }
}
Where OutputName gives a convenient name for the block (i.e. SpeedControl).
Input is the full path name of the input signal
Gain is the gain applied to the input signal
Limits are optional and saturate the output if defined
*/
void ControlGainClass::Configure(const rapidjson::Value& Config,std::string RootPath,DefinitionTree *DefinitionTreePtr) {
  std::string OutputName;
  if (Config.HasMember("Output")) {
    OutputName = RootPath + "/" + Config["Output"].GetString();
  } else {
    throw std::runtime_error(std::string("ERROR")+RootPath+std::string(": Output not specified in configuration."));
  }
  if (Config.HasMember("Gain")) {
    config_.Gain = Config["Gain"].GetFloat();
  } else {
    throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Gain value not specified in configuration."));
  }
  if (Config.HasMember("Input")) {
    if (DefinitionTreePtr->GetValuePtr<float*>(Config["Input"].GetString())) {
      config_.Input = DefinitionTreePtr->GetValuePtr<float*>(Config["Input"].GetString());
    } else {
      throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Input ")+Config["Input"].GetString()+std::string(" not found in global data."));
    }
  } else {
    throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Input not specified in configuration."));
  }
  if (Config.HasMember("Limits")) {
    config_.SaturateOutput = true;
    // pointer to log saturation data
    DefinitionTreePtr->InitMember(OutputName+"/Saturated",&data_.Saturated,"Control law saturation, 0 if not saturated, 1 if saturated on the upper limit, and -1 if saturated on the lower limit",true,false);
    if (Config["Limits"].HasMember("Lower")&&Config["Limits"].HasMember("Upper")) {
      config_.UpperLimit = Config["Limits"]["Upper"].GetFloat();
      config_.LowerLimit = Config["Limits"]["Lower"].GetFloat();
    } else {
      throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Either upper or lower limit not specified in configuration."));
    }
  }
  // pointer to log run mode data
  DefinitionTreePtr->InitMember(OutputName+"/Mode",&data_.Mode,"Control law mode",true,false);
  // pointer to log command data
  DefinitionTreePtr->InitMember(OutputName+"/Output",&data_.Command,"Control law output",true,false);
}

/* gain block run method, outputs the mode and value */
void ControlGainClass::Run(Mode mode) {
  data_.Mode = (uint8_t) mode;
  switch (mode) {
    // Zero the State and Command
    case kReset: {
      data_.Command = 0;
      data_.Saturated = 0;
      break;
    }
    // Do Nothing, State and Command are unchanged
    case kStandby: {
      break;
    }
    // Run Commands
    case kHold: {
      CalculateCommand();
      break;
    }
    // Initialize State then Run Commands
    case kInitialize: {
      CalculateCommand();
      break;
    }
    // Update the State then Run Commands
    case kEngage: {
      CalculateCommand();
      break;
    }
  }
}

/* calculate the command and apply saturation, if enabled */
void ControlGainClass::CalculateCommand() {
  data_.Command = *config_.Input*config_.Gain;
  // saturate command
  if (config_.SaturateOutput) {
    if (data_.Command <= config_.LowerLimit) {
      data_.Command = config_.LowerLimit;
      data_.Saturated = -1;
    } else if (data_.Command >= config_.UpperLimit) {
      data_.Command = config_.UpperLimit;
      data_.Saturated = 1;
    } else {
      data_.Saturated = 0;
    }
  }
}

/* control sum class methods */

/* method for configuring the sum block */
/* example JSON configuration:
{
  "Output": "OutputName",
  "Inputs": ["InputName1","InputName2",...],
  "Limits": {
    "Upper": X,
    "Lower": X
  }
}
Where OutputName gives a convenient name for the block (i.e. SpeedControl).
Input is an array of full path names of the input signals. An unlimited number
of input signals can be used.
Limits are optional and saturate the output if defined
*/
void ControlSumClass::Configure(const rapidjson::Value& Config,std::string RootPath,DefinitionTree *DefinitionTreePtr) {
  std::string OutputName;
  if (Config.HasMember("Output")) {
    OutputName = RootPath + "/" + Config["Output"].GetString();
  } else {
    throw std::runtime_error(std::string("ERROR")+RootPath+std::string(": Output not specified in configuration."));
  }
  if (Config.HasMember("Inputs")) {
    for (size_t i=0; i < Config["Inputs"].Size(); i++) {
      const rapidjson::Value& Input = Config["Inputs"][i];
      if (DefinitionTreePtr->GetValuePtr<float*>(Input.GetString())) {
        config_.Inputs.push_back(DefinitionTreePtr->GetValuePtr<float*>(Input.GetString()));
      } else {
        throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Input ")+Input.GetString()+std::string(" not found in global data."));
      }
    }
  } else {
    throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Inputs not specified in configuration."));
  }
  if (Config.HasMember("Limits")) {
    config_.SaturateOutput = true;
    // pointer to log saturation data
    DefinitionTreePtr->InitMember(OutputName+"/Saturated",&data_.Saturated,"Control law saturation, 0 if not saturated, 1 if saturated on the upper limit, and -1 if saturated on the lower limit",true,false);
    if (Config["Limits"].HasMember("Lower")&&Config["Limits"].HasMember("Upper")) {
      config_.UpperLimit = Config["Limits"]["Upper"].GetFloat();
      config_.LowerLimit = Config["Limits"]["Lower"].GetFloat();
    } else {
      throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Either upper or lower limit not specified in configuration."));
    }
  }
  // pointer to log run mode data
  DefinitionTreePtr->InitMember(OutputName+"/Mode",&data_.Mode,"Run mode",true,false);
  // pointer to log command data
  DefinitionTreePtr->InitMember(OutputName+"/Command",&data_.Command,"Control law output",true,false);
}

/* sum block run method, outputs the mode and value */
void ControlSumClass::Run(Mode mode) {
  data_.Mode = (uint8_t) mode;
  switch (mode) {
    // Zero the State and Command
    case kReset: {
      data_.Command = 0;
      break;
    }
    // Do Nothing, State and Command are unchanged
    case kStandby: {
      break;
    }
    // Run Commands
    case kHold: {
      CalculateCommand();
      break;
    }
    // Initialize State then Run Commands
    case kInitialize: {
      CalculateCommand();
      break;
    }
    // Update the State then Run Commands
    case kEngage: {
      CalculateCommand();
      break;
    }
  }
}

/* calculate the command and apply saturation, if enabled */
void ControlSumClass::CalculateCommand() {
  data_.Command = 0;
  for (size_t i=0; i < config_.Inputs.size(); i++) {
    data_.Command += *config_.Inputs[i];
  }
  // saturate command
  if (config_.SaturateOutput) {
    if (data_.Command <= config_.LowerLimit) {
      data_.Command = config_.LowerLimit;
      data_.Saturated = -1;
    } else if (data_.Command >= config_.UpperLimit) {
      data_.Command = config_.UpperLimit;
      data_.Saturated = 1;
    } else {
      data_.Saturated = 0;
    }
  }
}

/* washout filter class methods */

/* method for configuring the washout filter block */
/* example JSON configuration:
{
  "Output": "OutputName",
  "Input": "InputName",
  "Sample-Time": "SampleTimeName",
  "Time-Constant": X
}
Where OutputName gives a convenient name for the block (i.e. SpeedControl).
Input is the full path name of the input signal
Sample-Time is the full path name of the sample time signal
Time-Constant is the time constant of the filter
*/
void WashoutFilterClass::Configure(const rapidjson::Value& Config,std::string RootPath,DefinitionTree *DefinitionTreePtr) {
  std::string OutputName;
  if (Config.HasMember("Output")) {
    config_.OutputName = Config["Output"].GetString();
    OutputName = RootPath + "/" + Config["Output"].GetString();
  } else {
    throw std::runtime_error(std::string("ERROR")+RootPath+std::string(": Output not specified in configuration."));
  }
  if (Config.HasMember("Input")) {
    if (DefinitionTreePtr->GetValuePtr<float*>(Config["Input"].GetString())) {
      config_.Input = DefinitionTreePtr->GetValuePtr<float*>(Config["Input"].GetString());
    } else {
      throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Input ")+Config["Input"].GetString()+std::string(" not found in global data."));
    }
  } else {
    throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Input not specified in configuration."));
  }
  if (Config.HasMember("Sample-Time")) {
    if (DefinitionTreePtr->GetValuePtr<float*>(Config["Sample-Time"].GetString())) {
      config_.dt = DefinitionTreePtr->GetValuePtr<float*>(Config["Sample-Time"].GetString());
    } else {
      throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Sample time ")+Config["Sample-Time"].GetString()+std::string(" not found in global data."));
    }
  } else {
    throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Sample time not specified in configuration."));
  }
  if (Config.HasMember("Time-Constant")) {
    config_.Tf = Config["Time-Constant"].GetFloat();
  } else {
    throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Time constant not specified in configuration."));
  }
  // pointer to log run mode data
  DefinitionTreePtr->InitMember(OutputName+"/Mode",&data_.Mode,"Control law mode",true,false);
  // pointer to log command data
  DefinitionTreePtr->InitMember(OutputName+"/Output",&data_.Command,"Control law output",true,false);
}

/* sets a pointer to the previous output in order to initialize to a transient free state */
void WashoutFilterClass::SetPreviousOutput(std::string RootPath,DefinitionTree *DefinitionTreePtr) {
  if (DefinitionTreePtr->GetValuePtr<float*>(RootPath+ "/" + config_.OutputName)) {
    config_.PreviousCommand = DefinitionTreePtr->GetValuePtr<float*>(RootPath+ "/" + config_.OutputName);
  } else {
    throw std::runtime_error(std::string("ERROR")+std::string(": Command ")+RootPath+ "/" + config_.OutputName+std::string(" not found in global data."));
  }
}

/* washout filter run method, outputs the mode and value */
void WashoutFilterClass::Run(Mode mode) {
  data_.Mode = (uint8_t) mode;
  switch (mode) {
    // Zero the State and Command
    case kReset: {
      data_.Command = 0;
      break;
    }
    // Do Nothing, State and Command are unchanged
    case kStandby: {
      break;
    }
    // Run Commands
    case kHold: {
      CalculateCommand();
      break;
    }
    // Initialize State then Run Commands
    case kInitialize: {
      InitializeState();
      CalculateCommand();
      break;
    }
    // Update the State and Run Commands
    case kEngage: {
      CalculateCommand();
      UpdateState();
      break;
    }
  }
}

/* initialize the state */
void WashoutFilterClass::InitializeState() {
  states_.Filter = *config_.Input - *config_.PreviousCommand;
}

/* update the state */
void WashoutFilterClass::UpdateState() {
  if (config_.Tf > *config_.dt) {
    states_.Filter = (1.0f-(*config_.dt/config_.Tf))*states_.Filter + (*config_.dt/config_.Tf)*(*config_.Input);
  }
}

/* calculate the command */
void WashoutFilterClass::CalculateCommand() {
  if (config_.Tf > *config_.dt) {
    data_.Command = *config_.Input - states_.Filter;
  } else {
    data_.Command = *config_.Input;
  }
}

/* control PID class methods */

/* method for configuring the PID block */
/* example JSON configuration:
{
  "Output": "OutputName",
  "Reference": "ReferenceName",
  "Feedback": "FeedbackName",
  "Gains": {
    "Proportional": Kp,
    "Integral": Ki,
    "Derivative": Kd,
  },
  "Time-Constant": X,
  "Limits": {
    "Upper": X,
    "Lower": X
  },
  "Sample-Time": "SampleTime"
}
Where OutputName gives a convenient name for the block (i.e. SpeedControl).
Reference is the full path name of the reference signal
Feedback is the full path name of the feedback signal
Gains are float values for the proportional, integral, and derivative gains
Time constant is the time constant for the first order derivative filter
Limits are optional and saturate the output if defined
Sample time is the full path name of the sample time signal in seconds
*/
// void ControlPIDClass::Configure(const rapidjson::Value& Config,std::string RootPath,DefinitionTree *DefinitionTreePtr) {
//
// }
//
// void ControlPIDClass::Run(Mode mode) {
//   // error
//   states_.Error = *config_.Reference-*config_.Feedback;
//   // derivative of Error
//   states_.DerivativeErrorState = 0.0f;
//   if (*config_.dt > 0.0 ) {
//       states_.DerivativeErrorState = 1.0f/(config_.Tf+*config_.dt/(states_.Error-states_.PreviousError));
//   }
//   states_.PreviousError = states_.Error;
//   switch (mode) {
//     // Zero the State and Command
//     case kReset: {
//       states_.Error = 0;
//       states_.PreviousError = 0;
//       data_.Command = 0;
//       break;
//     }
//     // Do Nothing, State and Command are unchanged
//     case kStandby: {
//       break;
//     }
//     // Run Commands
//     case kHold: {
//       CalculateCommand();
//       break;
//     }
//     // Initialize State then Run Commands
//     case kInitialize: {
//       InitializeState(0.0f);
//       CalculateCommand();
//       break;
//     }
//     // Update the State then Run Commands
//     case kEngage: {
//       UpdateState();
//       CalculateCommand();
//       break;
//     }
//   }
// }
//
// void ControlPIDClass::InitializeState(float Command) {
//   // Protect for Ki == 0
//   if (config_.Ki != 0.0f) {
//     IntegralErrorState_ = (Command-(config_.Kp*ProportionalError_+config_.Kd*DerivativeErrorState_))/config_.Ki;
//   } else {
//     IntegralErrorState_ = 0.0f;
//   }
// }
//
// // Update the state
// void ControlPIDClass::UpdateState() {
//   // Protect for unlimited windup when Ki == 0
//   if (config_.Ki != 0.0f) {
//     IntegralErrorState_ += (*config_.dt*IntegralError_);
//   } else {
//     IntegralErrorState_ = 0.0;
//   }
// }
//
// void ControlPIDClass::CalculateCommand() {
//   float ProportionalCommand = config_.Kp*ProportionalError_;
//   float IntegralCommand = config_.Ki*IntegralErrorState_;
//   float DerivativeCommand = config_.Kd*DerivativeErrorState_;
//   data_.Command = ProportionalCommand+IntegralCommand+DerivativeCommand;
//   // saturate cmd, set iErr to limit that produces saturated cmd
//   if (data_.Command <= config_.Limit[0]) {
//     data_.Command = config_.Limit[0];
//     // Re-compute the integrator state
//     InitializeState(data_.Command);
//   } else if (data_.Command >= config_.Limit[1]) {
//     data_.Command = config_.Limit[1];
//     // Re-compute the integrator state
//     InitializeState(data_.Command);
//   }
// }
//
// /* control PID2 class methods */
// void ControlPID2Class::Configure(const rapidjson::Value& Config,std::string RootPath,DefinitionTree *DefinitionTreePtr) {
//
// }
//
// void ControlPID2Class::Run(Mode mode) {
//   // error for proportional
//   ProportionalError_ = (config_.b*(*config_.Reference))-*config_.Feedback;
//   // error for integral
//   IntegralError_ = *config_.Reference-*config_.Feedback;
//   // error for derivative
//   DerivativeError_ = (config_.c*(*config_.Reference))-*config_.Feedback;
//   // derivative of Error
//   DerivativeErrorState_ = 0.0f;
//   if (*config_.dt > 0.0 ) {
//       DerivativeErrorState_ = 1.0f/(config_.Tf+*config_.dt/(DerivativeError_-PreviousDerivativeError_));
//   }
//   PreviousDerivativeError_ = DerivativeError_;
//   switch (mode) {
//     // Zero the State and Command
//     case kReset: {
//       ProportionalError_ = 0.0f;
//       DerivativeError_ = 0.0f;
//       PreviousDerivativeError_ = 0.0f;
//       IntegralError_ = 0.0f;
//       DerivativeErrorState_ = 0.0f;
//       IntegralErrorState_ = 0.0f;
//       data_.Command = 0;
//       break;
//     }
//     // Do Nothing, State and Command are unchanged
//     case kStandby: {
//       break;
//     }
//     // Run Commands
//     case kHold: {
//       CalculateCommand();
//       break;
//     }
//     // Initialize State then Run Commands
//     case kInitialize: {
//       InitializeState(0.0f);
//       CalculateCommand();
//       break;
//     }
//     // Update the State then Run Commands
//     case kEngage: {
//       UpdateState();
//       CalculateCommand();
//       break;
//     }
//   }
// }
//
// void ControlPID2Class::InitializeState(float Command) {
//   // Protect for Ki == 0
//   if (config_.Ki != 0.0f) {
//     IntegralErrorState_ = (Command-(config_.Kp*ProportionalError_+config_.Kd*DerivativeErrorState_))/config_.Ki;
//   } else {
//     IntegralErrorState_ = 0.0f;
//   }
// }
//
// // Update the state
// void ControlPID2Class::UpdateState() {
//   // Protect for unlimited windup when Ki == 0
//   if (config_.Ki != 0.0f) {
//     IntegralErrorState_ += (*config_.dt*IntegralError_);
//   } else {
//     IntegralErrorState_ = 0.0;
//   }
// }
//
// void ControlPID2Class::CalculateCommand() {
//   float ProportionalCommand = config_.Kp*ProportionalError_;
//   float IntegralCommand = config_.Ki*IntegralErrorState_;
//   float DerivativeCommand = config_.Kd*DerivativeErrorState_;
//   data_.Command = ProportionalCommand+IntegralCommand+DerivativeCommand;
//   // saturate cmd, set iErr to limit that produces saturated cmd
//   if (data_.Command <= config_.Limit[0]) {
//     data_.Command = config_.Limit[0];
//     // Re-compute the integrator state
//     InitializeState(data_.Command);
//   } else if (data_.Command >= config_.Limit[1]) {
//     data_.Command = config_.Limit[1];
//     // Re-compute the integrator state
//     InitializeState(data_.Command);
//   }
// }
//
// /* control state space class methods */
// void ControlStateSpaceClass::Configure(const rapidjson::Value& Config,std::string RootPath,DefinitionTree *DefinitionTreePtr) {
//
// }
//
// void ControlStateSpaceClass::Run(Mode mode) {
//
// }

/* configures control laws given a JSON value and registers data with global defs */
void ControlLaws::Configure(const rapidjson::Value& Config, DefinitionTree *DefinitionTreePtr) {
  // configuring baseline control laws
  if (Config.HasMember("Baseline")) {
    std::string PathName = RootPath_ + "/" + "Baseline";
    const rapidjson::Value& BaselineConfig = Config["Baseline"];
    BaselineControlGroup_.resize(BaselineConfig.Size());
    for (size_t i=0; i < BaselineConfig.Size(); i++) {
      if (BaselineConfig[i].HasMember("Level-Name")&&BaselineConfig[i].HasMember("Components")) {
        BaselineLevelNames_.push_back(BaselineConfig[i]["Level-Name"].GetString());
        for (size_t j=0; j < BaselineConfig[i]["Components"].Size(); j++) {
          const rapidjson::Value& Component = BaselineConfig[i]["Components"][j];
          if (Component.HasMember("Type")) {
            if (Component["Type"] == "Constant") {
              ControlConstantClass Temp;
              BaselineControlGroup_[i].push_back(std::make_shared<ControlConstantClass>(Temp));
              BaselineControlGroup_[i][j]->Configure(Component,PathName,DefinitionTreePtr);
            }
            if (Component["Type"] == "Gain") {
              ControlGainClass Temp;
              BaselineControlGroup_[i].push_back(std::make_shared<ControlGainClass>(Temp));
              BaselineControlGroup_[i][j]->Configure(Component,PathName,DefinitionTreePtr);
            }
            // if (Component["Type"] == "PID") {
            //   ControlPIDClass Temp;
            //   BaselineControlGroup_[i].push_back(std::make_shared<ControlPIDClass>(Temp));
            //   BaselineControlGroup_[i][j]->Configure(Component,PathName,DefinitionTreePtr);
            // }
            // if (Component["Type"] == "PID2") {
            //   ControlPID2Class Temp;
            //   BaselineControlGroup_[i].push_back(std::make_shared<ControlPID2Class>(Temp));
            //   BaselineControlGroup_[i][j]->Configure(Component,PathName,DefinitionTreePtr);
            // }
            // if (Component["Type"] == "State-Space") {
            //   ControlStateSpaceClass Temp;
            //   BaselineControlGroup_[i].push_back(std::make_shared<ControlStateSpaceClass>(Temp));
            //   BaselineControlGroup_[i][j]->Configure(Component,PathName,DefinitionTreePtr);
            // }
          } else {
            throw std::runtime_error(std::string("ERROR")+PathName+std::string(": Control type not specified in configuration."));
          }
        }
      } else {
        throw std::runtime_error(std::string("ERROR")+PathName+std::string(": Level name or components not specified in configuration."));
      }
    }
    // getting a list of all baseline keys and adding to superset of output keys
    std::vector<std::string> BaselineKeys;
    DefinitionTreePtr->GetKeys(PathName,&BaselineKeys);
    for (size_t i=0; i < BaselineKeys.size(); i++) {
      std::string MemberName = RootPath_ + BaselineKeys[i].erase(0,PathName.size());
      OutputKeys_[MemberName] = MemberName;
    }
  } else {
    throw std::runtime_error(std::string("ERROR")+RootPath_+std::string(": Baseline not specified in configuration."));
  }
  // configuring research sensor processing groups
  if (Config.HasMember("Research")) {
    const rapidjson::Value& ResearchConfig = Config["Research"];
    for (size_t i=0; i < ResearchConfig.Size(); i++) {
      if (ResearchConfig[i].HasMember("Group-Name")&&ResearchConfig[i].HasMember("Components")) {
        ResearchGroupKeys_.push_back(ResearchConfig[i]["Group-Name"].GetString());
        std::string PathName = RootPath_ + "/" + ResearchConfig[i]["Group-Name"].GetString();
        const rapidjson::Value& Components = ResearchConfig[i]["Components"];
        ResearchControlGroups_[ResearchConfig[i]["Group-Name"].GetString()].resize(Components.Size());
        for (size_t j=0; j < Components.Size(); j++) {
          if (Components[j].HasMember("Level-Name")&&Components[j].HasMember("Components")) {
            ResearchLevelNames_[ResearchConfig[i]["Group-Name"].GetString()].push_back(Components[j]["Level-Name"].GetString());
            for (size_t k=0; k < Components[j]["Components"].Size(); k++) {
              const rapidjson::Value& Component = Components[j]["Components"][k];
              if (Component.HasMember("Type")) {
                if (Component["Type"] == "Constant") {
                  ControlConstantClass Temp;
                  ResearchControlGroups_[ResearchConfig[i]["Group-Name"].GetString()][j].push_back(std::make_shared<ControlConstantClass>(Temp));
                  ResearchControlGroups_[ResearchConfig[i]["Group-Name"].GetString()][j][k]->Configure(Component,PathName,DefinitionTreePtr);
                }
                if (Component["Type"] == "Gain") {
                  ControlGainClass Temp;
                  ResearchControlGroups_[ResearchConfig[i]["Group-Name"].GetString()][j].push_back(std::make_shared<ControlGainClass>(Temp));
                  ResearchControlGroups_[ResearchConfig[i]["Group-Name"].GetString()][j][k]->Configure(Component,PathName,DefinitionTreePtr);
                }
                // if (Component["Type"] == "PID") {
                //   ControlPIDClass Temp;
                //   ResearchControlGroups_[ResearchConfig[i]["Group-Name"].GetString()][j].push_back(std::make_shared<ControlPIDClass>(Temp));
                //   ResearchControlGroups_[ResearchConfig[i]["Group-Name"].GetString()][j][k]->Configure(Component,PathName,DefinitionTreePtr);
                // }
                // if (Component["Type"] == "PID2") {
                //   ControlPID2Class Temp;
                //   ResearchControlGroups_[ResearchConfig[i]["Group-Name"].GetString()][j].push_back(std::make_shared<ControlPID2Class>(Temp));
                //   ResearchControlGroups_[ResearchConfig[i]["Group-Name"].GetString()][j][k]->Configure(Component,PathName,DefinitionTreePtr);
                // }
                // if (Component["Type"] == "State-Space") {
                //   ControlStateSpaceClass Temp;
                //   ResearchControlGroups_[ResearchConfig[i]["Group-Name"].GetString()][j].push_back(std::make_shared<ControlStateSpaceClass>(Temp));
                //   ResearchControlGroups_[ResearchConfig[i]["Group-Name"].GetString()][j][k]->Configure(Component,PathName,DefinitionTreePtr);
                // }
              } else {
                throw std::runtime_error(std::string("ERROR")+PathName+std::string(": Control type not specified in configuration."));
              }
            }
          } else {
            throw std::runtime_error(std::string("ERROR")+PathName+std::string(": Level name or components not specified in configuration."));
          }
        }
        // getting a list of all keys and adding to superset of output keys
        std::vector<std::string> ResearchKeys;
        DefinitionTreePtr->GetKeys(PathName,&ResearchKeys);
        for (size_t j=0; j < ResearchKeys.size(); j++) {
          std::string MemberName = RootPath_ + ResearchKeys[j].erase(0,PathName.size());
          OutputKeys_[MemberName] = MemberName;
        }
      } else {
        throw std::runtime_error(std::string("ERROR")+RootPath_+std::string(": Group name or components not specified in configuration."));
      }
    }
  }
  // resize vectors
  BaselineDataPtr_.resize(OutputKeys_.size());
  OutputData_.resize(OutputKeys_.size());
  // map baseline data pointers to superset of output keys
  std::string PathName = RootPath_ + "/" + "Baseline";
  std::vector<std::string> BaselineKeys;
  DefinitionTreePtr->GetKeys(PathName,&BaselineKeys);
  size_t i=0;
  for (auto const& element : OutputKeys_) {
    std::string MemberName;
    DefinitionTree::VariableDefinition TempDef;
    MemberName = RootPath_ + BaselineKeys[i].substr(PathName.size());
    if (MemberName == element.second) {
      DefinitionTreePtr->GetMember(BaselineKeys[i],&TempDef);
      BaselineDataPtr_[i] = TempDef.Value;
      if (DefinitionTreePtr->Size(MemberName)==0) {
        if (DefinitionTreePtr->GetValuePtr<uint64_t*>(BaselineKeys[i])) {
          OutputData_[i] = *(DefinitionTreePtr->GetValuePtr<uint64_t*>(BaselineKeys[i]));
          DefinitionTreePtr->InitMember(MemberName,std::get_if<uint64_t>(&OutputData_[i]),TempDef.Description,true,false);
        }
        if (DefinitionTreePtr->GetValuePtr<uint32_t*>(BaselineKeys[i])) {
          OutputData_[i] = *(DefinitionTreePtr->GetValuePtr<uint32_t*>(BaselineKeys[i]));
          DefinitionTreePtr->InitMember(MemberName,std::get_if<uint32_t>(&OutputData_[i]),TempDef.Description,true,false);
        }
        if (DefinitionTreePtr->GetValuePtr<uint16_t*>(BaselineKeys[i])) {
          OutputData_[i] = *(DefinitionTreePtr->GetValuePtr<uint16_t*>(BaselineKeys[i]));
          DefinitionTreePtr->InitMember(MemberName,std::get_if<uint16_t>(&OutputData_[i]),TempDef.Description,true,false);
        }
        if (DefinitionTreePtr->GetValuePtr<uint8_t*>(BaselineKeys[i])) {
          OutputData_[i] = *(DefinitionTreePtr->GetValuePtr<uint8_t*>(BaselineKeys[i]));
          DefinitionTreePtr->InitMember(MemberName,std::get_if<uint8_t>(&OutputData_[i]),TempDef.Description,true,false);
        }
        if (DefinitionTreePtr->GetValuePtr<int64_t*>(BaselineKeys[i])) {
          OutputData_[i] = *(DefinitionTreePtr->GetValuePtr<int64_t*>(BaselineKeys[i]));
          DefinitionTreePtr->InitMember(MemberName,std::get_if<int64_t>(&OutputData_[i]),TempDef.Description,true,false);
        }
        if (DefinitionTreePtr->GetValuePtr<int32_t*>(BaselineKeys[i])) {
          OutputData_[i] = *(DefinitionTreePtr->GetValuePtr<int32_t*>(BaselineKeys[i]));
          DefinitionTreePtr->InitMember(MemberName,std::get_if<int32_t>(&OutputData_[i]),TempDef.Description,true,false);
        }
        if (DefinitionTreePtr->GetValuePtr<int16_t*>(BaselineKeys[i])) {
          OutputData_[i] = *(DefinitionTreePtr->GetValuePtr<int16_t*>(BaselineKeys[i]));
          DefinitionTreePtr->InitMember(MemberName,std::get_if<int16_t>(&OutputData_[i]),TempDef.Description,true,false);
        }
        if (DefinitionTreePtr->GetValuePtr<int8_t*>(BaselineKeys[i])) {
          OutputData_[i] = *(DefinitionTreePtr->GetValuePtr<int8_t*>(BaselineKeys[i]));
          DefinitionTreePtr->InitMember(MemberName,std::get_if<int8_t>(&OutputData_[i]),TempDef.Description,true,false);
        }
        if (DefinitionTreePtr->GetValuePtr<float*>(BaselineKeys[i])) {
          OutputData_[i] = *(DefinitionTreePtr->GetValuePtr<float*>(BaselineKeys[i]));
          DefinitionTreePtr->InitMember(MemberName,std::get_if<float>(&OutputData_[i]),TempDef.Description,true,false);
        }
        if (DefinitionTreePtr->GetValuePtr<double*>(BaselineKeys[i])) {
          OutputData_[i] = *(DefinitionTreePtr->GetValuePtr<double*>(BaselineKeys[i]));
          DefinitionTreePtr->InitMember(MemberName,std::get_if<double>(&OutputData_[i]),TempDef.Description,true,false);
        }
      }
    }
    i++;
  }
  // map research data pointers to superset of output keys
  for (size_t i=0; i < ResearchGroupKeys_.size(); i++) {
    std::string PathName = RootPath_ + "/" + ResearchGroupKeys_[i];
    ResearchDataPtr_[ResearchGroupKeys_[i]].resize(OutputKeys_.size());
    std::vector<std::string> ResearchKeys;
    DefinitionTreePtr->GetKeys(PathName,&ResearchKeys);
    size_t j=0;
    for (auto const& element : OutputKeys_) {
      std::string MemberName;
      DefinitionTree::VariableDefinition TempDef;
      MemberName = RootPath_ + ResearchKeys[j].substr(PathName.size());
      if (MemberName == element.second) {
        DefinitionTreePtr->GetMember(ResearchKeys[j],&TempDef);
        ResearchDataPtr_[ResearchGroupKeys_[i]][j] = TempDef.Value;
        if (DefinitionTreePtr->Size(MemberName)==0) {
          if (DefinitionTreePtr->GetValuePtr<uint64_t*>(ResearchKeys[j])) {
            OutputData_[i] = *(DefinitionTreePtr->GetValuePtr<uint64_t*>(ResearchKeys[j]));
            DefinitionTreePtr->InitMember(MemberName,std::get_if<uint64_t>(&OutputData_[j]),TempDef.Description,true,false);
          }
          if (DefinitionTreePtr->GetValuePtr<uint32_t*>(ResearchKeys[j])) {
            OutputData_[i] = *(DefinitionTreePtr->GetValuePtr<uint32_t*>(ResearchKeys[j]));
            DefinitionTreePtr->InitMember(MemberName,std::get_if<uint32_t>(&OutputData_[j]),TempDef.Description,true,false);
          }
          if (DefinitionTreePtr->GetValuePtr<uint16_t*>(ResearchKeys[j])) {
            OutputData_[i] = *(DefinitionTreePtr->GetValuePtr<uint16_t*>(ResearchKeys[j]));
            DefinitionTreePtr->InitMember(MemberName,std::get_if<uint16_t>(&OutputData_[j]),TempDef.Description,true,false);
          }
          if (DefinitionTreePtr->GetValuePtr<uint8_t*>(ResearchKeys[j])) {
            OutputData_[i] = *(DefinitionTreePtr->GetValuePtr<uint8_t*>(ResearchKeys[j]));
            DefinitionTreePtr->InitMember(MemberName,std::get_if<uint8_t>(&OutputData_[j]),TempDef.Description,true,false);
          }
          if (DefinitionTreePtr->GetValuePtr<int64_t*>(ResearchKeys[j])) {
            OutputData_[i] = *(DefinitionTreePtr->GetValuePtr<int64_t*>(ResearchKeys[j]));
            DefinitionTreePtr->InitMember(MemberName,std::get_if<int64_t>(&OutputData_[j]),TempDef.Description,true,false);
          }
          if (DefinitionTreePtr->GetValuePtr<int32_t*>(ResearchKeys[j])) {
            OutputData_[i] = *(DefinitionTreePtr->GetValuePtr<int32_t*>(ResearchKeys[j]));
            DefinitionTreePtr->InitMember(MemberName,std::get_if<int32_t>(&OutputData_[j]),TempDef.Description,true,false);
          }
          if (DefinitionTreePtr->GetValuePtr<int16_t*>(ResearchKeys[j])) {
            OutputData_[i] = *(DefinitionTreePtr->GetValuePtr<int16_t*>(ResearchKeys[j]));
            DefinitionTreePtr->InitMember(MemberName,std::get_if<int16_t>(&OutputData_[j]),TempDef.Description,true,false);
          }
          if (DefinitionTreePtr->GetValuePtr<int8_t*>(ResearchKeys[j])) {
            OutputData_[i] = *(DefinitionTreePtr->GetValuePtr<int8_t*>(ResearchKeys[j]));
            DefinitionTreePtr->InitMember(MemberName,std::get_if<int8_t>(&OutputData_[j]),TempDef.Description,true,false);
          }
          if (DefinitionTreePtr->GetValuePtr<float*>(ResearchKeys[j])) {
            OutputData_[i] = *(DefinitionTreePtr->GetValuePtr<float*>(ResearchKeys[j]));
            DefinitionTreePtr->InitMember(MemberName,&(std::get<float>(OutputData_[j])),TempDef.Description,true,false);
          }
          if (DefinitionTreePtr->GetValuePtr<double*>(ResearchKeys[j])) {
            OutputData_[i] = *(DefinitionTreePtr->GetValuePtr<double*>(ResearchKeys[j]));
            DefinitionTreePtr->InitMember(MemberName,std::get_if<double>(&OutputData_[j]),TempDef.Description,true,false);
          }
        }
      }
      j++;
    }
  }
  // setting pointers to the output values so control laws can initialize transient free
  for (size_t i=0; i < BaselineControlGroup_.size(); i++) {
    for (size_t j=0; j < BaselineControlGroup_[i].size(); j++) {
      BaselineControlGroup_[i][j]->SetPreviousOutput(RootPath_,DefinitionTreePtr);
    }
  }
  for (size_t i=0; i < ResearchGroupKeys_.size(); i++) {
    for (size_t j=0; j < ResearchControlGroups_[ResearchGroupKeys_[i]].size(); j++) {
      for (size_t k=0; k < ResearchControlGroups_[ResearchGroupKeys_[i]][j].size(); k++) {
        ResearchControlGroups_[ResearchGroupKeys_[i]][j][k]->SetPreviousOutput(RootPath_,DefinitionTreePtr);
      }
    }
  }
  Configured_ = true;
}

/* returns whether control has been configured */
bool ControlLaws::Configured() {
  return Configured_;
}

/* sets the control law that is engaged and currently output */
void ControlLaws::SetEngagedController(std::string ControlGroupName) {
  EngagedGroup_ = ControlGroupName;
}

/* sets the control law that is running and computing states to enable a transient free engage */
void ControlLaws::SetArmedController(std::string ControlGroupName) {
  ArmedGroup_ = ControlGroupName;
}

/* returns the number of levels for the engaged control law */
size_t ControlLaws::ActiveControlLevels() {
  if (EngagedGroup_ == "Baseline") {
    return BaselineControlGroup_.size();
  } else {
    return ResearchControlGroups_[EngagedGroup_].size();
  }
}

/* returns the name of the level for the engaged control law */
std::string ControlLaws::GetActiveLevel(size_t ControlLevel) {
  if (EngagedGroup_ == "Baseline") {
    return BaselineLevelNames_[ControlLevel];
  } else {
    return ResearchLevelNames_[EngagedGroup_][ControlLevel];
  }
}

/* computes control law data */
void ControlLaws::Run(size_t ControlLevel) {
  if (EngagedGroup_ == "Baseline") {
    // running baseline control laws as engaged
    for (size_t i=0; i < BaselineControlGroup_[ControlLevel].size(); i++) {
      BaselineControlGroup_[ControlLevel][i]->Run(ControlFunctionClass::kEngage);
    }
    // run armed research control laws
    if (ArmedGroup_ != "Baseline") {
      for (size_t i=0; i < ResearchControlGroups_[ArmedGroup_][ControlLevel].size(); i++) {
        ResearchControlGroups_[ArmedGroup_][ControlLevel][i]->Run(ControlFunctionClass::kInitialize);
      }
    }
    // output baseline control laws
    for (size_t i=0; i < OutputData_.size(); i++) {
      if (std::get_if<uint64_t*>(&BaselineDataPtr_[i])) {
        OutputData_[i] = **(std::get_if<uint64_t*>(&BaselineDataPtr_[i]));
      }
      if (std::get_if<uint32_t*>(&BaselineDataPtr_[i])) {
        OutputData_[i] = **(std::get_if<uint32_t*>(&BaselineDataPtr_[i]));
      }
      if (std::get_if<uint16_t*>(&BaselineDataPtr_[i])) {
        OutputData_[i] = **(std::get_if<uint16_t*>(&BaselineDataPtr_[i]));
      }
      if (std::get_if<uint8_t*>(&BaselineDataPtr_[i])) {
        OutputData_[i] = **(std::get_if<uint8_t*>(&BaselineDataPtr_[i]));
      }
      if (std::get_if<int64_t*>(&BaselineDataPtr_[i])) {
        OutputData_[i] = **(std::get_if<int64_t*>(&BaselineDataPtr_[i]));
      }
      if (std::get_if<int32_t*>(&BaselineDataPtr_[i])) {
        OutputData_[i] = **(std::get_if<int32_t*>(&BaselineDataPtr_[i]));
      }
      if (std::get_if<int16_t*>(&BaselineDataPtr_[i])) {
        OutputData_[i] = **(std::get_if<int16_t*>(&BaselineDataPtr_[i]));
      }
      if (std::get_if<int8_t*>(&BaselineDataPtr_[i])) {
        OutputData_[i] = **(std::get_if<int8_t*>(&BaselineDataPtr_[i]));
      }
      if (std::get_if<float*>(&BaselineDataPtr_[i])) {
        OutputData_[i] = **(std::get_if<float*>(&BaselineDataPtr_[i]));
      }
      if (std::get_if<double*>(&BaselineDataPtr_[i])) {
        OutputData_[i] = **(std::get_if<double*>(&BaselineDataPtr_[i]));
      }
    }
  } else {
    // running engaged research control laws
    for (size_t i=0; i < ResearchControlGroups_[EngagedGroup_][ControlLevel].size(); i++) {
      ResearchControlGroups_[EngagedGroup_][ControlLevel][i]->Run(ControlFunctionClass::kEngage);
    }
    // running armed research control laws
    if (ArmedGroup_ != "Baseline") {
      for (size_t i=0; i < ResearchControlGroups_[ArmedGroup_][ControlLevel].size(); i++) {
        ResearchControlGroups_[ArmedGroup_][ControlLevel][i]->Run(ControlFunctionClass::kInitialize);
      }
    }
    // running baseline control laws as armed
    for (size_t i=0; i < BaselineControlGroup_[ControlLevel].size(); i++) {
      BaselineControlGroup_[ControlLevel][i]->Run(ControlFunctionClass::kInitialize);
    }
    // output research control laws
    std::vector<std::variant<uint64_t*,uint32_t*,uint16_t*,uint8_t*,int64_t*,int32_t*,int16_t*,int8_t*,float*,double*>> ResearchPtr;
    ResearchPtr = ResearchDataPtr_[EngagedGroup_];
    for (size_t i=0; i < OutputData_.size(); i++) {
      if (std::get_if<uint64_t*>(&ResearchPtr[i])) {
        OutputData_[i] = **(std::get_if<uint64_t*>(&ResearchPtr[i]));
      }
      if (std::get_if<uint32_t*>(&ResearchPtr[i])) {
        OutputData_[i] = **(std::get_if<uint32_t*>(&ResearchPtr[i]));
      }
      if (std::get_if<uint16_t*>(&ResearchPtr[i])) {
        OutputData_[i] = **(std::get_if<uint16_t*>(&ResearchPtr[i]));
      }
      if (std::get_if<uint8_t*>(&ResearchPtr[i])) {
        OutputData_[i] = **(std::get_if<uint8_t*>(&ResearchPtr[i]));
      }
      if (std::get_if<int64_t*>(&ResearchPtr[i])) {
        OutputData_[i] = **(std::get_if<int64_t*>(&ResearchPtr[i]));
      }
      if (std::get_if<int32_t*>(&ResearchPtr[i])) {
        OutputData_[i] = **(std::get_if<int32_t*>(&ResearchPtr[i]));
      }
      if (std::get_if<int16_t*>(&ResearchPtr[i])) {
        OutputData_[i] = **(std::get_if<int16_t*>(&ResearchPtr[i]));
      }
      if (std::get_if<int8_t*>(&ResearchPtr[i])) {
        OutputData_[i] = **(std::get_if<int8_t*>(&ResearchPtr[i]));
      }
      if (std::get_if<float*>(&ResearchPtr[i])) {
        OutputData_[i] = **(std::get_if<float*>(&ResearchPtr[i]));
      }
      if (std::get_if<double*>(&ResearchPtr[i])) {
        OutputData_[i] = **(std::get_if<double*>(&ResearchPtr[i]));
      }
    }
  }
}
