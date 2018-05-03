/*
excitation.cc
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

#include "excitation.hxx"

void ExcitationFunctionClass::Configure(const rapidjson::Value& Config,std::string RootPath,DefinitionTree *DefinitionTreePtr) {}
void ExcitationFunctionClass::Run(Mode mode) {}

void Pulse::Configure(const rapidjson::Value& Config,std::string RootPath,DefinitionTree *DefinitionTreePtr) {
  std::string SignalName;
  std::string OutputName;
  if (Config.HasMember("Signal")) {
    SignalName = Config["Signal"].GetString();
    OutputName = RootPath + SignalName.substr(SignalName.rfind("/"));
    // pointer to log run mode data
    DefinitionTreePtr->InitMember(OutputName+"/Mode",&data_.Mode,"Run mode",true,false);
    // pointer to log excitation data
    DefinitionTreePtr->InitMember(OutputName+"/Excitation",&data_.Excitation,"Excitation system output",true,false);
    if (DefinitionTreePtr->GetValuePtr<float*>(Config["Signal"].GetString())) {
      config_.Signal = DefinitionTreePtr->GetValuePtr<float*>(Config["Signal"].GetString());
    } else {
      throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Signal not found in global data."));
    }
  } else {
    throw std::runtime_error(std::string("ERROR")+RootPath+std::string(": Signal not specified in configuration."));
  }
  if (Config.HasMember("Start-Time")) {
    config_.StartTime_s = Config["Start-Time"].GetFloat();
  } else {
    throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Start time not specified in configuration."));
  }
  if (Config.HasMember("Duration")) {
    config_.Duration_s = Config["Duration"].GetFloat();
  } else {
    throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Duration not specified in configuration."));
  }
  if (Config.HasMember("Amplitude")) {
    config_.Amplitude = Config["Amplitude"].GetFloat();
  } else {
    throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Amplitude not specified in configuration."));
  }
}

void Pulse::Run(Mode mode) {
  if (mode == kEngage) {
    // initialize the time when first called
    if (!TimeLatch) {
      Time_us = 0;
      TimeLatch = true;
    }
    // pulse logic
    if (Time_us < (config_.StartTime_s)*1e6){
      // do nothing
      data_.Excitation = 0;
    } else if (Time_us < (config_.StartTime_s+config_.Duration_s)*1e6) {
      // add the pulse to the signal
      data_.Excitation = config_.Amplitude;
    } else {
      // do nothing
      data_.Excitation = 0;
    }
  } else {
    // reset the time latch
    TimeLatch = false;
    // do nothing
    data_.Excitation = 0;
  }
  data_.Mode = (uint8_t)mode;
  *config_.Signal = *config_.Signal + data_.Excitation;
}

void Doublet::Configure(const rapidjson::Value& Config,std::string RootPath,DefinitionTree *DefinitionTreePtr) {
  std::string SignalName;
  std::string OutputName;
  if (Config.HasMember("Signal")) {
    SignalName = Config["Signal"].GetString();
    OutputName = RootPath + SignalName.substr(SignalName.rfind("/"));
    // pointer to log run mode data
    DefinitionTreePtr->InitMember(OutputName+"/Mode",&data_.Mode,"Run mode",true,false);
    // pointer to log excitation data
    DefinitionTreePtr->InitMember(OutputName+"/Excitation",&data_.Excitation,"Excitation system output",true,false);
    if (DefinitionTreePtr->GetValuePtr<float*>(Config["Signal"].GetString())) {
      config_.Signal = DefinitionTreePtr->GetValuePtr<float*>(Config["Signal"].GetString());
    } else {
      throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Signal not found in global data."));
    }
  } else {
    throw std::runtime_error(std::string("ERROR")+RootPath+std::string(": Signal not specified in configuration."));
  }
  if (Config.HasMember("Start-Time")) {
    config_.StartTime_s = Config["Start-Time"].GetFloat();
  } else {
    throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Start time not specified in configuration."));
  }
  if (Config.HasMember("Duration")) {
    config_.Duration_s = Config["Duration"].GetFloat();
  } else {
    throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Duration not specified in configuration."));
  }
  if (Config.HasMember("Amplitude")) {
    config_.Amplitude = Config["Amplitude"].GetFloat();
  } else {
    throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Amplitude not specified in configuration."));
  }
}

void Doublet::Run(Mode mode) {
  if (mode == kEngage) {
    // initialize the time when first called
    if (!TimeLatch) {
      Time_us = 0;
      TimeLatch = true;
    }
    // doublet logic
    if (Time_us < (config_.StartTime_s)*1e6){
      // do nothing
      data_.Excitation = 0;
    } else if (Time_us < (config_.StartTime_s+config_.Duration_s)*1e6) {
      // add the doublet to the signal
      data_.Excitation = config_.Amplitude;
    } else if (Time_us < (config_.StartTime_s+2.0f*config_.Duration_s)*1e6) {
      // add the doublet to the signal
      data_.Excitation = -1*config_.Amplitude;
    } else {
      // do nothing
      data_.Excitation = 0;
    }
  } else {
    // reset the time latch
    TimeLatch = false;
    // do nothing
    data_.Excitation = 0;
  }
  data_.Mode = (uint8_t)mode;
  *config_.Signal = *config_.Signal + data_.Excitation;
}

void Doublet121::Configure(const rapidjson::Value& Config,std::string RootPath,DefinitionTree *DefinitionTreePtr) {
  std::string SignalName;
  std::string OutputName;
  if (Config.HasMember("Signal")) {
    SignalName = Config["Signal"].GetString();
    OutputName = RootPath + SignalName.substr(SignalName.rfind("/"));
    // pointer to log run mode data
    DefinitionTreePtr->InitMember(OutputName+"/Mode",&data_.Mode,"Run mode",true,false);
    // pointer to log excitation data
    DefinitionTreePtr->InitMember(OutputName+"/Excitation",&data_.Excitation,"Excitation system output",true,false);
    if (DefinitionTreePtr->GetValuePtr<float*>(Config["Signal"].GetString())) {
      config_.Signal = DefinitionTreePtr->GetValuePtr<float*>(Config["Signal"].GetString());
    } else {
      throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Signal not found in global data."));
    }
  } else {
    throw std::runtime_error(std::string("ERROR")+RootPath+std::string(": Signal not specified in configuration."));
  }
  if (Config.HasMember("Start-Time")) {
    config_.StartTime_s = Config["Start-Time"].GetFloat();
  } else {
    throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Start time not specified in configuration."));
  }
  if (Config.HasMember("Duration")) {
    config_.Duration_s = Config["Duration"].GetFloat();
  } else {
    throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Duration not specified in configuration."));
  }
  if (Config.HasMember("Amplitude")) {
    config_.Amplitude = Config["Amplitude"].GetFloat();
  } else {
    throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Amplitude not specified in configuration."));
  }
}
void Doublet121::Run(Mode mode) {
  if (mode == kEngage) {
    // initialize the time when first called
    if (!TimeLatch) {
      Time_us = 0;
      TimeLatch = true;
    }
    // doublet logic, 1-2-1
    if (Time_us < (config_.StartTime_s)*1e6){
      // do nothing
      data_.Excitation = 0;
    } else if (Time_us < (config_.StartTime_s+config_.Duration_s)*1e6) {
      // add the doublet to the signal
      data_.Excitation = config_.Amplitude;
    } else if (Time_us < (config_.StartTime_s+3.0f*config_.Duration_s)*1e6) {
      // add the doublet to the signal
      data_.Excitation = -1*config_.Amplitude;
    } else if (Time_us < (config_.StartTime_s+4.0f*config_.Duration_s)*1e6) {
      // add the doublet to the signal
      data_.Excitation = config_.Amplitude;
    } else {
      // do nothing
      data_.Excitation = 0;
    }
  } else {
    // reset the time latch
    TimeLatch = false;
    // do nothing
    data_.Excitation = 0;
  }
  data_.Mode = (uint8_t)mode;
  *config_.Signal = *config_.Signal + data_.Excitation;
}

void Doublet3211::Configure(const rapidjson::Value& Config,std::string RootPath,DefinitionTree *DefinitionTreePtr) {
  std::string SignalName;
  std::string OutputName;
  if (Config.HasMember("Signal")) {
    SignalName = Config["Signal"].GetString();
    OutputName = RootPath + SignalName.substr(SignalName.rfind("/"));
    // pointer to log run mode data
    DefinitionTreePtr->InitMember(OutputName+"/Mode",&data_.Mode,"Run mode",true,false);
    // pointer to log excitation data
    DefinitionTreePtr->InitMember(OutputName+"/Excitation",&data_.Excitation,"Excitation system output",true,false);
    if (DefinitionTreePtr->GetValuePtr<float*>(Config["Signal"].GetString())) {
      config_.Signal = DefinitionTreePtr->GetValuePtr<float*>(Config["Signal"].GetString());
    } else {
      throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Signal not found in global data."));
    }
  } else {
    throw std::runtime_error(std::string("ERROR")+RootPath+std::string(": Signal not specified in configuration."));
  }
  if (Config.HasMember("Start-Time")) {
    config_.StartTime_s = Config["Start-Time"].GetFloat();
  } else {
    throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Start time not specified in configuration."));
  }
  if (Config.HasMember("Duration")) {
    config_.Duration_s = Config["Duration"].GetFloat();
  } else {
    throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Duration not specified in configuration."));
  }
  if (Config.HasMember("Amplitude")) {
    config_.Amplitude = Config["Amplitude"].GetFloat();
  } else {
    throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Amplitude not specified in configuration."));
  }
}
void Doublet3211::Run(Mode mode) {
  if (mode == kEngage) {
    // initialize the time when first called
    if (!TimeLatch) {
      Time_us = 0;
      TimeLatch = true;
    }
    // doublet logic, 3-2-1-1
    if (Time_us < (config_.StartTime_s)*1e6){
      // do nothing
      data_.Excitation = 0;
    } else if (Time_us < (config_.StartTime_s+3.0f*config_.Duration_s)*1e6) {
      // add the doublet to the signal
      data_.Excitation = config_.Amplitude;
    } else if (Time_us < (config_.StartTime_s+5.0f*config_.Duration_s)*1e6) {
      // add the doublet to the signal
      data_.Excitation = -1*config_.Amplitude;
    } else if (Time_us < (config_.StartTime_s+6.0f*config_.Duration_s)*1e6) {
      // add the doublet to the signal
      data_.Excitation = config_.Amplitude;
    } else if (Time_us < (config_.StartTime_s+7.0f*config_.Duration_s)*1e6) {
      // add the doublet to the signal
      data_.Excitation = -1*config_.Amplitude;
    } else {
      // do nothing
      data_.Excitation = 0;
    }
  } else {
    // reset the time latch
    TimeLatch = false;
    // do nothing
    data_.Excitation = 0;
  }
  data_.Mode = (uint8_t)mode;
  *config_.Signal = *config_.Signal + data_.Excitation;
}

void LinearChirp::Configure(const rapidjson::Value& Config,std::string RootPath,DefinitionTree *DefinitionTreePtr) {
  std::string SignalName;
  std::string OutputName;
  if (Config.HasMember("Signal")) {
    SignalName = Config["Signal"].GetString();
    OutputName = RootPath + SignalName.substr(SignalName.rfind("/"));
    // pointer to log run mode data
    DefinitionTreePtr->InitMember(OutputName+"/Mode",&data_.Mode,"Run mode",true,false);
    // pointer to log excitation data
    DefinitionTreePtr->InitMember(OutputName+"/Excitation",&data_.Excitation,"Excitation system output",true,false);
    if (DefinitionTreePtr->GetValuePtr<float*>(Config["Signal"].GetString())) {
      config_.Signal = DefinitionTreePtr->GetValuePtr<float*>(Config["Signal"].GetString());
    } else {
      throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Signal not found in global data."));
    }
  } else {
    throw std::runtime_error(std::string("ERROR")+RootPath+std::string(": Signal not specified in configuration."));
  }
  if (Config.HasMember("Start-Time")) {
    config_.StartTime_s = Config["Start-Time"].GetFloat();
  } else {
    throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Start time not specified in configuration."));
  }
  if (Config.HasMember("Duration")) {
    config_.Duration_s = Config["Duration"].GetFloat();
  } else {
    throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Duration not specified in configuration."));
  }
  if (Config.HasMember("Amplitude")) {
    if (Config["Amplitude"].Size() != 2) {
      throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Amplitude size incorrect; should be two [min,max]"));
    } else {
      config_.Amplitude[0] = Config["Amplitude"][0].GetFloat();
      config_.Amplitude[1] = Config["Amplitude"][1].GetFloat();
    }
  } else {
    throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Amplitude not specified in configuration."));
  }
  if (Config.HasMember("Frequency")) {
    if (Config["Frequency"].Size() != 2) {
      throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Frequency size incorrect; should be two [min,max]"));
    } else {
      config_.Frequency[0] = Config["Frequency"][0].GetFloat();
      config_.Frequency[1] = Config["Frequency"][1].GetFloat();
    }
  } else {
    throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Frequency not specified in configuration."));
  }
}

void LinearChirp::Run(Mode mode) {
  if (mode == kEngage) {
    // initialize the time when first called
    if (!TimeLatch) {
      Time_us = 0;
      TimeLatch = true;
    }
    // chirp logic
    if (Time_us < (config_.StartTime_s)*1e6){
      // do nothing
      data_.Excitation = 0;
    } else if (Time_us < config_.Duration_s*1e6) {
      // linear varying instantanious frequency
      float freq_rps = config_.Frequency[0]+(config_.Frequency[1]-config_.Frequency[0])/(2.0f*config_.Duration_s*1e6)*Time_us;
      // linear varying amplitude
      float amp_nd = config_.Amplitude[0]+(config_.Amplitude[1]-config_.Amplitude[0])*Time_us/(config_.Duration_s*1e6);
      // chirp Equation
      data_.Excitation = amp_nd*sinf(freq_rps*Time_us);
    } else {
      // do nothing
      data_.Excitation = 0;
    }
  } else {
    // reset the time latch
    TimeLatch = false;
    // do nothing
    data_.Excitation = 0;
  }
  data_.Mode = (uint8_t)mode;
  *config_.Signal = *config_.Signal + data_.Excitation;
}

void MultiSine::Configure(const rapidjson::Value& Config,std::string RootPath,DefinitionTree *DefinitionTreePtr) {
  std::string SignalName;
  std::string OutputName;
  if (Config.HasMember("Signal")) {
    SignalName = Config["Signal"].GetString();
    OutputName = RootPath + SignalName.substr(SignalName.rfind("/"));
    // pointer to log run mode data
    DefinitionTreePtr->InitMember(OutputName+"/Mode",&data_.Mode,"Run mode",true,false);
    // pointer to log excitation data
    DefinitionTreePtr->InitMember(OutputName+"/Excitation",&data_.Excitation,"Excitation system output",true,false);
    if (DefinitionTreePtr->GetValuePtr<float*>(Config["Signal"].GetString())) {
      config_.Signal = DefinitionTreePtr->GetValuePtr<float*>(Config["Signal"].GetString());
    } else {
      throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Signal not found in global data."));
    }
  } else {
    throw std::runtime_error(std::string("ERROR")+RootPath+std::string(": Signal not specified in configuration."));
  }
  if (Config.HasMember("Start-Time")) {
    config_.StartTime_s = Config["Start-Time"].GetFloat();
  } else {
    throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Start time not specified in configuration."));
  }
  if (Config.HasMember("Duration")) {
    config_.Duration_s = Config["Duration"].GetFloat();
  } else {
    throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Duration not specified in configuration."));
  }
  if (Config.HasMember("Amplitude")) {
    config_.Amplitude.resize(Config["Amplitude"].Size(),1);
    for (size_t i=0; i < Config["Amplitude"].Size(); i++) {
      config_.Amplitude(i,0) = Config["Amplitude"][i].GetFloat();
    }
  } else {
    throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Amplitude not specified in configuration."));
  }
  if (Config.HasMember("Frequency")) {
    config_.Frequency.resize(Config["Frequency"].Size(),1);
    for (size_t i=0; i < Config["Frequency"].Size(); i++) {
      config_.Frequency(i,0) = Config["Frequency"][i].GetFloat();
    }
  } else {
    throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Frequency not specified in configuration."));
  }
  if (Config.HasMember("Phase")) {
    config_.Phase.resize(Config["Phase"].Size(),1);
    for (size_t i=0; i < Config["Phase"].Size(); i++) {
      config_.Phase(i,0) = Config["Phase"][i].GetFloat();
    }
  } else {
    throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Phase not specified in configuration."));
  }
  if ((Config["Amplitude"].Size() != Config["Frequency"].Size())||(Config["Amplitude"].Size() != Config["Phase"].Size())) {
    throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Amplitude, frequency, and phase arrays are not the same length."));
  }
}

void MultiSine::Run(Mode mode) {
  if (mode == kEngage) {
    // initialize the time when first called
    if (!TimeLatch) {
      Time_us = 0;
      TimeLatch = true;
    }
    // multisine logic
    if (Time_us < (config_.StartTime_s)*1e6){
      // do nothing
      data_.Excitation = 0;
    } else if (Time_us < config_.Duration_s*1e6) {
      // Scale the waveform to preserve unity
      float scale = sqrtf(1.0f/((float)config_.Amplitude.size()));
      // Compute the Waveform - scale * sum(amp .* cos(freq * t + phase))
      data_.Excitation=scale*(config_.Amplitude*(config_.Frequency*Time_us+config_.Phase).cos()).sum();
    } else {
      // do nothing
      data_.Excitation = 0;
    }
  } else {
    // reset the time latch
    TimeLatch = false;
    // do nothing
    data_.Excitation = 0;
  }
  data_.Mode = (uint8_t)mode;
  *config_.Signal = *config_.Signal + data_.Excitation;
}

/* configures excitation system given a JSON value and registers data with global defs */
void ExcitationSystem::Configure(const rapidjson::Value& Config, DefinitionTree *DefinitionTreePtr) {
  ExcitationGroups_.resize(Config.Size());
  for (size_t i=0; i < Config.Size(); i++) {
    if (Config[i].HasMember("Group-Name")&&Config[i].HasMember("Components")) {
      ExcitationGroupKeys_.push_back(Config[i]["Group-Name"].GetString());
      std::string PathName = RootPath_ + "/" + Config[i]["Group-Name"].GetString();
      const rapidjson::Value& Components = Config[i]["Components"];
      ExcitationGroups_[i].resize(Components.Size());
      for (size_t j=0; j < Components.Size(); j++) {
        if (Components[j].HasMember("Level-Name")&&Components[j].HasMember("Components")) {
          ExcitationGroupLevels_.push_back(Components[j]["Level-Name"].GetString());
          for (size_t k=0; k < Components[j]["Components"].Size(); k++) {
            const rapidjson::Value& Component = Components[j]["Components"][k];
            if (Component.HasMember("Type")) {
              if (Component["Type"] == "Pulse") {
                Pulse Temp;
                ExcitationGroups_[i][j].push_back(std::make_shared<Pulse>(Temp));
                ExcitationGroups_[i][j][k]->Configure(Component,PathName,DefinitionTreePtr);
              }
              if (Component["Type"] == "Doublet") {
                Doublet Temp;
                ExcitationGroups_[i][j].push_back(std::make_shared<Doublet>(Temp));
                ExcitationGroups_[i][j][k]->Configure(Component,PathName,DefinitionTreePtr);
              }
              if (Component["Type"] == "Doublet121") {
                Doublet121 Temp;
                ExcitationGroups_[i][j].push_back(std::make_shared<Doublet121>(Temp));
                ExcitationGroups_[i][j][k]->Configure(Component,PathName,DefinitionTreePtr);
              }
              if (Component["Type"] == "Doublet3211") {
                Doublet3211 Temp;
                ExcitationGroups_[i][j].push_back(std::make_shared<Doublet3211>(Temp));
                ExcitationGroups_[i][j][k]->Configure(Component,PathName,DefinitionTreePtr);
              }
              if (Component["Type"] == "Linear-Chirp") {
                LinearChirp Temp;
                ExcitationGroups_[i][j].push_back(std::make_shared<LinearChirp>(Temp));
                ExcitationGroups_[i][j][k]->Configure(Component,PathName,DefinitionTreePtr);
              }
              if (Component["Type"] == "MultiSine") {
                MultiSine Temp;
                ExcitationGroups_[i][j].push_back(std::make_shared<MultiSine>(Temp));
                ExcitationGroups_[i][j][k]->Configure(Component,PathName,DefinitionTreePtr);
              }
            } else {
              throw std::runtime_error(std::string("ERROR")+PathName+std::string(": Excitation type not specified in configuration."));
            }
          }
        } else {
          throw std::runtime_error(std::string("ERROR")+PathName+std::string(": Level name or components not specified in configuration."));
        }
      }
    } else {
      throw std::runtime_error(std::string("ERROR")+RootPath_+std::string(": Group name or components not specified in configuration."));
    }
  }
  Configured_ = true;
}

/* return whether the excitation system has been configured */
bool ExcitationSystem::Configured() {
  return Configured_;
}

/* sets the engaged excitation group */
void ExcitationSystem::SetEngagedExcitation(std::string ExcitationGroupName) {
  EngagedGroup_ = ExcitationGroupName;
}

/* run all excitation functions at a given control level */
void ExcitationSystem::Run(std::string ControlLevel) {
  for (size_t i=0; i < ExcitationGroupKeys_.size(); i++) {
    for (size_t j=0; j < ExcitationGroupLevels_.size(); j++) {
      if ((ExcitationGroupKeys_[i] == EngagedGroup_)&&(ExcitationGroupLevels_[j] == ControlLevel)) {
        for (size_t k=0; k < ExcitationGroups_[i][j].size(); k++) {
          ExcitationGroups_[i][j][k]->Run(ExcitationFunctionClass::kEngage);
        }
      } else {
        for (size_t k=0; k < ExcitationGroups_[i][j].size(); k++) {
          ExcitationGroups_[i][j][k]->Run(ExcitationFunctionClass::kArm);
        }
      }
    }
  }
}
