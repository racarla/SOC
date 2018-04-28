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
bool ControlFunctionClass::Initialized() {}
void ControlFunctionClass::Run(Mode mode) {}

/* control empty class methods */
void ControlEmptyClass::Configure(const rapidjson::Value& Config,std::string RootPath,DefinitionTree *DefinitionTreePtr) {}
bool ControlEmptyClass::Initialized() {}
void ControlEmptyClass::Run(Mode mode) {}

/* control constant class methods */
void ControlConstantClass::Configure(const rapidjson::Value& Config,std::string RootPath,DefinitionTree *DefinitionTreePtr) {
  std::string OutputName;
  if (Config.HasMember("Output-Name")) {
    OutputName = RootPath + "/" + Config["Output-Name"].GetString();
  } else {
    throw std::runtime_error(std::string("ERROR")+RootPath+std::string(": Output-Name not specified in configuration."));
  }
  if (Config.HasMember("Constant")) {
    config_.Constant = Config["Constant"].GetFloat();
  } else {
    throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Constant value not specified in configuration."));
  }
  // pointer to log run mode data
  DefinitionTreePtr->InitMember(OutputName+"/Mode",&data_.Mode,"Run mode",true,false);
  // pointer to log command data
  DefinitionTreePtr->InitMember(OutputName+"/Command",&data_.Command,"Control law output",true,false);
}
bool ControlConstantClass::Initialized() {
  return true;
}
void ControlConstantClass::Run(Mode mode) {
  data_.Mode = (uint8_t) mode;
  data_.Command = config_.Constant;
}

/* control gain class methods */
void ControlGainClass::Configure(const rapidjson::Value& Config,std::string RootPath,DefinitionTree *DefinitionTreePtr) {}
bool ControlGainClass::Initialized() {}
void ControlGainClass::Run(Mode mode) {}

/* control PID class methods */
void ControlPIDClass::Configure(const rapidjson::Value& Config,std::string RootPath,DefinitionTree *DefinitionTreePtr) {}
bool ControlPIDClass::Initialized() {}
void ControlPIDClass::Run(Mode mode) {}

/* control PID2 class methods */
void ControlPID2Class::Configure(const rapidjson::Value& Config,std::string RootPath,DefinitionTree *DefinitionTreePtr) {}
bool ControlPID2Class::Initialized() {}
void ControlPID2Class::Run(Mode mode) {}

/* control state space class methods */
void ControlStateSpaceClass::Configure(const rapidjson::Value& Config,std::string RootPath,DefinitionTree *DefinitionTreePtr) {}
bool ControlStateSpaceClass::Initialized() {}
void ControlStateSpaceClass::Run(Mode mode) {}

/* configures control laws given a JSON value and registers data with global defs */
void ControlLaws::Configure(const rapidjson::Value& Config, DefinitionTree *DefinitionTreePtr) {
  // configuring baseline control laws
  if (Config.HasMember("Baseline")) {
    std::string PathName = RootPath_ + "/" + "Baseline";
    const rapidjson::Value& BaselineConfig = Config["Baseline"];
    BaselineControlGroup_.resize(BaselineConfig.Size());
    for (size_t i=0; i < BaselineConfig.Size(); i++) {
      if (BaselineConfig[i].HasMember("Level-Name")&&BaselineConfig[i].HasMember("Components")) {
        for (size_t j=0; j < BaselineConfig[i]["Components"].Size(); j++) {
          const rapidjson::Value& Component = BaselineConfig[i]["Components"][j];
          if (Component.HasMember("Type")) {
            if (Component["Type"] == "Empty") {
              ControlEmptyClass Temp;
              Temp.Configure(Component,PathName,DefinitionTreePtr);
              BaselineControlGroup_[i].push_back(Temp);
            }
            if (Component["Type"] == "Constant") {
              ControlConstantClass Temp;
              Temp.Configure(Component,PathName,DefinitionTreePtr);
              BaselineControlGroup_[i].push_back(Temp);
            }
            if (Component["Type"] == "Gain") {
              ControlGainClass Temp;
              Temp.Configure(Component,PathName,DefinitionTreePtr);
              BaselineControlGroup_[i].push_back(Temp);
            }
            if (Component["Type"] == "PID") {
              ControlPIDClass Temp;
              Temp.Configure(Component,PathName,DefinitionTreePtr);
              BaselineControlGroup_[i].push_back(Temp);
            }
            if (Component["Type"] == "PID2") {
              ControlPID2Class Temp;
              Temp.Configure(Component,PathName,DefinitionTreePtr);
              BaselineControlGroup_[i].push_back(Temp);
            }
            if (Component["Type"] == "State-Space") {
              ControlStateSpaceClass Temp;
              Temp.Configure(Component,PathName,DefinitionTreePtr);
              BaselineControlGroup_[i].push_back(Temp);
            }
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
            for (size_t k=0; k < Components[j]["Components"].Size(); k++) {
              const rapidjson::Value& Component = Components[j]["Components"][k];
              if (Component.HasMember("Type")) {
                if (Component["Type"] == "Empty") {
                  ControlEmptyClass Temp;
                  Temp.Configure(Component,PathName,DefinitionTreePtr);
                  ResearchControlGroups_[ResearchConfig[i]["Group-Name"].GetString()][j].push_back(Temp);
                }
                if (Component["Type"] == "Constant") {
                  ControlConstantClass Temp;
                  Temp.Configure(Component,PathName,DefinitionTreePtr);
                  ResearchControlGroups_[ResearchConfig[i]["Group-Name"].GetString()][j].push_back(Temp);
                }
                if (Component["Type"] == "Gain") {
                  ControlGainClass Temp;
                  Temp.Configure(Component,PathName,DefinitionTreePtr);
                  ResearchControlGroups_[ResearchConfig[i]["Group-Name"].GetString()][j].push_back(Temp);
                }
                if (Component["Type"] == "PID") {
                  ControlPIDClass Temp;
                  Temp.Configure(Component,PathName,DefinitionTreePtr);
                  ResearchControlGroups_[ResearchConfig[i]["Group-Name"].GetString()][j].push_back(Temp);
                }
                if (Component["Type"] == "PID2") {
                  ControlPID2Class Temp;
                  Temp.Configure(Component,PathName,DefinitionTreePtr);
                  ResearchControlGroups_[ResearchConfig[i]["Group-Name"].GetString()][j].push_back(Temp);
                }
                if (Component["Type"] == "State-Space") {
                  ControlStateSpaceClass Temp;
                  Temp.Configure(Component,PathName,DefinitionTreePtr);
                  ResearchControlGroups_[ResearchConfig[i]["Group-Name"].GetString()][j].push_back(Temp);
                }
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
          DefinitionTreePtr->InitMember(MemberName,std::get_if<uint64_t>(&OutputData_[i]),TempDef.Description,true,false);
        }
        if (DefinitionTreePtr->GetValuePtr<uint32_t*>(BaselineKeys[i])) {
          DefinitionTreePtr->InitMember(MemberName,std::get_if<uint32_t>(&OutputData_[i]),TempDef.Description,true,false);
        }
        if (DefinitionTreePtr->GetValuePtr<uint16_t*>(BaselineKeys[i])) {
          DefinitionTreePtr->InitMember(MemberName,std::get_if<uint16_t>(&OutputData_[i]),TempDef.Description,true,false);
        }
        if (DefinitionTreePtr->GetValuePtr<uint8_t*>(BaselineKeys[i])) {
          DefinitionTreePtr->InitMember(MemberName,std::get_if<uint8_t>(&OutputData_[i]),TempDef.Description,true,false);
        }
        if (DefinitionTreePtr->GetValuePtr<int64_t*>(BaselineKeys[i])) {
          DefinitionTreePtr->InitMember(MemberName,std::get_if<int64_t>(&OutputData_[i]),TempDef.Description,true,false);
        }
        if (DefinitionTreePtr->GetValuePtr<int32_t*>(BaselineKeys[i])) {
          DefinitionTreePtr->InitMember(MemberName,std::get_if<int32_t>(&OutputData_[i]),TempDef.Description,true,false);
        }
        if (DefinitionTreePtr->GetValuePtr<int16_t*>(BaselineKeys[i])) {
          DefinitionTreePtr->InitMember(MemberName,std::get_if<int16_t>(&OutputData_[i]),TempDef.Description,true,false);
        }
        if (DefinitionTreePtr->GetValuePtr<int8_t*>(BaselineKeys[i])) {
          DefinitionTreePtr->InitMember(MemberName,std::get_if<int8_t>(&OutputData_[i]),TempDef.Description,true,false);
        }
        if (DefinitionTreePtr->GetValuePtr<float*>(BaselineKeys[i])) {
          DefinitionTreePtr->InitMember(MemberName,std::get_if<float>(&OutputData_[i]),TempDef.Description,true,false);
        }
        if (DefinitionTreePtr->GetValuePtr<double*>(BaselineKeys[i])) {
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
            DefinitionTreePtr->InitMember(MemberName,std::get_if<uint64_t>(&OutputData_[j]),TempDef.Description,true,false);
          }
          if (DefinitionTreePtr->GetValuePtr<uint32_t*>(ResearchKeys[j])) {
            DefinitionTreePtr->InitMember(MemberName,std::get_if<uint32_t>(&OutputData_[j]),TempDef.Description,true,false);
          }
          if (DefinitionTreePtr->GetValuePtr<uint16_t*>(ResearchKeys[j])) {
            DefinitionTreePtr->InitMember(MemberName,std::get_if<uint16_t>(&OutputData_[j]),TempDef.Description,true,false);
          }
          if (DefinitionTreePtr->GetValuePtr<uint8_t*>(ResearchKeys[j])) {
            DefinitionTreePtr->InitMember(MemberName,std::get_if<uint8_t>(&OutputData_[j]),TempDef.Description,true,false);
          }
          if (DefinitionTreePtr->GetValuePtr<int64_t*>(ResearchKeys[j])) {
            DefinitionTreePtr->InitMember(MemberName,std::get_if<int64_t>(&OutputData_[j]),TempDef.Description,true,false);
          }
          if (DefinitionTreePtr->GetValuePtr<int32_t*>(ResearchKeys[j])) {
            DefinitionTreePtr->InitMember(MemberName,std::get_if<int32_t>(&OutputData_[j]),TempDef.Description,true,false);
          }
          if (DefinitionTreePtr->GetValuePtr<int16_t*>(ResearchKeys[j])) {
            DefinitionTreePtr->InitMember(MemberName,std::get_if<int16_t>(&OutputData_[j]),TempDef.Description,true,false);
          }
          if (DefinitionTreePtr->GetValuePtr<int8_t*>(ResearchKeys[j])) {
            DefinitionTreePtr->InitMember(MemberName,std::get_if<int8_t>(&OutputData_[j]),TempDef.Description,true,false);
          }
          if (DefinitionTreePtr->GetValuePtr<float*>(ResearchKeys[j])) {
            DefinitionTreePtr->InitMember(MemberName,std::get_if<float>(&OutputData_[j]),TempDef.Description,true,false);
          }
          if (DefinitionTreePtr->GetValuePtr<double*>(ResearchKeys[j])) {
            DefinitionTreePtr->InitMember(MemberName,std::get_if<double>(&OutputData_[j]),TempDef.Description,true,false);
          }
        }
      }
      j++;
    }
  }
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

/* initializes control laws */
bool ControlLaws::Initialized() {
  if (InitializedLatch_) {
    return true;
  } else {
    bool initialized = true;
    // initializing baseline control laws
    for (size_t i=0; i < BaselineControlGroup_.size(); i++) {
      for (size_t j=0; j < BaselineControlGroup_[i].size(); j++) {
        if (!BaselineControlGroup_[i][j].Initialized()) {
          initialized = false;
        }
      }
    }
    // initializing research control laws
    for (size_t i=0; i < ResearchGroupKeys_.size(); i++) {
      for (size_t j=0; j < ResearchControlGroups_[ResearchGroupKeys_[i]].size(); j++) {
        for (size_t k=0; k < ResearchControlGroups_[ResearchGroupKeys_[i]][j].size(); k++) {
          if (!ResearchControlGroups_[ResearchGroupKeys_[i]][j][k].Initialized()) {
            initialized = false;
          }
        }
      }
    }
    if (initialized) {
      InitializedLatch_ = true;
    }
    return initialized;
  }
}

/* computes control law data */
void ControlLaws::Run(size_t ControlLevel) {
  if (EngagedGroup_ == "Baseline") {
    // running baseline control laws as engaged
    for (size_t i=0; i < BaselineControlGroup_[ControlLevel].size(); i++) {
      BaselineControlGroup_[ControlLevel][i].Run(ControlFunctionClass::kEngage);
    }
    // run armed research control laws
    for (size_t i=0; i < ResearchControlGroups_[ArmedGroup_][ControlLevel].size(); i++) {
      ResearchControlGroups_[ArmedGroup_][ControlLevel][i].Run(ControlFunctionClass::kInitialize);
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
      ResearchControlGroups_[EngagedGroup_][ControlLevel][i].Run(ControlFunctionClass::kEngage);
    }
    // running armed research control laws
    for (size_t i=0; i < ResearchControlGroups_[ArmedGroup_][ControlLevel].size(); i++) {
      ResearchControlGroups_[ArmedGroup_][ControlLevel][i].Run(ControlFunctionClass::kInitialize);
    }
    // running baseline control laws as armed
    for (size_t i=0; i < BaselineControlGroup_[ControlLevel].size(); i++) {
      BaselineControlGroup_[ControlLevel][i].Run(ControlFunctionClass::kInitialize);
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
