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

/* see control.hxx for configuration details */

/* configures control laws given a JSON value and registers data with global defs */
void ControlLaws::Configure(const rapidjson::Value& Config, DefinitionTree *DefinitionTreePtr) {
  // configuring research control laws
  if (Config.HasMember("Research")) {
    const rapidjson::Value& ResearchConfig = Config["Research"];
    for (size_t i=0; i < ResearchConfig.Size(); i++) {
      if (Config.HasMember(ResearchConfig[i].GetString())) {
        ResearchGroupKeys_.push_back(ResearchConfig[i].GetString());
        std::string PathName = RootPath_+"/"+ResearchConfig[i].GetString();
        const rapidjson::Value& GroupDefinition = Config[ResearchConfig[i].GetString()];
        ResearchControlGroups_[ResearchGroupKeys_.back()].resize(GroupDefinition.Size());
        for (size_t j=0; j < GroupDefinition.Size(); j++) {
          const rapidjson::Value& GroupMember = GroupDefinition[j];
          if (GroupMember.HasMember("Level")&&GroupMember.HasMember("Components")) {
            ResearchLevelNames_[ResearchGroupKeys_.back()].push_back(GroupMember["Level"].GetString());
            const rapidjson::Value& Components = GroupMember["Components"];
            for (size_t k=0; k < Components.Size(); k++) {
              const rapidjson::Value& Component = Components[k];
              if (Component.HasMember("Type")) {
                if (Component["Type"] == "Constant") {
                  ConstantClass Temp;
                  ResearchControlGroups_[ResearchGroupKeys_.back()][j].push_back(std::make_shared<ConstantClass>(Temp));
                  ResearchControlGroups_[ResearchGroupKeys_.back()][j].back()->Configure(Component,PathName,DefinitionTreePtr);
                }
                if (Component["Type"] == "Gain") {
                  GainClass Temp;
                  ResearchControlGroups_[ResearchGroupKeys_.back()][j].push_back(std::make_shared<GainClass>(Temp));
                  ResearchControlGroups_[ResearchGroupKeys_.back()][j].back()->Configure(Component,PathName,DefinitionTreePtr);
                }
                if (Component["Type"] == "Sum") {
                  SumClass Temp;
                  ResearchControlGroups_[ResearchGroupKeys_.back()][j].push_back(std::make_shared<SumClass>(Temp));
                  ResearchControlGroups_[ResearchGroupKeys_.back()][j].back()->Configure(Component,PathName,DefinitionTreePtr);
                }
                if (Component["Type"] == "PID") {
                  PIDClass Temp;
                  ResearchControlGroups_[ResearchGroupKeys_.back()][j].push_back(std::make_shared<PIDClass>(Temp));
                  ResearchControlGroups_[ResearchGroupKeys_.back()][j].back()->Configure(Component,PathName,DefinitionTreePtr);
                }
              } else {
                // throw an error
              }
            }
          } else {
            // throw an error
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
        // throw an error
      }
    }
  }

  // resize vectors
  OutputData_.resize(OutputKeys_.size());
  // map research data pointers to superset of output keys
  for (size_t i=0; i < ResearchGroupKeys_.size(); i++) {
    std::string PathName = RootPath_ +"/"+ ResearchGroupKeys_[i];
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
            OutputData_[j] = *(DefinitionTreePtr->GetValuePtr<uint64_t*>(ResearchKeys[j]));
            DefinitionTreePtr->InitMember(MemberName,std::get_if<uint64_t>(&OutputData_[j]),TempDef.Description,true,false);
          }
          if (DefinitionTreePtr->GetValuePtr<uint32_t*>(ResearchKeys[j])) {
            OutputData_[j] = *(DefinitionTreePtr->GetValuePtr<uint32_t*>(ResearchKeys[j]));
            DefinitionTreePtr->InitMember(MemberName,std::get_if<uint32_t>(&OutputData_[j]),TempDef.Description,true,false);
          }
          if (DefinitionTreePtr->GetValuePtr<uint16_t*>(ResearchKeys[j])) {
            OutputData_[j] = *(DefinitionTreePtr->GetValuePtr<uint16_t*>(ResearchKeys[j]));
            DefinitionTreePtr->InitMember(MemberName,std::get_if<uint16_t>(&OutputData_[j]),TempDef.Description,true,false);
          }
          if (DefinitionTreePtr->GetValuePtr<uint8_t*>(ResearchKeys[j])) {
            OutputData_[j] = *(DefinitionTreePtr->GetValuePtr<uint8_t*>(ResearchKeys[j]));
            DefinitionTreePtr->InitMember(MemberName,std::get_if<uint8_t>(&OutputData_[j]),TempDef.Description,true,false);
          }
          if (DefinitionTreePtr->GetValuePtr<int64_t*>(ResearchKeys[j])) {
            OutputData_[j] = *(DefinitionTreePtr->GetValuePtr<int64_t*>(ResearchKeys[j]));
            DefinitionTreePtr->InitMember(MemberName,std::get_if<int64_t>(&OutputData_[j]),TempDef.Description,true,false);
          }
          if (DefinitionTreePtr->GetValuePtr<int32_t*>(ResearchKeys[j])) {
            OutputData_[j] = *(DefinitionTreePtr->GetValuePtr<int32_t*>(ResearchKeys[j]));
            DefinitionTreePtr->InitMember(MemberName,std::get_if<int32_t>(&OutputData_[j]),TempDef.Description,true,false);
          }
          if (DefinitionTreePtr->GetValuePtr<int16_t*>(ResearchKeys[j])) {
            OutputData_[j] = *(DefinitionTreePtr->GetValuePtr<int16_t*>(ResearchKeys[j]));
            DefinitionTreePtr->InitMember(MemberName,std::get_if<int16_t>(&OutputData_[j]),TempDef.Description,true,false);
          }
          if (DefinitionTreePtr->GetValuePtr<int8_t*>(ResearchKeys[j])) {
            OutputData_[j] = *(DefinitionTreePtr->GetValuePtr<int8_t*>(ResearchKeys[j]));
            DefinitionTreePtr->InitMember(MemberName,std::get_if<int8_t>(&OutputData_[j]),TempDef.Description,true,false);
          }
          if (DefinitionTreePtr->GetValuePtr<float*>(ResearchKeys[j])) {
            OutputData_[j] = *(DefinitionTreePtr->GetValuePtr<float*>(ResearchKeys[j]));
            DefinitionTreePtr->InitMember(MemberName,&(std::get<float>(OutputData_[j])),TempDef.Description,true,false);
          }
          if (DefinitionTreePtr->GetValuePtr<double*>(ResearchKeys[j])) {
            OutputData_[j] = *(DefinitionTreePtr->GetValuePtr<double*>(ResearchKeys[j]));
            DefinitionTreePtr->InitMember(MemberName,std::get_if<double>(&OutputData_[j]),TempDef.Description,true,false);
          }
        }
      }
      j++;
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
    return 0;
  } else {
    return ResearchControlGroups_[EngagedGroup_].size();
  }
}

/* returns the name of the level for the engaged control law */
std::string ControlLaws::GetActiveLevel(size_t ControlLevel) {
  if (EngagedGroup_ == "Baseline") {
    return NULL;
  } else {
    return ResearchLevelNames_[EngagedGroup_][ControlLevel];
  }
}

/* computes control law data */
void ControlLaws::RunEngaged(size_t ControlLevel) {
  if (EngagedGroup_ != "Baseline") {
    // running engaged research control laws
    for (size_t i=0; i < ResearchControlGroups_[EngagedGroup_][ControlLevel].size(); i++) {
      ResearchControlGroups_[EngagedGroup_][ControlLevel][i]->Run(GenericFunction::kEngage);
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

/* computes control law data */
void ControlLaws::RunArmed() {
  if (EngagedGroup_ != ArmedGroup_) {
    for (size_t i=0; i < ResearchGroupKeys_.size(); i++) {
      for (size_t j=0; j < ResearchControlGroups_[ResearchGroupKeys_[i]].size(); j++) {
        for (size_t k=0; k < ResearchControlGroups_[ResearchGroupKeys_[i]][j].size(); k++) {
          if (ResearchGroupKeys_[i] == ArmedGroup_) {
            ResearchControlGroups_[ResearchGroupKeys_[i]][j][k]->Run(GenericFunction::kArm);
          } else {
            ResearchControlGroups_[ResearchGroupKeys_[i]][j][k]->Run(GenericFunction::kStandby);
          }
        }
      }
    }
  }
  if (EngagedGroup_ == "Baseline") {
    std::vector<std::variant<uint64_t*,uint32_t*,uint16_t*,uint8_t*,int64_t*,int32_t*,int16_t*,int8_t*,float*,double*>> ResearchPtr;
    ResearchPtr = ResearchDataPtr_[ArmedGroup_];
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
