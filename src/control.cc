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
  std::map<std::string,std::string> OutputKeysMap;
  // configuring research control laws, baseline control laws are on FMU
  if (Config.HasMember("Research")) {
    const rapidjson::Value& ResearchConfig = Config["Research"];
    // iterate over research control law names
    for (auto &GroupName : ResearchConfig.GetArray()) {
      // grab research control law definition
      if (Config.HasMember(GroupName.GetString())) {
        // store the group key
        ResearchGroupKeys_.push_back(GroupName.GetString());
        // json group definition
        const rapidjson::Value& GroupDefinition = Config[GroupName.GetString()];
        // resize the control group by the number of levels
        ResearchControlGroups_[ResearchGroupKeys_.back()].resize(GroupDefinition.Size());
        // resize the data keys by the number of levels
        ResearchDataKeys_[ResearchGroupKeys_.back()].resize(GroupDefinition.Size());
        // iterate over the levels
        for (rapidjson::Value::ConstValueIterator Member = GroupDefinition.Begin(); Member != GroupDefinition.End(); ++Member) {
          if (Member->HasMember("Level")&&Member->HasMember("Components")) {
            auto level = std::distance(GroupDefinition.Begin(),Member);
            // store the level names
            ResearchLevelNames_[ResearchGroupKeys_.back()].push_back((*Member)["Level"].GetString());
            // path for the research functions /Control/"Group-Name"
            std::string PathName = RootPath_+"/"+ResearchGroupKeys_.back()+"/"+ResearchLevelNames_[ResearchGroupKeys_.back()].back();
            // json components on a given level
            const rapidjson::Value& Components = (*Member)["Components"];
            // iterate over the components on each level
            for (auto &Func : Components.GetArray()) {
              if (Func.HasMember("Type")) {
                if (Func["Type"] == "Constant") {
                  ResearchControlGroups_[ResearchGroupKeys_.back()][level].push_back(std::make_shared<ConstantClass>()); 
                }
                if (Func["Type"] == "Gain") {
                  ResearchControlGroups_[ResearchGroupKeys_.back()][level].push_back(std::make_shared<GainClass>());
                }
                if (Func["Type"] == "Sum") {
                  ResearchControlGroups_[ResearchGroupKeys_.back()][level].push_back(std::make_shared<SumClass>());
                }
                if (Func["Type"] == "PID") {
                  ResearchControlGroups_[ResearchGroupKeys_.back()][level].push_back(std::make_shared<PIDClass>());
                }
                if (Func["Type"] == "Filter") {
                  ResearchControlGroups_[ResearchGroupKeys_.back()][level].push_back(std::make_shared<GeneralFilter>());
                }
                if (Func["Type"] == "PseudoInverse") {
                  ResearchControlGroups_[ResearchGroupKeys_.back()][level].push_back(std::make_shared<PseudoInverseAllocation>());
                }
                // configure the function
                ResearchControlGroups_[ResearchGroupKeys_.back()][level].back()->Configure(Func,PathName,DefinitionTreePtr);
              } else {
                throw std::runtime_error(std::string("ERROR")+PathName+std::string(": Type not specified in configuration."));
              }
            }
            // getting a list of all research keys and adding to superset of output keys
            // modify the key to remove the intermediate path 
            // (i.e. /Control/GroupName/Pitch --> /Control/Pitch)
            DefinitionTreePtr->GetKeys(PathName,&ResearchDataKeys_[ResearchGroupKeys_.back()][level]);
            for (auto Key : ResearchDataKeys_[ResearchGroupKeys_.back()][level]) {
              std::string MemberName = RootPath_+Key.substr(Key.rfind("/"));
              if (Key.substr(Key.rfind("/"))!="/Mode") {
                OutputKeysMap[MemberName] = MemberName;
              }
            }
          } else {
            throw std::runtime_error(std::string("ERROR")+RootPath_+std::string(": Level name or components not specified in configuration."));
          }
        }
      } else {
        throw std::runtime_error(std::string("ERROR")+RootPath_+std::string(": Group name not found in configuration."));
      }
    }
  }
  /* research outputs to superset of outputs */
  // iterate through output keys and check for matching keys in research
  for (auto OutputElem : OutputKeysMap) {
    // current output key
    std::string OutputKey = OutputElem.second;
    // iterate through research keys
    for (auto GroupKey : ResearchGroupKeys_) {
      // iterate through all levels
      for (auto Levels = ResearchLevelNames_[GroupKey].begin(); Levels != ResearchLevelNames_[GroupKey].end(); ++Levels) {
        auto Level = std::distance(ResearchLevelNames_[GroupKey].begin(),Levels);
        for (auto ResearchKey : ResearchDataKeys_[GroupKey][Level]) {
          // check for a match with output keys
          if (ResearchKey.substr(ResearchKey.rfind("/"))==OutputKey.substr(OutputKey.rfind("/"))) {
            std::string KeyName = ResearchKey.substr(ResearchKey.rfind("/"));
            // setup research data pointer
            DefinitionTree::VariableDefinition TempDef;
            DefinitionTreePtr->GetMember(ResearchKey,&TempDef);
            ResearchDataPtr_[GroupKey][KeyName] = TempDef.Value;
            // check to see if output key has already been registered
            if (DefinitionTreePtr->Size(OutputKey)==0) {
              // register output if it has not already been
              if (DefinitionTreePtr->GetValuePtr<uint64_t*>(ResearchKey)) {
                OutputData_[KeyName] = *(DefinitionTreePtr->GetValuePtr<uint64_t*>(ResearchKey));
                DefinitionTreePtr->InitMember(OutputKey,std::get_if<uint64_t>(&OutputData_[KeyName]),TempDef.Description,true,false);
              }
              if (DefinitionTreePtr->GetValuePtr<uint32_t*>(ResearchKey)) {
                OutputData_[KeyName] = *(DefinitionTreePtr->GetValuePtr<uint32_t*>(ResearchKey));
                DefinitionTreePtr->InitMember(OutputKey,std::get_if<uint32_t>(&OutputData_[KeyName]),TempDef.Description,true,false);
              }
              if (DefinitionTreePtr->GetValuePtr<uint16_t*>(ResearchKey)) {
                OutputData_[KeyName] = *(DefinitionTreePtr->GetValuePtr<uint16_t*>(ResearchKey));
                DefinitionTreePtr->InitMember(OutputKey,std::get_if<uint16_t>(&OutputData_[KeyName]),TempDef.Description,true,false);
              }
              if (DefinitionTreePtr->GetValuePtr<uint8_t*>(ResearchKey)) {
                OutputData_[KeyName] = *(DefinitionTreePtr->GetValuePtr<uint8_t*>(ResearchKey));
                DefinitionTreePtr->InitMember(OutputKey,std::get_if<uint8_t>(&OutputData_[KeyName]),TempDef.Description,true,false);
              }
              if (DefinitionTreePtr->GetValuePtr<int64_t*>(ResearchKey)) {
                OutputData_[KeyName] = *(DefinitionTreePtr->GetValuePtr<int64_t*>(ResearchKey));
                DefinitionTreePtr->InitMember(OutputKey,std::get_if<int64_t>(&OutputData_[KeyName]),TempDef.Description,true,false);
              }
              if (DefinitionTreePtr->GetValuePtr<int32_t*>(ResearchKey)) {
                OutputData_[KeyName] = *(DefinitionTreePtr->GetValuePtr<int32_t*>(ResearchKey));
                DefinitionTreePtr->InitMember(OutputKey,std::get_if<int32_t>(&OutputData_[KeyName]),TempDef.Description,true,false);
              }
              if (DefinitionTreePtr->GetValuePtr<int16_t*>(ResearchKey)) {
                OutputData_[KeyName] = *(DefinitionTreePtr->GetValuePtr<int16_t*>(ResearchKey));
                DefinitionTreePtr->InitMember(OutputKey,std::get_if<int16_t>(&OutputData_[KeyName]),TempDef.Description,true,false);
              }
              if (DefinitionTreePtr->GetValuePtr<int8_t*>(ResearchKey)) {
                OutputData_[KeyName] = *(DefinitionTreePtr->GetValuePtr<int8_t*>(ResearchKey));
                DefinitionTreePtr->InitMember(OutputKey,std::get_if<int8_t>(&OutputData_[KeyName]),TempDef.Description,true,false);
              }
              if (DefinitionTreePtr->GetValuePtr<float*>(ResearchKey)) {
                OutputData_[KeyName] = *(DefinitionTreePtr->GetValuePtr<float*>(ResearchKey));
                DefinitionTreePtr->InitMember(OutputKey,std::get_if<float>(&OutputData_[KeyName]),TempDef.Description,true,false);
              }
              if (DefinitionTreePtr->GetValuePtr<double*>(ResearchKey)) {
                OutputData_[KeyName] = *(DefinitionTreePtr->GetValuePtr<double*>(ResearchKey));
                DefinitionTreePtr->InitMember(OutputKey,std::get_if<double>(&OutputData_[KeyName]),TempDef.Description,true,false);
              }
            }
          }
        }
      }
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
    return 0;
  } else {
    return ResearchControlGroups_[EngagedGroup_].size();
  }
}

/* returns the name of the level for the engaged control law */
std::string ControlLaws::GetActiveLevel(size_t ControlLevel) {
  if (EngagedGroup_ == "Baseline") {
    return "";
  } else {
    return ResearchLevelNames_[EngagedGroup_][ControlLevel];
  }
}

/* computes control law data */
void ControlLaws::RunEngaged(size_t ControlLevel) {
  if (EngagedGroup_ != "Baseline") {
    // running engaged research control laws
    for (auto Func : ResearchControlGroups_[EngagedGroup_][ControlLevel]) {
      Func->Run(GenericFunction::kEngage);
    }
    // output research control laws
    for (auto Key : ResearchDataKeys_[EngagedGroup_][ControlLevel]) {
      std::string KeyName = Key.substr(Key.rfind("/"));
      if (KeyName!="/Mode") {
        if (std::get_if<uint64_t*>(&ResearchDataPtr_[EngagedGroup_][KeyName])) {
          OutputData_[KeyName] = **(std::get_if<uint64_t*>(&ResearchDataPtr_[EngagedGroup_][KeyName]));
        }
        if (std::get_if<uint32_t*>(&ResearchDataPtr_[EngagedGroup_][KeyName])) {
          OutputData_[KeyName] = **(std::get_if<uint32_t*>(&ResearchDataPtr_[EngagedGroup_][KeyName]));
        }
        if (std::get_if<uint16_t*>(&ResearchDataPtr_[EngagedGroup_][KeyName])) {
          OutputData_[KeyName] = **(std::get_if<uint16_t*>(&ResearchDataPtr_[EngagedGroup_][KeyName]));
        }
        if (std::get_if<uint8_t*>(&ResearchDataPtr_[EngagedGroup_][KeyName])) {
          OutputData_[KeyName] = **(std::get_if<uint8_t*>(&ResearchDataPtr_[EngagedGroup_][KeyName]));
        }
        if (std::get_if<int64_t*>(&ResearchDataPtr_[EngagedGroup_][KeyName])) {
          OutputData_[KeyName] = **(std::get_if<int64_t*>(&ResearchDataPtr_[EngagedGroup_][KeyName]));
        }
        if (std::get_if<int32_t*>(&ResearchDataPtr_[EngagedGroup_][KeyName])) {
          OutputData_[KeyName] = **(std::get_if<int32_t*>(&ResearchDataPtr_[EngagedGroup_][KeyName]));
        }
        if (std::get_if<int16_t*>(&ResearchDataPtr_[EngagedGroup_][KeyName])) {
          OutputData_[KeyName] = **(std::get_if<int16_t*>(&ResearchDataPtr_[EngagedGroup_][KeyName]));
        }
        if (std::get_if<int8_t*>(&ResearchDataPtr_[EngagedGroup_][KeyName])) {
          OutputData_[KeyName] = **(std::get_if<int8_t*>(&ResearchDataPtr_[EngagedGroup_][KeyName]));
        }
        if (std::get_if<float*>(&ResearchDataPtr_[EngagedGroup_][KeyName])) {
          OutputData_[KeyName] = **(std::get_if<float*>(&ResearchDataPtr_[EngagedGroup_][KeyName]));
        }
        if (std::get_if<double*>(&ResearchDataPtr_[EngagedGroup_][KeyName])) {
          OutputData_[KeyName] = **(std::get_if<double*>(&ResearchDataPtr_[EngagedGroup_][KeyName]));
        }
      }
    }
  }
}

/* computes control law data */
void ControlLaws::RunArmed() {
  // iterate through all groups
  for (auto Group : ResearchGroupKeys_) {
    // iterate through all levels
    for (auto Levels = ResearchControlGroups_[Group].begin(); Levels != ResearchControlGroups_[Group].end(); ++Levels) {
      auto Level = std::distance(ResearchControlGroups_[Group].begin(),Levels); 
      // iterate through all functions
      for (auto Func : ResearchControlGroups_[Group][Level]) {
        // make sure we don't run the engaged group
        if (Group != EngagedGroup_) {
          // run as arm if the armed group, otherwise standby
          if (Group == ArmedGroup_) {
            Func->Run(GenericFunction::kArm);
          } else {
            Func->Run(GenericFunction::kStandby);
          }
        }
      }
    }
  }
}
