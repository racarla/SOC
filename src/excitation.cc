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

/* configures excitation system given a JSON value and registers data with global defs */
void ExcitationSystem::Configure(const rapidjson::Value& Config, DefinitionTree *DefinitionTreePtr) {
  if (!Config.HasMember("Time")) {
    // throw error
  }
  if (Config.HasMember("Groups")) {
    const rapidjson::Value& Groups = Config["Groups"];
    ExcitationGroups_.resize(Groups.Size());
    for (size_t i=0; i < Groups.Size(); i++) {
      const rapidjson::Value& Group = Group[i];
      if (Group.HasMember("Name")&&Group.HasMember("Components")) {
        ExcitationGroupKeys_.push_back(Group["Name"].GetString());
        std::string PathName = RootPath_+"/"+ExcitationGroupKeys_.back();
        const rapidjson::Value& Components = Group["Components"];
        ExcitationGroups_[i].resize(Components.Size());
        for (size_t j=0; j < Components.Size(); j++) {
          if (Components[j].HasMember("Level")&&Components[j].HasMember("Components")) {
            ExcitationGroupLevels_.push_back(Components[j]["Level"].GetString());
            for (size_t k=0; k < Components[j]["Components"].Size(); k++) {
              const rapidjson::Value& Component = Components[j]["Components"][k];
              if (Component.HasMember("Waveform")&&Component.HasMember("Signal")&&Component.HasMember("Start-Time")&&Component.HasMember("Amplitude")) {
                if (Config.HasMember(Component["Waveform"].GetString())) {
                  const rapidjson::Value& WaveformValue = Config[Component["Waveform"].GetString()];
                  rapidjson::Document Waveform;
                  rapidjson::Document::AllocatorType& Allocator = Waveform.GetAllocator();
                  Waveform.SetObject();
                  Waveform.CopyFrom(WaveformValue,Allocator);
                  rapidjson::Value Time;
                  rapidjson::Value Signal;
                  Time.SetString(std::string(Config["Time"].GetString()).c_str(),std::string(Config["Time"].GetString()).size(),Allocator);
                  Signal.SetString(std::string(Config["Signal"].GetString()).c_str(),std::string(Config["Signal"].GetString()).size(),Allocator);
                  Waveform.AddMember("Time",Time,Allocator);
                  Waveform.AddMember("Signal",Signal,Allocator);
                  Waveform.AddMember("Start-Time",Component["Start-Time"].GetFloat(),Allocator);
                  Waveform.AddMember("Scale-Factor",Component["Amplitude"].GetFloat(),Allocator);
                  if (WaveformValue["Type"] == "Pulse") {
                    Pulse Temp;
                    ExcitationGroups_[i][j].push_back(std::make_shared<Pulse>(Temp));
                    ExcitationGroups_[i][j][k]->Configure(Waveform,PathName,DefinitionTreePtr);
                  }
                  if (WaveformValue["Type"] == "Doublet") {
                    Doublet Temp;
                    ExcitationGroups_[i][j].push_back(std::make_shared<Doublet>(Temp));
                    ExcitationGroups_[i][j][k]->Configure(Waveform,PathName,DefinitionTreePtr);
                  }
                  if (WaveformValue["Type"] == "Doublet121") {
                    Doublet121 Temp;
                    ExcitationGroups_[i][j].push_back(std::make_shared<Doublet121>(Temp));
                    ExcitationGroups_[i][j][k]->Configure(Waveform,PathName,DefinitionTreePtr);
                  }
                  if (WaveformValue["Type"] == "Doublet3211") {
                    Doublet3211 Temp;
                    ExcitationGroups_[i][j].push_back(std::make_shared<Doublet3211>(Temp));
                    ExcitationGroups_[i][j][k]->Configure(Waveform,PathName,DefinitionTreePtr);
                  }
                  if (WaveformValue["Type"] == "LinearChirp") {
                    LinearChirp Temp;
                    ExcitationGroups_[i][j].push_back(std::make_shared<LinearChirp>(Temp));
                    ExcitationGroups_[i][j][k]->Configure(Waveform,PathName,DefinitionTreePtr);
                  }
                  if (WaveformValue["Type"] == "MultiSine") {
                    MultiSine Temp;
                    ExcitationGroups_[i][j].push_back(std::make_shared<MultiSine>(Temp));
                    ExcitationGroups_[i][j][k]->Configure(Waveform,PathName,DefinitionTreePtr);
                  }
                }
              } else {
                // throw error
              }
            }
          } else {
            // throw error
          }
        }
      } else {
        // throw error
      }
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
          ExcitationGroups_[i][j][k]->Run(GenericFunction::kEngage);
        }
      } else {
        for (size_t k=0; k < ExcitationGroups_[i][j].size(); k++) {
          ExcitationGroups_[i][j][k]->Run(GenericFunction::kArm);
        }
      }
    }
  }
}
