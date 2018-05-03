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
void ExcitationFunctionClass::Run() {}

void Pulse::Configure(const rapidjson::Value& Config,std::string RootPath,DefinitionTree *DefinitionTreePtr) {}
void Pulse::Run() {}

void Doublet::Configure(const rapidjson::Value& Config,std::string RootPath,DefinitionTree *DefinitionTreePtr) {}
void Doublet::Run() {}

void Doublet121::Configure(const rapidjson::Value& Config,std::string RootPath,DefinitionTree *DefinitionTreePtr) {}
void Doublet121::Run() {}

void Doublet3211::Configure(const rapidjson::Value& Config,std::string RootPath,DefinitionTree *DefinitionTreePtr) {}
void Doublet3211::Run() {}

void LinearChirp::Configure(const rapidjson::Value& Config,std::string RootPath,DefinitionTree *DefinitionTreePtr) {}
void LinearChirp::Run() {}

/* configures excitation system given a JSON value and registers data with global defs */
void ExcitationSystem::Configure(const rapidjson::Value& Config, DefinitionTree *DefinitionTreePtr) {
  std::string PathName = RootPath_ + "/";
  for (size_t i=0; i < Config.Size(); i++) {
    const rapidjson::Value& Excitation = Config[i];
    if (Excitation.HasMember("Type")&&Excitation.HasMember("Level")) {
      if (Excitation["Type"] == "Pulse") {
        Pulse Temp;
        ExcitationGroup_[Excitation["Level"].GetString()].push_back(std::make_shared<Pulse>(Temp));
        ExcitationGroup_[Excitation["Level"].GetString()][i]->Configure(Excitation,PathName,DefinitionTreePtr);
      }
      if (Excitation["Type"] == "Doublet") {
        Doublet Temp;
        ExcitationGroup_[Excitation["Level"].GetString()].push_back(std::make_shared<Doublet>(Temp));
        ExcitationGroup_[Excitation["Level"].GetString()][i]->Configure(Excitation,PathName,DefinitionTreePtr);
      }
      if (Excitation["Type"] == "Doublet121") {
        Doublet121 Temp;
        ExcitationGroup_[Excitation["Level"].GetString()].push_back(std::make_shared<Doublet121>(Temp));
        ExcitationGroup_[Excitation["Level"].GetString()][i]->Configure(Excitation,PathName,DefinitionTreePtr);
      }
      if (Excitation["Type"] == "Doublet3211") {
        Doublet3211 Temp;
        ExcitationGroup_[Excitation["Level"].GetString()].push_back(std::make_shared<Doublet3211>(Temp));
        ExcitationGroup_[Excitation["Level"].GetString()][i]->Configure(Excitation,PathName,DefinitionTreePtr);
      }
      if (Excitation["Type"] == "Linear-Chirp") {
        LinearChirp Temp;
        ExcitationGroup_[Excitation["Level"].GetString()].push_back(std::make_shared<LinearChirp>(Temp));
        ExcitationGroup_[Excitation["Level"].GetString()][i]->Configure(Excitation,PathName,DefinitionTreePtr);
      }
    } else {
      throw std::runtime_error(std::string("ERROR")+PathName+std::string(": Excitation type or level not specified in configuration."));
    }
  }
  Configured_ = true;
}

/* return whether the excitation system has been configured */
bool ExcitationSystem::Configured() {
  return Configured_;
}

/* run all excitation functions at a given control level */
void ExcitationSystem::Run(std::string ControlLevel) {
  for (size_t i=0; i < ExcitationGroup_[ControlLevel].size(); i++) {
    ExcitationGroup_[ControlLevel][i]->Run();
  }
}
