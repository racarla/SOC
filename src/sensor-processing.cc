/*
sensor-processing.cc
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

#include "sensor-processing.hxx"

/* base function class methods */
void SensorProcessingFunctionClass::Configure(const rapidjson::Value& Config,std::string RootPath,DefinitionTree *DefinitionTreePtr) {}
bool SensorProcessingFunctionClass::Initialized() {}
void SensorProcessingFunctionClass::Run(Mode mode) {}

/* configure baseline airdata from JSON and global data definition tree */
void BaselineAirDataClass::Configure(const rapidjson::Value& Config,std::string RootPath,DefinitionTree *DefinitionTreePtr) {
  std::string OutputName;
  // get output name
  if (Config.HasMember("Output-Name")) {
    OutputName = RootPath + "/" + Config["Output-Name"].GetString();
  } else {
    throw std::runtime_error(std::string("ERROR")+RootPath+std::string(": Output-Name not specified in configuration."));
  }
  // get initialization time
  if (Config.HasMember("Initialization-Time")) {
    config_.InitializationTime_s = Config["Initialization-Time"].GetFloat();
  } else {
    throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Initialization time not specified in configuration."));
  }
  // get static pressure configuration
  if (Config.HasMember("Static-Pressure")) {
    const rapidjson::Value& StaticPressureSources = Config["Static-Pressure"];
    for (size_t i=0; i < StaticPressureSources.Size(); i++) {
      const rapidjson::Value& StaticPressureSource = StaticPressureSources[i];
      std::string source = StaticPressureSource.GetString() + std::string("/Pressure_Pa");
      if (DefinitionTreePtr->GetValuePtr<float*>(source)) {
        config_.StaticPressureSourcePtr.push_back(DefinitionTreePtr->GetValuePtr<float*>(source));
      } else {
        throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Static pressure source not found in global data."));
      }
    }
  }
  // get differential pressure configuration
  if (Config.HasMember("Differential-Pressure")) {
    const rapidjson::Value& DifferentialPressureSources = Config["Differential-Pressure"];
    for (size_t i=0; i < DifferentialPressureSources.Size(); i++) {
      const rapidjson::Value& DifferentialPressureSource = DifferentialPressureSources[i];
      std::string source = DifferentialPressureSource.GetString() + std::string("/Pressure_Pa");
      if (DefinitionTreePtr->GetValuePtr<float*>(source)) {
        config_.DifferentialPressureSourcePtr.push_back(DefinitionTreePtr->GetValuePtr<float*>(source));
      } else {
        throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Differential pressure source not found in global data."));
      }

    }
    config_.DifferentialPressureBiases.resize(DifferentialPressureSources.Size());
  }
  // get MSL altitude configuration
  if (Config.HasMember("MSL-Altitude")) {
    const rapidjson::Value& MslAltSources = Config["MSL-Altitude"];
    for (size_t i=0; i < MslAltSources.Size(); i++) {
      const rapidjson::Value& MslAltSource = MslAltSources[i];
      std::string source = MslAltSource.GetString() + std::string("/Altitude_m");
      std::string fix = MslAltSource.GetString() + std::string("/Fix");
      if (DefinitionTreePtr->GetValuePtr<float*>(source)) {
        config_.MslAltSourcePtr.push_back(DefinitionTreePtr->GetValuePtr<float*>(source));
      } else {
        throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": MSL altitude source not found in global data."));
      }
      if (DefinitionTreePtr->GetValuePtr<uint8_t*>(fix)) {
        config_.MslAltFixPtr.push_back(DefinitionTreePtr->GetValuePtr<uint8_t*>(fix));
      } else {
        throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": MSL altitude fix not found in global data."));
      }
    }
  }
  // pointer to log run mode data
  DefinitionTreePtr->InitMember(OutputName+"/Mode",&data_.Mode,"Run mode",true,false);
  // if at least one static pressure source, register: pressure altitude and AGL altitude
  if (config_.StaticPressureSourcePtr.size() > 0) {
    DefinitionTreePtr->InitMember(OutputName+"/Pressure/Static_Pa",&data_.StaticPressure_Pa,"Static pressure, Pa",true,false);
    DefinitionTreePtr->InitMember(OutputName+"/Altitude/Pressure_m",&data_.PressureAltitude_m,"Pressure altitude, m",true,false);
    DefinitionTreePtr->InitMember(OutputName+"/Altitude/AGL_m",&data_.AGL_m,"AGL altitude, m",true,false);
  }
  // if at least one differential pressure source, register: indicated airspeed
  if (config_.DifferentialPressureSourcePtr.size() > 0) {
    DefinitionTreePtr->InitMember(OutputName+"/Pressure/Differential_Pa",&data_.DifferentialPressure_Pa,"Differential pressure, Pa",true,false);
    DefinitionTreePtr->InitMember(OutputName+"/Airspeed/Indicated_ms",&data_.IAS_ms,"Indicated airspeed, m/s",true,false);
  }
  // if at least one static pressure and one MSL altitude source, register: MSL altitude
  if ((config_.MslAltSourcePtr.size() > 0)&&(config_.StaticPressureSourcePtr.size() > 0)) {
    DefinitionTreePtr->InitMember(OutputName+"/Altitude/MSL_m",&data_.MSL_m,"MSL altitude, m",true,false);
  }
}

/* initializes air data */
bool BaselineAirDataClass::Initialized() {
  if (InitializedLatch_) {
    return true;
  } else {
    bool GpsFix = true;
    for (size_t i=0; i < config_.MslAltFixPtr.size(); i++) {
      if (!*config_.MslAltFixPtr[i]) {
        GpsFix = false;
      }
    }
    if ((config_.MslAltSourcePtr.size() == 0)||(GpsFix)) {
      static elapsedMicros InitializationTimer = 0;
      static size_t NumberSamples = 1;
      if (InitializationTimer > 1e6) {
        // if we have at least one static pressure source
        if (config_.StaticPressureSourcePtr.size() > 0) {
          float AvgPress = 0;
          for (size_t i=0; i < config_.StaticPressureSourcePtr.size(); i++) {
            AvgPress += *config_.StaticPressureSourcePtr[i]/config_.StaticPressureSourcePtr.size();
          }
          // get the initial pressure altitude
          config_.InitialPressureAlt_m = config_.InitialPressureAlt_m+(airdata_.getPressureAltitude(AvgPress)-config_.InitialPressureAlt_m)/((float)NumberSamples);
        }
        // if we have at least one differential pressure source
        if (config_.DifferentialPressureSourcePtr.size() > 0) {
          for (size_t i=0; i < config_.DifferentialPressureSourcePtr.size(); i++) {
            config_.DifferentialPressureBiases[i] = config_.DifferentialPressureBiases[i] + (*config_.DifferentialPressureSourcePtr[i]-config_.DifferentialPressureBiases[i])/((float)NumberSamples);
          }
        }
        // if at least one static pressure and one MSL altitude source
        if ((config_.MslAltSourcePtr.size() > 0)&&(config_.StaticPressureSourcePtr.size() > 0)) {
          float AvgMSL = 0;
          for (size_t i=0; i < config_.MslAltSourcePtr.size(); i++) {
            AvgMSL += *config_.MslAltSourcePtr[i]/config_.MslAltSourcePtr.size();
          }
          // get the initial MSL altitude
          config_.InitialMSLAlt_m = config_.InitialMSLAlt_m+(AvgMSL-config_.InitialMSLAlt_m)/((float)NumberSamples);
        }
        NumberSamples++;
        // if time > duration return true
        if (InitializationTimer > (config_.InitializationTime_s + 1)*1e6) {
          InitializedLatch_ = true;
          return true;
        } else {
          return false;
        }
      } else {
        return false;
      }
    } else {
      return false;
    }
  }
}

/* compute air data values */
void BaselineAirDataClass::Run(Mode mode) {
  data_.Mode = (uint8_t) mode;
  // if we have at least one static pressure source
  if (config_.StaticPressureSourcePtr.size() > 0) {
    // average all static pressure sources
    data_.StaticPressure_Pa = 0;
    for (size_t i=0; i < config_.StaticPressureSourcePtr.size(); i++) {
      data_.StaticPressure_Pa += *config_.StaticPressureSourcePtr[i]/config_.StaticPressureSourcePtr.size();
    }
    // compute pressure altitude
    data_.PressureAltitude_m = airdata_.getPressureAltitude(data_.StaticPressure_Pa);
    // compute AGL altitude
    data_.AGL_m = airdata_.getAGL(data_.StaticPressure_Pa,config_.InitialPressureAlt_m);
  }
  // if we have at least one differential pressure source
  if (config_.DifferentialPressureSourcePtr.size() > 0) {
    // remove all diff pressure biases and average
    data_.DifferentialPressure_Pa = 0;
    for (size_t i=0; i < config_.DifferentialPressureSourcePtr.size(); i++) {
      data_.DifferentialPressure_Pa += (*config_.DifferentialPressureSourcePtr[i]-config_.DifferentialPressureBiases[i])/config_.DifferentialPressureSourcePtr.size();
    }
    // compute indicated airspeed
    data_.IAS_ms = airdata_.getIAS(data_.DifferentialPressure_Pa);
  }
  // if at least one static pressure and one MSL altitude source
  if ((config_.MslAltSourcePtr.size() > 0)&&(config_.StaticPressureSourcePtr.size() > 0)) {
    // compute MSL altitude
    data_.MSL_m = airdata_.getMSL(data_.AGL_m,config_.InitialMSLAlt_m);
  }
}

/* configures sensor processing given a JSON value and registers data with global defs */
void SensorProcessing::Configure(const rapidjson::Value& Config,DefinitionTree *DefinitionTreePtr) {
  // configuring baseline sensor processing
  if (Config.HasMember("Baseline")) {
    std::string PathName = RootPath_ + "/" + "Baseline";
    const rapidjson::Value& BaselineConfig = Config["Baseline"];
    for (size_t i=0; i < BaselineConfig.Size(); i++) {
      if (BaselineConfig[i].HasMember("Type")) {
        if (BaselineConfig[i]["Type"] == "BaselineAirData") {
          BaselineAirDataClass Temp;
          BaselineSensorProcessing_.push_back(std::make_shared<BaselineAirDataClass>(Temp));
          BaselineSensorProcessing_[i]->Configure(BaselineConfig[i],PathName,DefinitionTreePtr);
        }
      } else {
        throw std::runtime_error(std::string("ERROR")+PathName+std::string(": Type not specified in configuration."));
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
        for (size_t j=0; j < ResearchConfig[i]["Components"].Size(); j++) {
          if (ResearchConfig[i]["Components"][j].HasMember("Type")) {
            if (ResearchConfig[i]["Components"][j]["Type"] == "BaselineAirData") {
              BaselineAirDataClass Temp;
              ResearchSensorProcessingGroups_[ResearchConfig[i]["Group-Name"].GetString()].push_back(std::make_shared<BaselineAirDataClass>(Temp));
              ResearchSensorProcessingGroups_[ResearchConfig[i]["Group-Name"].GetString()][j]->Configure(ResearchConfig[i]["Components"][j],PathName,DefinitionTreePtr);
            }
          } else {
            throw std::runtime_error(std::string("ERROR")+PathName+std::string(": Type not specified in configuration."));
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
}

/* initializes sensor processing */
bool SensorProcessing::Initialized() {
  if (InitializedLatch_) {
    return true;
  } else {
    bool initialized = true;
    // initializing baseline sensor processing
    for (size_t i=0; i < BaselineSensorProcessing_.size(); i++) {
      if (!BaselineSensorProcessing_[i]->Initialized()) {
        initialized = false;
      }
    }
    // initializing research sensor processing
    for (size_t i=0; i < ResearchGroupKeys_.size(); i++) {
      for (size_t j=0; j < ResearchSensorProcessingGroups_[ResearchGroupKeys_[i]].size(); j++) {
        if (!ResearchSensorProcessingGroups_[ResearchGroupKeys_[i]][j]->Initialized()) {
          initialized = false;
        }
      }
    }
    if (initialized) {
      InitializedLatch_ = true;
    }
    return initialized;
  }
}

/* sets the sensor processing group to output */
void SensorProcessing::SetEngagedSensorProcessing(std::string EngagedSensorProcessing) {
  EngagedGroup_ = EngagedSensorProcessing;
}

/* computes sensor processing data */
void SensorProcessing::Run() {
  if (EngagedGroup_ == "Baseline") {
    // running baseline sensor processing
    for (size_t i=0; i < BaselineSensorProcessing_.size(); i++) {
      BaselineSensorProcessing_[i]->Run(SensorProcessingFunctionClass::kEngage);
    }
    // running research sensor processing
    for (size_t i=0; i < ResearchGroupKeys_.size(); i++) {
      for (size_t j=0; j < ResearchSensorProcessingGroups_[ResearchGroupKeys_[i]].size(); j++) {
        ResearchSensorProcessingGroups_[ResearchGroupKeys_[i]][j]->Run(SensorProcessingFunctionClass::kArm);
      }
    }
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
    // running baseline sensor processing
    for (size_t i=0; i < BaselineSensorProcessing_.size(); i++) {
      BaselineSensorProcessing_[i]->Run(SensorProcessingFunctionClass::kArm);
    }
    // running research sensor processing
    for (size_t i=0; i < ResearchGroupKeys_.size(); i++) {
      if (ResearchGroupKeys_[i] == EngagedGroup_) {
        for (size_t j=0; j < ResearchSensorProcessingGroups_[ResearchGroupKeys_[i]].size(); j++) {
          ResearchSensorProcessingGroups_[ResearchGroupKeys_[i]][j]->Run(SensorProcessingFunctionClass::kEngage);
        }
      } else {
        for (size_t j=0; j < ResearchSensorProcessingGroups_[ResearchGroupKeys_[i]].size(); j++) {
          ResearchSensorProcessingGroups_[ResearchGroupKeys_[i]][j]->Run(SensorProcessingFunctionClass::kArm);
        }
      }
    }
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
