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

/* configure baseline airdata from JSON and global data definition tree */
void BaselineAirDataClass::Configure(const rapidjson::Value& Config,DefinitionTree *DefinitionTreePtr) {
  std::string OutputName;
  if (Config.HasMember("Output-Name")) {
    OutputName = Config["Output-Name"].GetString();
  } else {
    throw std::runtime_error("Baseline air data output name not specified.");
  }
  // get static pressure configuration
  if (Config.HasMember("Static-Pressure")) {
    const rapidjson::Value& StaticPressureSources = Config["Static-Pressure"];
    for (size_t i=0; i < StaticPressureSources.Size(); i++) {
      const rapidjson::Value& StaticPressureSource = StaticPressureSources[i];
      config_.StaticPressureSourcePtr.push_back(DefinitionTreePtr->GetValuePtr<float*>(StaticPressureSource.GetString()));
    }
  }
  // get differential pressure configuration
  if (Config.HasMember("Differential-Pressure")) {
    const rapidjson::Value& DifferentialPressureSources = Config["Differential-Pressure"];
    for (size_t i=0; i < DifferentialPressureSources.Size(); i++) {
      const rapidjson::Value& DifferentialPressureSource = DifferentialPressureSources[i];
      config_.DifferentialPressureSourcePtr.push_back(DefinitionTreePtr->GetValuePtr<float*>(DifferentialPressureSource.GetString()));
    }
  }
  // get temperature configuration
  if (Config.HasMember("Temperature")) {
    const rapidjson::Value& TemperatureSources = Config["Temperature"];
    for (size_t i=0; i < TemperatureSources.Size(); i++) {
      const rapidjson::Value& TemperatureSource = TemperatureSources[i];
      config_.TemperatureSourcePtr.push_back(DefinitionTreePtr->GetValuePtr<float*>(TemperatureSource.GetString()));
    }
  }
  // get MSL altitude configuration
  if (Config.HasMember("MSL-Altitude")) {
    const rapidjson::Value& MslAltSources = Config["MSL-Altitude"];
    for (size_t i=0; i < MslAltSources.Size(); i++) {
      const rapidjson::Value& MslAltSource = MslAltSources[i];
      config_.MslAltSourcePtr.push_back(DefinitionTreePtr->GetValuePtr<float*>(MslAltSource.GetString()));
    }
  }
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
    DefinitionTreePtr->InitMember(OutputName+"/Airspeed/Equivalent_ms",&data_.EAS_ms,"Equivalent airspeed, m/s",true,false);
  }
  // if at least one static pressure and one differential pressure source, register: equivalent airspeed
  if ((config_.DifferentialPressureSourcePtr.size() > 0)&&(config_.StaticPressureSourcePtr.size() > 0)) {
    DefinitionTreePtr->InitMember(OutputName+"/Airspeed/Equivalent_ms",&data_.EAS_ms,"Equivalent airspeed, m/s",true,false);
  }

  // if at least one static pressure and one temperature source, register: temperature, density, and density altitude
  if ((config_.TemperatureSourcePtr.size() > 0)&&(config_.StaticPressureSourcePtr.size() > 0)) {
    DefinitionTreePtr->InitMember(OutputName+"/Temperature_C",&data_.Temperature_C,"Estimated outside air temperature, C",true,false);
    DefinitionTreePtr->InitMember(OutputName+"/Density_kgm3",&data_.Density_kgm3,"Estimated air density, kg/m^3",true,false);
    DefinitionTreePtr->InitMember(OutputName+"/Altitude/Density_m",&data_.DensityAltitude_m,"Density altitude, m",true,false);
  }
  // if at least one differential pressure and one temperature source, register: true airspeed
  if ((config_.TemperatureSourcePtr.size() > 0)&&(config_.DifferentialPressureSourcePtr.size() > 0)) {
    DefinitionTreePtr->InitMember(OutputName+"/Airspeed/True_ms",&data_.TAS_ms,"True airspeed, m/s",true,false);
  }
  // if at least one static pressure and one MSL altitude source, register: MSL altitude
  if ((config_.MslAltSourcePtr.size() > 0)&&(config_.StaticPressureSourcePtr.size() > 0)) {
    DefinitionTreePtr->InitMember(OutputName+"/Altitude/MSL_m",&data_.MSL_m,"MSL altitude, m",true,false);
  }
}

bool BaselineAirDataClass::Initialized() {

// Initialize based on time from config??
  static size_t NumberSamples = 1;
  // if we have at least one static pressure source
  if (config_.StaticPressureSourcePtr.size() > 0) {
    float AvgPress = 0;
    for (size_t i=0; i < config_.StaticPressureSourcePtr.size(); i++) {
      AvgPress += *config_.StaticPressureSourcePtr[i]/config_.StaticPressureSourcePtr.size();
    }
    // get the initial pressure altitude
    config_.InitialPressureAlt_m = config_.InitialPressureAlt_m+(airdata_->getPressureAltitude(AvgPress)-config_.InitialPressureAlt_m)/((float)NumberSamples);
  }
  // if at least one static pressure and one temperature source
  if ((config_.TemperatureSourcePtr.size() > 0)&&(config_.StaticPressureSourcePtr.size() > 0)) {
    float AvgTemp = 0;
    for (size_t i=0; i < config_.TemperatureSourcePtr.size(); i++) {
      AvgTemp += *config_.TemperatureSourcePtr[i]/config_.TemperatureSourcePtr.size();
    }
    // get the initial outside air temperature
    config_.InitialTemperature_C = config_.InitialTemperature_C+(AvgTemp-config_.InitialTemperature_C)/((float)NumberSamples);
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
  // if time > duration return true and reset numbersamples
}

void BaselineAirDataClass::Run() {
  // if we have at least one static pressure source
  if (config_.StaticPressureSourcePtr.size() > 0) {
    // average all static pressure sources
    data_.StaticPressure_Pa = 0;
    for (size_t i=0; i < config_.StaticPressureSourcePtr.size(); i++) {
      data_.StaticPressure_Pa += *config_.StaticPressureSourcePtr[i]/config_.StaticPressureSourcePtr.size();
    }
    // compute pressure altitude
    data_.PressureAltitude_m = airdata_->getPressureAltitude(data_.StaticPressure_Pa);
    // compute AGL altitude
    data_.AGL_m = airdata_->getAGL(data_.StaticPressure_Pa,config_.InitialPressureAlt_m);
  }
  // if we have at least one differential pressure source
  if (config_.DifferentialPressureSourcePtr.size() > 0) {
    // remove all diff pressure biases and average
    data_.DifferentialPressure_Pa = 0;
    for (size_t i=0; i < config_.DifferentialPressureSourcePtr.size(); i++) {
      data_.DifferentialPressure_Pa += *config_.DifferentialPressureSourcePtr[i]/config_.DifferentialPressureSourcePtr.size();
    }
    // compute indicated airspeed
    data_.IAS_ms = airdata_->getIAS(data_.DifferentialPressure_Pa);
  }
  // if at least one static pressure and one differential pressure source
  if ((config_.DifferentialPressureSourcePtr.size() > 0)&&(config_.StaticPressureSourcePtr.size() > 0)) {
    // compute equivalent airspeed
    data_.EAS_ms = airdata_->getEAS(data_.DifferentialPressure_Pa,data_.StaticPressure_Pa);
  }
  // if at least one static pressure and one temperature source
  if ((config_.TemperatureSourcePtr.size() > 0)&&(config_.StaticPressureSourcePtr.size() > 0)) {
    // compute outside air temperature
    data_.Temperature_C = airdata_->getApproxTemp(config_.InitialTemperature_C,data_.AGL_m);
    // compute air density
    data_.Density_kgm3 = airdata_->getDensity(data_.StaticPressure_Pa,data_.Temperature_C);
    // compute density altitude
    data_.DensityAltitude_m = airdata_->getDensityAltitude(data_.StaticPressure_Pa,data_.Temperature_C);
  }
  // if at least one differential pressure and one temperature source, register: true airspeed
  if ((config_.TemperatureSourcePtr.size() > 0)&&(config_.DifferentialPressureSourcePtr.size() > 0)) {
    // compute true airspeed
    data_.TAS_ms = airdata_->getTAS(data_.EAS_ms,data_.Temperature_C);
  }
  // if at least one static pressure and one MSL altitude source
  if ((config_.MslAltSourcePtr.size() > 0)&&(config_.StaticPressureSourcePtr.size() > 0)) {
    // compute MSL altitude
    data_.MSL_m = airdata_->getMSL(data_.AGL_m,config_.InitialMSLAlt_m);
  }
}

void SensorProcessing::Configure(const rapidjson::Value& Config, DefinitionTree *DefinitionTreePtr) {
  // configuring baseline air data
  if (Config.HasMember("Baseline")) {
    const rapidjson::Value& BaselineConfig = Config["Baseline"];
    for (size_t i=0; i < BaselineConfig.Size(); i++) {
      if (BaselineConfig[i].HasMember("Type")) {
        if (BaselineConfig[i]["Type"] == "BaselineAirData") {
          BaselineAirDataClass Temp;
          Temp.Configure(BaselineConfig[i],DefinitionTreePtr);
          BaselineSensorProcessing.push_back(Temp);
        }
      }
    }
  }
  // configuring research air data
  if (Config.HasMember("Research")) {
    const rapidjson::Value& ResearchConfig = Config["Research"];
    for (size_t i=0; i < ResearchConfig.Size(); i++) {
      std::cout << ResearchConfig.Size() << std::endl;
      if (ResearchConfig[i].HasMember("Group-Name")&&ResearchConfig[i].HasMember("Components")) {
        std::cout << "here?" << std::endl;
        for (size_t j=0; j < ResearchConfig[i]["Components"].Size(); j++) {
          std::cout << "67567" << std::endl;
          if (ResearchConfig[i]["Components"][j].HasMember("Type")) {
            std::cout << "1231" << std::endl;
            if (ResearchConfig[i]["Components"][j]["Type"] == "BaselineAirData") {
              std::cout << "hello?" << std::endl;
              BaselineAirDataClass Temp;
              Temp.Configure(ResearchConfig[i]["Components"][j],DefinitionTreePtr);
              ResearchSensorProcessingGroups[ResearchConfig[i]["Group-Name"].GetString()].push_back(Temp);
            }
          }
        }
      }
    }
  }

  std::cout << "REsearch" <<std::endl;
  std::cout << ResearchSensorProcessingGroups.size() << std::endl;
  for (auto const& element : ResearchSensorProcessingGroups) {
    std::cout << element.first << std::endl;
  }
}

bool SensorProcessing::Initialized() {
  bool initialized = true;
  // initializing baseline airdata
  // for (size_t i=0; i < classes_.BaselineAirData.size(); i++) {
  //   if(!classes_.BaselineAirData[i].Initialized()) {
  //     initialized = false;
  //   }
  // }
  return initialized;
}

void SensorProcessing::Run() {
  // running baseline airdata
  // for (size_t i=0; i < classes_.BaselineAirData.size(); i++) {
  //   classes_.BaselineAirData[i].Run();
  // }
}
