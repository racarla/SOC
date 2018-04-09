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

void BaselineAirData::UpdateConfiguration(const rapidjson::Value& BaselineAirDataConfig) {
  // get static pressure configuration
  if (BaselineAirDataConfig.HasMember("StaticPressure")) {
    const rapidjson::Value& StaticPressureSources = BaselineAirDataConfig["StaticPressure"];
    for (size_t i=0; i < StaticPressureSources.Size(); i++) {
      const rapidjson::Value& StaticPressureSource = StaticPressureSources[i];
      assert(StaticPressureSource.HasMember("Source"));
      config_.StaticPressureSourceName.push_back(StaticPressureSource["Source"].GetString());
    }
  }
  // get differential pressure configuration
  if (BaselineAirDataConfig.HasMember("DifferentialPressure")) {
    const rapidjson::Value& DifferentialPressureSources = BaselineAirDataConfig["DifferentialPressure"];
    for (size_t i=0; i < DifferentialPressureSources.Size(); i++) {
      const rapidjson::Value& DifferentialPressureSource = DifferentialPressureSources[i];
      assert(DifferentialPressureSource.HasMember("Source"));
      config_.DifferentialPressureSourceName.push_back(DifferentialPressureSource["Source"].GetString());
    }
  }

}

void BaselineAirData::RegisterGlobalData(DefinitionTree *DefinitionTreePtr) {
  // grabbing static pressure sources
  for (size_t i=0; i < config_.StaticPressureSourceName.size(); i++) {
    config_.StaticPressureSourcePtr.push_back(DefinitionTreePtr->GetValue<float>(config_.StaticPressureSourceName[i]));
  }
  // grabbing differential pressure sources
  for (size_t i=0; i < config_.DifferentialPressureSourceName.size(); i++) {
    config_.DifferentialPressureSourcePtr.push_back(DefinitionTreePtr->GetValue<float>(config_.DifferentialPressureSourceName[i]));
  }
  // if we have at least one static pressure source
  if (config_.StaticPressureSourcePtr.size() > 0) {
    // register processed static pressure
    
    // register pressure altitude

    // register AGL altitude
  }
}

void BaselineAirData::Run() {
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


  // remove all diff pressure biases and avg

}

void SensorProcessing::Begin() {

}

void SensorProcessing::UpdateConfiguration(const rapidjson::Value& SensorConfig) {
  // configuring baseline airdata
  if (SensorConfig.HasMember("BaselineAirData")) {
    BaselineAirData TempBaselineAirData;
    TempBaselineAirData.UpdateConfiguration(SensorConfig["BaselineAirData"]);
    classes_.Baselineairdata.push_back(TempBaselineAirData);
  }
}

void SensorProcessing::RegisterGlobalData(DefinitionTree *DefinitionTreePtr) {
  // registering baseline airdata with global data
  for (size_t i=0; i < classes_.Baselineairdata.size(); i++) {
    classes_.Baselineairdata[i].RegisterGlobalData(DefinitionTreePtr);
  }
}

bool SensorProcessing::Initialized() {

}

void SensorProcessing::Run() {
  // running baseline airdata
  for (size_t i=0; i < classes_.Baselineairdata.size(); i++) {
    classes_.Baselineairdata[i].Run();
  }
}
