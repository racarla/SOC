/*
airdata-functions.cc
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

#include "airdata-functions.hxx"


// /* base function class methods */
// void SensorProcessingFunctionClass::Configure(const rapidjson::Value& Config,std::string RootPath,DefinitionTree *DefinitionTreePtr) {}
// bool SensorProcessingFunctionClass::Initialized() {}
// void SensorProcessingFunctionClass::Run(Mode mode) {}

// /* configure baseline airdata from JSON and global data definition tree */
// void BaselineAirDataClass::Configure(const rapidjson::Value& Config,std::string RootPath,DefinitionTree *DefinitionTreePtr) {
//   std::string OutputName;
//   // get output name
//   if (Config.HasMember("Output")) {
//     OutputName = RootPath + "/" + Config["Output"].GetString();
//   } else {
//     throw std::runtime_error(std::string("ERROR")+RootPath+std::string(": Output not specified in configuration."));
//   }
//   // get time source
//   if (Config.HasMember("Time")) {
//     if (DefinitionTreePtr->GetValuePtr<uint64_t*>(Config["Time"].GetString())) {
//       config_.TimeSourcePtr = DefinitionTreePtr->GetValuePtr<uint64_t*>(Config["Time"].GetString());
//     } else {
//       throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Time source ")+Config["Time"].GetString()+std::string(" not found in global data."));
//     }
//   } else {
//     throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Time source not specified in configuration."));
//   }
//   // get initialization time
//   if (Config.HasMember("Initialization-Time")) {
//     config_.InitializationTime_s = Config["Initialization-Time"].GetFloat();
//   } else {
//     throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Initialization time not specified in configuration."));
//   }
//   // get static pressure configuration
//   if (Config.HasMember("Static-Pressure")) {
//     const rapidjson::Value& StaticPressureSources = Config["Static-Pressure"];
//     for (size_t i=0; i < StaticPressureSources.Size(); i++) {
//       const rapidjson::Value& StaticPressureSource = StaticPressureSources[i];
//       std::string source = StaticPressureSource.GetString() + std::string("/Pressure_Pa");
//       if (DefinitionTreePtr->GetValuePtr<float*>(source)) {
//         config_.StaticPressureSourcePtr.push_back(DefinitionTreePtr->GetValuePtr<float*>(source));
//       } else {
//         throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Static pressure source ")+source+std::string(" not found in global data."));
//       }
//     }
//   }
//   // get differential pressure configuration
//   if (Config.HasMember("Differential-Pressure")) {
//     const rapidjson::Value& DifferentialPressureSources = Config["Differential-Pressure"];
//     for (size_t i=0; i < DifferentialPressureSources.Size(); i++) {
//       const rapidjson::Value& DifferentialPressureSource = DifferentialPressureSources[i];
//       std::string source = DifferentialPressureSource.GetString() + std::string("/Pressure_Pa");
//       if (DefinitionTreePtr->GetValuePtr<float*>(source)) {
//         config_.DifferentialPressureSourcePtr.push_back(DefinitionTreePtr->GetValuePtr<float*>(source));
//       } else {
//         throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Differential pressure source ")+source+std::string(" not found in global data."));
//       }

//     }
//     config_.DifferentialPressureBiases.resize(DifferentialPressureSources.Size());
//   }
//   // get MSL altitude configuration
//   if (Config.HasMember("MSL-Altitude")) {
//     const rapidjson::Value& MslAltSources = Config["MSL-Altitude"];
//     for (size_t i=0; i < MslAltSources.Size(); i++) {
//       const rapidjson::Value& MslAltSource = MslAltSources[i];
//       std::string source = MslAltSource.GetString() + std::string("/Altitude_m");
//       std::string fix = MslAltSource.GetString() + std::string("/Fix");
//       if (DefinitionTreePtr->GetValuePtr<float*>(source)) {
//         config_.MslAltSourcePtr.push_back(DefinitionTreePtr->GetValuePtr<float*>(source));
//       } else {
//         throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": MSL altitude source ")+source+std::string(" not found in global data."));
//       }
//       if (DefinitionTreePtr->GetValuePtr<uint8_t*>(fix)) {
//         config_.MslAltFixPtr.push_back(DefinitionTreePtr->GetValuePtr<uint8_t*>(fix));
//       } else {
//         throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": MSL altitude fix ")+fix+std::string(" not found in global data."));
//       }
//     }
//   }
//   // pointer to log run mode data
//   DefinitionTreePtr->InitMember(OutputName+"/Mode",&data_.Mode,"Sensor processing mode",true,false);
//   // if at least one static pressure source, register: pressure altitude and AGL altitude
//   if (config_.StaticPressureSourcePtr.size() > 0) {
//     DefinitionTreePtr->InitMember(OutputName+"/Pressure/Static_Pa",&data_.StaticPressure_Pa,"Static pressure, Pa",true,false);
//     DefinitionTreePtr->InitMember(OutputName+"/Altitude/Pressure_m",&data_.PressureAltitude_m,"Pressure altitude, m",true,false);
//     DefinitionTreePtr->InitMember(OutputName+"/Altitude/AGL_m",&data_.AGL_m,"AGL altitude, m",true,false);
//   }
//   // if at least one differential pressure source, register: indicated airspeed
//   if (config_.DifferentialPressureSourcePtr.size() > 0) {
//     DefinitionTreePtr->InitMember(OutputName+"/Pressure/Differential_Pa",&data_.DifferentialPressure_Pa,"Differential pressure, Pa",true,false);
//     DefinitionTreePtr->InitMember(OutputName+"/Airspeed/Indicated_ms",&data_.IAS_ms,"Indicated airspeed, m/s",true,false);
//   }
//   // if at least one static pressure and one MSL altitude source, register: MSL altitude
//   if ((config_.MslAltSourcePtr.size() > 0)&&(config_.StaticPressureSourcePtr.size() > 0)) {
//     DefinitionTreePtr->InitMember(OutputName+"/Altitude/MSL_m",&data_.MSL_m,"MSL altitude, m",true,false);
//   }
// }

// /* initializes air data */
// bool BaselineAirDataClass::Initialized() {
//   if (InitializedLatch_) {
//     return true;
//   } else {
//     bool GpsFix = true;
//     for (size_t i=0; i < config_.MslAltFixPtr.size(); i++) {
//       if (!*config_.MslAltFixPtr[i]) {
//         GpsFix = false;
//       }
//     }
//     if ((config_.MslAltSourcePtr.size() == 0)||(GpsFix)) {
//       if (!TimeLatch_) {
//         Time0_us_ = *config_.TimeSourcePtr;
//         TimeLatch_ = true;
//       }
//       InitializationTimer_us_ = (float)(*config_.TimeSourcePtr - Time0_us_);
//       static size_t NumberSamples = 1;
//       // if we have at least one static pressure source
//       if (config_.StaticPressureSourcePtr.size() > 0) {
//         float AvgPress = 0;
//         for (size_t i=0; i < config_.StaticPressureSourcePtr.size(); i++) {
//           AvgPress += *config_.StaticPressureSourcePtr[i]/config_.StaticPressureSourcePtr.size();
//         }
//         // get the initial pressure altitude
//         config_.InitialPressureAlt_m = config_.InitialPressureAlt_m+(airdata_.getPressureAltitude(AvgPress)-config_.InitialPressureAlt_m)/((float)NumberSamples);
//       }
//       // if we have at least one differential pressure source
//       if (config_.DifferentialPressureSourcePtr.size() > 0) {
//         for (size_t i=0; i < config_.DifferentialPressureSourcePtr.size(); i++) {
//           config_.DifferentialPressureBiases[i] = config_.DifferentialPressureBiases[i] + (*config_.DifferentialPressureSourcePtr[i]-config_.DifferentialPressureBiases[i])/((float)NumberSamples);
//         }
//       }
//       // if at least one static pressure and one MSL altitude source
//       if ((config_.MslAltSourcePtr.size() > 0)&&(config_.StaticPressureSourcePtr.size() > 0)) {
//         float AvgMSL = 0;
//         for (size_t i=0; i < config_.MslAltSourcePtr.size(); i++) {
//           AvgMSL += *config_.MslAltSourcePtr[i]/config_.MslAltSourcePtr.size();
//         }
//         // get the initial MSL altitude
//         config_.InitialMSLAlt_m = config_.InitialMSLAlt_m+(AvgMSL-config_.InitialMSLAlt_m)/((float)NumberSamples);
//       }
//       NumberSamples++;
//       // if time > duration return true
//       if (InitializationTimer_us_ > config_.InitializationTime_s*1e6) {
//         InitializedLatch_ = true;
//         return true;
//       } else {
//         return false;
//       }
//     } else {
//       return false;
//     }
//   }
// }

// /* compute air data values */
// void BaselineAirDataClass::Run(Mode mode) {
//   data_.Mode = (uint8_t) mode;
//   // if we have at least one static pressure source
//   if (config_.StaticPressureSourcePtr.size() > 0) {
//     // average all static pressure sources
//     data_.StaticPressure_Pa = 0;
//     for (size_t i=0; i < config_.StaticPressureSourcePtr.size(); i++) {
//       data_.StaticPressure_Pa += *config_.StaticPressureSourcePtr[i]/config_.StaticPressureSourcePtr.size();
//     }
//     // compute pressure altitude
//     data_.PressureAltitude_m = airdata_.getPressureAltitude(data_.StaticPressure_Pa);
//     // compute AGL altitude
//     data_.AGL_m = airdata_.getAGL(data_.StaticPressure_Pa,config_.InitialPressureAlt_m);
//   }
//   // if we have at least one differential pressure source
//   if (config_.DifferentialPressureSourcePtr.size() > 0) {
//     // remove all diff pressure biases and average
//     data_.DifferentialPressure_Pa = 0;
//     for (size_t i=0; i < config_.DifferentialPressureSourcePtr.size(); i++) {
//       data_.DifferentialPressure_Pa += (*config_.DifferentialPressureSourcePtr[i]-config_.DifferentialPressureBiases[i])/config_.DifferentialPressureSourcePtr.size();
//     }
//     // compute indicated airspeed
//     data_.IAS_ms = airdata_.getIAS(data_.DifferentialPressure_Pa);
//   }
//   // if at least one static pressure and one MSL altitude source
//   if ((config_.MslAltSourcePtr.size() > 0)&&(config_.StaticPressureSourcePtr.size() > 0)) {
//     // compute MSL altitude
//     data_.MSL_m = airdata_.getMSL(data_.AGL_m,config_.InitialMSLAlt_m);
//   }
// }