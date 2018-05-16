/*
airdata-functions.hxx
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

#ifndef AIRDATA_FUNCTIONS_HXX_
#define AIRDATA_FUNCTIONS_HXX_

// class SensorProcessingFunctionClass {
//   public:
//     enum Mode {
//       kArm,
//       kEngage
//     };
//     virtual void Configure(const rapidjson::Value& Config,std::string RootPath,DefinitionTree *DefinitionTreePtr);
//     virtual bool Initialized();
//     virtual void Run(Mode mode);
// };

// class IirFilterClass: public SensorProcessingFunctionClass {

// };

// class BaselineAirDataClass: public SensorProcessingFunctionClass {
//   public:
//     void Configure(const rapidjson::Value& Config,std::string RootPath,DefinitionTree *DefinitionTreePtr);
//     bool Initialized();
//     void Run(Mode mode);
//   private:
//     struct Config {
//       uint64_t* TimeSourcePtr;
//       std::vector<float*> StaticPressureSourcePtr;
//       std::vector<float*> DifferentialPressureSourcePtr;
//       std::vector<float> DifferentialPressureBiases;
//       std::vector<float*> MslAltSourcePtr;
//       std::vector<uint8_t*> MslAltFixPtr;
//       float InitializationTime_s;
//       float InitialPressureAlt_m;
//       float InitialMSLAlt_m;
//     };
//     struct Data {
//       float StaticPressure_Pa;
//       float DifferentialPressure_Pa;
//       float IAS_ms;
//       float PressureAltitude_m;
//       float AGL_m;
//       float MSL_m;
//       uint8_t Mode;
//     };
//     AirData airdata_;
//     Config config_;
//     Data data_;
//     bool InitializedLatch_ = false;
//     bool TimeLatch_ = false;
//     uint64_t Time0_us_;
//     float InitializationTimer_us_;
// };

#endif
