/*
control-functions.hxx
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

#ifndef CONTROL_FUNCTIONS_HXX_
#define CONTROL_FUNCTIONS_HXX_

#include "rapidjson/document.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/writer.h"
#include "definition-tree.hxx"
#include "generic-function.hxx"

/* Control related functions. Each function describes its JSON
configuration below. See generic-function.hxx for more information
on the methods and modes. */

/* 
Constant Class - Outputs a constant value.
Example JSON configuration:
{
  "Output": "OutputName",
  "Constant": X
}
Where: 
   * Output gives a convenient name for the block (i.e. SpeedReference).
   * Constant is the value of the constant output.
Data type for the output is float.
*/
class ConstantClass: public GenericFunction {
  public:
    void Configure(const rapidjson::Value& Config,std::string RootPath,DefinitionTree *DefinitionTreePtr);
    void Initialize();
    bool Initialized();
    void Run(Mode mode);
    void Clear(DefinitionTree *DefinitionTreePtr);
  private:
    struct Config {
      float Constant = 0.0f;
    };
    struct Data {
      uint8_t Mode = kStandby;
      float Output = 0.0f;
    };
    Config config_;
    Data data_;
    std::string ModeKey_,OutputKey_;
};

/* 
Gain Class - Multiplies an input by a gain
Example JSON configuration:
{
  "Output": "OutputName",
  "Input": "InputName",
  "Gain": X,
  "Limits": {
    "Upper": X,
    "Lower": X
  }
}
Where: 
   * Output gives a convenient name for the block (i.e. SpeedControl).
   * Input is the full path name of the input signal.
   * Gain is the gain applied to the input signal.
   * Limits are optional and saturate the output if defined.
Data types for the input and output are both float.
*/
class GainClass: public GenericFunction {
  public:
    void Configure(const rapidjson::Value& Config,std::string RootPath,DefinitionTree *DefinitionTreePtr);
    void Initialize();
    bool Initialized();
    void Run(Mode mode);
    void Clear(DefinitionTree *DefinitionTreePtr);
  private:
    struct Config {
      float *Input;
      float Gain = 1.0f;
      bool SaturateOutput = false;
      float UpperLimit, LowerLimit = 0.0f;
    };
    struct Data {
      uint8_t Mode = kStandby;
      float Output = 0.0f;
      int8_t Saturated = 0;
    };
    Config config_;
    Data data_;
    std::string InputKey_,ModeKey_,SaturatedKey_,OutputKey_;
};

/* 
Sum Class - Sums all inputs
Example JSON configuration:
{
  "Output": "OutputName",
  "Inputs": ["InputName1","InputName2",...],
  "Limits": {
    "Upper": X,
    "Lower": X
  }
}
Where: 
   * Output gives a convenient name for the block (i.e. SpeedReference).
   * Inputs is a vector of full path names of the input signals. All inputs
     will be summed.
   * Limits are optional and saturate the output if defined.
Data types for the input and output are both float.
*/
class SumClass: public GenericFunction {
  public:
    void Configure(const rapidjson::Value& Config,std::string RootPath,DefinitionTree *DefinitionTreePtr);
    void Initialize();
    bool Initialized();
    void Run(Mode mode);
    void Clear(DefinitionTree *DefinitionTreePtr);
  private:
    struct Config {
      std::vector<float*> Inputs;
      bool SaturateOutput = false;
      float UpperLimit, LowerLimit = 0.0f;
    };
    struct Data {
      uint8_t Mode = kStandby;
      float Output = 0.0f;
      int8_t Saturated = 0;
    };
    Config config_;
    Data data_;
    std::vector<std::string> InputKeys_;
    std::string ModeKey_,SaturatedKey_,OutputKey_;
};

/* 
PID Class - PID and PID2 control law
Example JSON configuration:
{
  "Output": "OutputName",
  "Reference": "ReferenceName",
  "Feedback": "FeedbackName",
  "Sample-Time": "SampleTime" or X,
  "Time-Constant": X,
  "Gains": {
    "Proportional": Kp,
    "Integral": Ki,
    "Derivative": Kd,
  },
  "Setpoint-Weights": {
    "Proportional": b,
    "Derivative": c
  },
  "Limits": {
    "Upper": X,
    "Lower": X
  }
}
Where: 
   * Output gives a convenient name for the block (i.e. PitchControl).
   * Reference is the full path name of the reference signal.
   * Feedback is the full path name of the feedback signal.
   * Sample-Time is either: the full path name of the sample time signal in seconds, 
     or a fixed value sample time in seconds.
   * Time-Constant is the time constant for the derivative filter.
     If a time constant is not specified, then no filtering is used.
   * Gains specifies the proportional derivative and integral gains.
   * Setpoint weights optionally specifies the proportional and derivative setpoint
     weights used in the filter.
   * Limits are optional and saturate the output if defined.
Data types for all input and output values are float.
*/

class PIDClass: public GenericFunction {
  public:
    void Configure(const rapidjson::Value& Config,std::string RootPath,DefinitionTree *DefinitionTreePtr);
    void Initialize();
    bool Initialized();
    void Run(Mode mode);
    void Clear(DefinitionTree *DefinitionTreePtr);
  private:
    struct Config {
      float *Reference;
      float *Feedback;
      float *dt;
      float SampleTime;
      bool UseSampleTime = false;
      float Kp,Ki,Kd,Tf = 0.0f;
      float b,c = 1.0f;
      bool SaturateOutput = false;
      float UpperLimit, LowerLimit = 0.0f;
    };
    struct Data {
      uint8_t Mode = kStandby;
      float Output = 0.0f;
      int8_t Saturated = 0;
    };
    struct States {
      float ProportionalError_, DerivativeError_, PreviousDerivativeError_, IntegralError_ = 0.0f;
      float DerivativeErrorState_, IntegralErrorState_ = 0.0f;
    };
    Config config_;
    Data data_;
    States states_;
    std::string ReferenceKey_,FeedbackKey_,SampleTimeKey_,ModeKey_,SaturatedKey_,OutputKey_; 
    void InitializeState(float Command);
    void UpdateState();
    void CalculateCommand();
};

#endif
