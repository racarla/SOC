
#ifndef CONTROL_ALGORITHMS_HXX_
#define CONTROL_ALGORITHMS_HXX_

#include "generic-function.hxx"

class __PIDClass {
  public:
    void Configure(float Kp,float Ki,float Kd,float Tf,float b,float c,bool Sat,float UL,float LL);
    void Run(GenericFunction::Mode mode,float Reference,float Feedback,float dt,float *Output,int8_t *Saturated);
    void Clear();
  private:
    struct Config {
      float Kp = 0.0f;
      float Ki = 0.0f;
      float Kd = 0.0f;
      float Tf = 0.0f;
      float b = 1.0f;
      float c = 1.0f;
      bool SaturateOutput = false;
      float UpperLimit = 0.0f;
      float LowerLimit = 0.0f;
    };
    struct Data {
      uint8_t Mode = GenericFunction::Mode::kStandby;
      float Output = 0.0f;
      int8_t Saturated = 0;
    };
    struct States {
      float ProportionalError = 0.0f;
      float DerivativeError = 0.0f;
      float PreviousDerivativeError = 0.0f;
      float IntegralError = 0.0f;
      float DerivativeErrorState = 0.0f;
      float IntegralErrorState = 0.0f;
    };
    Config config_;
    Data data_;
    States states_;
    std::string ReferenceKey_,FeedbackKey_,SampleTimeKey_,ModeKey_,SaturatedKey_,OutputKey_;
    void InitializeState(float Command);
    void UpdateState(float dt);
    void CalculateCommand();
};

#endif
