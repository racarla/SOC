
#include "control-algorithms.hxx"

void __PIDClass::Configure(float Kp,float Ki,float Kd,float Tf,float b,float c,bool Sat,float UL,float LL) {
  config_.Kp = Kp;
  config_.Kd = Kd;
  config_.Ki = Ki;
  config_.Tf = Tf;
  config_.b = b;
  config_.c = c;
  config_.SaturateOutput = Sat;
  config_.UpperLimit = UL;
  config_.LowerLimit = LL;
}

void __PIDClass::Run(GenericFunction::Mode mode,float Reference,float Feedback,float dt,float *Output,int8_t *Saturated) {
  // error for proportional
  states_.ProportionalError = (config_.b*Reference)-Feedback;
  // error for integral
  states_.IntegralError = Reference-Feedback;
  // error for derivative
  states_.DerivativeError = (config_.c*Reference)-Feedback;
  // derivative of Error
  if (dt > 0.0f) {
    states_.DerivativeErrorState = 1.0f/(config_.Tf+dt/(states_.DerivativeError-states_.PreviousDerivativeError));
  }
  states_.PreviousDerivativeError = states_.DerivativeError;
  switch(mode) {
    case GenericFunction::Mode::kStandby: {
      break;
    }
    case GenericFunction::Mode::kArm: {
      InitializeState(0.0f);
      CalculateCommand();
      break;
    }
    case GenericFunction::Mode::kHold: {
      CalculateCommand();
      break;
    }
    case GenericFunction::Mode::kEngage: {
      UpdateState(dt);
      CalculateCommand();
      break;
    }
  }
  *Output = data_.Output;
  *Saturated = data_.Saturated;
}

void __PIDClass::InitializeState(float Command) {
  // Protect for Ki == 0
  if (config_.Ki != 0.0f) {
    states_.IntegralErrorState = (Command-(config_.Kp*states_.ProportionalError+config_.Kd*states_.DerivativeErrorState))/config_.Ki;
  } else {
    states_.IntegralErrorState = 0.0f;
  }
}

void __PIDClass::UpdateState(float dt) {
  // Protect for unlimited windup when Ki == 0
  if (config_.Ki != 0.0f) {
    states_.IntegralErrorState += (dt*states_.IntegralError);
  } else {
    states_.IntegralErrorState = 0.0;
  }
}

void __PIDClass::CalculateCommand() {
  float ProportionalCommand = config_.Kp*states_.ProportionalError;
  float IntegralCommand = config_.Ki*states_.IntegralErrorState;
  float DerivativeCommand = config_.Kd*states_.DerivativeErrorState;
  data_.Output = ProportionalCommand+IntegralCommand+DerivativeCommand;
  // saturate cmd, set iErr to limit that produces saturated cmd
  // saturate command
  if (config_.SaturateOutput) {
    if (data_.Output <= config_.LowerLimit) {
      data_.Output = config_.LowerLimit;
      data_.Saturated = -1;
      // Re-compute the integrator state
      InitializeState(data_.Output);
    } else if (data_.Output >= config_.UpperLimit) {
      data_.Output = config_.UpperLimit;
      data_.Saturated = 1;
      // Re-compute the integrator state
      InitializeState(data_.Output);
    } else {
      data_.Saturated = 0;
    }
  }
}

void __PIDClass::Clear() {
  config_.Kp = 0.0f;
  config_.Ki = 0.0f;
  config_.Kd = 0.0f;
  config_.Tf = 0.0f;
  config_.b = 1.0f;
  config_.c = 1.0f;
  config_.SaturateOutput = false;
  config_.UpperLimit = 0.0f;
  config_.LowerLimit = 0.0f;
  states_.ProportionalError = 0.0f;
  states_.DerivativeError = 0.0f;
  states_.PreviousDerivativeError = 0.0f;
  states_.IntegralError = 0.0f;
  states_.DerivativeErrorState = 0.0f;
  states_.IntegralErrorState = 0.0f;
  data_.Saturated = 0;
  data_.Output = 0.0f;
}
