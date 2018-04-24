/*
Classes and functions for Control Functions

See: LICENSE.md for Copyright and License Agreement
*/

#include <iostream>

#include "ctrlFunc.hxx"


// PID2 Controller, Set Parameters for the tunable PID2 controller
void CtrlFuncPid2::Config(const float &Kp, const float &Ki, const float &Kd, const float &b, const float &c, const float &cmdMin, const float &cmdMax)
{
  // Parameters match the PID2 system in MATLAB

  mode_ = kCtrlStandby; // Initialize in Standby

  Kp_ = Kp;
  Ki_ = Ki;
  Kd_ = Kd;

  b_ = b;
  c_ = c;

  cmdMin_ = cmdMin;
  cmdMax_ = cmdMax;

  iErrState_ = 0.0; // Initialize Integrator State
  dErrPrev_ = 0.0; // Set previous error to zero
}

void CtrlFuncPid2::Run(const float &ref, const float &meas, const float &dt_s, float *cmd)
{
  float pErr = (b_*ref) - meas; // Error for Proportional
  float iErr = ref - meas; // Error for Integrator
  float dErr = (c_*ref) - meas; // Error for Derivative

  // Derivative of Error
  float dErrState = 0.0;
  if (dt_s > 0.0 ) {
    dErrState = (dErr - dErrPrev_) / dt_s;
  }

  *cmd = 0.0;

  // Update the previous error state
  dErrPrev_ = dErr;

  switch(mode_) {
    case kCtrlReset: // Zero the State and Command
      iErrState_ = 0.0;
      pErr = 0.0;
      dErrPrev_ = 0.0;
      *cmd = 0.0;

      mode_ = kCtrlStandby;
      break;

    case kCtrlStandby: // Do Nothing, State and Command are unchanged
      break;

    case kCtrlHold: // Run Commands
      CalcCmd(pErr, dErr, cmd); // returns cmd
      break;

    case kCtrlInit: // Initialize State then Run Commands
      InitState(0.0 * iErr, iErr, dErrState, &iErrState_);
      CalcCmd(pErr, dErrState, cmd); // returns cmd
      break;

    case kCtrlEngage: // Update the State then Run Commands
      UpdState(iErr, dt_s, &iErrState_); // Update the state
      CalcCmd(pErr, dErrState, cmd); // returns cmd
      break;
  }
}

// Initialize for near-zero transient
void CtrlFuncPid2::InitState(const float &cmd, const float &pErr, const float &dErrState, float *iErrState)
{
  if (Ki_ != 0.0) { // Protect for Ki == 0
    *iErrState = (cmd - (Kp_ * pErr + Kd_ * dErrState)) / Ki_; // Run the required state
  } else {
    *iErrState = 0.0; // Run the required state
  }
}

// Initialize for near-zero transient
void CtrlFuncPid2::UpdState(const float &iErr, const float &dt_s, float *iErrState)
{
  // Update the state
  if (Ki_ != 0.0) { // Protect for unlimited windup when Ki == 0
    *iErrState += (dt_s * iErr);
  } else {
    *iErrState = 0.0;
  }
}

// Run the Command
void CtrlFuncPid2::CalcCmd(const float &pErr, const float &dErrState, float *cmd)
{
  float pCmd = Kp_ * pErr;
  float iCmd = Ki_ * iErrState_;
  float dCmd = Kd_ * dErrState;

  *cmd = pCmd + iCmd + dCmd;

  // saturate cmd, set iErr to limit that produces saturated cmd
  if (*cmd <= cmdMin_) {
    *cmd = cmdMin_;
    InitState(*cmd, pErr, dErrState, &iErrState_); // Re-compute the integrator state
  } else if (*cmd >= cmdMax_) {
    *cmd = cmdMax_;
    InitState(*cmd, pErr, dErrState, &iErrState_); // Re-compute the integrator state
  }
}
