/*
Classes and functions for Control Functions

See: LICENSE.md for Copyright and License Agreement
*/

#include <iostream>

#include "ctrlFunc.hxx"


// PID2 Controller, Set Parameters for the tunable PID2 controller
// Parameters match the PID2 system in MATLAB

// u = Kp*pErr + Ki*iErr + Kd*dErr
// pErrState = pErr = (b*r - y)
// iErrState = 1/s * (r-y) = 1/s * iErr
// dErrState = (s/(Tf*s + 1))*(c*r - y) = (1/(Tf + 1/s))*dErr

// Discretized:
// iErrState += (dt_s * iErr);
// dErrState = 1 / (Tf + (dt_s / (dErr - dErrPrev));
void CtrlFuncPid2::Config(const float &Kp, const float &Ki, const float &Kd, const float &Tf, const float &b, const float &c, const float &cmdMin, const float &cmdMax)
{
  mode_ = kCtrlStandby; // Initialize in Standby

  Kp_ = Kp;
  Ki_ = Ki;
  Kd_ = Kd;

  Tf_ = Tf;

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
    dErrState = 1 / (Tf_ + dt_s / (dErr - dErrPrev_));
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

// Damper Controller
// FIXIT - Copied syntax from Goldy1, should make continuous form with simple Euler derivative assumption for discrete
void CtrlFuncDamp::Config(const float &b0, const float &b1, const float &a0, const float &a1, const float &cmdMin, const float &cmdMax)
{
  mode_ = kCtrlReset; // Initialize in Reset, which dumps to Standby

  b0_ = b0;
  b1_ = b1;
  a0_ = a0;
  a1_ = a1;

  cmdMin_ = cmdMin;
  cmdMax_ = cmdMax;

  refPrev_ = 0.0; // Set previous reference to zero
  cmdPrev_ = 0.0; // Set previous cmd to zero
}

void CtrlFuncDamp::Run(const float &ref, const float &dt_s, float *cmd)
{

  *cmd = 0.0;

  switch(mode_) {
    case kCtrlReset: // Zero the State and Command
      refPrev_ = 0.0;
      cmdPrev_ = 0.0;
      *cmd = 0.0;

      mode_ = kCtrlStandby;
      break;

    case kCtrlStandby: // Do Nothing, State and Command are unchanged
      break;

    case kCtrlHold: // Run Commands
      CalcCmd(ref, cmd); // returns cmd
      refPrev_ = ref;
      break;

    case kCtrlInit: // Run Commands
      InitState(ref, *cmd); // sets cmdPrev_
      CalcCmd(ref, cmd); // returns cmd
      refPrev_ = ref;
      break;

    case kCtrlEngage: // Update the State then Run Commands
      CalcCmd(ref, cmd); // returns cmd  // Update the states
      refPrev_ = ref;
      cmdPrev_ = *cmd;
      break;
  }
}

// Initialize for near-zero transient
void CtrlFuncDamp::InitState(const float &ref, const float &cmd)
{
  if (a1_ != 0.0) {
    cmdPrev_ = 1/a1_ * (b0_ * ref + b1_ * refPrev_ - a0_ * cmd);
  }
}

// Run the Command
void CtrlFuncDamp::CalcCmd(const float &ref, float *cmd)
{
  if (a0_ != 0.0) {
    *cmd = 1/a0_ * (b0_ * ref + b1_ * refPrev_ - a1_ * cmdPrev_);
  }

  // saturate cmd, maintain state at theshold
  if (*cmd <= cmdMin_) {
    *cmd = cmdMin_;
    InitState(ref, *cmd);
  } else if (*cmd >= cmdMax_) {
    *cmd = cmdMax_;
    InitState(ref, *cmd);
  }
}
