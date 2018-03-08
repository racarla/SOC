/*
Classes and functions for Control Functions

See: LICENSE.md for Copyright and License Agreement

History:
2017-11-12 - Chris Regan - Defined CtrlPiDamp class and methods
2017-11-12 - Chris Regan - Defined CtrlPid class and methods
2017-11-12 - Chris Regan - Defined CtrlDamp class and methods

*/

#include <iostream>

#include "CtrlFunc.hxx"

// PID2 Controller
// Constructor
CtrlPid2::CtrlPid2()
{}

// Set Parameters for the tunable PID2 controller
void CtrlPid2::Config(const float& Kp, const float& Ki, const float& Kd, const float& b, const float& c, const float& refScale, const float& cmdRng)
{
  // Parameters match the PID2 system in MATLAB

  mode_ = kCtrlStandby; // Initialize in Standby

  Kp_ = Kp;
  KI_ = Ki;
  Kd_ = Kd;

  b_ = b;
  c_ = c;

  refScale_ = refScale;
  cmdRng_ = cmdRng;

  iErrState_ = 0.0; // Initialize Integrator State
  dErrPrev_ = 0.0; // Set previous error to zero
}

float CtrlPid2::Run(const float& ref, const float& meas, float& dt_s)
{
  ref = refScale_ * ref; // Scale the reference Signal

  float pErr = (b_*ref) - meas; // Error for Proportional
  float iErr = ref - meas; // Error for Integrator
  float dErr = (c_*ref) - meas; // Error for Derivative

  // Derivative of Error
  if (dt_s > 0.0 ) {
    dErrState = (dErr - dErrPrev_) / dt_s;
  } else {
    dErrState = 0.0;
  }

  float cmd = 0.0;

  // Update the previous error state
  dErrPrev_ = dErr;

  switch(mode_) {
    case kCtrlReset: // Zero the State and Command
      iErrState_ = 0.0;
      pErr = 0.0;
      dErrPrev_ = 0.0;
      cmd = 0.0;

      mode_ = kCtrlStandby;
      break;

    case kCtrlStandby: // Do Nothing, State and Command are unchanged
      break;

    case kCtrlHold: // Run Commands

      cmd = CalcCmd(pErr, dErr);
      break;

    case kCtrlInit: // Initialize State then Run Commands

      InitState(0.0, iErr, dErrState);
      cmd = CalcCmd(pErr, dErrState);
      break;

    case kCtrlEngage: // Update the State then Run Commands

      // Update the state
      UpdState(iErr, dt_s);
      cmd = CalcCmd(pErr, dErrState);
      break;
  }

  return cmd;
}

// Initialize for near-zero transient
void CtrlPid2::InitState(const float& cmd, const float& pErr, const float& dErrState)
{
  if (Ki_ != 0.0) { // Protect for Ki == 0
    iErrState_ = (cmd - (Kp_ * pErr + Kd_ * dErrState)) / Ki_; // Run the required state
  } else {
    iErrState_ = 0.0; // Run the required state
  }
}

// Initialize for near-zero transient
void CtrlPid2::UpdState(const float& iErr, const float& dt_s)
{
  // Update the state
  if (Ki_ != 0.0) { // Protect for unlimited windup when Ki == 0
      iErrState_ += (dt_s * iErr);
  } else {
    iErrState_ = 0.0;
  }
}

// Run the Command
float CtrlPid2::CalcCmd(const float& pErr, const float& dErrState)
{
  float pCmd = Kp_ * pErr;
  float iCmd = KI_ * iErrState_;
  float dCmd = Kd_ * dErrState;
  float cmd = pCmd + iCmd + dCmd;

  // saturate cmd, set iErr to limit that produces saturated cmd
  if (cmd <= cmdRng_[0]) {
    cmd = cmdRng_[0];
    InitState(cmd, pErr, dErrState); // Re-compute the integrator state
  } else if (cmd >= cmdRng_[1]) {
    cmd = cmdRng_[1];
    InitState(cmd, pErr, dErrState); // Re-compute the integrator state
  }

  return cmd;
}

// PI+Damper Controller
// Constructor
CtrlPiDamp::CtrlPid2Ext()
{}

// Set Parameters for the tunable controller
void CtrlPiDamp::Config(const float& Kp, const float& Ki, const float& Kd, const float& b, const float& refScale, const float& cmdRng)
{
  mode_ = kCtrlStandby; // Initialize in Standby

  Kp_ = Kp;
  KI_ = Ki;
  Kd_ = Kd;

  b_ = b;

  refScale_ = refScale;
  cmdRng_ = cmdRng;

  iErrState_ = 0.0; // Initialize Integrator State
  dErrPrev_ = 0.0; // Set previous error to zero
}

float CtrlPiDamp::Run(const float& ref, const float& meas, const float& dMeas, float& dt_s)
{
  ref = refScale_ * ref; // Scale the reference Signal

  float pErr = (b_*ref) - meas; // Error for Proportional
  float iErr = ref - meas; // Error for Integrator
  float dErrState = 0 - dMeas; // Error for Derivative

  float cmd = 0.0;

  switch(mode_) {
    case kCtrlReset: // Zero the State and Command
      iErrState_ = 0.0;
      pErr = 0.0;
      cmd = 0.0;

      mode_ = kCtrlStandby;
      break;

    case kCtrlStandby: // Do Nothing, State and Command are unchanged
      break;

    case kCtrlHold: // Run Commands

      cmd = CalcCmd(pErr, dErr);
      break;

    case kCtrlInit: // Initialize State then Run Commands

      InitState(0.0, iErr, dErrState);
      cmd = CalcCmd(pErr, dErrState);
      break;

    case kCtrlEngage: // Update the State then Run Commands

      // Update the state
      UpdState(iErr, dt_s);
      cmd = CalcCmd(pErr, dErrState);
      break;
  }

  return cmd;
}

// Initialize for near-zero transient
void CtrlPiDamp::InitState(const float& cmd, const float& pErr, const float& dErrState)
{
  if (Ki_ != 0.0) { // Protect for Ki == 0
    iErrState_ = (cmd - (Kp_ * pErr + Kd_ * dErrState)) / Ki_; // Run the required state
  } else {
    iErrState_ = 0.0; // Run the required state
  }
}

// Initialize for near-zero transient
void CtrlPiDamp::UpdState(const float& iErr, const float& dt_s)
{
  // Update the state
  if (Ki_ != 0.0) { // Protect for unlimited windup when Ki == 0
      iErrState_ += (dt_s * iErr);
  } else {
    iErrState_ = 0.0;
  }
}

// Run the Command
float CtrlPiDamp::CalcCmd(const float& pErr, const float& dErrState)
{
  float pCmd = Kp_ * pErr;
  float iCmd = KI_ * iErrState_;
  float dCmd = Kd_ * dErrState;
  float cmd = pCmd + iCmd + dCmd;

  // saturate cmd, set iErr to limit that produces saturated cmd
  if (cmd <= cmdRng_[0]) {
    cmd = cmdRng_[0];
    InitState(cmd, pErr, dErrState); // Re-compute the integrator state
  } else if (cmd >= cmdRng_[1]) {
    cmd = cmdRng_[1];
    InitState(cmd, pErr, dErrState); // Re-compute the integrator state
  }

  return cmd;
}
