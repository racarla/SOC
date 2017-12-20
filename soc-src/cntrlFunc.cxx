/*
Classes and functions for Control Functions

See: LICENSE.md for Copyright and License Agreement

History:
2017-11-12 - Chris Regan - Defined CntrlPiDamp class and methods
2017-11-12 - Chris Regan - Defined CntrlPid class and methods
2017-11-12 - Chris Regan - Defined CntrlDamp class and methods

*/

#include <iostream>

#include "cntrlFunc.hxx"

// CntrlManual
// Constructor
CntrlManual::CntrlManual()
{
}

// Set Parameters for the tunable controller
void CntrlManual::Init(const float& refScale, const float& cmdMin, const float& cmdMax)
{
  mode_ = kCntrlStandby; // Initialize in Standby

  refScale_ = refScale;

  cmdMin_ = cmdMin;
  cmdMax_ = cmdMax;
}

// Damper Controller
float CntrlManual::Compute(const float& ref)
{
  float err = (refScale_ * ref) - 0.0;
  float cmd;


  switch(mode_) {
    case kCntrlReset: // Zero the state and command

      cmd = 0.0;
      mode_ = kCntrlStandby;
      break;

    case kCntrlStandby: // Do Nothing, State and Command unchanged
      break;

    case kCntrlHold: // Compute Commands

      cmd = CalcCmd(err);
      break;

    case kCntrlInit: // Initialize State then Compute Commands

      cmd = CalcCmd(err);
      break;

    case kCntrlEngage: // Update the State then Compute Commands

      cmd = CalcCmd(err);
      break;
  }

  return cmd;
}

// Compute the Command for the Damper controller
float CntrlManual::CalcCmd(const float& err)
{
  float cmd = err;

  // saturate cmd, set iErr to limit that produces saturated cmd
  if (cmd <= cmdMin_) {
    cmd = cmdMin_;
  } else if (cmd >= cmdMax_) {
    cmd = cmdMax_;
  }

  return cmd;
}


// CntrlDamp
// Constructor
CntrlDamp::CntrlDamp()
{
}

// Set Parameters for the tunable controller
void CntrlDamp::Init(const float& refScale, const float& cmdMin, const float& cmdMax, const float& KD)
{
  mode_ = kCntrlStandby; // Initialize in Standby

  refScale_ = refScale;

  cmdMin_ = cmdMin;
  cmdMax_ = cmdMax;

  KD_ = KD;
}

// Damper Controller
float CntrlDamp::Compute(const float& ref, const float& dMeas)
{
  float err = (refScale_ * ref) - 0.0;
  float dErr = 0.0 - dMeas; // Measurement for the Damper
  float cmd = 0;

  switch(mode_) {
    case kCntrlReset: // Zero the state and command
      cmd = 0.0;
      mode_ = kCntrlStandby;
      break;

    case kCntrlStandby: // Do Nothing, State and Command unchanged
      break;

    case kCntrlHold: // Compute Commands

      cmd = CalcCmd(err, dErr);
      break;

    case kCntrlInit: // Initialize State then Compute Commands

      cmd = CalcCmd(err, dErr);
      break;

    case kCntrlEngage: // Update the State then Compute Commands

      cmd = CalcCmd(err, dErr);
      break;
  }

  return cmd;
}

// Compute the Command
float CntrlDamp::CalcCmd(const float& err, const float& dErr)
{
  float cmd = err + KD_ * dErr;

  // saturate cmd, set iErr to limit that produces saturated cmd
  if (cmd <= cmdMin_) {
    cmd = cmdMin_;
  } else if (cmd >= cmdMax_) {
    cmd = cmdMax_;
  }

  return cmd;
}

// CntrlPi
// Constructor
CntrlPi::CntrlPi()
{
}

// Set Parameters for the tunable controller
void CntrlPi::Init(const float& refScale, const float& cmdMin, const float& cmdMax, const float& KP, const float& KI)
{
  mode_ = kCntrlStandby; // Initialize in Standby

  refScale_ = refScale;

  cmdMin_ = cmdMin;
  cmdMax_ = cmdMax;

  KP_ = KP;
  KI_ = KI;

  iErr_ = 0.0; // Initialize Integrator State
}

// PI Controller
float CntrlPi::Compute(const float& ref, const float& meas, float& dt_s)
{
  float err = (refScale_ * ref) - meas; // Compute the Error

  if (dt_s <= 0.0 ) {
    dt_s = 0.0; // Prevents the Integrator state from changing
  }
  float cmd;

  switch(mode_) {
    case kCntrlReset: // Zero the state and command

      iErr_ = 0.0;
      mode_ = kCntrlStandby;
      cmd = 0.0;
      break;

    case kCntrlStandby: // Do Nothing, State and Command unchanged

      break;

    case kCntrlHold: // Hold Integrator State, Compute Commands

      cmd = CalcCmd(err);
      break;

    case kCntrlInit: // Initialize State then Compute Commands

      iErr_ = 0.0;
      cmd = CalcCmd(err); // Compute the Command normally
      InitState(cmd, 0.0); // Compute the integrator value that zeros the error

      break;

    case kCntrlEngage: // Update the State then Compute Commands

      // Update the state
      if (KI_ != 0.0) { // Protect for KI == 0
          iErr_ += (dt_s * err);
      } else {
        iErr_ = 0.0; // Compute the required state
      }

      cmd = CalcCmd(err);
      break;
  }

  return cmd;
}

// Initialize for near-zero transient
void CntrlPi::InitState(const float& cmd, const float& err)
{
  if (KI_ != 0.0) { // Protect for KI == 0
    iErr_ = (cmd - (KP_ * err)) / KI_; // Compute the required state
  }
}

// Compute the Command
float CntrlPi::CalcCmd(const float& err)
{
  float pCmd = KP_ * err;
  float iCmd = KI_ * iErr_;
  float cmd = pCmd + iCmd;

  // saturate cmd, set iErr to limit that produces saturated cmd
  if (cmd <= cmdMin_) {
    cmd = cmdMin_;
    InitState(cmd, 0.0); // Re-compute the integrator state
  } else if (cmd >= cmdMax_) {
    cmd = cmdMax_;
    InitState(cmd, 0.0); // Re-compute the integrator state
  }
}


// CntrlPiDamp
// Constructor
CntrlPiDamp::CntrlPiDamp()
{}

// Set Parameters for the tunable controller
void CntrlPiDamp::Init(const float& refScale, const float& cmdMin, const float& cmdMax, const float& KP, const float& KI, const float& KD)
{
  mode_ = kCntrlStandby; // Initialize in Standby

  refScale_ = refScale;

  cmdMin_ = cmdMin;
  cmdMax_ = cmdMax;

  KP_ = KP;
  KI_ = KI;
  KD_ = KD;

  iErr_ = 0.0; // Initialize Integrator State
}

// PI Controller plus Damper
float CntrlPiDamp::Compute(const float& ref, const float& meas, const float& dMeas, float& dt_s)
{
  float err = (refScale_ * ref) - meas; // Compute the Error
	float dErr = 0.0 - dMeas; // Measurement for the Damper

  if (dt_s <= 0.0 ) {
    dt_s = 0.0; // Prevents the Integrator state from changing
  }
  float cmd;

	switch(mode_) {
		case kCntrlReset: // Zero the state and command

			iErr_ = 0.0;
      mode_ = kCntrlStandby;
      cmd = 0.0;
     	break;

		case kCntrlStandby: // Do Nothing, State and Command unchanged
     	break;

		case kCntrlHold: // Hold Integrator State, Compute Commands

      cmd = CalcCmd(err, dErr);
     	break;

		case kCntrlInit: // Initialize State then Compute Commands

      iErr_ = 0.0; // Zero the integrator
      cmd = CalcCmd(err, dErr); // Compute the Command normally
      InitState(cmd, 0.0, 0.0); // Compute the integrator value that zeros the error
     	break;

		case kCntrlEngage: // Update the State then Compute Commands

      // Update the state
      if (KI_ != 0.0) { // Protect for KI == 0
          iErr_ += (dt_s * err);
      } else {
        iErr_ = 0.0; // Compute the required state
      }

			cmd = CalcCmd(err, dErr);
     	break;
	}

	return cmd;
}

// Initialize Controller for near-zero transient
void CntrlPiDamp::InitState(const float& cmd, const float& err, const float& dErr)
{
  if (KI_ != 0.0) { // Protect for KI == 0
    iErr_ = (cmd - (KP_ * err + KD_ * dErr)) / KI_; // Compute the required state
  } else {
    iErr_ = 0.0; // Compute the required state
  }
}

// Compute the Command
float CntrlPiDamp::CalcCmd(const float& err, const float& dErr)
{
	float pCmd = KP_ * err;
	float iCmd = KI_ * iErr_;
	float dCmd = KD_ * dErr;
	float cmd = pCmd + iCmd + dCmd;
//std::cout << "TEST\n" << err << "\t" << pCmd << "\t" << iErr_ << "\t" <<  iCmd << "\t" <<  dCmd << "\t" << cmd << std::endl;

	// saturate cmd, set iErr to limit that produces saturated cmd
	if (cmd <= cmdMin_) {
		cmd = cmdMin_;
		InitState(cmd, err, 0.0); // Re-compute the integrator state
	} else if (cmd >= cmdMax_) {
		cmd = cmdMax_;
		InitState(cmd, err, 0.0); // Re-compute the integrator state
	}
//std::cout << "CMD   \n" << err << "\t" << pCmd << "\t" << iErr_ << "\t" <<  iCmd << "\t" <<  dCmd << "\t" << cmd << std::endl;
}



// PID Controller
// Constructor
CntrlPid::CntrlPid()
{}

// Set Parameters for the tunable controller
void CntrlPid::Init(const float& refScale, const float& cmdMin, const float& cmdMax, const float& KP, const float& KI, const float& KD)
{
  mode_ = kCntrlStandby; // Initialize in Standby

  refScale_ = refScale;

  cmdMin_ = cmdMin;
  cmdMax_ = cmdMax;

  KP_ = KP;
  KI_ = KI;
  KD_ = KD;

  iErr_ = 0.0; // Initialize Integrator State
  errPrev_ = 0.0; // Set previous error to zero
}

float CntrlPid::Compute(const float& ref, const float& meas, float& dt_s)
{
  float err = (refScale_ * ref) - meas; // Compute the Error

  // Derivative of Error
  float dErr;
  if (dt_s > 0.0 ) {
    dErr = (err - errPrev_) / dt_s;
  } else {
    dt_s = 0.0; // Prevents the Integrator state from changing
    dErr = 0.0;
  }

  float cmd;

  // Update the previous error state
  errPrev_ = err;

  switch(mode_) {
    case kCntrlReset: // Zero the State and Command
      iErr_ = 0.0;
      err = 0.0;
      errPrev_ = 0.0;
      cmd = 0.0;

      mode_ = kCntrlStandby;
      break;

    case kCntrlStandby: // Do Nothing, State and Command are unchanged
      break;

    case kCntrlHold: // Compute Commands

      cmd = CalcCmd(err, dErr);
      break;

    case kCntrlInit: // Initialize State then Compute Commands

      InitState(0.0, err, dErr);
      cmd = CalcCmd(err, dErr);
      break;

    case kCntrlEngage: // Update the State then Compute Commands

      // Update the state
      if (KI_ != 0.0) { // Protect for KI == 0
          iErr_ += (dt_s * err);
      } else {
        iErr_ = 0.0; // Compute the required state
      }

      cmd = CalcCmd(err, dErr);
      break;
  }

  return cmd;
}

// Initialize for near-zero transient
void CntrlPid::InitState(const float& cmd, const float& err, const float& dErr)
{
  if (KI_ != 0.0) { // Protect for KI == 0
    iErr_ = (cmd - (KP_ * err + KD_ * dErr)) / KI_; // Compute the required state
  } else {
    iErr_ = 0.0; // Compute the required state
  }
}

// Compute the Command
float CntrlPid::CalcCmd(const float& err, const float& dErr)
{
  float pCmd = KP_ * err;
  float iCmd = KI_ * iErr_;
  float dCmd = KD_ * dErr;
  float cmd = pCmd + iCmd + dCmd;

  // saturate cmd, set iErr to limit that produces saturated cmd
  if (cmd <= cmdMin_) {
    cmd = cmdMin_;
    InitState(cmd, err, dErr); // Re-compute the integrator state
  } else if (cmd >= cmdMax_) {
    cmd = cmdMax_;
    InitState(cmd, err, dErr); // Re-compute the integrator state
  }

  return cmd;
}
