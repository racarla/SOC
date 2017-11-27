/*
Classes and functions for Control Functions

See: LICENSE.md for Copyright and License Agreement

History:
2017-11-12 - Chris Regan - Defined CntrlPiDamp class and methods
2017-11-12 - Chris Regan - Defined CntrlPid class and methods
2017-11-12 - Chris Regan - Defined CntrlDamp class and methods
*/

#include "cntrlFunc.hxx"
#include <iostream>


// CntrlManual
// Constructor
CntrlManual::CntrlManual()
{
  CntrlMode runMode_ = kCntrlStandby; // Initialize in Standby
  float refMin_, refMax_, cmdMin_, cmdMax_;
}

// Set Parameters for the tunable controller
void CntrlManual::Init(float refMin, float refMax, float cmdMin, float cmdMax)
{
  refMin_ = refMin;
  refMax_ = refMax;

  cmdMin_ = cmdMin;
  cmdMax_ = cmdMax;
}

// Damper Controller
float CntrlManual::Compute(float ref)
{
  float refScale = 0.5 * (refMax_ - refMin_) * ref;
  float err = refScale - 0.0;
  float cmd;


  switch(runMode_) {
    case kCntrlReset: // Zero the state and command

      cmd = 0.0;
      runMode_ = kCntrlStandby;
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
float  CntrlManual::CalcCmd(float err)
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
  CntrlMode runMode_ = kCntrlStandby; // Initialize in Standby
  float refMin_, refMax_, cmdMin_, cmdMax_;
  float KD_;
}

// Set Parameters for the tunable controller
void CntrlDamp::Init(float KD, float refMin, float refMax, float cmdMin, float cmdMax)
{
  KD_ = KD;

  refMin_ = refMin;
  refMax_ = refMax;

  cmdMin_ = cmdMin;
  cmdMax_ = cmdMax;
}

// Damper Controller
float CntrlDamp::Compute(float ref, float dMeas)
{
  float refScale = 0.5 * (refMax_ - refMin_) * ref;
  float err = refScale - 0.0;
  float dErr = 0.0 - dMeas; // Measurement for the Damper, Note the sign!!
  float cmd = 0;

  switch(runMode_) {
    case kCntrlReset: // Zero the state and command
      cmd = 0.0;
      runMode_ = kCntrlStandby;
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
float CntrlDamp::CalcCmd(float err, float dErr)
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
  CntrlMode runMode_ = kCntrlStandby; // Initialize in Standby
  float refMin_, refMax_, cmdMin_, cmdMax_;
  float KP_, KI_;

  float iErr_; // Initialize the Integrator
}

// Set Parameters for the tunable controller
void CntrlPi::Init(float KP, float KI, float refMin, float refMax, float cmdMin, float cmdMax)
{
  KP_ = KP;
  KI_ = KI;
  iErr_ = 0.0; // Initialize the Integrator

  refMin_ = refMin;
  refMax_ = refMax; 

  cmdMin_ = cmdMin;
  cmdMax_ = cmdMax;
}

// PI Controller
float CntrlPi::Compute(float ref, float meas, float dt_s)
{
  float refScale = 0.5 * (refMax_ - refMin_) * ref; // Scaled reference command
  float err = refScale - meas; // Compute the Error

  if (dt_s <= 0.0 ) {
    dt_s = 0.0; // Prevents the Integrator state from changing
  }
  float cmd;

  switch(runMode_) {
    case kCntrlReset: // Zero the state and command

      iErr_ = 0.0;
      runMode_ = kCntrlStandby;
      cmd = 0.0;
      break;

    case kCntrlStandby: // Do Nothing, State and Command unchanged
      break;

    case kCntrlHold: // Hold Integrator State, Compute Commands

      cmd = CalcCmd(err);
      break;

    case kCntrlInit: // Initialize State then Compute Commands

      InitState(0.0, err);
      cmd = CalcCmd(err);
      break;

    case kCntrlEngage: // Update the State then Compute Commands

      iErr_ += (dt_s * err); // Update the state
      cmd = CalcCmd(err);
      break;
  }

  return cmd;
}

// Initialize for near-zero transient
void CntrlPi::InitState(float cmd, float err)
{
  if (KI_ != 0.0) { // Protect for KI == 0
    float iErr_ = (cmd - (KP_ * err)) / KI_; // Compute the required state
  }
}

// Compute the Command
float CntrlPi::CalcCmd(float err)
{
  float pCmd = KP_ * err;
  float iCmd = KI_ * iErr_;
  float cmd = pCmd + iCmd;

  // saturate cmd, set iErr to limit that produces saturated cmd
  if (cmd <= cmdMin_) {
    cmd = cmdMin_;
    InitState(cmd, err); // Re-compute the integrator state
  } else if (cmd >= cmdMax_) {
    cmd = cmdMax_;
    InitState(cmd, err); // Re-compute the integrator state
  }
}


// CntrlPiDamp
// Constructor
CntrlPiDamp::CntrlPiDamp()
{
  CntrlMode runMode_ = kCntrlStandby; // Initialize in Standby
  float refMin_, refMax_, cmdMin_, cmdMax_;
  float KP_, KI_, KD_;
  
  float iErr_;
}

// Set Parameters for the tunable controller
void CntrlPiDamp::Init(float KP, float KI, float KD, float refMin, float refMax, float cmdMin, float cmdMax)
{
	KP_ = KP;
	KI_ = KI;
	KD_ = KD;

  iErr_ = 0.0; // Initialize the Integrator

  refMin_ = refMin;
  refMax_ = refMax; 

	cmdMin_ = cmdMin;
	cmdMax_ = cmdMax;
}

// PI Controller plus Damper
float CntrlPiDamp::Compute(float ref, float meas, float dMeas, float dt_s)
{
  float refScale = 0.5 * (refMax_ - refMin_) * ref; // Scaled reference command
  float err = refScale - meas; // Compute the Error
	float dErr = 0.0 - dMeas; // Measurement for the Damper, Note the sign!!

  if (dt_s <= 0.0 ) {
    dt_s = 0.0; // Prevents the Integrator state from changing
  }
  float cmd;

	switch(runMode_) {
		case kCntrlReset: // Zero the state and command

			iErr_ = 0.0;
      runMode_ = kCntrlStandby;
      cmd = 0.0;
     	break;

		case kCntrlStandby: // Do Nothing, State and Command unchanged
     	break;

		case kCntrlHold: // Hold Integrator State, Compute Commands

      cmd = CalcCmd(err, dErr);
     	break;

		case kCntrlInit: // Initialize State then Compute Commands

			InitState(0.0, err, dErr);
      cmd = CalcCmd(err, dErr);
     	break;

		case kCntrlEngage: // Update the State then Compute Commands

			iErr_ += (dt_s * err); // Update the state
			cmd = CalcCmd(err, dErr);
     	break;
	}

	return cmd;
}

// Initialize Controller for near-zero transient
void CntrlPiDamp::InitState(float cmd, float err, float dErr)
{
  if (KI_ != 0.0) { // Protect for KI == 0
    iErr_ = (cmd - (KP_ * err + KD_ * dErr)) / KI_; // Compute the required state
  } else {
    iErr_ = 0.0; // Compute the required state
  }
}

// Compute the Command
float CntrlPiDamp::CalcCmd(float err, float dErr)
{
	float pCmd = KP_ * err;
	float iCmd = KI_ * iErr_;
	float dCmd = KD_ * dErr;
	float cmd = pCmd + iCmd + dCmd;
//std::cout << "TEST\n" << err << "\t" << pCmd << "\t" << iErr_ << "\t" <<  iCmd << "\t" <<  dCmd << "\t" << cmd << std::endl;

	// saturate cmd, set iErr to limit that produces saturated cmd
	if (cmd <= cmdMin_) {
		cmd = cmdMin_;
		InitState(cmd, err, dErr); // Re-compute the integrator state
	} else if (cmd >= cmdMax_) {
		cmd = cmdMax_;
		InitState(cmd, err, dErr); // Re-compute the integrator state
	}
//std::cout << "CMD   \n" << err << "\t" << pCmd << "\t" << iErr_ << "\t" <<  iCmd << "\t" <<  dCmd << "\t" << cmd << std::endl;
}



// PID Controller
// Constructor
CntrlPid::CntrlPid()
{
  CntrlMode runMode_ = kCntrlStandby; // Initialize in Standby
  float refMin_, refMax_, cmdMin_, cmdMax_;
  float KP_, KI_, KD_;
  
  float errPrev_, iErr_;
}

// Set Parameters for the tunable controller
void CntrlPid::Init(float KP, float KI, float KD, float refMin, float refMax, float cmdMin, float cmdMax)
{
  KP_ = KP;
  KI_ = KI;
  KD_ = KD;

  iErr_ = 0.0; // Initialize Integrator State
  errPrev_ = 0.0; // Set previous error to zero

  refMin_ = refMin;
  refMax_ = refMax; 

  cmdMin_ = cmdMin;
  cmdMax_ = cmdMax;
}

float CntrlPid::Compute(float ref, float meas, float dt_s)
{
  float refScale = 0.5 * (refMax_ - refMin_) * ref; // Scaled reference command
  float err = refScale - meas; // Compute the Error

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

  switch(runMode_) {
    case kCntrlReset: // Zero the State and Command
      iErr_ = 0.0;
      err = 0.0;
      errPrev_ = 0.0;
      cmd = 0.0;

      runMode_ = kCntrlStandby;
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

      iErr_ += (dt_s * err); // Update the state
      cmd = CalcCmd(err, dErr);
      break;
  }

  return cmd;
}

// Initialize for near-zero transient
void CntrlPid::InitState(float cmd, float err, float dErr)
{
  if (KI_ != 0.0) { // Protect for KI == 0
    iErr_ = (cmd - (KP_ * err + KD_ * dErr)) / KI_; // Compute the required state
  } else {
    iErr_ = 0.0; // Compute the required state
  }
}

// Compute the Command
float CntrlPid::CalcCmd(float err, float dErr)
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
