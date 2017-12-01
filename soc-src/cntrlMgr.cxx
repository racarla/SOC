/*
Control System Manager - Defines Controllers, Mangages mode switching

See: LICENSE.md for Copyright and License Agreement

History:
2017-11-19 - Chris Regan - Created
*/

#include "cntrlMgr.hxx"
#include <iostream>

CntrlMgr::CntrlMgr()
{
  // Create Individual Controllers
  CntrlManual cntrlBaseRoll_, cntrlBasePitch_, cntrlBaseYaw_, cntrlBaseSpeed_;
  //CntrlPiDamp cntrlResRoll_, cntrlResPitch_, cntrlResYaw_, cntrlResSpeed_;
  CntrlDamp cntrlResRoll_, cntrlResPitch_, cntrlResYaw_;
  CntrlPi cntrlResSpeed_;

  VecCmd cntrlBaseCmd_, cntrlResCmd_, cntrlCmd_;
};

void CntrlMgr::Init()
{
  cntrlMode_ = kCntrlReset; // Controller mode

  timePrevBase_s_ = 0.0;
  timePrevRes_s_ = 0.0;

  cntrlBaseCmd_.setZero(4);
  cntrlResCmd_.setZero(4);
  cntrlCmd_.setZero(4);

  CntrlBaseDef();
  CntrlResDef();
};


void CntrlMgr::Mode(CntrlMode cntrlMode)
{
  cntrlMode_ = cntrlMode;

  switch (cntrlMode_) {
    case kCntrlReset:
      cntrlBaseRoll_.runMode_ = kCntrlEngage;
      cntrlBasePitch_.runMode_ = kCntrlEngage;
      cntrlBaseYaw_.runMode_ = kCntrlEngage;
      cntrlBaseSpeed_.runMode_ = kCntrlEngage;

      cntrlResRoll_.runMode_ = kCntrlReset;
      cntrlResPitch_.runMode_ = kCntrlReset;
      cntrlResYaw_.runMode_ = kCntrlReset;
      cntrlResSpeed_.runMode_ = kCntrlReset;
      break;

    case kCntrlHold:
      cntrlBaseRoll_.runMode_ = kCntrlEngage;
      cntrlBasePitch_.runMode_ = kCntrlEngage;
      cntrlBaseYaw_.runMode_ = kCntrlEngage;
      cntrlBaseSpeed_.runMode_ = kCntrlEngage;

      cntrlResRoll_.runMode_ = kCntrlHold;
      cntrlResPitch_.runMode_ = kCntrlHold;
      cntrlResYaw_.runMode_ = kCntrlHold;
      cntrlResSpeed_.runMode_ = kCntrlHold;
      break;

    case kCntrlStandby:
      cntrlBaseRoll_.runMode_ = kCntrlEngage;
      cntrlBasePitch_.runMode_ = kCntrlEngage;
      cntrlBaseYaw_.runMode_ = kCntrlEngage;
      cntrlBaseSpeed_.runMode_ = kCntrlEngage;

      cntrlResRoll_.runMode_ = kCntrlStandby;
      cntrlResPitch_.runMode_ = kCntrlStandby;
      cntrlResYaw_.runMode_ = kCntrlStandby;
      cntrlResSpeed_.runMode_ = kCntrlStandby;
      break;

    case kCntrlInit:
      cntrlBaseRoll_.runMode_ = kCntrlEngage;
      cntrlBasePitch_.runMode_ = kCntrlEngage;
      cntrlBaseYaw_.runMode_ = kCntrlEngage;
      cntrlBaseSpeed_.runMode_ = kCntrlEngage;

      cntrlResRoll_.runMode_ = kCntrlInit;
      cntrlResPitch_.runMode_ = kCntrlInit;
      cntrlResYaw_.runMode_ = kCntrlInit;
      cntrlResSpeed_.runMode_ = kCntrlInit;
      break;

    case kCntrlEngage:
      cntrlBaseRoll_.runMode_ = kCntrlInit;
      cntrlBasePitch_.runMode_ = kCntrlInit;
      cntrlBaseYaw_.runMode_ = kCntrlInit;
      cntrlBaseSpeed_.runMode_ = kCntrlInit;

      cntrlResRoll_.runMode_ = kCntrlEngage;
      cntrlResPitch_.runMode_ = kCntrlEngage;
      cntrlResYaw_.runMode_ = kCntrlEngage;
      cntrlResSpeed_.runMode_ = kCntrlEngage;
      break;
  }
}

// Define the Baseline Controller - FIXIT
VecCmd CntrlMgr::CmdBase(VecCmd refVec, float time_s)
{
  if (timePrevBase_s_ <= 0.0) timePrevBase_s_ = time_s;
  timePrevBase_s_ = time_s;

  // Zero the Command - FIXIT shouldn't be required, variable size
  cntrlBaseCmd_.setZero(4);

  // Run the Controllers
  cntrlBaseCmd_[0] = cntrlBaseRoll_.Compute(refVec[0]);
  cntrlBaseCmd_[1] = cntrlBasePitch_.Compute(refVec[1]);
  cntrlBaseCmd_[2] = cntrlBaseYaw_.Compute(refVec[2]);
  cntrlBaseCmd_[3] = cntrlBaseSpeed_.Compute(refVec[3]);

  return cntrlBaseCmd_;
}

// Define the Research Controller - FIXIT
VecCmd CntrlMgr::CmdRes(VecCmd refVec, VecCmd measVec, VecCmd dMeasVec, float time_s)
{
  if (timePrevRes_s_ <= 0.0) timePrevRes_s_ = time_s;
  float dt_s = time_s - timePrevRes_s_;
  timePrevRes_s_ = time_s;

  // Zero the Command - FIXIT shouldn't be required, variable size
  cntrlResCmd_.setZero(4);

  // Run the Controllers
  cntrlResCmd_[0] = cntrlResRoll_.Compute(refVec[0], dMeasVec[0]);
  cntrlResCmd_[1] = cntrlResPitch_.Compute(refVec[1], dMeasVec[1]);
  cntrlResCmd_[2] = cntrlResYaw_.Compute(refVec[2], dMeasVec[2]);
  //cntrlResCmd_[0] = cntrlResRoll_.Compute(refVec[0], measVec[0], dMeasVec[0], dt_s);
  //cntrlResCmd_[1] = cntrlResPitch_.Compute(refVec[1], measVec[1], dMeasVec[1], dt_s);
  //cntrlResCmd_[2] = cntrlResYaw_.Compute(refVec[2], measVec[2], dMeasVec[2], dt_s);
  cntrlResCmd_[3] = cntrlResSpeed_.Compute(refVec[3], measVec[3], dt_s);

//std::cout << cntrlResRoll_.iErr_ << "\t";
//std::cout << cntrlResPitch_.iErr_ << "\t";
//std::cout << cntrlResYaw_.iErr_ << "\t";
std::cout << refVec[3] << "\t";
std::cout << measVec[3] << "\t";
std::cout << cntrlResSpeed_.iErr_ << "\t";
std::cout << cntrlResCmd_[3] << "\t";

  return cntrlResCmd_;
}

VecCmd CntrlMgr::Cmd() {

  // Zero the Command - FIXIT shouldn't be required, variable size
  cntrlCmd_.setZero(4);

  // Switch the Command output to the Research Controller when engaged
  if (cntrlMode_ == kCntrlEngage)
  {
    cntrlCmd_ = cntrlResCmd_;
  } else {
    cntrlCmd_ = cntrlBaseCmd_;
  }

  return cntrlCmd_;
}

// Baseline Controller Definition
void CntrlMgr::CntrlBaseDef()
{
  // Command Range Limits, stick -1 to 1, converts to ref -100 to 100
  float refRollScale, refPitchScale, refYawScale, refThotScale;
  refRollScale  = 100*kD2R; // Roll rate range [rad/s]
  refPitchScale = 60*kD2R;  // Pitch rate range [rad/s]
  refYawScale   = 30*kD2R;  // Yaw rate range [rad/s]
  refThotScale  = 1;        // Throttle range [nd]

  // Command Range Limits, limits of the aircraft capability
  VecRng cmdRollRng, cmdPitchRng, cmdYawRng, cmdThrotRng;
  cmdRollRng  << -refRollScale, refRollScale;  // Roll rate command range [rad/s]
  cmdPitchRng << -refPitchScale, refPitchScale; // Pitch rate command range [rad/s]
  cmdYawRng   << -refYawScale, refYawScale;   // Yaw rate command range [rad/s]
  cmdThrotRng << 0, 1;   // Throttle command range [nd]

  // Initialize Individual Controllers
  cntrlBaseRoll_.Init(refRollScale, cmdRollRng[0], cmdRollRng[1]);
  cntrlBasePitch_.Init(refPitchScale, cmdPitchRng[0], cmdPitchRng[1]);
  cntrlBaseYaw_.Init(refYawScale, cmdYawRng[0], cmdYawRng[1]);
  cntrlBaseSpeed_.Init(refThotScale, cmdThrotRng[0], cmdThrotRng[1]);

  // Set the initial controller mode
  cntrlBaseRoll_.runMode_ = kCntrlInit;
  cntrlBasePitch_.runMode_ = kCntrlInit;
  cntrlBaseYaw_.runMode_ = kCntrlInit;
  cntrlBaseSpeed_.runMode_ = kCntrlInit;
}

// Research Control Law
void CntrlMgr::CntrlResDef()
{
  // Command Range Limits, stick -1 to 1, converts to ref -60 to 60
  float refRollScale, refPitchScale, refYawScale, refSpeedRng;
  refRollScale  = 100*kD2R; // Roll rate range [rad/s]
  refPitchScale = 60*kD2R;  // Pitch rate range [rad/s]
  refYawScale   = 30*kD2R;  // Yaw rate range [rad/s]
  refSpeedRng   = 30;      // Speed range [m/s]

  // Command Range Limits, limits of the aircraft capability
  VecRng cmdRollRng, cmdPitchRng, cmdYawRng, cmdThrotRng;
  cmdRollRng  << -refRollScale, refRollScale;  // Roll rate command range [rad/s]
  cmdPitchRng << -refPitchScale, refPitchScale; // Pitch rate command range [rad/s]
  cmdYawRng   << -refYawScale, refYawScale;   // Yaw rate command range [rad/s]
  cmdThrotRng << 0, 1; // Throttle command range [nd]

  // Correct for the reference and command range scales
  float cmdScaleRoll = 0.5 * (cmdRollRng[1] - cmdRollRng[0]) / refRollScale;
  float cmdScalePitch = 0.5 * (cmdPitchRng[1] - cmdPitchRng[0]) / refPitchScale;
  float cmdScaleYaw = 0.5 * (cmdYawRng[1] - cmdYawRng[0]) / refYawScale;
  float cmdScaleSpeed = (cmdThrotRng[1] - cmdThrotRng[0]) / refSpeedRng;

  // Controller Parameters, corrected to the normalized I/O ranges
  float KpRoll = 0.52 * cmdScaleRoll, KiRoll = 0.385 * KpRoll, KdampRoll = 10 * -0.08; // FIXIT Gains
  float KpPitch = 0.84 * cmdScalePitch, KiPitch = 0.274 * KpPitch, KdampPitch = 10 * -0.08;
  float KpYaw = 0.5 * cmdScaleYaw, KiYaw = 0.0 * KpYaw, KdampYaw = 10 * -0.08;
  float KpSpeed = 0.5 / 30, KiSpeed = 0.219 * KpSpeed, KdampSpeed = 0.0;

  // Initialize Individual Controllers
  cntrlResRoll_.Init(refRollScale, cmdRollRng[0], cmdRollRng[1], KdampRoll);
  cntrlResPitch_.Init(refPitchScale, cmdPitchRng[0], cmdPitchRng[1], KdampPitch);
  cntrlResYaw_.Init(refYawScale, cmdYawRng[0], cmdYawRng[1], KdampYaw);
  cntrlResSpeed_.Init(1.0, cmdThrotRng[0], cmdThrotRng[1], KpSpeed, KiSpeed);
/*
  cntrlResRoll_.Init(refRollScale, cmdRollRng[0], cmdRollRng[1], KpRoll, KiRoll, KdampRoll);
  cntrlResPitch_.Init(refPitchScale, cmdPitchRng[0], cmdPitchRng[1], KpPitch, KiPitch, KdampPitch);
  cntrlResYaw_.Init(refYawScale, cmdYawRng[0], cmdYawRng[1], KpYaw, KiYaw, KdampYaw);
  cntrlResSpeed_.Init(refSpeedRng, cmdThrotRng[0], cmdThrotRng[1], KpSpeed, KiSpeed, KdampSpeed);
*/
  // Set the initial controller mode
  cntrlResRoll_.runMode_ = kCntrlReset;
  cntrlResPitch_.runMode_ = kCntrlReset;
  cntrlResYaw_.runMode_ = kCntrlReset;
  cntrlResSpeed_.runMode_ = kCntrlReset;
}
