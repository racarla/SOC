/*
Control System Manager - Defines Controllers, Mangages mode switching

See: LICENSE.md for Copyright and License Agreement

History:
2017-11-19 - Chris Regan - Created
*/

#include "cntrlMgr.hxx"

CntrlMgr::CntrlMgr()
{
  // Create Individual Controllers
  CntrlManual cntrlBaseRoll_, cntrlBasePitch_, cntrlBaseYaw_, cntrlBaseSpeed_;
  CntrlPiDamp cntrlResRoll_, cntrlResPitch_, cntrlResYaw_, cntrlResSpeed_;

  CntrlMode cntrlMode_;
  VecCmd cntrlBaseCmd_, cntrlResCmd_, cntrlCmd_;

  float timePrev_s_;
};

void CntrlMgr::Init()
{
  cntrlMode_ = kCntrlReset; // Controller mode

  timePrev_s_ = -1.0;

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

    case kCntrlHold:
      cntrlBaseRoll_.runMode_ = kCntrlEngage;
      cntrlBasePitch_.runMode_ = kCntrlEngage;
      cntrlBaseYaw_.runMode_ = kCntrlEngage;
      cntrlBaseSpeed_.runMode_ = kCntrlEngage;

      cntrlResRoll_.runMode_ = kCntrlHold;
      cntrlResPitch_.runMode_ = kCntrlHold;
      cntrlResYaw_.runMode_ = kCntrlHold;
      cntrlResSpeed_.runMode_ = kCntrlHold;

    case kCntrlStandby:
      cntrlBaseRoll_.runMode_ = kCntrlEngage;
      cntrlBasePitch_.runMode_ = kCntrlEngage;
      cntrlBaseYaw_.runMode_ = kCntrlEngage;
      cntrlBaseSpeed_.runMode_ = kCntrlEngage;

      cntrlResRoll_.runMode_ = kCntrlStandby;
      cntrlResPitch_.runMode_ = kCntrlStandby;
      cntrlResYaw_.runMode_ = kCntrlStandby;
      cntrlResSpeed_.runMode_ = kCntrlStandby;

    case kCntrlInit:
      cntrlBaseRoll_.runMode_ = kCntrlEngage;
      cntrlBasePitch_.runMode_ = kCntrlEngage;
      cntrlBaseYaw_.runMode_ = kCntrlEngage;
      cntrlBaseSpeed_.runMode_ = kCntrlEngage;

      cntrlResRoll_.runMode_ = kCntrlInit;
      cntrlResPitch_.runMode_ = kCntrlInit;
      cntrlResYaw_.runMode_ = kCntrlInit;
      cntrlResSpeed_.runMode_ = kCntrlInit;

    case kCntrlEngage:
      cntrlBaseRoll_.runMode_ = kCntrlInit;
      cntrlBasePitch_.runMode_ = kCntrlInit;
      cntrlBaseYaw_.runMode_ = kCntrlInit;
      cntrlBaseSpeed_.runMode_ = kCntrlInit;

      cntrlResRoll_.runMode_ = kCntrlEngage;
      cntrlResPitch_.runMode_ = kCntrlEngage;
      cntrlResYaw_.runMode_ = kCntrlEngage;
      cntrlResSpeed_.runMode_ = kCntrlEngage;
  }
}

// Define the Baseline Controller - FIXIT
VecCmd CntrlMgr::CmdBase(VecCmd refVec, float time_s)
{
  timePrev_s_ = time_s;

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
  if (timePrev_s_ == -1.0) timePrev_s_ = time_s;
  float dt_s = time_s - timePrev_s_;
  timePrev_s_ = time_s;

  // Zero the Command - FIXIT shouldn't be required, variable size
  cntrlResCmd_.setZero(4);

  // Run the Controllers
  cntrlResCmd_[0] = cntrlResRoll_.Compute(refVec[0], measVec[0], dMeasVec[0], dt_s);
  cntrlResCmd_[1] = cntrlResPitch_.Compute(refVec[1], measVec[1], dMeasVec[1], dt_s);
  cntrlResCmd_[2] = cntrlResYaw_.Compute(refVec[2], measVec[2], dMeasVec[2], dt_s);
  cntrlResCmd_[3] = cntrlResSpeed_.Compute(refVec[3], measVec[3], dMeasVec[3], dt_s);

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
  VecRng refRollRng, refPitchRng, refYawRng, refThrotRng;
  refRollRng  << -100*kD2R, 100*kD2R;  // Roll rate range [rad/s]
  refPitchRng << -60*kD2R, 60*kD2R; // Pitch rate range [rad/s]
  refYawRng   << -30*kD2R, 30*kD2R;   // Yaw rate range [rad/s]
  refThrotRng << 0, 1;     // Throttle range [nd]

  // Command Range Limits, limits of the aircraft capability
  VecRng cmdRollRng, cmdPitchRng, cmdYawRng, cmdThrotRng;
  cmdRollRng  = refRollRng; // Roll rate command range [rad/s]
  cmdPitchRng = refPitchRng; // Pitch rate command range [rad/s]
  cmdYawRng   = refYawRng; // Yaw rate command range [rad/s]
  cmdThrotRng = refThrotRng; // Throttle command range [nd]

  // Initialize Individual Controllers
  cntrlBaseRoll_.Init(refRollRng[0], refRollRng[1], cmdRollRng[0], cmdRollRng[1]);
  cntrlBasePitch_.Init(refPitchRng[0], refPitchRng[1], cmdPitchRng[0], cmdPitchRng[1]);
  cntrlBaseYaw_.Init(refYawRng[0], refYawRng[1], cmdYawRng[0], cmdYawRng[1]);
  cntrlBaseSpeed_.Init(refThrotRng[0], refThrotRng[1], cmdThrotRng[0], cmdThrotRng[1]);

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
  VecRng refRollRng, refPitchRng, refYawRng, refSpeedRng;
  refRollRng  << -100*kD2R, 100*kD2R;  // Roll angle range [rad]
  refPitchRng << -60*kD2R, 60*kD2R; // Pitch angle range [rad]
  refYawRng   << -30*kD2R, 30*kD2R;   // Yaw rate range [rad/s]
  refSpeedRng << 0, 30;     // Speed range [m/s]

  // Command Range Limits, limits of the aircraft capability
  VecRng cmdRollRng, cmdPitchRng, cmdYawRng, cmdThrotRng;
  cmdRollRng  = refRollRng; // Roll rate command range [rad/s]
  cmdPitchRng = refPitchRng; // Pitch rate command range [rad/s]
  cmdYawRng   = refYawRng; // Yaw rate command range [rad/s]
  cmdThrotRng << 0, 1; // Throttle command range [nd]

  // Controller Parameters
  float KpRoll = 0.52, KiRoll = 0.20, KdampRoll = 0.07;
  float KpPitch = 0.84, KiPitch = 0.23, KdampPitch = 0.08;
  float KpYaw = 0.0, KiYaw = 0.0, KdampYaw = 0.0;
  float KpSpeed = 0.0278, KiSpeed = 0.0061, KdampSpeed = 0.0;

  // Initialize Individual Controllers
  cntrlResRoll_.Init(KpRoll, KiRoll, KdampRoll, refRollRng[0], refRollRng[1], cmdRollRng[0], cmdRollRng[1]);
  cntrlResPitch_.Init(KpPitch, KiPitch, KdampPitch, refPitchRng[0], refPitchRng[1], cmdPitchRng[0], cmdPitchRng[1]);
  cntrlResYaw_.Init(KpYaw, KiYaw, KdampYaw, refYawRng[0], refYawRng[1], cmdYawRng[0], cmdYawRng[1]);
  cntrlResSpeed_.Init(KpSpeed, KiSpeed, KdampSpeed, refSpeedRng[0], refSpeedRng[1], cmdThrotRng[0], cmdThrotRng[1]);

  // Set the initial controller mode
  cntrlResRoll_.runMode_ = kCntrlReset;
  cntrlResPitch_.runMode_ = kCntrlReset;
  cntrlResYaw_.runMode_ = kCntrlReset;
  cntrlResSpeed_.runMode_ = kCntrlReset;
}
