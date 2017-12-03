/*
Control System Manager - Defines Controllers, Mangages mode switching

See: LICENSE.md for Copyright and License Agreement

History:
2017-11-19 - Chris Regan - Created
*/

#include "cntrlMgr.hxx"
#include <iostream>


void CntrlMgr::Init()
{
  cntrlMgrData_.mode = kCntrlReset; // Controller mode

  timePrevBase_s_ = 0.0;
  timePrevRes_s_ = 0.0;

  cntrlMgrData_.cmdBase.setZero(4);
  cntrlMgrData_.cmdRes.setZero(4);
  cntrlMgrData_.cmd.setZero(4);


  CntrlBaseDef();
  CntrlResDef();
};


void CntrlMgr::Mode(CntrlMode mode)
{
  cntrlMgrData_.mode = mode;

  switch (cntrlMgrData_.mode) {
    case kCntrlReset:
      baseRoll_.mode_ = kCntrlEngage;
      basePitch_.mode_ = kCntrlEngage;
      baseYaw_.mode_ = kCntrlEngage;
      baseSpeed_.mode_ = kCntrlEngage;

      resRoll_.mode_ = kCntrlReset;
      resPitch_.mode_ = kCntrlReset;
      resYaw_.mode_ = kCntrlReset;
      resSpeed_.mode_ = kCntrlReset;
      break;

    case kCntrlHold:
      baseRoll_.mode_ = kCntrlEngage;
      basePitch_.mode_ = kCntrlEngage;
      baseYaw_.mode_ = kCntrlEngage;
      baseSpeed_.mode_ = kCntrlEngage;

      resRoll_.mode_ = kCntrlHold;
      resPitch_.mode_ = kCntrlHold;
      resYaw_.mode_ = kCntrlHold;
      resSpeed_.mode_ = kCntrlHold;
      break;

    case kCntrlStandby:
      baseRoll_.mode_ = kCntrlEngage;
      basePitch_.mode_ = kCntrlEngage;
      baseYaw_.mode_ = kCntrlEngage;
      baseSpeed_.mode_ = kCntrlEngage;

      resRoll_.mode_ = kCntrlStandby;
      resPitch_.mode_ = kCntrlStandby;
      resYaw_.mode_ = kCntrlStandby;
      resSpeed_.mode_ = kCntrlStandby;
      break;

    case kCntrlInit:
      baseRoll_.mode_ = kCntrlEngage;
      basePitch_.mode_ = kCntrlEngage;
      baseYaw_.mode_ = kCntrlEngage;
      baseSpeed_.mode_ = kCntrlEngage;

      resRoll_.mode_ = kCntrlInit;
      resPitch_.mode_ = kCntrlInit;
      resYaw_.mode_ = kCntrlInit;
      resSpeed_.mode_ = kCntrlInit;
      break;

    case kCntrlEngage:
      baseRoll_.mode_ = kCntrlInit;
      basePitch_.mode_ = kCntrlInit;
      baseYaw_.mode_ = kCntrlInit;
      baseSpeed_.mode_ = kCntrlInit;

      resRoll_.mode_ = kCntrlEngage;
      resPitch_.mode_ = kCntrlEngage;
      resYaw_.mode_ = kCntrlEngage;
      resSpeed_.mode_ = kCntrlEngage;
      break;
  }
}

// Define the Baseline Controller - FIXIT
VecCmd CntrlMgr::CmdBase(VecCmd refVec, float time_s)
{
  if (timePrevBase_s_ <= 0.0) timePrevBase_s_ = time_s;
  timePrevBase_s_ = time_s;

  // Zero the Command - FIXIT shouldn't be required, variable size
  cntrlMgrData_.cmdBase.setZero(4);

  // Run the Controllers
  cntrlMgrData_.cmdBase[0] = baseRoll_.Compute(refVec[0]);
  cntrlMgrData_.cmdBase[1] = basePitch_.Compute(refVec[1]);
  cntrlMgrData_.cmdBase[2] = baseYaw_.Compute(refVec[2]);
  cntrlMgrData_.cmdBase[3] = baseSpeed_.Compute(refVec[3]);

  return cntrlMgrData_.cmdBase;
}

// Define the Research Controller - FIXIT
VecCmd CntrlMgr::CmdRes(VecCmd refVec, VecCmd measVec, VecCmd dMeasVec, float time_s)
{
  if (timePrevRes_s_ <= 0.0) timePrevRes_s_ = time_s;
  float dt_s = time_s - timePrevRes_s_;
  timePrevRes_s_ = time_s;

  // Zero the Command - FIXIT shouldn't be required, variable size
  cntrlMgrData_.cmdRes.setZero(4);

  // Run the Controllers
  cntrlMgrData_.cmdRes[0] = resRoll_.Compute(refVec[0], dMeasVec[0]);
  cntrlMgrData_.cmdRes[1] = resPitch_.Compute(refVec[1], dMeasVec[1]);
  cntrlMgrData_.cmdRes[2] = resYaw_.Compute(refVec[2], dMeasVec[2]);
  //cntrlMgrData_.cmdRes[0] = resRoll_.Compute(refVec[0], measVec[0], dMeasVec[0], dt_s);
  //cntrlMgrData_.cmdRes[1] = resPitch_.Compute(refVec[1], measVec[1], dMeasVec[1], dt_s);
  //cntrlMgrData_.cmdRes[2] = resYaw_.Compute(refVec[2], measVec[2], dMeasVec[2], dt_s);
  cntrlMgrData_.cmdRes[3] = resSpeed_.Compute(refVec[3], measVec[3], dt_s);

//std::cout << resRoll_.iErr_ << "\t";
//std::cout << resPitch_.iErr_ << "\t";
//std::cout << resYaw_.iErr_ << "\t";
//std::cout << refVec[3] << "\t";
//std::cout << measVec[3] << "\t";
//std::cout << resSpeed_.iErr_ << "\t";
//std::cout << cntrlMgrData_.cmdRes[3] << "\t";

  return cntrlMgrData_.cmdRes;
}

CntrlMgrStruct CntrlMgr::Cmd() {

  // Zero the Command - FIXIT shouldn't be required, variable size
  cntrlMgrData_.cmd.setZero(4);

  // Switch the Command output to the Research Controller when engaged
  if (cntrlMgrData_.mode == kCntrlEngage)
  {
    cntrlMgrData_.cmd = cntrlMgrData_.cmdRes;
  } else {
    cntrlMgrData_.cmd = cntrlMgrData_.cmdBase;
  }

  return cntrlMgrData_;
}

// Baseline Controller Definition
void CntrlMgr::CntrlBaseDef()
{
  int numCmd = 4;

  // Command Range Limits, stick -1 to 1, converts to ref -100 to 100
  VecCmd refScale(numCmd);
  refScale[0] = 100*kD2R; // Roll rate range [rad/s]
  refScale[1] = 60*kD2R;  // Pitch rate range [rad/s]
  refScale[2] = 30*kD2R;  // Yaw rate range [rad/s]
  refScale[3] = 1;        // Throttle range [nd]

  // Command Range Limits, limits of the aircraft capability
  VecCmd cmdRngMin(numCmd), cmdRngMax(numCmd);
  cmdRngMax[0]  = refScale[0]; cmdRngMin[0] = -cmdRngMax[0]; // Roll rate command range [rad/s]
  cmdRngMax[1]  = refScale[1]; cmdRngMin[1] = -cmdRngMax[1]; // Pitch rate command range [rad/s]
  cmdRngMax[2]  = refScale[2]; cmdRngMin[2] = -cmdRngMax[2]; // Yaw rate command range [rad/s]
  cmdRngMax[3]  = 1; cmdRngMin[3] = 0; // Throttle command range [nd]

  // Initialize Individual Controllers
  baseRoll_.Init(refScale[0], cmdRngMin[0], cmdRngMax[0]);
  basePitch_.Init(refScale[1], cmdRngMin[1], cmdRngMax[1]);
  baseYaw_.Init(refScale[2], cmdRngMin[2], cmdRngMax[2]);
  baseSpeed_.Init(refScale[3], cmdRngMin[3], cmdRngMax[3]);

  // Set the initial controller mode
  baseRoll_.mode_ = kCntrlInit;
  basePitch_.mode_ = kCntrlInit;
  baseYaw_.mode_ = kCntrlInit;
  baseSpeed_.mode_ = kCntrlInit;
}

// Research Control Law
void CntrlMgr::CntrlResDef()
{
  int numCmd = 4;

  // Command Range Limits, stick -1 to 1, converts to ref -60 to 60
  VecCmd refScale(numCmd);
  refScale[0] = 100*kD2R; // Roll rate range [rad/s]
  refScale[1] = 60*kD2R;  // Pitch rate range [rad/s]
  refScale[2] = 30*kD2R;  // Yaw rate range [rad/s]
  refScale[3] = 30;      // Speed range [m/s]

  // Command Range Limits, limits of the aircraft capability
  VecCmd cmdRngMin(numCmd), cmdRngMax(numCmd);
  cmdRngMax[0]  = refScale[0]; cmdRngMin[0] = -cmdRngMax[0]; // Roll rate command range [rad/s]
  cmdRngMax[1]  = refScale[1]; cmdRngMin[1] = -cmdRngMax[1]; // Pitch rate command range [rad/s]
  cmdRngMax[2]  = refScale[2]; cmdRngMin[2] = -cmdRngMax[2]; // Yaw rate command range [rad/s]
  cmdRngMax[3]  = 1; cmdRngMin[3] = 0; // Throttle command range [nd]

  // Correct for the reference and command range scales
  VecCmd cmdScale(numCmd);
  cmdScale[0] = 0.5 * (cmdRngMax[0] - cmdRngMin[0]) / refScale[0];
  cmdScale[1] = 0.5 * (cmdRngMax[1] - cmdRngMin[1]) / refScale[1];
  cmdScale[2] = 0.5 * (cmdRngMax[2] - cmdRngMin[2]) / refScale[2];
  cmdScale[3] = 1.0 * (cmdRngMax[3] - cmdRngMin[3]) / refScale[3];

  // Controller Parameters, corrected to the normalized I/O ranges
  VecCmd Kp(numCmd), Ki(numCmd), Kd(numCmd);
  Kp[0] = 0.52 * cmdScale[0], Ki[0] = 0.385 * Kp[0], Kd[0] = 10 * -0.08; // FIXIT Gains
  Kp[1] = 0.84 * cmdScale[1], Ki[1] = 0.274 * Kp[1], Kd[1] = 10 * -0.08;
  Kp[2] = 0.50 * cmdScale[2], Ki[2] = 0.000 * Kp[2], Kd[2] = 10 * -0.08;
  Kp[3] = 0.50 * cmdScale[3], Ki[3] = 0.219 * Kp[3], Kd[3] = 0.0;

  // Initialize Individual Controllers
  resRoll_.Init(refScale[0], cmdRngMin[0], cmdRngMax[0], Kd[0]);
  resPitch_.Init(refScale[1], cmdRngMin[1], cmdRngMax[1], Kd[1]);
  resYaw_.Init(refScale[2], cmdRngMin[2], cmdRngMax[2], Kd[2]);
  resSpeed_.Init(1.0, cmdRngMin[3], cmdRngMax[3], Kp[3], Ki[3]);

  // Set the initial controller mode
  resRoll_.mode_ = kCntrlReset;
  resPitch_.mode_ = kCntrlReset;
  resYaw_.mode_ = kCntrlReset;
  resSpeed_.mode_ = kCntrlReset;
}
