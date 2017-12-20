/*
Control System Manager - Defines Controllers, Mangages mode switching

See: LICENSE.md for Copyright and License Agreement

History:
2017-11-19 - Chris Regan - Created
*/

#include "cntrlMgr.hxx"
#include <iostream>


void CntrlMgr::Init(const CntrlAllocDef& cntrlAllocDef)
{
  cntrlMgrOut_.mode = kCntrlReset; // Controller mode

  timePrevBase_s_ = 0.0;
  timePrevRes_s_ = 0.0;

  cntrlMgrOut_.cmdCntrlBase.setZero(kMaxCntrlCmd);
  cntrlMgrOut_.cmdCntrlRes.setZero(kMaxCntrlCmd);
  cntrlMgrOut_.cmdCntrl.setZero(kMaxCntrlCmd);
  cntrlMgrOut_.cmdEff.setZero(kMaxCntrlEff);


  CntrlBaseDef();
  CntrlResDef();

  cntrlAllocDef_ = cntrlAllocDef;

  numObj_ = cntrlAllocDef_.cntrlEff.rows();
  numEff_ = cntrlAllocDef_.cntrlEff.cols();
};


void CntrlMgr::Mode(CntrlMode mode)
{
  cntrlMgrOut_.mode = mode;

  switch (cntrlMgrOut_.mode) {
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
VecCmd CntrlMgr::CmdCntrlBase(const float& time_s, const FmuData& fmuData, const NavOut& navOut, const AirdataOut& airdataOut)
{
  if (timePrevBase_s_ <= 0.0) timePrevBase_s_ = time_s;
  timePrevBase_s_ = time_s;


  VecCmd refVec(kMaxCntrlCmd);
  refVec[0] = fmuData.SbusRx[0].Inceptors[0];
  refVec[1] = fmuData.SbusRx[0].Inceptors[1];
  refVec[2] = fmuData.SbusRx[0].Inceptors[2];
  refVec[3] = fmuData.SbusRx[0].Inceptors[4];

  // Zero the Command - FIXIT shouldn't be required, variable size
  cntrlMgrOut_.cmdCntrlBase.setZero(kMaxCntrlCmd);

  // Run the Controllers
  cntrlMgrOut_.cmdCntrlBase[0] = baseRoll_.Compute(refVec[0]);
  cntrlMgrOut_.cmdCntrlBase[1] = basePitch_.Compute(refVec[1]);
  cntrlMgrOut_.cmdCntrlBase[2] = baseYaw_.Compute(refVec[2]);
  cntrlMgrOut_.cmdCntrlBase[3] = baseSpeed_.Compute(refVec[3]);

  return cntrlMgrOut_.cmdCntrlBase;
}

// Define the Research Controller - FIXIT
VecCmd CntrlMgr::CmdCntrlRes(const float& time_s, const FmuData& fmuData, const NavOut& navOut, const AirdataOut& airdataOut)
{
  if (timePrevRes_s_ <= 0.0) timePrevRes_s_ = time_s;
  float dt_s = time_s - timePrevRes_s_;
  timePrevRes_s_ = time_s;


  VecCmd refVec(kMaxCntrlCmd);
  refVec[0] = fmuData.SbusRx[0].Inceptors[0];
  refVec[1] = fmuData.SbusRx[0].Inceptors[1];
  refVec[2] = fmuData.SbusRx[0].Inceptors[2];
  refVec[3] = 17; // Command speed - FIXIT - Pass-in

  VecCmd measVec(kMaxCntrlCmd);
  measVec[0] = navOut.Euler_rad[0];
  measVec[1] = navOut.Euler_rad[1];
  measVec[2] = navOut.Euler_rad[2];
  measVec[3] = airdataOut.vIasFilt_mps;

  VecCmd dMeasVec(kMaxCntrlCmd);
  dMeasVec[0] = fmuData.Mpu9250.Gyro_rads[0];
  dMeasVec[1] = fmuData.Mpu9250.Gyro_rads[1];
  dMeasVec[2] = fmuData.Mpu9250.Gyro_rads[2];
  dMeasVec[3] = 0.0;

  // Zero the Command - FIXIT shouldn't be required, variable size
  cntrlMgrOut_.cmdCntrlRes.setZero(kMaxCntrlCmd);

  // Run the Controllers
  cntrlMgrOut_.cmdCntrlRes[0] = resRoll_.Compute(refVec[0], measVec[0], dMeasVec[0], dt_s);
  cntrlMgrOut_.cmdCntrlRes[1] = resPitch_.Compute(refVec[1], measVec[1], dMeasVec[1], dt_s);
  cntrlMgrOut_.cmdCntrlRes[2] = resYaw_.Compute(refVec[2], measVec[2], dMeasVec[2], dt_s);
  cntrlMgrOut_.cmdCntrlRes[3] = resSpeed_.Compute(refVec[3], measVec[3], dt_s);


  return cntrlMgrOut_.cmdCntrlRes;
}

CntrlMgrOut CntrlMgr::CmdCntrl() {

  // Zero the Command - FIXIT shouldn't be required, variable size
  cntrlMgrOut_.cmdCntrl.setZero(kMaxCntrlCmd);

  // Switch the Command output to the Research Controller when engaged
  if (cntrlMgrOut_.mode == kCntrlEngage)
  {
    cntrlMgrOut_.cmdCntrl = cntrlMgrOut_.cmdCntrlRes;
  } else {
    cntrlMgrOut_.cmdCntrl = cntrlMgrOut_.cmdCntrlBase;
  }

  cntrlMgrOut_.vObj = cntrlMgrOut_.cmdCntrl.head(kMaxAllocObj); // FIXIT - Want roll, pitch, yaw components
  cntrlMgrOut_.cmdAlloc = AllocCompute(cntrlMgrOut_.vObj);

  return cntrlMgrOut_;
}


VecAllocEff CntrlMgr::AllocCompute(const VecAllocObj& vObj)
{
  VecAllocEff cmdAlloc = cntrlAllocDef_.uPref;

  cmdAlloc = CntrlAllocPseudo(cntrlAllocDef_.cntrlEff, vObj, cntrlAllocDef_.uPref);

  return cmdAlloc;

}

// Baseline Controller Definition
void CntrlMgr::CntrlBaseDef()
{
  int numCmd = kMaxCntrlCmd;

  // Command Range Limits, stick -1 to 1, converts to ref -100 to 100
  VecCmd refScale(numCmd);
  refScale[0] = 60*kD2R; // Roll rate range [rad/s]
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
  int numCmd = kMaxCntrlCmd;

  // Command Range Limits, stick -1 to 1, converts to ref -60 to 60
  VecCmd refScale(numCmd);
  refScale[0] = 60*kD2R; // Roll rate range [rad/s]
  refScale[1] = 60*kD2R;  // Pitch rate range [rad/s]
  refScale[2] = 30*kD2R;  // Yaw rate range [rad/s]
  refScale[3] = 30;      // Speed range [m/s]

  // Command Range Limits, limits of the aircraft capability
  VecCmd cmdRngMin(numCmd), cmdRngMax(numCmd);
  cmdRngMax[0] = refScale[0]; cmdRngMin[0] = -cmdRngMax[0]; // Roll rate command range [rad/s]
  cmdRngMax[1] = refScale[1]; cmdRngMin[1] = -cmdRngMax[1]; // Pitch rate command range [rad/s]
  cmdRngMax[2] = refScale[2]; cmdRngMin[2] = -cmdRngMax[2]; // Yaw rate command range [rad/s]
  cmdRngMax[3] = 1; cmdRngMin[3] = 0; // Throttle command range [nd]

  // Correct for the reference and command range scales
  VecCmd cmdScale(numCmd);
  cmdScale[0] = 0.5 * (cmdRngMax[0] - cmdRngMin[0]) / refScale[0];
  cmdScale[1] = 0.5 * (cmdRngMax[1] - cmdRngMin[1]) / refScale[1];
  cmdScale[2] = 0.5 * (cmdRngMax[2] - cmdRngMin[2]) / refScale[2];
  cmdScale[3] = 1.0 * (cmdRngMax[3] - cmdRngMin[3]) / refScale[3];

  // Controller Parameters, corrected to the normalized I/O ranges
  VecCmd Kp(numCmd), Ki(numCmd), Kd(numCmd);
  Kp[0] = 0.64 * cmdScale[0], Ki[0] = 0.20 * cmdScale[0], Kd[0] = 0.070;
  Kp[1] = 0.90 * cmdScale[1], Ki[1] = 0.30 * cmdScale[1], Kd[1] = 0.080;
  Kp[2] = 0.00 * cmdScale[2], Ki[2] = 0.00 * cmdScale[2], Kd[2] = 0.000;
  Kp[3] = 4.50 * cmdScale[3], Ki[3] = 1.20 * cmdScale[3], Kd[3] = 0.000;

  // Initialize Individual Controllers
  resRoll_.Init(refScale[0], cmdRngMin[0], cmdRngMax[0], Kp[0], Ki[0], Kd[0]);
  resPitch_.Init(refScale[1], cmdRngMin[1], cmdRngMax[1], Kp[1], Ki[1], Kd[1]);
  resYaw_.Init(refScale[2], cmdRngMin[2], cmdRngMax[2], Kp[2], Ki[2], Kd[2]);
  resSpeed_.Init(1.0, cmdRngMin[3], cmdRngMax[3], Kp[3], Ki[3]);

  // Set the initial controller mode
  resRoll_.mode_ = kCntrlReset;
  resPitch_.mode_ = kCntrlReset;
  resYaw_.mode_ = kCntrlReset;
  resSpeed_.mode_ = kCntrlReset;
}


CntrlMgrLog CntrlMgr::Log(const CntrlMgrOut& cntrlMgrOut)
{
  CntrlMgrLog cntrlMgrLog;

  cntrlMgrLog.mode = cntrlMgrOut.mode;

  for (int i = 0; i < kMaxCntrlCmd; i++) {
    cntrlMgrLog.cmdCntrlBase[i] = cntrlMgrOut.cmdCntrlBase[i];
    cntrlMgrLog.cmdCntrlRes[i] = cntrlMgrOut.cmdCntrlRes[i];
    cntrlMgrLog.cmdCntrl[i] = cntrlMgrOut.cmdCntrl[i];
  }

  for (int i = 0; i < kMaxAllocEff; i++) {
    cntrlMgrLog.cmdAlloc[i] = cntrlMgrOut.cmdAlloc[i];
  }

  for (int i = 0; i < kMaxCntrlEff; i++) {
    cntrlMgrLog.cmdEff[i] = cntrlMgrOut.cmdEff[i];
  }

  for (int i = 0; i < kMaxAllocObj; i++) {
    cntrlMgrLog.vObj[i] = cntrlMgrOut.vObj[i];
  }

  return cntrlMgrLog;
}