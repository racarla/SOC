/*
Control System Manager - Defines Controllers, Mangages mode switching, Computes commands

See: LICENSE.md for Copyright and License Agreement

History:
2017-11-19 - Chris Regan - Created
*/

#pragma once

#include "Eigen/Core"

#include <math.h>
#include <stdint.h>

#include "ctrlSys.hxx"
#include "allocFunc.hxx"
#include "fmu.hxx"

const uint8_t kMaxGuidCmd = 6;
const uint8_t kMaxScasCmd = kMaxGuidCmd;
const uint8_t kMaxCtrlEff = 20;

typedef Eigen::Matrix<float, -1, 1, 0, kMaxGuidCmd, 1> VecGuid;
typedef Eigen::Matrix<float, -1, 1, 0, kMaxScasCmd, 1> VecScas;
typedef Eigen::Matrix<float, -1, 1, 0, kMaxCtrlEff, 1> VecEff;

struct CntrlMgrOut {
  CntrlMode mode;
  VecCmd cmdCntrlBase;
  VecCmd cmdCntrlRes;
  VecCmd cmdCntrl;
  VecAllocObj vObj;
  VecAllocEff cmdAlloc;
  VecEff cmdEff;
};

struct CntrlMgrLog {
  CntrlMode mode;
  float cmdCntrlBase[kMaxScasCmd] = {0};
  float cmdCntrlRes[kMaxScasCmd] = {0};
  float cmdCntrl[kMaxScasCmd] = {0};
  float cmdEff[kMaxCtrlEff] = {0};

  float vObj[kMaxScasCmd] = {0};
  float cmdAlloc[kMaxCtrlEff] = {0};
};

struct CntrlAllocDef {
  MatCntrlEff cntrlEff;
  MatObj wtObj;
  MatEff wtEff;
  VecAllocEff uMin;
  VecAllocEff uMax;
  VecAllocEff uPref;
};


class CntrlMgr {
 public:

  CntrlMgr() {};   // Constructor
  ~CntrlMgr() {};  // Destructor

  void Init(const CntrlAllocDef& cntrlAllocDef);     // Initialize controllers and excitations
  void Mode(CntrlMode mode);     // Control the Mode of all the controllers

  VecCmd CmdCntrlBase(const float& time_s, const FmuData& fmuData, const NavOut& navOut, const AirdataOut& airdataOut);
  VecCmd CmdCntrlRes(const float& time_s, const FmuData& fmuData, const NavOut& navOut, const AirdataOut& airdataOut);
  CntrlMgrOut CmdCntrl();      // Compute Controller Commands

  VecAllocEff AllocCompute(const VecAllocObj& vObj);

  CntrlMgrLog Log(const CntrlMgrOut& cntrlMgrOut);

 private:
  uint8_t numObj_;
  uint8_t numEff_;
  CntrlMgrOut cntrlMgrOut_;

  float timePrevBase_s_, timePrevRes_s_;

  CntrlManual baseRoll_, basePitch_, baseYaw_;
  CntrlManual baseSpeed_;

  CntrlPiDamp resRoll_, resPitch_, resYaw_;
  CntrlPi resSpeed_;

  void CntrlBaseDef();
  void CntrlResDef();

  CntrlAllocDef cntrlAllocDef_;
};
