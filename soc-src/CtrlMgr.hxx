/*
Control System Manager - Defines Controllers, Mangages mode switching, Computes commands

See: LICENSE.md for Copyright and License Agreement

History:
2017-11-19 - Chris Regan - Created
*/

#ifndef CNTRLMGR_HXX_
#define CNTRLMGR_HXX_

#include <Eigen/Core>
#include <math.h>
#include <stdint.h>

#ifndef kMaxCntrlCmd
#define kMaxCntrlCmd 4
#endif

#ifndef kMaxCntrlEff
#define kMaxCntrlEff 7
#endif

#ifndef kMaxAllocObj
#define kMaxAllocObj 3
#endif

#ifndef kMaxAllocEff
#define kMaxAllocEff 6
#endif

#include "CntrlFunc.hxx"
#include "AllocFunc.hxx"
#include "navigation.hxx"
#include "structs.hxx"
#include "airdata.hxx"
#include "fmu.hxx"

typedef Eigen::Matrix<float, -1, 1, 0, kMaxCntrlCmd, 1> VecCmd;
typedef Eigen::Matrix<float, -1, 1, 0, kMaxCntrlEff, 1> VecEff;

typedef Eigen::Matrix<float, -1, 1, 0, kMaxAllocObj, 1> VecAllocObj;
typedef Eigen::Matrix<float, -1, 1, 0, kMaxAllocEff, 1> VecAllocEff;

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
  float cmdCntrlBase[kMaxCntrlCmd] = {0};
  float cmdCntrlRes[kMaxCntrlCmd] = {0};
  float cmdCntrl[kMaxCntrlCmd] = {0};
  float cmdEff[kMaxCntrlEff] = {0};

  float vObj[kMaxAllocObj] = {0};
  float cmdAlloc[kMaxAllocEff] = {0};
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


#endif // CNTRLMGR_HXX_
