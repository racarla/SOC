/*
Control System Manager - Defines Controllers, Mangages mode switching, Computes commands

See: LICENSE.md for Copyright and License Agreement

History:
2017-11-19 - Chris Regan - Created
*/

#ifndef CNTRLMGR_HXX_
#define CNTRLMGR_HXX_

#include "cntrlFunc.hxx"
#include <Eigen/Core>

#define MaxCntrlCmdDim 5

typedef Eigen::Matrix<float, -1, 1, 0, MaxCntrlCmdDim, 1> VecCmd;

struct CntrlMgrStruct {
  VecCmd cmdBase, cmdRes, cmd;
  CntrlMode mode;
};

class CntrlMgr {
 public:

  CntrlMgr() {};   // Constructor
  ~CntrlMgr() {};  // Destructor

  void Init();     // Initialize controllers and excitations
  void Mode(CntrlMode mode);     // Control the Mode of all the controllers

  VecCmd CmdBase(const VecCmd& refVec, float time_s);
  VecCmd CmdRes(const VecCmd& refVec, const VecCmd& measVec, const VecCmd& dMeasVec, float time_s);
  CntrlMgrStruct Cmd();      // Compute Controller Commands
 private:
  CntrlMgrStruct cntrlMgrData_;

  float timePrevBase_s_, timePrevRes_s_;

  CntrlManual baseRoll_, basePitch_, baseYaw_, baseSpeed_;
  //CntrlPiDamp resRoll_, resPitch_, resYaw_, resSpeed_;
  CntrlDamp resRoll_, resPitch_, resYaw_;
  CntrlPi resSpeed_;

  void CntrlBaseDef();
  void CntrlResDef();
};

#endif // CNTRLMGR_HXX_
