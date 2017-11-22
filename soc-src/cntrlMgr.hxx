/*
Control System Manager - Defines Controllers, Mangages mode switching, Computes commands

See: LICENSE.md for Copyright and License Agreement

History:
2017-11-19 - Chris Regan - Created
*/

#ifndef CNTRLMGR_HXX_
#define CNTRLMGR_HXX_

#include "cntrlFunc.hxx"
#include <Eigen/Dense>

#define MaxCntrlCmdDim 5

typedef Eigen::Matrix<float, -1, 1, 0, MaxCntrlCmdDim, 1> VecCmd;

class CntrlMgr {
 public:
  CntrlMode cntrlMode_; // Controller mode
  VecCmd cntrlBaseCmd_, cntrlResCmd_, cntrlCmd_;

  CntrlMgr() {};   // Constructor
  ~CntrlMgr() {};  // Destructor

  void Init();     // Initialize controllers and excitations
  void Mode(CntrlMode cntrlMode);     // Control the Mode of all the controllers

  VecCmd CmdBase(VecCmd refVec, float time_s);
  VecCmd CmdRes(VecCmd refVec, VecCmd measVec, VecCmd dMeasVec, float time_s);
  VecCmd Cmd();      // Compute Controller Commands
 private:
  float timePrev_s_, time_s_, dt_s_;

  CntrlManual cntrlBaseRoll_, cntrlBasePitch_, cntrlBaseYaw_, cntrlBaseSpeed_;
  CntrlPiDamp cntrlResRoll_, cntrlResPitch_, cntrlResYaw_, cntrlResSpeed_;

  void CntrlBaseDef();
  void CntrlResDef();
};



#endif // CNTRLMGR_HXX_

