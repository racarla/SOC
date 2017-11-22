/*
Allocation System Manager - Defines Control Allocation, Computes commands

See: LICENSE.md for Copyright and License Agreement

History:
2017-11-20 - Chris Regan - Created
*/

#ifndef CNTRLALLOCMGR_HXX_
#define CNTRLALLOCMGR_HXX_

#include <math.h>
#include <Eigen/Dense>
#include "cntrlAllocFunc.hxx"

#include <stdint.h>

class CntrlAllocMgr {
 public:
  VecEff uCmd_;

  CntrlAllocMgr() {};   // Constructor
  ~CntrlAllocMgr() {};  // Destructor

  void Init(MatCntrlEff cntrlEff, MatObj wtObj, MatEff wtEff, VecEff uMin, VecEff uMax, VecEff uPref); // Initialize excitations
  VecEff Compute(VecObj vObj);

 private:
  uint8_t numObj_;
  uint8_t numEff_;

  MatCntrlEff cntrlEff_;
  MatObj wtObj_;
  MatEff wtEff_;
  VecEff uMin_, uMax_, uPref_;
};

#endif // CNTRLALLOCMGR_HXX_
