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
  VecObj uCmd_;

  CntrlAllocMgr() {};   // Constructor
  ~CntrlAllocMgr() {};  // Destructor

  void Init();     // Initialize excitations
  VecEff Run(VecObj vObj);

 private:
  MatCntrlEff cntrlEff_;
  MatObj wtObj_;
  MatEff wtEff_;
  VecEff uMin_, uMax_, uPref_;
};

#endif // CNTRLALLOCMGR_HXX_
