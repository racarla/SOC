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

struct CntrlAllocDef {
  MatCntrlEff cntrlEff;
  MatObj wtObj;
  MatEff wtEff;
  VecEff uMin;
  VecEff uMax;
  VecEff uPref;
};

struct CntrlAllocStruct {
  VecEff cmdAlloc;
};

class CntrlAllocMgr {
 public:

  CntrlAllocMgr() {};   // Constructor
  ~CntrlAllocMgr() {};  // Destructor

  void Init(const CntrlAllocDef& cntrlAllocDef); // Initialize excitations
  CntrlAllocStruct Compute(const VecObj& vObj);

 private:
  uint8_t numObj_;
  uint8_t numEff_;

  CntrlAllocDef cntrlAllocDef_;
  CntrlAllocStruct cntrlAllocData_;
};


#endif // CNTRLALLOCMGR_HXX_
