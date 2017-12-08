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
#include <stdint.h>

#ifndef kMaxAllocObj
#define kMaxAllocObj 3
#endif

#ifndef kMaxAllocEff
#define kMaxAllocEff 6
#endif

#include "cntrlAllocFunc.hxx"

struct CntrlAllocDef {
  MatCntrlEff cntrlEff;
  MatObj wtObj;
  MatEff wtEff;
  VecEff uMin;
  VecEff uMax;
  VecEff uPref;
};

struct CntrlAllocOut {
  VecEff cmdAlloc;
};

struct CntrlAllocLog {
  float cmdAlloc[kMaxAllocEff] = {0};
};


class CntrlAllocMgr {
 public:

  CntrlAllocMgr() {};   // Constructor
  ~CntrlAllocMgr() {};  // Destructor

  void Init(const CntrlAllocDef& cntrlAllocDef); // Initialize excitations
  CntrlAllocOut Compute(const VecObj& vObj);
  CntrlAllocLog Log();

 private:
  uint8_t numObj_;
  uint8_t numEff_;

  CntrlAllocDef cntrlAllocDef_;
  CntrlAllocOut cntrlAllocData_;
};

#endif // CNTRLALLOCMGR_HXX_
