/*
Excite System Manager - Defines Excitations, Computes commands

See: LICENSE.md for Copyright and License Agreement

History:
2017-11-19 - Chris Regan - Created
*/

#include "cntrlAllocMgr.hxx"
#include <iostream>


void CntrlAllocMgr::Init(const CntrlAllocDef& cntrlAllocDef)
{
  cntrlAllocDef_ = cntrlAllocDef;

  numObj_ = cntrlAllocDef_.cntrlEff.rows();
  numEff_ = cntrlAllocDef_.cntrlEff.cols();

  cntrlAllocData_.cmdAlloc = cntrlAllocDef_.uPref;
}

CntrlAllocStruct CntrlAllocMgr::Compute(const VecObj& vObj)
{
  cntrlAllocData_.cmdAlloc = cntrlAllocDef_.uPref;

  cntrlAllocData_.cmdAlloc = CntrlAllocPseudo(cntrlAllocDef_.cntrlEff, vObj, cntrlAllocDef_.uPref);

  return cntrlAllocData_;

}
