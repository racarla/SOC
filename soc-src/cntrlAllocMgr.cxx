/*
Excite System Manager - Defines Excitations, Computes commands

See: LICENSE.md for Copyright and License Agreement

History:
2017-11-19 - Chris Regan - Created
*/

#include "cntrlAllocMgr.hxx"
#include <iostream>


void CntrlAllocMgr::Init(MatCntrlEff cntrlEff, MatObj wtObj, MatEff wtEff, VecEff uMin, VecEff uMax, VecEff uPref)
{
  cntrlEff_ = cntrlEff;
  wtObj_ = wtObj;
  wtEff_ = wtEff;
  uMin_ = uMin;
  uMax_ = uMax;
  uPref_ = uPref;

  numObj_ = cntrlEff.rows();
  numEff_ = cntrlEff.cols();

  VecEff uCmd_(numEff_);
}

VecEff CntrlAllocMgr::Compute(VecObj vObj)
{
  uCmd_.setZero(numEff_);

  uCmd_ = CntrlAllocPseudo(cntrlEff_, vObj, uPref_);

  return uCmd_;

}
