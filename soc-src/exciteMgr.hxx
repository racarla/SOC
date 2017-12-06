/*
Excite System Manager - Defines Excitations, Computes commands

See: LICENSE.md for Copyright and License Agreement

History:
2017-11-19 - Chris Regan - Created
*/

#ifndef EXCITEMGR_HXX_
#define EXCITEMGR_HXX_

#include <math.h>
#include <Eigen/Core>
#include "exciteGenFunc.hxx"

const float kHz2Rps = 2 * M_PI;

// Exitation output structure
struct ExciteMgrStruct {
  bool exciteMode;
  uint8_t indxTest;
  float timeExcite_s;
  VecChan cmdExcite;
};

class ExciteMgr {
 public:

  ExciteMgr() {};   // Constructor
  ~ExciteMgr() {};  // Destructor

  void Init();     // Initialize excitations
  ExciteMgrStruct Compute(const bool& exciteMode, const uint8_t& indxTest, const float& time_s);

 private:
  ExciteMgrStruct exciteMgrData_;
  float timeEngage_s_;

  ExciteMultisine exciteTest01_, exciteTest02_, exciteTest03_, exciteTest04_, exciteTest05_;
  ExciteChirp exciteTest06_, exciteTest07_, exciteTest08_, exciteTest09_;
  ExciteDisc exciteTest10_, exciteTest11_, exciteTest12_;
  ExciteDisc exciteTest13_, exciteTest14_, exciteTest15_, exciteTest16_;
  ExciteDisc exciteTest17_, exciteTest18_, exciteTest19_;

};


#endif // EXCITEMGR_HXX_
