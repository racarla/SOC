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

#ifndef kMaxExciteChan
#define kMaxExciteChan 4
#endif

#ifndef kMaxExciteElem
#define kMaxExciteElem 46
#endif

#include "WaveGenFunc.hxx"

const float kHz2Rps = 2 * M_PI;

// Exitation output structure
struct ExciteMgrOut {
  bool exciteMode;
  uint8_t indxTest;
  float tExcite_s;
  VecChan cmdExcite;
};

struct ExciteMgrLog {
  bool exciteMode;
  uint8_t indxTest;
  float tExcite_s;
  float cmdExcite[kMaxExciteChan] = {0};
};

class ExciteMgr {
 public:

  ExciteMgr() {};   // Constructor
  ~ExciteMgr() {};  // Destructor

  void Init();     // Initialize excitations
  ExciteMgrOut Compute(const bool& exciteMode, const uint8_t& indxTest, const float& t_s);

  ExciteMgrLog Log(const ExciteMgrOut& exciteMgrOut);

 private:
  ExciteMgrOut exciteMgrOut_;
  float tEngage_s_;

  WaveMultisine exciteTest01_, exciteTest02_, exciteTest03_, exciteTest04_, exciteTest05_;
  WaveChirp exciteTest06_, exciteTest07_, exciteTest08_, exciteTest09_;
  WaveDisc exciteTest10_, exciteTest11_, exciteTest12_;
  WaveDisc exciteTest13_, exciteTest14_, exciteTest15_, exciteTest16_;
  WaveDisc exciteTest17_, exciteTest18_, exciteTest19_;
};


#endif // EXCITEMGR_HXX_
