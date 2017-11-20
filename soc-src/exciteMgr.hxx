/*
Excite System Manager - Defines Excitations, Computes commands

See: LICENSE.md for Copyright and License Agreement

History:
2017-11-19 - Chris Regan - Created
*/

#ifndef EXCITEMGR_HXX_
#define EXCITEMGR_HXX_

#include <math.h>
#include <Eigen/Dense>
#include "exciteGenFunc.hxx"

#define MaxChan 16
#define MaxElem 30

typedef Eigen::Matrix<float, -1, 1, 0, MaxCntrlCmdDim, 1> VecCmd;

const double kD2R = M_PI / 180.0;

class ExciteMgr {
 public:
  VecCmd cmdExcite_;
  float cmdPitch_, cmdRoll_, cmdYaw_, cmdThrottle_;

  ExciteMgr() {};   // Constructor
  ~ExciteMgr() {};  // Destructor

  void Init();     // Initialize excitations
  bool Run(bool exciteMode, int indxTest, float timeCurr_s);
  VecCmd GetSignal();

 private:
  ExciteMultisine exciteTest01_, exciteTest02_, exciteTest03_, exciteTest04_, exciteTest05_;
  ExciteChirp exciteTest06_, exciteTest07_, exciteTest08_, exciteTest09_;
  ExciteDisc exciteTest10_, exciteTest11_, exciteTest12_;
  ExciteDisc exciteTest13_, exciteTest14_, exciteTest15_, exciteTest16_;
  ExciteDisc exciteTest17_, exciteTest18_, exciteTest19_;

};


#endif // EXCITEMGR_HXX_
