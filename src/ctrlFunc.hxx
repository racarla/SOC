/*
Classes and functions for Control Functions

See: LICENSE.md for Copyright and License Agreement
*/

#ifndef CTRLFUNC_HXX_
#define CTRLFUNC_HXX_

#include <math.h>

enum CtrlMode {kCtrlReset = -1, kCtrlStandby = 0, kCtrlHold = 1, kCtrlInit = 2, kCtrlEngage = 3};

// Define CtrlFuncNone Class, Just return zero
class CtrlFuncNone {
 public:
  CtrlMode mode_;

  CtrlFuncNone() {};
  ~CtrlFuncNone() {};
  void Config() {mode_ = kCtrlStandby;};
  void Run(float *cmd) {*cmd = 0.0;};
};

// Define CtrlFuncPiDamp Class
class CtrlFuncPiDamp {
 public:
  CtrlMode mode_;
  float iErrState_;

  CtrlFuncPiDamp() {};
  ~CtrlFuncPiDamp() {};
  void Config(const float &Kp, const float &Ki, const float &Kd, const float &b, const float &refScale, const float &cmdMin, const float &cmdMax);
  void Run(const float &ref, const float &meas, const float &dMeas, const float &dt_s, float *cmd);

 private:
  float Kp_, Ki_, Kd_;
  float b_;
  float refScale_;
  float cmdMin_, cmdMax_;

  void InitState(const float &cmd, const float &pErr, const float &dErrState, float *iErrState);
  void UpdState(const float &iErr, const float &dt_s, float *iErrState);
  void CalcCmd(const float &pErr, const float &dErrState, float *cmd);
};

// Define CtrlFuncPid2 Class
class CtrlFuncPid2 {
 public:
  CtrlMode mode_;
  float iErrState_;
  float dErrPrev_;

  CtrlFuncPid2() {};
  ~CtrlFuncPid2() {};
  void Config(const float &Kp, const float &Ki, const float &Kd, const float &b, const float &c, const float &refScale, const float &cmdMin, const float &cmdMax);
  void Run(const float &ref, const float &meas, const float &dt_s, float *cmd);

 private:
  float Kp_, Ki_, Kd_;
  float b_, c_;
  float refScale_;
  float cmdMin_, cmdMax_;

  void InitState(const float &cmd, const float &pErr, const float &dErrState, float *iErrState);
  void UpdState(const float &iErr, const float &dt_s, float *iErrState);
  void CalcCmd(const float &pErr, const float &dErrState, float *cmd);
};

// Define CtrlFuncSS Class
class CtrlFuncSS {
 public:
  CtrlMode mode_;

  CtrlFuncSS() {};
  ~CtrlFuncSS() {};
  void Config() {mode_ = kCtrlStandby;};
  void Run() {};
};


#endif // CTRLFUNC_HXX_
