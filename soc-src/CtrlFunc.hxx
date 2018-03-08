/*
Classes and functions for Control Functions

See: LICENSE.md for Copyright and License Agreement

History:
2017-11-12 - Chris Regan - Defined CtrlPiDamp class and methods
2017-11-12 - Chris Regan - Defined CtrlPid class and methods
2017-11-12 - Chris Regan - Defined CtrlDamp class and methods
*/

#ifndef CNTRLFUNC_HXX_
#define CNTRLFUNC_HXX_

#include <math.h>

static const float kD2R = M_PI / 180.0;

enum CtrlMode {kCtrlReset = -1, kCtrlStandby = 0, kCtrlHold = 1, kCtrlInit = 2, kCtrlEngage = 3};

class Ctrl { // Base Class for all Controllers (Guidance and SCAS types)
 public:
  CtrlMode mode_;

  Ctrl() {};
  virtual ~Ctrl() {};
  virtual void Config() {};
  virtual void Run() {};

 private:

};

// Define CtrlPiDamp Class
class CtrlPiDamp {
public:
  CtrlMode mode_;
  float iErrState_;

  CtrlPiDamp();
  ~CtrlPiDamp() {};
  void Config(const float& Kp, const float& Ki, const float& Kd, const float& b, const float& refScale, const float& cmdRng[2]);
  void Run(const float& ref, const float& meas, const float& dMeas, float& dt_s);

private:
  float Kp_, Ki_, Kd_;
  float b_;
  float refScale_;
  float cmdRng_;

  void InitState(const float& cmd, const float& pErr, const float& dErrState);
  void UpdState(const float& iErr, const float& dt_s);
  void CalcCmd(const float& pErr, const float& dErrState);
};

// Define CtrlPid2 Class
class CtrlPid2 {
public:
  CtrlMode mode_;
  float iErrState_;
  float dErrPrev_;

  CtrlPid2();
  ~CtrlPid2() {};
  void Config(const float& Kp, const float& Ki, const float& Kd, const float& b, const float& c, const float& refScale, const float& cmdRng[2]);
  void Run(const float& ref, const float& meas, float& dt_s);

private:
  float Kp_, Ki_, Kd_;
  float b_, c_;
  float refScale_;
  float cmdRng_;

  void InitState(const float& cmd, const float& pErr, const float& dErrState);
  void UpdState(const float& iErr, const float& dt_s);
  void CalcCmd(const float& pErr, const float& dErrState);
};

#endif // CNTRLFUNC_HXX_
