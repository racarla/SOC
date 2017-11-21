/*
Classes and functions for Control Functions

See: LICENSE.md for Copyright and License Agreement

History:
2017-11-12 - Chris Regan - Defined CntrlPiDamp class and methods
2017-11-12 - Chris Regan - Defined CntrlPid class and methods
2017-11-12 - Chris Regan - Defined CntrlDamp class and methods
*/

#ifndef CNTRLFUNC_HXX_
#define CNTRLFUNC_HXX_

#include <math.h>

const float kD2R = M_PI / 180.0;

enum CntrlMode {kCntrlReset = -1, kCntrlStandby = 0, kCntrlHold = 1, kCntrlInit = 2, kCntrlEngage = 3};

// Define CntrlManual Class
class CntrlManual {
public:
  CntrlMode runMode_;

  CntrlManual();
  ~CntrlManual() {};
  void Init(float refMin, float refMax, float cmdMin, float cmdMax);
  float Compute(float ref);

private:
  float refMin_, refMax_, cmdMin_, cmdMax_;

  float CalcCmd(float ref);
};


// Define CntrlDamp Class
class CntrlDamp {
public:
  CntrlMode runMode_;

  CntrlDamp();
  ~CntrlDamp() {};
  void Init(float KD, float refMin, float refMax, float cmdMin, float cmdMax);
  float Compute(float ref, float dMeas);

private:
  float refMin_, refMax_, cmdMin_, cmdMax_;
  float KD_;

  float CalcCmd(float err, float dErr);
};


// Define CntrlPi Class
class CntrlPi {
public:
  CntrlMode runMode_;
  float iErr_;

  CntrlPi();
  ~CntrlPi() {};
  void Init(float KP, float KI, float refMin, float refMax, float cmdMin, float cmdMax);
  float Compute(float ref, float meas, float dt_s);

private:
  float refMin_, refMax_, cmdMin_, cmdMax_;
  float KP_, KI_;

  void InitState(float cmd, float err);
  float CalcCmd(float err);
};


// Define CntrlPiDamp Class
class CntrlPiDamp {
public:
  CntrlMode runMode_;
  float iErr_;

  CntrlPiDamp();
  ~CntrlPiDamp() {};
  void Init(float KP, float KI, float KD, float refMin, float refMax, float cmdMin, float cmdMax);
  float Compute(float ref, float meas, float dMeas, float dt_s);

private:
  float refMin_, refMax_, cmdMin_, cmdMax_;
  float KP_, KI_, KD_;

  void InitState(float cmd, float err, float dErr);
  float CalcCmd(float err, float dErr);
};


// Define CntrlPid Class
class CntrlPid {
public:
  CntrlMode runMode_;
  float iErr_, errPrev_;

  CntrlPid();
  ~CntrlPid() {};
  void Init(float KP, float KI, float KD, float refMin, float refMax, float cmdMin, float cmdMax);
  float Compute(float ref, float meas, float dt_s);

private:
  float refMin_, refMax_, cmdMin_, cmdMax_;
  float KP_, KI_, KD_;

  void InitState(float cmd, float err, float dErr);
  float CalcCmd(float err, float dErr);
};

#endif // CNTRLFUNC_HXX_
