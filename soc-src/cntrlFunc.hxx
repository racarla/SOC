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

static const float kD2R = M_PI / 180.0;

enum CntrlMode {kCntrlReset = -1, kCntrlStandby = 0, kCntrlHold = 1, kCntrlInit = 2, kCntrlEngage = 3};

// Define CntrlManual Class
class CntrlManual {
public:
  CntrlMode mode_;

  CntrlManual();
  ~CntrlManual() {};
  void Init(const float& refScale, const float& cmdMin, const float& cmdMax);
  float Compute(const float& ref);

private:
  float refScale_, cmdMin_, cmdMax_;

  float CalcCmd(const float& ref);
};


// Define CntrlDamp Class
class CntrlDamp {
public:
  CntrlMode mode_;

  CntrlDamp();
  ~CntrlDamp() {};
  void Init(const float& refScale, const float& cmdMin, const float& cmdMax, const float& KD);
  float Compute(const float& ref, const float& dMeas);

private:
  float refScale_, cmdMin_, cmdMax_;

  float KD_;

  float CalcCmd(const float& err, const float& dErr);
};


// Define CntrlPi Class
class CntrlPi {
public:
  CntrlMode mode_;
  float iErr_;

  CntrlPi();
  ~CntrlPi() {};
  void Init(const float& refScale, const float& cmdMin, const float& cmdMax, const float& KP, const float& KI);
  float Compute(const float& ref, const float& meas, float& dt_s);

private:
  float refMin_, refMax_, cmdMin_, cmdMax_;
  float refScale_;

  float KP_, KI_;

  void InitState(const float& cmd, const float& err);
  float CalcCmd(const float& err);
};


// Define CntrlPiDamp Class
class CntrlPiDamp {
public:
  CntrlMode mode_;
  float iErr_;

  CntrlPiDamp();
  ~CntrlPiDamp() {};
  void Init(const float& refScale, const float& cmdMin, const float& cmdMax, const float& KP, const float& KI, const float& KD);
  float Compute(const float& ref, const float& meas, const float& dMeas, float& dt_s);

private:
  float refScale_, cmdMin_, cmdMax_;

  float KP_, KI_, KD_;

  void InitState(const float& cmd, const float& err, const float& dErr);
  float CalcCmd(const float& err, const float& dErr);
};


// Define CntrlPid Class
class CntrlPid {
public:
  CntrlMode mode_;
  float iErr_, errPrev_;

  CntrlPid();
  ~CntrlPid() {};
  void Init(const float& refScale, const float& cmdMin, const float& cmdMax, const float& KP, const float& KI, const float& KD);
  float Compute(const float& ref, const float& meas, float& dt_s);

private:
  float refScale_, cmdMin_, cmdMax_;

  float KP_, KI_, KD_;

  void InitState(const float& cmd, const float& err, const float& dErr);
  float CalcCmd(const float& err, const float& dErr);
};

#endif // CNTRLFUNC_HXX_
