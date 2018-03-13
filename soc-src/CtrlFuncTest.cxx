/*
Simple control system tester

See: LICENSE.md for Copyright and License Agreement

g++-5 -std=c++11 -Wall -O3 -g -I../soc-includes CtrlFunc.cxx CtrlFuncTest.cxx -o CtrlFuncTest
./CtrlFuncTest

*/

#include <iostream>
#include "CtrlFunc.hxx"


int main(void)  /* Program tester */
{
  float tStart_s = 0.0;
  float tInit_s = 1.0;
  float tEngage_s = 2.0;
  float tHold_s = 4.0;
  float tReset_s = 5.0;
  float tEnd_s = 6.0;
  float dt_s = 1.0/10.0;
  float tCurr_s;


  float Kp = 0.1;
  float Ki = 1.0;
  float Kd = 0.0;
  float b = 1.0;
  float c = 1.0;
  float refScale = 1.0;
  float cmdRng[2] = {-1, 1};


  CtrlPiDamp testPiDamp;
  testPiDamp.Config(Kp, Ki, Kd, b, refScale, cmdRng[0], cmdRng[1]);

  CtrlPiDamp testPiDamp2;
  testPiDamp2.Config(Kp, Ki, Kd, b, refScale, cmdRng[0], 0.0);

  CtrlPid2 testPid;
  testPid.Config(Kp, Ki, Kd, b, c, refScale, cmdRng[0], cmdRng[1]);

  float ref = 1.0;
  float meas = 0.0;
  float measStep = 0.1 * dt_s;
  float dMeas = 0.0;

  int numIter = (int) (tEnd_s / dt_s); // Number of Iterations

  std::cout << "time" << "\t" << "mode" << "\t" << "ref"  << "\t" << "meas"  << "\t" << "err"  << "\t" << "cmd"   << "\t" << "intErr" << std::endl;
  for(int iIter = 0 ; iIter < numIter ; iIter++){
    tCurr_s = (float) iIter * dt_s;

    if (tCurr_s <= tInit_s) {
      testPiDamp.mode_ = kCtrlStandby;

    } else if (tCurr_s <= tEngage_s) {
      testPiDamp.mode_ = kCtrlInit;

    } else if (tCurr_s <= tHold_s) {
      testPiDamp.mode_ = kCtrlEngage;

    } else if (tCurr_s <= tReset_s) {
      testPiDamp.mode_ = kCtrlHold;

    } else if (tCurr_s <= tEnd_s) {
      testPiDamp.mode_ = kCtrlReset;
    }

    meas += measStep;

    float cmd = 0.0;

    testPiDamp.Run(ref, meas, dMeas, dt_s, &cmd);
    testPiDamp2.Run(ref, meas, dMeas, dt_s, &cmd);
    testPid.Run(ref, meas, dt_s, &cmd);


    int runMode = testPid.mode_;
    float intErr = testPid.iErrState_;

    std::cout << tCurr_s << "\t" << runMode << "\t" << ref <<"\t" << meas << "\t" << (ref-meas) << "\t" << cmd << "\t" << intErr << std::endl;
  }
}
