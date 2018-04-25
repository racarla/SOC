/*
Simple control system tester

See: LICENSE.md for Copyright and License Agreement

g++-5 -std=c++11 -Wall -O3 -g -I../includes ctrlFunc.cc ctrlFuncTest.cc -o ctrlFuncTest
./ctrlFuncTest

*/

#include <iostream>
#include "ctrlFunc.hxx"


int main(void)  /* Program tester */
{

  // float Kp = 0.1;
  // float Ki = 1.0;
  // float Kd = 0.0;
  // float Tf = 0.0;
  // float b = 1.0;
  // float c = 1.0;
  float cmdRng[2] = {-1, 1};

  // CtrlFuncPid2 ctrl;
  // ctrl.Config(Kp, Ki, Kd, Tf, b, c, cmdRng[0], cmdRng[1]);

  float a0 = 1;
  float a1 = -0.9608;
  float b0 = 0.065;
  float b1 = -b0;

  CtrlFuncDamp ctrl;
  ctrl.Config(b0, b1, a0, a1, cmdRng[0], cmdRng[1]);


  float tStart_s = 0.0;
  float tInit_s = 1.0;
  float tEngage_s = 2.0;
  float tHold_s = 4.0;
  float tReset_s = 5.0;
  float tEnd_s = 6.0;
  float dt_s = 1.0/50.0;
  float tCurr_s;

  float ref = 1.0;
  float meas = 0.0;
  float measStep = 0.1 * dt_s;

  int numIter = (int) (tEnd_s / dt_s); // Number of Iterations

  std::cout << "time" << "\t" << "mode" << "\t" << "ref"  << "\t" << "meas"  << "\t" << "err"  << "\t" << "cmd"  << std::endl;
  for(int iIter = 0 ; iIter < numIter ; iIter++){
    tCurr_s = (float) iIter * dt_s;

    if (tCurr_s <= tInit_s) {
      ctrl.mode_ = kCtrlStandby;

    } else if (tCurr_s <= tEngage_s) {
      ctrl.mode_ = kCtrlInit;

    } else if (tCurr_s <= tHold_s) {
      ctrl.mode_ = kCtrlEngage;

    } else if (tCurr_s <= tReset_s) {
      ctrl.mode_ = kCtrlHold;

    } else if (tCurr_s <= tEnd_s) {
      ctrl.mode_ = kCtrlReset;
    }

    if (tCurr_s > 3.0) {
      meas = 2.0;
    } else if (tCurr_s > 1.5) {
      meas = 1.0;
    } else {
      meas = 0.0;
    }

    // meas += measStep;

    float cmd = 0.0;

    // ctrl.Run(ref, meas, dt_s, &cmd);
    ctrl.Run(meas, dt_s, &cmd);

    int runMode = ctrl.mode_;

    std::cout << tCurr_s << "\t" << runMode << "\t" << meas << "\t" << cmd << std::endl;
    // std::cout << tCurr_s << "\t" << runMode << "\t" << ref << "\t" << meas << "\t" << (ref-meas) << "\t" << cmd << std::endl;
  }
}
