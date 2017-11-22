/*
Simple control system tester

See: LICENSE.md for Copyright and License Agreement

History:
2017-11-12 - Chris Regan - Defined CntrlPiDamp class and methods
2017-11-12 - Chris Regan - Defined CntrlPid class and methods
2017-11-12 - Chris Regan - Defined CntrlDamp class and methods
*/

#include <iostream>
#include "cntrlFunc.hxx"


int main(void)  /* Program tester */
{
  float timeStart_s = 0.0;
  float timeInit_s = 1.0;
  float timeEngage_s = 2.0;
  float timeHold_s = 4.0;
  float TimeReset_s = 5.0;
  float TimeEnd_s = 6.0;
  float TimeStep_s = 1.0/10.0;
  float TimeCurr_s;


  float KP = 0.1;
  float KI = 1.0;
  float KD = 0.0;
  float KDamp = -0.0;
  float refRng[2] = {-1, 1};
  float cmdRng[2] = {-1, 1};


  CntrlManual testManual;
  testManual.Init(-1, 1, -1, 1);

  CntrlManual testManual2;
  testManual2.Init(-2, 2, -2, 2);

  CntrlPiDamp testPiDamp1;
  testPiDamp1.Init(KP, KI, KDamp, -1, 1, -1, 1);

  CntrlPiDamp testPiDamp2;
  testPiDamp2.Init(KP, KI, KDamp, -2, 2, -2, 2);

  float ref = 1.0;
  float meas = 0.0;
  float measStep = 0.5 * TimeStep_s;
  float dMeas = 0.0;

  int numIter = (int) (TimeEnd_s / TimeStep_s); // Number of Iterations

  std::cout << "time" << "\t" << "mode" << "\t" << "ref"  << "\t" << "meas"  << "\t" << "err"  << "\t" << "cmd"   << "\t" << "intErr" << std::endl;
  for(int iIter = 0 ; iIter < numIter ; iIter++){
    TimeCurr_s = (float) iIter * TimeStep_s;

    if (TimeCurr_s <= timeInit_s) {
      testManual.runMode_ = kCntrlStandby;

    } else if (TimeCurr_s <= timeEngage_s) {
      testManual.runMode_ = kCntrlInit;

    } else if (TimeCurr_s <= timeHold_s) {
      testManual.runMode_ = kCntrlEngage;
      
    } else if (TimeCurr_s <= TimeReset_s) {
      testManual.runMode_ = kCntrlHold;
      testPiDamp1.iErr_ = 0; // Should have no effect on Pid
      testPiDamp2.iErr_ = 0; // Should have no effect on Pid
      
    } else if (TimeCurr_s <= TimeEnd_s) {
      testManual.runMode_ = kCntrlReset;
    }

    meas += measStep;

    float cmdCntrl = testManual.Compute(ref);

    int runMode = testManual.runMode_;

    float cmdCntrl2 = testManual2.Compute(ref);
//    float intErr = testManual2.iErr_;

    std::cout << TimeCurr_s << "\t" << runMode << "\t" << ref <<"\t" << meas << "\t" << (ref-meas) << "\t" << cmdCntrl   << "\t" << std::endl;
//    std::cout << TimeCurr_s << "\t" << runMode << "\t" << ref <<"\t" << meas << "\t" << (ref-meas) << "\t" << cmdCntrl   << "\t" << intErr << std::endl;
  }
}
