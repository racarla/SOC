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


  float kP = 0.1;
  float kI = 1.0;
  float kD = 0.0;
  float kDamp = -0.0;
  float cmdRng[2] = {-0.5, 0.5};


  CntrlPid testPid1;
  testPid1.SetParam(kP, kI, kD, cmdRng[0], cmdRng[1]);

  CntrlPid testPid2;
  testPid2.SetParam(kP, kI, kD, cmdRng[0], cmdRng[1]);

  CntrlPiDamp testPiDamp;
  testPiDamp.SetParam(kP, kI, kDamp, cmdRng[0], cmdRng[1]);

  float ref = 1.0;
  float meas = 0.0;
  float measStep = 0.5 * TimeStep_s;
  float dMeas = 0.0;

  int numIter = (int) (TimeEnd_s / TimeStep_s); // Number of Iterations

  std::cout << "time" << "\t" << "mode" << "\t" << "ref"  << "\t" << "meas"  << "\t" << "err"  << "\t" << "cmd"   << "\t" << "intErr" << std::endl;
  for(int iIter = 0 ; iIter < numIter ; iIter++){
    TimeCurr_s = (float) iIter * TimeStep_s;

    if (TimeCurr_s <= timeInit_s) {
      testPid1.runMode = Standby;

    } else if (TimeCurr_s <= timeEngage_s) {
      testPid1.runMode = Init;

    } else if (TimeCurr_s <= timeHold_s) {
      testPid1.runMode = Engage;
      
    } else if (TimeCurr_s <= TimeReset_s) {
      testPid1.runMode = Hold;
      testPiDamp.iErr = 0; // Should have no effect on Pid
      testPid2.iErr = 0; // Should have no effect on Pid
      
    } else if (TimeCurr_s <= TimeEnd_s) {
      testPid1.runMode = Reset;
    }

    meas += measStep;

    float cmdPid = testPid1.Compute(ref, meas, TimeStep_s);
    int runMode = testPid1.runMode;
    float intErr = testPid1.iErr;

    float cmdPiDamp = testPiDamp.Compute(ref, meas, dMeas, TimeStep_s);

    std::cout << TimeCurr_s << "\t" << runMode << "\t" << ref <<"\t" << meas << "\t" << (ref-meas) << "\t" << cmdPid   << "\t" << intErr << std::endl;
  }
}
